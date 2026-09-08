# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Visual evidence that the standalone Bosch 800 asset runs under THIS repo's Isaac stack.

Loads ``assets/models/bosch800/bosch800.usdc`` (docs/bosch800_asset.md) as an Isaac Lab 2.1
``Articulation`` on Isaac Sim 4.5, drives its four authored joints through a full
open -> extend -> load -> retract -> close cycle with this repo's own props (a plate and a
cup from ``assets/props``, tinted per :func:`dishsim.config.display_color`), and records:

- ``media/bosch800_asset/loaded_extended_<cam>.png`` — the still (racks out, dishes settled)
- ``media/bosch800_asset/articulation.mp4``          — the whole cycle, 30 fps
- ``media/bosch800_asset/evidence.json``             — measured joint errors + dish retention

Pass/fail (``[RESULT]``): every hold reaches its joint target within tolerance, the dishes
settle on the extended racks, and they ride the racks back in (retained). The dish sites are
READ FROM THE ASSET (``<Rack>/Manipulation/*`` prims), never hard-coded.

    scripts/run_kit.sh scripts/evaluation/bosch800_asset_evidence.py --headless --enable_cameras
    scripts/run_kit.sh scripts/evaluation/bosch800_asset_evidence.py --headless --probe   # stage dump only
"""
import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Standalone Bosch 800 asset: articulation + load evidence.")
parser.add_argument("--usd", type=str, default=os.path.join(PROJECT_ROOT, "assets", "models", "bosch800", "bosch800.usdc"))
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "media", "bosch800_asset"))
parser.add_argument("--probe", action="store_true", help="Open the stage, print joints/drives/sites, exit.")
parser.add_argument("--no_video", action="store_true", help="Stills + measurements only.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from dishsim.checks import check, finish  # noqa: E402
from dishsim.media import release_sim_for_close  # noqa: E402

PRIM_PATH = "/World/Bosch800"
JOINTS = ["door_hinge", "lower_slide", "middle_slide", "third_slide"]
#: Extension targets of the validation run of record (validation.json): not the full travel,
#: because all three racks out at once is a visual overview, not an access state.
EXTENDED = {"door_hinge": math.radians(90.0), "lower_slide": -0.49, "middle_slide": -0.30, "third_slide": -0.20}
CLOSED = {j: 0.0 for j in JOINTS}
TOL = {"door_hinge": math.radians(0.5), "lower_slide": 0.005, "middle_slide": 0.005, "third_slide": 0.005}
HZ = 60
CAMS = {
    "iso": ((1.55, -2.05, 1.35), (0.0, -0.35, 0.45)),
    "front": ((0.0, -2.6, 0.9), (0.0, 0.0, 0.45)),
}


def probe_stage(stage: Usd.Stage, root: str) -> dict:
    """Joints (type/limits/drive), and the asset's named manipulation sites in world coords."""
    cache = UsdGeom.XformCache()
    info: dict = {"joints": {}, "sites": {}}
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(root):
            continue
        if prim.IsA(UsdPhysics.Joint) and prim.GetName() in JOINTS:
            attrs = {a.GetName(): a.Get() for a in prim.GetAttributes()
                     if a.GetName().startswith(("physics:lowerLimit", "physics:upperLimit", "drive:", "physics:body"))}
            info["joints"][prim.GetName()] = {"type": prim.GetTypeName(),
                                              **{k: (str(v) if not isinstance(v, (int, float)) else v)
                                                 for k, v in attrs.items()}}
        if "/Manipulation/" in path and prim.IsA(UsdGeom.Xformable):
            m = cache.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            rack = path.split("/Manipulation/")[0].rsplit("/", 1)[-1]
            info["sites"][f"{rack}/{path.split('/Manipulation/')[1]}"] = [round(t[0], 4), round(t[1], 4), round(t[2], 4)]
    return info


def main() -> int:
    import isaaclab.sim as sim_utils  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415
    from isaaclab.actuators import ImplicitActuatorCfg  # noqa: PLC0415
    from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg  # noqa: PLC0415
    from isaaclab.sim import SimulationContext  # noqa: PLC0415

    from dishsim import config  # noqa: PLC0415
    from dishsim.media import CameraRig, VideoWriter  # noqa: PLC0415

    check("asset present", os.path.isfile(args_cli.usd), args_cli.usd)
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / HZ, device=args_cli.device))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.9))
    light.func("/World/light", light)
    key = sim_utils.DistantLightCfg(intensity=2500.0, angle=1.0)
    key.func("/World/key", key, translation=(0.0, 0.0, 3.0),
             orientation=(0.9239, 0.3827, 0.0, 0.0))  # WXYZ: tilt 45 deg about +X

    # The asset carries its own drives (README: authored motor gains) and its own base_fixed
    # joint to the world, so nothing is overridden here: stiffness/damping None = read the USD.
    dw_cfg = ArticulationCfg(
        prim_path=PRIM_PATH,
        spawn=sim_utils.UsdFileCfg(usd_path=args_cli.usd, activate_contact_sensors=False),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), joint_pos=CLOSED),
        actuators={"all": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=None, damping=None)},
    )
    dw = Articulation(dw_cfg)

    stage = omni.usd.get_context().get_stage()
    info = probe_stage(stage, PRIM_PATH)
    print("[INFO] joints:", json.dumps(info["joints"], indent=1))
    print("[INFO] sites:", json.dumps(info["sites"], indent=1))
    check("four authored joints found", set(info["joints"]) == set(JOINTS), str(sorted(info["joints"])))
    if args_cli.probe:
        finish()
        return 0

    # this repo's props, tinted per class exactly as the benchmark renders them
    def prop(name: str, cls: str) -> RigidObject:
        color = config.display_color(cls)
        return RigidObject(RigidObjectCfg(
            prim_path=f"/World/{name}",
            spawn=sim_utils.UsdFileCfg(
                usd_path=os.path.join(config.ASSETS_DIR, "props", f"{cls}_physics.usd"),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
                visual_material=(sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color)) if color else None),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(2.0 if name == "plate" else 2.5, 2.0, 0.3)),
        ))

    plate, cup = prop("plate", "plate"), prop("cup", "cup")
    rig = CameraRig(CAMS)
    sim.reset()
    rig.apply_poses(sim.device)
    dt = sim.get_physics_dt()
    jidx = {n: dw.joint_names.index(n) for n in JOINTS}
    limits = dw.data.joint_pos_limits[0].cpu().numpy()
    print("[INFO] joint order", dw.joint_names)
    for n in JOINTS:
        print(f"[INFO] {n}: limits [{limits[jidx[n], 0]:+.3f}, {limits[jidx[n], 1]:+.3f}]")

    os.makedirs(args_cli.out_dir, exist_ok=True)
    video = None if args_cli.no_video else VideoWriter(os.path.join(args_cli.out_dir, "articulation.mp4"), fps=30)
    state = {"frame": 0}
    target = dw.data.default_joint_pos.clone()

    def step(n: int, capture: bool = True) -> None:
        for _ in range(n):
            dw.set_joint_position_target(target)
            dw.write_data_to_sim()
            sim.step()
            dw.update(dt)
            plate.update(dt)
            cup.update(dt)
            if capture and video is not None and state["frame"] % 2 == 0:
                rig.update(dt)
                video.add(rig.grab_one("iso"))
            state["frame"] += 1

    def ramp(goal: dict[str, float], seconds: float) -> None:
        start = target.clone()
        n = int(seconds * HZ)
        for k in range(1, n + 1):
            a = k / n
            for j, v in goal.items():
                target[0, jidx[j]] = (1 - a) * start[0, jidx[j]] + a * v
            step(1)

    def measure(tag: str, goal: dict[str, float]) -> dict:
        q = dw.data.joint_pos[0].cpu().numpy()
        err = {j: float(q[jidx[j]] - goal[j]) for j in goal}
        ok = all(abs(err[j]) <= TOL[j] for j in goal)
        check(f"{tag}: joints at target", ok, " ".join(f"{j}={err[j]:+.4f}" for j in goal))
        return {"state": tag, "position": {j: float(q[jidx[j]]) for j in JOINTS}, "error": err}

    # warm-up renders (first frames can come back black while the renderer spins up)
    for _ in range(10):
        step(1, capture=False)
        rig.update(dt)
    rig.save_stills(args_cli.out_dir, "closed")

    evidence: dict = {"asset": args_cli.usd, "sim": "Isaac Sim 4.5.0 / Isaac Lab 2.1.1 (PhysX)",
                      "device": str(sim.device), "joints": dw.joint_names, "measurements": []}
    step(int(0.75 * HZ))
    evidence["measurements"].append(measure("closed", CLOSED))
    ramp({"door_hinge": EXTENDED["door_hinge"]}, 2.0)
    step(int(0.75 * HZ))
    evidence["measurements"].append(measure("door_open", {"door_hinge": EXTENDED["door_hinge"]}))
    ramp({j: EXTENDED[j] for j in JOINTS if j != "door_hinge"}, 2.5)
    step(int(0.75 * HZ))
    evidence["measurements"].append(measure("racks_extended", EXTENDED))
    rig.update(dt)
    rig.save_stills(args_cli.out_dir, "racks_extended")

    # --- load: teleport the dishes onto sites the asset names, let them settle ---------------
    def pick(prefix: str, *needles: str) -> tuple[str, list[float]] | None:
        for name, pos in info["sites"].items():
            if name.startswith(prefix) and any(nd in name.lower() for nd in needles):
                return name, pos
        return None

    plate_site = pick("LowerRack/", "plate", "dish")
    cup_site = pick("MiddleRack/", "cup", "glass", "mug")
    drops = []
    # the repo's plate prop is a flat disc (axis +Z): stand it on edge, disc normal along the
    # rack's y (front/back) as plates sit between tines; the cup prop is upright at identity
    STAND = (0.7071068, 0.7071068, 0.0, 0.0)  # WXYZ, +90 deg about X
    UPRIGHT = (1.0, 0.0, 0.0, 0.0)
    for obj, site, joint, hover, quat in ((plate, plate_site, "lower_slide", 0.03, STAND),
                                          (cup, cup_site, "middle_slide", 0.02, UPRIGHT)):
        if site is None:
            print(f"[WARN] no site for {obj.cfg.prim_path} — skipping that drop")
            continue
        name, pos = site
        p = [pos[0], pos[1] + EXTENDED[joint], pos[2] + hover]  # site rides the extended rack
        pose = torch.tensor([[p[0], p[1], p[2], *quat]], device=sim.device)
        obj.write_root_pose_to_sim(pose)
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))
        drops.append((obj, name, p))
        print(f"[INFO] {obj.cfg.prim_path} -> {name} @ {np.round(p, 3).tolist()}")
    check("dish sites found in the asset", len(drops) == 2, f"{len(drops)}/2")
    step(int(2.0 * HZ))
    before = {}
    for obj, name, p in drops:
        q = obj.data.root_pos_w[0].cpu().numpy()
        before[obj.cfg.prim_path] = q.tolist()
        drift = float(np.linalg.norm(q[:2] - np.array(p[:2])))
        check(f"{os.path.basename(obj.cfg.prim_path)} settled on {name}",
              drift < 0.05 and q[2] > p[2] - 0.08, f"xy drift {drift * 1e3:.1f} mm, z {q[2]:.3f}")
    rig.update(dt)
    stills = rig.save_stills(args_cli.out_dir, "loaded_extended")
    for s in stills:
        check("still written", os.path.getsize(s) > 20_000, s)

    # --- retract with the load, close ---------------------------------------------------------
    ramp({j: 0.0 for j in JOINTS if j != "door_hinge"}, 2.5)
    step(int(0.75 * HZ))
    evidence["measurements"].append(measure("racks_retracted", {j: 0.0 for j in JOINTS if j != "door_hinge"}))
    retention = {}
    for obj, name, p in drops:
        joint = "lower_slide" if "LowerRack" in name else "middle_slide"
        q = obj.data.root_pos_w[0].cpu().numpy()
        b = np.array(before[obj.cfg.prim_path])
        rode = float(q[1] - b[1])  # the rack moved +y by -EXTENDED[joint]
        ok = abs(rode + EXTENDED[joint]) < 0.03 and abs(q[2] - b[2]) < 0.03
        retention[obj.cfg.prim_path] = {"rode_y_m": rode, "dz_m": float(q[2] - b[2]), "retained": bool(ok)}
        check(f"{os.path.basename(obj.cfg.prim_path)} retained through retraction", ok,
              f"rode {rode:+.3f} m (rack {-EXTENDED[joint]:+.3f}), dz {q[2] - b[2]:+.3f}")
    ramp({"door_hinge": 0.0}, 2.0)
    step(int(1.0 * HZ))
    evidence["measurements"].append(measure("closed_again", {"door_hinge": 0.0}))
    if video is not None:
        video.close()
        check("video written", video.frames > 100 and os.path.getsize(video.path) > 100_000,
              f"{video.path} ({video.frames} frames)")
        evidence["video"] = video.path
    evidence["retention"] = retention
    from dishsim.checks import FAILURES  # noqa: PLC0415

    evidence["result"] = "PASS" if not FAILURES else "FAIL"
    # numpy scalars are not JSON-native; coerce rather than truncate the report mid-write
    blob = json.dumps(evidence, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    with open(os.path.join(args_cli.out_dir, "evidence.json"), "w") as f:
        f.write(blob)
    finish()
    return 0


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
    except Exception:  # noqa: BLE001 — report, but NEVER skip the shutdown release (Kit spins otherwise)
        import traceback  # noqa: PLC0415

        traceback.print_exc()
        print("[RESULT] FAIL: unhandled exception")
    release_sim_for_close()
    simulation_app.close()
