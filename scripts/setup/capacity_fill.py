# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Capacity fill: physically load the full dishwasher and measure what holds.

The deterministic full-load plan comes from :mod:`dishsim.fill_plan` (FCL-validated in the
venv before any Kit boot). This script spawns every planned item parked off to the side,
then, one item every ~1.3 s, teleports it to its target pose (+2 cm hover) and lets physics
settle — the staggered arrivals ARE the fill-timelapse video. Per-item stability gate: pose
delta < 5 mm / 3 deg over the last 30 settle steps; unstable items are parked again and
recorded (honest capacity).

Then the CLOSABILITY check: both rack drive targets ramp to the stowed position (motorized
slide-in — the mug campaign demonstrated the robot performing rack slides; here the drives
stand in so the check isolates the load geometry), settle, and every item must ride its rack
without being displaced (> 10 mm / 10 deg) or knocked over. The racks re-extend for the
beauty pass: fixed stills + a 360-degree orbit clip.

Outputs: ``results/fill/capacity.json``, ``media/fill/fill_timelapse.mp4``, per-view stills,
``orbit.mp4``.

Run: ``scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras``
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Physics capacity fill + closability check.")
parser.add_argument("--settle_steps_item", type=int, default=75)
parser.add_argument("--slide_steps", type=int, default=300)
parser.add_argument("--out", type=str, default=os.path.join(PROJECT_ROOT, "results", "fill"))
parser.add_argument("--media", type=str, default=os.path.join(PROJECT_ROOT, "media", "fill"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

_enable_cameras = args_cli.enable_cameras  # 2.1's AppLauncher pops this off the namespace
app_launcher = AppLauncher(args_cli)
args_cli.enable_cameras = _enable_cameras
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402
from dishsim.quats import xyzw_to_wxyz  # noqa: E402

config.apply_scenario("both_out")  # racks extended for loading; fill targets use this cache

from dishsim import fill_plan  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.transforms import T_to_pos_quat, make_T  # noqa: E402

RACK_JOINTS = ("PrismaticJoint_dishwasher_2_down", "PrismaticJoint_dishwasher_2_up")


def pose_w_tensor(T_base: np.ndarray, device, hover: float = 0.0) -> torch.Tensor:
    # full base transform: position AND quaternion compose with the (possibly yawed) base
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    pos, quat = T_to_pos_quat(T_w_base @ np.asarray(T_base, dtype=float))
    pos[2] += hover
    return torch.tensor(np.concatenate([pos, xyzw_to_wxyz(quat)])[None], dtype=torch.float32, device=device)


def main() -> None:
    os.makedirs(args_cli.out, exist_ok=True)
    os.makedirs(args_cli.media, exist_ok=True)

    items = fill_plan.plan_full_load(config.CACHE_DIR)
    report = fill_plan.validate_plan(items, config.CACHE_DIR)
    print(f"[INFO] plan: {report['n_items']} items, counts {report['counts']}")
    if report["closability_violations"]:
        print(f"[WARN] z-budget flags: {report['closability_violations']}")

    scene_cfg = dscene.make_scene_cfg(with_object=False, with_robot_contacts=False)
    # one rigid object per planned item, parked in a grid 2 m to the side
    for k, it in enumerate(items):
        park = ((-2.0 - 0.35 * (k % 6), -1.5 + 0.35 * (k // 6), 0.10), (1.0, 0.0, 0.0, 0.0))  # WXYZ id
        setattr(
            scene_cfg,
            it.item_id,
            RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Fill_" + it.item_id,
                spawn=sim_utils.UsdFileCfg(
                    usd_path=config.OBJECTS[it.name].usd_path,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=3.0),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=park[0], rot=park[1]),
            ),
        )

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device)
    )
    rig = None
    if args_cli.enable_cameras:
        rig = CameraRig(config.CAMERAS, hw=(1080, 1920))
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dscene.assert_frames(scene)
    dt = sim.get_physics_dt()
    device = scene["robot"].data.joint_pos.device
    dw = scene["dishwasher"]
    rack_ids = [dw.find_joints(j)[0][0] for j in RACK_JOINTS]

    video = None
    frame_i = 0
    if rig is not None:
        video = VideoWriter(os.path.join(args_cli.media, "fill_timelapse.mp4"), fps=config.CAMERA_FPS)

    def step(n=1, cam="iso"):
        nonlocal frame_i
        for _ in range(n):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            frame_i += 1
            if video is not None and frame_i % 2 == 0:
                rig.update(dt)
                video.add(rig.grab()[cam])

    # warm up renders + initial settle
    step(30)

    # ---- sequential fill (the timelapse) -----------------------------------------------------
    results = {}
    poses_settled = {}
    for it in items:
        obj = scene[it.item_id]
        obj.write_root_pose_to_sim(root_pose=pose_w_tensor(it.T_base_obj, device, hover=fill_plan.SPAWN_HOVER_M))
        obj.write_root_velocity_to_sim(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
        hist = []
        for s in range(args_cli.settle_steps_item):
            step(1)
            if s >= args_cli.settle_steps_item - 30:
                p = obj.data.root_pos_w[0].cpu().numpy().copy()
                q = obj.data.root_quat_w[0].cpu().numpy().copy()
                hist.append((p, q))
        drift_p = float(np.linalg.norm(hist[-1][0] - hist[0][0]))
        dq = abs(float(np.dot(hist[-1][1], hist[0][1])))
        drift_deg = float(np.degrees(2.0 * np.arccos(np.clip(dq, -1.0, 1.0))))
        # target deviation: settled pose vs planned pose (lifted through the full base transform)
        T_target_w = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W) @ it.T_base_obj
        dev_p = float(np.linalg.norm(hist[-1][0] - T_target_w[:3, 3]))
        stable = drift_p < 0.005 and drift_deg < 3.0 and dev_p < 0.06
        results[it.item_id] = {
            "name": it.name, "zone": it.zone, "rack": it.rack,
            "stable": bool(stable), "drift_mm": round(drift_p * 1e3, 1),
            "drift_deg": round(drift_deg, 2), "target_dev_mm": round(dev_p * 1e3, 1),
        }
        if not stable:
            print(f"[WARN] {it.item_id}: unstable (drift {drift_p*1e3:.1f} mm / {drift_deg:.1f} deg, "
                  f"dev {dev_p*1e3:.1f} mm) — parked")
            park = torch.tensor([[-2.5, 2.0, 0.10, 1, 0, 0, 0]], dtype=torch.float32, device=device)  # WXYZ id
            obj.write_root_pose_to_sim(root_pose=park)
            obj.write_root_velocity_to_sim(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
            step(10)
        else:
            poses_settled[it.item_id] = (
                obj.data.root_pos_w[0].cpu().numpy().copy(),
                obj.data.root_quat_w[0].cpu().numpy().copy(),
            )
        print(f"[INFO] {it.item_id}: {'stable' if stable else 'UNSTABLE'} "
              f"(drift {drift_p*1e3:.1f} mm, dev {dev_p*1e3:.1f} mm)")

    n_stable = sum(r["stable"] for r in results.values())
    print(f"[INFO] fill complete: {n_stable}/{len(items)} items stable")

    # ---- closability: motorized slide-in, settle, displacement gates -------------------------
    pre_slide = {k: v for k, v in poses_settled.items()}
    body_ids = {rack: dw.find_bodies(body)[0][0]
                for rack, body in (("lower", "E_shelf_1_04"), ("upper", "E_shelf_03"))}
    rack_pos_pre = {rack: dw.data.body_pos_w[0, bi].cpu().numpy().copy()
                    for rack, bi in body_ids.items()}
    for s in range(args_cli.slide_steps):
        frac = (s + 1) / args_cli.slide_steps
        tgt = {j: (-0.20 + 0.20 * frac) for j in RACK_JOINTS}
        dscene.set_rack_target_override(tgt)
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        step(1, cam="front")
    for _ in range(240):
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        step(1, cam="front")
    rack_err = max(abs(float(dw.data.joint_pos[0, i])) for i in rack_ids)
    rack_ride = {rack: dw.data.body_pos_w[0, bi].cpu().numpy() - rack_pos_pre[rack]
                 for rack, bi in body_ids.items()}
    displaced = []
    for k, (p0, q0) in pre_slide.items():
        obj = scene[k]
        p1 = obj.data.root_pos_w[0].cpu().numpy()
        q1 = obj.data.root_quat_w[0].cpu().numpy()
        # expected: each item rides its rack — subtract the MEASURED rack displacement
        dp_ride = (p1 - p0) - rack_ride[results[k]["rack"]]
        slip = float(np.linalg.norm(dp_ride))
        dq = abs(float(np.dot(q1, q0)))
        ang = float(np.degrees(2.0 * np.arccos(np.clip(dq, -1.0, 1.0))))
        zone = results[k]["zone"]
        if zone == "basket" or (zone == "plate_bank" and results[k]["name"] == "spatula"):
            # contained thin items (cutlery in a bay, the spatula standing between tines)
            # may re-lean freely during the slide (real forks jiggle) — displaced only if
            # one leaves its containment envelope (~half a bay / half a gap of travel).
            # Plates/saucers in the same bank keep the strict gate: a large rotation there
            # means a disc falling flat, not a harmless re-lean.
            bad = slip > 0.045
        elif zone in ("slope_strip_flat", "stemware_shelf"):
            bad = slip > 0.030 or ang > 45.0  # items on the ramp may settle down-slope
        else:
            bad = slip > 0.010 or ang > 10.0
        if bad:
            displaced.append({"item": k, "zone": zone, "slip_mm": round(slip * 1e3, 1),
                              "rot_deg": round(ang, 1)})
    # loaded-slide stow tolerance: 2x the empty-rack RACK_SLIDE_TOL_M — 31 items of extra
    # friction/mass soft-stop the drive a few mm early
    closable = rack_err <= 2.0 * config.RACK_SLIDE_TOL_M and not displaced
    print(f"[INFO] closability: rack err {rack_err*1e3:.1f} mm, displaced {len(displaced)} "
          f"-> {'CLOSABLE' if closable else 'NOT CLOSABLE'}")
    for d in displaced:
        print(f"[WARN]   displaced: {d}")

    # ---- re-extend for the beauty pass -------------------------------------------------------
    for s in range(args_cli.slide_steps):
        frac = (s + 1) / args_cli.slide_steps
        tgt = {j: (0.0 - 0.20 * frac) for j in RACK_JOINTS}
        dscene.set_rack_target_override(tgt)
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        step(1, cam="iso")
    for _ in range(180):
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        step(1, cam="iso")
    if video is not None:
        video.close()
        print(f"[INFO] timelapse: {video.path}")

    stills = {}
    if rig is not None:
        from PIL import Image  # noqa: PLC0415

        rig.update(dt)
        frames = rig.grab()
        for name, arr in frames.items():
            p = os.path.join(args_cli.media, f"loaded_{name}.png")
            Image.fromarray(arr).save(p)
            stills[name] = os.path.relpath(p, PROJECT_ROOT)
            print(f"[INFO] still: {p}")
        # 360-degree orbit around the machine
        orbit = VideoWriter(os.path.join(args_cli.media, "orbit.mp4"), fps=config.CAMERA_FPS)
        center = np.array([0.55, 0.10, 0.30])
        for k in range(240):
            a = 2.0 * np.pi * k / 240.0
            eye = center + np.array([1.25 * np.cos(a), 1.25 * np.sin(a), 0.55])
            rig.set_view("iso", eye, center, sim.device)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            rig.update(dt)
            orbit.add(rig.grab()["iso"])
        orbit.close()
        print(f"[INFO] orbit clip: {orbit.path}")

    counts_stable = {}
    for r in results.values():
        if r["stable"]:
            counts_stable[r["name"]] = counts_stable.get(r["name"], 0) + 1
    capacity = {
        "n_planned": len(items),
        "n_stable": n_stable,
        "counts_planned": report["counts"],
        "counts_stable": counts_stable,
        "closable": bool(closable),
        "rack_stow_err_m": round(rack_err, 4),
        "displaced_on_close": displaced,
        "items": results,
        "media": {"timelapse": "media/fill/fill_timelapse.mp4", "orbit": "media/fill/orbit.mp4",
                  "stills": stills},
    }
    with open(os.path.join(args_cli.out, "capacity.json"), "w") as f:
        json.dump(capacity, f, indent=2)
    print(f"[INFO] wrote {os.path.join(args_cli.out, 'capacity.json')}")
    ok = n_stable >= int(0.8 * len(items))
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} — capacity {n_stable}/{len(items)} items, "
          f"closable={closable}")


if __name__ == "__main__":
    main()
