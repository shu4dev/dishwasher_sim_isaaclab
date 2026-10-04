#!/usr/bin/env python3
"""Convention and collider tests of the UR5e + 2F-85 on the LIVE asset (plan 2026-09-29-easy-s0 Phase 0.5, D7, D12).

    code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_conventions.py --headless --enable_cameras \\
        --run-id p0_conventions

Two robots stand in one stage: A as the episode builds it today (gripper de-instanced, arm colliders still
instanced) and B with the WHOLE robot de-instanced (D12's candidate). Measured, all as ANALYSIS (arms and bowl are
teleported into place; nothing here is a counted trial):

1. Arm-collider press test (D12): a 0.5 kg, 30 x 30 x 1 cm plate is dropped from 1 m onto each arm at HOME_Q, centred
   over the forearm and clear of the gripper's footprint. Live arm colliders stop it on the forearm (~0.6 m); dead
   (instanced) ones let it fall to the ground. The plate's resting height and the contact reports per link decide.
2. Conventions on robot B: TCP axes, the wrist-3 -> TCP direction (approach sign), the fingertip-pad centroids and
   contact-face normals in the TCP frame open and closed (jaw axis, pad normals), all 12 joint angles during an
   unloaded close (finger-joint signs vs ur5e.INNER_FINGER_SIGNS and the asset's mimic gearings).
3. Loaded close (D7's H2 runtime test, reused in Phase 1b): the rim wall of a HOTEC v2 bowl between B's pads
   (probe G0 geometry, 25 mm insert), all 12 joints logged per step, pad forces from an Isaac Lab ContactSensor AND
   from the harness's contact reports (cross-check of the harness monitor).

Writes data/artifacts/<run-id>/conventions.json (+ conventions.jsonl, frames/*.png). The Kit-free pytest reads a copy in
code/frigidaire/tests/fixtures/robot/conventions.json. [RESULT] PASS = every measurement completed (the assertions
live in code/frigidaire/tests/test_robot_conventions.py).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
BOWL_USD = ROOT / "data/assets/models/hotec_wheatstraw/v2/bowl.usda"
TCP_TARGET = (.45, 0., .30)          # base frame, TCP straight down (the grip probe's pose)
B_OFFSET = (3., 0., 0.)              # robot B stands 3 m away from robot A
PLATE = {"size_m": (.30, .30, .01), "mass_kg": .5, "drop_z_m": 1., "fall_s": 1.5, "caught_above_m": .30}
INSERT_MM = 25.
RIM_R, WALL_T, HEIGHT = .074, .0035, .075


def T_of(pos, quat_xyzw):
    import numpy as np
    x, y, z, w = quat_xyzw
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = pos
    return T


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--effort", type=float, default=8.)
    parser.add_argument("--stiffness", type=float, default=400.)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    cameras = bool(getattr(args, "enable_cameras", False))
    run_id = args.run_id or f"conventions_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    out_dir = ROOT / "data/artifacts" / run_id
    if out_dir.exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite {out_dir}")
    (out_dir / "frames").mkdir(parents=True)
    app = AppLauncher(args).app
    res = {"result": "FAIL", "run_id": run_id, "started_utc": datetime.now(timezone.utc).isoformat(),
           "label": "ANALYSIS (arms and bowl teleported into place; not a counted trial)",
           "args": {"effort": args.effort, "stiffness": args.stiffness, "insert_mm": INSERT_MM}}
    log = open(out_dir / "conventions.jsonl", "w", buffering=1)
    try:
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
        run(args, res, log, out_dir, cameras)
        res["result"] = "PASS"
    except Exception:
        res["error"] = traceback.format_exc()
        print(res["error"], flush=True)
    log.close()
    res["finished_utc"] = datetime.now(timezone.utc).isoformat()
    (out_dir / "conventions.json").write_text(json.dumps(res, indent=1) + "\n")
    press = {k: (v or {}).get("verdict") for k, v in res.get("press", {}).items()}
    print(f"[RESULT] {res['result']} robot conventions {run_id}: press {press}, "
          f"signs {res.get('unloaded_close', {}).get('signs_match')}, pads {res.get('pads', {}).get('closed', {}).get('gap_mm')}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


def run(args, res, log, out_dir, cameras):
    import numpy as np
    import torch
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from dishsim_frigidaire.robot import harness as H, kin, ur5e

    def jl(event, **payload):
        log.write(json.dumps({"event": event, "sim_s": round(state["tick"] / 120., 4), **payload}) + "\n")

    state = {"tick": 0}
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2000.).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.))
    robots = {}
    for name, off in (("A", (0., 0., 0.)), ("B", B_OFFSET)):
        robots[name] = Articulation(ur5e.robot_cfg(prim_path=f"/World/Robot{name}", pos=off, gripper_effort=args.effort,
                                                   gripper_stiffness=args.stiffness))
    stage = omni.usd.get_context().get_stage()
    res["deinstanced"] = {"A_gripper": ur5e.deinstance_gripper(stage, root="/World/RobotA/Gripper"),
                          "B_robot": H.deinstance(stage, "/World/RobotB")}
    res["pad_colliders"] = {n: ur5e.pad_material(stage, root=f"/World/Robot{n}/Gripper") for n in robots}
    res["authored"] = {n: H.author_sleep_and_reports(stage, [], robot_root=f"/World/Robot{n}") for n in robots}
    bowl = RigidObject(RigidObjectCfg(prim_path="/World/Bowl", spawn=sim_utils.UsdFileCfg(
        usd_path=str(BOWL_USD), rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 1.5, .0))))
    H.author_sleep_and_reports(stage, ["/World/Bowl"], robot_root="/World/RobotB")
    support = RigidObject(RigidObjectCfg(prim_path="/World/Support", spawn=sim_utils.CylinderCfg(
        radius=.03, height=.1, rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        collision_props=sim_utils.CollisionPropertiesCfg()), init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, -1.5, .05))))
    walls = {}                                                     # the press-test plates (one per robot)
    for name in robots:
        walls[name] = RigidObject(RigidObjectCfg(prim_path=f"/World/Plate{name}", spawn=sim_utils.CuboidCfg(
            size=PLATE["size_m"], rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=PLATE["mass_kg"]), collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.8, .45, .2))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(-4. - 2 * len(walls), 4., .05))))
    H.author_sleep_and_reports(stage, [f"/World/Plate{n}" for n in robots], robot_root="/World/RobotB")
    pads = ContactSensor(ContactSensorCfg(prim_path="/World/RobotB/Gripper/Robotiq_2F_85/.*_inner_finger",
                                          filter_prim_paths_expr=["/World/Bowl"], update_period=0.0))
    cam = None
    if cameras:
        from isaaclab.sensors import Camera, CameraCfg
        cam = Camera(CameraCfg(prim_path="/World/Cam", height=720, width=1280, data_types=["rgb"],
                               spawn=sim_utils.PinholeCameraCfg(focal_length=18.)))
    sim.reset()
    monitor = H.ContactMonitor(1 / 120, roots=("/World/RobotA", "/World/RobotB"))
    monitor.pad_N = {"RobotB:pad_L": 0., "RobotB:pad_R": 0., "RobotA:pad_L": 0., "RobotA:pad_R": 0.}
    T_w3_tcp = T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
    info = {}
    for name, r in robots.items():
        names, bodies = list(r.joint_names), list(r.body_names)
        info[name] = {"names": names, "bodies": bodies, "arm": [names.index(j) for j in ur5e.ARM_JOINTS],
                      "fj": names.index("finger_joint"), "w3": bodies.index("wrist_3_link"),
                      "target": r.data.default_joint_pos.clone()}
    res["joint_names"] = info["B"]["names"]
    res["body_names"] = info["B"]["bodies"]
    T_tcp = np.eye(4)
    T_tcp[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    T_tcp[:3, 3] = TCP_TARGET
    sols = kin.ik_wrist3_all(T_tcp @ np.linalg.inv(T_w3_tcp), q_seed=np.asarray(ur5e.HOME_Q))
    q_down = min(sols, key=lambda s: float(np.abs(s - np.asarray(ur5e.HOME_Q)).max()))
    for name, r in robots.items():
        t = info[name]["target"]
        t[0, info[name]["arm"]] = torch.tensor(q_down, dtype=t.dtype)
        r.write_joint_state_to_sim(t, torch.zeros_like(t))

    def step(n=1, on=None):
        for _ in range(n):
            for name, r in robots.items():
                r.set_joint_position_target(info[name]["target"])
                r.write_data_to_sim()
            sim.step()
            state["tick"] += 1
            for r in robots.values():
                r.update(sim.get_physics_dt())
            for o in (bowl, support, *walls.values()):
                o.update(sim.get_physics_dt())
            pads.update(sim.get_physics_dt())
            monitor.poll(state["tick"])
            if on is not None:
                on()

    def tcp(name):
        r, i = robots[name], info[name]["w3"]
        p = r.data.body_link_pos_w[0, i].cpu().numpy()
        q = r.data.body_link_quat_w[0, i].cpu().numpy()
        return T_of(p, (q[1], q[2], q[3], q[0])) @ T_w3_tcp

    def body_T(name, body):
        r = robots[name]
        i = info[name]["bodies"].index(body)
        p = r.data.body_link_pos_w[0, i].cpu().numpy()
        q = r.data.body_link_quat_w[0, i].cpu().numpy()
        return T_of(p, (q[1], q[2], q[3], q[0]))

    def shot(tag, eye, target):
        if cam is None:
            return None
        from PIL import Image
        cam.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32), torch.tensor([target], dtype=torch.float32))
        for _ in range(30):
            sim.render()
        cam.update(sim.get_physics_dt())
        path = out_dir / "frames" / f"{tag}.png"
        Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(path)
        return f"frames/{tag}.png"

    step(120)
    frames = {}
    # ------------------------------------------------------------------ 2. conventions on robot B
    from pxr import Usd, UsdGeom, UsdPhysics
    tips = {}
    for side in ("left", "right"):
        body_prim = stage.GetPrimAtPath(f"/World/RobotB/Gripper/Robotiq_2F_85/{side}_inner_finger")
        mesh = next(p for p in Usd.PrimRange(body_prim) if p.HasAPI(UsdPhysics.CollisionAPI) and "fingertips" in p.GetName())
        pts = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
        counts = np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexCountsAttr().Get())
        idx = np.asarray(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get())
        M = np.array(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        Bw = np.array(UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        tips[side] = {"B_M": np.linalg.inv(Bw) @ M, "pts": pts, "counts": counts, "idx": idx, "path": str(mesh.GetPath())}

    def pad_geometry():
        """Pad centroids (mm) and contact-face normals in the TCP frame of robot B."""
        T = tcp("B")
        Ti = np.linalg.inv(T)
        out = {}
        for side, t in tips.items():
            W = body_T("B", f"{side}_inner_finger") @ t["B_M"]
            P = (Ti @ W @ np.c_[t["pts"], np.ones(len(t["pts"]))].T).T[:, :3]
            out[side] = {"P": P, "centroid": P.mean(0)}
        for side, other in (("left", "right"), ("right", "left")):
            t, P = tips[side], out[side]["P"]
            toward = out[other]["centroid"] - out[side]["centroid"]
            toward = toward / max(np.linalg.norm(toward), 1e-12)
            normals, areas, k = [], [], 0
            for c in t["counts"]:
                f = t["idx"][k:k + c]
                k += c
                a, b, d = P[f[0]], P[f[1]], P[f[2]]
                n = np.cross(b - a, d - a)
                area = np.linalg.norm(n)
                if area < 1e-14:
                    continue
                n = n / area
                if np.dot(n, P[f].mean(0) - out[side]["centroid"]) < 0:     # outward
                    n = -n
                normals.append(n)
                areas.append(area)
            normals, areas = np.asarray(normals), np.asarray(areas)
            face = normals @ toward > .8                                     # faces looking at the other pad
            n = (normals[face] * areas[face, None]).sum(0) if face.any() else np.full(3, np.nan)
            n = n / np.linalg.norm(n) if face.any() else n
            out[side]["face_normal_tcp"] = n
            out[side]["face_area_mm2"] = float(areas[face].sum() / 2 * 1e6) if face.any() else 0.
        return {side: {"centroid_mm": np.round(v["centroid"] * 1e3, 2).tolist(),
                       "min_mm": np.round(v["P"].min(0) * 1e3, 2).tolist(), "max_mm": np.round(v["P"].max(0) * 1e3, 2).tolist(),
                       "face_normal_tcp": np.round(v["face_normal_tcp"], 4).tolist(), "face_area_mm2": round(v["face_area_mm2"], 2)}
                for side, v in out.items()}

    T = tcp("B")
    W3 = body_T("B", "wrist_3_link")
    res["tcp"] = {"axes_world": np.round(T[:3, :3], 5).tolist(), "commanded_axes": T_tcp[:3, :3].tolist(),
                  "position_base_frame": np.round(T[:3, 3] - np.asarray(B_OFFSET), 5).tolist(),
                  "wrist3_to_tcp_world": np.round(T[:3, 3] - W3[:3, 3], 5).tolist(),
                  "z_dot_wrist3_to_tcp": round(float(T[:3, 2] @ (T[:3, 3] - W3[:3, 3])), 6)}
    pads_open = pad_geometry()
    jl("pads_open", pads=pads_open, tcp=res["tcp"])
    B = robots["B"]
    nB = info["B"]["names"]
    gripper_ids = [i for i in range(len(nB)) if i not in info["B"]["arm"]]
    traj = []

    def log_joints(tag, extra=None):
        q = B.data.joint_pos[0].cpu().numpy()
        row = {"t": state["tick"], "tag": tag, "q": {nB[i]: round(float(q[i]), 5) for i in range(len(nB))}}
        if extra:
            row.update(extra)
        traj.append(row)
        jl("joints", **row)

    for s in np.linspace(0., .8, 90):                               # the rig's close: 90 steps to 0.8 rad
        ur5e.set_gripper(info["B"]["target"], nB, float(s))
        step(1, lambda: log_joints("unloaded_close"))
    step(60, lambda: log_joints("unloaded_hold"))
    pads_closed = pad_geometry()
    q = B.data.joint_pos[0].cpu().numpy()
    fj = float(q[info["B"]["fj"]])
    ratios = {nB[i]: round(float(q[i]) / fj, 4) for i in gripper_ids if nB[i] != "finger_joint"}
    gearing = {"right_outer_knuckle_joint": -1., "right_inner_finger_joint": -1., "right_inner_finger_knuckle_joint": 1.,
               "left_inner_finger_knuckle_joint": 1., "left_inner_finger_joint": 1.}          # mimic.usda :165 :193 :220 :246 :274
    expected = {k: -g for k, g in gearing.items()}                  # jointPosition + gearing * finger_joint = 0
    res["unloaded_close"] = {"finger_joint": round(fj, 5), "ratios": ratios, "expected_from_mimic": expected,
                             "code_inner_finger_signs": dict(ur5e.INNER_FINGER_SIGNS),
                             "signs_match": all(math.copysign(1, ratios[k]) == math.copysign(1, v) for k, v in expected.items() if k in ratios),
                             "trajectory": traj[::6] + [traj[-1]]}
    res["pads"] = {"open": pads_open, "closed": pads_closed,
                   "gap_mm": round(float(np.linalg.norm(np.asarray(pads_closed["left"]["centroid_mm"]) - pads_closed["right"]["centroid_mm"])), 2)}
    frames["closed_unloaded"] = shot("closed_unloaded", T[:3, 3] + np.array([.25, .30, .02]), T[:3, 3] + np.array([0., 0., -.03]))
    for s in np.linspace(.8, 0., 60):
        ur5e.set_gripper(info["B"]["target"], nB, float(s))
        step(1)
    step(60)
    # ------------------------------------------------------------------ 3. loaded close on the bowl rim (robot B)
    T = tcp("B")
    u = T[:3, 1].copy()
    u[2] = 0.
    u = u / np.linalg.norm(u)
    centre = T[:3, 3] + (RIM_R - WALL_T / 2) * u
    foot_z = T[2, 3] + INSERT_MM * 1e-3 - HEIGHT
    support.write_root_pose_to_sim(torch.tensor([[centre[0], centre[1], foot_z - .05, 1., 0., 0., 0.]], dtype=torch.float32))
    bowl.write_root_pose_to_sim(torch.tensor([[centre[0], centre[1], foot_z + .001, 1., 0., 0., 0.]], dtype=torch.float32))
    bowl.write_root_velocity_to_sim(torch.zeros(1, 6))
    step(60)
    traj = []
    fj_i = info["B"]["fj"]

    def loaded_row(tag):
        sens = [round(float(v), 3) for v in pads.data.force_matrix_w[0, :, 0].norm(dim=-1).cpu()]
        mon = {k: round(v, 3) for k, v in monitor.pad_N.items() if k.startswith("RobotB:")}
        per = {}                                    # bowl-robot normal force per robot collider label (pad vs finger side)
        for key, f in monitor.step_pairs.items():
            act = monitor.active.get(key)
            if act is None or not any("/World/Bowl" in x for x in key):
                continue
            other = key[0] if "/World/Bowl" in key[1] else key[1]
            if other.startswith("RobotB:"):
                per[other] = round(per.get(other, 0.) + f, 3)
        log_joints(tag, {"pad_sensor_N": sens, "pad_monitor_N": mon, "bowl_contact_N_by_robot_part": per,
                         "fj_cmd": round(float(info["B"]["target"][0, fj_i]), 4)})

    for s in np.linspace(0., .8, 90):
        ur5e.set_gripper(info["B"]["target"], nB, float(s))
        step(1, lambda: loaded_row("loaded_close"))
    step(60, lambda: loaded_row("loaded_hold"))
    q = B.data.joint_pos[0].cpu().numpy()
    fj = float(q[fj_i])
    last = traj[-1]
    sensor_sum = sum(last["pad_sensor_N"])
    monitor_sum = sum(last["pad_monitor_N"].values())
    res["loaded_close"] = {"finger_joint": round(fj, 5), "ratios": {nB[i]: round(float(q[i]) / fj, 4) for i in gripper_ids if nB[i] != "finger_joint"},
                           "pad_sensor_N": last["pad_sensor_N"], "pad_monitor_N": last["pad_monitor_N"],
                           "bowl_contact_N_by_robot_part": last["bowl_contact_N_by_robot_part"],
                           "note": "pad_sensor_N = Isaac Lab ContactSensor on the whole inner-finger BODY (pad + finger side) vs "
                                   "the bowl; pad_monitor_N = contact reports on the fingertip-pad COLLIDER only",
                           "monitor_vs_sensor_total": [round(monitor_sum, 3), round(sensor_sum, 3)],
                           "bowl_pairs": sorted("|".join(k) for k in monitor.active if any("Bowl" in x for x in k)),
                           "trajectory": traj[::3] + [traj[-1]]}
    frames["closed_loaded"] = shot("closed_loaded", T[:3, 3] + np.array([.25, .30, .02]), T[:3, 3] + np.array([0., 0., -.03]))
    for s in np.linspace(.8, 0., 60):
        ur5e.set_gripper(info["B"]["target"], nB, float(s))
        step(1)
    bowl.write_root_pose_to_sim(torch.tensor([[1.5, 1.5, .1, 1., 0., 0., 0.]], dtype=torch.float32))
    support.write_root_pose_to_sim(torch.tensor([[1.5, -1.5, .05, 1., 0., 0., 0.]], dtype=torch.float32))
    step(30)
    # ------------------------------------------------------------------ 1. arm-collider press test (A and B)
    home = np.asarray(ur5e.HOME_Q, dtype=float)
    for name, r in robots.items():
        t = info[name]["target"]
        t[0, info[name]["arm"]] = torch.tensor(home, dtype=t.dtype)
        ur5e.set_gripper(t, info[name]["names"], 0.)
        r.write_joint_state_to_sim(t, torch.zeros_like(t))
    step(120)
    press = {}
    arm_links = ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
    for name, r in robots.items():
        base = np.asarray(B_OFFSET if name == "B" else (0., 0., 0.))
        rad = {}
        for body in (*arm_links, "left_inner_finger", "right_inner_finger"):
            p = body_T(name, body)[:3, 3] - base
            rad[body] = {"r": round(float(np.hypot(p[0], p[1])), 4), "z": round(float(p[2]), 4),
                         "theta": round(float(math.atan2(p[1], p[0])), 4)}
        # over the forearm: halfway from the elbow to wrist 1, 30 x 30 cm, clear of the gripper hanging at r ~0.45 m
        mid = (body_T(name, "forearm_link")[:3, 3] + body_T(name, "wrist_1_link")[:3, 3]) / 2
        grip = (body_T(name, "left_inner_finger")[:3, 3] + body_T(name, "right_inner_finger")[:3, 3]) / 2
        clear_xy = float(np.max(np.abs(grip[:2] - mid[:2])))
        walls[name].write_root_pose_to_sim(torch.tensor([[mid[0], mid[1], PLATE["drop_z_m"], 1., 0., 0., 0.]], dtype=torch.float32))
        walls[name].write_root_velocity_to_sim(torch.zeros(1, 6))
        press[name] = {"link_radii": rad, "plate": {**PLATE, "centre_xy": np.round(mid[:2], 4).tolist(),
                                                    "forearm_mid_z_m": round(float(mid[2]), 4),
                                                    "gripper_offset_xy_m": round(clear_xy, 4),
                                                    "gripper_under_plate": bool(clear_xy < PLATE["size_m"][0] / 2 + .05)}}
    hits = {n: {} for n in robots}
    for k in range(round(PLATE["fall_s"] * 120)):
        step(1)
        for n in robots:
            for key, act in monitor.active.items():
                plate = [x for x in key if f"/Plate{n}" in x]
                if plate and act["last"] == state["tick"]:
                    other = key[0] if key[1] == plate[0] else key[1]
                    h = hits[n].setdefault(other, {"first_tick": state["tick"], "peak_N": 0., "cats": list(act["cats"])})
                    h["peak_N"] = max(h["peak_N"], monitor.step_pairs.get(key, 0.))
    for n, r in robots.items():
        z = float(walls[n].data.root_pos_w[0, 2])
        arm_hit = sorted(l for l, h in hits[n].items() if "arm" in h["cats"])
        grip_hit = sorted(l for l, h in hits[n].items() if any(c in ("pad", "finger", "palm") for c in h["cats"]))
        caught = z > PLATE["caught_above_m"]
        press[n].update(hits={l: {**h, "peak_N": round(h["peak_N"], 2)} for l, h in hits[n].items()},
                        arm_links_touching=arm_hit, gripper_links_touching=grip_hit, plate_rest_z_m=round(z, 4),
                        caught_by_arm=bool(caught),
                        verdict="contact" if (arm_hit and caught) else "no_contact" if not arm_hit and not caught else "ambiguous")
        jl("press", robot=n, **{k: v for k, v in press[n].items() if k != "link_radii"})
        p0 = np.asarray(B_OFFSET if n == "B" else (0., 0., 0.))
        frames[f"press_{n}"] = shot(f"press_{n}", p0 + np.array([1.4, -1.4, 1.2]), p0 + np.array([.15, .1, .45]))
    press["A"]["config"] = "gripper de-instanced, arm colliders instanced (the episode before D12)"
    press["B"]["config"] = "whole robot de-instanced (D12 candidate)"
    res["press"] = press
    res["contact_report_counts"] = dict(monitor.counts)
    res["frames"] = frames


if __name__ == "__main__":
    main()
