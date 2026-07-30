# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase C: build, verify, and visually document the static v0 scene.

Two modes:

``--measure`` (bootstrap, run once)
    Scene without the carried object. Measures the live ``wrist_3_link -> TCP`` transform and
    prints the constant to freeze into ``config.T_WRIST3_TCP_QUAT``, plus the FK cross-check.

default (requires the frozen TCP constant)
    Scene with the object welded to the wrist. Settles, asserts the static locks (door open,
    racks pinned), verifies the weld against the analytic grasp chain, logs every pose (world +
    robot-base frames) to ``media/C/scene_poses.json``, captures front/top/iso stills and a
    10 s wrist-wiggle clip proving the object tracks the TCP rigidly, and runs the
    reach/workspace check over the extended lower rack.

Run with:
    scripts/run_kit.sh scripts/10_v0_scene.py --headless --enable_cameras [--measure]
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="Static v0 scene: verify + document.")
parser.add_argument("--measure", action="store_true", help="Bootstrap: measure the TCP constant, no object.")
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "media", "C"))
parser.add_argument("--settle_steps", type=int, default=200)
parser.add_argument("--wiggle_seconds", type=float, default=10.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import math
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.transforms import T_inv, make_T, rot_angle_deg  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3  # noqa: E402

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def body_pose_np(articulation, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    ids, _ = articulation.find_bodies(body_name)
    pos = articulation.data.body_link_pos_w.torch[0, ids[0]].cpu().numpy()
    quat = articulation.data.body_link_quat_w.torch[0, ids[0]].cpu().numpy()
    return pos, quat


def rack_interior_grid(n: int = 6) -> np.ndarray:
    """World-frame sample grid just above the extended lower rack (from the v0 USD geometry)."""
    from pxr import Usd, UsdGeom  # noqa: PLC0415

    from dishsim.robots import DISHWASHER_V0_USD_PATH  # noqa: PLC0415

    stage = Usd.Stage.Open(DISHWASHER_V0_USD_PATH)
    rack_prim = None
    for prim in stage.Traverse():
        if prim.GetName() == "E_shelf_1_04":
            rack_prim = prim
            break
    assert rack_prim is not None, "lower rack body E_shelf_1_04 not found in the v0 USD"
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    rng = cache.ComputeWorldBound(rack_prim).ComputeAlignedRange()  # asset frame, racks stowed
    mn, mx = np.array(rng.GetMin()), np.array(rng.GetMax())
    T_asset_body = np.array(
        UsdGeom.Xformable(rack_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float
    ).T  # Gf matrices are row-major transposes of the usual convention
    # grid over the rack's top plane, in the rack BODY frame
    xs = np.linspace(mn[0] + 0.03, mx[0] - 0.03, n)
    ys = np.linspace(mn[1] + 0.03, mx[1] - 0.03, n)
    top_z = mx[2] + 0.02
    pts_asset = np.array([[x, y, top_z, 1.0] for x in xs for y in ys])
    pts_body = (T_inv(T_asset_body) @ pts_asset.T).T
    return pts_body  # homogeneous, body frame


def main() -> None:
    if not args_cli.measure and config.T_WRIST3_TCP_QUAT is None:
        raise SystemExit(
            "[FAIL] config.T_WRIST3_TCP_QUAT is unset — run this script with --measure first "
            "and freeze the printed constant into src/dishsim/config.py."
        )

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg())
    )
    sim.set_camera_view([1.8, -1.8, 1.2], [0.4, 0.0, 0.4])

    scene_cfg = dscene.make_scene_cfg(with_object=not args_cli.measure)
    scene = InteractiveScene(scene_cfg)

    weld_path = None
    if not args_cli.measure:
        weld_path = dscene.author_weld(scene.stage)
        print(f"[INFO] Weld authored at {weld_path}")

    rig = CameraRig(config.CAMERAS, hw=config.CAMERA_HW) if args_cli.enable_cameras else None

    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()

    # -- settle ------------------------------------------------------------------------------
    for i in range(args_cli.settle_steps):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        if rig is not None and i >= args_cli.settle_steps - 10:
            rig.update(dt)  # warm-up renders

    dscene.assert_frames(scene)
    robot = scene["robot"]

    statics = dscene.statics_report(scene)
    print(f"[INFO] statics after settle: { {k: round(v, 5) for k, v in statics.items()} }")
    check("door locked open", 88.3 <= statics["door_deg"] <= 90.05, f"{statics['door_deg']:.2f} deg")
    check("lower rack extended", statics["rack_lower_err_m"] < 2e-3, f"err {statics['rack_lower_err_m']*1e3:.2f} mm")
    check("upper rack stowed", statics["rack_upper_err_m"] < 2e-3, f"err {statics['rack_upper_err_m']*1e3:.2f} mm")

    vels = robot.data.joint_vel.torch[0]
    check("robot settled, no NaN", bool(torch.isfinite(vels).all()) and float(vels.abs().max()) < 0.5)

    # gripper aperture held
    fid, _ = robot.find_joints(config.GRIPPER_JOINT)
    aperture = float(robot.data.joint_pos.torch[0, fid[0]])
    check("gripper frozen at aperture", abs(aperture - config.GRIPPER_APERTURE_RAD) < 0.05, f"{aperture:.3f} rad")

    # -- FK cross-check ----------------------------------------------------------------------
    w3_pos, w3_quat = body_pose_np(robot, "wrist_3_link")
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_pred_w3 = T_w_base @ fk_wrist3(np.array(config.HOME_Q))
    fk_err_mm = float(np.linalg.norm(T_pred_w3[:3, 3] - w3_pos)) * 1e3
    fk_rot_err = rot_angle_deg(T_pred_w3, make_T(w3_pos, w3_quat))
    check("analytic FK matches live wrist_3", fk_err_mm < 1.0 and fk_rot_err < 0.5,
          f"pos err {fk_err_mm:.3f} mm, rot err {fk_rot_err:.3f} deg")

    if args_cli.measure:
        m = dscene.measure_tcp(scene)
        print("[INFO] measured wrist_3 -> TCP transform:")
        print(f"       pos  = {tuple(round(v, 6) for v in m['t_wrist3_tcp_pos'])}")
        print(f"       quat = {tuple(round(v, 8) for v in m['t_wrist3_tcp_quat_xyzw'])}  (XYZW)")
        print("[INFO] freeze into src/dishsim/config.py:")
        print(f"       T_WRIST3_TCP_POS = {tuple(round(v, 6) for v in m['t_wrist3_tcp_pos'])}")
        print(f"       T_WRIST3_TCP_QUAT = {tuple(round(v, 8) for v in m['t_wrist3_tcp_quat_xyzw'])}")
        pos_dev = np.linalg.norm(np.array(m["t_wrist3_tcp_pos"]) - np.array(config.T_WRIST3_TCP_POS))
        check("TCP translation near documented (0, 0.130, 0)", pos_dev < 5e-3, f"dev {pos_dev*1e3:.2f} mm")
        if rig is not None:
            paths = rig.save_stills(args_cli.out_dir, "measure")
            print(f"[INFO] stills: {paths}")
    else:
        # -- weld verification -------------------------------------------------------------
        obj = scene["carried_object"]
        obj_pos = obj.data.root_pos_w.torch[0].cpu().numpy()
        obj_quat = obj.data.root_quat_w.torch[0].cpu().numpy()
        pred_pos, pred_quat = dscene.grasp_pose_w()
        weld_pos_err_mm = float(np.linalg.norm(obj_pos - pred_pos)) * 1e3
        weld_rot_err = rot_angle_deg(make_T(pred_pos, pred_quat), make_T(obj_pos, obj_quat))
        check("weld holds object at analytic grasp pose", weld_pos_err_mm < 3.0 and weld_rot_err < 1.5,
              f"pos err {weld_pos_err_mm:.2f} mm, rot err {weld_rot_err:.2f} deg")

        # -- diagnostics: finger state + net contact force on the welded object --------------
        finger_names = [n for n in robot.joint_names if "finger" in n or "knuckle" in n]
        fids, fnames = robot.find_joints(finger_names, preserve_order=True)
        fpos = robot.data.joint_pos.torch[0, fids].cpu().numpy()
        print("[INFO] finger joints:", {n: round(float(v), 3) for n, v in zip(fnames, fpos)})
        print("[INFO] robot bodies:", robot.body_names)
        sensor = scene["object_contact"]
        contact_n = float(sensor.data.net_forces_w.torch[0].norm())
        check("no persistent contact on welded object", contact_n < config.CONTACT_FORCE_THRESH_N,
              f"net force {contact_n:.3f} N")
        if sensor.data.force_matrix_w is not None:
            fm = sensor.data.force_matrix_w.torch[0].reshape(-1, 3)
            mags = fm.norm(dim=-1).cpu().numpy()
            filters = [p.rsplit("/", 1)[-1] for p in sensor.cfg.filter_prim_paths_expr]
            print("[INFO] contact partners (N):",
                  {n: round(float(m), 2) for n, m in zip(filters, mags) if m > 0.01})
        # gripper-body positions in the TCP frame (locates interpenetration numerically)
        tcp_pos_d = scene["ee_frame"].data.target_pos_w.torch[0, 0].cpu().numpy()
        tcp_quat_d = scene["ee_frame"].data.target_quat_w.torch[0, 0].cpu().numpy()
        T_tcp_w = T_inv(make_T(tcp_pos_d, tcp_quat_d))
        for bname in robot.body_names:
            if "finger" in bname or "knuckle" in bname or bname == "base_link_0":
                bpos, _ = body_pose_np(robot, bname)
                p = (T_tcp_w @ np.append(bpos, 1.0))[:3]
                print(f"[INFO]   {bname} in TCP frame: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
        obj_in_tcp = (T_tcp_w @ np.append(obj_pos, 1.0))[:3]
        print(f"[INFO]   carried_object origin in TCP frame: {np.round(obj_in_tcp, 4)} "
              f"(analytic: {config.GRASP_TCP_OBJ_POS})")
        if rig is not None:
            from PIL import Image

            # scene stills FIRST (before any camera re-aiming can leave stale frames behind)
            stills = rig.save_stills(args_cli.out_dir, "scene")
            print(f"[INFO] stills: {stills}")
            for tag, offset in (("closeupA", (-0.28, 0.28, 0.12)), ("closeupB", (0.30, -0.22, 0.05))):
                rig.set_view("iso", obj_pos + np.array(offset), obj_pos, sim.device)
                for _ in range(3):
                    sim.step()
                    scene.update(dt)
                    rig.update(dt)
                Image.fromarray(rig.grab()["iso"]).save(os.path.join(args_cli.out_dir, f"grasp_{tag}.png"))
                print(f"[INFO] closeup: {os.path.join(args_cli.out_dir, f'grasp_{tag}.png')}")
            rig.set_view("iso", *config.CAMERAS["iso"], sim.device)
            for _ in range(3):
                sim.step()
                scene.update(dt)
                rig.update(dt)

        # -- workspace check over the extended lower rack ------------------------------------
        pts_body = rack_interior_grid()
        rack_pos, rack_quat = body_pose_np(scene["dishwasher"], "E_shelf_1_04")
        T_w_rack = make_T(rack_pos, rack_quat)
        pts_w = (T_w_rack @ pts_body.T).T[:, :3]
        shoulder_w = np.array(config.ROBOT_BASE_POS_W) + np.array([0.0, 0.0, 0.1625])
        dists = np.linalg.norm(pts_w - shoulder_w, axis=1)
        frac = float((dists < config.UR5E_REACH_M).mean())
        print(
            f"[INFO] workspace: rack-top grid x [{pts_w[:,0].min():.3f}, {pts_w[:,0].max():.3f}] m, "
            f"z ~ {pts_w[:,2].mean():.3f} m; shoulder distances [{dists.min():.3f}, {dists.max():.3f}] m; "
            f"{frac*100:.0f}% within {config.UR5E_REACH_M} m reach"
        )
        check("some of the rack is reachable", frac > 0.3, f"{frac*100:.0f}% reachable")

        # -- pose log -------------------------------------------------------------------------
        poses = {}

        def log_pose(name, pos, quat):
            poses[name] = {
                "world": {"pos": [round(float(v), 5) for v in pos], "quat_xyzw": [round(float(v), 6) for v in quat]},
                "robot_base": {"pos": [round(float(v), 5) for v in dscene.world_to_base(pos)]},
            }

        log_pose("robot_base", robot.data.root_pos_w.torch[0].cpu().numpy(),
                 robot.data.root_quat_w.torch[0].cpu().numpy())
        log_pose("wrist_3_link", w3_pos, w3_quat)
        tcp_pos = scene["ee_frame"].data.target_pos_w.torch[0, 0].cpu().numpy()
        tcp_quat = scene["ee_frame"].data.target_quat_w.torch[0, 0].cpu().numpy()
        log_pose("tcp", tcp_pos, tcp_quat)
        dw = scene["dishwasher"]
        for body in ("E_body_5", "E_door_4", "E_shelf_1_04", "E_shelf_03"):
            log_pose(body, *body_pose_np(dw, body))
        log_pose("carried_object", obj_pos, obj_quat)
        os.makedirs(args_cli.out_dir, exist_ok=True)
        poses_path = os.path.join(args_cli.out_dir, "scene_poses.json")
        with open(poses_path, "w") as f:
            json.dump({"frame_convention": config.FRAME_CONVENTION, "statics": statics, "poses": poses}, f, indent=2)
        print(f"[INFO] poses written to {poses_path}")

        # -- media: rigidity wiggle clip (stills already saved above) ------------------------
        if rig is not None:
            w3_obj_target = dscene.t_wrist3_obj()
            wiggle_ids, _ = robot.find_joints(["wrist_1_joint", "wrist_2_joint"])
            base_targets = robot.data.default_joint_pos.torch.clone()
            fid_all, _ = robot.find_joints(config.GRIPPER_JOINT)
            base_targets[:, fid_all[0]] = config.GRIPPER_APERTURE_RAD
            home_w1 = float(base_targets[0, wiggle_ids[0]])
            home_w2 = float(base_targets[0, wiggle_ids[1]])

            writers = {
                name: VideoWriter(os.path.join(args_cli.out_dir, f"rigidity_{name}.mp4"), fps=config.CAMERA_FPS)
                for name in ("iso", "front")
            }
            n_steps = int(args_cli.wiggle_seconds / config.SIM_DT)
            max_drift_mm, max_rot_drift = 0.0, 0.0
            for step in range(n_steps):
                t = step * config.SIM_DT
                targets = base_targets.clone()
                targets[:, wiggle_ids[0]] = home_w1 + 0.25 * math.sin(2.0 * math.pi * 0.4 * t)
                targets[:, wiggle_ids[1]] = home_w2 + 0.25 * (math.cos(2.0 * math.pi * 0.4 * t) - 1.0)
                robot.set_joint_position_target_index(target=targets)
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
                if step % 2 == 0:
                    rig.update(dt)
                    frames = rig.grab()
                    for name, wr in writers.items():
                        wr.add(frames[name])
                    # weld drift: live wrist->object vs the analytic weld transform
                    w3_p, w3_q = body_pose_np(robot, "wrist_3_link")
                    o_p = obj.data.root_pos_w.torch[0].cpu().numpy()
                    o_q = obj.data.root_quat_w.torch[0].cpu().numpy()
                    T_live = T_inv(make_T(w3_p, w3_q)) @ make_T(o_p, o_q)
                    max_drift_mm = max(
                        max_drift_mm, float(np.linalg.norm(T_live[:3, 3] - w3_obj_target[:3, 3])) * 1e3
                    )
                    max_rot_drift = max(max_rot_drift, rot_angle_deg(T_live, w3_obj_target))
            for wr in writers.values():
                wr.close()
                print(f"[INFO] clip: {wr.path} ({wr.frames} frames)")
            # gates allow the weld's transient elastic compliance under the +-0.25 rad wiggle
            # (permanent drift is separately bounded by the post-settle weld check above)
            check("object rigidly attached during wiggle", max_drift_mm < 2.0 and max_rot_drift < 2.0,
                  f"max drift {max_drift_mm:.2f} mm / {max_rot_drift:.2f} deg")

            statics_after = dscene.statics_report(scene)
            check("statics unchanged after wiggle",
                  abs(statics_after["door_deg"] - statics["door_deg"]) < 0.3
                  and statics_after["rack_lower_err_m"] < 2e-3,
                  f"door {statics_after['door_deg']:.2f} deg, rack err {statics_after['rack_lower_err_m']*1e3:.2f} mm")
        else:
            print("[WARN] cameras disabled — no media produced (run with --enable_cameras)")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
