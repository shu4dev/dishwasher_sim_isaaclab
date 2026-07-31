# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase C: build, verify, and visually document the static v0 scene.

Two modes:

``--measure`` (bootstrap, run once)
    Scene without the carried object. Measures the live ``wrist_3_link -> TCP`` transform and
    prints the constant to freeze into ``config.T_WRIST3_TCP_QUAT``, plus the FK cross-check.
    Then sweeps ``finger_joint`` and prints the pad map (jaw separation, pad height, and the
    closing axis in the TCP frame per aperture) — the geometric input to the grasp calibration
    (``scripts/11_calibrate_grasp.py``). Saved to ``media/C/pad_map.json``.

default (requires the frozen TCP + grasp-calibration constants)
    Scene with the object welded to the wrist between the open jaws. Settles open, gates on
    zero contact, then visibly closes the gripper onto the mug (``grasp_close_open.mp4``),
    verifies the calibrated pad-force band and the weld against the analytic grasp chain, logs
    every pose to ``media/C/scene_poses.json``, captures stills + pads-on-mug close-ups, runs
    the reach/workspace check, records a 10 s wrist-wiggle clip proving rigidity while
    monitoring the dynamic pad-force peak (the ``GRIP_FORCE_EXEC_MAX_N`` measurement), and
    finally opens the jaws on camera, verifying a clean release of the contact forces.

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
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()

    # -- settle (jaws fully open; the visible close happens later, on camera) -----------------
    for i in range(args_cli.settle_steps):
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
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

    # gripper fully open after the settle
    fid, _ = robot.find_joints(config.GRIPPER_JOINT)
    aperture = float(robot.data.joint_pos.torch[0, fid[0]])
    check("gripper open after settle", abs(aperture - config.GRIPPER_APERTURE_OPEN_RAD) < 0.05,
          f"{aperture:.3f} rad")

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

        # -- pad-geometry probe: finger_joint -> jaw separation / pad height map --------------
        # The ee_frame pad targets ride the left/right_inner_finger BODY frames, so they track
        # the aperture live — a free, physics-backed jaw readout. Pad-frame separation at open
        # is 118.4 mm vs the nominal 85 mm jaw opening: the difference (2 x ~16.7 mm) is the
        # pad body on each side, so jaw_width(theta) ~ pad_sep(theta) - PAD_BODY_2X below.
        probe = []
        for theta in (0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.6, 0.78):
            for _ in range(60):
                dscene.hold_targets(scene, aperture=theta)
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
            theta_meas = float(robot.data.joint_pos.torch[0, fid[0]])
            tcp_p = scene["ee_frame"].data.target_pos_w.torch[0, 0].cpu().numpy()
            tcp_q = scene["ee_frame"].data.target_quat_w.torch[0, 0].cpu().numpy()
            T_tcp_w = T_inv(make_T(tcp_p, tcp_q))
            pads_w = scene["ee_frame"].data.target_pos_w.torch[0, 1:3].cpu().numpy()
            pads_tcp = [(T_tcp_w @ np.append(p, 1.0))[:3] for p in pads_w]
            sep_vec = pads_tcp[0] - pads_tcp[1]
            sep = float(np.linalg.norm(sep_vec))
            mid = 0.5 * (pads_tcp[0] + pads_tcp[1])
            probe.append({
                "theta_cmd": theta,
                "theta_meas": round(theta_meas, 4),
                "pad_sep_m": round(sep, 5),
                "pad_mid_tcp": [round(float(v), 5) for v in mid],
                "close_axis_tcp": [round(float(v), 4) for v in sep_vec / max(sep, 1e-9)],
                "pad_L_tcp": [round(float(v), 5) for v in pads_tcp[0]],
                "pad_R_tcp": [round(float(v), 5) for v in pads_tcp[1]],
            })
            print(f"[INFO] pad map theta={theta:.3f}: meas={theta_meas:.4f} rad, "
                  f"sep={sep*1e3:.1f} mm, mid_z={mid[2]*1e3:+.1f} mm, "
                  f"close_axis={probe[-1]['close_axis_tcp']}")

        # derived: pad-body allowance from the open state (nominal 85 mm jaw at theta=0), the
        # aperture where the jaw meets the mug diameter, and the candidate rim height
        pad_body_2x = probe[0]["pad_sep_m"] - 0.085
        target_sep = 2.0 * config.OBJECT_RIM_RADIUS_M + pad_body_2x
        theta_pred, rim_z_cand = None, None
        for a, b in zip(probe[:-1], probe[1:]):
            if b["pad_sep_m"] <= target_sep <= a["pad_sep_m"]:
                f = (a["pad_sep_m"] - target_sep) / max(a["pad_sep_m"] - b["pad_sep_m"], 1e-9)
                theta_pred = a["theta_meas"] + f * (b["theta_meas"] - a["theta_meas"])
                mid_z = a["pad_mid_tcp"][2] + f * (b["pad_mid_tcp"][2] - a["pad_mid_tcp"][2])
                rim_z_cand = mid_z - config.PAD_ENGAGEMENT_M
                break
        check("jaw closes through the mug diameter", theta_pred is not None,
              f"target pad sep {target_sep*1e3:.1f} mm")
        if theta_pred is not None:
            print(f"[INFO] pad-body allowance 2x = {pad_body_2x*1e3:.1f} mm")
            print(f"[INFO] jaw meets the {2*config.OBJECT_RIM_RADIUS_M*1e3:.1f} mm mug at "
                  f"theta ~ {theta_pred:.3f} rad (pad sep {target_sep*1e3:.1f} mm)")
            print("[INFO] calibration starting point (scripts/11_calibrate_grasp.py):")
            print(f"       --rim_z {rim_z_cand:.4f}   # pad_mid_z(theta_pred) - PAD_ENGAGEMENT_M")
        os.makedirs(args_cli.out_dir, exist_ok=True)
        pad_map_path = os.path.join(args_cli.out_dir, "pad_map.json")
        with open(pad_map_path, "w") as f:
            json.dump({"probe": probe, "pad_body_2x_m": round(pad_body_2x, 5),
                       "theta_pred": None if theta_pred is None else round(theta_pred, 4),
                       "rim_z_candidate_m": None if rim_z_cand is None else round(rim_z_cand, 4)},
                      f, indent=2)
        print(f"[INFO] pad map written to {pad_map_path}")

        # leave the jaws open again (the settled template other scripts expect at boot)
        for _ in range(60):
            dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        if rig is not None:
            paths = rig.save_stills(args_cli.out_dir, "measure")
            print(f"[INFO] stills: {paths}")
    else:
        obj = scene["carried_object"]

        # -- open-state gate: the welded mug sits untouched between the fully-open jaws ------
        f_open = dscene.grip_forces(scene)
        touching = {k: round(v, 3) for k, v in f_open["partners"].items() if v > 0.005}
        check("no contact on the welded object at full open",
              max(f_open["partners"].values()) < 0.05
              and f_open["external_n"] < config.CONTACT_FORCE_THRESH_N,
              f"partners {touching}, external {f_open['external_n']:.3f} N")

        # -- visible close onto the mug (the on-camera grasp) --------------------------------
        close_writer = None
        if rig is not None:
            aim = obj.data.root_pos_w.torch[0].cpu().numpy()
            rig.set_view("iso", aim + np.array([-0.28, 0.28, 0.12]), aim, sim.device)
            close_writer = VideoWriter(
                os.path.join(args_cli.out_dir, "grasp_close_open.mp4"), fps=config.CAMERA_FPS
            )

        def ramp_capture(i):
            if close_writer is not None and i % 2 == 0:
                rig.update(dt)
                close_writer.add(rig.grab()["iso"])

        dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_GRASP_RAD,
                            config.GRIPPER_CLOSE_RAMP_STEPS, per_step=ramp_capture)
        for _ in range(60):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        aperture = float(robot.data.joint_pos.torch[0, fid[0]])
        # tolerance is wider than the open check: the settle is contact-limited (the pads stop
        # on the mug wall, slightly short of the over-commanded target)
        check("gripper holds grasp aperture under contact",
              abs(aperture - config.GRIPPER_APERTURE_GRASP_RAD) < 0.08, f"{aperture:.3f} rad")
        ok, detail = dscene.grip_gate(scene)
        check("grip force band after close", ok, detail or "in band")
        f_hold = dscene.grip_forces(scene)
        print(f"[INFO] steady grip: pads {[round(v, 2) for v in f_hold['pads_n']]} N, "
              f"external {f_hold['external_n']:.3f} N")
        if rig is not None:
            rig.set_view("iso", *config.CAMERAS["iso"], sim.device)
            for _ in range(3):
                sim.step()
                scene.update(dt)
                rig.update(dt)

        # -- weld verification -------------------------------------------------------------
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
        f_diag = dscene.grip_forces(scene)
        # the net force on the mug is nonzero BY DESIGN now (pinch asymmetry); what must stay
        # silent is everything that isn't the two pads
        check("only the pads touch the carried object",
              all(v < config.CONTACT_FORCE_THRESH_N for k, v in f_diag["partners"].items()
                  if k not in config.GRIP_PAD_BODIES)
              and f_diag["external_n"] < config.CONTACT_FORCE_THRESH_N,
              f"net {f_diag['net_n']:.2f} N (pad asymmetry), external {f_diag['external_n']:.3f} N")
        print("[INFO] contact partners (N):",
              {n: round(v, 2) for n, v in f_diag["partners"].items() if v > 0.01})
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

        # -- media: rigidity wiggle clip + dynamic pad-force measurement ---------------------
        # (stills already saved above; the wiggle also runs headless so the exec-force
        # envelope is measured either way)
        w3_obj_target = dscene.t_wrist3_obj()
        home = np.array(config.HOME_Q)
        writers = {}
        if rig is not None:
            writers = {
                name: VideoWriter(os.path.join(args_cli.out_dir, f"rigidity_{name}.mp4"), fps=config.CAMERA_FPS)
                for name in ("iso", "front")
            }
        n_steps = int(args_cli.wiggle_seconds / config.SIM_DT)
        max_drift_mm, max_rot_drift, pad_peak_wiggle = 0.0, 0.0, 0.0
        wiggle_trace = []
        for step in range(n_steps):
            t = step * config.SIM_DT
            q = home.copy()
            q[3] += 0.25 * math.sin(2.0 * math.pi * 0.4 * t)  # wrist_1_joint
            q[4] += 0.25 * (math.cos(2.0 * math.pi * 0.4 * t) - 1.0)  # wrist_2_joint
            dscene.hold_targets(scene, arm_q=q)  # squeeze stays commanded during motion
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            pads = dscene.grip_forces(scene)["pads_n"]
            pad_peak_wiggle = max(pad_peak_wiggle, max(pads))
            if step % 2 == 0:
                wiggle_trace.append([round(t, 3)] + [round(v, 3) for v in pads])
                if rig is not None:
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
        check("dynamic pad force within exec allowance",
              pad_peak_wiggle <= config.GRIP_FORCE_EXEC_MAX_N,
              f"peak {pad_peak_wiggle:.2f} N (allowance {config.GRIP_FORCE_EXEC_MAX_N:.1f} N)")
        print(f"[INFO] wiggle pad-force peak {pad_peak_wiggle:.2f} N — freeze "
              f"GRIP_FORCE_EXEC_MAX_N with margin above this")
        with open(os.path.join(args_cli.out_dir, "pad_wiggle_forces.json"), "w") as f:
            json.dump({"t_padL_padR": wiggle_trace, "peak_n": round(pad_peak_wiggle, 3)}, f)

        statics_after = dscene.statics_report(scene)
        check("statics unchanged after wiggle",
              abs(statics_after["door_deg"] - statics["door_deg"]) < 0.3
              and statics_after["rack_lower_err_m"] < 2e-3,
              f"door {statics_after['door_deg']:.2f} deg, rack err {statics_after['rack_lower_err_m']*1e3:.2f} mm")

        # -- visible open: jaws release the (still-welded) mug, forces drop to zero ----------
        if rig is not None:
            aim = obj.data.root_pos_w.torch[0].cpu().numpy()
            rig.set_view("iso", aim + np.array([-0.28, 0.28, 0.12]), aim, sim.device)
        dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_OPEN_RAD,
                            config.GRIPPER_OPEN_RAMP_STEPS, per_step=ramp_capture)
        for i in range(30):
            dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            ramp_capture(i)
        f_end = dscene.grip_forces(scene)
        check("pads release cleanly on open", max(f_end["partners"].values()) < 0.05,
              f"max partner force {max(f_end['partners'].values()):.3f} N")
        if close_writer is not None:
            close_writer.close()
            print(f"[INFO] clip: {close_writer.path} ({close_writer.frames} frames)")
        if rig is None:
            print("[WARN] cameras disabled — no media produced (run with --enable_cameras)")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
