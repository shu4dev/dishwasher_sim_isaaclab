# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Calibrate the contact pinch grasp on the welded mug.

With the mug welded at a candidate carry pose (rim plane ``--rim_z`` from the pad map printed
by ``scripts/setup/check_scene.py --measure``), this script:

1. settles with the jaws fully open and gates on **zero contact** (the mug must sit untouched
   between the open pads — verifies the ~2.6 mm/side clearance and no rim/knuckle grazing);
2. staircase-ramps ``finger_joint`` (``--dtheta`` per plateau, ``--dwell`` steps each), logging
   per-pad contact forces from the ``object_contact`` sensor until both pads report sustained
   contact (``theta_touch``), then continues ``--overshoot`` beyond it for the force-vs-theta
   curve;
3. measures the mimic signs of the two ``.*_inner_finger_joint`` drives and the
   Newton's-third-law sign convention used by :func:`dishsim.scene.unexpected_robot_contact`;
4. picks the grasp aperture where the steady per-pad force crosses ``--target_force``, holds it
   for ``--hold_steps`` verifying the force band, non-pad silence, and weld integrity;
5. opens on camera and verifies the contact forces drop to zero;
6. prints the constants block to freeze into ``src/dishsim/config.py`` and writes
   ``docs/grasp_calibration.md`` + media (``force_vs_theta.png``, close-ups, MP4) to
   ``media/calibration/``.

Run with:
    scripts/run_kit.sh scripts/setup/calibrate_grasp.py --headless --enable_cameras \
        --rim_z <from the pad map>

Re-run without ``--rim_z`` after freezing the constants to confirm the frozen configuration
passes end-to-end.
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Calibrate the contact pinch grasp.")
parser.add_argument("--object", type=str, default="mug", help="Object class (see config.OBJECTS).")
parser.add_argument("--rim_z", type=float, default=None,
                    help="Candidate GRASP_RIM_TCP_Z_M [m] (default: the frozen config value).")
parser.add_argument("--theta_max", type=float, default=0.35, help="Staircase hard stop [rad].")
parser.add_argument("--dtheta", type=float, default=0.005, help="Staircase step [rad].")
parser.add_argument("--dwell", type=int, default=30, help="Settle steps per plateau.")
parser.add_argument("--overshoot", type=float, default=0.06,
                    help="Continue this far past theta_touch for the force curve [rad].")
parser.add_argument("--target_force", type=float, default=5.0,
                    help="Steady per-pad force to aim the grasp aperture at [N].")
parser.add_argument("--hold_steps", type=int, default=300)
parser.add_argument("--pad_force_abort", type=float, default=30.0,
                    help="Abort the staircase if any pad exceeds this [N].")
parser.add_argument("--verify", action="store_true",
                    help="Verify-only mode: hold at the FROZEN config aperture and gate the "
                         "measured constants against config.py (touch aperture, force band, "
                         "mimic signs) instead of proposing new constants. Writes verify_* "
                         "outputs and leaves docs/grasp_calibration.md untouched.")
parser.add_argument("--out_dir", type=str, default=None,
                    help="Media dir (default: media/calibration for the mug, media/calibration/<object> otherwise).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

config.set_active_object(args_cli.object)
if args_cli.out_dir is None:
    args_cli.out_dir = os.path.join(PROJECT_ROOT, "media", "calibration")
    if args_cli.object != "mug":
        args_cli.out_dir = os.path.join(args_cli.out_dir, args_cli.object)

# candidate-grasp override BEFORE any scene/weld construction (t_wrist3_obj reads config
# lazily, so an in-memory override is enough — no file edits during iteration). The transform
# is rebuilt from the active spec's grasp family via config.grasp_transform — no object
# literals here.
if args_cli.rim_z is not None:
    import dataclasses  # noqa: PLC0415

    spec = config.active_object_spec()
    spec = dataclasses.replace(spec, grasp=dataclasses.replace(spec.grasp, rim_tcp_z_m=args_cli.rim_z))
    config.GRASP_RIM_TCP_Z_M = args_cli.rim_z
    config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT = config.grasp_transform(spec)
elif config.GRASP_TCP_OBJ_POS is None:
    raise SystemExit("[FAIL] no --rim_z given and config.GRASP_RIM_TCP_Z_M is not frozen — "
                     "run scripts/setup/check_scene.py --measure for the candidate value first.")

from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def main() -> None:
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg())
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=True, with_robot_contacts=True))
    weld_path = dscene.author_weld(scene.stage)
    rig = CameraRig(config.CAMERAS, hw=config.CAMERA_HW) if args_cli.enable_cameras else None
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()
    robot = scene["robot"]
    obj = scene["carried_object"]
    fid, _ = robot.find_joints(config.GRIPPER_JOINT)
    if_ids, if_names = robot.find_joints(".*_inner_finger_joint", preserve_order=True)
    gsensor = scene["robot_contacts_gripper"]
    gnames = list(getattr(gsensor, "body_names", []))

    os.makedirs(args_cli.out_dir, exist_ok=True)
    video = None
    if rig is not None:
        aim = obj.data.root_pos_w.torch[0].cpu().numpy()
        rig.set_view("iso", aim + np.array([-0.28, 0.28, 0.12]), aim, sim.device)
        video = VideoWriter(os.path.join(args_cli.out_dir, "close_open.mp4"), fps=config.CAMERA_FPS)

    frame_i = 0

    def capture() -> None:
        nonlocal frame_i
        if video is None:
            return
        frame_i += 1
        if frame_i % 4 == 0:  # staircase is slow — 4x timelapse keeps the clip short
            rig.update(dt)
            video.add(rig.grab()["iso"])

    def settle(aperture: float, steps: int) -> None:
        for _ in range(steps):
            dscene.hold_targets(scene, aperture=aperture)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            capture()

    def weld_err_mm() -> float:
        obj_pos = obj.data.root_pos_w.torch[0].cpu().numpy()
        return float(np.linalg.norm(obj_pos - dscene.grasp_pose_w()[0])) * 1e3

    def finger_state() -> dict:
        theta = float(robot.data.joint_pos.torch[0, fid[0]])
        inner = {n: float(robot.data.joint_pos.torch[0, i]) for i, n in zip(if_ids, if_names)}
        return {"theta": theta, "inner": inner}

    # ---- 1. open-state gate ------------------------------------------------------------------
    settle(config.GRIPPER_APERTURE_OPEN_RAD, 150)
    f0 = dscene.grip_forces(scene)
    touching = {k: round(v, 3) for k, v in f0["partners"].items() if v > 0.005}
    check("zero contact at full open", max(f0["partners"].values()) < 0.05
          and f0["external_n"] < config.CONTACT_FORCE_THRESH_N,
          f"partners {touching}, external {f0['external_n']:.3f} N")
    check("weld pose at open", weld_err_mm() < 3.0, f"{weld_err_mm():.2f} mm")
    if rig is not None:
        rig.update(dt)
        from PIL import Image  # noqa: PLC0415

        Image.fromarray(rig.grab()["iso"]).save(os.path.join(args_cli.out_dir, "open_state.png"))
    if FAILURES:
        print("[FAIL] open-state gate failed — the candidate rim_z puts the mug into the "
              "jaws/knuckles already; adjust --rim_z (more negative = deeper toward the wrist "
              "is WRONG direction — move the mug away, less deep) and re-run.")
        print("[RESULT] FAIL")
        raise SystemExit(1)

    # ---- 2. staircase to touch + force curve -------------------------------------------------
    curve = []
    theta_touch = None
    theta = 0.0
    print(f"[INFO] staircase: dtheta={args_cli.dtheta}, dwell={args_cli.dwell}, "
          f"touch threshold 0.2 N on both pads")
    while theta < args_cli.theta_max:
        theta = round(theta + args_cli.dtheta, 6)
        # dwell, averaging the pad forces over the last third of the plateau
        tail_forces = []
        for i in range(args_cli.dwell):
            dscene.hold_targets(scene, aperture=theta)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            capture()
            if i >= args_cli.dwell - max(5, args_cli.dwell // 3):
                tail_forces.append(dscene.grip_forces(scene))
        pads = np.mean([f["pads_n"] for f in tail_forces], axis=0)
        nonpad = max(
            max((v for k, v in f["partners"].items() if k not in config.GRIP_PAD_BODIES), default=0.0)
            for f in tail_forces
        )
        st = finger_state()
        curve.append({
            "theta_cmd": theta,
            "theta_meas": round(st["theta"], 4),
            "pad_forces_n": [round(float(v), 3) for v in pads],
            "nonpad_max_n": round(nonpad, 3),
            "external_n": round(tail_forces[-1]["external_n"], 3),
            "inner_pos": {k: round(v, 4) for k, v in st["inner"].items()},
        })
        if float(max(pads)) > args_cli.pad_force_abort:
            check("staircase force abort", False,
                  f"pad force {max(pads):.1f} N > {args_cli.pad_force_abort} N at theta={theta:.3f}")
            break
        if theta_touch is None and min(pads) >= 0.2:
            theta_touch = theta
            print(f"[INFO] touch at theta={theta:.3f} (meas {st['theta']:.4f}), "
                  f"pads {np.round(pads, 2)} N")
            if rig is not None:
                rig.update(dt)
                from PIL import Image  # noqa: PLC0415

                Image.fromarray(rig.grab()["iso"]).save(
                    os.path.join(args_cli.out_dir, "touch_state.png"))
        if theta_touch is not None and theta >= theta_touch + args_cli.overshoot:
            break

    check("pads reached the mug", theta_touch is not None,
          f"no sustained contact by theta={args_cli.theta_max}")
    if theta_touch is None or FAILURES:
        print("[RESULT] FAIL")
        raise SystemExit(1)

    # ---- 3. mimic signs (from the last pre-touch plateau: pure mimic, no contact loads) ------
    pre_touch = [c for c in curve if c["theta_cmd"] < theta_touch]
    ref = pre_touch[-1] if pre_touch else curve[0]
    signs = {}
    for name, val in ref["inner_pos"].items():
        ratio = val / max(ref["theta_meas"], 1e-6)
        signs[name] = 1.0 if ratio >= 0 else -1.0
        check(f"mimic ratio ~ +-1 for {name}", 0.5 < abs(ratio) < 1.5,
              f"ratio {ratio:+.3f} at theta={ref['theta_meas']:.3f}")
    print(f"[INFO] measured inner-finger signs: {signs}")

    # ---- 4. grasp aperture: pick from the curve, or verify the frozen one --------------------
    post = [c for c in curve if c["theta_cmd"] >= theta_touch]
    theta_grasp, steady = None, None
    if args_cli.verify:
        theta_grasp = float(config.GRIPPER_APERTURE_GRASP_RAD)
        # the frozen aperture is calibrated as touch + a small over-command; if touch drifted,
        # the frozen value would either not engage or over-squeeze
        check("frozen aperture just past measured touch", 0.0 < theta_grasp - theta_touch <= 0.020,
              f"theta_touch {theta_touch:.3f} vs frozen aperture {theta_grasp:.3f}")
    else:
        for a, b in zip(post[:-1], post[1:]):
            fa, fb = np.mean(a["pad_forces_n"]), np.mean(b["pad_forces_n"])
            if fa <= args_cli.target_force <= fb:
                frac = (args_cli.target_force - fa) / max(fb - fa, 1e-9)
                theta_grasp = round(a["theta_cmd"] + frac * (b["theta_cmd"] - a["theta_cmd"]), 3)
                break
        if theta_grasp is None:
            # curve topped out below target (effort-limited) — take the strongest plateau
            theta_grasp = post[-1]["theta_cmd"]
            print(f"[WARN] force curve peaked at {np.mean(post[-1]['pad_forces_n']):.2f} N "
                  f"< target {args_cli.target_force} N — using theta={theta_grasp:.3f}")

    # ---- 5. hold verification at theta_grasp -------------------------------------------------
    settle(theta_grasp, args_cli.dwell)
    hold_pads = []
    for i in range(args_cli.hold_steps):
        dscene.hold_targets(scene, aperture=theta_grasp)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        capture()
        hold_pads.append(dscene.grip_forces(scene)["pads_n"])
    hold_pads = np.array(hold_pads)
    steady = float(np.mean(hold_pads[-100:]))
    f_hold = dscene.grip_forces(scene)
    band_lo = round(max(0.3, steady / 3.0), 1)
    band_hi = round(max(3.0 * steady, steady + 3.0), 1)
    nonpad_hold = {k: round(v, 3) for k, v in f_hold["partners"].items()
                   if k not in config.GRIP_PAD_BODIES and v > 0.01}
    check("steady pad force sane", 0.5 <= steady <= args_cli.pad_force_abort, f"{steady:.2f} N")
    check("pads share the load", float(np.mean(hold_pads[-100:], axis=0).min()) > 0.2 * steady,
          f"per-pad steady {np.round(np.mean(hold_pads[-100:], axis=0), 2)} N")
    check("non-pad partners silent at hold", not nonpad_hold, f"{nonpad_hold}")
    check("external residual silent at hold",
          f_hold["external_n"] < config.CONTACT_FORCE_THRESH_N, f"{f_hold['external_n']:.3f} N")
    check("weld pose at hold", weld_err_mm() < 3.0, f"{weld_err_mm():.2f} mm")
    st = finger_state()
    check("aperture settles near command", abs(st["theta"] - theta_grasp) < 0.08,
          f"meas {st['theta']:.4f} vs cmd {theta_grasp:.3f}")
    if args_cli.verify:
        # gate the measured behavior against the FROZEN constants (config untouched on PASS)
        check("steady pad force within the frozen band",
              config.GRIP_FORCE_MIN_N <= steady <= config.GRIP_FORCE_MAX_N,
              f"{steady:.2f} N vs [{config.GRIP_FORCE_MIN_N}, {config.GRIP_FORCE_MAX_N}] N")
        frozen_signs = config.GRIPPER_INNER_FINGER_SIGNS or {}
        check("mimic signs match frozen config", signs == frozen_signs,
              f"measured {signs} vs frozen {frozen_signs}")
    for name, val in st["inner"].items():
        pred = signs[name] * st["theta"]
        print(f"[INFO] inner-finger {name}: measured {val:+.4f}, mimic-consistent {pred:+.4f}")

    # Newton's-third-law sign convention for unexpected_robot_contact: the pad body's own
    # net contact force should cancel against MINUS the force_matrix_w row (mug->pad reaction)
    names_row = dscene.object_contact_partners(scene)
    fm = scene["object_contact"].data.force_matrix_w.torch[0].reshape(-1, 3)
    sign_ok = True
    for pad in config.GRIP_PAD_BODIES:
        row = fm[names_row.index(pad)]
        net = gsensor.data.net_forces_w.torch[0][gnames.index(pad)]
        res_minus = float((net + row).norm())  # convention GRIP_REACTION_SIGN = -1
        res_plus = float((net - row).norm())  # wrong convention doubles instead of cancels
        print(f"[INFO] reaction-sign check {pad}: |net+row|={res_minus:.3f} N, "
              f"|net-row|={res_plus:.3f} N (row {float(row.norm()):.2f} N)")
        if res_minus > 0.5 * float(row.norm()) and float(row.norm()) > 0.5:
            sign_ok = False
    check("GRIP_REACTION_SIGN=-1 convention confirmed", sign_ok)

    if rig is not None:
        from PIL import Image  # noqa: PLC0415

        for tag, offset in (("A", (-0.28, 0.28, 0.12)), ("B", (0.30, -0.22, 0.05))):
            aim = obj.data.root_pos_w.torch[0].cpu().numpy()
            rig.set_view("iso", aim + np.array(offset), aim, sim.device)
            for _ in range(3):
                sim.step()
                scene.update(dt)
                rig.update(dt)
            Image.fromarray(rig.grab()["iso"]).save(
                os.path.join(args_cli.out_dir, f"pads_on_mug_closeup{tag}.png"))
        aim = obj.data.root_pos_w.torch[0].cpu().numpy()
        rig.set_view("iso", aim + np.array([-0.28, 0.28, 0.12]), aim, sim.device)

    # ---- 6. open on camera, forces must vanish -----------------------------------------------
    dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_OPEN_RAD,
                        config.GRIPPER_OPEN_RAMP_STEPS,
                        per_step=lambda i: capture())
    settle(config.GRIPPER_APERTURE_OPEN_RAD, 30)
    f_end = dscene.grip_forces(scene)
    check("forces vanish on open", max(f_end["partners"].values()) < 0.05,
          f"max partner {max(f_end['partners'].values()):.3f} N")
    if video is not None:
        video.close()
        print(f"[INFO] clip: {video.path} ({video.frames} frames)")

    # ---- 7. outputs --------------------------------------------------------------------------
    rim_z = config.GRASP_RIM_TCP_Z_M
    prefix = "verify_" if args_cli.verify else ""
    calib = {
        "mode": "verify" if args_cli.verify else "calibrate",
        "rim_z_m": rim_z,
        "theta_touch": theta_touch,
        "theta_grasp": theta_grasp,
        "steady_pad_force_n": round(steady, 2),
        "band_n": [band_lo, band_hi],
        "inner_finger_signs": signs,
        "target_force_n": args_cli.target_force,
        "curve": curve,
    }
    with open(os.path.join(args_cli.out_dir, f"{prefix}calibration.json"), "w") as f:
        json.dump(calib, f, indent=2)

    try:  # force-vs-theta figure (matplotlib is available in-Kit via the omni prebundle)
        import matplotlib  # noqa: PLC0415

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: PLC0415

        th = [c["theta_meas"] for c in curve]
        fL = [c["pad_forces_n"][0] for c in curve]
        fR = [c["pad_forces_n"][1] for c in curve]
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(th, fL, "o-", label="left pad")
        ax.plot(th, fR, "s-", label="right pad")
        ax.axvline(theta_touch, ls="--", c="gray", label=f"touch {theta_touch:.3f}")
        ax.axvline(theta_grasp, ls="-", c="k", label=f"grasp {theta_grasp:.3f}")
        ax.axhline(args_cli.target_force, ls=":", c="r")
        ax.set_xlabel("finger_joint [rad]")
        ax.set_ylabel("pad contact force [N]")
        ax.set_title(f"pinch force vs aperture (rim_z {rim_z:+.4f} m)")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(args_cli.out_dir, f"{prefix}force_vs_theta.png"), dpi=120)
        print(f"[INFO] figure: {os.path.join(args_cli.out_dir, f'{prefix}force_vs_theta.png')}")
    except ImportError:
        print("[WARN] matplotlib unavailable in-Kit — render force_vs_theta from "
              "calibration.json with the venv later")

    if args_cli.verify:
        print("[INFO] verify mode: frozen constants gated above — config.py and "
              "docs/grasp_calibration.md untouched")
        print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
        if FAILURES:
            raise SystemExit(1)
        return

    doc = os.path.join(PROJECT_ROOT, "docs", "grasp_calibration.md")
    with open(doc, "w") as f:
        f.write("# Grasp calibration (generated by scripts/setup/calibrate_grasp.py)\n\n")
        f.write(f"Setup: mug `{config.OBJECT_NAME}` welded at rim_z = {rim_z:+.4f} m; staircase "
                f"dtheta {args_cli.dtheta} rad / dwell {args_cli.dwell} steps; target steady "
                f"pad force {args_cli.target_force} N.\n\n")
        f.write(f"- theta_touch = **{theta_touch:.3f} rad** (both pads >= 0.2 N sustained)\n")
        f.write(f"- theta_grasp = **{theta_grasp:.3f} rad** (steady {steady:.2f} N/pad over the "
                f"last 100 hold steps)\n")
        f.write(f"- force band = **[{band_lo}, {band_hi}] N** (steady/3 .. 3x steady)\n")
        f.write(f"- inner-finger mimic signs = {signs}\n")
        f.write(f"- weld error at hold = {weld_err_mm():.2f} mm\n\n")
        f.write("| theta_cmd | theta_meas | pad L [N] | pad R [N] | non-pad max [N] |\n")
        f.write("|---|---|---|---|---|\n")
        for c in curve:
            f.write(f"| {c['theta_cmd']:.3f} | {c['theta_meas']:.4f} | {c['pad_forces_n'][0]:.2f} "
                    f"| {c['pad_forces_n'][1]:.2f} | {c['nonpad_max_n']:.2f} |\n")
    print(f"[INFO] doc: {doc}")

    print("[INFO] freeze into src/dishsim/config.py:")
    print(f"       GRASP_RIM_TCP_Z_M = {rim_z:+.4f}")
    print(f"       GRIPPER_APERTURE_GRASP_RAD = {theta_grasp:.3f}")
    print(f"       GRIPPER_INNER_FINGER_SIGNS = {signs}")
    print(f"       GRIP_FORCE_MIN_N = {band_lo}")
    print(f"       GRIP_FORCE_MAX_N = {band_hi}")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
