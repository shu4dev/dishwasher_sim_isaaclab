#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Bosch 800 world bring-up: first Kit validation of the self-authored machine.

Spawns the full v0 scene under ``apply_machine("bosch800")`` (machine + robot + pedestal +
counter), settles physics, and gates on:

- all three rack joints settled at the scenario targets (< 5 mm),
- the door resting inside its clamped band [open - band, open],
- zero residual body velocity (no explosion / joint fight),
- non-black camera frames.

Evidence: three stills (front/top/iso from the machine's camera set) plus a lower-rack
slide in/out MP4 under ``media/bosch800/bringup/`` — the prismatic joints working on
camera. Judge from ``[RESULT] PASS`` in the log, never the exit code.

    scripts/run_kit.sh scripts/setup/bosch_bringup.py --headless --enable_cameras
"""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

parser = argparse.ArgumentParser(description="Bosch 800 machine bring-up + media evidence.")
parser.add_argument("--scenario", type=str, default="placement",
                    help="Rack state to bring up (default: placement — lower rack out over the door).")
parser.add_argument("--settle_steps", type=int, default=300)
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "media", "bosch800", "bringup"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

_enable_cameras = args_cli.enable_cameras  # 2.1's AppLauncher pops this off the namespace
app_launcher = AppLauncher(args_cli)
args_cli.enable_cameras = _enable_cameras
simulation_app = app_launcher.app

import sys  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

config.apply_machine("bosch800")
config.apply_scenario(args_cli.scenario)

from dishsim import robots  # noqa: E402
from dishsim import scene as dscene  # noqa: E402

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def main() -> None:
    # Fabric ON (the default), unlike the extraction scripts: with use_fabric=False the
    # RTX renderer keeps drawing articulation links at their authored poses while physics
    # moves them (measured: static rack-slide video, moving joint states). Extraction
    # needs USD-readable state; MEDIA needs Fabric.
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device)
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=False))

    camera = None
    if args_cli.enable_cameras:
        camera = Camera(
            CameraCfg(
                prim_path="/World/Camera",
                update_period=0.0,
                height=config.CAMERA_HW[0],
                width=config.CAMERA_HW[1],
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955,
                    clipping_range=(0.1, 100.0),
                ),
            )
        )

    sim.reset()
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    for _ in range(args_cli.settle_steps):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    dscene.assert_frames(scene)

    # --- joint gates -------------------------------------------------------------------------
    dw = scene["dishwasher"]
    joint_names = list(dw.joint_names)
    q = dw.data.joint_pos[0].detach().cpu().numpy()
    qd = dw.data.joint_vel[0].detach().cpu().numpy()

    for jn, target in config.RACK_JOINT_TARGETS.items():
        idx = joint_names.index(jn)
        err = abs(q[idx] - target)
        check(f"rack joint {jn} at target", err < 5e-3, f"{q[idx]:+.4f} m vs {target:+.3f} m (err {err * 1e3:.1f} mm)")

    door_idx = joint_names.index(robots.DISHWASHER_DOOR_JOINT)
    door_deg = float(np.degrees(q[door_idx]))
    lo, hi = config.DOOR_OPEN_DEG - config.DOOR_BAND_DEG, config.DOOR_OPEN_DEG
    check("door inside the clamped band", lo - 0.2 <= door_deg <= hi + 0.2, f"{door_deg:.2f} deg in [{lo}, {hi}]")
    check("machine at rest", float(np.abs(qd).max()) < 5e-3, f"max |qd| {np.abs(qd).max():.2e}")

    root_vel = dw.data.root_vel_w[0].detach().cpu().numpy()
    check("base not moving (fix_root_link)", float(np.abs(root_vel).max()) < 1e-4, "")

    # --- media evidence ----------------------------------------------------------------------
    if camera is not None:
        os.makedirs(args_cli.out_dir, exist_ok=True)
        from PIL import Image  # noqa: PLC0415
        import imageio  # noqa: PLC0415

        def grab() -> np.ndarray:
            data = camera.data.output["rgb"][0]
            return data.detach().cpu().numpy()[..., :3].astype(np.uint8)

        def look(name: str) -> None:
            eye, target = config.CAMERAS[name]
            camera.set_world_poses_from_view(
                eyes=torch.tensor([list(eye)], device=sim.device),
                targets=torch.tensor([list(target)], device=sim.device),
            )
            for _ in range(10):  # renderer warm-up after a teleport
                sim.step()
                camera.update(dt)

        for name in config.CAMERAS:
            look(name)
            frame = grab()
            check(f"camera {name} non-black", float(frame.std()) > 5.0,
                  f"mean {frame.mean():.1f}, std {frame.std():.1f}")
            Image.fromarray(frame).save(os.path.join(args_cli.out_dir, f"bringup_{name}.png"))

        # motion evidence: slide the lower rack in and back out on the iso camera
        look("iso")
        mp4_path = os.path.join(args_cli.out_dir, "bringup_rack_slide.mp4")
        writer = imageio.get_writer(mp4_path, fps=30, codec="libx264", quality=7, macro_block_size=None)
        lower_joint = "PrismaticJoint_dishwasher_2_down"
        start = config.RACK_JOINT_TARGETS[lower_joint]
        frames_half = 90
        stds = []
        for leg_from, leg_to in ((start, 0.0), (0.0, start)):  # in, then back out
            for i in range(frames_half):
                alpha = (i + 1) / frames_half
                current = leg_from + (leg_to - leg_from) * alpha
                dscene.set_rack_target_override({lower_joint: current})
                dscene.hold_targets(scene)
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
                camera.update(dt)
                f = grab()
                stds.append(float(f.std()))
                writer.append_data(f)
        writer.close()
        dscene.set_rack_target_override(None)
        check("rack-slide MP4 non-black throughout", min(stds) > 5.0, f"min std {min(stds):.1f}")
        check("MP4 written", os.path.isfile(mp4_path) and os.path.getsize(mp4_path) > 50_000, mp4_path)

        # the video legs ramp ~4x faster than the calibrated slide rate (they are visual
        # evidence, not the action) — give the drive a short settle at the final target
        # before judging the endpoint
        for _ in range(60):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

        # the slide must end back at the scenario target
        q2 = dw.data.joint_pos[0].detach().cpu().numpy()
        err = abs(q2[joint_names.index(lower_joint)] - start)
        check("rack back at scenario target after slide", err < 8e-3, f"err {err * 1e3:.1f} mm")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL — ' + ', '.join(FAILURES)}")


if __name__ == "__main__":
    main()
    simulation_app.close()
