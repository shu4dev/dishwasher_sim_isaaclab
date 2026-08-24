# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Render trial videos from recorded trajectories — no planning, no physics.

Reads a trial's ``.npz`` recording, rebuilds the scene it describes, and replays it
kinematically with the cameras on. Because the geometry comes from the recording rather than
a re-execution, any trial can be re-rendered later at a different resolution or camera angle
without re-running the experiment, and experiments themselves can run headless.

Outputs per trial, into ``--out``:
    <trial>_<camera>.mp4   one clip per camera
    <trial>_final.png      the last recorded frame from the iso camera

Run:
    scripts/run_kit.sh scripts/evaluation/render_videos.py --headless --enable_cameras \\
        --trial results/trajectories/trial_07_00_0.npz
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import glob
import json
import os

import numpy as np

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Render trial videos from trajectory recordings.")
parser.add_argument("--trial", type=str, default=None, help="A trial .npz (or use --run_dir).")
parser.add_argument("--run_dir", type=str, default=None,
                    help="Directory of .npz recordings to render (all trials in it).")
parser.add_argument("--out", type=str, default=None, help="Output dir (default: media/trials/replay).")
parser.add_argument("--cameras", type=str, default=None,
                    help="Comma list of cameras to render (default: all in the recording).")
parser.add_argument("--use_current_cameras", action="store_true",
                    help="Use config.CAMERAS instead of the poses stored in the recording.")
parser.add_argument("--stride", type=int, default=1,
                    help="Render every Nth captured frame (>1 gives a faster preview).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

TRIALS = sorted(glob.glob(os.path.join(args_cli.run_dir, "*.npz"))) if args_cli.run_dir else []
if args_cli.trial:
    TRIALS.insert(0, args_cli.trial)
if not TRIALS:
    raise SystemExit("[FAIL] nothing to render — pass --trial or --run_dir")

# every recording in a batch must share object+scenario: the scene is built once, and the
# object/scenario must be applied before the scene modules are imported
with np.load(TRIALS[0], allow_pickle=False) as _z:
    META0 = json.loads(str(_z["meta"]))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

config.set_active_object(META0["object"])
config.apply_scenario(META0["scenario"])

from dishsim import replay as dreplay  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim import trajectory as dtraj  # noqa: E402
from dishsim.geometry import config_hash  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402

from dishsim.checks import FAILURES, check, finish  # noqa: E402


def main() -> None:
    out_dir = args_cli.out or os.path.join(PROJECT_ROOT, "media", "trials", "replay")
    os.makedirs(out_dir, exist_ok=True)

    cameras = {k: (tuple(v[0]), tuple(v[1])) for k, v in META0["cameras"].items()}
    if args_cli.use_current_cameras:
        cameras = dict(config.CAMERAS)
    if args_cli.cameras:
        want = {c.strip() for c in args_cli.cameras.split(",")}
        cameras = {k: v for k, v in cameras.items() if k in want}
    hw = tuple(META0.get("camera_hw", config.CAMERA_HW))
    fps = int(META0.get("camera_fps", config.CAMERA_FPS))

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device,
                                gravity=(0.0, 0.0, 0.0))
    )
    # order matters and mirrors the experiment runner: scene first, then the rig, then
    # reset, then poses (cameras created before the scene end up mis-posed)
    # no weld, no contact sensors: playback writes every body, nothing is simulated
    # The scene is built ONCE, before the per-trial loop, so it must be shaped by the first
    # recording — not by `rec`, which does not exist until the loop below binds it. Reading it
    # here raised NameError at startup and made this script unrunnable.
    rec0 = dtraj.TrajectoryRecording.load(TRIALS[0])
    objs = dreplay.scene_objects_for(rec0)
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=not objs, with_robot_contacts=False, objects=objs))
    rig = CameraRig(cameras, hw=hw)
    sim.reset()
    rig.apply_poses(sim.device)
    # the articulation ROOT poses (and the dishwasher's door/rack defaults) only reach the
    # physics view through this call — without it the machine sits ~25 cm off its spawn pose
    # and the replay renders a subtly wrong scene. The player overwrites joint positions
    # per frame; the roots come from here.
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)

    live_hash = config_hash()
    for path in TRIALS:
        rec = dtraj.TrajectoryRecording.load(path)
        meta = rec.meta
        if meta["object"] != META0["object"] or meta["scenario"] != META0["scenario"]:
            check(f"{meta['trial']} scene match", False,
                  "recording's object/scenario differs from the built scene — render it separately")
            continue
        if meta.get("config_hash") != live_hash:
            print(f"[WARN] {meta['trial']}: config drift "
                  f"({meta.get('config_hash')} vs live {live_hash}) — geometry may differ")

        player = dreplay.TrajectoryPlayer(scene, sim, rec)
        player.warmup(rig, config.SIM_DT)

        steps = rec.captured_steps[:: max(1, args_cli.stride)]
        writers = {name: VideoWriter(os.path.join(out_dir, f"{meta['trial']}_{name}.mp4"), fps=fps)
                   for name in cameras}
        kept: list[np.ndarray] = []
        first_cam = next(iter(cameras))
        for step in steps:
            player.write_step(int(step))
            player.advance(rig, config.SIM_DT)
            frames = rig.grab()
            for name, w in writers.items():
                w.add(frames[name])
            if len(kept) < 400:  # sample for the variation guard
                kept.append(frames[first_cam])
        for w in writers.values():
            w.close()

        # the final still: last recorded state, iso camera aimed like the live runner's
        still = None
        if "iso" in cameras:
            player.write_step(int(rec.captured_steps[-1]))
            player.advance(rig, config.SIM_DT)
            from PIL import Image  # noqa: PLC0415

            still = os.path.join(out_dir, f"{meta['trial']}_final.png")
            Image.fromarray(rig.grab()["iso"]).save(still)

        n_expected = len(steps)
        check(f"{meta['trial']} frame count", writers[first_cam].frames == n_expected,
              f"{writers[first_cam].frames} frames written, expected {n_expected}")
        varied, detail = dreplay.assert_frames_vary(kept)
        check(f"{meta['trial']} frames vary", varied, detail)
        dark = float(np.mean([f.std() for f in kept[:10]])) if kept else 0.0
        check(f"{meta['trial']} not black", dark > 5.0, f"mean std of the first frames {dark:.1f}")
        print(f"[INFO] {meta['trial']}: {n_expected} frames from {len(rec)} recorded steps -> {out_dir}"
              + (f", still {os.path.basename(still)}" if still else ""))

    finish()


if __name__ == "__main__":
    main()
