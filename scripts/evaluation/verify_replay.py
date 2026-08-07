# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Prove a recorded trajectory replays kinematically — without rendering anything.

Rebuilds the scene from the recording's own metadata, writes each recorded state, and compares
the *read-back* robot link poses against the checksums the recorder stored every
:data:`dishsim.trajectory.CHECK_STRIDE` steps. If those agree, the replayed video is showing
exactly the geometry the experiment produced; any disagreement is a real reconstruction bug,
isolated from all rendering noise.

This is the gate to run before trusting ``render_videos.py``. It needs no cameras and takes
seconds.

Run:
    scripts/run_kit.sh scripts/evaluation/verify_replay.py --headless --trial results/trajectories/trial_07_00_0.npz
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import json
import os

import numpy as np

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Verify a trajectory recording replays exactly.")
parser.add_argument("--trial", type=str, required=True, help="Path to a trial .npz recording.")
parser.add_argument("--tol_pos_m", type=float, default=1e-4,
                    help="Max link-position error [m] for the pure kinematic reconstruction.")
parser.add_argument("--tol_quat", type=float, default=1e-3, help="Max link-quaternion error.")
parser.add_argument("--tol_render_m", type=float, default=1e-3,
                    help="Max link-position error [m] on the render path (one step of settling).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# The recording carries its own object/scenario; both must be applied BEFORE the scene modules
# are imported (they bind the derived USD path and the object USD at import time).
with np.load(args_cli.trial, allow_pickle=False) as _z:
    META = json.loads(str(_z["meta"]))

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

config.set_active_object(META.get("object") or META["classes"][0])
config.apply_scenario(META["scenario"])

from dishsim import replay as dreplay  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim import trajectory as dtraj  # noqa: E402
from dishsim.geometry import config_hash  # noqa: E402


def main() -> None:
    rec = dtraj.TrajectoryRecording.load(args_cli.trial)
    print(f"[INFO] {META['trial']}: {len(rec)} steps, object={META['object']}, "
          f"scenario={META['scenario']}")
    live_hash = config_hash()
    if rec.meta.get("config_hash") != live_hash:
        print(f"[WARN] config drift: recording {rec.meta.get('config_hash')} vs live {live_hash} "
              "— geometry may differ from what was recorded")

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device,
                                gravity=(0.0, 0.0, 0.0), physics=PhysxCfg())
    )
    # no weld is authored and no contact sensors are needed: nothing is simulated, every body
    # is written from the recording each frame
    objs = dreplay.scene_objects_for(rec)
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=not objs, with_robot_contacts=False, objects=objs))
    sim.reset()
    # root poses + dishwasher defaults must reach the physics view before playback
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    player = dreplay.TrajectoryPlayer(scene, sim, rec)

    def sweep(propagate) -> tuple[float, float, int]:
        max_pos, max_quat, worst = 0.0, 0.0, -1
        for i, step in enumerate(rec.check_step_idx):
            player.write_step(int(step))
            propagate()
            got_pos, got_quat = player.read_body_poses()
            want = rec.check_body_pose_w[i]
            dp = float(np.abs(got_pos - want[:, :3]).max())
            # quaternions are sign-ambiguous: compare on the closer representation
            sign = np.sign(np.sum(got_quat * want[:, 3:], axis=1, keepdims=True))
            sign[sign == 0] = 1.0
            dq = float(np.abs(got_quat * sign - want[:, 3:]).max())
            if dp > max_pos:
                max_pos, worst = dp, int(step)
            max_quat = max(max_quat, dq)
        return max_pos, max_quat, worst

    n = len(rec.check_step_idx)
    # 1. the reconstruction itself: writing the recorded joints must reproduce every link pose
    #    exactly. A failure here is a real bug — wrong columns, a stale remap, a bad recording.
    kin_pos, kin_quat, kin_worst = sweep(sim.forward)
    ok_kin = kin_pos <= args_cli.tol_pos_m and kin_quat <= args_cli.tol_quat
    print(f"[{'OK' if ok_kin else 'FAIL'}] kinematic reconstruction over {n} checkpoints: "
          f"max link-position error {kin_pos * 1e3:.4f} mm (step {kin_worst}), "
          f"max quaternion error {kin_quat:.2e}")

    # 2. the path the renderer actually uses. Transforms only reach RTX through a physics
    #    step, so playback pays one step of settling; drives are commanded to the recorded
    #    pose to keep that at the sub-millimetre level.
    ren_pos, ren_quat, ren_worst = sweep(player.advance)
    ok_ren = ren_pos <= args_cli.tol_render_m
    print(f"[{'OK' if ok_ren else 'FAIL'}] render path over {n} checkpoints: "
          f"max link-position error {ren_pos * 1e3:.4f} mm (step {ren_worst}), "
          f"max quaternion error {ren_quat:.2e} (tolerance {args_cli.tol_render_m * 1e3:.1f} mm)")

    ok = ok_kin and ok_ren
    print(f"[RESULT] {'PASS' if ok else 'FAIL'}")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
