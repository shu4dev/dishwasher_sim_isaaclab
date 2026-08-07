# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency gate: planning stack inside the Kit process + headless media capture.

Verifies, in one Isaac Sim session:

1. ``ompl``, ``fcl``, ``coacd``, ``trimesh``, ``imageio`` import *inside* the Kit process
   (run_trials plans in-process, so a Kit/wheel symbol clash would sink the design — fail fast here).
2. A small RRT-Connect plan solves in-process.
3. A camera sensor renders headlessly and the frames are non-black: writes
   ``media/smoke/smoke.png`` and a 2 s, 720p, 30 fps ``media/smoke/smoke.mp4`` of a cube dropping.

Run with:
    /workspace/isaaclab/isaaclab.sh -p scripts/setup/kit_smoke.py --headless --enable_cameras
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Kit-process planning-stack + media smoke test.")
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "media", "smoke"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def test_imports() -> None:
    import coacd  # noqa: F401
    import fcl  # noqa: F401
    import imageio  # noqa: F401
    import trimesh  # noqa: F401
    from ompl import base, geometric, util  # noqa: F401

    import dishsim.ur5e_kin  # noqa: F401  (also runs its import-time self-check)

    check("in-Kit imports (ompl, fcl, coacd, trimesh, imageio, dishsim)", True)


def test_inprocess_plan() -> None:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import util as ou

    ou.setLogLevel(ou.LOG_WARN)
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(lambda s: math.hypot(s[0] - 0.5, s[1] - 0.5) > 0.25)
    start, goal = space.allocState(), space.allocState()
    start[0], start[1] = 0.05, 0.05
    goal[0], goal[1] = 0.95, 0.95
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    solved = ss.solve(5.0)
    check("in-process RRT-Connect toy plan", bool(solved) and ss.haveExactSolutionPath())


def main() -> None:
    test_imports()
    test_inprocess_plan()

    # --- minimal scene: ground, light, one falling cube -----------------------------------
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 60.0, device=args_cli.device, physics=PhysxCfg()))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/light", light_cfg)
    cube_cfg = sim_utils.CuboidCfg(
        size=(0.25, 0.25, 0.25),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.3, 0.1)),
    )
    cube_cfg.func("/World/cube", cube_cfg, translation=(0.0, 0.0, 1.0))

    camera = Camera(
        CameraCfg(
            prim_path="/World/Camera",
            update_period=0.0,
            height=720,
            width=1280,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 100.0)
            ),
        )
    )

    sim.reset()
    camera.set_world_poses_from_view(
        eyes=torch.tensor([[1.8, 1.8, 1.2]], device=sim.device),
        targets=torch.tensor([[0.0, 0.0, 0.3]], device=sim.device),
    )
    dt = sim.get_physics_dt()

    # warm-up renders (first frames can come back black while the renderer spins up)
    for _ in range(10):
        sim.step()
        camera.update(dt)

    def grab() -> np.ndarray:
        data = camera.data.output["rgb"].torch[0]
        frame = data.detach().cpu().numpy()
        return frame[..., :3].astype(np.uint8)

    os.makedirs(args_cli.out_dir, exist_ok=True)

    # --- PNG -------------------------------------------------------------------------------
    frame = grab()
    check(
        "camera frame is non-black",
        float(frame.std()) > 5.0,
        f"shape {frame.shape}, mean {frame.mean():.1f}, std {frame.std():.1f}",
    )
    from PIL import Image

    png_path = os.path.join(args_cli.out_dir, "smoke.png")
    Image.fromarray(frame).save(png_path)
    check("PNG written", os.path.isfile(png_path) and os.path.getsize(png_path) > 10_000, png_path)

    # --- 2 s MP4 of the cube dropping ------------------------------------------------------
    import imageio

    mp4_path = os.path.join(args_cli.out_dir, "smoke.mp4")
    writer = imageio.get_writer(mp4_path, fps=30, codec="libx264", quality=7, macro_block_size=None)
    stds = []
    for _ in range(60):
        sim.step()
        camera.update(dt)
        f = grab()
        stds.append(float(f.std()))
        writer.append_data(f)
    writer.close()
    check(
        "MP4 written (60 frames @ 30 fps, 720p)",
        os.path.isfile(mp4_path) and os.path.getsize(mp4_path) > 50_000,
        f"{mp4_path} ({os.path.getsize(mp4_path)} B)",
    )
    check("video frames non-black throughout", min(stds) > 5.0, f"min std {min(stds):.1f}")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
