# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Dependency gate: planning stack inside the Kit process + headless media capture.

Verifies, in one Isaac Sim session:

1. ``fcl``, ``coacd``, ``trimesh``, ``imageio`` import *inside* the Kit process (a Kit/wheel
   symbol clash would sink the design — fail fast here).
2. A camera sensor renders headlessly and the frames are non-black: writes
   ``data/media/smoke/smoke.png``.

Run with:
    code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))  # code/<stage>/<folder>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "code", "src"))

from dishsim.app import add_args, launch  # noqa: E402  (Kit-free; boots Kit below)

parser = argparse.ArgumentParser(description="Kit-process collision-stack + media smoke test.")
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "data", "media", "smoke"))
add_args(parser)
args_cli = parser.parse_args()

simulation_app = launch(args_cli)

"""Rest everything follows."""

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.renderers import IsaacRtxRendererCfg
from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxRigidBodyCfg

from dishsim.checks import check, finish  # noqa: E402
from dishsim.media import release_sim_for_close  # noqa: E402


def test_imports() -> None:
    import coacd  # noqa: F401
    import fcl  # noqa: F401
    import imageio  # noqa: F401
    import trimesh  # noqa: F401

    import dishsim.collision_world  # noqa: F401  (the Kit-free planning module imports in-Kit)

    check("in-Kit imports (fcl, coacd, trimesh, imageio, dishsim)", True)


def main() -> None:
    test_imports()

    # --- minimal scene: ground, light, one falling cube -----------------------------------
    sim = SimulationContext(sim_utils.SimulationCfg(
        dt=1.0 / 60.0, device=args_cli.device,
        physics=PhysxCfg(enable_external_forces_every_iteration=False)))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/light", light_cfg)
    cube_cfg = sim_utils.CuboidCfg(
        size=(0.25, 0.25, 0.25),
        rigid_props=[sim_utils.UsdPhysicsRigidBodyCfg(), PhysxRigidBodyCfg()],
        mass_props=sim_utils.MassCfg(mass=1.0),
        collision_props=[sim_utils.UsdPhysicsCollisionCfg(), PhysxCollisionCfg()],
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
            renderer_cfg=IsaacRtxRendererCfg(),
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

    finish()


if __name__ == "__main__":
    main()
    release_sim_for_close()
    simulation_app.close()
