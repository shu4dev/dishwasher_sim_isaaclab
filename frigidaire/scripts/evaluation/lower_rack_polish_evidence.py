# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Capture one PNG and one orbit MP4 of the revised lower rack in Isaac Sim.

scripts/run_kit.sh frigidaire/scripts/evaluation/lower_rack_polish_evidence.py --headless --enable_cameras

The rack settles on its eight wheel colliders before recording. The video moves the
camera around the rack, then overhead to expose the wire/tine layout. It does not
claim to validate dish loading, folding mechanisms, or the appliance articulation.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "frigidaire/src"))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, default=ROOT / "assets/models/bosch800/lower_rack.usdc")
parser.add_argument("--out_dir", type=Path, default=ROOT / "media/bosch800_lower_rack_polish")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1440)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if not args_cli.enable_cameras:
    parser.error("--enable_cameras is required for authentic RTX image/video capture")
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

from dishsim.media import CameraRig, VideoWriter, release_sim_for_close  # noqa: E402


def eye_at(yaw, elevation, distance=1.40):
    yaw, elevation = math.radians(yaw), math.radians(elevation)
    return (distance*math.cos(elevation)*math.cos(yaw),
            -0.010+distance*math.cos(elevation)*math.sin(yaw),
            0.082+distance*math.sin(elevation))


def camera_path(t):
    # Continuous positions and zero endpoint velocity at every move/hold boundary.
    def smooth(x):
        return x*x*x*(10+x*(-15+6*x))
    if t < 1:
        return -52, 42
    if t < 11:
        return -52-360*smooth((t-1)/10), 42
    if t < 14:
        a=smooth((t-11)/3)
        return -412-38*a, 42+44*a
    if t < 16:
        return -450, 86
    a=smooth(min(1, (t-16)/3))
    return -450+38*a, 86-44*a


def main():
    import isaaclab.sim as sim_utils
    import omni.usd
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sim import SimulationContext

    source = Usd.Stage.Open(str(args_cli.usd))
    assert source, f"Cannot open {args_cli.usd}"
    revision = source.GetDefaultPrim().GetAttribute("geometryRevision").Get()
    assert revision == "bottom_rack_four_views_v1", "Install the revised asset first, or pass --usd build/bosch800_lower_rack/lower_rack.usda"
    args_cli.out_dir.mkdir(parents=True, exist_ok=True)
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1/60, device=args_cli.device, use_fabric=False))
    ground = sim_utils.GroundPlaneCfg(color=(0.20, 0.23, 0.26))
    ground.func("/World/Ground", ground)
    dome = sim_utils.DomeLightCfg(intensity=1100.0, color=(0.90, 0.94, 1.0))
    dome.func("/World/Fill", dome)
    key = sim_utils.DistantLightCfg(intensity=2200.0, angle=14.0, color=(1.0, 0.97, 0.92))
    key.func("/World/Key", key, orientation=(0.9238795, 0.3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1000.0, angle=20.0, color=(0.85, 0.92, 1.0))
    rim.func("/World/Rim", rim, orientation=(0.7071068, -0.5, 0.5, 0))
    rack = RigidObject(RigidObjectCfg(
        prim_path="/World/LowerRack",
        spawn=sim_utils.UsdFileCfg(usd_path=str(args_cli.usd),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False, disable_gravity=False)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 0.065)),
    ))
    lens = {"focal_length": 50.0, "horizontal_aperture": 36.0}
    target = (0, -0.010, 0.082)
    rig = CameraRig({"hero": (eye_at(-52, 42), target, lens)}, hw=(args_cli.height, args_cli.width))
    sim.reset()
    rig.apply_poses(sim.device)
    # A physical settle gate catches missing/invalid colliders before any media passes.
    tail=[]
    for i in range(240):
        sim.step()
        rack.update(1/60)
        if i >= 180:
            tail.append(rack.data.root_pos_w[0].cpu().numpy().copy())
    pose = rack.data.root_pos_w[0].cpu().numpy()
    speed = float(torch.linalg.vector_norm(rack.data.root_lin_vel_w[0]))
    span = float(np.ptp(np.asarray(tail), axis=0).max())
    assert .024 < pose[2] < .029, f"Wheel/floor contact height failed: {pose}"
    assert np.linalg.norm(pose[:2]) < .01 and span < .001 and speed < .02, (pose, span, speed)
    for _ in range(24):
        sim.step()
        rack.update(1/60)
        rig.update(1/60)
    still_path = args_cli.out_dir / "lower_rack.png"
    still = rig.grab_one("hero")
    assert still.std() > 5 and still.max() > 50, "Blank camera output"
    Image.fromarray(still).save(still_path)
    video_path = args_cli.out_dir / "lower_rack.mp4"
    video = VideoWriter(str(video_path), fps=30)
    cam = rig.cams["hero"]
    target_tensor = torch.tensor([target], dtype=torch.float32, device=sim.device)
    try:
        for frame in range(570):
            yaw, elevation = camera_path(frame/30)
            cam.set_world_poses_from_view(
                eyes=torch.tensor([eye_at(yaw, elevation)], dtype=torch.float32, device=sim.device),
                targets=target_tensor,
            )
            for _ in range(2):
                sim.step()
                rack.update(1/60)
            rig.update(1/30)
            video.add(rig.grab_one("hero"))
            if frame % 60 == 0:
                print(f"[INFO] Captured {frame}/570 RTX frames", flush=True)
    finally:
        video.close()
    assert video.frames == 570 and video_path.stat().st_size > 100_000
    final_pose = rack.data.root_pos_w[0].cpu().numpy()
    assert np.linalg.norm(final_pose-pose) < .002, "Rack moved during the inspection"
    stage = omni.usd.get_context().get_stage()
    meshes = sum(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/LowerRack")))
    version_file = Path("/isaac-sim/VERSION")
    report = {
        "result": "PASS", "renderer": "Isaac Sim RTX camera sensor",
        "isaac_sim_version": version_file.read_text().strip() if version_file.exists() else "See run log",
        "isaac_lab_version": importlib.metadata.version("isaaclab"),
        "asset": str(args_cli.usd), "asset_sha256": hashlib.sha256(args_cli.usd.read_bytes()).hexdigest(),
        "revision": revision, "meshes": meshes, "settled_position_m": pose.tolist(),
        "settled_speed_m_s": speed, "settled_position_span_m": span,
        "image": str(still_path), "video": str(video_path), "frames": video.frames,
        "fps": 30, "resolution": [args_cli.width, args_cli.height],
        "scope": "Wheel/floor settling and camera inspection; dish contacts and articulation not tested",
    }
    (args_cli.out_dir / "evidence.json").write_text(json.dumps(report, indent=2)+"\n")
    print("[RESULT] PASS — lower rack settled; PNG and 19 s RTX orbit video captured", flush=True)


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        import traceback
        traceback.print_exc()
        print("[RESULT] FAIL", flush=True)
        raise
    finally:
        release_sim_for_close()
        simulation_app.close()
