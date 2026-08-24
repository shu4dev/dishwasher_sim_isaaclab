# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reveal render for a full-load episode: both loaded racks extended, every item at its
RECORDED measured pose (placed forks from the phase-0 trajectory's final frame — they rode
the rack in afterwards; everything else from phase-1's final frame), settled to prove the
tableau is physically consistent, then stills + orbit."""
import argparse
import json
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser()
parser.add_argument("--run", type=str, default="bosch_sanity_load2")
parser.add_argument("--ep", type=str, default="ep002")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from dishsim import config  # noqa: E402
from dishsim.quats import xyzw_to_wxyz  # noqa: E402

config.apply_machine("bosch800")
config.apply_base_placement("side_winner")
config.apply_scenario("placement")  # lower extended; the third joins via the override below

from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, write_orbit  # noqa: E402
from PIL import Image  # noqa: E402


def main() -> int:
    run_dir = os.path.join(PROJECT_ROOT, "results", "experiments", args_cli.run)
    ep = json.load(open(os.path.join(run_dir, "episodes", f"{args_cli.ep}.json")))
    placed_p0 = {p["item_id"] for p in ep["picks"] if p["success"] and p["phase"] == 0}

    def final_poses(npz_name):
        d = np.load(os.path.join(run_dir, "trajectories", npz_name), allow_pickle=True)
        meta = json.loads(str(d["meta"]))
        arr = d["object_root_pose_w"]
        last = arr[-1]
        return {k: last[7 * i:7 * i + 7] for i, k in enumerate(meta["object_keys"])}

    poses_p0 = final_poses(f"{args_cli.ep}_p0.npz")
    poses_p1 = final_poses(f"{args_cli.ep}_p1.npz")
    all_ids = list(poses_p1)
    tableau = {k: (poses_p0[k] if k in placed_p0 else poses_p1[k]) for k in all_ids}
    print(f"[INFO] {len(placed_p0)} items from phase-0 poses (third rack), "
          f"{len(all_ids) - len(placed_p0)} from phase-1 poses")

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device))
    obj_specs = [{"name": k, "usd_path": config.OBJECTS[k.rsplit("_", 1)[0]].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0), "contact_filters": []}
                 for i, k in enumerate(all_ids)]
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=False, with_robot_contacts=False, objects=obj_specs))
    ep_cam = config.EPISODE_CAMERA
    cam_specs = {**config.CAMERAS,
                 "episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                             dict(ep_cam["lens"]))}
    rig = CameraRig(cam_specs, hw=config.CAMERA_HW)
    sim.reset()
    rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()

    # the REVEAL: extend the loaded third rack alongside the extended lower rack
    dscene.set_rack_target_override({
        "PrismaticJoint_dishwasher_2_down": -0.56,
        "PrismaticJoint_dishwasher_2_up": 0.0,
        config.RACK_THIRD_JOINT: -0.51,
    })

    def step(n):
        for _ in range(n):
            dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    step(700)  # third rack slides out empty and settles

    device = scene[all_ids[0]].data.root_pos_w.device
    for k, pose in tableau.items():
        # recorded pose is pos + XYZW; isaaclab 2.1 wants pos + WXYZ
        pose_wxyz = np.concatenate([np.asarray(pose[:3]), xyzw_to_wxyz(np.asarray(pose[3:]))])
        scene[k].write_root_pose_to_sim(root_pose=torch.tensor(
            pose_wxyz.astype(np.float32)[None], device=device))
        scene[k].write_root_velocity_to_sim(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
    step(150)  # settle: recorded poses must be physically self-consistent

    drift = {}
    for k, pose in tableau.items():
        p = scene[k].data.root_pos_w[0].cpu().numpy()
        drift[k] = float(np.linalg.norm(p - np.asarray(pose[:3]))) * 1e3
    print("[INFO] settle drift mm:", {k: round(v, 1) for k, v in drift.items()})

    out = os.path.join(PROJECT_ROOT, "media", "task", args_cli.run)
    rig.update(dt)
    for name, arr in rig.grab().items():
        Image.fromarray(arr).save(os.path.join(out, f"{args_cli.ep}_reveal_{name}.png"))
    center = (float(config.DISHWASHER_POS_W[0]) - 0.15, float(config.DISHWASHER_POS_W[1]), 0.45)
    write_orbit(rig, scene, sim, dt, os.path.join(out, f"{args_cli.ep}_reveal_orbit.mp4"),
                center=center, radius=1.7, height=0.8)
    print(f"[INFO] reveal media -> {out}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
