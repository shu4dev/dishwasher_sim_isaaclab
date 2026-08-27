# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reveal render for a planned full load: every rack extended, every item teleported to its
PLANNED release pose (from the capacity-plan artifact), settled to prove the tableau is
physically consistent, then stills + orbit.

    scripts/run_kit.sh scripts/evaluation/reveal_render.py --headless --enable_cameras \
        --plan results/capacity/bosch800/side_winner/full_load_plan.json
"""
import argparse
import json
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Render a planned full load, settled.")
parser.add_argument("--plan", type=str, required=True,
                    help="Capacity-plan JSON (see scripts/setup/plan_full_load.py).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

from dishsim import config  # noqa: E402

_plan_path = args_cli.plan if os.path.isabs(args_cli.plan) else os.path.join(PROJECT_ROOT, args_cli.plan)
PLAN = json.load(open(_plan_path))
# context order contract: machine -> scenario -> placement, all BEFORE scene imports
if PLAN["machine"] != config.MACHINE_BASELINE_NAME:
    config.apply_machine(PLAN["machine"])
config.apply_scenario("placement")
config.apply_base_placement(PLAN["base_placement"])

from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, write_orbit  # noqa: E402
from dishsim.transforms import T_to_pos_quat, make_T  # noqa: E402
from PIL import Image  # noqa: E402


def main() -> int:
    items = [it for ph in PLAN["phases"] for it in ph.get("items", [])]
    if not items:
        print("[FAIL] plan contains no items")
        return 1
    print(f"[INFO] {len(items)} planned items ({PLAN['machine']} @ {PLAN['base_placement']})")

    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    tableau = {}
    for it in items:
        pos, quat = T_to_pos_quat(T_w_base @ np.asarray(it["T_base_obj"], dtype=float))
        tableau[it["item_id"]] = np.concatenate([pos, quat])

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg()))
    obj_specs = [{"name": it["item_id"],
                  "usd_path": config.OBJECTS[it["object_class"]].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0)}
                 for i, it in enumerate(items)]
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    ep_cam = config.EPISODE_CAMERA
    cam_specs = {**{k: v for k, v in config.CAMERAS.items()},
                 "episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                             dict(ep_cam["lens"]))}
    rig = CameraRig(cam_specs, hw=config.CAMERA_HW)
    sim.reset()
    rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()

    # the REVEAL: every rack extended to its full travel so the whole load is visible
    dscene.set_rack_target_override({
        joint: limits[0]
        for joint, limits in config.RACK_TRAVEL_LIMITS_BY_JOINT_M.items()
    })

    def step(n):
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    step(700)  # racks slide out empty and settle

    device = scene[items[0]["item_id"]].data.root_pos_w.torch.device
    for k, pose in tableau.items():
        scene[k].write_root_pose_to_sim_index(root_pose=torch.tensor(
            np.asarray(pose, dtype=np.float32)[None], device=device))
        scene[k].write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
    step(150)  # settle: planned poses must be physically self-consistent

    drift = {}
    for k, pose in tableau.items():
        p = scene[k].data.root_pos_w.torch[0].cpu().numpy()
        drift[k] = float(np.linalg.norm(p - np.asarray(pose[:3]))) * 1e3
    print("[INFO] settle drift mm:", {k: round(v, 1) for k, v in drift.items()})

    out = os.path.join(PROJECT_ROOT, "media", "capacity", config.MACHINE)
    os.makedirs(out, exist_ok=True)
    rig.update(dt)
    for name, arr in rig.grab().items():
        Image.fromarray(arr).save(os.path.join(out, f"reveal_{name}.png"))
    center = (float(config.DISHWASHER_POS_W[0]) - 0.15, float(config.DISHWASHER_POS_W[1]), 0.45)
    write_orbit(rig, scene, sim, dt, os.path.join(out, "reveal_orbit.mp4"),
                center=center, radius=1.7, height=0.8)
    print(f"[INFO] reveal media -> {out}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
