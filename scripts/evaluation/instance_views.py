# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Initial-vs-goal stills for one benchmark instance: what the algorithm is handed, and what
it is asked to produce.

An episode video shows the solving; these two tableaux show the PROBLEM. The roster is spawned
once and rendered twice — teleported to the instance's recorded ``T_base_init`` (the scrambled
start), then to each item's ``target.T_base_obj`` (the certified goal) — so the pair differs
only in the arrangement. Objects are tinted per class (:func:`~dishsim.config.display_color`),
because the sourced props share one dark-red material and an untinted multi-class load renders
as an undifferentiated mass.

The rack stays at the instance's own state extension (``config.apply_scenario(inst.state)``) —
NOT the full-travel reveal that :mod:`scripts.evaluation.reveal_render` performs — because the
instance's poses were certified against that state's collision cache.

    scripts/run_kit.sh scripts/evaluation/instance_views.py --headless --enable_cameras \
        --instance results/instances/bosch800/placement/perturbed_s0.json
"""
import argparse
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Render an instance's initial and goal arrangements.")
parser.add_argument("--instance", type=str, required=True, help="Instance JSON (gen_instances.py).")
parser.add_argument("--out", type=str, default=None,
                    help="Media dir (default: media/instances/<machine>/<state>).")
parser.add_argument("--settle_steps", type=int, default=90,
                    help="Physics steps to settle each tableau before its stills.")
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
from dishsim import rearrange  # noqa: E402

_inst_path = (args_cli.instance if os.path.isabs(args_cli.instance)
              else os.path.join(PROJECT_ROOT, args_cli.instance))
INST = rearrange.Instance.load(_inst_path)
# context order contract: machine -> scenario -> placement, all BEFORE scene imports
if INST.machine != config.MACHINE_BASELINE_NAME:
    config.apply_machine(INST.machine)
config.apply_scenario(INST.state)
config.apply_base_placement(INST.base_placement)

from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig  # noqa: E402
from dishsim.transforms import T_to_pos_quat, make_T  # noqa: E402

WARMUP_STEPS = 300  # racks settle at the state's extensions before anything arrives


def main() -> int:
    items = INST.items
    if not items:
        print("[RESULT] FAIL (instance has no items)")
        return 1
    counts: dict[str, int] = {}
    for it in items:
        counts[it["object_class"]] = counts.get(it["object_class"], 0) + 1
    print(f"[INFO] {INST.name}: {len(items)} items {counts} "
          f"({INST.machine} @ {INST.base_placement}, state {INST.state})")

    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)

    def world_pose(T_base) -> np.ndarray:
        pos, quat = T_to_pos_quat(T_w_base @ np.asarray(T_base, dtype=float))
        return np.concatenate([pos, quat])

    arrangements = {
        "initial": {it["item_id"]: world_pose(it["T_base_init"]) for it in items},
        "goal": {it["item_id"]: world_pose(it["target"]["T_base_obj"]) for it in items},
    }

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg()))
    obj_specs = [{"name": it["item_id"],
                  "usd_path": config.OBJECTS[it["object_class"]].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0),
                  "color": config.item_color(it["item_id"])}
                 for i, it in enumerate(items)]
    for it in items:  # the legend a viewer needs to read the tableaux
        c = config.item_color(it["item_id"])
        print(f"[INFO] color {it['item_id']:10s} rgb=({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})")
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    ep_cam = config.EPISODE_CAMERA
    rig = CameraRig({**{k: v for k, v in config.CAMERAS.items()},
                     "episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                                 dict(ep_cam["lens"]))}, hw=config.CAMERA_HW)
    sim.reset()
    rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    device = scene[items[0]["item_id"]].data.root_pos_w.torch.device

    def step(n: int) -> None:
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    step(WARMUP_STEPS)

    out_dir = (args_cli.out if args_cli.out else
               os.path.join(PROJECT_ROOT, "media", "instances", config.MACHINE, INST.state))
    written = []
    for tag, poses in arrangements.items():
        for item_id, pose in poses.items():
            scene[item_id].write_root_pose_to_sim_index(root_pose=torch.tensor(
                np.asarray(pose, dtype=np.float32)[None], device=device))
            scene[item_id].write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
        step(args_cli.settle_steps)
        rig.update(dt)
        written += rig.save_stills(out_dir, f"{INST.name}_{tag}")
        print(f"[INFO] {tag} tableau rendered")

    for p in written:
        print(f"[INFO] wrote {os.path.relpath(p, PROJECT_ROOT)}")
    print(f"[INFO] instance media -> {os.path.relpath(out_dir, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception as exc:  # a crash must still print the line the repo judges runs by
        import traceback

        traceback.print_exc()
        print(f"[RESULT] FAIL ({type(exc).__name__}: {exc})")
        code = 1
    simulation_app.close()
    raise SystemExit(code)
