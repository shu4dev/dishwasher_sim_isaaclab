# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Final-state stills for recorded episodes: the LAST image of each attempt.

Reads episode record JSONs (``run_rearrange.py``), teleports each record's roster to its
stored ``final_poses``, settles briefly, and renders the tableau — one Kit session serves
every record in the glob (they must share one machine/placement/state context). The natural
use is visualizing the FAILED episodes: where each algorithm actually left the world when it
aborted. ``init-mismatch`` stubs carry no final poses and are skipped with a note.

    scripts/run_kit.sh scripts/evaluation/episode_final_views.py --headless --enable_cameras \
        --records "results/rearrange/bosch800/placement/*/*.json" --failed_only
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Render episode final states from records.")
parser.add_argument("--records", type=str, required=True, help="Episode-record JSON glob.")
parser.add_argument("--failed_only", action="store_true",
                    help="Render only unsolved episodes (the default use case).")
parser.add_argument("--out", type=str, default=None,
                    help="Media dir (default: media/rearrange/<machine>/<state>/finals).")
parser.add_argument("--settle_steps", type=int, default=90,
                    help="Physics steps to re-seat each final tableau before its stills.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from dishsim import config  # noqa: E402

_pattern = (args_cli.records if os.path.isabs(args_cli.records)
            else os.path.join(PROJECT_ROOT, args_cli.records))
_paths = sorted(glob.glob(_pattern))
if not _paths:
    raise SystemExit(f"[FAIL] no records match {_pattern}")
_recs = []
for p in _paths:
    r = json.load(open(p))
    if args_cli.failed_only and r.get("solved"):
        continue
    if not r.get("final_poses"):
        print(f"[INFO] skipping {os.path.basename(p)}: no final_poses "
              f"(abort={r.get('abort')})")
        continue
    _recs.append(r)
if not _recs:
    raise SystemExit("[FAIL] nothing to render after filtering")
_ctx = {(r["machine"], r["state"]) for r in _recs}
if len(_ctx) != 1:
    raise SystemExit(f"[FAIL] one Kit session serves one (machine, state); got {sorted(_ctx)}")
MACHINE, STATE = next(iter(_ctx))


def _instance_path(rec):
    cell = (rec.get("instance_meta") or {}).get("cell")
    base = os.path.join(PROJECT_ROOT, "results", "instances", MACHINE, STATE)
    return os.path.join(base, cell, f"{rec['instance']}.json") if cell \
        else os.path.join(base, f"{rec['instance']}.json")


_inst0 = json.load(open(_instance_path(_recs[0])))
# context order contract: machine -> scenario -> placement, all BEFORE scene imports
if MACHINE != config.MACHINE_BASELINE_NAME:
    config.apply_machine(MACHINE)
config.apply_scenario(STATE)
config.apply_base_placement(_inst0["base_placement"])

from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, release_sim_for_close  # noqa: E402
from dishsim.quats import xyzw_to_wxyz  # noqa: E402
from dishsim.transforms import T_to_pos_quat, make_T  # noqa: E402

WARMUP_STEPS = 300  # racks settle at the state's extensions before anything arrives


def main() -> int:
    # union pool over every record's roster (item ids are <class>_<nn>, shared across cells)
    pool: dict = {}
    for r in _recs:
        for item_id in r["final_poses"]:
            pool[item_id] = item_id.rsplit("_", 1)[0]
    obj_specs = [{"name": item_id, "usd_path": config.OBJECTS[cls].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0), "color": config.item_color(item_id)}
                 for i, (item_id, cls) in enumerate(sorted(pool.items()))]
    park_w = {s["name"]: s["pos"] for s in obj_specs}

    sim = SimulationContext(sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device))
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    ep_cam = config.EPISODE_CAMERA
    rig = CameraRig({**config.CAMERAS,
                     "episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                                 dict(ep_cam["lens"]))}, hw=config.CAMERA_HW)
    sim.reset()
    rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    device = scene[obj_specs[0]["name"]].data.root_pos_w.device
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)

    def step(n):
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    def put(item_id, T_base=None, pos_w=None):
        if pos_w is None:
            pos, quat = T_to_pos_quat(T_w_base @ np.asarray(T_base, dtype=float))
        else:
            pos, quat = np.asarray(pos_w, dtype=float), (0.0, 0.0, 0.0, 1.0)
        pose = np.concatenate([pos, xyzw_to_wxyz(np.asarray(quat))])
        scene[item_id].write_root_pose_to_sim(root_pose=torch.tensor(
            pose[None], dtype=torch.float32, device=device))
        scene[item_id].write_root_velocity_to_sim(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))

    step(WARMUP_STEPS)

    out_dir = args_cli.out or os.path.join(PROJECT_ROOT, "media", "rearrange",
                                           MACHINE, STATE, "finals")
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for r in _recs:
        for item_id in pool:  # park everything, then place this record's roster
            put(item_id, pos_w=park_w[item_id])
        step(30)
        for item_id, T in r["final_poses"].items():
            put(item_id, T_base=np.asarray(T))
        step(args_cli.settle_steps)
        rig.update(dt)
        tag = f"{r['instance']}__{r['algorithm']}_final"
        written += rig.save_stills(out_dir, tag)
        print(f"[INFO] {tag}: solved={r.get('solved')} abort={r.get('abort')} "
              f"at_goal={r.get('at_goal_final')}/{r.get('n_items')}")

    for p in written:
        print(f"[INFO] wrote {os.path.relpath(p, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    release_sim_for_close()
    simulation_app.close()
    raise SystemExit(code)
