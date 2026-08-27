# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate rearrangement-benchmark instances: sampled initial arrangements, physically settled.

Targets come verbatim from the deterministic capacity plan for the state (certified
release-hover poses + their SlotFrames). Initial arrangements displace a seeded subset of the
roster into OTHER placeable slots or the counter buffer band (``--mode random`` displaces
everything — the two generators share one sampling routine), commit candidates first-FCL-free,
then teleport + settle in Isaac and record the MEASURED settled poses as the instance's
initial state. Instances are saved artifacts: every algorithm runs on byte-identical inputs.

Run: scripts/run_kit.sh scripts/setup/gen_instances.py --headless \
         --mode perturbed --state placement --n 10 --seed 0
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import math
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Generate settled rearrangement instances.")
parser.add_argument("--mode", choices=("perturbed", "random"), default="perturbed")
parser.add_argument("--state", type=str, default="placement")
parser.add_argument("--n", type=int, default=10)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--displace", type=int, default=None,
                    help="Items displaced from their targets (default: all for random, "
                         "ceil(n_items/2) for perturbed).")
parser.add_argument("--out", type=str, default=os.path.join(PROJECT_ROOT, "results", "instances"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

# context order contract: machine -> scenario -> placement, all BEFORE scene imports
config.apply_machine("bosch800")
config.apply_scenario(args_cli.state)
config.apply_base_placement("side_winner")

from dishsim import capacity, rearrange  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402

MAX_INSTANCE_ATTEMPTS = 4  # whole-instance re-rolls on dead-end sampling / unstable settles
# ponytail: whole-instance re-roll on a single unstable item; per-item resampling if yield matters


def sample_initials(roster, tables, world, rng, n_displace):
    """Commanded initial pose per item: displaced ones first-FCL-free into other placeable
    slots or the buffer band; the rest at their own targets. None on a dead end."""
    world.clear()
    displaced = {str(s) for s in
                 rng.choice([it.item_id for it in roster], size=n_displace, replace=False)}
    initials = {}
    for it in roster:
        if it.item_id not in displaced:
            T = np.asarray(it.T_base_obj)
        else:
            cls = it.object_class
            with config.active_object(cls):
                slot_poses = [capacity._nominal_release_pose(tables.slots[cls][sid])
                              for sid, ok in tables.placeable[cls].items()
                              if ok and sid != it.slot_id]
            candidates = slot_poses + world.buffer_poses(cls)
            # Generator.shuffle silently shuffles a stacked COPY of a list of arrays —
            # permute indices instead
            candidates = [candidates[j] for j in rng.permutation(len(candidates))]
            T = next((c for c in candidates
                      if not world.move_collides(it.item_id, c, object_class=cls)), None)
            if T is None:
                return None, displaced
        world.sync({it.item_id: T}, {it.item_id: it.object_class})
        initials[it.item_id] = T
    return initials, displaced


def main() -> int:
    plan = capacity.plan_full_load(log=lambda *_: None)
    phase = next((p for p in plan.phases if p.state == args_cli.state and p.items), None)
    if phase is None:
        print(f"[FAIL] the capacity plan places nothing in state {args_cli.state!r}")
        return 1
    roster = phase.items
    classes = sorted({it.object_class for it in roster})
    tables = capacity.load_state_tables(args_cli.state, classes)
    world = rearrange.ArrangementWorld(args_cli.state, classes)
    n_displace = args_cli.displace or (len(roster) if args_cli.mode == "random"
                                       else math.ceil(len(roster) / 2))
    n_displace = min(max(1, n_displace), len(roster))
    print(f"[INFO] state {args_cli.state}: roster {len(roster)} items "
          f"({', '.join(classes)}), displacing {n_displace}")

    targets = {}
    for it in roster:
        slot = tables.slots[it.object_class][it.slot_id]
        targets[it.item_id] = {"T_base_obj": np.asarray(it.T_base_obj),
                               "slot": slot.to_json()}

    # ---- Kit scene: roster parked off to the side, one scene for every instance ----------
    obj_specs = [{"name": it.item_id, "usd_path": config.OBJECTS[it.object_class].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0)}
                 for i, it in enumerate(roster)]
    park_w = {s["name"]: s["pos"] for s in obj_specs}
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg()))
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    sim.reset()
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    device = scene["dishwasher"].data.joint_pos.torch.device
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)

    def step(n):
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    def teleport(item_id, T_base=None, pos_w=None):
        if pos_w is None:
            pos_w, quat = T_to_pos_quat(T_w_base @ np.asarray(T_base))
        else:
            quat = (0.0, 0.0, 0.0, 1.0)
        scene[item_id].write_root_pose_to_sim_index(root_pose=torch.tensor(
            np.concatenate([pos_w, quat])[None], dtype=torch.float32, device=device))
        scene[item_id].write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))

    def measured(item_id):
        return T_base_w @ make_T(scene[item_id].data.root_pos_w.torch[0].cpu().numpy(),
                                 scene[item_id].data.root_quat_w.torch[0].cpu().numpy())

    step(300)  # racks settle at the scenario extensions

    out_dir = os.path.join(args_cli.out, config.MACHINE, args_cli.state)
    n_written = 0
    for k in range(args_cli.n):
        name = f"{args_cli.mode}_s{args_cli.seed + k}"
        settled = None
        for attempt in range(MAX_INSTANCE_ATTEMPTS):
            rng = np.random.default_rng(args_cli.seed + k + 1000 * attempt)
            initials, displaced = sample_initials(roster, tables, world, rng, n_displace)
            if initials is None:
                print(f"[WARN] {name}: sampling dead end (attempt {attempt}) — re-rolling")
                continue
            for it in roster:
                teleport(it.item_id, T_base=initials[it.item_id])
            hist = {it.item_id: [] for it in roster}
            for s in range(config.SETTLE_STEPS):
                step(1)
                if s >= config.SETTLE_STEPS - rearrange.DRIFT_WINDOW:
                    for it in roster:
                        hist[it.item_id].append(measured(it.item_id))
            bad = []
            for it in roster:
                first, last = hist[it.item_id][0], hist[it.item_id][-1]
                drift_p = float(np.linalg.norm(last[:3, 3] - first[:3, 3]))
                drift_deg = rearrange.rot_angle_deg(first, last)
                dev_p = float(np.linalg.norm(last[:3, 3] - np.asarray(initials[it.item_id])[:3, 3]))
                # fell-off detector only — a wedged/tilted initial state is legitimate
                if drift_p > rearrange.STABLE_POS_M or drift_deg > rearrange.STABLE_ROT_DEG or dev_p > 0.10:
                    bad.append((it.item_id, round(drift_p * 1e3, 1), round(dev_p * 1e3, 1)))
            if not bad:
                settled = {it.item_id: hist[it.item_id][-1] for it in roster}
                break
            print(f"[WARN] {name}: unstable initials {bad} (attempt {attempt}) — re-rolling")
        for it in roster:  # re-park for the next instance
            teleport(it.item_id, pos_w=park_w[it.item_id])
        step(30)
        if settled is None:
            print(f"[FAIL] {name}: no stable initial arrangement in {MAX_INSTANCE_ATTEMPTS} attempts")
            return 1
        inst = rearrange.Instance(
            name=name, machine=config.MACHINE, base_placement=config.BASE_PLACEMENT,
            state=args_cli.state,
            items=[{"item_id": it.item_id, "object_class": it.object_class,
                    "T_base_init": settled[it.item_id],
                    "target": targets[it.item_id]} for it in roster],
            meta={"mode": args_cli.mode, "seed": args_cli.seed + k,
                  "n_displaced": int(len(displaced)),
                  "config_hash": plan.config_hash_by_state[args_cli.state]})
        path = inst.dump(os.path.join(out_dir, f"{name}.json"))
        n_written += 1
        print(f"[INFO] wrote {os.path.relpath(path, PROJECT_ROOT)} "
              f"(displaced {sorted(displaced)})")

    print(f"[INFO] {n_written}/{args_cli.n} instances written to "
          f"{os.path.relpath(out_dir, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
