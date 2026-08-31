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
parser.add_argument("--cell", type=str, default=None,
                    help="Benchmark tier cell (a dishsim.tiers.CELLS key); overrides "
                         "--mode/--displace and adds spun targets, cycles, counter-cap "
                         "compliance and a cap-aware solvability certificate.")
parser.add_argument("--out", type=str, default=os.path.join(PROJECT_ROOT, "results", "instances"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys
from dataclasses import replace as dc_replace

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

# context order contract: machine -> scenario -> placement, all BEFORE scene imports
config.apply_machine("bosch800")
config.apply_scenario(args_cli.state)
config.apply_base_placement("side_winner")

from dishsim import capacity, compat, instance_gen, placement, rearrange, rrt, tiers  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.media import release_sim_for_close  # noqa: E402
from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402

if args_cli.cell is not None and args_cli.cell not in tiers.CELLS:
    raise SystemExit(f"[FAIL] unknown --cell {args_cli.cell!r} (have: {sorted(tiers.CELLS)})")

MAX_INSTANCE_ATTEMPTS = 4  # whole-instance re-rolls on dead-end sampling / unstable settles
TIER_INSTANCE_ATTEMPTS = 12  # cap-1/cap-0 + cycle cells reject more certificates
# ponytail: whole-instance re-roll on a single unstable item; per-item resampling if yield matters


def main() -> int:
    cell = tiers.CELLS[args_cli.cell] if args_cli.cell else None
    if cell:
        plan = capacity.plan_full_load(classes=sorted(cell["classes"]),
                                       per_class_cap=dict(cell["classes"]),
                                       settle_bar=cell["settle_bar"], log=lambda *_: None)
    else:
        plan = capacity.plan_full_load(log=lambda *_: None)
    phase = next((p for p in plan.phases if p.state == args_cli.state and p.items), None)
    if phase is None:
        print(f"[FAIL] the capacity plan places nothing in state {args_cli.state!r}")
        return 1
    roster = phase.items
    classes = sorted({it.object_class for it in roster})
    tables = capacity.load_state_tables(args_cli.state, classes)
    world = rearrange.ArrangementWorld(args_cli.state, classes)
    if cell:
        got: dict = {}
        for it in roster:
            got[it.object_class] = got.get(it.object_class, 0) + 1
        if got != cell["classes"]:
            print(f"[FAIL] cell {args_cli.cell}: plan produced {got}, wanted "
                  f"{cell['classes']} — funnel: {phase.funnel}")
            return 1
        n_displace = tiers.n_displace(args_cli.cell, len(roster))
    else:
        n_displace = args_cli.displace or (len(roster) if args_cli.mode == "random"
                                           else math.ceil(len(roster) / 2))
        n_displace = min(max(1, n_displace), len(roster))
    print(f"[INFO] state {args_cli.state}: roster {len(roster)} items "
          f"({', '.join(classes)}), displacing {n_displace}"
          + (f", cell {args_cli.cell} (cap {cell['counter_cap']})" if cell else ""))

    targets = {}
    for it in roster:
        slot = tables.slots[it.object_class][it.slot_id]
        targets[it.item_id] = {"T_base_obj": np.asarray(it.T_base_obj),
                               "slot": slot.to_json()}

    def spun_targets(rng):
        """Per-object spin-sampled targets, jointly FCL re-certified (roster order).

        Spin-only sampling keeps every target POSITION exactly on the compat lattice
        (``object_pose_for_mode`` spins about the mode axis, position untouched), so
        ``compat._nearest_location`` still resolves every goal; the sequential joint FCL
        pass catches hull asymmetries, with per-item fallback to the certified nominal.
        Returns ``{item_id: {"T_base_obj", "slot"}}`` or ``None`` (re-roll the tableau).
        """
        world.clear()
        out = {}
        for it in roster:
            slot = tables.slots[it.object_class][it.slot_id]
            with config.active_object(it.object_class):
                hover = float(config.placement_mode_params(slot.mode)["release_hover_m"])
                T = None
                for _ in range(20):
                    cand = placement.object_pose_for_mode(
                        slot, float(rng.uniform(0.0, 2.0 * np.pi)),
                        np.zeros(2), np.zeros(2), hover)
                    if not world.move_collides(it.item_id, cand,
                                               object_class=it.object_class):
                        T = cand
                        break
                if T is None:
                    T = np.asarray(it.T_base_obj)  # certified nominal fallback
                    if world.move_collides(it.item_id, T, object_class=it.object_class):
                        world.clear()
                        return None
            world.sync({it.item_id: T}, {it.item_id: it.object_class})
            out[it.item_id] = {"T_base_obj": T, "slot": slot.to_json()}
        world.clear()
        return out

    # ---- Kit scene: roster parked off to the side, one scene for every instance ----------
    obj_specs = [{"name": it.item_id, "usd_path": config.OBJECTS[it.object_class].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0)}
                 for i, it in enumerate(roster)]
    park_w = {s["name"]: s["pos"] for s in obj_specs}
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device))
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    sim.reset()
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    device = scene["dishwasher"].data.joint_pos.device
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
        # project poses are pos + XYZW; isaaclab 2.1 wants pos + WXYZ
        scene[item_id].write_root_pose_to_sim(root_pose=torch.tensor(
            np.concatenate([pos_w, xyzw_to_wxyz(np.asarray(quat))])[None],
            dtype=torch.float32, device=device))
        scene[item_id].write_root_velocity_to_sim(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))

    def measured(item_id):
        return T_base_w @ make_T(scene[item_id].data.root_pos_w[0].cpu().numpy(),
                                 wxyz_to_xyzw(scene[item_id].data.root_quat_w[0].cpu().numpy()))

    step(300)  # racks settle at the scenario extensions

    out_dir = os.path.join(args_cli.out, config.MACHINE, args_cli.state,
                           *( [args_cli.cell] if cell else [] ))
    n_written = 0
    attempts_max = TIER_INSTANCE_ATTEMPTS if cell else MAX_INSTANCE_ATTEMPTS
    cell_table = None  # compat table, built once and re-pointed per instance
    for k in range(args_cli.n):
        name = (f"{args_cli.cell}_s{args_cli.seed + k}" if cell
                else f"{args_cli.mode}_s{args_cli.seed + k}")
        settled = None
        for attempt in range(attempts_max):
            rng = np.random.default_rng(args_cli.seed + k + 1000 * attempt)
            if cell:
                targets_k = spun_targets(rng)
                if targets_k is None:
                    print(f"[WARN] {name}: spun-target tableau infeasible "
                          f"(attempt {attempt}) — re-rolling")
                    continue
                roster_k = [dc_replace(it, T_base_obj=targets_k[it.item_id]["T_base_obj"])
                            for it in roster]
                if cell["cycles"]:
                    forced, cycles_meta = instance_gen.author_cycles(
                        roster_k, cell["cycles"], rng)
                    if forced is None:
                        print(f"[FAIL] cell {args_cli.cell}: no class has enough members "
                              f"for cycles {cell['cycles']} — cell mis-specified")
                        return 1
                else:
                    forced, cycles_meta = {}, []
                initials, displaced = instance_gen.sample_initials(
                    roster_k, tables, world, rng, n_displace,
                    avoid_goal_slots=cell["no_goal_squat"],
                    max_counter=cell["counter_cap"], forced_slots=forced)
            else:
                targets_k, roster_k, cycles_meta = targets, roster, []
                initials, displaced = instance_gen.sample_initials(
                    roster, tables, world, rng, n_displace)
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
                # Reproduction gate: rehearse the runner's episode reset — teleport INTO the
                # settled contact poses and re-settle. A drop-from-hover occasionally wedges
                # a bowl 10-25 deg on a wire; that equilibrium does not survive a cold
                # teleport-into-contact and would abort the episode as init-mismatch. Record
                # the RE-settled poses so the artifact stores the fixed point itself.
                candidate = {it.item_id: hist[it.item_id][-1] for it in roster}
                for it in roster:
                    teleport(it.item_id, T_base=candidate[it.item_id])
                step(rearrange.SETTLE_STEPS_INIT)
                mism = []
                for it in roster:
                    T = measured(it.item_id)
                    dp = float(np.linalg.norm(T[:3, 3] - candidate[it.item_id][:3, 3]))
                    dr = rearrange.rot_angle_deg(candidate[it.item_id], T)
                    if dp > rearrange.INIT_MATCH_POS_M or dr > rearrange.INIT_MATCH_ROT_DEG:
                        mism.append((it.item_id, round(dp * 1e3, 1), round(dr, 1)))
                if not mism:
                    cand = {it.item_id: measured(it.item_id) for it in roster}
                    if cell:
                        # certificate gate 1: the settled initial state respects the cap
                        n_ctr = sum(rearrange.in_counter_band(T) for T in cand.values())
                        if n_ctr > cell["counter_cap"]:
                            print(f"[WARN] {name}: {n_ctr} items on the counter > cap "
                                  f"{cell['counter_cap']} (attempt {attempt}) — re-rolling")
                            continue
                        # certificate gate 2: provably solvable under this cell's cap
                        probe = rearrange.Instance(
                            name=name, machine=config.MACHINE,
                            base_placement=config.BASE_PLACEMENT, state=args_cli.state,
                            items=[{"item_id": it.item_id, "object_class": it.object_class,
                                    "T_base_init": cand[it.item_id],
                                    "target": targets_k[it.item_id]} for it in roster],
                            meta={})
                        n_opt, status, cell_table = compat.optimal_moves(
                            probe, table=cell_table, counter_cap=cell["counter_cap"])
                        if status == "bound":
                            n_opt, status, cell_table = compat.optimal_moves(
                                probe, table=cell_table, counter_cap=cell["counter_cap"],
                                max_seconds=600.0)
                        if status == "unsolvable":
                            print(f"[WARN] {name}: unsolvable at cap "
                                  f"{cell['counter_cap']} (attempt {attempt}) — re-rolling")
                            continue
                        upper_bound = None
                        if status == "bound":
                            # the exact A* ran out of budget: fall back to a CONSTRUCTIVE
                            # proof — an RRT-Connect plan proves solvability under the cap
                            # and its length upper-bounds the optimum
                            for pseed in range(3):
                                upper_bound = rrt.prove_solvable(
                                    probe, world, counter_cap=cell["counter_cap"],
                                    seconds=60.0, seed=pseed)
                                if upper_bound is not None:
                                    break
                            if upper_bound is None:
                                print(f"[WARN] {name}: no optimum AND no constructive "
                                      f"solvability proof (attempt {attempt}) — re-rolling")
                                continue
                    else:
                        n_opt, status, upper_bound = None, None, None
                    settled = cand
                    break
                print(f"[WARN] {name}: non-reproducible settle {mism} (attempt {attempt}) — re-rolling")
                continue
            print(f"[WARN] {name}: unstable initials {bad} (attempt {attempt}) — re-rolling")
        for it in roster:  # re-park for the next instance
            teleport(it.item_id, pos_w=park_w[it.item_id])
        step(30)
        if settled is None:
            print(f"[FAIL] {name}: no accepted initial arrangement in {attempts_max} attempts")
            return 1
        if cell:
            meta = {"mode": "tier", "cell": args_cli.cell, "seed": args_cli.seed + k,
                    "n_displaced": int(len(displaced)), "displace_frac": cell["displace"],
                    "counter_cap": cell["counter_cap"],
                    "no_goal_squat": cell["no_goal_squat"],
                    "classes": dict(cell["classes"]), "cycles": cycles_meta,
                    "settle_bar": cell["settle_bar"],
                    "optimum": n_opt, "optimum_status": status,
                    "upper_bound": upper_bound,
                    "config_hash": plan.config_hash_by_state[args_cli.state]}
        else:
            meta = {"mode": args_cli.mode, "seed": args_cli.seed + k,
                    "n_displaced": int(len(displaced)),
                    "config_hash": plan.config_hash_by_state[args_cli.state]}
        inst = rearrange.Instance(
            name=name, machine=config.MACHINE, base_placement=config.BASE_PLACEMENT,
            state=args_cli.state,
            items=[{"item_id": it.item_id, "object_class": it.object_class,
                    "T_base_init": settled[it.item_id],
                    "target": targets_k[it.item_id]} for it in roster],
            meta=meta)
        path = inst.dump(os.path.join(out_dir, f"{name}.json"))
        n_written += 1
        print(f"[INFO] wrote {os.path.relpath(path, PROJECT_ROOT)} "
              f"(attempt {attempt}, displaced {sorted(displaced)}"
              + (f", optimum {n_opt}" if cell else "") + ")")

    print(f"[INFO] {n_written}/{args_cli.n} instances written to "
          f"{os.path.relpath(out_dir, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    release_sim_for_close()
    simulation_app.close()
    raise SystemExit(code)
