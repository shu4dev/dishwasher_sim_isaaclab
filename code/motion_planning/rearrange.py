# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The closed-loop episode driver of the merged benchmark (code/paths/episodes.py, stage run): instances, moves, the greedy
baseline.

A move teleports one object to any commanded pose, in the racks or on the counter (just physical space). Episodes
ABORT on the first fatal fault (disturbed, unstable settle) and play on through refused commands and failed settles.

Kit-free by construction: :func:`run_episode` is parameterized by an ``oracle`` (executes a move, returns settled
poses + fault, judges at-goal; Isaac in ``code/sampler/scripts/frigidaire_bench_kit.py``, a toy in
``code/motion_planning/tests/test_rearrange.py``) and a ``world`` (FCL feasibility and the counter band, e.g. the Frigidaire
``planner.PlannerWorld``).
"""

import json
import os
import time
from dataclasses import dataclass

import numpy as np

# Fault/verdict knobs: the measured settle numbers of the retired capacity_fill/reveal campaigns (git history), not
# eyeballed; the Frigidaire bench oracle and the robot block read them. Module constants on purpose.
STABLE_POS_M = 0.005        # moved item must stop drifting (measured per-item gate)
STABLE_ROT_DEG = 3.0
MOVE_DEV_MAX_M = 0.06       # settled vs commanded pose: a planner replans past this (the move did not take)
DISTURB_POS_M = 0.010       # a NON-moved item beyond this from its pre-move pose = displaced
# A bowl on the rack wires rocks when a neighbor lands: measured 15.9 deg pure axis tilt at
# 3.6 mm translation, re-seating WITHOUT leaving its goal (first Kit batch, perturbed_s2).
# 20 deg passes that benign re-seat; a genuine knock-off also trips the 10 mm position gate.
DISTURB_ROT_DEG = 20.0
INIT_MATCH_POS_M = 0.010    # episode reset must reproduce the instance's settled initials
# Cross-session re-settle of a near-flat bowl measured 10.5 deg / 6.9 mm (first Kit batch,
# perturbed_s1) — the same tine re-seat physics; 15 deg clears it, position gate unchanged.
INIT_MATCH_ROT_DEG = 15.0
MAX_CONSEC_REFUSALS = 25    # abort "refusal-loop" after this many straight refused commands
# Same rationale as the refusal-loop, one level down: with no move budget a reactive planner
# that re-commands a move physics keeps rejecting (teleport-back on every settle) would spin
# forever — each retry costs settle steps but ~ms of planning time, so the planning budget
# never fires. Counts EXECUTED moves whose settle failed, back to back; any clean settle resets.
MAX_CONSEC_FAILED_SETTLES = 25  # abort "settle-loop"


def _no_counter(T_base_obj) -> bool:
    """A world without a counter band (``world.in_counter`` absent): nothing is ever on the counter."""
    return False


def rot_angle_deg(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """Rotation angle [deg] between two homogeneous transforms."""
    R = np.asarray(T_a)[:3, :3].T @ np.asarray(T_b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


@dataclass(frozen=True)
class Move:
    """One teleport: put ``item_id`` at ``T_base_obj`` (commanded pose, base frame)."""

    item_id: str
    T_base_obj: np.ndarray


@dataclass
class Instance:
    """One rearrangement problem, loaded from / dumped to a JSON artifact.

    ``items`` entries: ``{"item_id", "object_class", "T_base_init" (np [4,4], measured
    settled), "target": {"T_base_obj" (np [4,4], the commanded goal pose)}}``.
    """

    name: str
    machine: str
    base_placement: str
    state: str
    items: list
    meta: dict

    @staticmethod
    def load(path: str) -> "Instance":
        with open(path) as f:
            doc = json.load(f)
        for it in doc["items"]:
            it["T_base_init"] = np.array(it["T_base_init"])
            it["target"]["T_base_obj"] = np.array(it["target"]["T_base_obj"])
        return Instance(name=doc["name"], machine=doc["machine"],
                        base_placement=doc["base_placement"], state=doc["state"],
                        items=doc["items"], meta=doc.get("meta", {}))

    def dump(self, path: str) -> str:
        items = []
        for it in self.items:
            items.append({**it, "T_base_init": np.asarray(it["T_base_init"]).tolist(),
                          "target": {**it["target"],
                                     "T_base_obj": np.asarray(it["target"]["T_base_obj"]).tolist()}})
        doc = {"schema_version": 1, "name": self.name, "machine": self.machine,
               "base_placement": self.base_placement, "state": self.state,
               "meta": self.meta, "items": items}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(doc, f, indent=1)
        os.replace(tmp, path)
        return path

    def item(self, item_id: str) -> dict:
        return next(it for it in self.items if it["item_id"] == item_id)


def disturbed_items(pre: dict, poses: dict, moved_id: str) -> list:
    """Items other than ``moved_id`` that left their pre-move pose beyond the DISTURB_* gates."""
    return [k for k in poses if k != moved_id and (
        float(np.linalg.norm(poses[k][:3, 3] - pre[k][:3, 3])) > DISTURB_POS_M
        or rot_angle_deg(pre[k], poses[k]) > DISTURB_ROT_DEG)]


def _move_kind(instance: Instance, move: "Move") -> str:
    """``"goal"`` if the move commands the item's own target pose, else ``"buffer"``.

    Buffer trips are what separate a monotone plan from a non-monotone one, so the record has
    to label them rather than leave an aggregator to re-derive them by matching poses.
    """
    for it in instance.items:
        if it["item_id"] == move.item_id:
            T_goal = np.asarray(it["target"]["T_base_obj"], dtype=float)
            return "goal" if np.allclose(T_goal, np.asarray(move.T_base_obj, dtype=float),
                                         atol=1e-6) else "buffer"
    return "buffer"


def run_episode(instance: Instance, algo, world, oracle, budget: int | None,
                algorithm_name: str = "", time_budget_s: float | None = None,
                counter_cap: int | None = None) -> dict:
    """Drive one closed-loop episode; abort on the first fatal fault; return the record.

    ``oracle.execute(move) -> (poses, fault, move_info)`` — settled base-frame poses for the
    whole roster, ``fault in (None, "failed-settle", "unstable-settle", "disturbed")``, and
    a per-move info dict (e.g. settle_dev_mm, disturbed ids). ``"failed-settle"`` is the one
    NON-fatal fault: the oracle has already returned the item to its pre-move pose
    (teleport-back), the move counts, and the episode continues — algorithms may retry.
    ``oracle.at_goal(item, T) -> bool``.

    An INFEASIBLE commanded move (one the FCL mirror rejects) is refused and logged, and the
    episode continues: every algorithm can pre-check with the same oracle, so emitting one is
    a search error worth MEASURING (``infeasible_commands``), not a reason to destroy the
    episode. Ending it there would systematically flatter planners that pre-check — the
    baseline does — over sampling planners that do not. A move INTO the counter band while
    ``counter_cap`` items already occupy it is refused the same way (kind ``"counter-full"``,
    counted separately). ``MAX_CONSEC_REFUSALS`` straight refusals abort ``"refusal-loop"``
    (refusals cost no budget, so a deterministic replanner would otherwise spin forever).

    Args:
        budget: Maximum executed moves, or ``None`` for unlimited.
        time_budget_s: Planning-time budget [s] for the whole episode, or ``None`` for
            unlimited. Counts ONLY time inside ``algo.next_move`` — physics and the harness's
            own feasibility checks are the simulator's cost, not the planner's.
        counter_cap: Max simultaneous items inside the counter band, or ``None`` for no cap.
    """
    classes = {it["item_id"]: it["object_class"] for it in instance.items}
    poses = {it["item_id"]: np.asarray(it["T_base_init"]) for it in instance.items}
    world.sync(poses, classes)
    planned_s = 0.0
    in_ctr = getattr(world, "in_counter", _no_counter)

    def observe(moves_used):
        return {"items": {it["item_id"]: {"object_class": it["object_class"],
                                          "T_base_obj": poses[it["item_id"]],
                                          "at_goal": oracle.at_goal(it, poses[it["item_id"]])}
                          for it in instance.items},
                "moves_used": moves_used, "budget": budget,
                "time_budget_s": time_budget_s,
                "time_left_s": (None if time_budget_s is None
                                else max(0.0, time_budget_s - planned_s)),
                "counter_cap": counter_cap,
                "counter_count": int(sum(in_ctr(T) for T in poses.values()))}

    obs = observe(0)
    at_goal_initial = sum(v["at_goal"] for v in obs["items"].values())
    algo.reset(instance, world)

    solved, abort = False, None
    planning_time_s, move_log = [], []
    moves_used = infeasible_commands = counter_full_refusals = failed_settles = 0
    consec_refusals = consec_failed_settles = 0
    while True:
        if all(v["at_goal"] for v in obs["items"].values()):
            solved = True
            break
        t0 = time.perf_counter()
        move = algo.next_move(obs)
        dt_plan = time.perf_counter() - t0
        planned_s += dt_plan
        planning_time_s.append(round(dt_plan, 6))
        if move is None:
            abort = "give-up"
            break
        if time_budget_s is not None and planned_s > time_budget_s:
            abort = "time-budget"
            break
        if budget is not None and moves_used >= budget:
            abort = "budget"
            break
        if (counter_cap is not None and in_ctr(move.T_base_obj)
                and not in_ctr(poses[move.item_id])  # counter->counter keeps occupancy
                and obs["counter_count"] >= counter_cap):
            counter_full_refusals += 1
            consec_refusals += 1
            move_log.append({"item_id": move.item_id,
                             "T_base_obj": np.asarray(move.T_base_obj).tolist(),
                             "kind": "counter-full"})
            if consec_refusals >= MAX_CONSEC_REFUSALS:
                abort = "refusal-loop"
                break
            obs = observe(moves_used)
            continue
        # A world may keep a stricter planning predicate (e.g. a support rule) out of the harness
        # check, so moving a supporting dish reaches the oracle's fatal verdict instead of a refusal.
        if getattr(world, "harness_collides", world.move_collides)(move.item_id, move.T_base_obj):
            # refused, not fatal — see the docstring. The algorithm keeps its turn and its
            # clock; only the world is unchanged.
            infeasible_commands += 1
            consec_refusals += 1
            move_log.append({"item_id": move.item_id,
                             "T_base_obj": np.asarray(move.T_base_obj).tolist(),
                             "kind": "infeasible",
                             "blockers": world.blockers(move.item_id, move.T_base_obj)})
            if consec_refusals >= MAX_CONSEC_REFUSALS:
                abort = "refusal-loop"
                break
            obs = observe(moves_used)
            continue
        consec_refusals = 0
        T_from = np.asarray(poses[move.item_id]).tolist()  # travel distance needs the origin
        poses, fault, info = oracle.execute(move)
        poses = {k: np.asarray(v) for k, v in poses.items()}
        moves_used += 1
        move_log.append({"item_id": move.item_id,
                         "T_base_obj": np.asarray(move.T_base_obj).tolist(),
                         "T_base_from": T_from, "kind": _move_kind(instance, move), **info})
        world.sync(poses, classes)
        if fault == "failed-settle":
            # NON-fatal: the oracle already put the item back; the move counts, play on
            failed_settles += 1
            consec_failed_settles += 1
            move_log[-1]["kind"] = "failed-settle"
            if consec_failed_settles >= MAX_CONSEC_FAILED_SETTLES:
                abort = "settle-loop"
                break
            obs = observe(moves_used)
            continue
        if fault is not None:
            abort = fault
            break
        consec_failed_settles = 0
        obs = observe(moves_used)

    final = {it["item_id"]: oracle.at_goal(it, poses[it["item_id"]]) for it in instance.items}
    n = len(instance.items)
    return {
        "instance": instance.name, "algorithm": algorithm_name,
        "machine": instance.machine, "state": instance.state,
        "solved": bool(solved), "abort": abort,
        "n_items": n, "at_goal_initial": int(at_goal_initial),
        "at_goal_final": int(sum(final.values())),
        "fraction_at_goal": round(sum(final.values()) / n, 4) if n else 1.0,
        "moves_used": moves_used, "budget": budget,
        "time_budget_s": time_budget_s,
        "planning_time_s": planning_time_s,
        "planning_time_total_s": round(planned_s, 6),
        "infeasible_commands": infeasible_commands,
        "counter_cap": counter_cap,
        "counter_full_refusals": counter_full_refusals,
        "failed_settles": failed_settles,
        "buffer_moves": sum(m.get("kind") == "buffer" for m in move_log),
        "goal_moves": sum(m.get("kind") == "goal" for m in move_log),
        "travel_m": round(sum(
            float(np.linalg.norm(np.asarray(m["T_base_obj"])[:3, 3]
                                 - np.asarray(m["T_base_from"])[:3, 3]))
            for m in move_log if "T_base_from" in m), 6),
        "world_queries": getattr(world, "n_queries", None),
        "algo_stats": algo.stats() if hasattr(algo, "stats") else {},
        "instance_meta": dict(instance.meta),
        "moves": move_log,
        "final_poses": {k: np.asarray(v).tolist() for k, v in poses.items()},
    }


class Greedy:
    """Baseline: send home whatever fits; relocate one blocker to the buffer when stuck.

    # ponytail: one-blocker lookahead; swap-cycles beyond one relocation need a real planner
    """

    def reset(self, instance: Instance, world) -> None:
        self.instance, self.world = instance, world
        self._relocated: set = set()

    def next_move(self, obs):
        items = obs["items"]
        # pass 1: any misplaced item whose target release pose is free -> send it home
        for it in self.instance.items:
            item_id = it["item_id"]
            if items[item_id]["at_goal"]:
                self._relocated.discard(item_id)
                continue
            T_target = it["target"]["T_base_obj"]
            if not self.world.move_collides(item_id, T_target):
                return Move(item_id, T_target)
        # pass 2: relocate the first eligible blocker of a misplaced item to a buffer spot.
        # Under a counter cap, a full band means this baseline has no legal park — give up
        # cleanly instead of spamming refused commands into the refusal-loop guard.
        cap = obs.get("counter_cap")
        if cap is not None and obs.get("counter_count", 0) >= cap:
            return None
        for it in self.instance.items:
            item_id = it["item_id"]
            if items[item_id]["at_goal"]:
                continue
            blockers = self.world.blockers(item_id, it["target"]["T_base_obj"])
            if any(b not in items for b in blockers):
                continue  # static-blocked target — nothing a relocation can fix
            for b in blockers:
                if items[b]["at_goal"] or b in self._relocated:
                    continue
                for T_buf in self.world.buffer_poses(items[b]["object_class"]):
                    if not self.world.move_collides(b, T_buf):
                        self._relocated.add(b)
                        return Move(b, T_buf)
        return None  # give up: static-blocked, blockers spent their buffer trip, or no space


class OfflineGreedy:
    """Greedy's rule run to completion up front against the mirror, then replayed blind.

    The offline counterpart of :class:`Greedy`, mirroring the RRT family's contract: ALL
    thinking happens inside the first :meth:`next_move` call, against the noise-free FCL
    mirror where a commanded move lands exactly and at-goal is plan bookkeeping, not a
    physics judgement. Execution replays the finite plan one move per call, so an episode
    is bounded by plan length — the reactive rule's physics-noise churn (re-fixing items
    the settle keeps nudging, unbounded under no move budget) is impossible by
    construction. The price is symmetric honesty: when physics disagrees with the plan the
    replay cannot adapt, so divergence surfaces as ``disturbed``/give-up instead of being
    quietly repaired. Replans only when the previous command visibly did not take
    (settled > ``MOVE_DEV_MAX_M`` from commanded — the same rule RRT uses), which also
    covers refused commands.
    """

    def reset(self, instance: Instance, world) -> None:
        self.instance, self.world = instance, world
        self.plan: list | None = None
        self._last: Move | None = None
        self._stats = {"planner": "greedy_offline", "plans": 0, "plan_moves": 0}

    def _replan(self, obs) -> list:
        inner = Greedy()
        inner.reset(self.instance, self.world)
        classes = {i: v["object_class"] for i, v in obs["items"].items()}
        real = {i: np.asarray(v["T_base_obj"]) for i, v in obs["items"].items()}
        poses = dict(real)
        at_goal = {i: bool(v["at_goal"]) for i, v in obs["items"].items()}
        targets = {it["item_id"]: np.asarray(it["target"]["T_base_obj"])
                   for it in self.instance.items}
        in_ctr = getattr(self.world, "in_counter", _no_counter)
        cap = obs.get("counter_cap")
        left = obs.get("time_left_s")
        deadline = time.perf_counter() + (30.0 if left is None else left * 0.95)
        backstop = 10 * max(1, len(poses))  # the model solves in <= ~2n; anything more is a bug
        plan: list = []
        try:
            while (not all(at_goal.values()) and len(plan) < backstop
                   and time.perf_counter() < deadline):
                model_obs = {"items": {i: {"object_class": classes[i], "T_base_obj": poses[i],
                                           "at_goal": at_goal[i]} for i in poses},
                             "counter_cap": cap,
                             "counter_count": int(sum(in_ctr(T) for T in poses.values()))}
                mv = inner.next_move(model_obs)
                if mv is None:
                    break
                plan.append(mv)
                poses[mv.item_id] = np.asarray(mv.T_base_obj)
                at_goal[mv.item_id] = bool(np.allclose(poses[mv.item_id],
                                                       targets[mv.item_id]))
                self.world.sync(poses, classes)  # the rule reads the mirror, keep it modeled
        finally:
            self.world.sync(real, classes)  # hand the mirror back at observed reality
        self._stats["plans"] += 1
        self._stats["plan_moves"] += len(plan)
        return plan

    def next_move(self, obs):
        if self._last is not None and self.plan is not None:
            observed = np.asarray(obs["items"][self._last.item_id]["T_base_obj"])
            commanded = np.asarray(self._last.T_base_obj)
            if np.linalg.norm(observed[:3, 3] - commanded[:3, 3]) > MOVE_DEV_MAX_M:
                self.plan = None  # the move visibly did not take: plan state is fiction
        if self.plan is None:
            self.plan = self._replan(obs)
        if not self.plan:
            return None  # plan exhausted (or empty): nothing left this rule can do
        self._last = self.plan.pop(0)
        return self._last
