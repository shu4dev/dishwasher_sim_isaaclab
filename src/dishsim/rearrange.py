# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The rearrangement benchmark: instances, the closed-loop episode driver, the greedy baseline.

A problem instance fixes a machine rack state and gives every object a settled INITIAL pose
and an exact TARGET pose (the capacity plan's certified release-hover pose, carrying its
SlotFrame so the at-goal verdict reuses :func:`placement.evaluate_placement`). A move
teleports one object to any commanded pose — in-machine or on the counter buffer band, which
is just physical space. Episodes ABORT on the first fault (colliding command, unstable
settle, disturbed neighbor) or at the move budget.

Kit-free by construction: :func:`run_episode` is parameterized by an ``oracle`` (executes a
move, returns settled poses + fault — Isaac in ``scripts/experiment/run_rearrange.py``, a toy
in ``tests/test_rearrange.py``) and a ``world`` (FCL feasibility, :class:`ArrangementWorld`).

Context discipline: the CALLER applies machine -> object -> scenario -> placement before
using anything here (the instance records which context it was generated under).
"""

import json
import os
import time
from dataclasses import dataclass

import numpy as np

from . import config, placement
from .collision_world import CollisionWorld, load_object_pieces
from .transforms import T_inv, make_T

# Fault/verdict knobs. Defaults are the measured settle numbers from the retired
# capacity_fill/reveal campaigns (git history), not eyeballed. Module constants on purpose:
# nothing here may touch config.py (geometry.config_hash must stay byte-identical).
SETTLE_STEPS_MOVE = 75      # physics steps settled after every move
SETTLE_STEPS_INIT = 150     # physics steps settled at episode reset
DRIFT_WINDOW = 30           # stability window: pose drift measured over the last N steps
STABLE_POS_M = 0.005        # moved item must stop drifting (measured per-item gate)
STABLE_ROT_DEG = 3.0
MOVE_DEV_MAX_M = 0.06       # settled vs commanded pose (absorbs the hover drop + roll-to-tine)
DISTURB_POS_M = 0.010       # any NON-moved item beyond this from its pre-move pose = fault
# A bowl on the rack wires rocks when a neighbor lands: measured 15.9 deg pure axis tilt at
# 3.6 mm translation, re-seating WITHOUT leaving its goal (first Kit batch, perturbed_s2).
# 20 deg passes that benign re-seat; a genuine knock-off also trips the 10 mm position gate.
DISTURB_ROT_DEG = 20.0
INIT_MATCH_POS_M = 0.010    # episode reset must reproduce the instance's settled initials
# Cross-session re-settle of a near-flat bowl measured 10.5 deg / 6.9 mm (first Kit batch,
# perturbed_s1) — the same tine re-seat physics; 15 deg clears it, position gate unchanged.
INIT_MATCH_ROT_DEG = 15.0
COUNTER_GRID_PITCH_M = 0.11  # buffer candidate spacing; FCL gates actual use
# Extra buffer hover above the release hover: the counter's CoACD hull tops ~8 mm above the
# raw slab (decomposition slop on the large E_body_5 body), which eats the 12 mm release
# hover to under the candidate-inflation margin and FCL-blocks every buffer cell without it.
BUFFER_EXTRA_HOVER_M = 0.010
MAX_CONSEC_REFUSALS = 25    # abort "refusal-loop" after this many straight refused commands
# Same rationale as the refusal-loop, one level down: with no move budget a reactive planner
# that re-commands a move physics keeps rejecting (teleport-back on every settle) would spin
# forever — each retry costs 75 settle steps but ~ms of planning time, so the planning budget
# never fires. Counts EXECUTED moves whose settle failed, back to back; any clean settle resets.
MAX_CONSEC_FAILED_SETTLES = 25  # abort "settle-loop"


def rot_angle_deg(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """Rotation angle [deg] between two homogeneous transforms."""
    R = np.asarray(T_a)[:3, :3].T @ np.asarray(T_b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def in_counter_band(T_base_obj) -> bool:
    """Is this base-frame pose inside the counter staging band?

    World x/y inside ``config.TASK["spawn_rect_w"]`` and object center above the counter
    top plane. The extended rack's footprint ends below/behind the band on every machine,
    so no in-machine pose ever tests in-band.
    """
    T_w = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W) @ np.asarray(T_base_obj)
    x, y, z = T_w[:3, 3]
    r = config.TASK["spawn_rect_w"]
    top_z = config.COUNTERTOP_CENTER_W[2] + config.COUNTERTOP_SIZE[2] / 2.0
    return bool(r["x_min"] <= x <= r["x_max"] and r["y_min"] <= y <= r["y_max"] and z > top_z)


@dataclass(frozen=True)
class Move:
    """One teleport: put ``item_id`` at ``T_base_obj`` (commanded pose, base frame)."""

    item_id: str
    T_base_obj: np.ndarray


@dataclass
class Instance:
    """One rearrangement problem, loaded from / dumped to a JSON artifact.

    ``items`` entries: ``{"item_id", "object_class", "T_base_init" (np [4,4], measured
    settled), "target": {"T_base_obj" (np [4,4], certified release-hover pose),
    "slot": SlotFrame.to_json()}}``.
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


def at_goal(item: dict, T_base_obj: np.ndarray) -> bool:
    """Is this item's SETTLED pose inside its placement mode's tolerances at its target slot?"""
    slot = placement.SlotFrame.from_json(item["target"]["slot"])
    with config.active_object(item["object_class"]):
        return bool(placement.evaluate_placement(slot, np.asarray(T_base_obj))["ok"])


class ArrangementWorld:
    """The shared FCL mirror of the current arrangement (harness pre-check + algorithms).

    One :class:`CollisionWorld` per session: the statics of every per-class cache of one
    state are identical, so the first class's cache serves them all; each class's convex
    pieces load once.
    """

    def __init__(self, state: str, classes: list):
        classes = sorted(set(classes))
        with config.active_object(classes[0]):
            self.world = CollisionWorld(
                cache_dir=config.scenario_cache_dir(state, object_name=classes[0]))
        self.pieces = {}
        for cls in classes:
            with config.active_object(cls):
                self.pieces[cls] = load_object_pieces(
                    config.scenario_cache_dir(state, object_name=cls))
        self._poses: dict = {}
        self._cls: dict = {}
        #: Feasibility queries served, so a benchmark can separate an algorithmic win from a
        #: planner that simply asked fewer questions.
        self.n_queries = 0

    def snapshot(self) -> dict:
        """Copy of the mirrored arrangement, for cheap search rollback.

        ``sync`` stores pose arrays BY REFERENCE, so the copy has to be deep in the arrays or
        a later in-place write would corrupt the snapshot.
        """
        return {"poses": {k: np.array(v) for k, v in self._poses.items()},
                "classes": dict(self._cls)}

    def restore(self, snap: dict) -> None:
        """Return the mirror to a :meth:`snapshot` state, dropping anything added since."""
        for item_id in list(self._poses):
            if item_id not in snap["poses"]:
                self.world.remove_object(item_id)
                self._poses.pop(item_id, None)
                self._cls.pop(item_id, None)
        self.sync(snap["poses"], snap["classes"])

    def clear(self) -> None:
        """Drop every mirrored item (a generator starts each instance from the empty machine)."""
        for item_id in list(self._poses):
            self.world.remove_object(item_id)
        self._poses.clear()
        self._cls.clear()

    def sync(self, poses: dict, classes: dict) -> None:
        """Mirror measured settled poses (``{item_id: T_base_obj}``) into the FCL world."""
        for item_id, T in poses.items():
            T = np.asarray(T)
            self._cls[item_id] = classes[item_id]
            if self.world.has_object(item_id):
                self.world.set_object_pose(item_id, T)
            else:
                self.world.add_object(item_id, self.pieces[classes[item_id]], T)
            self._poses[item_id] = T

    def move_collides(self, item_id: str, T: np.ndarray, object_class: str = None) -> bool:
        """Would ``item_id`` at commanded ``T`` interpenetrate statics or any OTHER item?

        Takes the bool-only path deliberately: routing through :meth:`blockers` would ask for
        pairs and so forfeit ``object_in_collision``'s early exit on the first colliding piece
        — measured 427 ms vs 4 ms on a rejecting query, which is the difference between a
        search budget that buys hundreds of checks and one that buys hundreds of thousands.
        """
        cls = self._cls.get(item_id, object_class)
        had = self.world.has_object(item_id)
        if had:
            self.world.remove_object(item_id)
        try:
            self.n_queries += 1
            return bool(self.world.object_in_collision(self.pieces[cls], np.asarray(T)))
        finally:
            if had:
                self.world.add_object(item_id, self.pieces[cls], self._poses[item_id])

    def blockers(self, item_id: str, T: np.ndarray, object_class: str = None) -> list:
        """Names blocking ``item_id`` at ``T``: item ids and/or static body names.

        ``object_class`` is only needed for an item the world has never seen (a generator
        probing candidates before committing them).
        """
        cls = self._cls.get(item_id, object_class)
        had = self.world.has_object(item_id)
        if had:
            self.world.remove_object(item_id)
        try:
            self.n_queries += 1
            _, pairs = self.world.object_in_collision(
                self.pieces[cls], np.asarray(T), return_pairs=True)
        finally:
            if had:
                self.world.add_object(item_id, self.pieces[cls], self._poses[item_id])
        return sorted({partner for _, partner in pairs})

    def buffer_poses(self, object_class: str) -> list:
        """Candidate release poses on the counter buffer band (base frame), grid order.

        World-identity orientation (Bosch classes stage as authored), bottom at the counter
        top + release hover; FCL decides which cells are actually free.
        """
        rect = config.TASK["spawn_rect_w"]
        spec = config.OBJECTS[object_class]
        top_z = config.COUNTERTOP_CENTER_W[2] + config.COUNTERTOP_SIZE[2] / 2.0
        z = top_z + spec.bbox_half[2] + config.RELEASE_HOVER_M + BUFFER_EXTRA_HOVER_M
        T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
        out = []
        for x in np.arange(rect["x_min"] + 0.05, rect["x_max"] - 0.05 + 1e-9, COUNTER_GRID_PITCH_M):
            for y in np.arange(rect["y_min"] + 0.05, rect["y_max"] - 0.05 + 1e-9, COUNTER_GRID_PITCH_M):
                T_w = np.eye(4)
                T_w[:3, 3] = (x, y, z)
                out.append(T_base_w @ T_w)
        return out

    def in_counter(self, T_base_obj) -> bool:
        """Counter-band membership for a base-frame pose (see :func:`in_counter_band`)."""
        return in_counter_band(T_base_obj)


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
    in_ctr = getattr(world, "in_counter", in_counter_band)

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
        if world.move_collides(move.item_id, move.T_base_obj):
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
        in_ctr = getattr(self.world, "in_counter", in_counter_band)
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
