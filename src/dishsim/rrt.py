# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Sampling-based rearrangement planners over ARRANGEMENT space — Kit-free.

Not OMPL: OMPL's RRT family plans in a continuous configuration space with straight-line
steering, which would interpolate every object simultaneously. Here a state is a full
arrangement (one commanded pose per item) and an edge is ONE legal single-object teleport —
the arrangement-RRT formulation of Krontiris & Bekris. The three variants share one tree
core:

- :class:`RRT` — single tree, goal-biased uniform arrangement sampling.
- :class:`RRTConnect` — bidirectional (this IS BiRRT); the goal arrangement is fully
  specified (exact per-item targets), and teleport edges are reversible, so the goal tree's
  edges replay backwards at extraction.
- :class:`RRTStar` — RRT plus choose-parent/rewire on move-count cost within the
  one-move neighborhood.

Harness contract: ``reset(instance, world)`` / ``next_move(obs)``; the search runs inside
the FIRST ``next_move`` call (so it is charged to planning time), caches the move sequence,
and replays it; a move that visibly did not take effect (failed settle) triggers a replan
from the observed arrangement. The planner mutates the FCL mirror to test hypothetical
arrangements and ALWAYS re-syncs it to the observed state before returning — the driver's
own pre-check depends on it. Counter caps are respected during search via
``obs["counter_cap"]`` and the world's band predicate.

Candidate placements per class: the full placeable slot lattice (via
``capacity.load_state_tables``, matching the abstraction the compat optimum certifies) plus
the counter buffer cells; when the state tables are unavailable (toy tests), it falls back
to the roster's target poses plus buffer cells.
"""

import time

import numpy as np

#: search knobs (module constants — never in config.py, so they cannot touch config_hash)
GOAL_BIAS = 0.3           # probability a sample IS the goal arrangement
MAX_ITERS = 200_000       # hard iteration backstop under the time budget
EXTEND_TRIES = 4          # differing items tried per extend before the sample is discarded
TIME_MARGIN_FRAC = 0.05   # fraction of the remaining budget reserved for bookkeeping
POS_KEY_MM = 1.0          # arrangement dedup: positions quantized to this many mm


def _pos(T):
    return np.asarray(T)[:3, 3]


def _same_pose(Ta, Tb, tol=1e-6):
    return bool(np.linalg.norm(_pos(Ta) - _pos(Tb)) <= tol)


class _Node:
    __slots__ = ("poses", "parent", "move", "cost")

    def __init__(self, poses, parent=None, move=None, cost=0):
        self.poses = poses          # {item_id: T (4x4)} — commanded poses
        self.parent = parent
        self.move = move            # (item_id, T) that produced this node
        self.cost = cost


class RRT:
    """Single-tree arrangement RRT. ``seed=`` makes it replayable."""

    name = "rrt"

    def __init__(self, seed=None):
        self.rng = np.random.default_rng(seed)
        self._stats = {"nodes": 0, "samples": 0, "extends": 0, "replans": 0,
                       "planner": self.name}

    # ---- harness contract ----------------------------------------------------------------
    def reset(self, instance, world):
        self.instance, self.world = instance, world
        self.items = [it["item_id"] for it in instance.items]
        self.cls_of = {it["item_id"]: it["object_class"] for it in instance.items}
        self.goal = {it["item_id"]: np.asarray(it["target"]["T_base_obj"])
                     for it in instance.items}
        self.in_band = getattr(world, "in_counter", lambda T: False)
        self.candidates = self._candidates(instance, world)
        self.plan: list = []
        self.expected: tuple | None = None  # (item_id, T) of the last returned move

    def next_move(self, obs):
        from .rearrange import MOVE_DEV_MAX_M, Move  # local: avoids an import cycle

        poses = {i: np.asarray(obs["items"][i]["T_base_obj"]) for i in self.items}
        if self.expected is not None:
            item, T_cmd = self.expected
            if np.linalg.norm(_pos(poses[item]) - _pos(T_cmd)) > MOVE_DEV_MAX_M:
                self.plan = []          # last move did not take effect: replan
                self._stats["replans"] += 1
            self.expected = None
        if not self.plan:
            self.plan = self._search(poses, obs) or []
            if not self.plan:
                return None             # no solution within the budget: give up
        item, T = self.plan.pop(0)
        self.expected = (item, T)
        return Move(item, T)

    def stats(self):
        return dict(self._stats)

    # ---- candidate lattice ---------------------------------------------------------------
    def _candidates(self, instance, world):
        """Per class: placeable slot poses + buffer cells (fallback: targets + buffers)."""
        classes = sorted({it["object_class"] for it in instance.items})
        cands: dict = {}
        try:
            from . import capacity, config  # noqa: PLC0415

            tables = capacity.load_state_tables(instance.state, classes)
            for cls in classes:
                with config.active_object(cls):
                    cands[cls] = [capacity._nominal_release_pose(tables.slots[cls][sid])
                                  for sid, ok in tables.placeable[cls].items() if ok]
        except Exception:  # toy tests / no caches: the roster's own targets stand in
            for cls in classes:
                cands[cls] = [np.asarray(it["target"]["T_base_obj"])
                              for it in instance.items if it["object_class"] == cls]
        for cls in classes:
            cands[cls] = cands[cls] + [np.asarray(T) for T in world.buffer_poses(cls)]
        return cands

    # ---- search core ---------------------------------------------------------------------
    def _deadline(self, obs):
        left = obs.get("time_left_s")
        if left is None:
            return time.perf_counter() + 30.0  # unbudgeted: a sane self-imposed slice
        return time.perf_counter() + max(0.05, left * (1.0 - TIME_MARGIN_FRAC))

    def _key(self, poses):
        q = 1e-3 * POS_KEY_MM
        return tuple((i, tuple(np.round(_pos(poses[i]) / q).astype(int))) for i in self.items)

    def _sample(self):
        self._stats["samples"] += 1
        if self.rng.random() < GOAL_BIAS:
            return dict(self.goal)
        return {i: self.candidates[self.cls_of[i]][
                    int(self.rng.integers(len(self.candidates[self.cls_of[i]])))]
                for i in self.items}

    def _diff(self, a, b):
        return [i for i in self.items if not _same_pose(a[i], b[i], tol=1e-4)]

    def _band_count(self, poses):
        return sum(bool(self.in_band(T)) for T in poses.values())

    def _move_ok(self, poses, item, T_dest, counter_cap):
        """One-teleport feasibility from arrangement ``poses``: FCL + counter cap."""
        if any(_same_pose(T_dest, poses[j], tol=1e-4) for j in self.items if j != item):
            return False  # destination cell already occupied
        if counter_cap is not None and self.in_band(T_dest) \
                and not self.in_band(poses[item]) \
                and self._band_count(poses) >= counter_cap:
            return False
        self.world.clear()
        self.world.sync(poses, self.cls_of)
        return not self.world.move_collides(item, T_dest,
                                            object_class=self.cls_of[item])

    def _extend(self, tree, keys, sample, goal, counter_cap):
        """One extend toward ``sample``: nearest node by differing-item count, move one
        differing item to its sampled pose. Returns the new node or None."""
        self._stats["extends"] += 1
        best, best_d = None, None
        for node in tree:
            d = len(self._diff(node.poses, sample))
            if best_d is None or d < best_d:
                best, best_d = node, d
        if best is None or best_d == 0:
            return None
        diff = self._diff(best.poses, sample)
        order = [diff[k] for k in self.rng.permutation(len(diff))[:EXTEND_TRIES]]
        for item in order:
            T_dest = sample[item]
            if self._move_ok(best.poses, item, T_dest, counter_cap):
                child_poses = dict(best.poses)
                child_poses[item] = np.asarray(T_dest)
                key = self._key(child_poses)
                if key in keys:
                    continue
                child = _Node(child_poses, parent=best, move=(item, T_dest),
                              cost=best.cost + 1)
                tree.append(child)
                keys.add(key)
                self._stats["nodes"] += 1
                return child
        return None

    def _path(self, node):
        out = []
        while node.parent is not None:
            out.append(node.move)
            node = node.parent
        return list(reversed(out))

    def _restore_mirror(self, poses):
        self.world.clear()
        self.world.sync(poses, self.cls_of)

    def _search(self, start_poses, obs):
        counter_cap = obs.get("counter_cap")
        deadline = self._deadline(obs)
        try:
            return self._grow(start_poses, counter_cap, deadline)
        finally:
            self._restore_mirror(start_poses)

    def _grow(self, start_poses, counter_cap, deadline):
        root = _Node({i: np.asarray(T) for i, T in start_poses.items()})
        tree, keys = [root], {self._key(root.poses)}
        for _ in range(MAX_ITERS):
            if time.perf_counter() > deadline:
                return None
            node = self._extend(tree, keys, self._sample(), self.goal, counter_cap)
            if node is not None and not self._diff(node.poses, self.goal):
                return self._path(node)
        return None


def prove_solvable(instance, world, counter_cap=None, seconds=30.0, seed=0):
    """Constructive solvability proof: one RRT-Connect search from the instance's initials.

    A found plan PROVES the instance solvable under ``counter_cap`` and its length is an
    UPPER bound on the optimum — the certificate of record when the exact A*
    (:func:`compat.optimal_moves`) exhausts its budget on hard instances. ``None`` means no
    proof (not a proof of infeasibility).
    """
    planner = RRTConnect(seed=seed)
    planner.reset(instance, world)
    start = {it["item_id"]: np.asarray(it["T_base_init"]) for it in instance.items}
    deadline = time.perf_counter() + seconds
    try:
        path = planner._grow(start, counter_cap, deadline)
    finally:
        planner._restore_mirror(start)
    return len(path) if path else None


class RRTConnect(RRT):
    """Bidirectional arrangement RRT (a.k.a. BiRRT). Teleport edges are reversible, and the
    goal arrangement is exact, so the goal tree's edges replay backwards: an edge that moved
    ``item`` from pose P to pose Q replays as "move ``item`` from Q back to P"."""

    name = "rrt_connect"

    def _grow(self, start_poses, counter_cap, deadline):
        a = [_Node({i: np.asarray(T) for i, T in start_poses.items()})]
        b = [_Node({i: np.asarray(T) for i, T in self.goal.items()})]
        keys_a, keys_b = {self._key(a[0].poses)}, {self._key(b[0].poses)}
        a_is_start = True
        for _ in range(MAX_ITERS):
            if time.perf_counter() > deadline:
                return None
            new = self._extend(a, keys_a, self._sample(), self.goal, counter_cap)
            if new is not None:
                # CONNECT: greedily extend the other tree toward the new arrangement
                while True:
                    if time.perf_counter() > deadline:
                        return None
                    got = self._extend(b, keys_b, new.poses, self.goal, counter_cap)
                    if got is None:
                        break
                    if not self._diff(got.poses, new.poses):
                        na, nb = (new, got) if a_is_start else (got, new)
                        return self._join(na, nb)
            a, b = b, a
            keys_a, keys_b = keys_b, keys_a
            a_is_start = not a_is_start
        return None

    def _join(self, start_node, goal_node):
        forward = self._path(start_node)
        back = []
        node = goal_node
        while node.parent is not None:
            item, _ = node.move
            back.append((item, node.parent.poses[item]))  # reverse: send it back
            node = node.parent
        return forward + back


class RRTStar(RRT):
    """RRT with choose-parent and rewire on move-count cost (unit edges); the one-move
    neighborhood is every tree node whose arrangement differs by exactly the moved item."""

    name = "rrt_star"

    def _grow(self, start_poses, counter_cap, deadline):
        root = _Node({i: np.asarray(T) for i, T in start_poses.items()})
        tree, keys = [root], {self._key(root.poses)}
        best_goal = None
        for _ in range(MAX_ITERS):
            if time.perf_counter() > deadline:
                break
            node = self._extend(tree, keys, self._sample(), self.goal, counter_cap)
            if node is None:
                continue
            item, T_dest = node.move
            # choose-parent: any node differing ONLY in `item` and able to make this move
            for cand in tree:
                if cand is node or cand.cost + 1 >= node.cost:
                    continue
                d = self._diff(cand.poses, node.poses)
                if d != [item]:
                    continue
                if self._move_ok(cand.poses, item, T_dest, counter_cap):
                    node.parent, node.move = cand, (item, T_dest)
                    node.cost = cand.cost + 1
            # rewire: route one-move neighbors through the new node when cheaper
            for cand in tree:
                if cand is node or cand.parent is None or node.cost + 1 >= cand.cost:
                    continue
                d = self._diff(node.poses, cand.poses)
                if len(d) != 1:
                    continue
                j = d[0]
                if self._move_ok(node.poses, j, cand.poses[j], counter_cap):
                    cand.parent, cand.move = node, (j, cand.poses[j])
                    cand.cost = node.cost + 1
            if not self._diff(node.poses, self.goal):
                if best_goal is None or node.cost < best_goal.cost:
                    best_goal = node  # anytime: keep improving until the deadline
        return self._path(best_goal) if best_goal is not None else None
