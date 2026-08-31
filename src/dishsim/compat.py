# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Discrete ground truth for the rearrangement benchmark: a pairwise compatibility table and
an exact optimal solver — Kit-free.

An algorithm comparison that only ranks algorithms against each other cannot say whether the
winner is any *good*. This module supplies the missing anchor: the provably minimum number of
moves for an instance, so a result reads "9 moves against an optimum of 9" instead of "fewer
moves than the other one".

It rests on one measured property of the collision model: feasibility here is exactly
**pairwise-decomposable**. :meth:`~dishsim.collision_world.CollisionWorld.object_in_collision`
is "candidate vs statics OR (candidate vs some placed item)", with no three-body term, so an
arrangement is feasible iff every occupied cell is statically free and every occupied PAIR is
compatible. That turns feasibility from a per-state FCL query into a table lookup: the whole
table costs a few thousand queries once, and the search then does zero geometry.

Two honest limits, which belong in any write-up that quotes these numbers:

- The optimum is a **geometric-relaxation** optimum. Cells are the modes' nominal
  release-hover poses, while settled poses deviate from the commanded pose by a measured
  12.8-22.0 mm. A planner running under physics can therefore occasionally beat this number —
  measured once, when a neighbour's settle nudged an unmoved item into tolerance.
- Restricting placements to a finite cell set removes continuous buffer placement. The optimum
  is exact *within that abstraction* and an upper bound on the continuous optimum.
"""

import heapq
import time
import itertools

import numpy as np

from . import capacity, config, placement
from .collision_world import CollisionWorld, load_object_pieces

#: Locations are ``(object_class, key)``. ``key`` is a slot id, ``("buf", i)`` for a counter
#: buffer cell, or ``("init", item_id)`` for an item's start pose.
Location = tuple


class CompatTable:
    """Static feasibility + pairwise compatibility over a discrete location set.

    Attributes:
        poses: ``{location: T_base_obj}`` [m], the 4x4 pose an object of that location's class
            occupies.
        static_ok: ``{location: bool}`` — collision-free in the EMPTY machine.
        n_queries: FCL queries spent building the table (reported, so the cost is not hidden).
    """

    def __init__(self, state: str, classes: list[str], instance=None):
        self.state, self.classes = state, sorted(set(classes))
        self.poses: dict = {}
        self.static_ok: dict = {}
        self._pair_ok: dict = {}
        self.n_queries = 0
        self._pieces: dict = {}
        self._worlds: dict = {}
        self.counter_locs: set = set()
        self._build_locations()
        self._extend_table(list(self.poses))
        self._refresh_counter_locs()
        if instance is not None:
            self.set_instance(instance)

    # ---- construction -------------------------------------------------------------------
    def _build_locations(self) -> None:
        from .rearrange import ArrangementWorld  # local: avoids an import cycle

        aw = ArrangementWorld(self.state, self.classes)
        for cls in self.classes:
            cdir = config.scenario_cache_dir(self.state, object_name=cls)
            # every cache read is hash-guarded against the ACTIVE object, so the whole
            # per-class block has to run under it — not just the slot derivation
            with config.active_object(cls):
                self._pieces[cls] = load_object_pieces(cdir)
                self._worlds[cls] = CollisionWorld(cache_dir=cdir)
                for slot in placement.derive_slots(cdir):
                    self.poses[(cls, slot.slot_id)] = capacity._nominal_release_pose(slot)
            for i, T in enumerate(aw.buffer_poses(cls)):
                self.poses[(cls, ("buf", i))] = np.asarray(T)

    def set_instance(self, instance) -> "CompatTable":
        """(Re)point the ``("init", item)`` locations at THIS instance's start poses.

        A table is reusable across instances of one (state, classes) batch ONLY through
        this: init locations are per-instance, and reusing a stale table silently solves
        against the PREVIOUS instance's blockers (the bug that produced the pre-2026-08-31
        s1/s2 optimum pins).
        """
        stale = [l for l in self.poses if isinstance(l[1], tuple) and l[1][0] == "init"]
        for loc in stale:
            self.poses.pop(loc)
            self.static_ok.pop(loc, None)
        if stale:
            gone = set(stale)
            self._pair_ok = {k: v for k, v in self._pair_ok.items()
                            if k[0] not in gone and k[1] not in gone}
        new = []
        for it in instance.items:  # start poses are locations too: an item there blocks cells
            key = (it["object_class"], ("init", it["item_id"]))
            self.poses[key] = np.asarray(it["T_base_init"], dtype=float)
            new.append(key)
        self._extend_table(new)
        self._refresh_counter_locs()
        return self

    def _refresh_counter_locs(self) -> None:
        """Locations that occupy the counter band: every buffer cell, plus any init pose
        geometrically inside the band (items STARTING on the counter count against a cap)."""
        from .rearrange import in_counter_band  # local: avoids an import cycle

        self.counter_locs = {
            loc for loc, T in self.poses.items()
            if isinstance(loc[1], tuple) and (
                loc[1][0] == "buf"
                or (loc[1][0] == "init" and in_counter_band(T)))}

    def _extend_table(self, new_locs: list) -> None:
        """Static test per new location, then one pair test per (new, any-other) pair.

        Pairs are computed over EVERY location, not only the statically-free ones. A settled
        start pose is not a legal destination — a resting object's margin-inflated hull touches
        the wire floor it stands on, so it fails the static test — but an object sitting there
        still blocks its neighbours, and dropping those pairs would make every move look free.
        """
        new_set = set(new_locs)
        existing = [l for l in self.poses if l not in new_set]
        for loc in new_locs:
            cls = loc[0]
            self.n_queries += 1
            self.static_ok[loc] = not self._worlds[cls].object_in_collision(
                self._pieces[cls], self.poses[loc])
        # Pairwise tests run with the statics DISABLED, so the pair term is isolated from the
        # static term. Mixing them would make every pair involving a statically-blocked
        # location read "incompatible" — and a settled start pose IS statically blocked (a
        # resting object touches its own support), which would poison the whole table.
        for world in self._worlds.values():
            for name in list(world._static_objs):
                world.set_static_enabled(name, False)
        try:
            for a, b in itertools.combinations(new_locs, 2):
                self._pair_ok[(a, b)] = self._pair_free(a, b)
            for a in new_locs:
                for b in existing:
                    self._pair_ok[(a, b)] = self._pair_free(a, b)
        finally:
            for world in self._worlds.values():
                for name in list(world._static_objs):
                    world.set_static_enabled(name, True)

    def _pair_free(self, a: Location, b: Location) -> bool:
        """Can an object sit at ``a`` while another sits at ``b``? (Pair term only.)"""
        cls_a, cls_b = a[0], b[0]
        world = self._worlds[cls_a]
        world.add_object("__probe__", self._pieces[cls_b], self.poses[b])
        try:
            self.n_queries += 1
            return not world.object_in_collision(self._pieces[cls_a], self.poses[a])
        finally:
            world.remove_object("__probe__")

    # ---- queries ------------------------------------------------------------------------
    def compatible(self, a: Location, b: Location) -> bool:
        """Whether objects at two locations can coexist (symmetric).

        Purely pairwise: whether a location is a legal DESTINATION is
        :attr:`static_ok`, a separate question, because an object may legitimately occupy a
        pose it could not be commanded into.
        """
        if a == b:
            return False
        return self._pair_ok.get((a, b), self._pair_ok.get((b, a), False))

    def arrangement_ok(self, locs) -> bool:
        """Whether a whole assignment of items to locations is jointly feasible."""
        locs = list(locs)
        return all(self.compatible(a, b) for a, b in itertools.combinations(locs, 2))


def optimal_moves(instance, table: CompatTable | None = None, max_expansions: int = 2_000_000,
                  max_seconds: float | None = 120.0, counter_cap: int | None = None):
    """Provably minimum number of moves to bring every item to its target location.

    A* over assignments of items to locations, with the admissible and consistent heuristic
    "number of items not at their target" — each misplaced item needs at least one move, and
    one move fixes at most one item.

    Args:
        instance: A ``rearrange.Instance``.
        table: Prebuilt :class:`CompatTable`; built here if omitted. A passed table is
            re-pointed at THIS instance's start poses (:meth:`CompatTable.set_instance`).
        max_expansions: Safety bound on A* expansions.
        max_seconds: Wall-clock bound, or ``None`` for unlimited.
        counter_cap: Max SIMULTANEOUS items at counter-band locations
            (``table.counter_locs``), matching the episode driver's cap; ``None`` = no cap.
            The cap gates only moves whose DESTINATION is in the band (moving out is always
            legal), exactly the driver's semantics.

    Returns:
        ``(n_moves, status, table)`` with ``status`` in ``("solved", "bound",
        "unsolvable")``. ``"unsolvable"`` is PROVEN (a goal is off-lattice, or the open list
        exhausted under an admissible heuristic); ``"bound"`` means the expansion/time
        budget ran out and the optimum is unknown. ``n_moves`` is ``None`` unless solved.
    """
    classes = sorted({it["object_class"] for it in instance.items})
    if table is None:
        table = CompatTable(instance.state, classes, instance=instance)
    elif hasattr(table, "set_instance"):
        table.set_instance(instance)

    from .rearrange import at_goal  # local: avoids an import cycle

    items = [it["item_id"] for it in instance.items]
    cls_of = {it["item_id"]: it["object_class"] for it in instance.items}
    start, goal = [], []
    for it in instance.items:
        cls = it["object_class"]
        goal_loc = _nearest_location(table, cls, it["target"]["T_base_obj"])
        goal.append(goal_loc)
        # An item already AT GOAL starts on its goal cell, not on a synthetic one — otherwise
        # the solver charges it a move to go where the benchmark already considers it placed,
        # inflating every optimum. Pose-snapping cannot decide this: recorded initials are
        # SETTLED poses, 12.8-22.0 mm below the nominal hover cell they correspond to, so the
        # harness's own tolerance-based predicate is the right judge.
        T_init = np.asarray(it["T_base_init"], dtype=float)
        target = it.get("target") or {}
        if "slot" in target:
            home = at_goal(it, T_init)
        else:  # synthetic instance (tests): no slot frame to evaluate, so compare poses
            home = _nearest_location(table, cls, T_init) == goal_loc
        start.append(goal_loc if (goal_loc is not None and home)
                     else (cls, ("init", it["item_id"])))
    start, goal = tuple(start), tuple(goal)
    if any(loc is None for loc in goal):
        return None, "unsolvable", table

    # Index the locations and precompute compatibility as BITMASKS. The search runs millions of
    # "can these coexist?" checks; a dict lookup or a numpy fancy-index per pair is what makes a
    # Python A* over 15 items intractable, whereas one big-int AND answers it for a whole state.
    loc_list = list(table.poses)
    loc_idx = {loc: i for i, loc in enumerate(loc_list)}
    n_loc = len(loc_list)
    conflict = [0] * n_loc  # conflict[i] = bitmask of locations that CANNOT coexist with i
    for i, a in enumerate(loc_list):
        m = 1 << i  # a location always conflicts with itself (one object per cell)
        for j in range(n_loc):
            if j != i and not table.compatible(a, loc_list[j]):
                m |= 1 << j
        conflict[i] = m

    # counter-band occupancy mask for the cap: buffer cells + in-band init poses. Stub
    # tables without counter_locs fall back to the ("buf", i) convention.
    counter_locs = getattr(table, "counter_locs", None)
    if counter_locs is None:
        counter_locs = {l for l in loc_list if isinstance(l[1], tuple) and l[1][0] == "buf"}
    buf_mask = 0
    for i, l in enumerate(loc_list):
        if l in counter_locs:
            buf_mask |= 1 << i

    start_i = tuple(loc_idx[s] for s in start)
    goal_i = tuple(loc_idx[g] for g in goal)
    n = len(items)

    # candidate destinations per item: its own class's statically-free locations, minus every
    # item's start cell (a settled start pose is not a re-enterable cell)
    other_inits = {(cls_of[i], ("init", i)) for i in items}
    dests = {
        k: [loc_idx[loc] for loc in table.poses
            if loc[0] == cls_of[item] and table.static_ok.get(loc, False)
            and loc not in other_inits]
        for k, item in enumerate(items)
    }

    def h(state):
        """Admissible: each misplaced item needs a move, and an at-goal item that blocks a
        misplaced item's goal must leave AND come back — two moves neither term counts."""
        misplaced = [k for k in range(n) if state[k] != goal_i[k]]
        base = len(misplaced)
        extra = 0
        for k in range(n):
            if state[k] == goal_i[k] and any(conflict[state[k]] >> goal_i[m] & 1
                                             for m in misplaced):
                extra += 2
        return base + extra

    def mask_of(state):
        m = 0
        for s in state:
            m |= 1 << s
        return m

    # tie-break counter: heapq falls through to comparing the payload on equal (f, g), and
    # comparing states is both meaningless and (for mixed key types) an error
    tick = itertools.count()
    open_heap = [(h(start_i), 0, next(tick), start_i)]
    best = {start_i: 0}
    expansions = 0
    t_start = time.perf_counter()
    while open_heap:
        _, g, _, state = heapq.heappop(open_heap)
        if state == goal_i:
            return g, "solved", table
        if g > best.get(state, float("inf")):
            continue
        expansions += 1
        if expansions > max_expansions:
            return None, "bound", table
        if max_seconds is not None and not expansions % 1024 \
                and time.perf_counter() - t_start > max_seconds:
            return None, "bound", table

        occ = mask_of(state)
        misplaced = [k for k in range(n) if state[k] != goal_i[k]]
        # Sound prune: an at-goal item is worth moving ONLY if it blocks a misplaced item's
        # goal. Moving one that blocks nothing can never shorten a plan, while keeping the
        # blockers movable is what preserves optimality on non-monotone instances.
        movable = list(misplaced)
        movable += [k for k in range(n) if state[k] == goal_i[k]
                    and any(conflict[state[k]] >> goal_i[m] & 1 for m in misplaced)]
        for idx in movable:
            others = occ & ~(1 << state[idx])
            for loc in dests[idx]:
                if conflict[loc] & others:
                    continue  # occupied or incompatible with something already placed
                if counter_cap is not None and (1 << loc) & buf_mask and \
                        ((others | (1 << loc)) & buf_mask).bit_count() > counter_cap:
                    continue  # counter band at capacity — mirrors the driver's refusal
                nxt = state[:idx] + (loc,) + state[idx + 1:]
                ng = g + 1
                if ng < best.get(nxt, float("inf")):
                    best[nxt] = ng
                    heapq.heappush(open_heap, (ng + h(nxt), ng, next(tick), nxt))
    return None, "unsolvable", table  # open list exhausted: PROVEN infeasible


def _nearest_location(table: CompatTable, cls: str, T_target, tol_m: float = 1e-3):
    """The table location matching a target pose, or ``None`` if the target is off-lattice."""
    T_target = np.asarray(T_target, dtype=float)
    best, best_d = None, float("inf")
    for loc, T in table.poses.items():
        if loc[0] != cls or (isinstance(loc[1], tuple) and loc[1][0] == "init"):
            continue
        d = float(np.linalg.norm(np.asarray(T)[:3, 3] - T_target[:3, 3]))
        if d < best_d:
            best, best_d = loc, d
    return best if best_d <= tol_m else None
