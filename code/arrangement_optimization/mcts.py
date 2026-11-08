"""Move-level Monte Carlo tree search for the HOTEC benchmark's open track (track B).

State: where every dish is now (the FCL mirror of the start: counter pile + messy rack drops) plus the dishes
already COMMITTED to their final pose. Action: one move of one uncommitted dish,

    place(d, c)  -> one of d's top-``TOP_POSES`` free catalogue poses (ranked by stand-alone exposure), committed;
    keep(d)      -> a rack dish stays at its start pose (commanded 2 mm above it), committed;
    buffer(d)    -> park a rack dish on the counter (only while the counter holds fewer than its cap).

A move must be legal on the mirror (``world.move_collides``: FCL against the appliance, the counter slab and
every dish where it is now; a counter dish another rests on may not move) and a placement must obey the load
rules (``free_iter``: one dish per slot, no nesting bowls, >= 3 mm from committed dishes, no banned pose/pair).

Value of a leaf: a first-fit ROLLOUT completes the load from the committed dishes (keeps first, then roster
order) and scores it with the low-resolution exposure score; the reward is S, 0 when a dish pools or the load
cannot be completed. Tree policy: UCT on min-max normalised values with progressive widening (a node may hold
``PW_C * visits ** PW_ALPHA`` children), children generated lazily in a fixed priority order (dishes in the
sequencing order, each dish's poses by stand-alone exposure, then keep, then buffer). Transpositions share
their rollout value through a cache keyed by the committed set.

The answer is the best complete load any rollout found (ties: fewer moves): the tree's move prefix that led
to it, then the default policy (``planner.sequence``, greedy with buffering on the mirror under the counter
cap) completes the remaining dishes. Loads in ``exclude`` (earlier attempts) are skipped.

The module is Kit-free and takes the benchmark library (``frigidaire_bench``) as ``B`` to reuse its catalogue,
load rules, packing and scorer.
"""
from __future__ import annotations

import math
import time

import numpy as np

TOP_POSES = 6          # catalogue poses per dish and node (the rest are never tried)
POOL_SECONDS = 15.     # at most this much of the budget builds the pool
HEAD_FIRST_FIT, HEAD_RANKED = 200, 50
RANKED_POSES = 3       # of a dish's TOP_POSES placements, the best stand-alone catalogue poses (the rest: pool poses)
POOL_SIZE = 12         # complete packings (first-fit + seeded shuffles) whose poses form the action set
REPAIRS = 8           # squeaky-wheel restarts of a rollout's first-fit completion
SCAN_POSES = 24        # fitting catalogue poses examined per dish and node before giving up on more placements
UCT_C = 1 / math.sqrt(2)
PW_C, PW_ALPHA = 2.0, 0.5
ROLLOUT_SAMPLES, ROLLOUT_DIRECTIONS = 120, 32      # the analysis trace resolution
KEEP_RESERVE_S = 5.    # of the budget, kept for completing and re-scoring the answer


class Node:
    __slots__ = ("parent", "move", "poses", "committed", "entries", "dishes", "children", "gen", "exhausted",
                 "visits", "value_sum", "depth", "counter_n")

    def __init__(self, parent, move, poses, committed, entries, dishes, depth, counter_n):
        self.parent, self.move, self.poses, self.committed = parent, move, poses, committed
        self.entries, self.dishes, self.depth, self.counter_n = entries, dishes, depth, counter_n
        self.children, self.gen, self.exhausted = [], None, False
        self.visits, self.value_sum = 0, 0.

    def key(self):
        return frozenset((e["id"], e["slot"], e["variant"]) for e in self.entries)


class Conflicts:
    """The load rules of ``free_iter`` (one dish per slot, no nesting bowls, >= 3 mm apart, bans) as cached pairwise
    tests: a rollout re-tests the same catalogue poses against the same neighbours thousands of times."""

    def __init__(self, B, parts, points, bans):
        import fcl
        self.B, self.points, self.bans, self.fcl = B, points, bans, fcl
        self.geo = B.DishSet(parts, points)                 # its pose caches: AABB and FCL manager per pose
        self.clearance = B.GOAL_CLEARANCE_M
        self.pairs, self.single, self.ids, self.objs = {}, {}, {}, []

    def index(self, c):
        """A stable integer per candidate pose object (catalogue and keep candidates are long-lived objects)."""
        i = self.ids.get(id(c))
        if i is None:
            i = self.ids[id(c)] = len(self.objs)
            self.objs.append(c)                              # keep it alive: ids are only unique while it lives
        return i

    def banned1(self, c):
        i = self.index(c)
        v = self.single.get(i)
        if v is None:
            v = self.single[i] = bool(self.bans) and any(frozenset((x,)) in self.bans for x in self.B.cand_keys(c))
        return v

    def pair(self, c, e):
        a, b = self.index(c), self.index(e)
        k = (a, b) if a <= b else (b, a)
        v = self.pairs.get(k)
        if v is None:
            v = self.pairs[k] = self._pair(c, e)
        return v

    def _pair(self, c, e):
        if self.bans and any(frozenset((x, y)) in self.bans for x in self.B.cand_keys(c) for y in self.B.cand_keys(e)):
            return True
        if c["kind"] == "bowl" and e["kind"] == "bowl" and c["rack"] == e["rack"] and self.B.nests(self.points, c, None, [e]):
            return True
        (lo, hi), (olo, ohi), cl = self.geo._bounds(c), self.geo._bounds(e), self.clearance
        if np.any(hi + cl < olo) or np.any(ohi + cl < lo):
            return False
        fcl = self.fcl
        data = fcl.DistanceData(request=fcl.DistanceRequest(), result=fcl.DistanceResult())
        self.geo._manager(c)[0].distance(self.geo._manager(e)[0], data, fcl.defaultDistanceCallback)
        return data.result.min_distance < cl

    def clash(self, c, e):
        return (c["kind"] != "bowl" and e["rack"] == c["rack"] and e["slot"] == c["slot"]) or self.pair(c, e)

    def fits(self, c, cands):
        """``cands``: the committed candidate OBJECTS (not entry copies)."""
        if c["kind"] != "bowl" and any(e["rack"] == c["rack"] and e["slot"] == c["slot"] for e in cands):
            return False
        if self.banned1(c):
            return False
        return not any(self.pair(c, e) for e in cands)


class MoveMCTS:
    """One search. ``ctx`` carries the benchmark objects (see ``plan``); call ``run(deadline)``."""

    def __init__(self, B, instance, world, fam, masks, parts, points, scorer, bans, seed=0, exclude=(), log=print):
        P = B._planner()
        self.B, self.P, self.inst, self.world = B, P, instance, world
        self.fam, self.masks, self.parts, self.points, self.scorer, self.bans = fam, masks, parts, points, scorer, bans
        self.rng = np.random.default_rng(seed)
        self.exclude, self.log = set(exclude), log
        self.kinds = {o["object_id"]: o["kind"] for o in instance["objects"]}
        self.ids = [(o["object_id"], o["kind"]) for o in instance["objects"]]            # roster order
        self.order = P.item_order(instance)                                               # sequencing order
        self.cap = instance["counter"]["cap"]
        self.keeps = B.keep_candidates(instance, points)
        self.cf = Conflicts(B, parts, points, bans)
        self.ff_order = {kind: [c for (c, o), ok in zip(fam[kind], masks[kind]) if ok] for kind in B.KINDS}
        # every masked catalogue pose, best stand-alone exposure first (one Warp batch per kind)
        self.ranked = {}
        for kind in B.KINDS:
            cands = [(c, o) for (c, o), ok in zip(fam[kind], masks[kind]) if ok]
            iso = np.asarray(scorer.iso([c for c, _ in cands])) if cands else np.zeros(0)
            self.ranked[kind] = [cands[i] for i in np.argsort(-iso, kind="stable")]
        self.iso_rank = {kind: {id(c): r for r, (c, _) in enumerate(self.ranked[kind])} for kind in B.KINDS}
        self.pool, self.pool_poses = [], {kind: [] for kind in B.KINDS}
        self.ranked_list = {kind: [c for c, _ in self.ranked[kind]] for kind in B.KINDS}
        self.rollouts, self.cache, self.best, self.use_keeps = 0, {}, [], None
        self.outcomes = {"scored": 0, "pooling": 0, "no_packing": 0, "excluded": 0}
        self.toxic = set()          # best: [(S, moves, entries, node)] sorted
        self.history, self.nodes, self.vmin, self.vmax = [], 0, None, None
        start = {oid: np.asarray(T, dtype=float) for oid, T in world.initial.items()}
        self.root = Node(None, None, start, frozenset(), (), (), 0,
                         sum(bool(world.in_counter(T)) for T in start.values()))

    # ------------------------------------------------------------------ packing pool
    def _pack(self, order_of_kind, entries=(), cands=()):
        entries, cands = list(entries), list(cands)
        placed = {e["id"] for e in entries}
        for oid, kind in self.ids:
            if oid in placed:
                continue
            pick = next((c for c in order_of_kind[kind] if self.cf.fits(c, cands)), None)
            if pick is None:
                return None
            entries.append(self.B.entry_of(oid, pick))
            cands.append(pick)
        return entries, cands

    def build_pool(self, deadline, size=POOL_SIZE):
        """Complete packings from first-fit and seeded shuffles of it: the poses a dish may be moved to (a pose that
        belongs to some complete load), and templates for completing a rollout."""
        orders = [self.ff_order]
        while len(self.pool) < size and time.monotonic() < deadline and len(orders) < 4 * size:
            # plates keep the first-fit order (a shuffled plate order scatters them into the bowls' space: 20 of 20
            # shuffled packs failed on medium, 2026-09-28). Bowls and cups shuffle the HEAD of an order: the first
            # 200 of the first-fit order completed 6/6 packs on medium_s2 (a full shuffle 0/6), the first 50 of the
            # exposure ranking 1/6 but with better poses; alternate the two.
            if len(orders) == 1 and not self.pool:
                o = self.ff_order
            else:
                ranked_head = len(orders) % 3 == 0
                o = {}
                for k, v in self.ff_order.items():
                    if k == "plate":
                        o[k] = v
                        continue
                    src = [c for c, _ in self.ranked[k]] if ranked_head else list(v)
                    n = HEAD_RANKED if ranked_head else HEAD_FIRST_FIT
                    head = src[:n]
                    o[k] = [head[i] for i in self.rng.permutation(len(head))] + src[n:]
            orders.append(o)
            got = self._pack(o)
            if got is not None:
                self.pool.append(got)
        seen = {kind: set() for kind in self.B.KINDS}
        for entries, cands in self.pool:
            for c in cands:
                if id(c) not in seen[c["kind"]]:
                    seen[c["kind"]].add(id(c))
                    self.pool_poses[c["kind"]].append(c)
        for kind in self.B.KINDS:
            self.pool_poses[kind].sort(key=lambda c: self.iso_rank[kind][id(c)])

    # ------------------------------------------------------------------ actions
    def _T(self, rack, c):
        T = self.P.goal_T(self.world, rack, {"position_m": list(c["position"]), "quaternion_xyzw": list(c["quaternion_xyzw"])})
        self.world.certify(T)                      # masked catalogue poses clear the appliance at their hover
        return T

    def _sync(self, node):
        self.world.sync(node.poses, self.kinds)

    def _legal(self, node, oid, T):
        self._sync(node)
        return not self.world.move_collides(oid, T)

    def _actions(self, node):
        """Lazy generator of the node's legal moves in priority order: placements and keeps round-robin over the
        uncommitted dishes (sequencing order, each dish's poses by stand-alone exposure), then counter parks (a
        park changes no dish's final pose, so it only matters when it unblocks a placement)."""
        todo = [oid for oid in self.order if oid not in node.committed]
        per = {oid: self._dish_actions(node, oid) for oid in todo}
        while per:
            for oid in list(per):
                a = next(per[oid], None)
                if a is None:
                    del per[oid]
                else:
                    yield a
        for oid in todo:
            a = self._buffer_action(node, oid)
            if a is not None:
                yield a

    def _dish_actions(self, node, oid):
        """Up to TOP_POSES placements: the RANKED_POSES best stand-alone poses of the whole catalogue that fit, then
        poses from the packing pool (each belongs to some complete load), then the keep."""
        kind = self.kinds[oid]
        cands = node.dishes
        n, tried = 0, set()
        for source, limit in ((self.ranked_list[kind], RANKED_POSES), (self.pool_poses[kind], TOP_POSES)):
            scanned = 0
            for c in source:
                if n >= limit or scanned >= SCAN_POSES:
                    break
                if id(c) in tried or id(c) in self.toxic or not self.cf.fits(c, cands):
                    continue
                tried.add(id(c))
                scanned += 1
                T = self._T(c["rack"], c)
                if self._legal(node, oid, T):
                    n += 1
                    yield ("place", oid, c, T)
        k = self.keeps.get(oid)
        if k is not None and self.cf.fits(k, cands):
            T = self._T(k["rack"], k)
            if self._legal(node, oid, T):
                yield ("keep", oid, k, T)

    def _buffer_action(self, node, oid):
        if node.counter_n >= self.cap or self.world.in_counter(node.poses[oid]):
            return None
        for T in self.world.buffer_poses(self.kinds[oid]):
            if self._legal(node, oid, T):
                return ("buffer", oid, None, T)
        return None

    def _child(self, node, action):
        kind_, oid, c, T = action
        poses = dict(node.poses)
        was_counter = bool(self.world.in_counter(poses[oid]))
        poses[oid] = T
        counter_n = node.counter_n + (1 if kind_ == "buffer" else 0) - (1 if was_counter else 0)
        committed, entries, dishes = node.committed, node.entries, node.dishes
        if kind_ in ("place", "keep"):
            committed = committed | {oid}
            entries = entries + (self.B.entry_of(oid, c, keep=kind_ == "keep"),)
            dishes = dishes + (c,)
        child = Node(node, (kind_, oid, T), poses, committed, entries, dishes, node.depth + 1, counter_n)
        self.nodes += 1
        return child

    # ------------------------------------------------------------------ rollout
    def _complete(self, node):
        """First-fit completion from the committed dishes: keeps first, then roster order (the keeps are dropped
        for good when they block the root's packing, as ``goal_search`` falls back); None if it fails."""
        if self.use_keeps is None:
            got = self._complete_with(node, True)
            self.use_keeps = got is not None
            return got or self._complete_with(node, False)
        return self._complete_with(node, self.use_keeps)

    def _complete_with(self, node, use_keeps):
        """First-fit with squeaky-wheel repair: the dish that found no free pose moves to the front of the order
        and the packing restarts (``REPAIRS`` times). Plain first-fit only completes when every dish takes its
        usual pose; once the tree commits a dish elsewhere it failed in 11 of 12 rollouts (medium_s2)."""
        base_entries, base = list(node.entries), list(node.dishes)
        placed = {e["id"] for e in base_entries}
        for oid, k in (self.keeps.items() if use_keeps else ()):
            if oid in placed or not self.cf.fits(k, base):
                continue
            base_entries.append(self.B.entry_of(oid, k, keep=True))
            base.append(k)
            placed.add(oid)
        order = [(oid, kind) for oid, kind in self.ids if oid not in placed]
        for _ in range(REPAIRS + 1):
            entries, cands, failed = list(base_entries), list(base), None
            for oid, kind in order:
                pick = next((c for c in self.ff_order[kind] if self.cf.fits(c, cands)), None)
                if pick is None:
                    failed = (oid, kind)
                    break
                entries.append(self.B.entry_of(oid, pick))
                cands.append(pick)
            if failed is None:
                return entries, cands
            order.remove(failed)
            order.insert(0, failed)
        for _, tpl in self.pool:                             # complete from a pooled packing's poses first
            pref = {kind: [c for c in tpl if c["kind"] == kind] + self.ff_order[kind] for kind in self.B.KINDS}
            got = self._pack(pref, base_entries, base)
            if got is not None:
                return got
        return None

    def _local(self, node):
        """Rollout by LOCAL repair of the parent's completed load (the goal search's one-dish move): the moved dish
        takes its new pose, the uncommitted dishes it now clashes with are taken out and re-placed first-fit
        (squeaky-wheel order); a failure falls back to the full completion. Completing from scratch failed in
        37 of 39 medium rollouts once the tree put a bowl on a high-exposure pose."""
        parent = node.parent
        if parent is None or node.move[0] == "buffer":
            return "n/a"
        got = self.cache.get(parent.key())
        if got is None or got[1] is None:
            return "n/a"
        oid, c, e_new = node.move[1], node.dishes[-1], node.entries[-1]
        load = {e["id"]: (e, cc) for e, cc in zip(got[1], got[2])}
        vacated = [load[oid][1]] if oid in load else []     # the moved dish's old pose: a displaced dish of the same
        load[oid] = (e_new, c)                               # kind takes it first (the swap), then first-fit
        removed = [d for d, (e, cc) in load.items() if d != oid and d not in node.committed and self.cf.clash(c, cc)]
        for d in removed:
            del load[d]
        order = [(d, self.kinds[d]) for d in removed]
        for _ in range(REPAIRS + 1):
            cands = [cc for _, cc in load.values()]
            placed, failed = {}, None
            for d, kind in order:
                first = [v for v in vacated if v["kind"] == kind and not v.get("variant") == "keep"]
                pick = next((x for x in first + self.ff_order[kind] if self.cf.fits(x, cands)), None)
                if pick is None:
                    failed = (d, kind)
                    break
                placed[d] = (self.B.entry_of(d, pick), pick)
                cands.append(pick)
            if failed is None:
                load.update(placed)
                ordered = [load[d] for d, _ in self.ids]
                return [e for e, _ in ordered], [cc for _, cc in ordered]
            order.remove(failed)
            order.insert(0, failed)
        self.toxic.add(id(c))                                # this pose leaves no room for the rest: never offered again
        return None

    def _rollout(self, node):
        key = node.key()
        if key in self.cache:
            return self.cache[key][0]
        got = self._local(node)
        if isinstance(got, str):                             # root or park: complete from the committed dishes
            got = self._complete(node)
        entries, cands = got if got is not None else (None, None)
        value = 0.
        if entries is None:
            self.outcomes["no_packing"] += 1
        elif self.B.load_key(entries) in self.exclude:
            self.outcomes["excluded"] += 1
        else:
            res = self.scorer.hx.full_score(entries, "rollout", self.scorer.device, ROLLOUT_SAMPLES, ROLLOUT_DIRECTIONS)
            value = float(res["score"]) if res["feasible"] else 0.
            self.outcomes["scored" if res["feasible"] else "pooling"] += 1
        self.cache[key] = (value, entries, cands)
        self.rollouts += 1
        if entries is not None and value > 0:
            self.best.append((value, node.depth, entries, node))
            self.best.sort(key=lambda b: (-b[0], b[1]))
            del self.best[8:]
        return value

    # ------------------------------------------------------------------ search
    def _norm(self, v):
        if self.vmax is None or self.vmax - self.vmin < 1e-12:
            return .5
        return (v - self.vmin) / (self.vmax - self.vmin)

    def _select(self, node):
        logn = math.log(max(1, node.visits))
        return max(node.children, key=lambda ch: self._norm(ch.value_sum / ch.visits) + UCT_C * math.sqrt(logn / ch.visits))

    def _expand(self, node):
        if node.gen is None:
            node.gen = self._actions(node)
        a = next(node.gen, None)
        if a is None:
            node.exhausted = True
            return None
        ch = self._child(node, a)
        node.children.append(ch)
        return ch

    def simulate(self):
        node = self.root
        while True:
            if len(node.committed) == len(self.ids):
                break                                            # terminal: every dish committed
            may_widen = not node.exhausted and len(node.children) < max(1, PW_C * max(1, node.visits) ** PW_ALPHA)
            if may_widen:
                ch = self._expand(node)
                if ch is not None:
                    node = ch
                    break
            if not node.children:
                break                                            # dead end: no legal move
            node = self._select(node)
        v = self._rollout(node)
        if v <= 0.:
            node.exhausted = True                                # a dead end: its load cannot be completed
        self.vmin = v if self.vmin is None else min(self.vmin, v)
        self.vmax = v if self.vmax is None else max(self.vmax, v)
        while node is not None:
            node.visits += 1
            node.value_sum += v
            node = node.parent
        return v

    def run(self, deadline, max_simulations=None):
        t0 = time.monotonic()
        self.build_pool(min(deadline, t0 + POOL_SECONDS))
        self._rollout(self.root)                                 # the plain first-fit completion is the floor
        ff = self.cache[self.root.key()]
        self.first_fit_value = ff[0]
        for entries, cands in self.pool:                         # every pooled packing is a complete load too:
            if self.B.load_key(entries) in self.exclude:         # the root's load is the best of them
                continue
            res = self.scorer.hx.full_score(entries, "pool", self.scorer.device, ROLLOUT_SAMPLES, ROLLOUT_DIRECTIONS)
            v = float(res["score"]) if res["feasible"] else 0.
            if v > 0:
                self.best.append((v, 0, entries, self.root))
            if v > self.cache[self.root.key()][0]:
                self.cache[self.root.key()] = (v, entries, cands)
        self.best.sort(key=lambda b: (-b[0], b[1]))
        del self.best[8:]
        self.root.visits = 1
        self.root.value_sum = self.cache[self.root.key()][0]
        sims = 0
        while time.monotonic() < deadline and (max_simulations is None or sims < max_simulations):
            self.simulate()
            sims += 1
            if self.best:
                self.history.append((round(time.monotonic() - t0, 2), sims, self.best[0][0]))
        self.simulations = sims
        return self.best

    # ------------------------------------------------------------------ answer
    def prefix(self, node):
        moves = []
        while node is not None and node.move is not None:
            moves.append(node.move)
            node = node.parent
        return moves[::-1]

    def answer(self):
        """Best completable load: (entries, moves [(kind, id, T)], S_low, prefix_len) or None."""
        for value, _, entries, node in self.best:
            prefix = self.prefix(node)
            self._sync(node)
            targets = {e["id"]: self._T(e["rack"], e) for e in entries}
            for (k, oid, T) in prefix:                          # committed dishes sit exactly at their targets
                if k in ("place", "keep"):
                    targets[oid] = T
            snapshot = self.world.snapshot()
            try:
                rest = self.P.sequence(self.world, targets, self.order, self.cap)
            finally:
                self.world.sync(snapshot, self.kinds)
            if rest is None:
                continue
            moves = list(prefix) + [("buffer" if self.world.in_counter(np.asarray(m.T_base_obj)) else "place",
                                     m.item_id, np.asarray(m.T_base_obj)) for m in rest]
            return entries, moves, value, len(prefix)
        return None

    def stats(self):
        visits = sorted(((ch.visits, ch.move[0], ch.move[1]) for ch in self.root.children), reverse=True)
        depth = 0
        stack = [self.root]
        while stack:
            n = stack.pop()
            depth = max(depth, n.depth)
            stack.extend(n.children)
        return {"simulations": getattr(self, "simulations", 0), "rollouts": self.rollouts, "rollout_outcomes": dict(self.outcomes), "toxic_poses": len(self.toxic),
                "nodes": self.nodes,
                "max_depth": depth, "root_children": len(self.root.children),
                "root_visits": [list(v) for v in visits[:10]], "best_S_low": [round(b[0], 5) for b in self.best],
                "first_fit_S_low": round(getattr(self, "first_fit_value", 0.), 5),
                "root_S_low": round(self.cache[self.root.key()][0], 5), "pool": len(self.pool),
                "pool_poses": {k: len(v) for k, v in self.pool_poses.items()},
                "history": self.history[:: max(1, len(self.history) // 200)]}


def plan(B, instance, world, fam, masks, parts, points, scorer, bans, deadline, seed=0, exclude=(), log=print,
         max_simulations=None):
    """Run a search until ``deadline`` (monotonic); returns (answer or None, stats)."""
    search = MoveMCTS(B, instance, world, fam, masks, parts, points, scorer, bans, seed=seed, exclude=exclude, log=log)
    search.run(deadline - KEEP_RESERVE_S, max_simulations=max_simulations)
    return search.answer(), search.stats()
