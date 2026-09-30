"""Kit-free tests of the move-level MCTS (dishsim_frigidaire.mcts) with a toy load model: no FCL, no scorer."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire import mcts as M  # noqa: E402


def cand(kind, rack, slot, x):
    return {"kind": kind, "rack": rack, "slot": slot, "variant": "v", "position": [x, 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}


class ToyConflicts(M.Conflicts):
    """Load rules with a 1-D overlap test instead of FCL: poses closer than 0.1 clash."""

    def __init__(self, bans=()):
        self.B = SimpleNamespace(cand_keys=lambda c: (f"{c['rack']}|{c['slot']}|{c['position'][0]}",),
                                 nests=lambda *a: False)
        self.bans, self.pairs, self.single, self.ids, self.objs = set(bans), {}, {}, {}, []

    def _pair(self, c, e):
        return abs(c["position"][0] - e["position"][0]) < .1


def test_fits_enforces_one_dish_per_slot_bans_and_clearance():
    a, b = cand("plate", "LowerRack", "g1", 0.), cand("plate", "LowerRack", "g1", 1.)
    c, bowl = cand("plate", "LowerRack", "g2", .05), cand("bowl", "LowerRack", "g1", 5.)
    cf = ToyConflicts(bans={frozenset(("LowerRack|g9|3.0",))})
    assert not cf.fits(b, [a])                                   # same slot, another plate
    assert cf.fits(bowl, [a])                                    # bowls ignore the slot rule
    assert not cf.fits(c, [a])                                   # too close
    assert not cf.fits(cand("cup", "LowerRack", "g9", 3.), [])   # banned pose
    assert cf.pair(a, c) is cf.pair(c, a)                        # symmetric and cached
    assert len(cf.pairs) == 2                                    # (a, bowl) and (a, c), each once


def test_clash_is_slot_or_pair():
    cf = ToyConflicts()
    a, b = cand("cup", "UpperRack", "s1", 0.), cand("cup", "UpperRack", "s1", 9.)
    assert cf.clash(a, b) and not cf.clash(a, cand("cup", "UpperRack", "s2", 9.))


def _search(first_fit, pool=()):
    """A MoveMCTS shell with toy rules: dishes d1 (plate) and d2 (plate); no world, no scorer."""
    s = object.__new__(M.MoveMCTS)
    s.B = SimpleNamespace(entry_of=lambda oid, c, keep=False: {"id": oid, **c, "keep": keep}, KINDS=("plate",),
                          load_key=lambda es: frozenset((e["id"], e["slot"]) for e in es))
    s.cf = ToyConflicts()
    s.ids, s.kinds = [("d1", "plate"), ("d2", "plate")], {"d1": "plate", "d2": "plate"}
    s.ff_order, s.keeps, s.pool, s.use_keeps = {"plate": first_fit}, {}, list(pool), False
    s.cache, s.toxic, s.exclude = {}, set(), set()
    return s


def test_local_repair_swaps_the_displaced_dish_into_the_vacated_pose():
    g1, g2, g3 = cand("plate", "L", "g1", 0.), cand("plate", "L", "g2", 1.), cand("plate", "L", "g3", 1.05)
    s = _search([g1, g2, g3])
    root = M.Node(None, None, {}, frozenset(), (), (), 0, 0)
    entries, cands = s._complete(root)
    assert [c["slot"] for c in cands] == ["g1", "g2"]             # first-fit
    s.cache[root.key()] = (1., entries, cands)
    # move d1 onto g3, which clashes with d2 on g2: d2 must take d1's vacated g1
    e3 = s.B.entry_of("d1", g3)
    child = M.Node(root, ("place", "d1", None), {}, frozenset({"d1"}), (e3,), (g3,), 1, 0)
    got = s._local(child)
    assert [e["slot"] for e in got[0]] == ["g3", "g1"]
    assert s._local(M.Node(root, ("buffer", "d1", None), {}, frozenset(), (), (), 1, 0)) == "n/a"


def test_local_repair_failure_marks_the_pose_toxic():
    g1, g2 = cand("plate", "L", "g1", 0.), cand("plate", "L", "g2", 1.)
    wide = cand("plate", "L", "g3", .5)
    wide["position"] = [0.5, 0., 0.]
    s = _search([g1, g2])
    s.cf._pair = lambda c, e: c is wide or e is wide             # the wide pose clashes with everything
    root = M.Node(None, None, {}, frozenset(), (), (), 0, 0)
    entries, cands = s._complete(root)
    s.cache[root.key()] = (1., entries, cands)
    child = M.Node(root, ("place", "d1", None), {}, frozenset({"d1"}), (s.B.entry_of("d1", wide),), (wide,), 1, 0)
    assert s._local(child) is None and id(wide) in s.toxic


def test_uct_prefers_the_better_child_after_equal_visits():
    s = object.__new__(M.MoveMCTS)
    s.vmin, s.vmax = .1, .2
    parent = M.Node(None, None, {}, frozenset(), (), (), 0, 0)
    parent.visits = 10
    good, bad = (M.Node(parent, ("place", "a", None), {}, frozenset(), (), (), 1, 0) for _ in range(2))
    good.visits = bad.visits = 5
    good.value_sum, bad.value_sum = 5 * .2, 5 * .1
    parent.children = [bad, good]
    assert s._select(parent) is good
    assert s._norm(.15) == pytest.approx(.5) and M.UCT_C == pytest.approx(1 / math.sqrt(2))
