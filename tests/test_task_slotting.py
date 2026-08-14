# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The shared slot-assignment rule (task/slotting.py).

Two properties are load-bearing enough to pin with fixtures rather than assertions about
intent:

1. **Equivalence with the pre-refactor runner algorithm.** ``run_task._assign_slots`` was
   extracted verbatim into :func:`dishsim.task.slotting.assign_slots`; a copy of the original
   loop lives here and both are run on the same synthetic inputs. For every non-plate mode the
   outputs must be identical — the extraction is a move, not a change.
2. **The plate-slot occupancy change is deliberate.** The old circle rule read a plate as a
   ~70 mm sphere, so adjacent 40-50 mm tine gaps could never both be assigned and a "full
   load" of plates capped at every-third-gap. Distinct gaps are now compatible by design
   (the tine bank's pitch is the separation); the same gap stays one receptacle.
"""

import numpy as np
import pytest

from dishsim.placement import SlotFrame
from dishsim.task import slotting


def _slot(sid, xyz, mode="floor_stand", width=0.06):
    T = np.eye(4)
    T[:3, 3] = xyz
    return SlotFrame(sid, T, width, "derived", mode=mode)


def _grid_slots(nx=3, ny=2, pitch=0.06, mode="floor_stand"):
    slots = {}
    for j in range(ny):
        for i in range(nx):
            sid = j * nx + i
            slots[sid] = _slot(sid, (0.3 + i * pitch, -0.1 + j * pitch, 0.0), mode=mode)
    return slots


def _legacy_assign(items, slots_by_class, goal_sets, slot_names_by_class, *,
                   type_slots, slot_pools, margin, mode_of, radius_of):
    """Verbatim copy of the pre-refactor run_task._assign_slots loop (circle rule for all
    modes). The equivalence fixture: slotting.assign_slots must match this on every
    non-plate input."""
    feasible, sources = {}, {}
    for it in items:
        cls = it.object_class
        mode = mode_of(cls)
        names = (type_slots or {}).get(cls)
        pool = slot_pools.get(mode)
        if names is not None:
            by_name = slot_names_by_class[cls]
            unknown = [n for n in names if n not in by_name]
            if unknown:
                raise SystemExit("unknown names")
            ids, sources[it.item_id] = [by_name[n] for n in names], "type_slots"
        elif pool is not None:
            ids, sources[it.item_id] = list(pool), "slot_pools"
        else:
            ids, sources[it.item_id] = sorted(slots_by_class[cls]), "all"
        feasible[it.item_id] = [s for s in ids if len(goal_sets[cls].get(s, ())) > 0]

    out, taken = {}, []
    for it in sorted(items, key=lambda i: (len(feasible[i.item_id]), i.item_id)):
        r = float(radius_of(it.object_class))
        for sid in feasible[it.item_id]:
            slot = slots_by_class[it.object_class][sid]
            centre = slot.T_base_slot[:3, 3]
            if any(float(np.linalg.norm(centre - c)) < (r + rr + margin) for c, rr in taken):
                continue
            taken.append((centre, r))
            out[it.item_id] = slot
            break
    return out


def _req(*pairs):
    return [slotting.SlotRequest(f"{cls}_{i:02d}", cls) for i, cls in enumerate(pairs)]


CUP_R = 0.033
GOALS = [[0.0] * 6]  # any non-empty goal set


class TestEquivalenceWithLegacy:
    """Extraction is a move, not a change: non-plate inputs must match the legacy loop."""

    def _run_both(self, reqs, slots_by_class, goal_sets, names_by_class, type_slots=None,
                  slot_pools=None, margin=0.010, radii=None):
        radii = radii or {}
        pools = slot_pools if slot_pools is not None else {"floor_stand": None}
        kw = dict(type_slots=type_slots, slot_pools=pools, margin_m=margin,
                  mode_fn=lambda c: "floor_stand",
                  rim_radius_fn=lambda c: radii.get(c, CUP_R))
        new = slotting.assign_slots(reqs, slots_by_class, goal_sets, names_by_class, **kw)
        old = _legacy_assign(reqs, slots_by_class, goal_sets, names_by_class,
                             type_slots=type_slots, slot_pools=pools, margin=margin,
                             mode_of=lambda c: "floor_stand",
                             radius_of=lambda c: radii.get(c, CUP_R))
        assert {k: v.slot_id for k, v in new.items()} == {k: v.slot_id for k, v in old.items()}
        return new

    def test_floor_grid_two_classes(self):
        slots = _grid_slots()
        sbc = {"cup": slots, "tumbler": slots}
        gs = {"cup": {0: GOALS, 1: GOALS, 4: GOALS}, "tumbler": {s: GOALS for s in slots}}
        names = {"cup": {}, "tumbler": {}}
        out = self._run_both(_req("cup", "tumbler", "tumbler"), sbc, gs, names)
        # the constrained cup is served first and everyone lands somewhere non-overlapping
        assert len(out) == 3

    def test_most_constrained_first_is_load_bearing(self):
        # cup can use ONLY slot 0; tumbler can use 0 and 5 (far apart). Serving the tumbler
        # first by id would take slot 0 and strand the cup.
        slots = {0: _slot(0, (0.3, 0.0, 0.0)), 5: _slot(5, (0.9, 0.0, 0.0))}
        sbc = {"cup": slots, "tumbler": slots}
        gs = {"cup": {0: GOALS}, "tumbler": {0: GOALS, 5: GOALS}}
        names = {"cup": {}, "tumbler": {}}
        out = self._run_both(_req("tumbler", "cup"), sbc, gs, names)
        assert out["cup_01"].slot_id == 0 and out["tumbler_00"].slot_id == 5

    def test_empty_type_slots_list_means_nowhere(self):
        slots = _grid_slots()
        sbc, names = {"cup": slots}, {"cup": {"near_left": 0}}
        gs = {"cup": {s: GOALS for s in slots}}
        out = self._run_both(_req("cup"), sbc, gs, names, type_slots={"cup": []})
        assert out == {}

    def test_overlap_rejects_adjacent_cells(self):
        slots = _grid_slots(nx=2, ny=1)  # two cells 60 mm apart
        sbc = {"cup": slots}
        gs = {"cup": {s: GOALS for s in slots}}
        out = self._run_both(_req("cup", "cup"), sbc, gs, {"cup": {}},
                             radii={"cup": 0.033})  # 33+33+10 mm margin > 60 mm pitch
        assert len(out) == 1

    def test_determinism(self):
        slots = _grid_slots()
        sbc = {"cup": slots, "tumbler": slots}
        gs = {c: {s: GOALS for s in slots} for c in sbc}
        names = {c: {} for c in sbc}
        reqs = _req("tumbler", "cup", "cup", "tumbler")
        a = slotting.assign_slots(reqs, sbc, gs, names, type_slots=None,
                                  slot_pools={"floor_stand": None}, margin_m=0.01,
                                  mode_fn=lambda c: "floor_stand", rim_radius_fn=lambda c: CUP_R)
        b = slotting.assign_slots(reqs, sbc, gs, names, type_slots=None,
                                  slot_pools={"floor_stand": None}, margin_m=0.01,
                                  mode_fn=lambda c: "floor_stand", rim_radius_fn=lambda c: CUP_R)
        assert {k: v.slot_id for k, v in a.items()} == {k: v.slot_id for k, v in b.items()}

    def test_unknown_type_slot_name_fails_loudly(self):
        slots = _grid_slots()
        with pytest.raises(SystemExit):
            slotting.assign_slots(_req("cup"), {"cup": slots}, {"cup": {0: GOALS}},
                                  {"cup": {}}, type_slots={"cup": ["no_such_slot"]},
                                  slot_pools={"floor_stand": None}, margin_m=0.01,
                                  mode_fn=lambda c: "floor_stand",
                                  rim_radius_fn=lambda c: CUP_R)


class TestPlateOccupancy:
    """The documented v2 change: distinct tine gaps are compatible, the same gap is not."""

    def _bank(self, pitch=0.050, n=4):
        return {i: _slot(i, (0.3 + i * pitch, 0.1, 0.0), mode="plate_slot", width=pitch)
                for i in range(n)}

    def _assign_plates(self, n_plates, slots):
        gs = {"plate": {s: GOALS for s in slots}}
        return slotting.assign_slots(
            _req(*(["plate"] * n_plates)), {"plate": slots}, gs, {"plate": {}},
            type_slots=None, slot_pools={"plate_slot": None}, margin_m=0.010,
            mode_fn=lambda c: "plate_slot", rim_radius_fn=lambda c: 0.0706)

    def test_adjacent_gaps_both_assignable(self):
        out = self._assign_plates(4, self._bank())
        assert len(out) == 4  # every gap fills; the old circle rule capped this at 2
        assert len({v.slot_id for v in out.values()}) == 4

    def test_same_gap_stays_one_receptacle(self):
        out = self._assign_plates(2, self._bank(n=1))
        assert len(out) == 1

    def test_old_circle_rule_documented_as_replaced(self):
        # the legacy loop on the same bank: 70.6 mm radii + 10 mm margin demand 151.2 mm of
        # separation, and even gaps 0 and 3 sit only 150 mm apart — ONE plate per 4-gap bank
        # was the measured deficiency that forced the change
        slots = self._bank()
        gs = {"plate": {s: GOALS for s in slots}}
        old = _legacy_assign(_req("plate", "plate", "plate", "plate"), {"plate": slots}, gs,
                             {"plate": {}}, type_slots=None, slot_pools={"plate_slot": None},
                             margin=0.010, mode_of=lambda c: "plate_slot",
                             radius_of=lambda c: 0.0706)
        assert len(old) == 1  # the behavior this change retires

    def test_plate_vs_standing_neighbour_keeps_circle_rule(self):
        # a floor cell 60 mm from a plate gap: mixed pair stays conservative
        slots_p = {0: _slot(0, (0.30, 0.10, 0.0), mode="plate_slot")}
        slots_c = {0: _slot(0, (0.30, 0.16, 0.0), mode="floor_stand")}
        gs = {"plate": {0: GOALS}, "cup": {0: GOALS}}
        modes = {"plate": "plate_slot", "cup": "floor_stand"}
        radii = {"plate": 0.0706, "cup": 0.033}
        out = slotting.assign_slots(
            [slotting.SlotRequest("plate_00", "plate"), slotting.SlotRequest("cup_00", "cup")],
            {"plate": slots_p, "cup": slots_c}, gs, {"plate": {}, "cup": {}},
            type_slots=None, slot_pools={"plate_slot": None, "floor_stand": None},
            margin_m=0.010, mode_fn=modes.__getitem__, rim_radius_fn=radii.__getitem__)
        assert len(out) == 1  # 70.6 + 33 + 10 mm > 60 mm separation


class TestEffectiveModeLookup:
    """The Bosch latent-bug fix: pool lookup must follow the EFFECTIVE mode.

    Written failing-first against the old behavior: the pre-fix runner looked up
    ``slot_pools`` by the RAW registry mode, so a machine override (Bosch fork:
    ``basket_drop`` -> ``flat_lay_third``) silently missed a pool keyed by the effective
    mode. ``assign_slots`` receives ``mode_fn`` — the runner now passes
    ``config.effective_placement_mode`` — and this test locks the contract.
    """

    def test_pool_keyed_by_effective_mode_is_found(self):
        slots = {i: _slot(i, (0.3 + 0.06 * i, 0.0, 0.0), mode="flat_lay_third")
                 for i in range(3)}
        gs = {"fork": {s: GOALS for s in slots}}
        # a pool restricting forks to slot 2, keyed by the EFFECTIVE mode
        pools = {"flat_lay_third": [2], "basket_drop": None}
        out_effective = slotting.assign_slots(
            _req("fork"), {"fork": slots}, gs, {"fork": {}}, type_slots=None,
            slot_pools=pools, margin_m=0.01,
            mode_fn=lambda c: "flat_lay_third",  # what effective_placement_mode returns
            rim_radius_fn=lambda c: 0.0082)
        assert out_effective["fork_00"].slot_id == 2
        # the raw-mode lookup (the old bug) falls through to "all" and takes slot 0 instead
        out_raw = slotting.assign_slots(
            _req("fork"), {"fork": slots}, gs, {"fork": {}}, type_slots=None,
            slot_pools=pools, margin_m=0.01,
            mode_fn=lambda c: "basket_drop",
            rim_radius_fn=lambda c: 0.0082)
        assert out_raw["fork_00"].slot_id == 0


class TestTakenSeed:
    """The capacity planner grows a load incrementally through the ``taken`` parameter."""

    def test_pre_occupied_cell_is_respected(self):
        # radii small enough that ADJACENT cells coexist (20+20+10 mm < 60 mm pitch), so the
        # only thing steering the outcome is the seeded occupancy of cell 0
        slots = _grid_slots(nx=2, ny=1)
        gs = {"cup": {s: GOALS for s in slots}}
        occupied = [("floor_stand", slots[0].T_base_slot[:3, 3], 0.020)]
        out = slotting.assign_slots(
            _req("cup"), {"cup": slots}, gs, {"cup": {}}, type_slots=None,
            slot_pools={"floor_stand": None}, margin_m=0.010,
            mode_fn=lambda c: "floor_stand", rim_radius_fn=lambda c: 0.020,
            taken=occupied)
        assert out["cup_00"].slot_id == 1
