# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Benchmark difficulty tiers: 3 presets + one-knob ablations off medium (12 cells).

Module constants, deliberately OUTSIDE ``config.py`` — ``geometry.config_hash()`` digests
named ``config`` attributes only, so nothing here can ever invalidate a collision cache
(same guarantee the ``rearrange.py`` fault knobs rely on).

Cell fields:
    classes:       ``{object_class: count}`` — the roster, produced by
                   ``capacity.plan_full_load(classes=, per_class_cap=)``.
    counter_cap:   max SIMULTANEOUS objects inside the counter band
                   (``rearrange.in_counter_band``); enforced at generation (initial states),
                   by the episode driver (counter-full refusal), and by the cap-aware
                   optimum (``compat.optimal_moves(counter_cap=)``).
    displace:      fraction of the roster displaced at start (1.0 = all);
                   ``n = ceil(frac * n_items)``, at least 1.
    no_goal_squat: displaced items may not START on another item's goal slot (easy only).
    cycles:        authored swap-cycle lengths (``instance_gen.author_cycles``); () = none.
    settle_bar:    per-call override of ``capacity.SETTLE_RELIABILITY_BAR`` (None = default
                   0.90). 0.70 admits the tumbler (measured 64/88 = 72.7%); cup never
                   appears in any cell's class list, so nothing else is re-admitted.

STAGE-2 FOLLOW-ON (documented, not implemented): a cutlery basket on the Bosch lower rack
for silverware. Enable = add a ``"basket"`` sub-dict to ``_RACK_GEN_BOSCH["E_shelf_1_04"]``
(baseline template in config.py; rear-right patch x 0.30-0.46 / y 0.30-0.46 measured to
cost zero bowl slots) + delete ``MACHINE_PLACEMENT_MODE_OVERRIDES["bosch800"]`` + add fork
to ``POLICIES["plates_first"]["placement"]``. ``RACK_GEN`` is HASHED: all bosch caches
invalidate at once → rebake ~3-5 min/context (3 for placement-only, 14 for all states),
plus extract fork/placement at side_winner (the existing fork cache was baked @ front),
regenerate instances, re-pin test_compat optima. One slot PER BAY (one bay per fork).
Fork drop-column safety at the Bosch basket position is UNMEASURED.
"""

import math

PRESETS = ("easy", "medium", "hard")

CELLS: dict[str, dict] = {
    "easy":   {"classes": {"bowl": 5},                           "counter_cap": 6,
               "displace": 1 / 3, "no_goal_squat": True,  "cycles": (),     "settle_bar": None},
    "medium": {"classes": {"plate": 5, "bowl": 5},               "counter_cap": 3,
               "displace": 1 / 2, "no_goal_squat": False, "cycles": (),     "settle_bar": None},
    "hard":   {"classes": {"plate": 6, "bowl": 6, "tumbler": 3}, "counter_cap": 1,
               "displace": 1.0,   "no_goal_squat": False, "cycles": (2, 3), "settle_bar": 0.70},
}


def _med(**kw):
    c = dict(CELLS["medium"])
    c.update(kw)
    return c


CELLS.update({
    # count knob (mix stays near-half; count15 is the plan's own placement maximum 7p+8b)
    "med_count5":      _med(classes={"plate": 2, "bowl": 3}),
    "med_count15":     _med(classes={"plate": 7, "bowl": 8}),
    # types knob. Single-class tops out at the certified per-class maximum (bowl 8 /
    # plate 7 on this rack — separation margins), so the single-class ablation runs at
    # count 8, not medium's 10; the aggregator reports n_items per cell, and the count
    # confound is called out in the summary caption.
    "med_types_bowls": _med(classes={"bowl": 8}),
    "med_types_mix":   _med(classes={"plate": 4, "bowl": 3, "tumbler": 3}, settle_bar=0.70),
    # counter-cap knob
    "med_cap6":        _med(counter_cap=6),
    "med_cap1":        _med(counter_cap=1),
    "med_cap0":        _med(counter_cap=0),
    # displacement knob
    "med_disp_third":  _med(displace=1 / 3),
    "med_disp_allcyc": _med(displace=1.0, cycles=(2, 3)),
})


def n_displace(cell_name: str, n_items: int) -> int:
    """Displaced-item count for a cell: ``ceil(frac * n)``, clamped to [1, n]."""
    return min(n_items, max(1, math.ceil(CELLS[cell_name]["displace"] * n_items)))
