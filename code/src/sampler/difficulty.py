"""Difficulty presets: the HOTEC bench's kind/allowance table, as data (moved from frigidaire_bench.py, which re-exports it).

easy = bowls, counter allowance n + 3; medium = + plates, n + 2; hard = + cups, n + 1. A preset fills --kinds, --count,
--counter-allowance (as slack over n) and draws --on-counter from (tier, seed); explicit flags override it.
"""
import math

# 7 bowls in every tier: the 8th-bowl poses settle past the tub wall (outer front-right) or wedge (third rear pair)
# in Isaac goal gates (2026-09-23); without plates an 8-bowl packing exists but first-fit found it once in 300
# shuffles, so open-track planners could not rack easy at all within 60 s
TIERS = {"easy": {"inventory": {"bowl": 7}, "slack": 3},
         "medium": {"inventory": {"plate": 8, "bowl": 7}, "slack": 2},     # was 1: one parking slot could not unwind the
         "hard": {"inventory": {"plate": 8, "bowl": 7, "cup": 8}, "slack": 1}}   # plate-bank chains on the 2026-09-28 racks (was 0)
TIER_INDEX = {name: i for i, name in enumerate(TIERS)}


def roster(tier):
    """Canonical object ids of a tier, e.g. plate_01..08, bowl_01..08."""
    return [(f"{kind}_{k:02d}", kind) for kind, n in TIERS[tier]["inventory"].items() for k in range(1, n + 1)]


def draw_n(tier, seed):
    """Dishes on the counter at the start: uniform in [N/4, 3N/4], drawn from (tier, seed) only."""
    import numpy as np
    total = sum(TIERS[tier]["inventory"].values())
    rng = np.random.default_rng([int(seed), TIER_INDEX[tier], 7])
    return int(rng.integers(math.ceil(total / 4), math.floor(3 * total / 4) + 1))


def cap_of(tier, n):
    return int(n) + TIERS[tier]["slack"]


def resolve(difficulty=None, kinds=None, count=None, on_counter=None, counter_allowance=None):
    """The sampler's dish parameters: a preset's values, overridden by every explicit one (None = not given).
    on_counter stays None when neither gives it (the initial stage draws it from (tier, seed)); counter_allowance is
    the slack over on_counter."""
    preset = TIERS.get(difficulty, {"inventory": {}, "slack": 0})
    count = dict(count if count is not None else preset["inventory"])
    kinds = list(kinds if kinds is not None else count)
    return {"difficulty": difficulty, "kinds": kinds, "count": {k: count[k] for k in kinds if k in count},
            "on_counter": on_counter, "counter_allowance": counter_allowance if counter_allowance is not None else preset["slack"]}
