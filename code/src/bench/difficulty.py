"""Level labels of a roster (plans/2026-11-01-merged-benchmark.md, Decisions: tiers = three parameters with three levels).

The roster comes from the explicit flags (--kinds, --count, --counter-allowance); the levels are labels computed from
them and stored in the sample's params (metadata.params), never a source of a roster:
    kinds      1 / 2 / all of the set's kinds     -> easy / medium / hard
    count      1-8 / 9-16 / 17-24 dishes          -> easy / medium / hard
    allowance  counter slack n+3 / n+2 / n+1      -> easy / medium / hard
A value outside the three levels is labelled None. n (the dishes that start on the counter) is drawn from the seed,
uniform in [N/4, 3N/4], unless --on-counter gives it.
"""
import math

LEVELS = ("easy", "medium", "hard")
COUNT_LEVELS = ((1, 8), (9, 16), (17, 24))
ALLOWANCE_LEVELS = {3: "easy", 2: "medium", 1: "hard"}


def roster(count):
    """Canonical object ids of a roster, e.g. plate_01..08, bowl_01..07: [(id, kind)]."""
    return [(f"{kind}_{k:02d}", kind) for kind, n in count.items() for k in range(1, n + 1)]


def labels(count, counter_allowance, set_kinds=3):
    """{kinds, count, allowance}: each parameter's level label (easy | medium | hard | None)."""
    n_kinds = sum(1 for n in count.values() if n > 0)
    total = sum(count.values())
    return {"kinds": "hard" if n_kinds >= set_kinds else LEVELS[n_kinds - 1] if 1 <= n_kinds <= 2 else None,
            "count": next((LEVELS[i] for i, (lo, hi) in enumerate(COUNT_LEVELS) if lo <= total <= hi), None),
            "allowance": ALLOWANCE_LEVELS.get(counter_allowance)}


def draw_n(count, seed):
    """Dishes on the counter at the start: uniform in [N/4, 3N/4], drawn from the seed and the roster's number of
    kinds (rng [seed, kinds - 1, 7]: for the three rosters of the old HOTEC tiers this is the old (tier, seed) draw)."""
    import numpy as np
    total = sum(count.values())
    kinds = sum(1 for n in count.values() if n > 0)
    rng = np.random.default_rng([int(seed), kinds - 1, 7])
    return int(rng.integers(math.ceil(total / 4), math.floor(3 * total / 4) + 1))


def resolve(kinds, count, on_counter=None, counter_allowance=1, set_kinds=3):
    """The sample's roster parameters from the explicit flags; ``levels`` are the labels. on_counter stays None when not
    given (the initial stage draws it); counter_allowance is the slack over on_counter."""
    kinds = list(kinds if kinds is not None else count)
    count = {k: int(count[k]) for k in kinds if k in count}
    return {"kinds": kinds, "count": count, "on_counter": on_counter, "counter_allowance": int(counter_allowance),
            "levels": labels(count, int(counter_allowance), set_kinds)}


def tag(params):
    """A roster tag for file names, e.g. plate8-bowl7_a2 (``_n<k>`` when --on-counter fixed n)."""
    t = "-".join(f"{k}{n}" for k, n in params["count"].items()) + f"_a{params['counter_allowance']}"
    return t + (f"_n{params['on_counter']}" if params.get("on_counter") is not None else "")
