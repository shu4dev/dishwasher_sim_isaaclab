"""initial stage: the HOTEC bench's start generator (frigidaire_bench_kit.py --start, Kit, by subprocess): n dishes in
messy counter stacks on the worktop slab, the rest dropped into the extended racks and settled, with the bench's
reproduction and unstack rehearsals. Its generator takes a tier and a seed only, so the dish parameters must be a
difficulty preset (n is drawn from (tier, seed) by difficulty.draw_n).
"""
import json
from pathlib import Path

from . import rel, run
from .difficulty import TIERS

KIT = "code/planner/frigidaire/frigidaire_bench_kit.py"


def stage_initial(state, params):
    p, tier, seed = state["params"], state["params"].get("difficulty"), state["seed"]
    preset = TIERS.get(tier)
    if preset is None or p["count"] != preset["inventory"] or p["counter_allowance"] != preset["slack"] \
            or p["on_counter"] is not None:
        return state, "FAIL", "the bench start generator takes --difficulty only (no explicit count/on-counter/allowance)"
    out = Path(params["work"]) / "initial"
    for attempt in range(params.get("max_attempts", 6)):
        ok, _ = run("kit", [KIT, "--start", "--tier", tier, "--seed", seed, "--attempt", attempt, "--out", rel(out),
                            "--headless"], out / "logs" / f"start_a{attempt}.log")
        path = out / "instances" / "attempts" / f"{tier}_s{seed}_a{attempt}" / "start.json"
        if ok and path.is_file():
            start = json.loads(path.read_text())
            state["initial"] = {"source": rel(path), "objects": start["objects"],
                                "poses": start["initial_snapshot"]["poses"],
                                "rack_joints": start["initial_snapshot"]["joints"], "counter": start["counter"],
                                "support": start["support"], "settle": start["validation"], "gate": None}
            return state, "PASS", f"attempt {attempt}: {start['counter']['start_count']} on the counter"
    return state, "FAIL", "no start attempt passed"
