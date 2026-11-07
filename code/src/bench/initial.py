"""initial stage: the start generator (frigidaire_bench_kit.py --start, Kit, by subprocess): n dishes in messy counter
stacks on the worktop slab, the rest dropped into the extended racks and settled, with the reproduction and unstack
rehearsals (code/frigidaire/docs/hotec_bench.md, Start generation). The roster is the sample's explicit count; n is
--on-counter or drawn from the seed (difficulty.draw_n), the counter cap is n + the allowance. The initial block comes
from the start record (sample.initial_of); its path and settle figures go to metadata.provenance.
"""
import json
from pathlib import Path

from . import rel, run
from .difficulty import draw_n
from .sample import initial_of

KIT = "code/planner/frigidaire/frigidaire_bench_kit.py"


def stage_initial(state, params):
    meta = state["metadata"]
    p, seed = meta["params"], meta["seed"]
    n = p["on_counter"] if p["on_counter"] is not None else draw_n(p["count"], seed)
    cap = n + p["counter_allowance"]
    count = ",".join(f"{k}={v}" for k, v in p["count"].items())
    out = Path(params["work"]) / "initial"
    for attempt in range(params.get("max_attempts", 6)):
        ok, _ = run("kit", [KIT, "--start", "--count", count, "--n", n, "--cap", cap, "--seed", seed,
                            "--attempt", attempt, "--out", rel(out), "--headless"], out / "logs" / f"start_a{attempt}.log")
        path = out / "instances" / "attempts" / f"s{seed}_a{attempt}" / "start.json"
        if ok and path.is_file():
            start = json.loads(path.read_text())
            state["initial"] = initial_of(start)
            meta["provenance"].update(initial_record=rel(path), settle=start["validation"], gate_initial=None)
            return state, "PASS", f"attempt {attempt}: {start['counter']['start_count']} on the counter, cap {cap}"
    return state, "FAIL", "no start attempt passed"
