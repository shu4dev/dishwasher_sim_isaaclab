#!/usr/bin/env python3
"""sample(dishwasher, dish set, seed) -> <out>/<machine>__<dishes>__<seed>.json: an initial and a goal configuration.

Stages, each stage(state, params) -> state over the pair file, which is rewritten after every stage with a `stages`
ledger entry (stage, params, code SHA, UTC time, PASS | FAIL | SKIP):
    load -> initial -> gate_initial -> build -> gate_goal -> certify -> score -> graph
Kit stages (initial, build, gate_goal, certify) run the existing Kit scripts through code/util/run_kit.sh; build packs
with numpy and score runs on the GPU, so run the whole loop inside the container:
    code/util/run_py.sh code/src/sampler/sample.py --difficulty hard --n-samples 1
Rerun stages on stored pairs (poses are stored, never re-derived from the seed):
    code/util/run_py.sh code/src/sampler/sample.py --from score --to graph --input <pair.json> [--variant score.reference=none]
A --variant run writes a new pair file tagged with the variant; the input pair is never overwritten.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
if __name__ == "__main__":                       # as a script: code/src instead of the script dir (no top-level 'build')
    sys.path[:1] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]

STAGES = ("load", "initial", "gate_initial", "build", "gate_goal", "certify", "score", "graph")
STAGE_FN = {"load": ("loader", "stage_load"), "initial": ("initial", "stage_initial"),
            "gate_initial": ("gate", "stage_gate_initial"), "build": ("build", "stage_build"),
            "gate_goal": ("gate", "stage_gate_goal"), "certify": ("goal", "stage_certify"),
            "score": ("score", "stage_score"), "graph": ("graph", "stage_graph")}
DEFAULT_COUNT = "plate=8,bowl=8,cup=8"


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--machine", default="frigidaire_fdpc4221as", help="data/assets/<machine>/machine.json")
    ap.add_argument("--dishes", default="hotec_wheatstraw", help="data/assets/<dishes>/<kind>.json")
    ap.add_argument("--kinds", help="comma-separated dish kinds, e.g. plate,bowl,cup (default: the preset's or --count's)")
    ap.add_argument("--count", help=f"kind=n,... (default {DEFAULT_COUNT}, or the preset's); capped by the catalog")
    ap.add_argument("--on-counter", type=int, help="dishes that start in counter stacks (default: drawn by the preset)")
    ap.add_argument("--counter-allowance", type=int, help="counter capacity beyond --on-counter (default: the preset's)")
    ap.add_argument("--difficulty", choices=("easy", "medium", "hard"),
                    help="preset (the HOTEC bench table): easy = bowls, n+3; medium = + plates, n+2; hard = + cups, n+1")
    ap.add_argument("--n-samples", type=int, default=1, help="valid pairs wanted")
    ap.add_argument("--max-attempts", type=int, default=6, help="seeds tried at most (seed, seed+1, ...)")
    ap.add_argument("--max-hours", type=float, default=3., help="stop starting new seeds after this long")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=ROOT / "data/results/sampler")
    ap.add_argument("--from", dest="start", choices=STAGES, default=STAGES[0], help="first stage to run")
    ap.add_argument("--to", dest="stop", choices=STAGES, default=STAGES[-1], help="last stage to run")
    ap.add_argument("--input", nargs="*", type=Path, default=[], help="stored pair files to rerun --from..--to on")
    ap.add_argument("--variant", action="append", default=[], metavar="KEY=VALUE",
                    help="swap one stage's rule: build.order, build.release, build.plates, gate, score.reference, "
                         "graph, initial.on_counter")
    return ap.parse_args(argv)


def kv(text):
    return {k: v for k, v in (item.split("=", 1) for item in text.split(",") if item)} if text else {}


def code_sha():
    import subprocess
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip() or None
    except OSError:
        return None


def utc():
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def write(path, state):
    from sampler import loader
    loader.validate(state, "pair")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(state, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    os.replace(tmp, path)


def new_pair(args, seed):
    from sampler import difficulty
    count = kv(args.count) if args.count else (None if args.difficulty else kv(DEFAULT_COUNT))
    params = difficulty.resolve(args.difficulty, args.kinds.split(",") if args.kinds else None,
                                {k: int(v) for k, v in count.items()} if count is not None else None,
                                args.on_counter, args.counter_allowance)
    return {"schema_version": 1, "machine": args.machine, "dishes": args.dishes, "seed": seed, "variant": None,
            "params": params, "load": None, "initial": None, "goal": None, "stages": []}


def run_stages(state, path, stages, variants, max_attempts=6):
    """Run ``stages`` in order on ``state``, writing ``path`` after each; stops at the first FAIL. Returns True when
    none failed."""
    import importlib
    if "initial.on_counter" in variants:
        state["params"]["on_counter"] = int(variants["initial.on_counter"])
    params = {**variants, "work": str(path.parent / "_work" / path.stem), "max_attempts": max_attempts}
    for stage in stages:
        module, name = STAGE_FN[stage]
        fn = getattr(importlib.import_module(f"sampler.{module}"), name)
        try:
            state, result, note = fn(state, params)
        except Exception as exc:                                 # noqa: BLE001  the ledger records it
            result, note = "FAIL", f"exception {exc!r}"
        state["stages"].append({"stage": stage, "params": {k: v for k, v in params.items() if k != "work"},
                                "code_sha": code_sha(), "utc": utc(), "result": result, "note": note})
        write(path, state)
        print(f"[{stage}] {result} {note}", flush=True)
        if result == "FAIL":
            return False
    return True


def main(argv=None):
    args = parse_args(argv)
    stages = STAGES[STAGES.index(args.start):STAGES.index(args.stop) + 1]
    if not stages:
        raise SystemExit("[RESULT] FAIL --from comes after --to")
    variants = kv(",".join(args.variant))
    tag = "__".join(f"{k}={v}" for k, v in sorted(variants.items())).replace("/", "-")
    args.out.mkdir(parents=True, exist_ok=True)
    if args.input:
        valid = 0
        for src in args.input:
            state = json.loads(src.read_text())
            path = src.with_name(f"{src.stem}__{tag}.json") if tag else src
            if tag:
                state["variant"] = tag
            valid += run_stages(state, path, stages, variants, args.max_attempts)
        print(f"[RESULT] {'PASS' if valid == len(args.input) else 'FAIL'} {valid} of {len(args.input)} reruns passed")
        return
    started, valid = time.monotonic(), 0
    for seed in range(args.seed, args.seed + args.max_attempts):
        if valid >= args.n_samples or time.monotonic() - started > args.max_hours * 3600:
            break
        path = args.out / f"{args.machine}__{args.dishes}__{seed}{'__' + tag if tag else ''}.json"
        state = new_pair(args, seed)
        state["variant"] = tag or None
        valid += run_stages(state, path, stages, variants, args.max_attempts)
    print(f"[RESULT] {'PASS' if valid >= args.n_samples else 'FAIL'} {valid} of {args.n_samples} pairs, stages {stages[0]}..{stages[-1]}")


if __name__ == "__main__":
    main()
