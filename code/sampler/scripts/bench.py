#!/usr/bin/env python3
"""The benchmark CLI: Frigidaire twin + HOTEC set, a messy start, the fit-score-prove goal as the ground truth, four
algorithms (plans/2026-11-01-merged-benchmark.md, plans/2026-11-02-fit-score-prove.md; stages in code/sampler/sample.py).

The roster comes from the explicit flags (--kinds, --count, --counter-allowance); its level labels (kinds 1 / 2 / all,
count 1-8 / 9-16 / 17-24, allowance n+3 / n+2 / n+1 = easy / medium / hard) are stored in the sample. Up to
--max-attempts seeds are tried (seed, seed+1, ...) until --n-samples samples pass every stage asked for. Not a Kit
script: run it inside the container (its Kit stages call code/util/run_kit.sh themselves):

    code/util/run_py.sh code/sampler/scripts/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report \\
        --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
    code/util/run_py.sh code/sampler/scripts/bench.py --from run --to report --input data/results/bench/v1/<sample>.json

Samples (metadata, initial, goal, dag; plans/2026-11-02-sample-schema.md) and index.jsonl go to --out, the algorithms'
results to <out>/runs/<sample>.json, stage work to <out>/_work/<sample>/, media (stills, videos, the report page) to the
matching data/media/ folder (data/results/bench/v1 -> data/media/bench/v1). --to stills stops after the two stills.
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:1] = [str(ROOT / "code"), str(ROOT / "code/util")]   # not code/sampler/scripts: the CLI's own folder stays off sys.path

from sampler import sample  # noqa: E402

STAGES = sample.STAGES


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--machine", default="frigidaire_fdpc4221as", help="data/assets/<machine>/machine.json")
    ap.add_argument("--dishes", default="hotec_wheatstraw", help="data/assets/<dishes>/<kind>.json")
    ap.add_argument("--kinds", help="comma-separated dish kinds, e.g. plate,bowl,cup (default: --count's)")
    ap.add_argument("--count", help=f"kind=n,... (default {sample.DEFAULT_COUNT}); capped by the catalog")
    ap.add_argument("--on-counter", type=int, help="dishes that start in counter stacks (default: drawn from the seed)")
    ap.add_argument("--counter-allowance", type=int, default=1, help="counter capacity beyond --on-counter (slack)")
    ap.add_argument("--algorithms", default=",".join(sample.ALGORITHMS),
                    help=f"run stage: comma-separated, of {','.join(sample.ALGORITHMS)}")
    ap.add_argument("--n-samples", type=int, default=1, help="valid samples wanted")
    ap.add_argument("--max-attempts", type=int, default=6, help="seeds tried at most (seed, seed+1, ...)")
    ap.add_argument("--max-hours", type=float, default=3., help="stop starting new seeds after this long")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=ROOT / "data/results/bench/v1")
    ap.add_argument("--media", type=Path, help="media root (default: --out's data/media twin)")
    ap.add_argument("--wait-ceilings", type=float, default=0., metavar="MIN",
                    help="build: wait up to MIN minutes for the 6.2 ceilings file (a box run computes it alongside)")
    ap.add_argument("--from", dest="start", choices=STAGES, default=STAGES[0], help="first stage to run")
    ap.add_argument("--to", dest="stop", choices=STAGES, default=STAGES[-1], help="last stage to run")
    ap.add_argument("--input", nargs="*", type=Path, default=[], help="stored sample files to rerun --from..--to on")
    ap.add_argument("--hardware", type=Path, metavar="MOUNT_JSON",
                    help="fit: the hardware-aware goal (plans/2026-11-05-hardware-aware-sampler.md, sampler/hwgate.py) at this "
                         "mount file; the sample id gets __hw")
    ap.add_argument("--hardware-basket", choices=("in", "out"), default="in",
                    help="--hardware: the robot's gate with the SilverwareBasket in (default) or lifted out (--no-basket; the "
                         "sample id gets __hwnb; the Isaac stages keep the basket)")
    ap.add_argument("--hardware-max-iter", type=int, default=None,
                    help="--hardware: the loop's iteration cap (default sampler/hwgate.HW_MAX_ITER, the plan's 6)")
    ap.add_argument("--start-from", type=Path, metavar="SAMPLE_JSON",
                    help="copy this sample's metadata and initial block (its load, initial and gate_initial copied in the "
                         "ledger) instead of running those stages; its seed")
    ap.add_argument("--variant", action="append", default=[], metavar="KEY=VALUE",
                    help="swap one stage's rule: gate, score.reference, graph, initial.on_counter")
    args = ap.parse_args(argv)
    bad = [a for a in args.algorithms.split(",") if a and a not in sample.ALGORITHMS]
    if bad:
        ap.error(f"--algorithms: unknown {bad}")
    return args


def media_root(out, media=None):
    """--media, else data/media/<x> for an --out under data/results/<x>, else <out>/media."""
    if media:
        return Path(media).absolute()
    out = Path(out).absolute()
    try:
        return ROOT / "data/media" / out.relative_to(ROOT / "data/results")
    except ValueError:
        return out / "media"


def main(argv=None):
    args = parse_args(argv)
    stages = STAGES[STAGES.index(args.start):STAGES.index(args.stop) + 1]
    if not stages:
        raise SystemExit("[RESULT] FAIL --from comes after --to")
    variants = sample.kv(",".join(args.variant))
    tag = "__".join(f"{k}={v}" for k, v in sorted(variants.items())).replace("/", "-")
    extra = {"algorithms": [a for a in args.algorithms.split(",") if a], "ceilings_wait_s": args.wait_ceilings * 60.,
             "out": str(Path(args.out).absolute()), "media": str(media_root(args.out, args.media))}
    if args.hardware:
        extra["hardware"] = sample.rel_root(args.hardware)
        extra["hardware_basket"] = args.hardware_basket
        extra["hardware_max_iter"] = args.hardware_max_iter
    if args.input:
        valid = 0
        for src in args.input:
            state = json.loads(src.read_text())
            if "metadata" not in state:
                raise SystemExit(f"[RESULT] FAIL {src} is a schema-1 pair file (python -m sampler.convert its folder)")
            path = src.with_name(f"{src.stem}__{tag}.json") if tag else src
            if tag:
                state["metadata"]["variant"] = tag
                run = sample.read_run(src)
                if run:                                           # the variant carries the input's runs file along
                    sample.write_run(path, {**run, "sample_id": path.stem})
            extra["out"] = str(path.parent.absolute())
            valid += sample.run_stages(state, path, stages, variants, args.max_attempts, extra)
        print(f"[RESULT] {'PASS' if valid == len(args.input) else 'FAIL'} {valid} of {len(args.input)} reruns passed")
        return
    args.out.mkdir(parents=True, exist_ok=True)
    # a seed is spent on the sampling stages (load..graph); a sample that passed them goes on to stills, run, video and
    # report, and a failure there is that sample's (no new seed is drawn for it)
    sampling = [s for s in stages if STAGES.index(s) <= STAGES.index("graph")]
    rest = [s for s in stages if s not in sampling]
    started, valid, ran = time.monotonic(), 0, 0
    if args.start_from:                               # the start is copied: one sample, the source's seed, no start stages
        sampling = [s for s in sampling if s not in ("load", "initial", "gate_initial")]
    seeds = [None] if args.start_from else range(args.seed, args.seed + args.max_attempts)
    for seed in seeds:
        if valid >= args.n_samples or time.monotonic() - started > args.max_hours * 3600:
            break
        state = sample.new_sample(args, seed)
        if args.start_from:
            state = sample.copy_start(state, args.start_from)
        state["metadata"]["variant"] = tag or None
        path = sample.sample_path(args.out, state, "__".join(t for t in (tag, ("hwnb" if args.hardware_basket == "out" else "hw") if args.hardware else "") if t))
        ok = sample.run_stages(state, path, sampling, variants, args.max_attempts, extra) if sampling else True
        valid += ok
        if ok and rest:
            ran += sample.run_stages(state, path, rest, variants, args.max_attempts, extra)
    if not valid and "report" in stages:              # an unsampled roster still shows on the page (failing stage, reason)
        from paths import report
        print(f"[report] {report.write_page(extra['out'], extra['media'])}", flush=True)
    tail = f", {ran} through {rest[-1]}" if rest else ""
    print(f"[RESULT] {'PASS' if valid >= args.n_samples and (not rest or ran >= args.n_samples) else 'FAIL'} "
          f"{valid} of {args.n_samples} samples, stages {stages[0]}..{stages[-1]}{tail}")


if __name__ == "__main__":
    main()
