# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Generate the planner instance set: one fresh Kit process per attempt, re-rolling failures.

    scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_planner_generate.py --seeds 0 1 2 3 4 --sizes 9 18

Kit-free driver (runs inside the container, spawns scripts/run_kit.sh per attempt). Skips
instances that already exist. Judges each attempt by its [RESULT] line, never the exit code.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    p.add_argument("--sizes", type=int, nargs="+", default=[9, 18])
    p.add_argument("--max-attempts", type=int, default=3)
    p.add_argument("--start-attempt", type=int, default=0, help="first attempt index (attempts seed the draw; skip ones already tried)")
    p.add_argument("--out-dir", type=Path, default=ROOT / "results/planner/frigidaire/instances")
    p.add_argument("--max-wall-seconds", type=float, default=300)
    p.add_argument("--cutlery-start", choices=("auto", "basket"), default="auto")
    p.add_argument("--counter-share", type=float, default=.5)
    args = p.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "logs").mkdir(exist_ok=True)
    summary = []
    for n in args.sizes:
        for seed in args.seeds:
            target = args.out_dir / f"s{seed}_n{n}.json"
            if target.exists():
                print(f"[SKIP] {target.name} exists", flush=True)
                summary.append((target.name, "exists", 0))
                continue
            status = "failed"
            for attempt in range(args.start_attempt, args.start_attempt + args.max_attempts):
                log = args.out_dir / "logs" / f"s{seed}_n{n}_a{attempt}.log"
                cmd = [str(ROOT / "scripts/run_kit.sh"), str(ROOT / "frigidaire/scripts/experiment/frigidaire_planner_instances.py"),
                       "--seed", str(seed), "--n-objects", str(n), "--attempt", str(attempt), "--out-dir", str(args.out_dir),
                       "--max-wall-seconds", str(args.max_wall_seconds), "--cutlery-start", args.cutlery_start,
                       "--counter-share", str(args.counter_share),
                       "--headless", "--device", "cpu"]
                started = time.time()
                with log.open("w") as f:
                    subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=args.max_wall_seconds + 240)
                result = next((line for line in log.read_text().splitlines() if line.startswith("[RESULT]")), "[RESULT] none")
                print(f"[ATTEMPT] s{seed}_n{n} a{attempt}: {result} ({time.time() - started:.0f}s)", flush=True)
                if "[RESULT] PASS" in result:
                    status = f"pass@{attempt}"
                    break
            summary.append((target.name, status, attempt + 1))
    print("[SUMMARY] " + ", ".join(f"{name}: {status}" for name, status, _ in summary))
    print("[RESULT] " + ("PASS" if all(s != "failed" for _, s, _ in summary) else "FAIL"))


if __name__ == "__main__":
    main()
