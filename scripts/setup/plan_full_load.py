#!/workspace/isaaclab/env_isaaclab/bin/python
# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Plan the reachable full load — the machine's capacity counted by what the arm can place.

Kit-free (venv python): reads the baked ``slots/goal_sets.json`` caches, certifies the load
jointly (every item keeps a collision-free goal config with all earlier items resting at
their goals), applies the transition z-budget gate, and writes the plan artifact the
full-load episode consumes plus the reach-map figure. See :mod:`dishsim.capacity` for what
"counted honestly" means.

    scripts/setup/plan_full_load.py --machine bosch800 --placement side_winner
    scripts/setup/plan_full_load.py --machine bosch800 --placement side_winner \
        --cap fork=8 --fig_docs
"""

import argparse
import os
import sys
from datetime import datetime, timezone

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import capacity, config  # noqa: E402

parser = argparse.ArgumentParser(description="Plan the UR5e-reachable full load.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement (see config.BASE_PLACEMENTS).")
parser.add_argument("--classes", type=str, default=None,
                    help="Comma list restricting the class pool (default: robot-capable set).")
parser.add_argument("--cap", type=str, default=None,
                    help="Per-class caps, e.g. 'fork=8,plate=4'.")
parser.add_argument("--policy", type=str, default="plates_first",
                    choices=sorted(capacity.POLICIES))
parser.add_argument("--out", type=str, default=None,
                    help="Plan JSON path (default: results/capacity/<machine>/<placement>/"
                         "full_load_plan.json).")
parser.add_argument("--fig", type=str, default=None,
                    help="Figure PNG path (default: next to the plan JSON).")
parser.add_argument("--fig_docs", action="store_true",
                    help="Also copy the figure to docs/figures/ (curated report figure).")
args = parser.parse_args()

if args.machine:
    config.apply_machine(args.machine)
if args.placement:
    config.apply_base_placement(args.placement)


def main() -> int:
    classes = [c.strip() for c in args.classes.split(",")] if args.classes else None
    caps = None
    if args.cap:
        caps = {}
        for tok in args.cap.split(","):
            k, v = tok.split("=")
            caps[k.strip()] = int(v)

    plan = capacity.plan_full_load(classes=classes, policy=args.policy, per_class_cap=caps)

    out = args.out or os.path.join(PROJECT_ROOT, "results", "capacity", config.MACHINE,
                                   config.BASE_PLACEMENT, "full_load_plan.json")
    fig = args.fig or os.path.join(os.path.dirname(out), "reachability.png")

    # tables for the figure, reloaded per state under its own scenario (same hash checks)
    tables = {}
    entry = config.SCENARIO_NAME
    try:
        for phase in plan.phases:
            if not phase.funnel:
                continue
            config.apply_scenario(phase.state)
            tables[phase.state] = capacity.load_state_tables(
                phase.state, sorted(phase.funnel))
    finally:
        config.apply_scenario(entry)
    capacity.render_reachability_figure(
        plan, tables, fig) if tables else print("[WARN] no phases with tables; figure skipped")

    capacity.write_plan(plan, out, extra={
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "figure": os.path.relpath(fig, PROJECT_ROOT),
    })

    print(f"\n[INFO] plan:   {os.path.relpath(out, PROJECT_ROOT)}")
    print(f"[INFO] figure: {os.path.relpath(fig, PROJECT_ROOT)}")
    print(f"\n{'state':<12} {'placed':<7} composition")
    for phase in plan.phases:
        comp = ", ".join(f"{k}={v}" for k, v in sorted(phase.counts().items())) or "-"
        print(f"{phase.state:<12} {len(phase.items):<7} {comp}")
        for cls, f in sorted(phase.funnel.items()):
            print(f"  {cls:<10} slots {f['slots_total']:>3}  reachable {f['reachable']:>3}  "
                  f"assigned {f['assigned']:>3}  stopped_by {f['stopped_by']}")
    print(f"\n[INFO] FULL LOAD = {plan.total_items} items "
          f"({config.MACHINE} @ {config.BASE_PLACEMENT}, policy {plan.policy})")

    if args.fig_docs:
        import shutil
        dst = os.path.join(PROJECT_ROOT, "docs", "figures",
                           f"{config.MACHINE}_reachable_capacity.png")
        shutil.copyfile(fig, dst)
        print(f"[INFO] figure copied to {os.path.relpath(dst, PROJECT_ROOT)} "
              f"(add provenance to docs/figures/README.md)")

    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
