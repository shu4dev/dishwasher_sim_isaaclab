#!/workspace/isaaclab/env_isaaclab/bin/python
# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Bake the collision caches a machine state needs, for one or more object classes.

A machine state is a pair of rack extensions. Those extensions are part of
:func:`dishsim.geometry.config_hash`, and caches are keyed by state *name*, so a state cannot be
planned against until its geometry has been extracted, decomposed, parity-checked and turned
into goal sets — per object class. That is four scripts times N classes, three of which need
Kit, which is exactly the sort of chain that gets typed wrong at 5 pm.

This runs the chain in the right order and stops at the first real failure. It is the command
``run_task.py`` prints when its cache pre-flight finds nothing to plan against.

Success is judged from log CONTENT, never exit status: ``isaaclab.sh -p`` exits 0 even when the
wrapped script dies, so a green exit code proves nothing here.

Two stages are allowed to "fail" without stopping the build, because for some (state, class)
pairs zero feasible slots is the honest measured answer rather than an error — a plate has no
reachable slot in any currently-baked state. Those are reported and the build continues; what
matters is that the artifacts exist so the runner can load them and report the scarcity itself.

Run with:
    scripts/setup/build_state.py --state placement --classes mug,cup,tumbler
    scripts/setup/build_state.py --state placement_open --classes mug --skip_parity
"""

import argparse
import os
import subprocess
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

parser = argparse.ArgumentParser(description="Bake a machine state's collision caches.")
parser.add_argument("--state", type=str, required=True,
                    help="Machine state name (a key of config.SCENARIOS / INTERNAL_STATES).")
parser.add_argument("--classes", type=str, required=True,
                    help="Comma list of object classes to bake, e.g. mug,cup,tumbler.")
parser.add_argument("--skip_parity", action="store_true",
                    help="Skip the FCL-vs-PhysX parity gate (faster; you lose the safety check).")
parser.add_argument("--skip_goals", action="store_true",
                    help="Skip the goal-funnel stage. The base-pose sweep derives slots and "
                         "runs its own funnels, so bakes that only feed the sweep never need "
                         "the baked goal sets; episode runs DO (rebake without this flag).")
parser.add_argument("--force", action="store_true", help="Re-decompose even if pieces exist.")
parser.add_argument("--dry_run", action="store_true", help="Print the commands and exit.")
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement to bake at (see config.BASE_PLACEMENTS).")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
args = parser.parse_args()

if args.machine:
    config.apply_machine(args.machine)  # before resolve_rack_state — states are per-machine

_MACHINE_ARGS = (["--machine", args.machine] if args.machine else []) \
    + (["--placement", args.placement] if args.placement else [])

RUN_KIT = os.path.join(PROJECT_ROOT, "scripts", "run_kit.sh")
VENV_PY = "/workspace/isaaclab/env_isaaclab/bin/python"


def stages(obj: str) -> list:
    """The four-stage chain for one (state, class), in dependency order."""
    setup = os.path.join(PROJECT_ROOT, "scripts", "setup")
    out = [
        ("extract_geometry", [RUN_KIT, os.path.join(setup, "extract_geometry.py"),
                              "--headless", "--object", obj, "--scenario", args.state]
                             + _MACHINE_ARGS, True),
        ("decompose_meshes", [VENV_PY, os.path.join(setup, "decompose_meshes.py"),
                              "--object", obj, "--scenario", args.state] + _MACHINE_ARGS
                             + (["--force"] if args.force else []), True),
    ]
    if not args.skip_parity:
        out.append(("parity_check", [RUN_KIT, os.path.join(setup, "parity_check.py"),
                                     "--headless", "--object", obj, "--scenario", args.state]
                                    + _MACHINE_ARGS, True))
    if not args.skip_goals:
        # zero feasible slots is a measured outcome for some classes, not a build error
        out.append(("goal_configs", [RUN_KIT, os.path.join(setup, "goal_configs.py"),
                                     "--headless", "--object", obj, "--scenario", args.state,
                                     "--sheets_per_slot", "0"] + _MACHINE_ARGS, False))
    return out


def main() -> int:
    try:
        state = config.resolve_rack_state(args.state)
    except (ValueError, RuntimeError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    unknown = [c for c in classes if c not in config.OBJECTS]
    if unknown:
        print(f"[FAIL] unknown object class(es) {unknown}")
        return 1

    sc = {**config.SCENARIOS, **config.INTERNAL_STATES}[state]
    print(f"[INFO] baking state {state!r} (lower {sc['rack_lower_m']} m, upper "
          f"{sc['rack_upper_m']} m) for {classes}")

    failures = []
    for obj in classes:
        for name, cmd, fatal in stages(obj):
            label = f"{obj}/{name}"
            if args.dry_run:
                print(f"  {' '.join(cmd)}")
                continue
            print(f"[INFO] {label} ...", flush=True)
            proc = subprocess.run(cmd, cwd=PROJECT_ROOT, capture_output=True, text=True)
            log = proc.stdout + proc.stderr
            # exit status is meaningless here — judge from the log
            crashed = "Traceback (most recent call last)" in log
            passed = "[RESULT] PASS" in log or ("[FAIL]" not in log and not crashed)
            if crashed or not passed:
                detail = next((ln for ln in log.splitlines() if ln.startswith("[FAIL]")), "")
                if fatal:
                    print(f"[FAIL] {label} did not complete. {detail}")
                    print(log[-1500:])
                    return 1
                print(f"[WARN] {label}: {detail or 'no feasible slots'} — continuing; the runner "
                      f"reports slot scarcity itself")
                failures.append(label)
            else:
                print(f"[INFO] {label} ok")

    if args.dry_run:
        return 0
    print(f"[INFO] state {state!r} built for {classes}"
          + (f"; no feasible slots for {failures}" if failures else ""))
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
