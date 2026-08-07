# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Freeze a scripts/setup/calibrate_grasp.py calibration result into the object registry (config.OBJECTS).

Reads the calibration JSON written by ``calibrate_grasp.py`` and patches the object's ``GraspSpec`` in
``src/dishsim/config.py`` with the measured aperture + force band (the same derivation the
mug's frozen constants used: band = steady/3 .. 3x steady, exec cap = max(12, 2x steady)).
Numeric provenance: every written number traces to the calibration run, never eyeballed.

Venv-side: ``env_isaaclab/bin/python scripts/setup/freeze_calibration.py --object bowl``
"""

import argparse
import json
import os
import re
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Freeze scripts/setup/calibrate_grasp.py calibration into config.OBJECTS.")
parser.add_argument("--object", type=str, required=True)
args = parser.parse_args()


def main() -> None:
    # mirrors calibrate_grasp's output rule: the mug writes to media/calibration/,
    # every other object to media/calibration/<object>/
    cal_dir = os.path.join(PROJECT_ROOT, "media", "calibration")
    if args.object != "mug":
        cal_dir = os.path.join(cal_dir, args.object)
    cal_path = os.path.join(cal_dir, "calibration.json")
    with open(cal_path) as f:
        cal = json.load(f)
    theta = float(cal["theta_grasp"])
    steady = float(cal["steady_pad_force_n"])
    lo = round(max(0.3, steady / 3.0), 1)
    hi = round(max(3.0 * steady, steady + 3.0), 1)
    exec_hi = round(max(12.0, 2.0 * steady), 1)

    cfg_path = os.path.join(PROJECT_ROOT, "src", "dishsim", "config.py")
    src = open(cfg_path).read()
    # locate this object's GraspSpec( ... ) span inside its ObjectSpec entry
    m = re.search(rf'"{args.object}": ObjectSpec\(', src)
    assert m, f"no registry entry for {args.object!r}"
    g = src.index("grasp=GraspSpec(", m.end())
    i = g + len("grasp=GraspSpec(")
    depth = 1
    while depth:
        if src[i] == "(":
            depth += 1
        elif src[i] == ")":
            depth -= 1
        i += 1
    span = src[g:i]
    assert "aperture_rad" not in span, f"{args.object}: aperture already frozen — remove it first"
    inject = (
        f" aperture_rad={theta}, force_min_n={lo}, force_max_n={hi},"
        f" force_exec_max_n={exec_hi},"
    )
    # insert before the closing paren, keeping the existing trailing style
    body = span[:-1].rstrip()
    sep = "" if body.endswith(",") else ","
    new_span = body + sep + inject + span[-1]
    src = src[:g] + new_span + src[i:]
    open(cfg_path, "w").write(src)
    print(
        f"[OK] froze {args.object}: aperture {theta} rad, band [{lo}, {hi}] N "
        f"(steady {steady:.2f} N), exec cap {exec_hi} N"
    )


if __name__ == "__main__":
    main()
