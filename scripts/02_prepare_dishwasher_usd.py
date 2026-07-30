# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate the passive-door derived copy of the ArtVIP dishwasher USD.

The derived copy (``model_<variant>_rl.usda``, written next to the original) removes the
world-weld fixed joint and neutralizes the authored door drive — see
``src/dishsim/usd_prep.py`` for details. Running this is optional if you run
``scripts/00_inspect_scene.py`` first (it generates the file on demand).

Run with:
    /workspace/isaaclab/isaaclab.sh -p scripts/02_prepare_dishwasher_usd.py [--variant dishwasher_2] [--force]
"""

import argparse
import glob
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Prepare the RL-ready dishwasher USD.")
parser.add_argument("--variant", type=str, default="dishwasher_2", help="ArtVIP dishwasher variant name.")
parser.add_argument("--force", action="store_true", help="Regenerate even if the derived file exists.")
args = parser.parse_args()


def main():
    from dishsim.usd_prep import make_dishwasher_rl_usd

    pattern = os.path.join(
        PROJECT_ROOT, "assets", "artvip", "Articulated_objects", "major_appliances", "dishwasher",
        args.variant, f"model_{args.variant}.usd*",
    )
    matches = [p for p in glob.glob(pattern) if not p.endswith("_rl.usda")]
    if not matches:
        raise FileNotFoundError(f"No source USD for variant '{args.variant}' (looked for {pattern})")
    dst = make_dishwasher_rl_usd(matches[0], force=args.force)
    print(f"[OK] {dst}")


if __name__ == "__main__":
    main()
