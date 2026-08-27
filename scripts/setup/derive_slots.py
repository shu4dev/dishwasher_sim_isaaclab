#!/workspace/isaaclab/env_isaaclab/bin/python
# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive a state's placement slots and render the evidence (Kit-free, venv python).

Slots derive live from the collision cache (no bake — see :func:`dishsim.placement.derive_slots`);
this script exists for inspection: the per-slot table, the empty-machine placeability verdict
per slot (the same pre-scan ``dishsim.capacity`` runs), and the top-down
``media/goals/slot_detection.png`` figure.

Run with:
    scripts/setup/derive_slots.py --object cup --scenario placement
    scripts/setup/derive_slots.py --machine bosch800 --placement side_winner \
        --object plate --scenario placement
"""

import argparse
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

parser = argparse.ArgumentParser(description="Derive placement slots + placeability evidence.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement (see config.BASE_PLACEMENTS); default: the machine's.")
parser.add_argument("--object", type=str, default="mug", help="Object class (see config.OBJECTS).")
parser.add_argument("--scenario", type=str, default=None,
                    help="Rack-state scenario (default: the placement state).")
parser.add_argument("--out", type=str, default=None,
                    help="Figure PNG (default: media/goals[/<scenario>]/slot_detection.png).")
args = parser.parse_args()

if args.machine:
    config.apply_machine(args.machine)  # first: it resets scenario + base placement
config.set_active_object(args.object)
config.apply_scenario(args.scenario or config.PLACEMENT_STATE)
if args.placement:
    config.apply_base_placement(args.placement)  # after machine/scenario — they reset it

from dishsim import placement  # noqa: E402
from dishsim.capacity import _nominal_release_pose  # noqa: E402
from dishsim.collision_world import CollisionWorld, load_object_pieces  # noqa: E402


def render_slot_detection(slots, out_png: str) -> None:
    """Top-down plot: loaded-rack wire vertices (base frame) + slot cells."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import trimesh

    from dishsim.geometry import load_manifest

    manifest = load_manifest(config.scenario_cache_dir())
    body = placement._floor_stand_rack_body()
    entry = manifest["statics"][body]
    mesh = trimesh.load(os.path.join(config.scenario_cache_dir(), entry["mesh"]), force="mesh")
    T = np.array(entry["T_base_body"])
    verts = (T @ np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))]).T).T[:, :3]

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(verts[:, 0], verts[:, 1], s=0.2, c="0.55", linewidths=0, label="rack wires")
    for s in slots:
        cx, cy = s.T_base_slot[0, 3], s.T_base_slot[1, 3]
        half = s.width_m / 2.0
        ax.add_patch(plt.Rectangle((cx - half, cy - half), 2 * half, 2 * half, fill=False, ec="tab:blue"))
        ax.annotate(str(s.slot_id), (cx, cy), ha="center", va="center", color="tab:red", fontsize=12)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (base frame)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"Slot derivation (top-down, {body}) — scenario {config.SCENARIO_NAME}")
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] slot detection plot: {out_png}")


def main() -> int:
    cdir = config.scenario_cache_dir()
    if not os.path.exists(os.path.join(cdir, "scene_state.json")):
        print(f"[FAIL] no cache under {cdir} — bake it with: scripts/setup/build_state.py "
              f"--state {config.SCENARIO_NAME} --classes {args.object}"
              + (f" --machine {args.machine}" if args.machine else "")
              + (f" --placement {args.placement}" if args.placement else ""))
        return 1
    slots = placement.derive_slots(cdir)
    print(f"[INFO] {args.object}/{config.SCENARIO_NAME}: derived {len(slots)} slots "
          f"(mode {config.effective_placement_mode()})")

    world = CollisionWorld(cache_dir=cdir)
    pieces = load_object_pieces(cdir)
    names = {v: k for k, v in placement.slot_names(slots).items()}
    n_ok = 0
    for s in slots:
        ok = not world.object_in_collision(pieces, _nominal_release_pose(s))
        n_ok += ok
        c = s.T_base_slot[:3, 3]
        print(f"[INFO]   slot {s.slot_id:>3} ({names.get(s.slot_id, '?'):<12}) "
              f"center ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})  "
              f"{'placeable' if ok else 'BLOCKED'}")
    print(f"[INFO] {n_ok}/{len(slots)} slots placeable in the empty machine")

    out = args.out or os.path.join(config.scenario_media_dir("goals"), "slot_detection.png")
    render_slot_detection(slots, out)
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
