#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free carry-pose pre-gate: FCL-check a candidate grasp/carry transform in seconds.

Grasp calibration in Kit costs ~25 minutes per candidate transform. Most bad candidates
fail for a purely geometric reason — the carried object interpenetrates a gripper link at
the carry pose. This gate answers that in seconds from the baked caches, so only
geometrically sane candidates go to Kit calibration (``scripts/setup/calibrate_grasp.py``).

Checks the object's collision pieces, posed at the candidate ``T_tcp_obj``, against every
gripper link mesh from the collision cache:

- ``left/right_inner_finger`` (the pads): contact is EXPECTED — the calibrated pinch is
  real pad contact. Reported, never fatal.
- every other link: must clear by ``--margin`` (default 1.5 mm). Any violation fails.

The verdict is geometric only — a PASS still needs Kit calibration for force/aperture;
a FAIL never needs Kit time at all.

Usage (venv python, no Kit):

    scripts/setup/carry_pregate.py --object mug                       # configured transform
    scripts/setup/carry_pregate.py --object plate --pos 0,0,0.05 --quat 0,0,0,1
    scripts/setup/carry_pregate.py --object tumbler --machine bosch800 --state placement
"""

import argparse
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

#: pad links — the calibrated pinch contacts these by design; contact is reported, not fatal
PAD_LINKS = ("left_inner_finger", "right_inner_finger")


def _parse_vec(text: str, n: int) -> tuple[float, ...]:
    parts = tuple(float(v) for v in text.split(","))
    if len(parts) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated floats, got {text!r}")
    return parts


def main() -> int:
    parser = argparse.ArgumentParser(description="Kit-free FCL pre-gate for a candidate carry pose.")
    parser.add_argument("--object", required=True, help="object class (config.OBJECTS key)")
    parser.add_argument("--machine", default=None, help="machine name (default: config default)")
    parser.add_argument("--state", default=None,
                        help="rack state whose cache to read (default: PLACEMENT_STATE)")
    parser.add_argument("--pos", type=lambda s: _parse_vec(s, 3), default=None,
                        help="candidate TCP->object position x,y,z [m] (default: configured grasp)")
    parser.add_argument("--quat", type=lambda s: _parse_vec(s, 4), default=None,
                        help="candidate TCP->object quaternion x,y,z,w (default: configured grasp)")
    parser.add_argument("--margin", type=float, default=0.0015,
                        help="required clearance [m] for non-pad links (default 1.5 mm)")
    args = parser.parse_args()

    if args.machine:
        config.apply_machine(args.machine)
    config.set_active_object(args.object)
    state = config.resolve_rack_state(args.state or config.PLACEMENT_STATE)
    cache_dir = config.scenario_cache_dir(state)

    manifest_path = os.path.join(cache_dir, "scene_state.json")
    if not os.path.isfile(manifest_path):
        print(f"[FAIL] no collision cache for object={args.object} state={state}")
        print(f"       bake it first:  scripts/setup/build_state.py --state {state} --classes {args.object}")
        return 2

    import json  # noqa: PLC0415

    import fcl  # noqa: PLC0415
    import trimesh  # noqa: PLC0415

    from dishsim.collision_world import _fcl_convex, _tf, load_object_pieces  # noqa: PLC0415
    from dishsim.transforms import make_T  # noqa: PLC0415

    manifest = json.load(open(manifest_path))
    pieces = load_object_pieces(cache_dir)
    if not pieces:
        print(f"[FAIL] cache at {cache_dir} carries no object pieces for {args.object}")
        return 2

    pos = args.pos if args.pos is not None else config.GRASP_TCP_OBJ_POS
    quat = args.quat if args.quat is not None else config.GRASP_TCP_OBJ_QUAT
    # candidate carry: object in the TCP frame; lift to wrist_3 via the fixed TCP offset
    T_w3_tcp = make_T(config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT)
    T_w3_obj = T_w3_tcp @ make_T(pos, quat)

    print(f"[INFO] pre-gate: object={args.object} state={state} machine={config.MACHINE}")
    print(f"[INFO] T_tcp_obj pos={tuple(round(v, 4) for v in pos)} quat={tuple(round(v, 4) for v in quat)}")

    def _fcl_obj(mesh: trimesh.Trimesh, T: np.ndarray) -> fcl.CollisionObject:
        # RAW convex hulls, deliberately uninflated — the gate reports physical clearances;
        # the planner's inflation margin is a separate concern
        return fcl.CollisionObject(_fcl_convex(mesh.convex_hull), _tf(T))

    piece_objs = [_fcl_obj(p, T_w3_obj) for p in pieces]

    failures, contacts = [], []
    for name, entry in manifest["gripper_links"].items():
        link_mesh = trimesh.load(os.path.join(cache_dir, entry["mesh"]), force="mesh")
        link_obj = _fcl_obj(link_mesh, np.array(entry["T_wrist3_link"]))
        dmin = np.inf
        for po in piece_objs:
            d = fcl.distance(link_obj, po, fcl.DistanceRequest(), fcl.DistanceResult())
            dmin = min(dmin, d)
        touching = dmin <= 0.0
        if name in PAD_LINKS:
            contacts.append((name, dmin))
            print(f"[OK]   pad  {name:<22} {'CONTACT' if touching else f'clearance {dmin * 1000:6.2f} mm'}")
        elif dmin < args.margin:
            failures.append((name, dmin))
            print(f"[BAD]  link {name:<22} clearance {dmin * 1000:6.2f} mm < margin {args.margin * 1000:.2f} mm")
        else:
            print(f"[OK]   link {name:<22} clearance {dmin * 1000:6.2f} mm")

    if not any(d <= 0.002 for _, d in contacts):
        print("[WARN] neither pad is within 2 mm of the object — the pinch would not engage "
              "(carry would hang on the weld alone); check the transform before calibrating")

    if failures:
        worst = min(failures, key=lambda f: f[1])
        print(f"[RESULT] FAIL — {len(failures)} link(s) violate the margin "
              f"(worst: {worst[0]} at {worst[1] * 1000:.2f} mm)")
        return 1
    print("[RESULT] PASS — geometrically sane; proceed to Kit calibration "
          "(scripts/setup/calibrate_grasp.py)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
