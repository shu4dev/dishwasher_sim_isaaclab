#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Where a lone plate rests after the insertion move (boot-first; plans/2026-10-22-plate-rest-pose-dag.md).

One plate alone in the Frigidaire twin (the benchmark baseline's extended, empty racks) is dropped by the sampler's
insertion move exactly as build_sequence does it (frigidaire_bench.upright_pose(slot, side) with the 3 mm lean bias,
lowered from 20 mm above the rim in 5 mm steps against the appliance, released 20 mm above the stop, settled 150 + 60
ticks at 120 Hz, parked 60 ticks before), into every lower-rack plate gap (the plate families' front and rear gaps) from
each lean side (-1, +1), all in one Kit session. Writes <out>/plate_rest.json: per (bank, gap, side) the rest pose
rack-local in the baseline's LowerRack (the frame of the records' entries_settled: position, quaternion XYZW), tilt (the
plate axis from the rack's up, 90 = upright), the lowest visual point over the plate floor (mode 'tines' above 30 mm,
else 'gap'), the settle deviation (rest origin from the move pose origin), the drift over the last 60 ticks, the contact
depth and the move's path; with the code SHA and the move's constants.
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --out data/results/sampler/plate_rest
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
PARK_TICKS, TINES_ABOVE_M, PID = 60, .030, "plate_01"


def code_sha():
    """HEAD's commit read from .git (git itself fails in the container)."""
    head = (ROOT / ".git/HEAD").read_text().strip()
    if not head.startswith("ref: "):
        return head
    ref = head[5:]
    if (ROOT / ".git" / ref).is_file():
        return (ROOT / ".git" / ref).read_text().strip()
    packed = ROOT / ".git/packed-refs"
    return next((ln.split()[0] for ln in (packed.read_text().splitlines() if packed.is_file() else [])
                 if ln.endswith(" " + ref)), None)


def settle_all(args, report):
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import LOWER_PLATE_FLOOR

    base = json.loads(Path(args.baseline).read_text())
    snapshot = base.get("snapshot", base)
    rack = snapshot["poses"]["LowerRack"]
    backend = K._sequence_scene(args, B, [{"id": PID, "kind": "plate", "rack": "LowerRack"}], snapshot,
                                args.out / "_kit", report)
    if backend is None:
        raise RuntimeError("the extended baseline did not hold")
    X, checker = B.HX(), B.release_checker()
    pts = X.visual_points(B.ASSETS / "plate.usda")
    teeth, _ = X.lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    park = {"position_m": [3., 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]}
    rows = []
    for bank, gaps in (("front", X.HL._FRONT_USABLE), ("rear", X.geometry.lower_plate_gaps("rear"))):
        for gap in gaps:
            for side in (-1, 1):
                slot = f"lower_{bank}_{gap:02d}"
                backend.set_rigid_pose(backend.objects[PID], park)
                for _ in range(PARK_TICKS):
                    backend.tick()
                live = backend.poses()
                move = P.world_from_local(rack, B.upright_pose(slot, side))
                up, rim_z, top_z = B.rack_heights(live, "LowerRack")
                m = B.insertion_release(checker, [("plate", move, PID)], [], B.appliance_bodies(checker, live), up,
                                        rim_z, top_z)
                row = {"bank": bank, "gap": int(gap), "slot": slot, "side": side, "gap_mid_x_m": float(mids[gap]),
                       **B.path_info(m)}
                rows.append(row)
                if m["poses"] is None:
                    print(f"{slot} side {side:+d}: blocked {m['blocked']}", flush=True)
                    continue
                backend.set_rigid_pose(backend.objects[PID], m["poses"][0])
                for t in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
                    frames, _ = backend.tick()
                    if t == K.SETTLE_TICKS - 1:
                        window = frames[PID]
                now = backend.poses()[PID]
                local = P.local_from_world(rack, now)
                R = B.quat_matrix(local["quaternion_xyzw"])
                low = float((pts @ R.T)[:, 2].min()) + float(local["position_m"][2]) - LOWER_PLATE_FLOOR
                drift = K.pose_distance(window, now)
                deep = max((d for k, d in backend.latest_contact["depths_m"].items() if PID in k.split("|")), default=0.)
                row.update(position=[float(v) for v in local["position_m"]],
                           quaternion_xyzw=[float(v) for v in local["quaternion_xyzw"]],
                           tilt_deg=round(math.degrees(math.acos(float(np.clip(R[2, 2], -1., 1.)))), 2),
                           dx_gap_mm=round((float(local["position_m"][0]) - float(mids[gap])) * 1e3, 1),
                           low_mm=round(low * 1e3, 1), mode="tines" if low > TINES_ABOVE_M else "gap",
                           settle_dev_mm=round(math.dist(now["position_m"], move["position_m"]) * 1e3, 1),
                           drift_mm=round(drift[0] * 1e3, 2), drift_deg=round(drift[1], 2),
                           contact_mm=round(deep * 1e3, 2))
                print(f"{slot} side {side:+d}: tilt {row['tilt_deg']} deg, dx {row['dx_gap_mm']} y "
                      f"{row['position'][1] * 1e3:.1f} z {row['position'][2] * 1e3:.1f} mm, low {row['low_mm']} mm "
                      f"({row['mode']}), dev {row['settle_dev_mm']} mm, drift {row['drift_mm']} mm / {row['drift_deg']} "
                      f"deg, stop {row['stop']} at {row['lowest_lift_mm']} mm", flush=True)
    out = {"code_sha": code_sha(), "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
           "baseline": str(args.baseline), "frame": "rack-local in the baseline's extended LowerRack (the records' entries_settled frame)",
           "move": {"step_mm": B.INSERT_STEP_M * 1e3, "above_rim_mm": B.INSERT_ABOVE_RIM_M * 1e3,
                    "gap_mm": B.INSERT_GAP_M * 1e3, "lean_bias_mm": B.INSERT_LEAN_BIAS_M * 1e3,
                    "settle_ticks": K.SETTLE_TICKS, "window_ticks": K.WINDOW_TICKS, "hz": 120, "park_ticks": PARK_TICKS},
           "entries": rows}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "plate_rest.json").write_text(json.dumps(out, indent=1) + "\n")
    rested = sum("position" in r for r in rows)
    report.update(result="PASS" if rested == len(rows) else "FAIL",
                  reason=f"{rested} of {len(rows)} settles, {sum(r.get('mode') == 'gap' for r in rows)} in the gap")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, required=True, help="writes <out>/plate_rest.json")
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    parser.add_argument("--max-wall-seconds", type=float, default=1800.)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    report, app = {"result": "FAIL", "reason": None}, None
    try:
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        settle_all(args, report)
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        print(f"[RESULT] {report['result']} {report['reason']}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
