#!/usr/bin/env python3
"""The empty machine's rack retraction, measured as the joint gate measures it (plans/2026-11-01-rack-wheels-visual.md).

One fresh Kit process. It restores the recorded empty-machine baseline (both racks out at the flush travel) with
IsaacInitialStateBackend.prepare_baseline, then retracts both racks with the gate's own ramp (IsaacInitialStateBackend.ramp:
a cubic smoothstep under the 0.08 m/s commanded cap) in each order, restoring the baseline again before each order.
For each rack and order it reports:
- the peak measured speed of the retracting slide, |delta slide| * 120 Hz per tick, as the gate's ramp computes it. The
  ramp's own peak over both slides is kept beside it, and the larger of the two is judged.
- the stalls: each stretch where the slide lags its command by more than 15 mm, with the slide where the lag passed
  15 mm and how long the stretch lasted.

It prints [RESULT] PASS retract check when every restore held, every peak is within the gate's measured cap (LIMITS
rack_speed_m_s, 0.10 m/s, plus the gate's 1e-5 m/s tolerance) and no stall occurred. Otherwise it prints
[RESULT] FAIL retract check <peaks and stalls>. The JSON (peaks, stalls, 10 Hz slide curves) goes to --out.

    code/util/run_kit.sh code/sampler/scripts/frigidaire_retract_check.py --headless \
        [--order both|lower_first|upper_first] [--out data/logs/wheels/retract_check.json]
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
USD = ROOT/"data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc"
BASELINE = ROOT/"data/results/benchmark/frigidaire_hotec/baseline/result.json"
ORDERS = {"lower_first": ("LowerRack", "UpperRack"), "upper_first": ("UpperRack", "LowerRack")}
STALL_LAG_M = .015
MAX_WALL_S = 900.


def analyse(rows, hz, lag_m=STALL_LAG_M):
    """``rows``: (slide, command) per ramp tick, led by the slide before the first tick (command None).

    Returns the peak per-tick speed |delta slide| * hz with the tick, slide and command where it happened, and the stalls
    (each run of ticks lagging the command by more than ``lag_m``: the slide at its first tick, its length in seconds, and
    whether it was still open when the ramp ended)."""
    rows = [(float(slide), None if command is None else float(command)) for slide, command in rows]
    speeds = [abs(rows[k][0]-rows[k-1][0])*hz for k in range(1, len(rows))]
    k = max(range(len(speeds)), key=speeds.__getitem__)
    stalls, start = [], None
    for i in range(1, len(rows)):
        lagging = abs(rows[i][0]-rows[i][1]) > lag_m
        if lagging and start is None:
            start = i
        elif not lagging and start is not None:
            stalls.append({"slide_m": round(rows[start][0], 4), "seconds": round((i-start)/hz, 3), "open": False})
            start = None
    if start is not None:
        stalls.append({"slide_m": round(rows[start][0], 4), "seconds": round((len(rows)-start)/hz, 3), "open": True})
    return {"peak_m_s": speeds[k], "peak_tick": k+1, "peak_slide_m": rows[k+1][0], "peak_command_m": rows[k+1][1],
            "stalls": stalls, "curve_10hz": [[i, rows[i][0], rows[i][1], speeds[i-1]] for i in range(1, len(rows), 12)]}


def summary(record):
    restores = [f"{order} baseline restore {o['baseline_restore']['result']}" for order, o in record["orders"].items()
                if o["baseline_restore"]["result"] != "PASS"]
    return "; ".join(restores+[f"{order} {rack} peak {r['peak_m_s']:.3f} (gate {r['gate_ramp_peak_m_s']:.3f}) stalls "
                               f"{[(s['slide_m'], s['seconds']) for s in r['stalls']]}"
                               for order, o in record["orders"].items() for rack, r in o["racks"].items()])


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--order", choices=["both", *ORDERS], default="both")
    parser.add_argument("--out", type=Path, default=BASELINE.parent/"retract_check.json")
    sys.path[:0] = [str(ROOT/"code"), str(ROOT/"code/util")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit in the try below
    add_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    record = {"started_utc": datetime.now(timezone.utc).isoformat(), "usd": str(USD.relative_to(ROOT)),
              "baseline": str(BASELINE.relative_to(ROOT)), "stall_lag_m": STALL_LAG_M, "orders": {}}
    passed, app = False, None
    try:
        app = launch(args)
        from sampler.initial_state_runtime import (IsaacInitialStateBackend, RACK_COMMAND_SPEED_M_S,
                                                              RACK_SPEED_MEASUREMENT_TOLERANCE_M_S)
        from sampler.random_poses import LIMITS, source_geometry_domains
        from sampler.random_pose_runtime import RACK_JOINT
        cap = LIMITS["rack_speed_m_s"]+RACK_SPEED_MEASUREMENT_TOLERANCE_M_S
        record.update(measured_cap_m_s=LIMITS["rack_speed_m_s"], tolerance_m_s=RACK_SPEED_MEASUREMENT_TOLERANCE_M_S,
                      commanded_cap_m_s=RACK_COMMAND_SPEED_M_S)
        backend = IsaacInitialStateBackend(USD, args.out.parent, device=args.device, domains=source_geometry_domains(),
                                           deadline=started+MAX_WALL_S, app=app, candidates=[])
        base = json.loads(BASELINE.read_text())
        rows, tick = [], backend.tick

        def traced():
            frames, joints = tick()
            rows.append((joints, {j: float(backend.target[0, backend.indices[j]]) for j in RACK_JOINT.values()}))
            return frames, joints

        backend.tick = traced
        ok = True
        for order in (ORDERS if args.order == "both" else [args.order]):
            restore = backend.prepare_baseline(base)
            ok = ok and restore["result"] == "PASS"
            entry = record["orders"][order] = {"baseline_restore": {"result": restore["result"],
                "reason": restore["hold"].get("reason"), "joints": restore["joints"]}, "racks": {}}
            print(f"{order}: baseline restore {restore['result']} ({restore['hold'].get('reason')}) {restore['joints']}",
                  flush=True)
            for rack in ORDERS[order]:
                joint = RACK_JOINT[rack]
                before = backend.joints()[joint]
                rows.clear()
                motion = backend.ramp({joint: 0.}, "retract_"+rack)
                r = analyse([(before, None)]+[(j[joint], c[joint]) for j, c in rows], backend.hz)
                r.update(joint=joint, gate_ramp_peak_m_s=float(motion["peak_measured_rack_speed_m_s"]),
                         ramp_duration_s=float(motion["duration_s"]), end_slide_m=float(rows[-1][0][joint]))
                entry["racks"][rack] = r
                ok = ok and max(r["peak_m_s"], r["gate_ramp_peak_m_s"]) <= cap and not r["stalls"]
                print(f"{order} {rack}: peak {r['peak_m_s']:.3f} m/s at tick {r['peak_tick']} ({joint} "
                      f"{r['peak_slide_m']:+.4f}, command {r['peak_command_m']:+.4f}); gate ramp peak over both slides "
                      f"{r['gate_ramp_peak_m_s']:.3f}; stalls (slide where the lag passed 15 mm, s): "
                      f"{[(s['slide_m'], s['seconds']) for s in r['stalls']]}; end {r['end_slide_m']:+.4f}", flush=True)
        passed = bool(ok)
    except Exception as exc:
        traceback.print_exc()
        record["error"] = repr(exc)
    finally:
        record["passed"] = passed
        try:
            record["isaac_sim_version"] = Path("/isaac-sim/VERSION").read_text().strip()
        except OSError:
            pass
        record["wall_seconds"] = time.monotonic()-started
        try:
            args.out.write_text(json.dumps(record, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))+"\n")
        except (OSError, TypeError, ValueError) as exc:
            print(f"retract check: the JSON was not written: {exc!r}", flush=True)
        detail = summary(record) if "error" not in record else "error "+record["error"]
        print(f"[RESULT] {'PASS' if passed else 'FAIL'} retract check {detail} ({args.out})", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
