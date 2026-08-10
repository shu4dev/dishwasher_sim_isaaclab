# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Venv side (no Kit): sweep candidate robot base poses and rank them against the success bar.

Stages (each checkpointed to ``--out_dir``, so a rerun can be trimmed with the range args):

1. **gates** — every grid candidate through the hard gates (static clearance, HOME_Q free,
   rack pull + push feasible). Cheap; kills most of the grid.
2. **proxy** — survivors scored at the ``proxy`` level (6-sample funnels on a 4-combo subset,
   coarse pick grid). Ranks the field; keeps ``--stage2_top``.
3. **full** — the keepers (plus the front baseline) at the ``full`` level (16-sample funnels
   on every baked combo, 0.05 m pick grid). Optionally refined around the leader with a
   half-step neighborhood. Keeps ``--final_top``.
4. **final** — finalists re-scored at the shipped 64-sample budget; the winner and the front
   baseline are reported side by side against the success-bar criteria.

Worlds load once per process; candidates only re-pose statics (see dishsim.base_sweep).

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/setup/base_pose_sweep.py
    /workspace/isaaclab/env_isaaclab/bin/python scripts/setup/base_pose_sweep.py \
        --x_range -0.15 0.30 --y_range -0.75 0.10 --yaw_range -30 75 --procs 8
"""

import argparse
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import base_sweep, config  # noqa: E402
from dishsim.base_sweep import FRONT, Candidate  # noqa: E402

parser = argparse.ArgumentParser(description="Sweep robot base poses against the cached scene.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline. "
                         "Selecting bosch800 also switches the default ranges to its "
                         "elevated/side-mount region and adds the height axis.")
parser.add_argument("--x_range", type=float, nargs=2, default=None,
                    help="World-x range of the base origin [m].")
parser.add_argument("--y_range", type=float, nargs=2, default=None,
                    help="World-y range of the base origin [m].")
parser.add_argument("--yaw_range", type=float, nargs=2, default=None,
                    help="Base z-yaw range [deg] (positive turns +x toward the counter side).")
parser.add_argument("--z_range", type=float, nargs=2, default=None,
                    help="Base HEIGHT range [m] (pedestal-top). Omit for the v1 fixed height.")
parser.add_argument("--z_step", type=float, default=0.15, help="Height pitch [m].")
parser.add_argument("--xy_step", type=float, default=0.075, help="Coarse grid pitch [m].")
parser.add_argument("--yaw_step", type=float, default=22.5, help="Coarse yaw pitch [deg].")
parser.add_argument("--stage2_top", type=int, default=48,
                    help="Survivors promoted from the proxy ranking to full scoring.")
parser.add_argument("--final_top", type=int, default=6,
                    help="Candidates promoted to the 64-sample final scoring.")
parser.add_argument("--no_refine", action="store_true",
                    help="Skip the half-step neighborhood refinement around the leader.")
parser.add_argument("--procs", type=int, default=8, help="Worker processes.")
parser.add_argument("--out_dir", type=str, default=None)
parser.add_argument("--media_dir", type=str, default=None)
args = parser.parse_args()

if args.machine:
    config.apply_machine(args.machine)

# Per-machine default sweep regions. v1: the measured 420-candidate study's region around
# the front placement. Bosch: the human-loader region — beside and behind the open door
# (door runway x 0.13-0.87 at y ±0.30), base up to counter height, yawed toward the machine.
if config.MACHINE == "bosch800":
    _DEFAULTS = {"x_range": (-0.20, 0.70), "y_range": (-0.90, -0.35),
                 "yaw_range": (0.0, 90.0), "z_range": (0.25, 1.00)}
    _SUBDIR = "bosch800"
else:
    _DEFAULTS = {"x_range": (-0.15, 0.30), "y_range": (-0.75, 0.10),
                 "yaw_range": (-30.0, 75.0), "z_range": None}
    _SUBDIR = None
for _k, _v in _DEFAULTS.items():
    if getattr(args, _k) is None:
        setattr(args, _k, _v)
if args.out_dir is None:
    args.out_dir = os.path.join(PROJECT_ROOT, "results", "base_sweep",
                                *([_SUBDIR] if _SUBDIR else []))
if args.media_dir is None:
    args.media_dir = os.path.join(PROJECT_ROOT, "media", "base_sweep",
                                  *([_SUBDIR] if _SUBDIR else []))

_CTX = None


def _ensure_ctx():
    global _CTX
    if _CTX is None:
        _CTX = base_sweep.SweepContext()
    return _CTX


def _task_gates(cand: Candidate) -> dict:
    ctx = _ensure_ctx()
    return {"candidate": cand.to_json(), "key": cand.key, "gates": base_sweep.run_gates(ctx, cand)}


def _task_score(payload: tuple) -> dict:
    cand, level = payload
    ctx = _ensure_ctx()
    return base_sweep.score_candidate(ctx, cand, level)


def _grid() -> list[Candidate]:
    xs = np.arange(args.x_range[0], args.x_range[1] + 1e-9, args.xy_step)
    ys = np.arange(args.y_range[0], args.y_range[1] + 1e-9, args.xy_step)
    yaws = np.arange(args.yaw_range[0], args.yaw_range[1] + 1e-9, args.yaw_step)
    if args.z_range is None:
        zs: list[float | None] = [None]  # v1: height pinned to the placement's
    else:
        zs = [round(float(z), 3)
              for z in np.arange(args.z_range[0], args.z_range[1] + 1e-9, args.z_step)]
    return [Candidate(round(float(x), 4), round(float(y), 4), round(float(w), 2), z)
            for x in xs for y in ys for w in yaws for z in zs]


def _neighborhood(cand: Candidate) -> list[Candidate]:
    """Half-step neighborhood around a candidate (refinement grid; z refines too when swept)."""
    dxy, dyaw = args.xy_step / 2.0, args.yaw_step / 2.0
    dzs = (0.0,) if cand.z is None else (-args.z_step / 2.0, 0.0, args.z_step / 2.0)
    out = []
    for dx in (-dxy, 0.0, dxy):
        for dy in (-dxy, 0.0, dxy):
            for dw in (-dyaw, 0.0, dyaw):
                for dz in dzs:
                    z = None if cand.z is None else round(cand.z + dz, 3)
                    out.append(Candidate(round(cand.x + dx, 4), round(cand.y + dy, 4),
                                         round(cand.yaw_deg + dw, 2), z))
    return out


def _run_pool(pool, fn, payloads, label: str) -> list[dict]:
    t0 = time.time()
    out = []
    for k, res in enumerate(pool.imap(fn, payloads, chunksize=1), 1):
        out.append(res)
        if k % 25 == 0 or k == len(payloads):
            rate = (time.time() - t0) / k
            print(f"[INFO] {label}: {k}/{len(payloads)} "
                  f"({rate:.1f} s/cand, ~{rate * (len(payloads) - k) / 60.0:.1f} min left)",
                  flush=True)
    return out


def _save(name: str, payload) -> None:
    os.makedirs(args.out_dir, exist_ok=True)
    with open(os.path.join(args.out_dir, name), "w") as f:
        json.dump(payload, f, indent=2)


def _fmt(card: dict) -> str:
    c, p = card.get("counts", {}), card.get("pick", {})
    return (f"{card['key']:<34} crit {card.get('n_criteria', 0)}/5 w {card.get('weighted', 0.0):7.2f}  "
            f"plate {c.get('plate', 0):2d} bowl {c.get('bowl', 0)} fork {c.get('fork', 0)} "
            f"floor* {c.get('floor_common', 0):2d} depth {p.get('depth_x_m', 0.0):.3f} m")


def _heatmaps(cards: list[dict], out_png: str) -> None:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    by_yaw: dict[float, list[dict]] = {}
    for card in cards:
        by_yaw.setdefault(card["candidate"]["yaw_deg"], []).append(card)
    yaws = sorted(by_yaw)
    ncol = min(3, len(yaws))
    nrow = int(np.ceil(len(yaws) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 4.0 * nrow), squeeze=False)
    for k, yaw in enumerate(yaws):
        ax = axes[k // ncol][k % ncol]
        pts = by_yaw[yaw]
        xs = [c["candidate"]["x"] for c in pts]
        ys = [c["candidate"]["y"] for c in pts]
        ws = [c["weighted"] if c.get("feasible") else np.nan for c in pts]
        sc = ax.scatter(ys, xs, c=ws, s=170, marker="s", cmap="viridis",
                        plotnonfinite=False)
        gated = [(c["candidate"]["y"], c["candidate"]["x"]) for c in pts if not c.get("feasible")]
        if gated:
            gy, gx = zip(*gated)
            ax.scatter(gy, gx, c="0.85", s=170, marker="s")
        ax.scatter([config.ROBOT_BASE_POS_W[1]], [config.ROBOT_BASE_POS_W[0]],
                   marker="*", s=180, c="crimson", label="front")
        ax.set_title(f"yaw {yaw:+.1f} deg", fontsize=9)
        ax.set_xlabel("world y [m]", fontsize=8)
        ax.set_ylabel("world x [m]", fontsize=8)
        ax.set_aspect("equal")
        fig.colorbar(sc, ax=ax, shrink=0.8)
    for k in range(len(yaws), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle("Base-pose sweep: weighted score by base position (grey = gated out)", fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"[INFO] heatmaps: {os.path.relpath(out_png, PROJECT_ROOT)}")


def main() -> int:
    candidates = _grid()
    print(f"[INFO] coarse grid: {len(candidates)} candidates "
          f"(x {args.x_range}, y {args.y_range}, yaw {args.yaw_range})")
    print("[INFO] loading collision worlds in the parent (forked workers share them)...")
    t0 = time.time()
    _ensure_ctx()
    print(f"[INFO] worlds loaded in {time.time() - t0:.1f} s")

    with mp.Pool(processes=args.procs) as pool:
        # ---- stage 1: hard gates -------------------------------------------------------------
        gate_rows = _run_pool(pool, _task_gates, candidates, "stage1 gates")
        survivors = [Candidate.from_json(r["candidate"]) for r in gate_rows if r["gates"]["ok"]]
        _save("stage1_gates.json", gate_rows)
        print(f"[INFO] stage1: {len(survivors)}/{len(candidates)} candidates pass the gates")
        if not survivors:
            print("[FAIL] no candidate passes the hard gates — widen the ranges")
            return 1

        # ---- stage 2: proxy ranking ----------------------------------------------------------
        proxy = _run_pool(pool, _task_score, [(c, "proxy") for c in survivors], "stage2 proxy")
        proxy.sort(key=base_sweep.rank_key, reverse=True)
        _save("stage2_proxy.json", proxy)
        keep = [Candidate.from_json(c["candidate"])
                for c in proxy[: args.stage2_top] if c.get("feasible")]
        print(f"[INFO] stage2 leaders:")
        for card in proxy[:8]:
            print(f"[INFO]   {_fmt(card)}")

        # ---- stage 3: full scoring (+ optional refinement) -----------------------------------
        full_payloads = [(c, "full") for c in {*keep, FRONT}]
        full = _run_pool(pool, _task_score, full_payloads, "stage3 full")
        full.sort(key=base_sweep.rank_key, reverse=True)
        if not args.no_refine:
            leader = Candidate.from_json(full[0]["candidate"])
            seen = {c["key"] for c in full}
            extra = [c for c in _neighborhood(leader) if c.key not in seen]
            print(f"[INFO] refining {len(extra)} half-step neighbors around {leader.key}")
            full += _run_pool(pool, _task_score, [(c, "full") for c in extra], "stage3 refine")
            full.sort(key=base_sweep.rank_key, reverse=True)
        _save("stage3_full.json", full)
        gated = [{"candidate": r["candidate"], "feasible": False}
                 for r in gate_rows if not r["gates"]["ok"]]
        _heatmaps(proxy + gated, os.path.join(args.media_dir, "sweep_proxy_heatmap.png"))
        print(f"[INFO] stage3 leaders:")
        for card in full[:8]:
            print(f"[INFO]   {_fmt(card)}")

        # ---- stage 4: final scoring at the shipped budget ------------------------------------
        finalists = [Candidate.from_json(c["candidate"]) for c in full[: args.final_top]]
        final_payloads = [(c, "final") for c in {*finalists, FRONT}]
        final = _run_pool(pool, _task_score, final_payloads, "stage4 final")
        final.sort(key=base_sweep.rank_key, reverse=True)
        _save("stage4_final.json", final)

    front_card = next(c for c in final if c["key"] == FRONT.key)
    winner = next((c for c in final if c["key"] != FRONT.key), front_card)
    if base_sweep.rank_key(front_card) >= base_sweep.rank_key(winner):
        winner = front_card
    _save("winner.json", {"winner": winner, "front": front_card})

    print("[INFO] ---- final ranking (64-sample funnels) ----")
    for card in final:
        tag = " <- FRONT" if card["key"] == FRONT.key else ""
        print(f"[INFO]   {_fmt(card)}{tag}")
    print(f"[INFO] winner: {winner['key']}")
    print(f"[INFO]   criteria: {winner.get('criteria')}")
    print(f"[INFO]   vs front: {front_card.get('criteria')}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
