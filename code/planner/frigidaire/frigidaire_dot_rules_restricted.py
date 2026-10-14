#!/usr/bin/env python3
"""Restricted random loads (the 2026-10-08 page's "truly random" sampler) scored under New+J only.

All 24 dishes in one shuffled order with the kinds mixed; each takes a uniformly random free pose from its kind's
candidate list (the top-5 run's level-0 families and masks: slots, gaps and leans; no appliance or rack-retraction
collision, dishes may touch by 0.8 mm, no nested bowls, gate bans). A try where any dish finds no free pose is thrown
away. Loads are scored at their packed poses (not settled), New+J at theta 30, J lines only.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_restricted.py --sample --worker 0 --until-utc 12:40
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_restricted.py --score --N 262144 --until-utc 12:50
"""
import argparse
import datetime as dt
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location("top5", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
B = T.B
OUT = ROOT / "data/results/dot_rules/restricted"


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def until_ts(hhmm):
    h, m = (int(v) for v in hhmm.split(":"))
    return dt.datetime.now(dt.timezone.utc).replace(hour=h, minute=m, second=0, microsecond=0).timestamp()


def pack_random(fam, masks, parts, points, bans, seed):
    """One try: shuffled candidate lists (first free = uniform among free) and one shuffled order of all 24 dishes."""
    rng = np.random.default_rng(seed)
    perm = {k: rng.permutation(len(fam[k])) for k in fam}
    fam = {k: [fam[k][i] for i in perm[k]] for k in fam}
    masks = {k: masks[k][perm[k]] for k in masks}
    order = [T.ROSTER[i] for i in rng.permutation(len(T.ROSTER))]
    dishes, entries = T.TolerantDishSet(parts, points), []
    for oid, kind in order:
        pick = next(B.free_iter({"id": oid, "kind": kind}, entries, fam, masks, parts, points, dishes, bans), None)
        if pick is None:
            return None, oid, [o for o, _ in order]
        entries.append(B.entry_of(oid, pick[0]))
        dishes.add(oid, pick[0])
    return entries, None, [o for o, _ in order]


def sample(worker, until):
    fam, parts, points = T.level_families(0)
    masks = T.level_masks(fam, parts, points, 0)
    bans = T.all_bans()
    (OUT / "loads").mkdir(parents=True, exist_ok=True)
    tries_path = OUT / f"tries_w{worker:02d}.jsonl"
    t = 0
    while time.time() < until:
        seed = T.seed_of("restricted", worker, t)
        t0 = time.monotonic()
        entries, failed, order = pack_random(fam, masks, parts, points, bans, seed)
        rec = {"worker": worker, "try": t, "seed": seed, "complete": entries is not None, "failed_at": failed,
               "seconds": round(time.monotonic() - t0, 2)}
        with tries_path.open("a") as f:
            f.write(json.dumps(rec) + "\n")
        if entries is not None:
            lid = f"r{worker:02d}_{t:05d}"
            T.atomic_write(OUT / "loads" / f"{lid}.json", json.dumps(
                {"load_id": lid, "seed": seed, "order": order, "entries": entries}, indent=1) + "\n")
            log(f"complete {lid} after {rec['seconds']} s")
        t += 1
    log(f"[RESULT] PASS worker {worker}: {t} tries")


def score(N, theta, until, poll=20., only=None):
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
    import warp as wp
    eng = DR.DotRuleEngine(N)
    out = OUT / f"scores_N{N:07d}"
    out.mkdir(parents=True, exist_ok=True)
    while True:
        todo = [p for p in sorted((OUT / "loads").glob("*.json")) if not (out / p.name).is_file()
                and (only is None or p.stem in only)]
        for p in todo:
            t0 = time.perf_counter()
            entries = json.loads(p.read_text())["entries"]
            dots, owner, recs, objs = DR.load_dots(entries)
            mesh = eng.build_mesh(DR.load_soup(entries, objs))
            sums = eng.ray_sums(mesh, dots, [theta], fixed=False)
            WF, WG, WJ, _, _ = eng.wstar(dots, sums, [theta])
            nj = eng.scores(sums, WF, WG, WJ)[0]["NJ"][:, 0].cpu().numpy()
            areas = np.array([r["area_m2"] for r in recs])
            E_d = np.array([nj[owner == i].mean() for i in range(len(recs))])
            rec = {"load_id": p.stem, "N": N, "theta_deg": theta, "S_NJ": float((areas * E_d).sum() / areas.sum()),
                   "W_NJ": float(E_d.min()), "above1": int((nj > 1 + 1e-9).sum()),
                   "pooling": [r["id"] for r in recs if r["pools"]],
                   "per_dish": [{"id": r["id"], "kind": r["kind"], "rack": r["rack"], "NJ": float(e)}
                                for r, e in zip(recs, E_d)],
                   "entries": entries, "seconds": round(time.perf_counter() - t0, 1)}
            del mesh
            wp.synchronize_device(eng.device)
            T.atomic_write(out / p.name, json.dumps(rec, indent=1) + "\n")
            log(f"[RESULT] PASS {p.stem}: S_NJ {rec['S_NJ']:.4f}, worst {rec['W_NJ']:.4f}, {rec['seconds']} s")
        if time.time() >= until:
            break
        time.sleep(poll)
    log("[RESULT] PASS score done")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sample", action="store_true")
    ap.add_argument("--score", action="store_true")
    ap.add_argument("--worker", type=int, default=0)
    ap.add_argument("--until-utc", required=True)
    ap.add_argument("--N", type=int, default=262144)
    ap.add_argument("--theta", type=float, default=30.)
    ap.add_argument("--only", default=None, help="comma-separated load ids (rescore a subset)")
    a = ap.parse_args()
    if a.sample:
        sample(a.worker, until_ts(a.until_utc))
    if a.score:
        score(a.N, a.theta, until_ts(a.until_utc), only=set(a.only.split(",")) if a.only else None)


if __name__ == "__main__":
    sys.exit(main())
