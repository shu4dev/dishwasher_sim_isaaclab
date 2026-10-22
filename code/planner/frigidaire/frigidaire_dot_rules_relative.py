#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""The fair load score: each dish over its kind's ceiling under its arm, the plain mean over the dishes (Kit-free,
standard library).

The kind-ceiling score (2026-10-21, plans/2026-10-21-kind-ceiling-score.md; computed per arm since 2026-10-22, never a
typed number): ceiling(kind, arm, theta) = the largest <kind>_obj New+J over every cell of the given grids at the records'
theta whose spot lies inside the arm's disc (distance from the arm centre <= the arm's radius, read from ARM_SOURCES in
exposure.py) and over every tilt swept: the window.GRID payload of a saved Dot Score Rules page (--grid-payload, tilts
0..180, the food side leaning out) and each --grid-obj-dir's grid.json (repeatable; the inward-tilt grid, tilts -60..0).
The grids are cast under the lower arm's disc (dot_rules.GRID_ARM_R); a middle-arm dish uses them too this round, limited
to the middle arm's radius, and the record says so (grids_cast_for). Every .relative.json keeps each ceiling's provenance
in ceilings_kind[arm][kind] (kind, arm, theta, radius, the grids with their tilt ranges and N, the argmax cell, the
value), and reference_kind states the computed values in one line.
Per dish S_kind = min(1, NJ / ceiling[its arm][kind]), 0 for a dish that pools (the pooling list of the score
record beside the .relative.json, exposure.pools: interior more than 2 mm below the rim; a state whose score record is not
there is scored with pooling NOT checked, n_pooling None, and the [RESULT] says PARTIAL); ratio_kind = NJ / ceiling
unclipped. Per state S = the plain mean of S_kind, S_area = its area-weighted mean ("surface reached"), S_by_kind,
n_pooling, n_clipped_kind; written next to the existing fields of every saved .relative.json of the given dirs:
  --loads-dir DIR          <id>.relative.json beside the farm loads' <id>.json score records (one state, "settled")
  --dependency-dir DIR     <id>.relative.json beside the dependency loads' <id>.scores.json (states packed / settled /
                           built when present; repeatable)
  --restricted-dir DIR     DIR/scores_N0262144/<id>.relative.json beside the restricted loads' <id>.json (one state,
                           "packed")
The per-dish NJ, kind and area come from the saved .relative.json, which the per-dish engine reference pass of
2026-10-14 to 2026-10-16 wrote (relative S and the per-spot rescale, retired 2026-10-22: in git history); a score record
with no .relative.json beside it is not scored. --summary prints the ceilings, the Spearman of S and S_area against S_NJ
and against S_rel_best (the retired passes' saved fields), the farm top and bottom five and the pooling / clipped counts.

    python3 code/planner/frigidaire/frigidaire_dot_rules_relative.py --grid-payload data/media/dot_rules/page_v22/index.html \\
        --loads-dir data/results/dot_rules/loads/N0262144 --restricted-dir data/results/dot_rules/restricted \\
        --dependency-dir data/results/dot_rules/dependency/samples_v5 --summary \\
        --grid-obj-dir data/results/dot_rules/ceiling_inward/N0262144
"""
from __future__ import annotations

import argparse
import ast
import datetime as dt
import json
import os
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
RESTRICTED_SCORES = "scores_N0262144"     # the restricted loads rescored at the farm's and the grid's N
EXPOSURE = ROOT / "code/frigidaire/src/dishsim_frigidaire/exposure.py"     # ARM_SOURCES: each arm's disc radius
KINDS = ("plate", "bowl", "cup")


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def spearman(a, b):
    """Rank correlation (ties: mean rank), standard library only."""
    def ranks(x):
        order = sorted(range(len(x)), key=lambda i: x[i])
        r = [0.] * len(x)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and x[order[j + 1]] == x[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2.
            i = j + 1
        return r
    ra, rb = ranks(list(a)), ranks(list(b))
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    cov = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    return cov / (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** .5


def rel(p):
    return os.path.relpath(Path(p).resolve(), ROOT)


def arm_radii():
    """{arm: disc radius in m} from ARM_SOURCES in exposure.py, parsed, not imported (exposure needs numpy)."""
    node = next(n.value for n in ast.parse(EXPOSURE.read_text()).body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "ARM_SOURCES" for t in n.targets))
    return {arm: float(spec["radius"]) for arm, spec in ast.literal_eval(node).items()}


def read_grids(args):
    """[(name, grid)]: the window.GRID payload of a saved Dot Score Rules page, then each --grid-obj-dir's grid.json."""
    grids = []
    if args.grid_payload:
        text = Path(args.grid_payload).read_text()
        i = text.index("window.GRID =") + len("window.GRID =")
        grids.append((rel(args.grid_payload) + " (window.GRID)", json.JSONDecoder().raw_decode(text[i:].lstrip())[0]))
    for d in args.grid_obj_dir or []:
        grids.append((rel(Path(d) / "grid.json"), json.loads((Path(d) / "grid.json").read_text())))
    return grids


def ceilings(grids, theta, radii):
    """{arm: {kind: provenance}}: ceiling(kind, arm, theta) = the largest <kind>_obj New+J over every cell of every grid at
    theta (a grid node) whose spot lies inside the arm's disc (distance from the arm centre <= its radius; grid spots are
    in cm) and over every tilt; the first grid wins a tie. Skipped poses are negative."""
    used = [{"grid": name, "tilts_deg": [min(g["tilts"]), max(g["tilts"])], "N": g["n_points"]} for name, g in grids]
    out = {}
    for arm, r in radii.items():
        for kind in KINDS:
            cells = []
            for name, g in grids:
                nH, nD, nQ = (len(g[k]) for k in ("heights", "distances", "thetas"))
                qi, nj = g["thetas"].index(theta), g[f"{kind}_obj"]["NJ"]
                cells += [(nj[((t * nH + hi) * nD + di) * nQ + qi] / float(g["scale"]), name, h, d, tilt)
                          for t, tilt in enumerate(g["tilts"]) for hi, h in enumerate(g["heights"])
                          for di, d in enumerate(g["distances"]) if d / 100. <= r]
            v, name, h, d, tilt = max(cells, key=lambda c: c[0])
            out.setdefault(arm, {})[kind] = {
                "kind": kind, "arm": arm, "theta_deg": theta, "radius_m": r,
                "radius_from": f"ARM_SOURCES['{arm}']['radius'] in {rel(EXPOSURE)}", "grids": used,
                "grids_cast_for": "lower_arm",       # dot_rules.GRID_ARM_R; every arm uses these grids this round
                "argmax": {"grid": name, "height_cm": h, "distance_cm": d, "tilt_deg": tilt}, "value": v}
    return out


def ceil_line(ceil):
    """Per arm its radius, per kind the ceiling and its argmax cell (tilt, cm up, cm out)."""
    return "; ".join(f"{arm} r {next(iter(c.values()))['radius_m']:g} m: " + ", ".join(
        f"{k} {e['value']:.4f} ({e['argmax']['tilt_deg']:g} deg, {e['argmax']['height_cm']:g} up, "
        f"{e['argmax']['distance_cm']:g} out)" for k, e in c.items()) for arm, c in ceil.items())


def kind_text(ceil):
    """The rule and the computed ceilings with their provenance, one line (each record's reference_kind)."""
    e = next(iter(next(iter(ceil.values())).values()))
    grids = ", ".join(f"{x['grid']} (tilts {x['tilts_deg'][0]:g} to {x['tilts_deg'][1]:g} deg, N {x['N']})"
                      for x in e["grids"])
    return (f"ceiling(kind, arm, theta) = the largest <kind>_obj New+J over every cell at theta {e['theta_deg']:g} whose "
            "spot lies inside the arm's disc (distance from the arm centre <= the arm's radius in exposure.ARM_SOURCES) "
            f"and over every tilt of {grids}; the grids are cast under the {e['grids_cast_for']} disc and serve every "
            f"arm this round. Computed: {ceil_line(ceil)}. S_kind = min(1, NJ / ceiling(kind, the dish's arm)), 0 for a "
            "dish that pools (interior more than 2 mm below the rim); S = mean of S_kind, S_area = area-weighted mean")


def kind_jobs(args):
    """[(score record path, .relative.json path)] for the kind pass: every saved .relative.json of the given dirs and
    the score record beside it (farm <id>.json, dependency <id>.scores.json, restricted <id>.json), present or not."""
    dirs = ([(Path(args.loads_dir), ".json")] if args.loads_dir else []) \
        + [(Path(d), ".scores.json") for d in args.dependency_dir or []] \
        + ([(Path(args.restricted_dir) / RESTRICTED_SCORES, ".json")] if args.restricted_dir else [])
    return [(p.with_name(p.name[:-len(".relative.json")] + ext), p) for d, ext in dirs
            for p in sorted(d.glob("*.relative.json"))]


def pooling_of(src, state):
    """The pooling ids of one state from its score record (dependency: per state; farm, restricted: one state), or None
    when the record is not here (its states are then scored with pooling not checked)."""
    if not src.is_file():
        return None
    rec = json.loads(src.read_text())
    rec = rec[state] if isinstance(rec.get(state), dict) else rec
    if "pooling" not in rec:
        raise SystemExit(f"{src} {state}: the score record has no pooling list")
    return rec["pooling"]


def kind_state(s, ceil, pooling):
    """One state over the kind ceilings: per dish ratio_kind = NJ / ceiling, S_kind = min(1, ratio_kind) or 0 when the
    dish pools; S (plain mean), S_area (area-weighted), S_by_kind, pooling, n_pooling (both None when not checked),
    n_clipped_kind."""
    per = s["per_dish"]
    pset = set(pooling or ())
    unknown = pset - {p["id"] for p in per}
    if unknown:
        raise SystemExit(f"pooling ids not among the dishes: {sorted(unknown)}")
    for p in per:
        k, pools = p["kind"].replace("hotec_", ""), p["id"] in pset
        ratio = p["NJ"] / ceil[p["arm"]][k]["value"]
        p.update({"pools": pools, "ratio_kind": ratio, "S_kind": 0. if pools else min(1., ratio)})
    a = sum(p["area_m2"] for p in per)
    s["S"] = sum(p["S_kind"] for p in per) / len(per)
    s["S_area"] = sum(p["area_m2"] * p["S_kind"] for p in per) / a
    kinds = sorted({p["kind"].replace("hotec_", "") for p in per})
    s["S_by_kind"] = {k: statistics.mean(p["S_kind"] for p in per if p["kind"].replace("hotec_", "") == k) for k in kinds}
    s["pooling"] = None if pooling is None else sorted(pset)
    s["n_pooling"] = None if pooling is None else len(pset)
    s["n_clipped_kind"] = sum(p["ratio_kind"] > 1 and not p["pools"] for p in per)


def kind_pass(kj, ceil, theta):
    """The kind pass: kind_state per state of every .relative.json, the pooling ids from the score record beside it."""
    for src, dst in kj:
        rec = json.loads(dst.read_text())
        if float(rec["theta_deg"]) != theta:
            raise SystemExit(f"{dst}: theta {rec['theta_deg']}, the ceilings are at {theta}")
        for st, s in rec["states"].items():
            kind_state(s, ceil, pooling_of(src, st))
        rec.pop("ceilings_source", None)          # before 2026-10-22: the grid paths only
        rec.update({"reference_kind": kind_text(ceil), "ceilings_kind": ceil,
                    "kind_utc": dt.datetime.utcnow().isoformat() + "Z"})
        tmp = dst.with_name(dst.name + ".tmp")
        tmp.write_text(json.dumps(rec, indent=1) + "\n")
        os.replace(tmp, dst)


def summary_kind(kj, args, ceil):
    """--summary of the kind pass: per group the counts and S range; farm and restricted: Spearman of S and S_area
    against S_NJ and S_rel_best, top and bottom five by S; dependency loads one line per state."""
    def group(dst):
        if args.loads_dir and str(dst).startswith(str(Path(args.loads_dir))):
            return "farm"
        if args.restricted_dir and str(dst).startswith(str(Path(args.restricted_dir))):
            return "restricted"
        return os.path.relpath(dst.parent, ROOT)
    log(kind_text(ceil))
    groups = {}
    for _, dst in kj:
        r = json.loads(dst.read_text())
        groups.setdefault(group(dst), []).append((r["load_id"], r))
    for name, recs in groups.items():
        sts = [s for _, r in recs for s in r["states"].values()]
        dishes = [p for s in sts for p in s["per_dish"]]
        over = [p["ratio_kind"] for p in dishes if p["ratio_kind"] > 1 and not p["pools"]]
        pk = {}
        for p in dishes:
            if p["pools"]:
                pk[p["kind"]] = pk.get(p["kind"], 0) + 1
        unchecked = sum(s["n_pooling"] is None for s in sts)
        log(f"{name}: {len(recs)} loads, {len(sts)} states, {len(dishes)} dishes; pooling "
            + (f"NOT CHECKED in {unchecked} states (no score record here), " if unchecked else "")
            + f"{sum(pk.values())} pooling dishes ({', '.join(f'{k} {v}' for k, v in sorted(pk.items())) or 'none'}) in "
            f"{sum((s['n_pooling'] or 0) > 0 for s in sts)} states; clipped at 1: {len(over)} dishes"
            + (f" ({min(over):.3f} to {max(over):.3f})" if over else "")
            + f"; S min / median / max {min(s['S'] for s in sts):.4f} / {statistics.median(s['S'] for s in sts):.4f} / "
            f"{max(s['S'] for s in sts):.4f}")
        if name not in ("farm", "restricted"):
            for lid, r in recs:
                for k, s in r["states"].items():
                    log(f"{name} {lid} {k}: S {s['S']:.4f}, S_area {s['S_area']:.4f}, "
                        + ", ".join(f"{kk} {v:.4f}" for kk, v in s["S_by_kind"].items())
                        + f"; pooling {s['n_pooling']}, clipped {s['n_clipped_kind']}; S_NJ {s['S_NJ']:.4f}, "
                        f"S_rel_best {s['S_rel_best']:.4f}")
            continue
        st = "settled" if name == "farm" else "packed"
        rows = [(lid, r["states"][st]) for lid, r in recs]
        rank = {key: {x[0]: n + 1 for n, x in enumerate(sorted(rows, key=lambda x: -x[1][key]))}
                for key in ("S_NJ", "S_rel_best")}
        for key in ("S", "S_area"):
            log(f"{name}: Spearman({key}, S_NJ) {spearman([x[1][key] for x in rows], [x[1]['S_NJ'] for x in rows]):.3f}, "
                f"Spearman({key}, S_rel_best) "
                f"{spearman([x[1][key] for x in rows], [x[1]['S_rel_best'] for x in rows]):.3f} over {len(rows)} loads"
                + (f"; Spearman(S, S_area) {spearman([x[1]['S'] for x in rows], [x[1]['S_area'] for x in rows]):.3f}"
                   if key == "S" else ""))
        by_s = sorted(rows, key=lambda x: -x[1]["S"])
        for word, five in (("top", by_s[:5]), ("bottom", by_s[-5:])):
            log(f"{name} {word} five by S: " + "; ".join(
                f"{lid} {s['S']:.4f} (S_area {s['S_area']:.4f}; "
                + ", ".join(f"{k} {v:.3f}" for k, v in s["S_by_kind"].items())
                + f"; pooling {s['n_pooling']}; S_NJ {s['S_NJ']:.4f} rank {rank['S_NJ'][lid]}, S_rel_best rank "
                f"{rank['S_rel_best'][lid]} of {len(rows)})" for lid, s in five))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loads-dir", default=None)
    ap.add_argument("--dependency-dir", action="append", default=None)
    ap.add_argument("--restricted-dir", default=None, help=f"restricted sampler dir: its {RESTRICTED_SCORES}/<id>.json")
    ap.add_argument("--grid-obj-dir", action="append", default=None,
                    help="an object-level grid level (grid.json); repeatable, the ceilings are the max over every grid")
    ap.add_argument("--grid-payload", default=None, help="a saved Dot Score Rules index.html (its window.GRID)")
    ap.add_argument("--summary", action="store_true", help="Spearman, top / bottom five, counts")
    args = ap.parse_args()
    if not (args.grid_obj_dir or args.grid_payload):
        raise SystemExit("the kind ceilings need --grid-obj-dir or --grid-payload")
    t0 = time.time()
    kj = kind_jobs(args)
    tn = {(float(r["theta_deg"]), r["N"]) for r in (json.loads(dst.read_text()) for _, dst in kj)}
    grids = read_grids(args)
    if len(tn) != 1 or any(g["n_points"] != next(iter(tn))[1] for _, g in grids):
        raise SystemExit(f"mixed theta / N: records {tn}, grids {[g['n_points'] for _, g in grids]}")
    theta = next(iter(tn))[0]
    ceil = ceilings(grids, theta, arm_radii())
    kind_pass(kj, ceil, theta)
    if args.summary:
        summary_kind(kj, args, ceil)
    sts = [s for _, dst in kj for s in json.loads(dst.read_text())["states"].values()]
    dishes = [p for s in sts for p in s["per_dish"]]
    bad = [p["id"] for p in dishes if not 0. <= p["S_kind"] <= 1.]
    unchecked = sum(s["n_pooling"] is None for s in sts)
    log(f"[RESULT] {'FAIL' if bad else 'PARTIAL' if unchecked else 'PASS'} kind ceilings: {len(kj)} files, "
        f"{len(sts)} states, {len(dishes)} dishes, ceilings at theta {theta:g} " + ceil_line(ceil) + "; "
        + (f"pooling NOT CHECKED in {unchecked} states (no score record beside their .relative.json); " if unchecked
           else "") + f"{sum(p['pools'] for p in dishes)} pooling (scored 0), "
        f"{sum(p['ratio_kind'] > 1 and not p['pools'] for p in dishes)} clipped at 1"
        + (f"; S_kind outside [0, 1]: {bad}" if bad else "") + f"; in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
