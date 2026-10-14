#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Figures for the Dot Score Rules page at large N (Kit-free, matplotlib Agg).

Reads the dot-rule grid (perdot_<dish>.npz, grid.json), the object-level grid (<dish>_obj entities), the
convergence sweep, the load scores, the Isaac random-load farm and the Isaac dependency graphs, and writes into
--out (three rules, Original / New / New+J, in every figure but the convergence plot):
  dots_<dish>.png                             500 dots x 3 rules x 5 tilts, dish in plate bowl cup (old style)
  convergence.png                             |New+G - New+J| vs N per theta; D+G/D+J vs (a/R)^2
  rand_top5_<r>.png, rand_bottom5_<r>.png     r in o n nj (old r_top5_new.png style)
  rand_spread.png, rand_record.png
  objlevel_slices.png                         object level vs dot level, three slices (--grid-obj-dir)
  depgraph_<id>.png                           ordered-pair dependency graph + rack top views (--dependency-dir)
  depgraph_<id>_resettled.png                 the same on a goal re-settled by a sequential build (--resettled-dir)
The page builder (frigidaire_dot_rules_page.py) imports the readers below. Run from the repo root:
  code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_figures.py \
      --grid-dir data/results/dot_rules/grid/N0262144 \
      --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 \
      --sweep data/results/dot_rules/grid/convergence_sweep.json \
      --loads-dir data/results/dot_rules/loads/N0262144 \
      --dependency-dir data/results/dot_rules/dependency/pairs \
      --farm-dir data/results/random_loads/rl_20261009 --out data/media/dot_rules/figures
A grid dir may also be the grid script's --out: its largest N*/ level holding grid.json is used.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
RULES = ("O", "D", "DG", "DJ", "N", "NG", "NJ")
NAMES = {"O": "Original", "D": "D", "DG": "D+G", "DJ": "D+J", "N": "New", "NG": "New+G", "NJ": "New+J"}
SLUG = {r: r.lower() for r in RULES}
ALIASES = {"o": "O", "orig": "O", "original": "O", "rev5": "O", "d": "D", "dg": "DG", "d+g": "DG", "gd": "DG",
           "dj": "DJ", "d+j": "DJ", "jd": "DJ", "n": "N", "new": "N", "ng": "NG", "new+g": "NG", "newg": "NG",
           "nj": "NJ", "new+j": "NJ", "newj": "NJ"}
TILTS_PERDOT = (0, 45, 90, 135, 180)
R_LOWER, R_MIDDLE = 0.233, 0.191                       # exposure.ARM_SOURCES radii [m]
ARM_Z = {"LowerRack": 0.185, "UpperRack": 0.540}       # disc heights, world, racks in [m]
ARM_Y = 0.008
BODY = {"LowerRack": np.array([0.0, 0.008, 0.215]), "UpperRack": np.array([0.0, 0.008, 0.590])}
BOXES = {"LowerRack": ((-.2625, .2625), (-.2805, .2805), (.002, .3563)),    # release boxes, rack-local [m]
         "UpperRack": ((-.240, .240), (-.2575, .2575), (-.0149, .227))}
EXTENDED = {"LowerRack": {"position_m": [0.0, -0.482, 0.215], "quaternion_xyzw": [0, 0, 0, 1]},   # racks out [m]
            "UpperRack": {"position_m": [0.0, -0.432, 0.590], "quaternion_xyzw": [0, 0, 0, 1]}}
KIND_CMAP = {"plate": "Blues", "bowl": "Oranges", "cup": "Greens"}
RULES3 = ("O", "N", "NJ")                              # the rules shown outside the explorer and the appendix
RULE_COLOUR = {"O": "#8a9099", "N": "#d55e00", "NJ": "#0072b2"}
EDGE_DARK = "#333333"                                   # dependency edges: one style, arrow to the later dish
REASONS = ("knock", "need", "block")                   # why only that order lands both dishes (an edge's JSON attribute)
DRIFT_MM = 2.0                                         # the context re-settles in place: more drift before a drop displaced it
FAIL_RED = "#d00000"
DEFAULT_ASSETS = ROOT / "data/assets/models/hotec_wheatstraw/v2"


def log(*a):
    print("[figures]", *a, flush=True)


def resolve(path):
    p = Path(path)
    if p.is_absolute() or p.exists():
        return p
    return ROOT / p


def read_json(path):
    with open(path) as f:
        return json.load(f)


def rule_dict(d):
    """{rule: float} from a dict keyed O..NJ (or the page's names); None if it has no rule keys."""
    if not isinstance(d, dict):
        return None
    out = {}
    for k, v in d.items():
        r = ALIASES.get(str(k).strip().lower().replace(" ", ""), k if k in RULES else None)
        if r in RULES and isinstance(v, (int, float)):
            out[r] = float(v)
    return out if out else None


def fmt3(v):
    """Old page style: .152, 0 as .000, negatives with a minus sign."""
    s = f"{v:.3f}"
    return s.replace("0.", ".", 1) if abs(v) < 1 else s


# ------------------------------------------------------------------ grid / per-dot / sweep readers
def load_grid(grid_dir):
    return read_json(resolve(grid_dir) / "grid.json")


def level_dir(path):
    """The grid dir itself when it holds grid.json, else its largest N*/ level that does (the grid script's --out)."""
    p = resolve(path)
    if (p / "grid.json").is_file():
        return p
    levels = sorted(p.glob("N*/grid.json"))
    if not levels:
        raise SystemExit(f"no grid.json in {p} or in its N*/ levels")
    log("grid level", levels[-1].parent)
    return levels[-1].parent


def flat_index(grid, tilt, h, d, theta):
    ti, hi = grid["tilts"].index(tilt), grid["heights"].index(h)
    di, qi = grid["distances"].index(d), grid["thetas"].index(theta)
    return ((ti * len(grid["heights"]) + hi) * len(grid["distances"]) + di) * len(grid["thetas"]) + qi


def grid_at(grid, entity, tilt, h, d, theta):
    g, i = grid[entity], flat_index(grid, tilt, h, d, theta)
    sc = float(grid.get("scale", 1e6))
    out = {r: g[r][i] / sc for r in RULES if r in g}
    for k in ("zG", "zJ", "nG", "nJ"):
        if k in g:
            out[k] = g[k][i]
    for k in ("mDown", "mGap"):
        if k in g:
            out[k] = g[k][i] / 10.0
    return out


def load_perdot(grid_dir, dish):
    """perdot_<dish>.npz with the rule axis (scores, means, n_above1) put in RULES order."""
    z = np.load(resolve(grid_dir) / f"perdot_{dish}.npz")
    d = {k: z[k] for k in z.files}
    if "rules" in d:
        order = [list(map(str, d["rules"])).index(r) for r in RULES]
        d["scores"] = d["scores"][:, order]
        for k in ("means", "n_above1"):
            if k in d:
                d[k] = d[k][:, order]
    return d


def _norm_key(k):
    return re.sub(r"[^a-z0-9]", "", str(k).lower())


def _flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = prefix + _norm_key(k)
        if isinstance(v, dict):
            out.update(_flatten(v, key))
        else:
            out[key] = v
    return out


# Field matchers for one (N, theta) row of the sweep: (any-of tokens, all-of tokens, none-of tokens).
_FIELDS = {
    "gap_max": (("ngnj", "gap", "gj", "absdiff"), ("max",), ("dot", "arg", "ratio", "dgdj", "lim", "cell", "p95", "rel")),
    "gap_mean": (("ngnj", "gap", "gj", "absdiff"), ("mean",), ("dot", "ratio", "dgdj", "lim", "rel")),
    "gap_p95": (("ngnj", "gap", "gj", "absdiff"), ("p95",), ("dot", "ratio")),
    "dot_max": (("dot",), ("max",), ("ratio", "arg", "dgdj", "above", "score", "lim")),
    "dot_mean": (("dot",), ("mean",), ("ratio", "dgdj", "score", "lim")),
    "dr_med": (("ratio", "dgdj", "dratio", "arsq", "ar2"), ("med",), ("dot",)),
    "dr_max": (("ratio", "dgdj", "dratio", "arsq", "ar2"), ("max",), ("dot", "arg")),
    "dlim": (("limit", "exact"), (), ("dot",)),
    "dl_med": (("limit",), ("med",), ("dot",)),
    "dl_max": (("limit",), ("max",), ("dot",)),
}
_N_KEYS = ("n", "npoints", "npts", "nlines", "level")
_T_KEYS = ("theta", "thetadeg", "th", "thdeg")


def _match(flat, field):
    anyof, allof, noneof = _FIELDS[field]
    for k, v in flat.items():
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            continue
        if any(t in k for t in anyof) and all(t in k for t in allof) and not any(t in k for t in noneof):
            return k, float(v)
    return None, None


def _rows(obj, ctx=None):
    """Yield flat dicts that carry N and theta, from any nesting of lists / dicts keyed by N or theta."""
    ctx = dict(ctx or {})
    if isinstance(obj, list):
        for x in obj:
            yield from _rows(x, ctx)
        return
    if not isinstance(obj, dict):
        return
    flat_here = {_norm_key(k): v for k, v in obj.items()}
    for k in _N_KEYS:
        if isinstance(flat_here.get(k), (int, float)) and flat_here[k] >= 16:
            ctx["N"] = int(flat_here[k])
            break
    for k in _T_KEYS:
        if isinstance(flat_here.get(k), (int, float)):
            ctx["theta"] = float(flat_here[k])
            break
    scalars = {k: v for k, v in obj.items() if not isinstance(v, (dict, list))}
    subdicts = {k: v for k, v in obj.items() if isinstance(v, dict)}
    if "N" in ctx and "theta" in ctx:
        row = _flatten({k: v for k, v in obj.items() if not isinstance(v, list)})
        row["__N"], row["__theta"] = ctx["N"], ctx["theta"]
        yield row
        return
    for k, v in obj.items():
        if not isinstance(v, (dict, list)):
            continue
        sub = dict(ctx)
        m = re.fullmatch(r"(?:n|N)?_?0*(\d+)", str(k))
        t = re.fullmatch(r"(?:theta|th|t|q)?_?(\d+(?:\.\d+)?)", str(k))
        if m and int(m.group(1)) >= 64 and "N" not in ctx:
            sub["N"] = int(m.group(1))
        elif t and float(t.group(1)) <= 90 and "N" in ctx:
            sub["theta"] = float(t.group(1))
        yield from _rows(v, sub)


def load_sweep(path):
    """Normalized sweep: {"N": [...], "thetas": [...], "rows": {(N, theta): {field: value}}, "keys": {...}}."""
    path = resolve(path)
    if str(path).endswith(".csv"):
        with open(path) as f:
            raw = [dict((k, float(v)) if _isnum(v) else (k, v) for k, v in r.items()) for r in csv.DictReader(f)]
    else:
        raw = read_json(path)
    rows, used = {}, {}
    for r in _rows(raw):
        rec = {}
        for field in _FIELDS:
            k, v = _match(r, field)
            if k is not None:
                rec[field] = v
                used.setdefault(field, k)
        if isinstance(r.get("ncells"), (int, float)):
            rec["n_cells"] = int(r["ncells"])
        if rec:
            rows.setdefault((r["__N"], r["__theta"]), {}).update(rec)
    if not rows or not any("gap_max" in v for v in rows.values()):
        raise SystemExit(f"convergence sweep {path}: found no (N, theta) rows with a max |NG-NJ| field")
    log("sweep fields matched:", used)
    Ns = sorted({k[0] for k in rows})
    thetas = sorted({k[1] for k in rows})
    meta = raw.get("meta", {}) if isinstance(raw, dict) else {}
    return {"N": Ns, "thetas": thetas, "rows": rows, "keys": used, "meta": meta}


def _isnum(v):
    try:
        float(v)
        return True
    except (TypeError, ValueError):
        return False


# ------------------------------------------------------------------ loads / farm readers
def qmat(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _kind_of(oid, kind=None):
    k = (kind or str(oid).split("_")[0]).lower()
    for base in ("plate", "bowl", "cup"):
        if base in k:
            return base
    return k


def _pose(p):
    pos = p.get("position_m", p.get("position"))
    q = p.get("quaternion_xyzw", p.get("quaternion"))
    return np.asarray(pos, float), np.asarray(q, float)


def load_objects(result):
    """Rack-local dish poses [{object_id, kind, rack, t (3,), Rm (3,3)}] from a farm result.json or entries."""
    out = []
    if isinstance(result.get("objects"), list) and result["objects"]:
        snap = (result.get("initial_snapshot") or {}).get("poses", {}) or {}
        racks = result.get("racks") or {}
        for o in result["objects"]:
            oid = o.get("object_id", o.get("id"))
            rack = racks.get(oid, o.get("rack"))
            if o.get("rack_local_pose"):
                t, q = _pose(o["rack_local_pose"])
                Rm = qmat(q)
            else:
                pw, qw = _pose(o.get("pose_world") or snap[oid])
                pr, qr = _pose(snap.get(rack) or EXTENDED.get(rack, EXTENDED["LowerRack"]))
                Rr = qmat(qr)
                t, Rm = Rr.T @ (pw - pr), Rr.T @ qmat(qw)
            out.append({"object_id": oid, "kind": _kind_of(oid, o.get("kind")), "rack": rack, "t": t, "Rm": Rm})
    elif isinstance(result.get("entries"), list):
        for e in result["entries"]:
            t, q = _pose(e)
            oid = e.get("id", e.get("object_id"))
            out.append({"object_id": oid, "kind": _kind_of(oid, e.get("kind")), "rack": e["rack"],
                        "t": t, "Rm": qmat(q)})
    elif isinstance(result.get("initial_snapshot"), dict) and result.get("racks"):
        snap = result["initial_snapshot"]["poses"]
        for oid, rack in result["racks"].items():
            pw, qw = _pose(snap[oid])
            pr, qr = _pose(snap[rack])
            Rr = qmat(qr)
            out.append({"object_id": oid, "kind": _kind_of(oid), "rack": rack,
                        "t": Rr.T @ (pw - pr), "Rm": Rr.T @ qmat(qw)})
    for o in out:
        if o["rack"] not in BODY:            # basket-supported dishes count as the lower rack
            o["rack"] = "LowerRack"
    return out


def load_farm(farm_dir):
    farm_dir = resolve(farm_dir)
    harvest = read_json(farm_dir / "harvest.json") if (farm_dir / "harvest.json").exists() else {}
    accepted = []
    if (farm_dir / "accepted.jsonl").exists():
        for line in open(farm_dir / "accepted.jsonl"):
            line = line.strip()
            if not line:
                continue
            try:
                accepted.append(json.loads(line))
            except json.JSONDecodeError:
                continue                            # a partial last line
    config = read_json(farm_dir / "config.json") if (farm_dir / "config.json").exists() else {}
    precheck = (farm_dir / "CLOSURE_PRECHECK").exists() or bool(_find(config, "precheck_enabled"))
    return {"dir": farm_dir, "harvest": harvest, "accepted": accepted, "config": config, "precheck": precheck}


def _find(obj, key):
    """First value of `key` anywhere in a nested dict / list (None if absent)."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            r = _find(v, key)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find(v, key)
            if r is not None:
                return r
    return None


def load_records(loads_dir, farm):
    """Scored accepted loads in finish order: [{load_id, S, W, feasible, pooling_count, N, theta, finished_utc,
    result_path, order_index}]."""
    loads_dir = resolve(loads_dir)
    acc = {a["load_id"]: a for a in farm["accepted"] if "load_id" in a}
    recs = []
    for path in sorted(glob.glob(str(loads_dir / "*.json"))):
        name = Path(path).stem
        if name in ("summary", "meta", "calibration"):
            continue
        try:
            d = read_json(path)
        except (json.JSONDecodeError, OSError):
            continue
        S = rule_dict(d.get("S")) or rule_dict({k[2:]: v for k, v in d.items() if str(k).startswith("S_")})
        if not S or any(r not in S for r in RULES):
            continue
        lid = d.get("load_id", name)
        a = acc.get(lid, {})
        rp = a.get("path") or d.get("source") or d.get("result_path") or d.get("path")
        if rp is None:
            rp = str(farm["dir"] / "loads" / lid / "result.json")
        da = d.get("accepted") if isinstance(d.get("accepted"), dict) else {}
        recs.append({"load_id": lid, "S": S,
                     "W": rule_dict(d.get("W")) or {},
                     "feasible": d.get("feasible"), "pooling_count": d.get("pooling_count"),
                     "N": d.get("N", d.get("n_points")), "theta": d.get("theta", d.get("theta_deg")),
                     "finished_utc": a.get("finished_utc") or da.get("finished_utc") or d.get("finished_utc") or "",
                     "wall_s": a.get("wall_s", da.get("wall_s")), "result_path": rp, "json": d})
    if acc:                                                   # rank only this farm run's accepted loads
        dropped = [r["load_id"] for r in recs if r["load_id"] not in acc]
        recs = [r for r in recs if r["load_id"] in acc]
        if dropped:
            log(f"ignored {len(dropped)} scored loads not in {farm['dir']}/accepted.jsonl: {dropped[:5]}")
    recs.sort(key=lambda r: (r["finished_utc"] or "~", r["load_id"]))
    for i, r in enumerate(recs):
        r["order_index"] = i + 1
    return recs


def attach_objects(recs):
    for r in recs:
        if "objects" in r:
            continue
        p = resolve(r["result_path"])
        try:
            res = read_json(p)
        except (OSError, json.JSONDecodeError):
            res = r["json"]
        r["objects"] = load_objects(res) or load_objects(r["json"])
        r["params"] = res.get("params", {}) if isinstance(res, dict) else {}
        r["switches"] = {k: bool(res.get(k)) for k in ("precheck_enabled", "contact_undo_enabled",
                                                        "failfast_contact_enabled")}
    return recs


def spearman(a, b):
    from scipy.stats import spearmanr
    if len(a) < 3:
        return float("nan")
    return float(spearmanr(a, b).correlation)


def summarize(recs):
    """Numbers shared by the figure captions and the page."""
    S = {r: np.array([x["S"][r] for x in recs]) for r in RULES}
    out = {"n": len(recs), "rules": {}}
    for r in RULES:
        if not len(recs):
            break
        order = np.argsort(-S[r], kind="stable")
        out["rules"][r] = {"min": float(S[r].min()), "median": float(np.median(S[r])), "max": float(S[r].max()),
                           "best": recs[int(order[0])]["load_id"], "worst": recs[int(order[-1])]["load_id"],
                           "top5": [recs[int(i)]["load_id"] for i in order[:5]],
                           "bottom5": [recs[int(i)]["load_id"] for i in order[::-1][:5]]}
    pairs = (("O", "N"), ("N", "NJ"), ("O", "NJ"))                      # the three-rule spread and captions
    out["spearman"] = {f"{a}-{b}": spearman(S[a], S[b]) for a, b in pairs} if len(recs) >= 3 else {}
    out["pooling_loads"] = sum(1 for x in recs if x["feasible"] is False or (x["pooling_count"] or 0) > 0)
    out["by_id"] = {x["load_id"]: x for x in recs}
    return out


# ------------------------------------------------------------------ geometry for outlines
def kind_points(assets):
    """3-D convex-hull vertices of each HOTEC visual mesh (asset frame); projections keep the same 2-D hull."""
    sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
    from dishsim_frigidaire import loading
    from scipy.spatial import ConvexHull
    pts = {}
    for k in ("plate", "bowl", "cup"):
        p = loading.visual_points(Path(assets) / f"{k}.usda")
        pts[k] = p[ConvexHull(p).vertices]
    return pts


def hull2(xy):
    from scipy.spatial import ConvexHull
    try:
        return xy[ConvexHull(xy).vertices]
    except Exception:                                      # degenerate (edge-on) projection
        return xy


def tint(o):
    m = re.search(r"(\d+)$", str(o["object_id"]))
    i = (int(m.group(1)) - 1) % 8 if m else abs(hash(o["object_id"])) % 8
    import matplotlib.pyplot as plt
    return plt.get_cmap(KIND_CMAP.get(o["kind"], "Greys"))(np.linspace(0.35, 0.85, 8)[i])


# ------------------------------------------------------------------ figures
def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.titlesize": 10.5, "figure.dpi": 100, "savefig.dpi": 100})
    return plt


def fig_perdot(npz, dish, verts_local, n_points, out, rules=RULES3):
    plt = _mpl()
    from matplotlib.colors import Normalize
    from matplotlib.colorbar import ColorbarBase
    cmap, norm = plt.get_cmap("viridis"), Normalize(0.0, 1.0)
    tilts = [int(t) for t in npz["tilts"]]
    scores, sumc, nl = npz["scores"].astype(float), npz["sumc"], npz["nlines"]
    means = npz["means"] if "means" in npz else scores.mean(axis=2)
    above = npz["n_above1"] if "n_above1" in npz else (scores > 1 + 1e-9).sum(axis=2)
    h_cm = int(npz["h_cm"]) if "h_cm" in npz else 14
    d_cm = int(npz["d_cm"]) if "d_cm" in npz else 0
    theta = float(npz["theta_deg"]) if "theta_deg" in npz else 30.0
    nfmt = f"{int(n_points):,}"
    posed, sil = [], []
    for ti in range(len(tilts)):
        R, p = npz["R"][ti], npz["p"][ti]
        pos = npz["pos"][ti]
        if abs(pos[:, 2].mean() - h_cm / 100) > 0.03:     # local dots: pose them like the dish
            pos = pos @ R.T + p
        posed.append(pos)
        V = verts_local @ R.T + p
        sil.append(hull2(V[:, [0, 2]]))
    allx = np.concatenate([np.abs(s[:, 0]) for s in sil] + [np.abs(q[:, 0]) for q in posed])
    allz = np.concatenate([s[:, 1] for s in sil] + [q[:, 2] for q in posed])
    xr, zt = max(0.13, allx.max() + 0.012), allz.max() + 0.015
    names = [NAMES[r] for r in rules]
    which = ("the seven rules" if len(rules) == len(RULES) else names[0] if len(rules) == 1
             else ", ".join(names[:-1]) + " and " + names[-1])
    where = "centre" if d_cm == 0 else f"centre + {d_cm} cm"
    lines = [f"Alone, own mesh the only blocker; food-contact centre {h_cm} cm over the lower-arm disc {where}; "
             f"θ {theta:g}°, N = {nfmt} lines per dot.",
             "Seen along world Y (nearer dots on top); black bar: the disc. Tilt 180°: the dish's own orientation."]
    meaning = {"O": "Original: Σ c·open ÷ the dot's own weight Σ c over all the fixed lines.",
               "N": f"New: Σ c·open over all {nfmt} fixed lines ÷ the best-facing weight over the same lines.",
               "NG": "New+G: New over the fixed lines within θ of vertical below the dot.",
               "NJ": "New+J: New over the dot's own cone lines on the disc (its θ-circle, radius h·tan θ)."}
    lines += [meaning[r] for r in rules if r in meaning]
    if any(r in ("D", "DG", "DJ") for r in rules):
        lines.append("D, D+G, D+J: Σ c·open over all fixed lines / those within θ / the dot's own cone lines, ÷ N.")
    lines.append("One 0-to-1 scale; dark outline: a dot above 1 (counted in the title); hollow ring: a dot with no "
                 "weight (Σ c = 0, or no line in its set).")
    nc = len(rules)
    head = 0.62 + len(lines) * 10 * 1.4 / 72 + 0.7      # title, subtitle and the first row's two-line titles [in]
    W, H = 2.0 + 3.0 * nc, head + 3.05 * len(tilts)
    fig, axes = plt.subplots(len(tilts), nc, figsize=(W, H), squeeze=False)
    fig.subplots_adjust(left=0.9 / W, right=1 - 1.1 / W, top=1 - head / H, bottom=0.2 / H, wspace=0.12, hspace=0.3)
    colt = {"O": "Original (revision 5)", "D": f"D: divide by all {nfmt}", "DG": f"D+G: within θ, ÷ {nfmt}",
            "DJ": f"D+J: cone on disc, ÷ {nfmt}", "N": f"New: all {nfmt} fixed lines", "NG": "New+G: within θ",
            "NJ": "New+J: cone on disc"}
    short = {"O": "original", "D": "D", "DG": "D+G", "DJ": "D+J", "N": "New", "NG": "New+G", "NJ": "New+J"}
    rowlab = {0: "\nfacing the arm", 90: "\non edge", 180: "\nfood side up"}
    for ti, t in enumerate(tilts):
        pos = posed[ti]
        order = np.argsort(-pos[:, 1], kind="stable")      # seen along +Y: far (large y) first, near on top
        for ci, r in enumerate(rules):
            ri = RULES.index(r)                            # the npz rule axis is in RULES order (load_perdot)
            ax = axes[ti, ci]
            s = scores[ti, ri]
            if r in ("O", "D"):
                empty = sumc[ti, 0] <= 0
            elif r == "DG":
                empty = sumc[ti, 1] <= 0
            elif r == "DJ":
                empty = sumc[ti, 2] <= 0
            elif r == "NG":
                empty = nl[ti, 0] <= 0
            elif r == "NJ":
                empty = nl[ti, 1] <= 0
            else:
                empty = np.zeros(len(s), bool)
            hi = s > 1 + 1e-9
            fc = cmap(norm(np.clip(s, 0, 1)))
            fc[empty] = (0, 0, 0, 0)
            ec = fc.copy()
            ec[empty] = cmap(0.0)
            ec[hi] = (0.1, 0.05, 0.0, 1.0)
            lw = np.where(empty | hi, 0.9, 0.0)
            ax.fill(sil[ti][:, 0], sil[ti][:, 1], color="#d6d6d6", zorder=0, lw=0)
            ax.plot([-R_LOWER, R_LOWER], [0, 0], color="k", lw=3.2, solid_capstyle="butt", zorder=1)
            ax.scatter(pos[order, 0], pos[order, 2], s=11, c=fc[order], edgecolors=ec[order],
                       linewidths=lw[order], zorder=2)
            ax.set_xlim(-xr, xr)
            ax.set_ylim(-0.012, zt)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_color("#cccccc")
            title = f"{short[r]}: {means[ti, ri]:.3f}"
            if int(above[ti, ri]) > 0:
                title += f"  ({int(above[ti, ri])} above 1)"
            ax.set_title((colt[r] + "\n" + title) if ti == 0 else title, fontsize=10.5)
        axes[ti, 0].set_ylabel(f"tilt {t}°" + rowlab.get(t, ""), fontsize=11)
    cax = fig.add_axes([1 - 0.85 / W, 0.12, 0.17 / W, 0.62])
    ColorbarBase(cax, cmap=cmap, norm=norm, extend="max", label="dot score")
    fig.text(0.9 / W, 1 - 0.2 / H, f"HOTEC v2 {dish}: the 500 dot scores under {which}, tilt 0° to 180°",
             fontsize=15, va="top")
    fig.text(0.9 / W, 1 - 0.62 / H, "\n".join(lines), fontsize=10, va="top", color="#555555", linespacing=1.4)
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


def fig_convergence(sw, out):
    plt = _mpl()
    Ns, thetas, rows = sw["N"], sw["thetas"], sw["rows"]
    cm = plt.get_cmap("viridis")
    fig, axes = plt.subplots(1, 3, figsize=(19, 5.6))
    fig.subplots_adjust(left=0.05, right=0.88, top=0.84, bottom=0.12, wspace=0.28)
    panels = ((axes[0], "gap_max", "gap_mean", "dish means: max (solid) and mean (dashed) of |New+G − New+J|"),
              (axes[1], "dot_max", "dot_mean", "single dots: max (solid) and mean (dashed) of |New+G − New+J|"),
              (axes[2], "dl_med", "_none",
               "D+G against its limit Σ_i (a_i/R)²·D+J_i / 500, a_i = h_i·tan θ per dot: median relative gap"))
    x = np.array(Ns, float)
    for ax, k1, k2, title in panels:
        for i, th in enumerate(thetas):
            c = cm(i / max(1, len(thetas) - 1))
            for key, ls in ((k1, "-"), (k2, "--")):
                y = np.array([rows.get((n, th), {}).get(key, np.nan) for n in Ns], float)
                y[~(y > 0)] = np.nan
                if np.isfinite(y).any():
                    ax.plot(x, y, ls, color=c, lw=1.6 if ls == "-" else 1.1, marker="o" if ls == "-" else None,
                            ms=3.5, label=f"θ {th:g}°" if ls == "-" else None)
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_xlabel("N, lines per set")
        ax.set_title(title, fontsize=10.5, loc="left", wrap=True)
        ax.grid(True, which="major", color="#e4e4e4", lw=0.8)
        ylo, yhi = ax.get_ylim()
        if np.isfinite(ylo) and len(Ns) > 1:
            x0 = x[0]
            ref = [v for v in (rows.get((Ns[0], th), {}).get(k1) for th in thetas) if v and v > 0]
            if ref:
                y0 = float(np.median(ref))
                ax.plot(x, y0 * (x / x0) ** -0.5, color="#888888", lw=1, ls=":", label="∝ N^-1/2")
                ax.plot(x, y0 * (x / x0) ** -1.0, color="#bbbbbb", lw=1, ls="-.", label="∝ N^-1")
                ax.set_ylim(ylo, yhi)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="center right", fontsize=9.5, frameon=False, bbox_to_anchor=(0.995, 0.5))
    cells = [v.get("n_cells") for v in rows.values() if v.get("n_cells")]
    fig.suptitle("New+G and New+J meet as N grows; D+G tends to (a/R)² × D+J dot by dot  "
                 f"(full grid sweep, N = {Ns[0]:,} to {Ns[-1]:,}"
                 + (f"; {max(cells):,} dish poses per θ" if cells else "") + ")", fontsize=13, x=0.05, ha="left")
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


def _box(ax, lo, hi, **kw):
    ax.plot([lo[0], hi[0], hi[0], lo[0], lo[0]], [lo[1], lo[1], hi[1], hi[1], lo[1]], **kw)


def fig_loads(sel, rule, top, n_total, kpts, boxes, n_points, theta, out):
    plt = _mpl()
    from matplotlib.patches import Patch
    fig, axes = plt.subplots(3, 5, figsize=(21, 12.8))
    fig.subplots_adjust(left=0.04, right=0.99, top=0.83, bottom=0.08, wspace=0.28, hspace=0.22)
    word = "highest" if top else "lowest"
    for col in range(5):
        if col >= len(sel):
            for ax in axes[:, col]:
                ax.axis("off")
            continue
        rec = sel[col]
        objs = rec["objects"]
        shapes = []
        for o in objs:
            P = kpts[o["kind"]] @ o["Rm"].T + o["t"]
            shapes.append((o, P, P + BODY[o["rack"]]))
        for row, rack in ((0, "LowerRack"), (1, "UpperRack")):
            ax = axes[row, col]
            mine = sorted([s for s in shapes if s[0]["rack"] == rack], key=lambda s: s[1][:, 2].mean())
            for o, P, _ in mine:
                xy = hull2(P[:, :2])
                ax.fill(xy[:, 0], xy[:, 1], facecolor=tint(o), edgecolor="#333333", lw=0.5, alpha=0.85)
            (x0, x1), (y0, y1), _ = boxes[rack]
            _box(ax, (x0, y0), (x1, y1), color="#777777", ls="--", lw=0.9)
            ax.set_xlim(-0.31, 0.31)
            ax.set_ylim(-0.31, 0.31)
            ax.set_aspect("equal")
            ax.set_xlabel("x [m]", fontsize=8.5, labelpad=1)
            ax.set_ylabel("y [m]", fontsize=8.5, labelpad=1)
            ax.tick_params(labelsize=7.5)
            ax.text(0.02, 0.985, f"top, {'lower' if row == 0 else 'upper'} rack", transform=ax.transAxes,
                    fontsize=8, va="top", color="#444444")
        ax = axes[2, col]
        for o, _, Pw in sorted(shapes, key=lambda s: s[2][:, 0].mean()):     # seen from +x: far (small x) first
            yz = hull2(Pw[:, 1:])
            ax.fill(yz[:, 0], yz[:, 1], facecolor=tint(o), edgecolor="#333333", lw=0.5, alpha=0.85)
        for rack in ("LowerRack", "UpperRack"):
            _, (y0, y1), (z0, z1) = boxes[rack]
            b = BODY[rack]
            _box(ax, (y0 + b[1], z0 + b[2]), (y1 + b[1], z1 + b[2]), color="#777777", ls="--", lw=0.9)
        for rack, Rd in (("LowerRack", R_LOWER), ("UpperRack", R_MIDDLE)):
            ax.plot([ARM_Y - Rd, ARM_Y + Rd], [ARM_Z[rack]] * 2, color="#444444", lw=2.6, solid_capstyle="butt")
        ax.set_xlim(-0.34, 0.34)
        ax.set_ylim(0.15, 0.86)
        ax.set_aspect("equal")
        ax.set_xlabel("y [m]", fontsize=8.5, labelpad=1)
        ax.set_ylabel("z [m]", fontsize=8.5, labelpad=1)
        ax.tick_params(labelsize=7.5)
        ax.text(0.02, 0.985, "side (from +x)", transform=ax.transAxes, fontsize=8, va="top", color="#444444")
        S = rec["S"]
        pool = "" if rec.get("feasible") is not False else "  (pools)"
        axes[0, col].set_title(f"{word} #{col + 1} by {NAMES[rule]}: {rec['load_id']}{pool}\n"
                               + "  ".join(f"{NAMES[r]} {fmt3(S[r])}" for r in RULES3), fontsize=9.5)
    fig.suptitle(f"The {min(5, n_total)} {word}-scoring of {n_total} accepted random 24-dish loads under {NAMES[rule]}\n"
                 f"(sequential random drops, settled and gated in Isaac; N = {int(n_points):,} lines per dot, "
                 f"θ {theta:g}°, per-rack arms; S under Original, New and New+J; dashed: the release box)", fontsize=12.5)
    fig.legend(handles=[Patch(facecolor=plt.get_cmap(c)(0.6), edgecolor="#333333", label=f"{k} (8 tints)")
                        for k, c in KIND_CMAP.items()], loc="lower center", ncol=3, frameon=False, fontsize=10)
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


def fig_spread(recs, summ, out):
    plt = _mpl()
    S = {r: np.array([x["S"][r] for x in recs]) for r in RULES3}
    n = len(recs)
    fig, axes = plt.subplots(1, 4, figsize=(22, 5.3), gridspec_kw={"width_ratios": [1, 1, 1, 1.12]})
    fig.subplots_adjust(left=0.04, right=0.97, top=0.88, bottom=0.13, wspace=0.27)
    bins = max(8, min(40, int(round(2 * n ** 0.5))))
    for ax, r in ((axes[0], "N"), (axes[1], "NJ")):
        ax.hist(S[r], bins=bins, color="#56b4e9", edgecolor="#1f5f87", lw=0.8)
        med, best = float(np.median(S[r])), float(S[r].max())
        ax.axvline(med, color="#d55e00", lw=1.6, label=f"median {med:.3f}")
        ax.axvline(best, color="k", lw=1.1, ls=":", label=f"best {best:.3f}")
        ax.set_title(f"{NAMES[r]}: {n} accepted loads of 24 dishes, median {med:.3f}")
        ax.set_xlabel(f"S under {NAMES[r]}")
        ax.set_ylabel("loads")
        ax.legend(fontsize=8.5, frameon=False)
    sc = None
    for ax, (a, b) in ((axes[2], ("O", "N")), (axes[3], ("N", "NJ"))):
        sc = ax.scatter(S[a], S[b], c=S["NJ"], cmap="viridis", s=16, edgecolors="none")
        ia, ib = int(np.argmax(S[a])), int(np.argmax(S[b]))
        ax.scatter([S[a][ia]], [S[b][ia]], s=120, facecolors="none", edgecolors="k", lw=1.3,
                   label=f"best by {NAMES[a]}")
        ax.scatter([S[a][ib]], [S[b][ib]], s=120, marker="s", facecolors="none", edgecolors="k", lw=1.3,
                   label=f"best by {NAMES[b]}")
        rho = summ["spearman"].get(f"{a}-{b}", float("nan"))
        ax.set_title(f"{NAMES[a]} vs {NAMES[b]} (Spearman {rho:.2f})")
        ax.set_xlabel(f"S under {NAMES[a]}")
        ax.set_ylabel(f"S under {NAMES[b]}")
        ax.legend(fontsize=8.5, frameon=False, loc="lower right")
    cb = fig.colorbar(sc, ax=axes[3], fraction=0.05, pad=0.02)
    cb.set_label("S under New+J (colour of both scatters)")
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


def fig_record(recs, out, rules=RULES3):
    plt = _mpl()
    n = np.arange(1, len(recs) + 1)
    k = len(rules) + 1
    nc = 2 if k <= 4 else 4
    nr = -(-k // nc)
    W, H = 6.8 * nc, 4.3 * nr + 1.2
    fig, axes = plt.subplots(nr, nc, figsize=(W, H), squeeze=False)
    fig.subplots_adjust(left=0.8 / W, right=0.98, top=1 - 1.15 / H, bottom=0.6 / H, wspace=0.28, hspace=0.38)
    flat = list(axes.flat)
    for ax, r in zip(flat, rules):
        s = np.array([x["S"][r] for x in recs])
        best, worst = np.maximum.accumulate(s), np.minimum.accumulate(s)
        ax.scatter(n, s, s=7, color="#b8c4d0", zorder=1, label="each load")
        ax.step(n, best, where="post", color="#0072b2", lw=1.8, label="best so far", zorder=2)
        ax.step(n, worst, where="post", color="#d55e00", lw=1.5, ls="--", label="worst so far", zorder=2)
        ax.set_title(f"{NAMES[r]}: best {best[-1]:.3f}, worst {worst[-1]:.3f}")
        ax.set_xlabel("n, accepted loads in finish order")
        ax.set_ylabel(f"S under {NAMES[r]}")
        ax.grid(True, color="#ececec", lw=0.8)
    flat[0].legend(fontsize=8, frameon=False)
    ax = flat[len(rules)]
    p = np.minimum(1.0, 5.0 / (n + 1))
    ax.plot(n, p, color="k", lw=1.6)
    ax.axhline(0.02, color="#d55e00", lw=1, ls=":", label="2%")
    ax.set_yscale("log")
    ax.set_title(f"5/(n+1) = {p[-1]:.3f} at n = {len(recs)}")
    ax.set_xlabel("n, accepted loads")
    ax.set_ylabel("chance one more load enters the top 5")
    ax.legend(fontsize=8, frameon=False)
    ax.grid(True, which="both", color="#ececec", lw=0.8)
    for ax in flat[len(rules) + 1:]:
        ax.axis("off")
    fig.suptitle("Record curves: best-so-far and worst-so-far S as accepted random loads arrive\n"
                 "(independent draws, so the next load beats the current top five with chance 5/(n+1))",
                 fontsize=12.5, x=0.8 / W, ha="left")
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


def fig_objlevel(grid, out, rules=RULES3, tilt=0, h=14, d=0, theta=30):
    """Object level (solid) against dot level (dashed) per dish: tilt, height and distance slices at one θ.
    Returns False (nothing drawn) when the grid has no <dish>_obj entity."""
    plt = _mpl()
    from matplotlib.lines import Line2D
    dishes = [k for k in ("plate", "bowl", "cup") if k in grid and f"{k}_obj" in grid]
    if not dishes:
        log("the grid has no <dish>_obj entity: objlevel_slices.png not drawn")
        return False
    sc, ndots = float(grid.get("scale", 1e6)), int(grid.get("dots", 500))

    def near(key, v):                                       # the grid node nearest to v
        return grid[key][int(np.argmin(np.abs(np.asarray(grid[key], float) - v)))]
    t0, h0, d0, q0 = near("tilts", tilt), near("heights", h), near("distances", d), near("thetas", theta)
    cols = (("tilts", "tilt (°): 0 = facing the arm, 90 = on edge, 180 = facing away",
             f"tilt slice, {h0} cm up, {d0} cm out"),
            ("heights", "height above the disc (cm)", f"height slice, tilt {t0}°, {d0} cm out"),
            ("distances", "distance from the disc centre (cm)", f"distance slice, tilt {t0}°, {h0} cm up"))
    H = 3.7 * len(dishes) + 1.9
    fig, axes = plt.subplots(len(dishes), 3, figsize=(17, H), squeeze=False)
    fig.subplots_adjust(left=0.065, right=0.985, top=1 - 1.05 / H, bottom=1.05 / H, wspace=0.16, hspace=0.45)
    for ri, dish in enumerate(dishes):
        for ci, (key, xlabel, title) in enumerate(cols):
            ax = axes[ri, ci]
            xs = list(grid[key])
            for ent, ls, lw in ((dish, "--", 1.1), (f"{dish}_obj", "-", 2.0)):   # the dot level under the object level
                g = grid[ent]
                for r in rules:
                    if r not in g:
                        continue
                    ys = []
                    for x in xs:
                        p = {"tilts": t0, "heights": h0, "distances": d0}
                        p[key] = x
                        v = g[r][flat_index(grid, p["tilts"], p["heights"], p["distances"], q0)]
                        ys.append(np.nan if v < 0 else v / sc)           # -1: a skipped pose, drawn as a gap
                    ax.plot(xs, ys, ls=ls, lw=lw, color=RULE_COLOUR[r], marker="o" if ls == "-" else None, ms=2.8,
                            zorder=3 if ls == "-" else 2)
            if key == "distances":
                ax.axvline(100 * R_LOWER, color="#999999", lw=0.9, ls=":", zorder=1)
            ax.set_xlim(min(xs), max(xs))
            ax.set_ylim(0, 1.02)
            ax.grid(True, color="#ececec", lw=0.8)
            ax.set_xlabel(xlabel, fontsize=9.5)
            if ci == 0:
                ax.set_ylabel(f"{dish}\nscore", fontsize=11)
            ax.set_title(f"{dish}, {title}", fontsize=10.5, loc="left")
    handles = [Line2D([], [], color=RULE_COLOUR[r], lw=2.4, label=NAMES[r]) for r in rules]
    handles += [Line2D([], [], color="#333333", lw=2.0, marker="o", ms=3, label="object level"),
                Line2D([], [], color="#333333", lw=1.1, ls="--", label="dot level"),
                Line2D([], [], color="#999999", lw=0.9, ls=":", label="disc edge, 23.3 cm")]
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), frameon=False, fontsize=10.5)
    fig.suptitle(f"Object level against dot level, each dish alone (own mesh the only blocker), θ {q0:g}°, "
                 f"N = {int(grid['n_points']):,} lines per set\n"
                 f"Object level (solid): Σ over the {ndots} dots of the numerator ÷ Σ of the per-dot denominators; "
                 f"dot level (dashed): the mean of the {ndots} dot scores; gaps: skipped poses",
                 fontsize=12, x=0.065, ha="left")
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)
    return True


# ------------------------------------------------------------------ Isaac dependency graphs (ordered-pair test)
def load_dependency(dep_dir):
    """[{dep, scores, path}] for every <id>.json of a dependency dir; scores from the <id>.scores.json beside it
    (names starting with _ are the Kit's own records, skipped)."""
    out = []
    for p in sorted(resolve(dep_dir).glob("*.json")):
        if p.name.endswith(".scores.json") or p.name.startswith("_"):
            continue
        try:
            dep = read_json(p)
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(dep, dict) or "status" not in dep:
            continue
        dep.setdefault("load_id", p.stem)
        sp = p.with_name(p.stem + ".scores.json")
        try:
            scores = read_json(sp) if sp.is_file() else None
        except (OSError, json.JSONDecodeError):
            scores = None
        out.append({"dep": dep, "scores": scores, "path": p})
    return out


def dep_edges(dep):
    """(edges [(a, b)], two-way pairs [(a, b) with a < b], reason counts) of an ordered-pair record. An edge a before
    b carries its reasons (knock | need | block); a pair present in both directions, or listed in cycles, is two-way."""
    edges, count = [], dict.fromkeys(REASONS, 0)
    for e in dep.get("edges") or []:
        a, b, rs = (e.get("a"), e.get("b"), e.get("reasons", e.get("reason"))) if isinstance(e, dict) else (*e[:2], None)
        edges.append((str(a), str(b)))
        for r in rs if isinstance(rs, list) else [rs]:
            if r in count:
                count[r] += 1
    s = set(edges)
    two = {tuple(sorted(p)) for p in s if p[::-1] in s}
    two |= {tuple(sorted(map(str, c))) for c in dep.get("cycles") or [] if len(c) == 2}
    return edges, sorted(two), count


def layers(ids, edges):
    """Longest-path layer of every dish over (a before b) edges, sources at 0. A cycle the edges still hold is broken
    at its dish with the fewest open predecessors (layout only)."""
    succ = {i: set() for i in ids}
    for a, b in edges:
        if a in succ and b in succ and a != b:
            succ[a].add(b)
    indeg = dict.fromkeys(ids, 0)
    for a in ids:
        for b in succ[a]:
            indeg[b] += 1
    wave, left, todo = dict.fromkeys(ids, 0), set(ids), [i for i in ids if indeg[i] == 0]
    while left:
        i = todo.pop() if todo else min(left, key=lambda j: (indeg[j], j))
        if i not in left:
            continue
        left.discard(i)
        for j in succ[i]:
            if j in left:
                wave[j] = max(wave[j], wave[i] + 1)
                indeg[j] -= 1
                if indeg[j] == 0:
                    todo.append(j)
    return wave


def _plural(n, word):
    return f"{n} {word}{'' if n == 1 else 's'}"


def context_drift(dep):
    """(drops whose context drifted more than DRIFT_MM before the drop, drops, largest drift [mm]) from
    tests[].context_drift_mm; None without tests."""
    v = [float(t["context_drift_mm"]) for t in dep.get("tests") or []
         if isinstance(t, dict) and isinstance(t.get("context_drift_mm"), (int, float))]
    return (sum(x > DRIFT_MM for x in v), len(v), max(v)) if v else None


def drift_tail(dep):
    """(tests before the context stayed displaced, the smallest drift from there on [mm], drops in that tail with only
    cups out of the load) when the drift stays above DRIFT_MM to the last test and that tail holds most of the load's
    drifted drops; None otherwise."""
    tests = [t for t in dep.get("tests") or [] if isinstance(t, dict) and isinstance(t.get("context_drift_mm"), (int, float))]
    v = [float(t["context_drift_mm"]) for t in tests]
    s = len(v)
    while s > 0 and v[s - 1] > DRIFT_MM:
        s -= 1
    if len(v) - s < 3 or 2 * (len(v) - s) <= sum(x > DRIFT_MM for x in v):
        return None
    kind = {e.get("id"): _kind_of(e.get("id"), e.get("kind")) for e in dep.get("entries") or []}
    cups = sum(all(kind.get(i, _kind_of(i)) == "cup" for i in [t.get("dish"), *(t.get("removed") or [])])
               for t in tests[s:])
    return s, min(v[s:]), cups


def resettle_failure(dep):
    """(resettle builds tried, the dish the last one stopped at, how) of a goal that did not re-settle."""
    rb = dep.get("resettle_builds") or []
    last = rb[-1] if rb else {}
    n = len(rb) or int((dep.get("kit") or {}).get("resettle_builds_run") or 0)
    dish = dep.get("failed_dish") or last.get("dish") or "?"
    mv = last.get("moved")
    how = (f"placing {dish} moved {', '.join(map(str, mv))}" if last.get("status") == "moved" and mv
           else f"{dish} did not land" if last.get("status") == "landing" else f"{last.get('status') or '?'} at {dish}")
    return n, dish, how


def goal_moved(dep):
    """(largest lateral or height move [mm], largest tilt [deg]) of a re-settled goal, from goal_delta's max (or the
    largest per-dish entry when it has none); None without goal_delta."""
    gd = dep.get("goal_delta")
    if not isinstance(gd, dict):
        return None
    mx = gd.get("max")
    if not isinstance(mx, dict):
        rows = [v for k, v in (gd.get("per_dish") or gd).items() if isinstance(v, dict) and k != "max"]
        mx = {k: max(abs(float(r.get(k) or 0.0)) for r in rows) for k in ("lateral_mm", "dz_mm", "tilt_deg")} if rows else {}
    num = {k: abs(float(mx[k])) for k in ("lateral_mm", "dz_mm", "tilt_deg") if isinstance(mx.get(k), (int, float))}
    mm = [num[k] for k in ("lateral_mm", "dz_mm") if k in num]
    return (max(mm) if mm else None), num.get("tilt_deg")


def _num(v, unit):
    """12 -> '12 mm', 3.4 -> '3.4 mm', None -> '? mm'."""
    return f"? {unit}" if v is None else f"{v:.0f} {unit}" if v >= 10 else f"{v:.1f} {unit}"


def dep_summary(dep, scores=None):
    """Counts and words shared by the depgraph figure and the page captions (ordered-pair records)."""
    status = str(dep.get("status") or "?")
    ok, rejected, cycle = status == "ok", status == "gate", status == "cycle"
    unsettled = status == "resettle_failed"                # a re-settled goal the sequential build did not reach
    edges, two, count = dep_edges(dep)
    builds = dep.get("builds") or []
    last = builds[-1] if builds else {}
    nb = len(builds) or int((dep.get("kit") or {}).get("builds_run") or 0)
    failed = None if ok or rejected or cycle or unsettled else (dep.get("failed_dish") or last.get("dish"))
    extra = [tuple(map(str, x[:2])) for x in dep.get("extra_constraints") or []]
    gate = dep.get("gate") if isinstance(dep.get("gate"), dict) else {}
    why = str(gate.get("reason") or str(gate.get("outcome") or "").replace("_", " ") or dep.get("reason") or "?")
    why = why[:1].lower() + why[1:] if why[1:2].islower() else why
    ids = [str(e.get("id", e.get("object_id"))) for e in dep.get("entries") or []]
    jw = dep.get("waves") if isinstance(dep.get("waves"), dict) else {}
    tw = set(two)
    if jw and all(i in jw for i in ids):                   # the JSON's waves (an acyclic graph)
        waves = {i: int(jw[i]) for i in ids}
    else:                                                  # a cyclic graph: the waves without its two-way pairs
        waves = layers(ids, [p for p in edges if tuple(sorted(p)) not in tw])
    nw = max(waves.values()) + 1 if waves and not (rejected or unsettled) else None
    nc = max(len(two), len(dep.get("cycles") or []))
    nr = len(dep.get("resettle_builds") or [])
    if rejected:
        text = f"the joint gate rejected the load: {why}"
    elif unsettled:
        text = "the sequential build did not re-settle the goal" + (f" in {_plural(nr, 'build')}" if nr else "")
    elif cycle:
        text = ("two-way pair" + ("s " if len(two) > 1 else " ") + ", ".join(f"{a} ⇄ {b}" for a, b in two) if two
                else "a cycle") + ": no valid one-at-a-time order"
    elif ok:
        text = "acyclic, and the build in its order lands every dish"
    else:
        mv = last.get("moved")
        how = (f"placing {failed} moved {', '.join(map(str, mv))}" if last.get("status") == "moved" and mv
               else f"{failed or '?'} did not land")
        text = f"acyclic, but no order built in {_plural(nb, 'build')}: {how}"
        if "cycle" in str(dep.get("reason") or ""):        # the build's learned constraints closed a cycle
            text += "; the learned constraints close a cycle"
    if rejected or unsettled:
        counts = "no pairs tested, no edges, no build"
    else:
        wv = "–" if nw is None else str(nw) + (" without the two-way pair" + ("s" if len(two) != 1 else "")
                                              if two else " with the cycle broken for the layout" if cycle else "")
        bl = ("no build" if cycle else f"built in {_plural(nb, 'build')}, {_plural(len(extra), 'extra constraint')}"
              if ok else f"no order in {_plural(nb, 'build')}, {_plural(len(extra), 'extra constraint')} learned")
        counts = (f"pairs tested {len(dep.get('pairs_tested') or [])}; edges {len(edges)} (knock {count['knock']}, "
                  f"need {count['need']}, block {count['block']}); cycles {nc}; waves {wv}; {bl}")
    S = {}
    for side in ("packed", "settled", "built"):
        rec = (scores or {}).get(side)
        S[side] = rule_dict(rec.get("S")) if isinstance(rec, dict) else None
    built = (scores or {}).get("built")
    mm = {k: float(gate[k]) * 1e3 for k in ("maximum_settle_penetration_m", "maximum_cycle_penetration_m",
                                             "peak_penetration_limit_m") if isinstance(gate.get(k), (int, float))}
    peaks = [mm[k] for k in ("maximum_settle_penetration_m", "maximum_cycle_penetration_m") if k in mm]
    gs = gate.get("wall_seconds")
    return {"status": status, "ok": ok, "rejected": rejected, "cycle": cycle, "builds": nb, "failed": failed,
            "extra": extra, "text": text, "counts": counts, "edges": edges, "two_way": two, "reasons": count,
            "n_edges": len(edges), "n_pairs": len(dep.get("pairs_tested") or []), "n_cycles": nc, "waves": waves,
            "n_waves": nw, "S": S, "pooling_built": built.get("pooling") if isinstance(built, dict) else None,
            "why": why, "outcome": str(gate.get("outcome") or "").replace("_", " ") or None,
            "peak_mm": max(peaks) if peaks else None, "limit_mm": mm.get("peak_penetration_limit_m"),
            "gate_s": float(gs) if isinstance(gs, (int, float)) else None, "unsettled": unsettled,
            "goal_moved": goal_moved(dep), "resettle_builds": nr}


def _short(oid):
    """plate_01 -> p01 (the top-view labels)."""
    return re.sub(r"^([a-z])[a-z]*_?", r"\1", str(oid))


def _arc_mid(ax, p, q, rad):
    """Data point at the middle of an arc3 connection from p to q (matplotlib bends it in display space)."""
    P, Q = ax.transData.transform([p, q])
    return ax.transData.inverted().transform((P + Q) / 2 + 0.5 * rad * np.array([Q[1] - P[1], P[0] - Q[0]]))


def _legend_arrow(style):
    """Legend handler drawing an arrow (style '-|>' or '<|-|>') in place of a line."""
    from matplotlib.legend_handler import HandlerPatch
    from matplotlib.patches import FancyArrowPatch

    def make(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
        y = height / 2 - ydescent
        return FancyArrowPatch((-xdescent, y), (width - xdescent, y), arrowstyle=style, mutation_scale=12)
    return HandlerPatch(patch_func=make)


def fig_depgraph(dep, scores, kpts, out, resettled=False):
    """Left: the layered graph of the ordered-pair test (columns = waves, over the one-way edges in a cyclic load; one
    row per kind when there is one wave or none). Right: top views of both racks with the same edges, slightly curved
    so arrows along a plate bank stay apart. One edge style, arrow to the later dish; a two-way pair is one
    double-headed edge marked two-way. resettled: a goal re-settled by a sequential build (--resettled-dir)."""
    plt = _mpl()
    import matplotlib.patheffects as pe
    from matplotlib.lines import Line2D
    from matplotlib.patches import FancyArrowPatch, Patch
    sm = dep_summary(dep, scores)
    rej = sm["rejected"]
    if sm["ok"] and dep.get("built_entries"):
        ents, posed = dep["built_entries"], "built"
    elif dep.get("entries_settled") and not rej:            # the targets: the jointly settled poses
        ents, posed = dep["entries_settled"], "re-settled goal" if resettled else "jointly settled"
    else:
        ents, posed = dep.get("entries") or [], "packed"
    objs = load_objects({"entries": ents}) if ents else []
    by = {o["object_id"]: o for o in objs}
    ids = list(by)
    two = [] if rej else [p for p in sm["two_way"] if p[0] in by and p[1] in by]
    one = [] if rej else sorted({p for p in sm["edges"] if p[0] in by and p[1] in by and tuple(sorted(p)) not in two})
    col = {i: int(sm["waves"].get(i, 0)) for i in ids}
    ncol = max(col.values(), default=0) + 1
    rows = ncol == 1                       # one wave or none: a row per kind, not one 24-high column
    rack_rank, kind_rank = {"LowerRack": 0, "UpperRack": 1}, {"plate": 0, "bowl": 1, "cup": 2}
    y, lone = {}, []
    if rows:
        kinds = sorted({by[i]["kind"] for i in ids}, key=lambda kd: (kind_rank.get(kd, 3), kd))
        for r, kd in enumerate(kinds):
            for k, i in enumerate(sorted(j for j in ids if by[j]["kind"] == kd)):
                col[i], y[i] = k, 1.0 - (r + 0.5) / len(kinds)
        ncol = max(col.values(), default=0) + 1
    else:
        pred, succ, mate = {i: set() for i in ids}, {i: set() for i in ids}, {i: set() for i in ids}
        for a, b in one + two + [p[::-1] for p in two]:
            pred[b].add(a)
            succ[a].add(b)
        for a, b in two:
            mate[a].add(b)
            mate[b].add(a)
        lone = [i for i in ids if not (succ[i] or pred[i])]
        for i in lone:                     # dishes no edge orders: a column of their own, left of wave 0
            col[i] = -1

        def base(i):
            return rack_rank.get(by[i]["rack"], 2), kind_rank.get(by[i]["kind"], 3), i

        def place(c, nbrs=lambda i: ()):
            """Evenly spaced rows. A dish takes the free row nearest its neighbours' mean row in other columns (a
            two-way partner without one, its partner's); the dishes with none fill the bottom rows first, two-way
            partners together."""
            mem = [i for i in ids if col[i] == c]
            free = [1.0 - (k + 0.5) / len(mem) for k in range(len(mem))]
            want = {i: [y[j] for j in nbrs(i) if j in y and col[j] != c] for i in mem}
            want = {i: float(np.mean(v)) for i, v in want.items() if v}
            for i in mem:
                t = [want[j] for j in sorted(mate[i]) if j in want]
                if i not in want and t:
                    want[i] = t[0]
            rest = sorted((i for i in mem if i not in want), key=lambda i: (min(map(base, [i, *mate[i]])), base(i)))
            for i, v in zip(rest, free[len(free) - len(rest):]):
                y[i] = v
            free = free[:len(free) - len(rest)]
            for i in sorted(want, key=lambda i: (-want[i], base(i))):
                y[i] = min(free, key=lambda v: abs(v - want[i]))
                free.remove(y[i])

        place(-1)
        place(0)
        for sweep in range(2):             # barycentre sweeps: each dish near its earlier predecessors, then the
            for c in range(1, ncol):       # sources near their successors (fewer crossings)
                place(c, lambda i: [j for j in pred[i] if col[j] < col[i]])
            if sweep == 0:
                place(0, lambda i: succ[i])
    fig = plt.figure(figsize=(24, 9.8))
    gs = fig.add_gridspec(1, 3, width_ratios=[2.25, 1, 1], left=0.025, right=0.99, top=0.87, bottom=0.12, wspace=0.1)
    ax = fig.add_subplot(gs[0])
    ax.set_xlim((-1.6 if lone else -0.6), ncol - 0.4)
    ax.set_ylim(-0.05, 1.02)
    halo = [pe.withStroke(linewidth=2.2, foreground="white")]
    for a, b in one:
        dx = col[b] - col[a]
        rad = 0.2 if rows else 0.0 if dx == 1 else (0.35 if dx == 0 else 0.12)
        ax.add_patch(FancyArrowPatch((col[a], y[a]), (col[b], y[b]), arrowstyle="-|>", mutation_scale=11,
                                     color=EDGE_DARK, linewidth=1.3, shrinkA=8, shrinkB=8,
                                     connectionstyle=f"arc3,rad={rad}", zorder=3))
    paired = set()                         # dishes of a two-way pair in one column: their labels go to the left
    for a, b in two:                       # one double-headed edge; in one column it runs north-south between the two
        p, q = (col[a], y[a]), (col[b], y[b])   # dishes, so its heads never sit where one-way edges leave or arrive
        same = col[a] == col[b] and not rows
        if same:
            paired |= {a, b}
            between = any(col[i] == col[a] and min(y[a], y[b]) < y[i] < max(y[a], y[b]) for i in ids)
            rad = (-0.3 if y[a] > y[b] else 0.3) if between else 0.0      # around a dish in between, bulging right
        else:
            rad = 0.2 if rows else 0.0
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle="<|-|>", mutation_scale=11, color=EDGE_DARK, linewidth=1.6,
                                     shrinkA=8, shrinkB=8, connectionstyle=f"arc3,rad={rad}", zorder=4))
        ax.annotate("two-way", _arc_mid(ax, p, q, rad), xytext=(6, 0) if same else (0, 5), textcoords="offset points",
                    ha="left" if same else "center", va="center" if same else "bottom", fontsize=9,
                    fontweight="bold", color=EDGE_DARK, zorder=8, path_effects=halo)
    crowd = {c: sum(1 for i in ids if col[i] == c) > 12 for c in set(col.values())}   # crowded: labels to the left
    for i in ids:
        bad, side = i == sm["failed"], crowd[col[i]] or i in paired
        ax.scatter([col[i]], [y[i]], s=200, c=[tint(by[i])], edgecolors=FAIL_RED if bad else "#333333",
                   linewidths=2.8 if bad else 0.8, zorder=6)
        ax.annotate(i, (col[i], y[i]), xytext=(-9, 0) if side else (0, -10), textcoords="offset points",
                    ha="right" if side else "center", va="center" if side else "top", fontsize=7.5, zorder=7,
                    path_effects=halo)
    ticks = [] if rows else ([-1] if lone else []) + list(range(ncol))
    ax.set_xticks(ticks, ["no edge" if t < 0 else str(t) for t in ticks])
    ax.set_yticks([])
    for side in ("left", "right", "top"):
        ax.spines[side].set_visible(False)
    if rows:
        xl = ("no pair tests, no build" if rej else "one wave once the two-way pair" + ("s are" if len(two) > 1 else
              " is") + " removed" if two else "one wave: no edges") + "; one row per kind"
    else:
        xl = ("wave: the longest path from a source" + (", the two-way pair" + ("s" if len(two) > 1 else "")
                                                         + " removed" if two else "")
              + "; a dish can go in once every earlier wave is in" + ("; no edge: nothing orders the dish" if lone
                                                                       else ""))
    ax.set_xlabel(xl, fontsize=9.5)
    ax.set_title("no dependency graph: the joint gate rejected the load" if rej else
                 "dependency graph from the ordered-pair test, arrows to the later dish", fontsize=10.5, loc="left")
    shapes = [(o, kpts[o["kind"]] @ o["Rm"].T + o["t"]) for o in objs if o["kind"] in kpts]
    for k, rack in enumerate(("LowerRack", "UpperRack")):
        axr = fig.add_subplot(gs[k + 1])
        mine = sorted([s for s in shapes if s[0]["rack"] == rack], key=lambda s: s[1][:, 2].mean())
        cen = {}
        for o, P in mine:
            xy = hull2(P[:, :2])
            bad = o["object_id"] == sm["failed"]
            axr.fill(xy[:, 0], xy[:, 1], facecolor=tint(o), edgecolor=FAIL_RED if bad else "#333333",
                     lw=2.2 if bad else 0.5, alpha=0.85, zorder=1)
            cen[o["object_id"]] = xy.mean(axis=0)
        (x0, x1), (y0, y1), _ = BOXES[rack]
        _box(axr, (x0, y0), (x1, y1), color="#777777", ls="--", lw=0.9)
        for (a, b), style in [(p, "-|>") for p in one] + [(p, "<|-|>") for p in two]:
            if a in cen and b in cen:
                axr.add_patch(FancyArrowPatch(tuple(cen[a]), tuple(cen[b]), arrowstyle=style, mutation_scale=9,
                                              color=EDGE_DARK, linewidth=1.1 if style == "-|>" else 1.4, shrinkA=4,
                                              shrinkB=4, connectionstyle="arc3,rad=0.25", zorder=10))
        for i, p in cen.items():
            axr.text(p[0], p[1], _short(i), fontsize=6, ha="center", va="center", zorder=9,
                     path_effects=[pe.withStroke(linewidth=1.8, foreground="white")])
        axr.set_xlim(-0.31, 0.31)
        axr.set_ylim(-0.31, 0.31)
        axr.set_aspect("equal")
        axr.set_xlabel("x [m]", fontsize=8.5, labelpad=1)
        axr.set_ylabel("y [m]", fontsize=8.5, labelpad=1)
        axr.tick_params(labelsize=7.5)
        axr.set_title(f"top, {'lower' if k == 0 else 'upper'} rack: {len(mine)} dishes at their {posed} poses",
                      fontsize=10)
    handles, hmap = [], {}
    for style, lab, show in (("-|>", "A before B: only that order lands both dishes (reason in the JSON)", not rej),
                             ("<|-|>", "two-way pair: no valid one-at-a-time order", bool(two))):
        if show:
            h = FancyArrowPatch((0, 0), (1, 0), color=EDGE_DARK, linewidth=1.5, label=lab)
            handles.append(h)
            hmap[h] = _legend_arrow(style)
    handles += [Patch(facecolor=plt.get_cmap(cm)(0.6), edgecolor="#333333", label=f"{kd} ({kd[0]})")
                for kd, cm in KIND_CMAP.items()]
    if sm["failed"]:
        handles.append(Line2D([], [], ls="none", marker="o", ms=11, markerfacecolor="white", markeredgecolor=FAIL_RED,
                              markeredgewidth=2.6, label=f"the failing dish, {sm['failed']}"))
    fig.legend(handles=handles, handler_map=hmap, loc="lower center", ncol=len(handles), frameon=False, fontsize=9.5)
    sides = [s for s in ("packed", "settled", "built") if (sm["S"][s] or {}).get("NJ") is not None]
    lab = {"settled": "re-settled"} if resettled else {}
    sS = ("; S under New+J " + " → ".join(f"{lab.get(s, s)} {sm['S'][s]['NJ']:.3f}" for s in sides)) if sides else ""
    dr = context_drift(dep)
    line2 = sm["counts"] + (f"; drift > {DRIFT_MM:g} mm in {dr[0]} of {dr[1]} drops" if dr else "")
    if rej and sm["peak_mm"] is not None:
        line2 += (f"; deepest momentary contact {sm['peak_mm']:.2f} mm"
                  + (f" (limit {sm['limit_mm']:.0f} mm)" if sm["limit_mm"] is not None else ""))
    head = f"{dep.get('load_id', '?')}: {sm['text']}"
    if resettled:
        L, T = sm["goal_moved"] or (None, None)
        head = f"{dep.get('load_id', '?')}, goal re-settled by a sequential build: {sm['text']}"
        line2 = (f"goal moved at most {_num(L, 'mm')} / {_num(T, 'deg')}; "
                 f"{_plural(sm['resettle_builds'], 'resettle build')}; " + line2)
    fig.suptitle(f"{head}\n{line2}{sS}", fontsize=13, x=0.025, ha="left")
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


# ------------------------------------------------------------------ driver
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid-dir", default=None)
    ap.add_argument("--grid-obj-dir", default=None,
                    help="object-level grid (<dish>_obj entities, cup): draws objlevel_slices.png, and the per-dot "
                         "figures use its perdot_<dish>.npz instead of --grid-dir's")
    ap.add_argument("--sweep", default=None)
    ap.add_argument("--loads-dir", default=None)
    ap.add_argument("--farm-dir", default="data/results/random_loads/rl_20261009")
    ap.add_argument("--dependency-dir", default=None,
                    help="Isaac ordered-pair dependency graphs (<id>.json, <id>.scores.json; "
                         "data/results/dot_rules/dependency/pairs): one depgraph_<id>.png per load")
    ap.add_argument("--resettled-dir", default=None,
                    help="goals re-settled by a sequential build (<id>.json, goal_source sequential_build): one "
                         "depgraph_<id>_resettled.png per record, none for status resettle_failed")
    ap.add_argument("--assets", default=str(DEFAULT_ASSETS))
    ap.add_argument("--out", default="data/media/dot_rules/figures")
    ap.add_argument("--only", default="perdot,convergence,loads,spread,record,objlevel,depgraph",
                    help="comma list of figure groups to draw")
    a = ap.parse_args()
    only = set(a.only.split(","))
    out = resolve(a.out)
    out.mkdir(parents=True, exist_ok=True)
    kpts = kind_points(a.assets)
    gsrc = a.grid_obj_dir or a.grid_dir
    if gsrc and "perdot" in only:
        pdir = level_dir(gsrc)
        grid = load_grid(pdir)
        for dish in ("plate", "bowl", "cup"):
            if (pdir / f"perdot_{dish}.npz").is_file():
                fig_perdot(load_perdot(pdir, dish), dish, kpts[dish], grid["n_points"], out / f"dots_{dish}.png")
    if a.grid_obj_dir and "objlevel" in only:
        fig_objlevel(load_grid(level_dir(a.grid_obj_dir)), out / "objlevel_slices.png")
    if a.sweep and "convergence" in only:
        fig_convergence(load_sweep(a.sweep), out / "convergence.png")
    if a.loads_dir and only & {"loads", "spread", "record"}:
        farm = load_farm(a.farm_dir)
        recs = attach_objects(load_records(a.loads_dir, farm))
        if not recs:
            log(f"no scored loads in {a.loads_dir}: load figures skipped")
            only -= {"loads", "spread", "record"}
        summ = summarize(recs)
        boxes = BOXES
        pb = recs[0].get("params", {}).get("boxes") if recs else None
        if isinstance(pb, dict) and all(k in pb for k in BODY):
            boxes = {k: tuple(tuple(map(float, ax)) for ax in _box_axes(pb[k])) for k in BODY}
        npts = (recs[0]["N"] or 0) if recs else 0
        theta = float(recs[0]["theta"] or 30) if recs else 30.0
        if "loads" in only:
            for r in RULES3:
                ranked = sorted(recs, key=lambda x: -x["S"][r])
                fig_loads(ranked[:5], r, True, len(recs), kpts, boxes, npts, theta, out / f"rand_top5_{SLUG[r]}.png")
                fig_loads(ranked[::-1][:5], r, False, len(recs), kpts, boxes, npts, theta,
                          out / f"rand_bottom5_{SLUG[r]}.png")
        if "spread" in only:
            fig_spread(recs, summ, out / "rand_spread.png")
        if "record" in only:
            fig_record(recs, out / "rand_record.png")
    if a.dependency_dir and "depgraph" in only:
        deps = load_dependency(a.dependency_dir)
        if not deps:
            log(f"no dependency results in {a.dependency_dir}: depgraph figures skipped")
        for x in deps:
            fig_depgraph(x["dep"], x["scores"], kpts, out / f"depgraph_{x['dep']['load_id']}.png")
    if a.resettled_dir and "depgraph" in only:
        for x in load_dependency(a.resettled_dir):
            lid = x["dep"]["load_id"]
            if x["dep"].get("status") == "resettle_failed":
                log(f"{lid}: the goal did not re-settle, no figure")
                continue
            fig_depgraph(x["dep"], x["scores"], kpts, out / f"depgraph_{lid}_resettled.png", resettled=True)
    print(f"[RESULT] PASS figures in {out}", flush=True)


def _box_axes(b):
    """((x0,x1),(y0,y1),(z0,z1)) from {lower:[..], upper:[..]} or [[x0,y0,z0],[x1,y1,z1]] or [[x0,x1],...]."""
    if isinstance(b, dict):
        lo, hi = b.get("lower", b.get("min")), b.get("upper", b.get("max"))
        return tuple((lo[i], hi[i]) for i in range(3))
    b = np.asarray(b, float)
    return tuple(map(tuple, b.T)) if b.shape == (2, 3) else tuple(map(tuple, b))


def run_main(main, name):
    """Run a script's main so that the last stdout line is [RESULT] PASS (main's own) or [RESULT] FAIL."""
    import traceback
    try:
        main()
    except SystemExit as e:
        if e.code in (None, 0):
            raise
        if not isinstance(e.code, int):
            print(e.code, file=sys.stderr, flush=True)
        print(f"[RESULT] FAIL {name}: {e.code}", flush=True)
        sys.exit(e.code if isinstance(e.code, int) else 1)
    except Exception as e:                                  # report it, then exit non-zero
        traceback.print_exc()
        print(f"[RESULT] FAIL {name}: {type(e).__name__}: {e}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    run_main(main, "figures")
