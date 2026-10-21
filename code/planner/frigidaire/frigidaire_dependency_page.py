#!/usr/bin/env python3
"""The "Dependency graphs" page of the re-posing sampler's samples (plans/2026-10-15-reposing-sampler-five-samples.md; the
lean-side plate order, plans/2026-10-16-lean-side-plate-order.md; backtracking and the learning loop,
plans/2026-10-16-backtracking-sampler-learning-loop.md): one HTML file with every sample's records embedded as JSON, the
clips under media/ (Kit-free, standard library only; runs anywhere). A built sample whose library verdict is invalid is
shown too, labelled with the orders that broke its graph; the panel on why the sampler stopped shows whenever packings did
not build (--failure-note: that round's explanation; the plate chains come from the witness). Validity: every obeying
library order built all 24 and every violation failed somewhere (the library table shows the dish it failed at beside the
predicted one); an obeying order that failed taught a learned edge (drawn dotted) and the next round's fresh orders
re-checked the graph, so a sample is valid when its last round passed.

Per valid sample, behind a sample selector, the tuple: the load (a both-racks still of the built goal and its S), the
graph (the swap-tested edges along the witness order with their reasons; a conditional edge, b needs one of {a, C...},
drawn as a fan from every dish that lets b go in) and the witness animation; under it the per-edge clips (and each
conditional edge's confirmation clips), an order checker and the library table judged by the refined graph. The yield of
the run (every packing attempted and how it ended) comes from the sampler's state file. The height-sweep panels of the
2026-10-15 insertion run are gone (its run note keeps them).

    python3 code/planner/frigidaire/frigidaire_dependency_page.py --records r07_00012=data/results/dot_rules/dependency/samples/r07_00012.json ... --clips r07_00012=data/media/dot_rules/dependency_samples/r07_00012 ... --state data/logs/dot_rules/samples/scheduler_state.json --out data/media/dot_rules/dependency_page_samples

Writes <out>/index.html and <out>/media/<key>_<file> (the clips, posters and load still named in each <clips>/clips.json);
a <load_id>.scores.json beside a record (frigidaire_dependency_score.py) adds its S; prints [RESULT].
"""
import argparse
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frigidaire_dependency_kit import judge_order  # noqa: E402  (stdlib only at import)

MAX_FILE = 15 * 2 ** 20


def short(i):
    kind, n = i.rsplit("_", 1)
    return f"{kind[0]}{n}"


def keyed(items, what):
    """{key: Path} from KEY=PATH arguments, in the given order."""
    out = {}
    for item in items or []:
        key, sep, path = item.partition("=")
        if not sep or not key:
            raise SystemExit(f"{what}: expected KEY=PATH, got {item!r}")
        out[key] = Path(path)
    return out


def scores_of(record_path, load_id):
    """S per rule (O, N, NJ) of the packing's packed poses and of the built goal (the record's entries_settled) when
    frigidaire_dependency_score.py wrote <load_id>.scores.json beside the record, else None; with the rescaled S
    ("rel": S_rel_best of frigidaire_dot_rules_relative.py --reference best, 2026-10-16) when <load_id>.relative.json
    beside it has one; with the kind-ceiling score ("kind": S, S_area, S_by_kind, n_pooling of --reference kind,
    2026-10-21) and the ceilings when it has those."""
    path = record_path.parent / f"{load_id}.scores.json"
    if not path.is_file():
        return None
    d = json.loads(path.read_text())
    pick = lambda r: {k: round(r["S"][k], 4) for k in ("O", "N", "NJ") if k in r["S"]} if r else None   # noqa: E731
    out = {"packed": pick(d.get("packed")), "built": pick(d.get("settled")), "N": d.get("N")}
    rel = record_path.parent / f"{load_id}.relative.json"
    rr = json.loads(rel.read_text()) if rel.is_file() else {}
    states = rr.get("states", {})
    for key, state in (("packed", "packed"), ("built", "settled")):
        if out[key] and "S_rel_best" in (states.get(state) or {}):
            out[key]["rel"] = round(states[state]["S_rel_best"], 4)
        if out[key] and "S" in (states.get(state) or {}):
            out[key]["kind"] = {k: states[state][k] for k in ("S", "S_area", "S_by_kind", "n_pooling")}
    out["ceilings"] = rr.get("ceilings_kind")
    return out


def media(clips, prefix, name):
    return f"media/{prefix}{name}" if name else None


def page_data(rec, clips, prefix, scores):
    wit = rec["witness"]
    pos = {i: n for n, i in enumerate(wit["order"])}
    clips = clips or {}
    E = {(e["a"], e["b"]): e for e in rec["edges"]}
    confirmed = {}
    for c in (rec.get("refinement") or {}).get("confirmations") or []:
        if c["confirmed"]:
            k = f"{c['edge'][0]}>{c['edge'][1]}|{c['c']}"
            cc = clips.get("conditional", {}).get(k)
            confirmed.setdefault(tuple(c["edge"]), []).append(
                {"c": c["c"], "clip": media(clips, prefix, cc and cc["file"]), "poster": media(clips, prefix, cc and cc["poster"])})
    edges = []
    for e in rec["edges"]:
        c = clips.get("edges", {}).get(f"{e['a']}>{e['b']}")
        edges.append({k: e[k] for k in ("a", "b", "reason", "step", "fails", "support", "learned", "detail",
                                          "placed_before", "affected", "swap")}
                     | {"margin": e["margin"]["min"], "any_of": e.get("any_of"), "fan": confirmed.get((e["a"], e["b"]), []),
                        "round": e.get("round"), "from_order": e.get("from_order"),
                        "clip": media(clips, prefix, c and c["file"]), "poster": media(clips, prefix, c and c["poster"])})
    taught = {x["order"]: x for r in (rec.get("learning") or {}).get("rounds") or [] for x in r["diagnoses"]}
    lib = []
    for x in rec["library"]:
        pr, ok, _ = judge_order(x, E)
        o = x.get("outcome")
        t = taught.get(x["index"])
        lib.append({k: x[k] for k in ("index", "kind", "order", "edge")}
                   | {"predicted": pr["predicted"], "predicted_position": pr["predicted_position"],
                      "first_predicted": x.get("predicted"), "as_predicted": ok, "round": x.get("round") or 0,
                      "taught": t and t["edge"], "taught_why": t and (t["why"] or "; ".join(f"{c[0]} > {c[1]}: {c[3]}"
                                                                                             for c in t["rejected"])),
                      "outcome": None if o is None else {k: o.get(k) for k in ("status", "failed_dish", "failed_position",
                                                                                "placed", "moved")}})
    learn = rec.get("learning") or {}
    learning = {"max": learn.get("max_rounds"), "edges": learn.get("edges_added", 0),
                "rounds": [{"round": r["round"], "from": r["from_orders"], "edges": r["edges"], "orders": len(r["orders"])}
                           for r in learn.get("rounds") or []], "unlearnable": learn.get("unlearnable")}
    s = rec.get("reposing") or {}
    tries = s.get("attempts") or []
    backs = s.get("backtracks") or []
    ents = {e["id"]: e for e in rec.get("entries_chosen") or rec["entries"]}
    steps = wit.get("steps") or []
    w = clips.get("witness")
    ref = rec.get("refinement")
    swaps = rec.get("swaps") or []
    return {"key": rec["load_id"], "load": rec["load_id"], "generated": rec.get("generated_utc", "")[:10], "valid": rec.get("valid"),
            "release": rec.get("release") or {"mode": "clear"},
            "dishes": sorted(({"id": i, "kind": e["kind"], "rack": e["rack"], "short": short(i)} for i, e in ents.items()),
                             key=lambda d: pos.get(d["id"], 99)),
            "witness": {"order": wit["order"], "support": wit["support"], "learned": wit["learned"],
                        "plate_order": wit.get("plate_order") or "random", "chains": wit.get("chains"),
                        "clip": media(clips, prefix, w and w["file"]), "poster": media(clips, prefix, w and w["poster"]),
                        "seconds": w["seconds"] if w else None,
                        "lowest_mm": [s_.get("lowest_lift_mm") for s_ in steps],
                        "dev_mm": max((s_.get("settle_dev_mm") or 0) for s_ in steps) if steps else None},
            "still": media(clips, prefix, clips.get("load")), "score": scores,
            "sampler": {"attempts": len(tries), "failed": sum(t["status"] != "ok" for t in tries),
                        "blocked": sum(bool(t["step"].get("blocked")) for t in tries),
                        "deferred": sorted({t["dish"] for t in tries if t["status"] != "ok"}),
                        "reposed": [{"id": i, "from": f"{r['from_slot']} {r['from_variant']}", "to": f"{r['slot']} {r['variant']}"}
                                    for i, r in sorted((s.get("reposed") or {}).items())],
                        "order": s.get("order"), "backtrack": s.get("backtrack"),
                        "backtracks": [{k: b.get(k) for k in ("dish", "undone", "outcome", "back", "deferred", "moved")}
                                       for b in backs]},
            "goal_delta": (rec.get("goal_delta") or {}).get("max"), "edges": edges,
            "swaps": {"tested": len(swaps), "failed": sum(x["failed"] for x in swaps),
                      "invalid_context": sum(not x["context_valid"] for x in swaps),
                      "near": sum("near" in x["source"] for x in swaps),
                      "learned": sum("learned" in x["source"] for x in swaps)},
            "library": lib, "learning": learning,
            "verdict": {k: v for k, v in (rec.get("verdict") or {}).items() if k not in ("rows", "mismatches")},
            "refinement": None if not ref else {"edges": ref["edges"], "fresh": ref["fresh"], "ok": ref["ok"]},
            "thresholds": rec.get("thresholds", {}).get("at_goal"), "tolerance": tolerance_of(rec),
            "rejudged": None if not rec.get("rejudged") else {
                k: rec["rejudged"].get(k) for k in ("from", "from_tolerance", "edges_before", "edges_after", "dropped",
                                                    "untested_step2", "marginal_unrerun", "learned_dropped",
                                                    "verdict_before")}}


def tolerance_of(rec):
    """{name, text} of the record's at-goal tolerance (plans/2026-10-16-doubled-tolerance-rerun.md; bench before it)."""
    t = rec.get("thresholds") or {}
    tol = "; ".join(f"{k} {v['lateral_mm']:g} mm lateral / {v['dz_mm']:g} mm height / {v['tilt_deg']:g}°"
                    for k, v in (t.get("at_goal") or {}).items())
    return {"name": rec.get("tolerance") or "bench", "text": f"{tol}; contact under {t.get('contact_limit_mm', 2.):g} mm"}


def yield_of(state):
    """Every packing the sampler attempted, from its state file: [{load, stage, reason, orders}]."""
    if not state:
        return []
    rule = state.get("plate_order") or "random"
    return [{"load": k, "stage": v["stage"], "reason": v.get("reason"), "orders": v.get("orders"), "rule": rule,
             "backtracks": v.get("backtracks"), "rounds": v.get("rounds"), "tolerance": state.get("tolerance") or "bench",
             "resumed": k in (state.get("resumed") or [])}
            for k, v in state["packings"].items() if v.get("started") is not None]


def failure_of(folder, screen):
    """Why the sampler stopped, over every record in ``folder`` that did not build: dishes placed per packing, the dishes
    left out by kind and by how their last attempt ended, the re-posing calls that found no candidate, the placed plates'
    faces from vertical and settle deviations (each witness's last frame), and the Kit-free screen per kind."""
    import math
    import statistics
    rows = [json.loads(p.read_text()) for p in sorted(folder.glob("r*.json"))]
    rows = [d for d in rows if d.get("status") == "resettle_failed" and d.get("reposing")]
    if not rows:
        return None
    placed, left, why, calls, faces, devs, backs = [], {}, {}, {}, [], [], {}
    for d in rows:
        tries = d["reposing"]["attempts"]
        build = (d.get("resettle_builds") or [{}])[0].get("order") or []      # the sampler's build: placed + stuck
        ok = set(build[:-1]) if build else {t["dish"] for t in tries if t["status"] == "ok"}   # (a backtrack can undo
        placed.append(len(ok))                                                # a dish that had gone in)
        last = {t["dish"]: t for t in tries if t["dish"] not in ok}
        for i, t in last.items():
            kind, s = i.split("_")[0], t["step"]
            left[kind] = left.get(kind, 0) + 1
            by = sorted({x.split("_")[0] for x in s.get("blockers") or [] if "_" in x})
            k = (f"{kind}: descent blocked by a placed {' or '.join(by) or 'dish'}" + (" at its start" if s.get("blocked") == "start"
                 else " on the way down") if s.get("blocked") else f"{kind}: " + ("knocked a placed dish" if t["status"] == "moved"
                                                                               else "landed out of tolerance"))
            why[k] = why.get(k, 0) + 1
        for c in d["reposing"].get("repose_calls") or []:
            kind = c["dish"].split("_")[0]
            n, z = calls.get(kind, (0, 0))
            calls[kind] = (n + 1, z + (c["found"] == 0))
        devs += [t["step"]["settle_dev_mm"] for t in tries if t["status"] == "ok" and t["dish"].startswith("plate")]
        for b in d["reposing"].get("backtracks") or []:
            backs[b["outcome"]] = backs.get(b["outcome"], 0) + 1
        traj = folder / "_kit" / d["load_id"] / "traj" / "witness.json"
        if traj.is_file():
            w = json.loads(traj.read_text())
            f = w["frames"][-1]
            for n, i in enumerate(w["ids"]):
                if i.startswith("plate") and f[7 * n] <= 2:
                    qx, qy = f[7 * n + 3], f[7 * n + 4]
                    faces.append(math.degrees(math.asin(min(1., abs(1 - 2 * (qx * qx + qy * qy))))))
    out = {"packings": len(rows), "placed": sorted(placed), "left": left,
           "why": sorted(([k, v] for k, v in why.items()), key=lambda x: -x[1]),
           "calls": {k: {"calls": n, "none": z} for k, (n, z) in calls.items()},
           "faces": {"n": len(faces), "upright": sum(x < 15 for x in faces), "lean": sum(15 <= x <= 40 for x in faces),
                     "over": sum(x > 40 for x in faces), "median": round(statistics.median(faces)) if faces else None},
           "plate_dev_mm": [min(devs), round(statistics.median(devs), 1), max(devs)] if devs else None, "screen": None,
           "backtracks": backs}
    if isinstance(screen, dict) and "summary" in screen:      # screen_backtrack.json: the sampler on geometry alone
        out["screen_text"] = screen["summary"]["text"]
    elif screen:
        sc, key = {}, ("chain" if "chain" in screen[0] else "first")     # the lean-side order's screen, else the random one
        for kind in ("plate", "bowl", "cup"):
            a = [x for r in screen for x in r["alone"].values() if x["kind"] == kind]
            f = [r[key][i] for r in screen for i, x in r["alone"].items() if x["kind"] == kind]
            stop = [x["lowest_mm"] for x in a if x["stop"] == "appliance"]
            sc[kind] = {"n": len(a), "start_blocked": sum(bool(x["blocked"]) for x in a), "stopped": len(stop),
                        "stop_mm": [min(stop), statistics.median(stop), max(stop)] if stop else None,
                        "reach": sum(x["stop"] == "target" for x in a),
                        "first_start": sum(x["blocked"] == "start" for x in f), "first_dish": sum(x["blocked"] == "dish" for x in f)}
        out["screen"] = {"packings": len(screen), "kinds": sc, "order": key}
    return out


# --------------------------------------------------------------------------- --edges physical (plans/2026-10-16-physical-cause-graph.md)
# Per valid sample exactly three PNG images and one video (coordinator, 2026-10-16): the load still (S and the gate numbers
# in its caption), the graph along the witness order, the same graph layered as a DAG, and the witness rebuild. One edge
# style; an arrow X -> Y means Y has to be in place before X (it points at the prerequisite: the record's edge a -> b is
# drawn b -> a). The graphs are SVG rasterized by rsvg-convert (2x).

KIND_FILL = {"plate": "#2171b5", "bowl": "#d94801", "cup": "#238b45"}
INK, EDGE_INK, BG = "#1b2630", "#3b4856", "#ffffff"
FONT = "Helvetica, Arial, sans-serif"


def _svg(w, h, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" viewBox="0 0 {w:.0f} {h:.0f}">'
            f'<defs><marker id="m" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
            f'<path d="M0,0 L10,5 L0,10 z" fill="{EDGE_INK}"/></marker></defs><rect width="{w:.0f}" height="{h:.0f}" '
            f'fill="{BG}"/>{body}</svg>')


def _node(x, y, i, kind, free, r=11, side=False):
    fill, stroke = ("#ffffff", KIND_FILL[kind]) if free else (KIND_FILL[kind], "#ffffff")
    tx, ty, anchor = (x + r + 4, y + 5, "start") if side else (x, y + r + 16, "middle")
    return (f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{3 if free else 2}"/>'
            f'<text x="{tx:.1f}" y="{ty:.1f}" text-anchor="{anchor}" font-family="{FONT}" font-size="14" '
            f'fill="{INK}" stroke="{BG}" stroke-width="4" paint-order="stroke">{short(i)}</text>')


def svg_arcs(order, kinds, edges, free):
    """Nodes on a line in the witness order (left to right), one arc per prerequisite pair above it, the arrowhead at the
    prerequisite."""
    step, x0 = 52., 40.
    x = {i: x0 + step * n for n, i in enumerate(order)}
    pairs = sorted({(e["a"], e["b"]) for e in edges}, key=lambda p: abs(x[p[1]] - x[p[0]]), reverse=True)
    top = max((min(26 + .42 * abs(x[b] - x[a]), 300) for a, b in pairs), default=40)
    y0 = top + 40
    w, h = 2 * x0 + step * (len(order) - 1), y0 + 60
    body = ""
    body += f'<line x1="{x0 - 20}" x2="{w - x0 + 20}" y1="{y0}" y2="{y0}" stroke="#cdd6df" stroke-width="2"/>'
    for a, b in pairs:                                        # drawn b -> a: b needs a in place first
        xa, xb = x[a], x[b]
        hh = min(26 + .42 * abs(xb - xa), 300)
        body += (f'<path d="M{xb:.1f},{y0 - 13} C{xb:.1f},{y0 - 13 - hh * 1.33:.1f} {xa:.1f},{y0 - 13 - hh * 1.33:.1f} '
                 f'{xa:.1f},{y0 - 13}" fill="none" stroke="{EDGE_INK}" stroke-width="1.8" marker-end="url(#m)"/>')
    for i in order:
        body += _node(x[i], y0, i, kinds[i], i in free)
    return _svg(w, h, body)


def svg_layers(order, kinds, edges, layers, free, place=None):
    """The same graph layered (Sugiyama-like): layer 0 at the bottom holds the dishes with no prerequisite (in the witness
    order, the free ones at its right end after a gap), each other dish sits one layer above its deepest prerequisite, as
    near the mean x of its prerequisites as the spacing allows (ties: the witness order); ``place`` = the edges that set
    those positions (default ``edges``; the variants all use the direct ones, so toggling keeps every node in place)."""
    pos = {i: n for n, i in enumerate(order)}
    pre = {i: [e["a"] for e in (place or edges) if e["b"] == i] for i in order}
    n_layers = max(layers.values()) + 1 if layers else 1
    rows = {k: [i for i in order if layers.get(i, 0) == k and i not in free] for k in range(n_layers)}
    step, x0, row, right = 58., 84., 96., 64.                # left margin for the layer labels, right for the last name
    loose = [i for i in order if i in free]
    base = rows[0] + ([None] if loose else []) + loose
    w = x0 + right + step * max(len(base) - 1, max(len(r) for r in rows.values()) - 1, 1)
    x = {i: x0 + n * step for n, i in enumerate(base) if i is not None}
    for k in range(1, n_layers):                             # upward: near the mean x of the prerequisites
        want = {i: sum(x[p] for p in pre[i]) / len(pre[i]) for i in rows[k]}
        ids = sorted(rows[k], key=lambda i: (want[i], pos[i]))
        xs = []
        for i in ids:
            xs.append(want[i] if not xs else max(want[i], xs[-1] + step))
        shift = sum(v - want[i] for v, i in zip(xs, ids)) / len(ids)
        shift += max(0., (xs[-1] - shift) - (w - right))     # keep the layer inside the drawing
        shift -= max(0., x0 - (xs[0] - shift))
        for v, i in zip(xs, ids):
            x[i] = v - shift
    y = {i: 40 + row * (n_layers - 1 - layers.get(i, 0)) for i in order}
    h = 40 + row * (n_layers - 1) + 44
    body = ""
    for k in range(n_layers):
        yk = 40 + row * (n_layers - 1 - k)
        body += (f'<line x1="{x0 - 24:.1f}" x2="{w - 8:.1f}" y1="{yk:.1f}" y2="{yk:.1f}" stroke="#eef1f4" stroke-width="1"/>'
                 f'<text x="8" y="{yk + 4:.1f}" font-family="{FONT}" font-size="12" fill="#6a7886">layer {k}</text>')
    for a, b in sorted({(e["a"], e["b"]) for e in edges}):  # from b (above) down to its prerequisite a
        span = layers.get(b, 0) - layers.get(a, 0)          # an edge that skips layers bends right, wider per layer
        cx, cy = (x[a] + x[b]) / 2 + 44 * max(span - 1, 0), (y[a] + y[b]) / 2
        dx, dy = x[a] - cx, y[a] - cy                        # (the end tangent points from the control point)
        d = max((dx * dx + dy * dy) ** .5, 1.)
        sdx, sdy = cx - x[b], cy - y[b]
        sd = max((sdx * sdx + sdy * sdy) ** .5, 1.)
        sx, sy, ex, ey = x[b] + 13 * sdx / sd, y[b] + 13 * sdy / sd, x[a] - 14 * dx / d, y[a] - 14 * dy / d
        body += (f'<path d="M{sx:.1f},{sy:.1f} Q{cx:.1f},{cy:.1f} {ex:.1f},{ey:.1f}" fill="none" stroke="{EDGE_INK}" '
                 f'stroke-width="1.8" marker-end="url(#m)"/>')
    for i in order:
        body += _node(x[i], y[i], i, kinds[i], i in free, side=True)
    return _svg(w, h, body)


VARIANTS = (("direct", "Direct: only the measured physical causes"),
            ("reduction", "Reduction (Hasse diagram): the fewest arrows that imply the same order"),
            ("closure", "Closure: every must-before pair"))


def order_variants(order, edges):
    """{direct, reduction, closure} of the record's edges (a before b) as [{a, b}]: the edges as measured, the transitive
    reduction (a -> b kept when no c lies between them) and the transitive closure; the same partial order, so the same
    layers and the same free dishes (user addition, 2026-10-16)."""
    pairs = {(e["a"], e["b"]) for e in edges}
    after = {}

    def down(i):                                             # every dish that must come after i
        if i not in after:
            after[i] = set()
            for a, b in pairs:
                if a == i:
                    after[i] |= {b} | down(b)
        return after[i]
    closure = {(a, b) for a in order for b in down(a)}
    reduction = {(a, b) for a, b in closure if not any(b in down(c) for c in down(a))}
    return {v: [{"a": a, "b": b} for a, b in sorted(x)] for v, x in (("direct", pairs), ("reduction", reduction),
                                                                      ("closure", closure))}


def rasterize(svg, path):
    """SVG text to a PNG at 2x (rsvg-convert on the Mac, librsvg 2.58)."""
    import subprocess
    src = path.with_suffix(".svg")
    src.write_text(svg)
    subprocess.run(["rsvg-convert", "-z", "2", "-o", str(path), str(src)], check=True)
    src.unlink()


def physical_page(a, records, clip_dirs, state):
    """--edges physical: the page (module comment above): the claim, the legend, per valid sample its four items behind a
    selector, every packing of the run and how it ended, how it was measured."""
    import html
    esc = html.escape
    out_media = a.out / "media"
    out_media.mkdir(parents=True, exist_ok=True)
    copied, samples = [], []
    for key, path in records.items():
        rec = json.loads(path.read_text())
        cdir = clip_dirs[key]
        clips = json.loads((cdir / "clips.json").read_text())
        g, gate, cert = rec["graph"], rec.get("gate") or {}, rec.get("certify") or {}
        files = {}
        for role, name in (("load", clips["load"]), ("video", clips["witness"]["file"]),
                           ("poster", clips["witness"]["poster"])):
            shutil.copy2(cdir / name, out_media / f"{key}_{name}")
            copied.append(out_media / f"{key}_{name}")
            files[role] = f"media/{key}_{name}"
        kinds = {e["id"]: e["kind"] for e in rec.get("entries_chosen") or rec["entries"]}
        order, free = rec["witness"]["order"], set(g["free"])
        var = order_variants(order, g["edges"])
        for v, ev in var.items():                            # direct / reduction / closure, each drawn both ways
            for role, svg in (("arcs", svg_arcs(order, kinds, ev, free)),
                              ("layers", svg_layers(order, kinds, ev, g["layers"], free, place=var["direct"]))):
                p = out_media / f"{key}_graph_{role}_{v}.png"
                rasterize(svg, p)
                copied.append(p)
                files[f"{role}_{v}"] = f"media/{p.name}"
        arrows = {v: len(ev) for v, ev in var.items()}
        sc = scores_of(path, rec["load_id"])
        s_text = (f"S (New+J) {sc['built']['NJ']:.4f} for the built poses (New {sc['built']['N']:.4f}, Old "
                  f"{sc['built']['O']:.4f}" + (f"; the packing it came from {sc['packed']['NJ']:.4f}" if sc.get("packed")
                                              else "") + ")" if sc and sc.get("built") else "S not scored")
        if sc and "rel" in (sc.get("built") or {}):          # the rescaled S, absolute in parentheses (2026-10-16)
            b, p = sc["built"], sc.get("packed") or {}
            s_text = (f"S (New+J), rescaled: {b['rel']:.4f} for the built poses (absolute {b['NJ']:.4f}; New "
                      f"{b['N']:.4f}, Old {b['O']:.4f})" + (f"; the packing it came from {p['rel']:.4f} ({p['NJ']:.4f})"
                                                           if "rel" in p else "")
                      + ". Rescaled: each dish's New+J over the most the same dish alone reaches at its spot (its "
                        "height and distance from its arm, the best of the grid's tilts away from the arm's centre), "
                        "that reference floored at 0.05, each dish's ratio clipped at 1, then area-weighted like S")
        if sc and "kind" in (sc.get("built") or {}):         # the kind-ceiling score, absolute after it (2026-10-21)
            b, p, c = sc["built"], sc.get("packed") or {}, sc["ceilings"]
            k, kp = b["kind"], p.get("kind")
            means = ", ".join(f"{n} {k['S_by_kind'][kk]:.4f}" for n, kk in (("plates", "plate"), ("bowls", "bowl"),
                                                                            ("cups", "cup")) if kk in k["S_by_kind"])
            pool = ("no dish pools" if not k["n_pooling"] else
                    f"{k['n_pooling']} dish{'es pool' if k['n_pooling'] > 1 else ' pools'}, scored 0")
            s_text = (f"S {k['S']:.4f} for the built poses (surface reached {k['S_area']:.4f}; {means}; {pool})"
                      + (f" and {kp['S']:.4f} for the packing it came from (surface reached {kp['S_area']:.4f})"
                         if kp else "")
                      + ". S: each dish's object-level New+J over the best its kind reaches alone anywhere under the "
                        f"arm at θ 30° (plate {c['plate']:.3f}, bowl {c['bowl']:.3f}, cup {c['cup']:.3f}: the "
                        "lower arm's, used for the middle arm too), clipped at 1 and 0 for a dish that pools water (its "
                        "interior more than 2 mm below its rim), then the plain mean over the dishes; surface reached "
                        f"weights the same values by food-contact area. Absolute S (New+J) {b['NJ']:.4f} (New "
                        f"{b['N']:.4f}, Old {b['O']:.4f})")
        lower =list(gate.get("order") or []) == ["LowerRack", "UpperRack"]      # the gate records its retraction order
        g_text = (f"Joint gate accepted{' with the lower rack retracted first (upper first: ' + gate['first_try']['outcome'] + ')' if lower and gate.get('first_try') else ''}: "
                  f"all 24 settled together with a peak penetration of {gate.get('settle_peak_mm')} mm and a last-window "
                  f"median of {gate.get('window_median_mm')} mm (limits {gate.get('peak_limit_mm'):g} and "
                  f"{gate.get('median_limit_mm'):g} mm), both racks retracted with a peak of {gate.get('cycle_peak_mm')} mm, "
                  f"every dish contained")
        st = cert.get("steps") or []
        dev = [max((s_.get(k) or 0) for s_ in st) for k in ("lateral_mm", "dz_mm", "tilt_deg")] if st else None
        n = g["counts"]
        samples.append({"key": key, "load": rec["load_id"], "files": files, "s": s_text, "gate": g_text, "arrows": arrows,
                        "n": n, "free": len(free), "video_s": clips["witness"]["seconds"],
                        "dev": dev, "edges": len({(e["a"], e["b"]) for e in g["edges"]}),   # arrows (distinct pairs)
                        "disj": [f"{short(d['b'])} needs one of {', '.join(map(short, d['suffice']))}" if d["suffice"]
                                 else f"{short(d['b'])} needs several of {', '.join(map(short, d['supports']))}"
                                 for d in g.get("disjunctions") or []]})
    gdir = (Path(__file__).resolve().parents[3] / "data/results/dot_rules/dependency" / a.state.parent.name / "_gate"
            if a.state else None)                            # the scheduler names its log folder after --out-dir
    rows = yield_rows(state, gdir)
    att = len(rows)
    nv = sum(r["stage"] == "valid" for r in rows)
    sections = []
    for k, s in enumerate(samples):
        nn = s["n"]
        sections.append(
            f'<section class="sample" data-key="{esc(s["key"])}"{"" if k == 0 else " hidden"}>'
            f'<h2>Sample {esc(s["load"])}</h2>'
            f'<figure><img src="{esc(s["files"]["load"])}" alt="{esc("Load " + s["load"] + " from four angles (oblique, top of the lower rack, top of the upper rack, front), every dish labelled with its id")}" '
            f'width="1280" height="912"><figcaption><b>The load from four angles</b> (oblique; top of the lower rack; top of '
            f'the upper rack; front), every dish labelled with its id (p plate, b bowl, c cup). {esc(s["s"])}. '
            f'{esc(s["gate"])}.</figcaption></figure><div class="two">'
            f'<figure><video controls preload="metadata" playsinline poster="{esc(s["files"]["poster"])}" '
            f'src="{esc(s["files"]["video"])}"></video><figcaption><b>The witness, rebuilt in a fresh Isaac process</b> '
            f'({s["video_s"]:g} s): empty racks, the witness order, the same move; all 24 landed'
            + (f", the furthest {s['dev'][0]:g} mm / {s['dev'][1]:g} mm / {s['dev'][2]:g}° from the built pose" if s["dev"] else "")
            + '.</figcaption></figure></div>'
            f'<div class="variants"><div class="seg" role="group" aria-label="Arrows shown">'
            + "".join(f'<button type="button" data-v="{v}" aria-pressed="{str(v == "direct").lower()}">{v}</button>'
                      for v, _ in VARIANTS)
            + '</div><p class="muted">' + "".join(f'<span data-variant="{v}"{"" if v == "direct" else " hidden"}><b>Shown: '
                                                  f'{esc(t)}</b> ({s["arrows"][v]} arrows).</span>' for v, t in VARIANTS)
            + ' All three carry the same ordering information: the same pairs must go in before one another, only the '
              'number of arrows drawn differs.</p></div>'
            f'<figure><div class="graphbox">'
            + "".join(f'<img data-variant="{v}" src="{esc(s["files"]["arcs_" + v])}" alt="{esc("Prerequisite arrows of " + s["load"] + " along the witness order, " + v)}"{"" if v == "direct" else " hidden"}>' for v, _ in VARIANTS)
            + '</div>'
            f'<figcaption><b>Arrows along the witness order</b> (the order that built it, left to right). '
            f'{nn["support"] + nn["access"]} cause(s) found: {nn["support"]} hard support, {nn["access"]} access'
            f'{", drawn as " + str(s["edges"]) + " arrows" if s["edges"] != nn["support"] + nn["access"] else ""}; '
            f'{s["free"]} free dish(es).</figcaption></figure>'
            f'<figure><div class="graphbox">'
            + "".join(f'<img data-variant="{v}" src="{esc(s["files"]["layers_" + v])}" alt="{esc("The same arrows of " + s["load"] + " layered, " + v)}"{"" if v == "direct" else " hidden"}>' for v, _ in VARIANTS)
            + '</div>'
            f'<figcaption><b>The same arrows, layered.</b> A dish sits one layer above its deepest prerequisite; layer 0 '
            f'holds the dishes that need nothing in place first, the free ones at its right end.</figcaption></figure>'
            f'</section>')
    buttons = "".join(f'<button type="button" data-k="{esc(s["key"])}" aria-pressed="{str(k == 0).lower()}">'
                      f'{esc(s["load"])}</button>' for k, s in enumerate(samples))
    stage_n = lambda *ks: sum(r["stage"] in ks for r in rows)                              # noqa: E731
    stats = [(f"{nv} / {att}", "packings with a valid sample"), (stage_n("failed"), "the sampler could not place all 24"),
             (sum(r["stage"] == "invalid" and (r["gate"] or "accepted") != "accepted" for r in rows), "failed the joint gate"),
             (sum(r["stage"] == "invalid" and r["gate"] == "accepted" for r in rows), "witness did not rebuild"),
             (stage_n("killed", "error"), "stopped or errored")]
    trs = "".join(f'<tr><td class="mono">{esc(r["load"])}{" <span class=muted>imported</span>" if r["resumed"] else ""}</td>'
                  f'<td class="stage-{esc(r["stage"])}">{esc(r["stage"])}</td><td>{esc(r["gate"] or "")}</td>'
                  f'<td>{esc(r["certified"])}</td><td>{esc(r["edges"])}</td><td class="wrap">{esc(r["reason"] or "")}</td></tr>'
                  for r in rows)
    disj = "; ".join(f'{s["load"]}: ' + ", ".join(s["disj"]) for s in samples if s["disj"])
    body = PHYSICAL_BODY.format(
        date=esc(((state or {}).get("updated_utc") or "")[:10]), buttons=buttons, sections="".join(sections),
        stats="".join(f'<div class="stat"><b>{esc(str(b))}</b><span>{esc(t)}</span></div>' for b, t in stats),
        nosample="" if samples else '<p class="lead"><b>No valid sample in this run.</b> The table below says where each '
                                    'packing stopped.</p>',
        rows=trs, disj=esc(disj or "none"), target=esc(str((state or {}).get("target_valid", 5))),
        maxp=esc(str((state or {}).get("max_packings", 20))))
    style = TEMPLATE.split("<style>", 1)[1].split("</style>", 1)[0]
    text = PHYSICAL_HEAD.replace("__STYLE__", style + PHYSICAL_STYLE) + body + PHYSICAL_SCRIPT
    (a.out / "index.html").write_text(text)
    files = [a.out / "index.html", *copied]
    big = [p.name for p in files if p.stat().st_size > MAX_FILE]
    print(f"[RESULT] {'FAIL' if big else 'PASS'} page {a.out / 'index.html'}: {len(samples)} valid samples "
          f"({', '.join(s['load'] for s in samples) or 'none'}), {att} packings in the yield, "
          f"{(a.out / 'index.html').stat().st_size} B, {len(copied)} media files, {sum(p.stat().st_size for p in files)} B"
          + (f"; over 15 MB: {big}" if big else ""), flush=True)


def gate_reason(gdir, lid):
    """A joint-gate failure in words from <gdir>/<lid>[_lower_first]/result.json, else None."""
    tries = [json.loads(p.read_text()) for p in (gdir / lid / "result.json", gdir / f"{lid}_lower_first" / "result.json")
             if p.is_file()]
    if not tries or tries[-1].get("outcome") == "accepted":
        return None
    g = tries[-1]
    if g.get("outcome") == "initial_collision":
        bad = [p for p in (g.get("initial_geometry") or {}).get("pairs") or [] if not p["valid"]]
        return ("joint gate: the 1 mm preflight stops it before any settle, "
                + ", ".join(f"{'/'.join(p['bodies'])} overlap {p['max_penetration_m'] * 1e3:.2f} mm" for p in bad)
                + " at the built poses")
    speeds = [f"{m['peak_measured_rack_speed_m_s']:.3f} m/s" for t in tries for m in t.get("loaded_rack_motions") or []
              if not m["passed"] and "speed" in (m.get("reason") or "")]
    if speeds:
        return (f"joint gate: the loaded lower rack moved {' and '.join(speeds)} while retracting (cap 0.10 m/s), "
                + ("upper rack first and then lower rack first" if len(tries) > 1 else "upper rack first"))
    return None


def yield_rows(state, gdir=None):
    """Every packing the physical run started: {load, stage, reason, gate, certified, edges, resumed}; ``gdir`` = the
    run's _gate folder (the failure reasons in words)."""
    out = []
    for k, v in ((state or {}).get("packings") or {}).items():
        if v.get("started") is None:
            continue
        n = v.get("edges") or {}
        gates = v.get("gates") or ([v["gate"]] if v.get("gate") else [])
        reason = (gate_reason(gdir, k) if gdir is not None and v["stage"] == "invalid" else None) or v.get("reason") or ""
        reason = reason.replace("(['UpperRack', 'LowerRack'])", "(upper rack first)").replace(
            "(['LowerRack', 'UpperRack'])", "(lower rack first)")
        out.append({"load": k, "stage": v["stage"], "reason": reason, "resumed": v.get("resumed"),
                    "gate": " then ".join(gates) or None,
                    "certified": "" if v.get("certified") is None else "24 of 24" if v["certified"] else "no",
                    "edges": f"{n.get('support', 0)} support, {n.get('access', 0)} access" if n else ""})
    return out


PHYSICAL_HEAD = r"""<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Dependency graphs</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>__STYLE__</style>
"""

PHYSICAL_STYLE = """
/* the graph PNGs keep fixed colours in both themes: their legend swatches use these */
:root { --img-plate: #2171b5; --img-bowl: #d94801; --img-cup: #238b45 }
figure { margin: 0; display: grid; gap: 8px; min-width: 0 }
figure img { width: 100%; max-width: 100%; height: auto; display: block; border-radius: 8px; border: 1px solid var(--line) }
.graphbox { overflow-x: auto; min-width: 0 }
.graphbox img { min-width: 760px; max-width: none }
figure video { width: 100%; max-width: 100%; aspect-ratio: 640 / 480; background: var(--sunk); border-radius: 8px }
figcaption { color: var(--muted); font-size: .95rem; max-width: 80ch }
figcaption b { color: var(--ink); font-weight: 600 }
.sample { display: grid; gap: 28px }
ol.claim { margin: 0; padding-left: 1.3rem; display: grid; gap: 6px; max-width: 80ch }
.legendbox { display: grid; gap: 6px; max-width: 80ch; font-size: .95rem }
.dot { display: inline-block; width: 12px; height: 12px; border-radius: 50%; vertical-align: -1px; margin-right: 4px }
.dot.plate { background: var(--img-plate) } .dot.bowl { background: var(--img-bowl) } .dot.cup { background: var(--img-cup) }
"""

PHYSICAL_BODY = """<div class="wrap">
<header>
  <div class="eyebrow">Frigidaire twin · HOTEC set · Isaac Sim 4.5 · physical causes · {date}</div>
  <h1>Dependency graphs</h1>
  <p class="lead">Each sample is a load of 24 HOTEC dishes that Isaac built one dish at a time, every dish lowered from above
  its rack's rim (plates upright, so they tip into a lean) and let go 20 mm above where the rack stops it. A sample is valid
  when three things hold, and it claims nothing more:</p>
  <ol class="claim">
    <li><b>Physically possible and stable.</b> The built poses pass the bench's joint gate: all 24 placed at once settle
    with a peak penetration under 5 mm and a median under 1 mm, both racks retract with them, every dish stays inside.</li>
    <li><b>One sequential order exists.</b> The witness order, replayed from empty racks in a fresh Isaac process with the same
    move, lands all 24 within the bench's at-goal tolerance of the built poses.</li>
    <li><b>An arrow only for a physical cause in the finished load.</b> No arrow means no physical cause was found, not that
    any order works.</li>
  </ol>
  <div class="stats">{stats}</div>
  {nosample}
</header>

<section class="legendbox" aria-label="How to read the graphs">
  <h2>Reading the graphs</h2>
  <p>An arrow from X to Y means <b>Y has to be in place before X</b>: the arrow points at the prerequisite. One style
  for both causes. <b>Support:</b> X rests on Y and, dropped into the finished load without Y alone, does not land.
  <b>Access:</b> X, at its final pose, covers the path Y is lowered along, so Y must go in first. A dish resting on several
  dishes of which no single one is necessary gets no arrow (listed below the table). Hollow circles are free dishes: in no
  arrow. Fill: <span class="dot plate"></span>plate · <span class="dot bowl"></span>bowl ·
  <span class="dot cup"></span>cup; labels p, b, c and the dish number. A toggle above each sample's graphs switches
  the arrows drawn: <b>direct</b> (the measured causes, as above), <b>reduction</b> (the fewest arrows that imply the same
  order, so a chain shows as a path) and <b>closure</b> (every must-before pair); the three carry the same ordering
  information, in the same arrow style and direction.</p>
</section>

<div class="samplebar" id="samplebar">
  <span class="muted" id="slabel">Valid sample</span>
  <div class="seg" id="samples" role="group" aria-labelledby="slabel">{buttons}</div>
</div>
{sections}

<section id="yieldsec">
  <h2>Every packing this run tried</h2>
  <p class="muted">The two imported packings were built by the sampler in the previous rounds (their sampler output reused);
  the rest are new packings of the fresh list, sampled until {target} valid samples, {maxp} packings or three hours of
  the Isaac box; r07_00012 was then sampled again with the overlap rule below (2026-10-16, its row is that rerun).
  Failed: the sampler could not place all 24. Invalid: the joint gate rejected the built poses, or the
  witness did not rebuild. Killed: still running at the stop.</p>
  <div class="panel scroll"><table><tr><th>Packing</th><th>Outcome</th><th>Joint gate</th><th>Witness rebuild</th>
  <th>Arrows</th><th>Why</th></tr>{rows}</table></div>
  <p class="muted">Disjunctive supports (no arrow): {disj}.</p>
</section>

<section id="rules">
  <h2>How it was measured</h2>
  <ul class="rules">
    <li>Move: a plate stands upright in its slot, 3 mm toward the side its packed pose leans to; a bowl or cup keeps its
    packed pose. It starts 20 mm above its rack's rim (top under the tub ceiling) and goes down in 5 mm steps, each checked
    against the placed dishes and the appliance with the 1 mm allowance; a placed dish in the way blocks the move, the rack
    alone stops it; it is let go 20 mm above the lowest clear step and settles 150 + 60 ticks at 120 Hz.</li>
    <li>Sampler (unchanged): a random order with each plate bank filled from its lean side, re-posing (up to five other
    free poses with a clear descent), deferral, no backtracking; a dish goes in when it lands within 80 mm of its pose with
    contact under 2 mm and moves no placed dish out of its tolerance. In r07_00012's rerun also: after each drop the
    placed dishes are checked pairwise, and against the racks, basket, cabinet and door, with the joint gate's own 1 mm
    preflight (the same collision pieces); a drop that leaves an overlap over 1 mm is undone and re-posed like a failed
    landing.</li>
    <li>Support: after the fresh-process rebuild, every dish-dish contact at rest whose net contact force on X from Y points
    up by more than 0.05 N (the force sign is checked on the dish-rack contacts). Each supported dish is dropped by the move
    into the finished load without its supports (dishes covering its path and everything resting on it or on a removed
    support also out), with each support alone, and, with several, with all of them and with only one removed. An arrow
    X to Y when X lands with Y and does not land without Y alone.</li>
    <li>Access: the move's own path check at the final poses, without Isaac: X's final pose fails the 1 mm allowance against
    Y somewhere between Y's start and its lowest clear step.</li>
    <li>A pair with causes both ways gets no arrow, and a cause the witness order itself breaks is not drawn (the witness
    built the load that way); so every arrow points back along the witness order and the layers exist.</li>
  </ul>
</section>
</div>
"""

PHYSICAL_SCRIPT = """<script>
"use strict";
const bar = document.getElementById("samples");
function show(key) {
  document.querySelectorAll(".sample").forEach((s) => { s.hidden = s.dataset.key !== key; });
  bar.querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.k === key)));
  document.querySelectorAll("video").forEach((v) => v.pause());
  try { history.replaceState(null, "", "#" + key); } catch (e) { /* sandboxed */ }
}
bar.addEventListener("click", (ev) => { const b = ev.target.closest("button[data-k]"); if (b) show(b.dataset.k); });
document.querySelectorAll(".variants").forEach((g) => g.addEventListener("click", (ev) => {
  const b = ev.target.closest("button[data-v]");
  if (!b) return;
  g.querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
  g.closest(".sample").querySelectorAll("[data-variant]").forEach((el) => { el.hidden = el.dataset.variant !== b.dataset.v; });
}));
const first = bar.querySelector("button");
if (!first) document.getElementById("samplebar").hidden = true;
else show((location.hash && bar.querySelector('button[data-k="' + location.hash.slice(1) + '"]')) ? location.hash.slice(1) : first.dataset.k);
</script>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--records", nargs="*", default=[], metavar="KEY=PATH",
                    help="one valid --edges swaps --release insertion record per sample (page order)")
    ap.add_argument("--clips", nargs="*", default=[], metavar="KEY=DIR",
                    help="frigidaire_dependency_clips.py --out (with clips.json) per key")
    ap.add_argument("--state", type=Path, help="the --sample scheduler's state (scheduler_state.json): the yield")
    ap.add_argument("--failures", type=Path, metavar="DIR",
                    help="the --sample records folder: why the packings that did not build stopped")
    ap.add_argument("--failed-clip", metavar="LOAD=DIR", help="a failed record's clips (its last sampler build)")
    ap.add_argument("--screen", type=Path, help="the Kit-free screen (screen_reposing.json, screen_chain.json: its counts "
                    "per kind; screen_backtrack.json: its summary line)")
    ap.add_argument("--failure-note", help="the round's explanation of why the sampler stopped (default: the 2026-10-15 b one)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    records, clip_dirs = keyed(a.records, "--records"), keyed(a.clips, "--clips")
    state = json.loads(a.state.read_text()) if a.state and a.state.is_file() else None
    if (state or {}).get("procedure") == "physical":         # plans/2026-10-16-physical-cause-graph.md
        return physical_page(a, records, clip_dirs, state)
    out_media = a.out / "media"
    out_media.mkdir(parents=True, exist_ok=True)
    copied, samples = [], []
    for key, path in records.items():
        rec = json.loads(path.read_text())
        cdir = clip_dirs.get(key)
        clips = json.loads((cdir / "clips.json").read_text()) if cdir and (cdir / "clips.json").is_file() else None
        names = []
        for c in ([clips["witness"]] if clips and clips.get("witness") else []) + list((clips or {}).get("edges", {}).values()) \
                + list((clips or {}).get("conditional", {}).values()):
            names += [c["file"], c["poster"]]
        if clips and clips.get("load"):
            names.append(clips["load"])
        for name in names:
            shutil.copy2(cdir / name, out_media / f"{key}_{name}")
            copied.append(out_media / f"{key}_{name}")
        samples.append(page_data(rec, clips, f"{key}_", scores_of(path, rec["load_id"])))
    screen = json.loads(a.screen.read_text()) if a.screen and a.screen.is_file() else None
    failure = failure_of(a.failures, screen) if a.failures else None
    if failure and a.failed_clip:
        load, _, cdir = a.failed_clip.partition("=")
        c = json.loads((Path(cdir) / "clips.json").read_text())["witness"]
        for name in (c["file"], c["poster"]):
            shutil.copy2(Path(cdir) / name, out_media / f"failed_{load}_{name}")
            copied.append(out_media / f"failed_{load}_{name}")
        failure["clip"] = {"load": load, "file": f"media/failed_{load}_{c['file']}",
                           "poster": f"media/failed_{load}_{c['poster']}", "seconds": c["seconds"]}
    if failure and a.failure_note:
        failure["note"] = a.failure_note
    page = {"samples": samples, "yield": yield_of(state), "failure": failure,
            "generated": (state or {}).get("updated_utc", "")[:10],
            "backtrack": (state or {}).get("backtrack", 3), "resumed": (state or {}).get("resumed") or [],
            "skipped": sum(v.get("stage") == "skipped" for v in ((state or {}).get("packings") or {}).values()),
            "tolerance": samples[0]["tolerance"] if samples else {"name": (state or {}).get("tolerance") or "bench",
                                                                   "text": ""}}
    text = TEMPLATE.replace("__DATA__", json.dumps(page, separators=(",", ":")).replace("</", "<\\/"))
    (a.out / "index.html").write_text(text)
    files = [a.out / "index.html", *copied]
    big = [p.name for p in files if p.stat().st_size > MAX_FILE]
    total = sum(p.stat().st_size for p in files)
    print(f"[RESULT] {'FAIL' if big else 'PASS'} page {a.out / 'index.html'}: {len(samples)} samples "
          f"({', '.join(s['load'] for s in samples)}), {len(page['yield'])} packings in the yield, "
          f"{(a.out / 'index.html').stat().st_size} B, {len(copied)} media files, {total} B in all"
          + (f"; over 15 MB: {big}" if big else ""), flush=True)


TEMPLATE = r"""<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Dependency graphs</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600;700&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>
/* Layout: one 1120px column: the yield of the run, a sticky sample selector, then per sample the tuple (load still
   and S beside the witness video, the graph full width under them), the edge clips, the checker, the library table. */
:root {
  --bg: #edf0f3; --surface: #ffffff; --sunk: #e3e8ed; --ink: #15212c; --muted: #536373; --line: #cdd6df;
  --accent: #0b5d8f; --accent-ink: #ffffff;
  --plate: #2171b5; --bowl: #d94801; --cup: #238b45;
  --need: #8b3fd9; --block: #0e7490; --knock: #c0264b; --learned: #6b5a00;
  --ok: #1d7a46; --bad: #b42318; --warn: #9a5b00;
  --display: "Barlow Condensed", "Arial Narrow", "Helvetica Neue", sans-serif;
  --body: "Source Sans 3", "Segoe UI", system-ui, sans-serif;
  --mono: "JetBrains Mono", ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0d1218; --surface: #151c24; --sunk: #1b242e; --ink: #e3eaf1; --muted: #93a4b5; --line: #2b3744;
  --accent: #5fb0e8; --accent-ink: #0d1218;
  --plate: #6baed6; --bowl: #fd8d3c; --cup: #74c476;
  --need: #b98cf5; --block: #3fc1d9; --knock: #f07390; --learned: #e0c34a;
  --ok: #4cc285; --bad: #f2726b; --warn: #e3a64a; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #0d1218; --surface: #151c24; --sunk: #1b242e; --ink: #e3eaf1; --muted: #93a4b5; --line: #2b3744;
  --accent: #5fb0e8; --accent-ink: #0d1218;
  --plate: #6baed6; --bowl: #fd8d3c; --cup: #74c476;
  --need: #b98cf5; --block: #3fc1d9; --knock: #f07390; --learned: #e0c34a;
  --ok: #4cc285; --bad: #f2726b; --warn: #e3a64a; color-scheme: dark }
* { box-sizing: border-box }
[hidden] { display: none !important }
body { background: var(--bg); color: var(--ink); font: 16px/1.55 var(--body); margin: 0; padding-inline: 16px; padding-block: 28px 64px }
.wrap { max-width: 1120px; margin: 0 auto; display: grid; gap: 40px }
h1, h2, h3 { font-family: var(--display); font-weight: 600; letter-spacing: .01em; line-height: 1.1; margin: 0; text-wrap: balance }
h1 { font-size: clamp(2.2rem, 5vw, 3.2rem) }
h2 { font-size: 1.75rem }
h3 { font-size: 1.2rem }
p { margin: 0; max-width: 68ch }
.eyebrow { font: 600 .78rem/1.2 var(--mono); letter-spacing: .08em; text-transform: uppercase; color: var(--muted) }
.lead { font-size: 1.08rem }
.muted { color: var(--muted) }
.mono, code { font-family: var(--mono); font-size: .88em }
section { display: grid; gap: 16px; min-width: 0 }
header { display: grid; gap: 12px }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 1px; background: var(--line); border: 1px solid var(--line); border-radius: 10px; overflow: hidden }
.stat { background: var(--surface); padding: 14px 16px; display: grid; gap: 2px; align-content: start }
.stat b { font: 600 1.9rem/1.1 var(--display); font-variant-numeric: tabular-nums }
.stat span { color: var(--muted); font-size: .9rem }
.two { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 24px; align-items: start }
@media (max-width: 820px) { .two { grid-template-columns: minmax(0, 1fr) } }
.tile { display: grid; gap: 10px; align-content: start; min-width: 0 }
.tile h3 { display: flex; gap: 10px; align-items: baseline; flex-wrap: wrap }
video, .still { width: 100%; max-width: 100%; aspect-ratio: 640 / 480; background: #121a22; border-radius: 8px; display: block; object-fit: contain }
.facts { display: grid; gap: 8px; margin: 0 }
.facts div { display: grid; grid-template-columns: 8.5rem minmax(0, 1fr); gap: 12px; border-bottom: 1px solid var(--line); padding-bottom: 6px }
.facts dt { color: var(--muted); font-size: .9rem }
.facts dd { margin: 0; min-width: 0; font-size: .95rem }
.panel { background: var(--surface); border: 1px solid var(--line); border-radius: 10px; padding: 18px }
.scroll { overflow-x: auto; min-width: 0 }
svg text { font-family: var(--mono); fill: var(--ink) }
.legend { display: flex; flex-wrap: wrap; gap: 8px 18px; font-size: .9rem; color: var(--muted); align-items: center }
.key { display: inline-flex; align-items: center; gap: 6px }
.swatch { width: 22px; height: 0; border-top: 3px solid currentColor; display: inline-block }
.swatch.dash { border-top-style: dashed }
.swatch.dot { border-top-style: dotted }
.need { color: var(--need) } .block { color: var(--block) } .knock { color: var(--knock) } .learned { color: var(--learned) }
.pill { display: inline-block; font: 600 .74rem/1.4 var(--mono); letter-spacing: .04em; text-transform: uppercase; padding: 1px 8px; border-radius: 999px; border: 1px solid currentColor; white-space: nowrap }
.pill.yes { color: var(--ok) } .pill.no { color: var(--bad) } .pill.untested { color: var(--warn) }
.score { display: flex; flex-wrap: wrap; gap: 6px 18px; align-items: baseline }
.score b { font: 600 2rem/1 var(--display); font-variant-numeric: tabular-nums }
.tool { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 20px; align-items: start }
@media (max-width: 900px) { .tool { grid-template-columns: minmax(0, 1fr) } }
.controls { display: flex; flex-wrap: wrap; gap: 8px; align-items: center }
button, select { font: 600 .9rem var(--body); border-radius: 7px; border: 1px solid var(--line); background: var(--surface); color: var(--ink); padding: 7px 12px; cursor: pointer }
button.primary { background: var(--accent); color: var(--accent-ink); border-color: var(--accent) }
button:focus-visible, select:focus-visible, .chip:focus-visible { outline: 3px solid var(--accent); outline-offset: 2px }
select { max-width: 100%; font-weight: 400 }
.strip { display: grid; grid-template-columns: repeat(auto-fill, minmax(58px, 1fr)); gap: 6px; user-select: none }
.chip { display: grid; gap: 0; justify-items: start; padding: 5px 7px 6px; border-radius: 7px; border: 1px solid var(--line); background: var(--sunk); color: var(--ink); cursor: grab; text-align: left; font-weight: 400; position: relative }
.chip .n { font: 400 .68rem/1 var(--mono); color: var(--muted) }
.chip .s { font: 600 .95rem/1.2 var(--mono) }
.chip::before { content: ""; position: absolute; left: 0; top: 6px; bottom: 6px; width: 3px; border-radius: 2px; background: var(--k) }
.chip.picked { border-color: var(--accent); box-shadow: 0 0 0 2px var(--accent) inset }
.chip.inedge { border-color: var(--knock) }
.chip.fails { background: color-mix(in srgb, var(--bad) 16%, var(--sunk)); border-color: var(--bad) }
.chip.over { box-shadow: -4px 0 0 0 var(--accent) }
.result { display: grid; gap: 12px; min-width: 0 }
.verdict { font: 600 1.25rem/1.25 var(--display) }
.vlist { display: grid; gap: 8px; margin: 0; padding: 0; list-style: none }
.vlist li { display: grid; grid-template-columns: auto minmax(0, 1fr) auto; gap: 10px; align-items: center; border-top: 1px solid var(--line); padding-top: 8px }
.isaac { border-left: 3px solid var(--line); padding-left: 12px; display: grid; gap: 4px }
.cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 16px }
.card { background: var(--surface); border: 1px solid var(--line); border-radius: 10px; padding: 12px; display: grid; gap: 8px; align-content: start; min-width: 0 }
.card h3 { display: flex; gap: 8px; align-items: baseline; flex-wrap: wrap }
.tags { display: flex; gap: 6px; flex-wrap: wrap }
.card p { font-size: .93rem }
.fan { display: grid; gap: 6px; border-top: 1px dashed var(--line); padding-top: 8px }
table { border-collapse: collapse; width: 100%; font-size: .92rem; font-variant-numeric: tabular-nums }
th, td { text-align: left; padding: 7px 10px; border-bottom: 1px solid var(--line); vertical-align: top; white-space: nowrap }
td.wrap { white-space: normal; min-width: 16rem }
th { font: 600 .74rem/1.2 var(--mono); letter-spacing: .06em; text-transform: uppercase; color: var(--muted) }
tr.sel td { background: color-mix(in srgb, var(--accent) 10%, transparent) }
ul.rules { margin: 0; padding-left: 1.1rem; display: grid; gap: 6px; max-width: 80ch }
.empty { color: var(--muted); font-style: italic }
@media (prefers-reduced-motion: reduce) { * { scroll-behavior: auto !important; transition: none !important } }
.samplebar { position: sticky; top: env(safe-area-inset-top, 0px); z-index: 5; background: var(--bg); padding-block: 10px; margin-block: -28px; border-bottom: 1px solid var(--line); display: flex; flex-wrap: wrap; gap: 8px 14px; align-items: center }
.seg { display: inline-flex; flex-wrap: wrap; border: 1px solid var(--line); border-radius: 8px; overflow: hidden; background: var(--surface) }
.seg button { border: 0; border-radius: 0; border-left: 1px solid var(--line); font: 600 .85rem var(--mono); padding: 7px 12px }
.seg button:first-child { border-left: 0 }
.seg button[aria-pressed="true"] { background: var(--accent); color: var(--accent-ink) }
.variants { display: grid; gap: 8px; justify-items: start }
.stage-valid { color: var(--ok) } .stage-invalid, .stage-error { color: var(--bad) } .stage-failed, .stage-killed { color: var(--warn) }
</style>

<div class="wrap">
<header>
  <div class="eyebrow">Frigidaire twin · HOTEC set · Isaac Sim 4.5 · re-posing sampler · <span id="gen"></span></div>
  <h1>Dependency graphs</h1>
  <p class="lead" id="lead"></p>
  <div class="stats" id="stats"></div>
</header>

<section id="yieldsec">
  <h2>Every packing the sampler tried</h2>
  <p class="muted" id="yieldnote"></p>
  <div class="panel scroll"><table id="yieldtable"></table></div>
</section>

<section id="nosample" hidden>
  <h2 id="nstitle">Why no sample is valid</h2>
  <p id="nsnote"></p>
  <div class="two">
    <div class="tile">
      <div id="nsvideo"></div>
      <p class="muted" id="nscaption"></p>
    </div>
    <dl class="facts" id="nsfacts"></dl>
  </div>
</section>

<div class="samplebar" id="samplebar">
  <span class="muted" id="slabel">Sample</span>
  <div class="seg" id="samples" role="group" aria-labelledby="slabel"></div>
  <span class="muted" id="snote"></span>
</div>

<section id="tuple">
  <h2 id="tupletitle"></h2>
  <p id="verdictline" hidden></p>
  <div class="two">
    <div class="tile">
      <h3>The load <span class="muted" style="font: 400 .9rem var(--body)">the built goal, both racks</span></h3>
      <div id="still"></div>
      <div class="score" id="score"></div>
      <dl class="facts" id="loadfacts"></dl>
    </div>
    <div class="tile">
      <h3>The witness <span class="muted" style="font: 400 .9rem var(--body)">the order that built it</span></h3>
      <div id="wvideo"></div>
      <dl class="facts" id="wfacts"></dl>
    </div>
  </div>
  <div class="tile">
    <h3>The graph <span class="muted" style="font: 400 .9rem var(--body)">drawn along the witness order</span></h3>
    <div class="legend">
      <span class="key need"><span class="swatch"></span>need</span>
      <span class="key block"><span class="swatch"></span>block</span>
      <span class="key knock"><span class="swatch"></span>knock</span>
      <span class="key learned"><span class="swatch dot"></span>learned: an order that broke no edge failed, and its build named this pair</span>
      <span class="key"><span class="swatch dash"></span>conditional fan: b needs any one of these first</span>
      <span class="key"><svg width="12" height="12" aria-hidden="true"><circle cx="6" cy="6" r="4" fill="currentColor"></circle></svg>support (b rests on a)</span>
      <span class="key"><span class="mono">p</span> plate · <span class="mono">b</span> bowl · <span class="mono">c</span> cup</span>
      <span class="key" style="color:var(--plate)"><span class="swatch dash"></span>plate chain under the line (F front bank, R rear bank, in the order the plates went in: each leans onto the one before)</span>
    </div>
    <div class="panel scroll"><svg id="arcs" role="img" aria-label="Dependency arcs along the witness order"></svg></div>
    <p class="muted" id="graphnote"></p>
  </div>
</section>

<section id="clips">
  <h2>Each edge and the swap that made it</h2>
  <p class="muted" id="clipsnote"></p>
  <div class="cards" id="cards"></div>
</section>

<section id="checker">
  <h2>Order checker</h2>
  <p class="muted">Drag a dish to a new place, or click it and then click the dish it should go before. Arrow keys move the focused dish. The checker lists every edge the order breaks and the dish the graph says fails first; a conditional edge holds once any of its dishes is in.</p>
  <div class="tool">
    <div class="panel" style="display:grid;gap:12px">
      <div class="controls">
        <button class="primary" id="reset" type="button">Witness order</button>
        <button id="random" type="button">Random obeying order</button>
        <label class="muted" for="libsel" style="font-size:.9rem">Library</label>
        <select id="libsel"></select>
      </div>
      <div class="strip" id="strip" aria-label="Placement order"></div>
    </div>
    <div class="panel result" id="result" aria-live="polite"></div>
  </div>
</section>

<section id="library">
  <h2>What Isaac did with the library</h2>
  <p class="muted" id="libnote"></p>
  <div class="panel scroll"><table id="libtable"></table></div>
</section>

<section id="rules">
  <h2>How it was measured</h2>
  <ul class="rules" id="ruleslist"></ul>
</section>
</div>

<script id="data" type="application/json">__DATA__</script>
<script>
"use strict";
const ALL = JSON.parse(document.getElementById("data").textContent);
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
const KIND = {plate: "var(--plate)", bowl: "var(--bowl)", cup: "var(--cup)"};
const RULE = {"lean-side": "lean-side plate chains", random: "uniformly random"};
const KINDNAME = {witness: "witness", edge_violation: "edge violation", obeying: "random obeying", random_violation: "random single violation", refined_obeying: "fresh obeying (refined graph)", learned_obeying: "fresh obeying (learning round)"};
const TOLNAME = {library: "doubled", bench: "bench"};
const tolText = (t) => (t && t.name === "library" ? "the at-goal tolerance doubled for the swaps, the library and learning (" + t.text + "); the sampler keeps its own landing rule (80 mm, contact under 2 mm)" : "the bench's at-goal tolerance" + (t && t.text ? " (" + t.text + ")" : ""));
const shortId = (id) => { const k = id.slice(0, id.lastIndexOf("_")), n = id.slice(id.lastIndexOf("_") + 1); return k[0] + n; };
let D = null, dish = {}, witness = [], order = [], picked = null, current = 0;
const sh = (id) => (dish[id] ? dish[id].short : shortId(id));
const members = (e) => (e.any_of && e.any_of.length ? e.any_of : [e.a]);
function reasonText(r) {
  if (r === "need") return "the later dish does not land without the earlier one";
  if (r === "block") return "the earlier dish cannot go in once the later one is there (its descent is blocked, or it does not land)";
  if (r === "learned") return "an order that broke no edge failed here, and the build's own diagnosis named this pair";
  return "a drop moves a placed dish out of its tolerance";
}

function check(ord) {
  const pos = Object.fromEntries(ord.map((id, i) => [id, i]));
  const violated = D.edges.filter((e) => members(e).every((m) => pos[m] > pos[e.b]));
  let first = null;
  for (const e of violated) { const p = pos[e.fails]; if (first === null || p < first.pos) first = {pos: p, dish: e.fails}; }
  const key = ord.join(" ");
  const lib = D.library.find((l) => l.order.join(" ") === key) || null;
  return {violated, first, lib};
}

function outcomeText(o) {
  if (!o) return "listed in the library but not built";
  if (o.status === "ok") return "all " + witness.length + " dishes landed";
  if (o.status === "stopped") return "no failure up to its stop (" + o.placed + " placed)";
  const what = o.status === "moved" ? "moved " + (o.moved || []).map(sh).join(", ") : "did not go in";
  return "failed at " + sh(o.failed_dish) + " (position " + (o.failed_position + 1) + "): " + what;
}

// ---- yield and header (built once)
function yieldTable() {
  const y = ALL.yield;
  if (!y.length) { $("yieldsec").hidden = true; return; }
  const n = (st) => y.filter((r) => r.stage === st).length;
  const bt = ALL.backtrack ? "The sampler backtracks: a dish that cannot go in undoes up to three of its most recently placed blockers, goes in, and they go back. " : "No backtracking: a dish that goes in neither at its own pose nor at a re-pose waits until later in the order, and the load fails when no dish left goes in. ";
  const rs = ALL.resumed.length ? ALL.resumed.join(" and ") + " were built by the previous run; here their saved swap drops were judged again under the tolerance below and their library and learning were built again in Isaac. New packings of the fresh list start only once " + (ALL.resumed.length === 2 ? "both are" : "all of them are") + " valid" + (ALL.skipped ? "; " + (ALL.resumed.length === 2 ? "neither was" : "not all were") + ", so the other " + ALL.skipped + " packings of this run were never started" : "") + ". " : "";
  $("yieldnote").textContent = "Packings in the order of the fresh list, run in parallel. " + rs + "Order rule: lean-side plate chains = a random order of all 24 dishes in which each plate bank's plates go in from the slot furthest on its lean side outward, bowls and cups at random among them. " + bt + "Tolerance: " + tolText(ALL.tolerance) + ". Valid: every library order that obeys the graph built all 24 and every order that breaks an edge failed somewhere; an obeying order that failed taught a learned edge, and the last of up to three learning rounds of 10 fresh obeying orders passed. Invalid: it did not. Failed: the sampler could not place all 24 dishes. Killed: still running when the run stopped.";
  $("yieldtable").innerHTML = "<tr><th>Packing</th><th>Order rule</th><th>Tolerance</th><th>Outcome</th><th>Backtracks</th><th>Library</th><th>Learning rounds</th><th>Why</th></tr>" + y.map((r) => '<tr><td class="mono">' + esc(r.load) + (r.resumed ? ' <span class="muted">rerun</span>' : "") + "</td><td>" + esc(RULE[r.rule] || r.rule || "") + "</td><td>" + esc(TOLNAME[r.tolerance] || r.tolerance || "") + '</td><td class="stage-' + esc(r.stage) + '">' + esc(r.stage) + "</td><td>" + esc(r.backtracks === undefined || r.backtracks === null ? "" : r.backtracks) + "</td><td>" + esc(r.orders || "") + "</td><td>" + esc(r.rounds === undefined || r.rounds === null ? "" : r.rounds) + '</td><td class="wrap">' + esc(r.reason || "") + "</td></tr>").join("");
  $("stats").innerHTML = [[n("valid") + " / " + y.length, "packings with a valid sample"], [n("invalid"), "built, graph not confirmed by the library"], [n("failed"), "the sampler could not place all 24"], [n("killed") + n("error"), "stopped or errored before a verdict"]].map(([b, s]) => '<div class="stat"><b>' + esc(b) + "</b><span>" + esc(s) + "</span></div>").join("");
}
function lead() {
  $("gen").textContent = ALL.samples.length ? ALL.samples[0].generated : ALL.generated;
  $("lead").textContent = "Each sample is a load of 24 HOTEC dishes that Isaac built one dish at a time: every dish is lowered from above its rack's rim (plates upright, so they tip into a lean) and let go 20 mm above where the rack stops it. The order is random, except that each plate bank fills from its lean side outward, so every plate leans onto the one before it. A dish that cannot go in is tried at up to five other free poses (a plate only further along its bank's chain); if none goes in, " + (ALL.backtrack ? "up to three of the placed dishes that blocked it are taken out, it goes in, and they go back (or wait until later)" : "it waits until later in the order (this run samples without backtracking" + (ALL.resumed.length ? "; the rerun samples were built with it allowed and used none" : "") + ")") + ". The order that worked is the witness, the poses it reached are the goal, and swapping pairs of that order gives the dependency graph. A sample counts when every order in its library that obeys the graph builds all 24 and every order that breaks an edge fails somewhere; an obeying order that fails teaches the graph a learned edge, and fresh obeying orders check it again, up to three rounds. Tolerance in force: " + tolText(ALL.tolerance) + ".";
}

// ---- sample selector
function selector() {
  $("samples").innerHTML = ALL.samples.map((s, i) => '<button type="button" data-k="' + i + '" aria-pressed="' + (i === current) + '">' + esc(s.load) + (s.valid === true ? "" : " · " + esc((s.verdict && s.verdict.stage) || "pending")) + "</button>").join("");
  const nv = ALL.samples.filter((s) => s.valid === true).length;
  $("snote").textContent = nv + " valid of " + ALL.samples.length + " built";
}
$("samples").addEventListener("click", (ev) => { const b = ev.target.closest("button[data-k]"); if (b) setSample(+b.dataset.k); });

// ---- the tuple
function tuple() {
  $("tupletitle").textContent = "Sample " + D.load + (D.valid === true ? "" : " · " + ((D.verdict && D.verdict.stage) || "pending"));
  $("verdictline").hidden = D.valid === true;
  $("verdictline").innerHTML = D.valid === true ? "" : '<span class="pill no">' + esc((D.verdict && D.verdict.stage) || "pending") + "</span> Not a valid sample: " + esc((D.verdict && D.verdict.reason) || "") + ". The run stops building a sample's library once it is invalid, so the orders listed as not built never ran.";
  $("still").innerHTML = D.still ? '<img class="still" src="' + esc(D.still) + '" alt="' + esc("Both racks of load " + D.load + " as the witness left them") + '">' : '<p class="empty">No still.</p>';
  const sc = D.score;
  $("score").innerHTML = sc && sc.built ? '<span class="muted">S (New+J)</span><b>' + sc.built.NJ.toFixed(4) + '</b><span class="muted">built goal' + (sc.packed ? " · packing " + sc.packed.NJ.toFixed(4) : "") + "</span>" : '<span class="empty">S not scored.</span>';
  const s = D.sampler, g = D.goal_delta;
  const rows = [
    ["Re-posed", s.reposed.length ? s.reposed.map((r) => sh(r.id) + ": " + r.from + " → " + r.to).join("; ") : "none: every dish went in at its packed pose"],
    ["Attempts", s.attempts + " in Isaac (" + s.failed + " failed, " + s.blocked + " of them blocked on the way down)" + (s.deferred.length ? "; " + s.deferred.map(sh).join(", ") + " needed more than one try" : "")],
    ["Backtracks", s.backtracks.length ? s.backtracks.map((b) => sh(b.dish) + " took out " + b.undone.map(sh).join(", ") + ": " + (b.outcome === "placed" ? "went in" + (b.deferred && b.deferred.length ? ", " + b.deferred.map(sh).join(", ") + " went back later" : ", they went back") : b.outcome === "abandoned" ? "abandoned (" + (b.moved || []).map(sh).join(", ") + " moved)" : "still blocked, all put back")).join("; ") : "none"],
    ["Goal vs packing", g ? "up to " + g.lateral_mm + " mm lateral, " + g.dz_mm + " mm in height and " + g.tilt_deg + "° from the poses the dishes went in with" : "n/a"],
  ];
  if (sc && sc.built) rows.push(["S per rule", "built " + Object.entries(sc.built).map(([k, v]) => k + " " + v.toFixed(4)).join(" · ") + (sc.packed ? "; packing " + Object.entries(sc.packed).map(([k, v]) => k + " " + v.toFixed(4)).join(" · ") : "")]);
  $("loadfacts").innerHTML = rows.map(([t, d]) => "<div><dt>" + esc(t) + "</dt><dd>" + esc(d) + "</dd></div>").join("");
  const w = D.witness;
  $("wvideo").innerHTML = w.clip ? '<video controls preload="metadata" playsinline poster="' + esc(w.poster) + '" src="' + esc(w.clip) + '"></video>' : '<p class="empty">No witness clip.</p>';
  const lows = w.lowest_mm.filter((x) => x !== null && x !== undefined);
  $("wfacts").innerHTML = [
    ["Order", w.order.map(sh).join(" ")],
    ["Descents", lows.length ? "each dish stopped " + Math.min(...lows) + "–" + Math.max(...lows) + " mm above its move pose (0 = all the way down) and fell 20 mm; the furthest landed " + w.dev_mm + " mm from its pose (80 mm allowed)" : "n/a"],
    ["Contacts", w.support.length + " support pairs (b rests on a); " + D.edges.filter((e) => e.support).length + " of them are also edges"],
    ["Plate chains", w.chains ? Object.entries(Object.entries(w.chains).filter(([, c]) => c.placed_index !== undefined && c.placed_index !== null).reduce((m, [id, c]) => ((m[c.bank] = m[c.bank] || []).push([c.placed_index, id, c]), m), {})).map(([b, l]) => b.split(" ")[1] + " bank: " + l.sort((p, q) => p[0] - q[0]).map(([, id, c]) => sh(id) + " " + c.placed_slot.split("_")[2] + (c.placed_slot !== c.slot ? " (planned " + c.slot.split("_")[2] + ")" : "")).join(" → ")).join("; ") + " (plate, slot gap)" : "uniformly random order (no chains)"],
  ].map(([t, d]) => "<div><dt>" + esc(t) + '</dt><dd class="' + (t === "Order" ? "mono" : "") + '">' + esc(d) + "</dd></div>").join("");
  const nc = D.edges.filter((e) => e.any_of).length;
  const nl = D.edges.filter((e) => e.reason === "learned").length, L = D.learning || {rounds: []};
  $("graphnote").textContent = "Every dish sits on the line in the witness order. An arc goes from the dish that must go first to the one that must wait, one per failed swap." + (nc ? " " + nc + " edge" + (nc === 1 ? " is" : "s are") + " conditional: the library built an order that broke it, and a drop confirmed which other dish lets b go in instead, so b waits for any one dish of its fan." : " No edge needed the refinement.") + (nl ? " " + nl + " dotted edge" + (nl === 1 ? " was" : "s were") + " learned in " + L.rounds.length + " round" + (L.rounds.length === 1 ? "" : "s") + ": an order that obeyed every edge failed, and its build named the pair." : L.rounds.length ? "" : " No obeying order failed, so nothing was learned.");
}

// ---- arc diagram (a conditional edge: one dashed arc per member of its fan)
const STEP = 44, X0 = 28, Y0 = 210;
function arcs(highlight) {
  const svg = $("arcs");
  const pos = Object.fromEntries(witness.map((id, i) => [id, i]));
  const chains = D.witness.chains || {}, banks = {};
  for (const id of witness) { const c = chains[id]; if (c && c.placed_index !== undefined && c.placed_index !== null) (banks[c.bank] = banks[c.bank] || []).push(id); }
  const W = X0 * 2 + STEP * (witness.length - 1), Ht = Y0 + (Object.keys(banks).length ? 96 : 40);
  svg.setAttribute("viewBox", "0 0 " + W + " " + Ht);
  svg.setAttribute("width", W);
  svg.setAttribute("height", Ht);
  const x = (id) => X0 + STEP * pos[id];
  const hl = new Set((highlight || []).map((e) => e.a + ">" + e.b));
  let s = "<defs>" + ["need", "block", "knock", "learned"].map((r) => '<marker id="m-' + r + '" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8 z" style="fill:var(--' + r + ')"></path></marker>').join("") + "</defs>";
  s += '<line x1="' + (X0 - 14) + '" x2="' + (W - X0 + 14) + '" y1="' + Y0 + '" y2="' + Y0 + '" style="stroke:var(--line)" stroke-width="2"></line>';
  for (const e of D.edges) {
    const on = hl.has(e.a + ">" + e.b), ms = members(e), fan = ms.length > 1;
    for (const m of ms) {
      const xa = x(m), xb = x(e.b), r = Math.abs(xb - xa) / 2, h = Math.min(24 + Math.abs(xb - xa) * 0.42, Y0 - 24);
      s += '<path d="M' + xa + "," + (Y0 - 10) + " A" + r + "," + h + " 0 0 " + (xa < xb ? 1 : 0) + " " + xb + "," + (Y0 - 10) + '" fill="none" style="stroke:var(--' + e.reason + ')" stroke-width="' + (on ? 4.5 : fan ? 1.9 : 2.4) + '"' + (fan ? ' stroke-dasharray="6 4"' : e.reason === "learned" ? ' stroke-dasharray="1.5 4.5" stroke-linecap="round"' : "") + ' marker-end="url(#m-' + e.reason + ')"><title>' + esc((fan ? sh(e.b) + " needs one of " + ms.map(sh).join(", ") : sh(e.a) + " before " + sh(e.b)) + " · " + e.reason + (e.round ? " (round " + e.round + ")" : "") + ": " + e.detail) + "</title></path>";
      if (e.support && m === e.a) s += '<circle cx="' + (xa + xb) / 2 + '" cy="' + (Y0 - 10 - h) + '" r="4.5" style="fill:var(--' + e.reason + ')"></circle>';
    }
    if (fan) s += '<text x="' + x(e.b) + '" y="' + (Y0 - 16) + '" text-anchor="middle" font-size="10" style="fill:var(--' + e.reason + ')">1 of ' + ms.length + "</text>";
  }
  Object.entries(banks).forEach(([b, ids], bi) => {          // each bank's plates in the order they went in
    const tag = (b.split(" ")[1] || "?")[0].toUpperCase();
    ids.forEach((id, k) => {
      s += '<text x="' + x(id) + '" y="' + (Y0 + 38) + '" text-anchor="middle" font-size="10" style="fill:var(--plate)">' + esc(tag + (k + 1)) + "</text>";
      if (k + 1 < ids.length) {
        const xa = x(id), xb = x(ids[k + 1]), h = Math.min(12 + Math.abs(xb - xa) * 0.12, 40);
        s += '<path d="M' + xa + "," + (Y0 + 44) + " Q" + (xa + xb) / 2 + "," + (Y0 + 44 + 2 * h) + " " + xb + "," + (Y0 + 44) + '" fill="none" style="stroke:var(--plate)" stroke-width="1.6" stroke-dasharray="' + (bi ? "1.5 3" : "5 3") + '"><title>' + esc(b + ": " + sh(ids[k + 1]) + " (" + (chains[ids[k + 1]].placed_slot || "") + ") went in after " + sh(id) + " (" + (chains[id].placed_slot || "") + ")") + "</title></path>";
      }
    });
  });
  for (const id of witness) {
    const d = dish[id];
    s += '<circle cx="' + x(id) + '" cy="' + Y0 + '" r="8" style="fill:' + KIND[d.kind] + ';stroke:var(--surface)" stroke-width="2"><title>' + esc(id + " · " + d.rack + (chains[id] ? " · " + chains[id].bank + ", chain " + (chains[id].index + 1) + ", slot " + (chains[id].placed_slot || chains[id].slot) : "")) + "</title></circle>";
    s += '<text x="' + x(id) + '" y="' + (Y0 + 24) + '" text-anchor="middle" font-size="11">' + esc(d.short) + "</text>";
  }
  svg.innerHTML = s;
}

// ---- checker
function render() {
  const res = check(order);
  const inEdge = new Set(res.violated.flatMap((e) => [...members(e), e.b]));
  $("strip").innerHTML = order.map((id, i) => '<button type="button" class="chip' + (id === picked ? " picked" : "") + (inEdge.has(id) ? " inedge" : "") + (res.first && res.first.dish === id ? " fails" : "") + '" draggable="true" data-id="' + id + '" style="--k:' + KIND[dish[id].kind] + '" aria-label="' + esc(id + ", position " + (i + 1)) + '"><span class="n">' + (i + 1) + '</span><span class="s">' + esc(dish[id].short) + "</span></button>").join("");
  let h = "";
  if (!res.violated.length) {
    h += '<div class="verdict">Obeys all ' + D.edges.length + " edges.</div><p>The graph predicts that every dish goes in.</p>";
  } else {
    h += '<div class="verdict">Breaks ' + res.violated.length + " edge" + (res.violated.length === 1 ? "" : "s") + ". Predicted to fail at " + esc(sh(res.first.dish)) + " (position " + (res.first.pos + 1) + ").</div>";
    h += '<ul class="vlist">' + res.violated.map((e, i) => '<li><span class="pill ' + e.reason + '">' + e.reason + "</span><span>" + (e.any_of ? "one of <b class=\"mono\">" + esc(members(e).map(sh).join(", ")) + "</b>" : '<b class="mono">' + esc(sh(e.a)) + "</b>") + ' must go before <b class="mono">' + esc(sh(e.b)) + "</b>. " + esc(e.detail) + "</span>" + (e.clip ? '<button type="button" data-clip="' + i + '">Clip</button>' : "<span></span>") + "</li>").join("") + "</ul>";
  }
  const l = res.lib;
  if (l) {
    const m = l.as_predicted;
    h += '<div class="isaac"><div><span class="pill ' + (m === null ? "untested" : m ? "yes" : l.taught ? "learned" : "no") + '">' + (m === null ? "not built" : m ? "as the rule asks" : l.taught ? "failed, taught an edge" : "not as the rule asks") + "</span> Library order " + l.index + " (" + esc(KINDNAME[l.kind] || l.kind) + ", round " + l.round + ")</div><div>Isaac: " + esc(outcomeText(l.outcome)) + ".</div></div>";
  } else {
    h += '<div class="isaac"><div><span class="pill untested">untested</span></div><div>Isaac did not build this order; only the ' + D.library.length + " library orders were.</div></div>";
  }
  h += '<div id="player"></div>';
  $("result").innerHTML = h;
  $("result").querySelectorAll("button[data-clip]").forEach((b) => b.addEventListener("click", () => {
    const e = res.violated[+b.dataset.clip];
    $("player").innerHTML = '<video controls autoplay muted playsinline poster="' + esc(e.poster) + '" src="' + esc(e.clip) + '"></video>';
  }));
  $("libsel").value = l ? String(l.index) : "";
  document.querySelectorAll("#libtable tr[data-i]").forEach((tr) => tr.classList.toggle("sel", l !== null && +tr.dataset.i === l.index));
  arcs(res.violated);
}
function move(id, before) {
  order.splice(order.indexOf(id), 1);
  order.splice(before === null ? order.length : order.indexOf(before), 0, id);
}
const strip = $("strip");
strip.addEventListener("click", (ev) => {
  const c = ev.target.closest(".chip"); if (!c) return;
  const id = c.dataset.id;
  if (picked === null) picked = id; else { if (picked !== id) move(picked, id); picked = null; }
  render();
});
strip.addEventListener("keydown", (ev) => {
  const c = ev.target.closest(".chip"); if (!c || (ev.key !== "ArrowLeft" && ev.key !== "ArrowRight")) return;
  ev.preventDefault();
  const id = c.dataset.id, i = order.indexOf(id), j = ev.key === "ArrowLeft" ? Math.max(0, i - 1) : Math.min(order.length - 1, i + 1);
  order.splice(i, 1); order.splice(j, 0, id); picked = null; render();
  strip.querySelector('[data-id="' + id + '"]').focus();
});
let dragged = null;
strip.addEventListener("dragstart", (ev) => { const c = ev.target.closest(".chip"); if (!c) return; dragged = c.dataset.id; ev.dataTransfer.effectAllowed = "move"; ev.dataTransfer.setData("text/plain", dragged); });
strip.addEventListener("dragover", (ev) => { const c = ev.target.closest(".chip"); if (!c || !dragged) return; ev.preventDefault(); strip.querySelectorAll(".over").forEach((x) => x.classList.remove("over")); c.classList.add("over"); });
strip.addEventListener("drop", (ev) => { const c = ev.target.closest(".chip"); ev.preventDefault(); if (dragged && c && c.dataset.id !== dragged) move(dragged, c.dataset.id); dragged = null; picked = null; render(); });
strip.addEventListener("dragend", () => { dragged = null; strip.querySelectorAll(".over").forEach((x) => x.classList.remove("over")); });
$("reset").addEventListener("click", () => { order = witness.slice(); picked = null; render(); });
$("random").addEventListener("click", () => {
  const placed = new Set(), out = [], left = witness.slice();
  while (left.length) {
    const ready = left.filter((i) => D.edges.every((e) => e.b !== i || members(e).some((m) => placed.has(m))));
    const i = ready[Math.floor(Math.random() * ready.length)];
    left.splice(left.indexOf(i), 1); placed.add(i); out.push(i);
  }
  order = out; picked = null; render();
});
function libOptions() {
  $("libsel").innerHTML = '<option value="">choose an order</option>' + D.library.map((l) => '<option value="' + l.index + '">' + l.index + " · " + esc(KINDNAME[l.kind] || l.kind) + (l.edge ? " " + esc(sh(l.edge[0]) + "/" + sh(l.edge[1])) : "") + "</option>").join("");
}
$("libsel").addEventListener("change", (ev) => { const l = D.library[+ev.target.value]; if (ev.target.value !== "" && l) { order = l.order.slice(); picked = null; render(); } });

// ---- clip cards
function cards() {
  $("clipsnote").textContent = "Each clip replays the failed swap from the poses Isaac recorded: the later dish goes in first, into the partial load the witness had when the earlier dish went in, then the earlier dish follows. Every drop starts with its descent, one frame per 5 mm step; a descent a placed dish blocks stops there, held in red. A conditional edge adds one clip per dish of its fan: b goes in with that dish placed and a absent. Tinted dishes are the pair and anything it moved; grey dishes are the rest of that partial load. A learned edge has no clip: it comes from a library order that failed, and library builds keep no trajectory.";
  $("cards").innerHTML = D.edges.length ? D.edges.map((e) => '<article class="card"><h3><span class="mono">' + esc(e.any_of ? "{" + members(e).map(sh).join(", ") + "}" : sh(e.a)) + " → " + esc(sh(e.b)) + '</span><span class="pill ' + e.reason + '">' + e.reason + "</span></h3>" + (e.clip ? '<video controls preload="none" playsinline poster="' + esc(e.poster) + '" src="' + esc(e.clip) + '"></video>' : '<p class="empty">' + (e.reason === "learned" ? "No clip: a learned edge comes from a library build, which keeps no trajectory." : "No clip.") + "</p>") + "<p>" + esc(sh(e.a) + " must go in before " + sh(e.b) + ": " + reasonText(e.reason) + ". " + e.detail + ".") + '</p><div class="tags">' + (e.support ? '<span class="pill">support</span>' : "") + (e.learned ? '<span class="pill">seen by the sampler</span>' : "") + (e.reason === "learned" ? '<span class="pill">round ' + e.round + '</span><span class="pill">from order ' + e.from_order + "</span>" : '<span class="pill">step ' + e.step + '</span><span class="pill">after ' + e.placed_before + " placed</span>") + (e.margin !== null && e.margin !== undefined ? '<span class="pill">margin ' + Math.round(e.margin * 100) + "%</span>" : "") + "</div>" + (e.fan.length ? '<div class="fan"><p><b>Conditional.</b> ' + esc(sh(e.b) + " also goes in once " + e.fan.map((f) => sh(f.c)).join(" or ") + " is in, so it needs any one of " + members(e).map(sh).join(", ") + ".") + "</p>" + e.fan.map((f) => f.clip ? '<video controls preload="none" playsinline poster="' + esc(f.poster) + '" src="' + esc(f.clip) + '" aria-label="' + esc(sh(e.b) + " with " + sh(f.c)) + '"></video>' : "").join("") + "</div>" : "") + "</article>").join("") : '<p class="empty">No swap failed: the graph has no edges.</p>';
}

// ---- library table (judged by the refined graph)
function libraryTable() {
  const built = D.library.filter((l) => l.outcome).length, yes = D.library.filter((l) => l.as_predicted).length;
  const L = D.learning || {rounds: []};
  $("libnote").textContent = built + " of " + D.library.length + " orders were built in Isaac with the same move and judged against the built goal's at-goal tolerance. The rule: an order that obeys every edge must land all 24; an order that breaks one must fail somewhere, at the predicted dish or not (both columns are shown). " + yes + " of the " + built + " built did" + (D.refinement ? "; the graph as refined: " + D.refinement.edges.map((r) => shortId(r.a) + " → " + shortId(r.b) + " also holds with " + (r.confirmed.map(shortId).join(" or ") || "nothing")).join("; ") + ", then " + D.refinement.fresh.length + " fresh orders obeying the refined graph" : "") + (L.rounds.length ? "; " + L.rounds.map((r) => "learning round " + r.round + " learned " + (r.edges.map((e) => shortId(e[0]) + " → " + shortId(e[1])).join(", ") || "nothing") + " from order " + r.from.join(", ") + (r.orders ? " and built " + r.orders + " fresh obeying orders" : "")).join("; ") : "") + ". Each order is judged by the graph it was drawn from (round 0 = the swap-tested graph); an obeying order that failed and taught an edge is marked. A violation is built until two dishes past its predicted failure. Load any row into the checker.";
  const head = "<tr><th>#</th><th>Kind</th><th>Round</th><th>Edge reversed</th><th>Predicted at</th><th>Failed at</th><th>Isaac</th><th>As the rule asks</th><th></th></tr>";
  $("libtable").innerHTML = head + D.library.map((l) => {
    const m = l.as_predicted, o = l.outcome;
    const pill = m === null ? '<span class="pill untested">not built</span>' : m ? '<span class="pill yes">yes</span>' : l.taught ? '<span class="pill learned" title="' + esc(l.taught_why || "") + '">no · taught ' + esc(sh(l.taught[0]) + " → " + sh(l.taught[1])) + "</span>" : '<span class="pill no">no</span>';
    const same = o && o.failed_dish && l.predicted ? (o.failed_dish === l.predicted ? " = predicted" : " ≠ predicted") : "";
    return '<tr data-i="' + l.index + '"><td>' + l.index + "</td><td>" + esc(KINDNAME[l.kind] || l.kind) + "</td><td>" + l.round + '</td><td class="mono">' + (l.edge ? esc(sh(l.edge[0]) + " → " + sh(l.edge[1])) : "") + "</td><td>" + (l.predicted ? esc(sh(l.predicted)) + " (pos " + (l.predicted_position + 1) + ")" : "all land") + "</td><td>" + (o ? (o.failed_dish ? esc(sh(o.failed_dish)) + " (pos " + (o.failed_position + 1) + ")" + '<span class="muted">' + same + "</span>" : "none") : "") + '</td><td class="wrap">' + esc(outcomeText(o)) + "</td><td>" + pill + '</td><td><button type="button" data-load="' + l.index + '">Load</button></td></tr>';
  }).join("");
}
$("libtable").addEventListener("click", (ev) => { const b = ev.target.closest("button[data-load]"); if (!b) return; order = D.library[+b.dataset.load].order.slice(); picked = null; render(); $("checker").scrollIntoView({behavior: "smooth"}); });

// ---- rules
function rules() {
  const t = D.thresholds || {};
  const tol = Object.entries(t).map(([k, v]) => k + " " + v.lateral_mm + " mm lateral / " + v.dz_mm + " mm height / " + v.tilt_deg + "°").join("; ");
  $("ruleslist").innerHTML = [
    "Move (every drop on this page: the sampler, both swap steps, the confirmations, the library): a plate stands upright in its slot, 3 mm toward the side its packed pose leans to; a bowl or cup keeps its packed pose. The dish starts with its lowest point 20 mm above its rack's rim (its top kept under the tub ceiling) and goes down in 5 mm steps, each checked against the placed dishes and the appliance with the 1 mm allowance. A placed dish in the way blocks the move; the rack alone just stops it. It is let go 20 mm above the lowest clear step and settles 150 + 60 ticks at 120 Hz.",
    D.witness.plate_order === "lean-side" ? "Sampler: a random order of all 24 dishes in which each plate bank's positions are refilled by its plates in chain order: the slot furthest on the bank's lean side first, then outward, so each plate leans onto the one before (a plate leaning about 30° hangs over the next three or four slots on that side). A dish lands when it ends within 80 mm of its pose with contact under 2 mm and moves no placed dish out of its tolerance. A bowl or cup that does not go in is undone and tried at up to five other free poses of its kind whose descent is clear, else it moves to the back of the order; a plate is tried only at its bank's next free slots along the chain, else it moves after the plates still waiting. The load fails only when no dish left can go in." : "Sampler: a random order per packing. A dish lands when it ends within 80 mm of its pose with contact under 2 mm and moves no placed dish out of its tolerance. A dish that does not go in is undone and tried at up to five other free poses of its kind from the packing's candidate list whose descent is clear (one per slot for plates); if none goes in, it moves to the back of the order. The load fails only when no dish left can go in.",
    "Candidates: every pair whose collision pieces come within 20 mm at the goal (" + D.swaps.near + " pairs), plus pairs the sampler saw block or knock each other (" + D.swaps.learned + ").",
    "Swap: for a pair placed a then b by the witness, the partial load the witness had when a went in is restored (those dishes at their recorded poses, the rest parked, 150 ticks), b goes in, then a. Need: b does not land without a. Block: b's descent is blocked, or a cannot go in once b is there. Knock: a drop moves a placed dish out of its tolerance (or more than 10 mm / 20° if it was not at its goal).",
    "Landing in the swaps, confirmations and library (" + (D.tolerance.name === "library" ? "the at-goal tolerance doubled" : "the bench's at-goal tolerance") + "): within " + tol + " of the built pose, contact under " + (D.tolerance.text.split("contact under ")[1] || "2 mm") + "; a placed dish at its goal is disturbed only when it ends outside that tolerance (one not at its goal: moved more than 10 mm / 20°). A drop within 20% of a threshold is rerun under three releases perturbed by 3 mm / 3°, and the majority decides.",
    D.sampler.backtrack && !D.sampler.backtracks.length ? "Backtracking: allowed when this sample was built (up to " + D.sampler.backtrack + " blockers taken out and put back), but no dish needed it, so the witness order is exactly how the dishes went in." : D.sampler.backtrack ? "Backtracking: a dish that went in nowhere (its own pose and up to five re-poses) names the placed dishes that blocked its descent or that it moved. The three most recently placed of them are taken out (parked, the others put back where they were, 60 ticks), the dish goes in, and they go back in their order at the poses they had; one that no longer fits waits its turn later in the order. If taking them out moves a dish left in, or the dish still does not go in, everything is put back. At most two backtracks per dish; the load fails only when every dish left failed since the last placement." : "No backtracking: a dish that goes in nowhere (its own pose and up to five re-poses) waits until later in the order; the load fails only when every dish left failed since the last placement, so the witness order is exactly how the dishes went in.",
    "Library: the witness, each edge's violating order (b moved to just before a), 10 random orders that obey every edge and 10 random orders that break one edge. The rule: an obeying order must land all 24; a violating order must fail somewhere. The graph predicts the dish it fails at (b for a step-1 edge, a for a step-2 edge); the dish it really failed at is shown beside it and need not match, since a cascade in a new context is physics, not a wrong edge.",
    "Learning: an obeying order that fails at a dish X teaches one edge from its build: X before the dish it moved or that blocked it, else a dish the witness placed before X and this order had not goes before it (what X rests on in the witness first, then the nearest), else X before the dish placed just before it in that order. An edge against the witness order would make the order that built the goal a violation, so it is dropped and the next candidate taken. Then 10 fresh orders obeying the graph are built; up to three rounds, and the last one must pass. Earlier orders stay judged by the graph they were drawn from.",
    ...(D.rejudged ? ["Re-judged: this sample was built and swap-tested by the previous run under the " + D.rejudged.from_tolerance + " tolerance (verdict " + (D.rejudged.verdict_before || "none") + "). Its saved swap drops were judged again under the tolerance above, Kit-free: " + D.rejudged.edges_before + " swap edges became " + D.rejudged.edges_after + (D.rejudged.dropped.length ? " (dropped: " + D.rejudged.dropped.map((e) => shortId(e[0]) + " → " + shortId(e[1]) + " " + e[2]).join(", ") + ")" : "") + (D.rejudged.untested_step2.length ? "; " + D.rejudged.untested_step2.map((p) => shortId(p[0]) + "/" + shortId(p[1])).join(", ") + " passed step 1 now, and its step 2 was never dropped, so the pair has no edge and the library tests it" : "") + (D.rejudged.marginal_unrerun.length ? "; " + D.rejudged.marginal_unrerun.length + " drop(s) lie within 20% of a new threshold with no saved rerun, judged by the single drop" : "") + ". Its " + D.rejudged.learned_dropped + " learned edges and the old library were dropped; the library and the learning loop were built again in Isaac."] : []),
    "Refinement: when a violating order still builds, its edge becomes conditional. Every dish placed before b in that order but missing from the swap is tried once: b dropped into the swap's partial load with that dish added and a removed (a block edge then drops a as well). The dishes that pass join the edge's fan, and 10 fresh orders obeying the refined graph are built once more.",
    D.swaps.invalid_context ? D.swaps.invalid_context + " swap(s) ran in a partial load where a restored dish fell during the re-settle; their verdicts are kept." : "Every swap's partial load re-settled without a dish falling.",
  ].map((s) => "<li>" + esc(s) + "</li>").join("");
}

function setSample(k) {
  current = k;
  D = ALL.samples[k];
  dish = Object.fromEntries(D.dishes.map((d) => [d.id, d]));
  witness = D.witness.order;
  order = witness.slice();
  picked = null;
  document.querySelectorAll("video").forEach((v) => v.pause());
  selector();
  tuple();
  libOptions();
  cards();
  libraryTable();
  rules();
  render();
  try { history.replaceState(null, "", "#" + D.key); } catch (e) { /* sandboxed */ }
}
// ---- no valid sample: why the sampler stopped
function failureView() {
  const F = ALL.failure;
  if (!F) return;
  $("nosample").hidden = false;
  const n = F.packings, lo = F.placed[0], hi = F.placed[F.placed.length - 1], med = F.placed[Math.floor(F.placed.length / 2)];
  if (ALL.samples.length) {
    $("nstitle").textContent = "Why the sampler stopped";
    const nv = ALL.samples.filter((s) => s.valid === true).length;
    if (!nv) $("lead").textContent += " No sample is valid: " + n + " packings stopped in the sampler at " + lo + "–" + hi + " of 24 dishes (median " + med + "), and the " + ALL.samples.length + " that built all 24 have graphs their library did not confirm; " + (ALL.samples.length === 2 ? "both are" : "they are") + " shown below, labelled invalid.";
  } else $("lead").textContent = "No sample is valid. In all " + n + " packings the re-posing sampler placed " + lo + "–" + hi + " of the 24 dishes (median " + med + ") and then had a dish it could not put in at its own pose, at any of up to five other free poses, or later in the order. No load was built, so no packing reached the swap tests or the library.";
  $("nsnote").textContent = F.note || "A plate lowered upright stops on the tine tips and, let go 20 mm higher, drops into its slot and leans about 30° toward its lean side. Leaning that far, its upper half hangs over the next three or four slots, so a plate whose slot is on that side can no longer be lowered from above once the leaning plate is in. A random order puts some plate in before its lean-side neighbours in almost every packing; waiting does not help, because a placed dish never moves away, and re-posing finds no free slot whose descent is clear once a few plates lean across the bank.";
  if (F.clip) {
    $("nsvideo").innerHTML = '<video controls preload="metadata" playsinline poster="' + esc(F.clip.poster) + '" src="' + esc(F.clip.file) + '"></video>';
    $("nscaption").textContent = "The last sampler build of " + F.clip.load + " (" + F.clip.seconds + " s): every dish that went in, in the order it went in; each drop starts with its descent, one frame per 5 mm step. The counter's total counts the dishes that went in plus the one that stopped the build; the other dishes left out are not counted.";
  }
  const S = F.screen, fc = F.faces, C = F.calls;
  const rows = [
    ["Placed per packing", F.placed.join(", ") + " of 24"],
    ["Left out", Object.entries(F.left).map(([k, v]) => v + " " + k + "s").join(", ") + (F.left.cup ? "" : " (no cup)")],
    ["Their last try", F.why.map(([k, v]) => v + " × " + k).join("; ")],
    ["Re-posing", Object.entries(C).map(([k, v]) => k + "s " + v.calls + " calls, " + v.none + " with no free pose whose descent is clear").join("; ")],
    ["Placed plates", fc.n + " plates: " + fc.lean + " leaned 15–40° from vertical, " + fc.upright + " stayed within 15° of upright, " + fc.over + " tipped further; they came to rest " + F.plate_dev_mm[0] + "–" + F.plate_dev_mm[2] + " mm from their packed poses (median " + F.plate_dev_mm[1] + ")"],
  ];
  const bk = F.backtracks || {};
  if (Object.keys(bk).length) rows.push(["Backtracks", Object.values(bk).reduce((a, b) => a + b, 0) + " in these packings: " + (bk.placed || 0) + " put their dish in, " + (bk.failed || 0) + " found it still blocked, " + (bk.abandoned || 0) + " abandoned because taking the blockers out moved another dish"]);
  if (F.screen_text) rows.push(["Screen, before Isaac", F.screen_text + "."]);
  if (S) rows.push(["Screen, before Isaac", S.packings + " packings, appliance only: no dish blocked at its start; every plate stops " + S.kinds.plate.stop_mm[0] + "–" + S.kinds.plate.stop_mm[2] + " mm above its upright pose; " + S.kinds.bowl.reach + " of " + S.kinds.bowl.n + " bowls and all " + S.kinds.cup.n + " cups reach their packed pose. In the " + (S.order === "chain" ? "lean-side order" : "first random order") + " with earlier dishes at their packed poses, " + S.kinds.plate.first_start + " plates and " + S.kinds.bowl.first_start + " bowls are blocked at their start by a placed dish, " + S.kinds.plate.first_dish + " and " + S.kinds.bowl.first_dish + " lower down."]);
  $("nsfacts").innerHTML = rows.map(([t, d]) => "<div><dt>" + esc(t) + "</dt><dd>" + esc(d) + "</dd></div>").join("");
}

lead();
yieldTable();
if (ALL.samples.length) {
  const start = ALL.samples.findIndex((s) => "#" + s.key === location.hash);
  setSample(start >= 0 ? start : 0);
  window.addEventListener("hashchange", () => { const k = ALL.samples.findIndex((s) => "#" + s.key === location.hash); if (k >= 0 && k !== current) setSample(k); });
} else {
  ["tuple", "clips", "checker", "library", "rules", "samplebar"].forEach((id) => { $(id).hidden = true; });
}
failureView();
</script>
"""

if __name__ == "__main__":
    main()
