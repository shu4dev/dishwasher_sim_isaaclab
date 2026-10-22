#!/usr/bin/env python3
"""The "Dependency graphs" page of the physical-cause samples (plans/2026-10-16-physical-cause-graph.md; the labelled views
and the graph variants, plans/2026-10-16-labelled-views-overlap-rule.md; the kind-ceiling score,
plans/2026-10-21-kind-ceiling-score.md): one HTML file and its media (Kit-free, standard library only; rsvg-convert draws
the graph PNGs).

Per valid sample, behind a sample selector: the load from four angles (its S and the joint-gate numbers in the caption),
the witness rebuilt in a fresh Isaac process (video), and the prerequisite arrows along the witness order and layered as
a DAG, each in three variants behind a toggle (direct, reduction, closure); then every packing the run tried, from the
--sample scheduler's state, and how it was measured.

    python3 code/planner/frigidaire/frigidaire_dependency_page.py --records r05_00027=data/results/dot_rules/dependency/samples_v5/r05_00027.json --clips r05_00027=data/media/dot_rules/dependency_samples_v6/r05_00027 --state data/logs/dot_rules/samples_v6b/scheduler_state_merged.json --out data/media/dot_rules/dependency_page_kind

Writes <out>/index.html and <out>/media/<key>_<file> (the load still, the witness video and poster named in
<clips>/clips.json, six graph PNGs); a <load_id>.scores.json beside a record (frigidaire_dependency_score.py) adds the
absolute S, a <load_id>.relative.json (frigidaire_dot_rules_relative.py) the kind-ceiling S; prints [RESULT].
"""
import argparse
import json
from pathlib import Path
import shutil

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
    frigidaire_dependency_score.py wrote <load_id>.scores.json beside the record, else None; with the kind-ceiling score
    ("kind": S, S_area, S_by_kind, n_pooling of frigidaire_dot_rules_relative.py, 2026-10-21) and the ceilings when
    <load_id>.relative.json beside it has those."""
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
        if out[key] and "S" in (states.get(state) or {}):
            out[key]["kind"] = {k: states[state][k] for k in ("S", "S_area", "S_by_kind", "n_pooling")}
    out["ceilings"] = rr.get("ceilings_kind")
    return out


# --------------------------------------------------------------------------- the page (plans/2026-10-16-physical-cause-graph.md)
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
    """The page (module comment above): the claim, the legend, per valid sample its four items behind a selector, every
    packing of the run and how it ended, how it was measured."""
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
    text = PHYSICAL_HEAD.replace("__STYLE__", STYLE + PHYSICAL_STYLE) + body + PHYSICAL_SCRIPT
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
                    help="one valid (certified) frigidaire_dependency_kit.py record per sample (page order)")
    ap.add_argument("--clips", nargs="*", default=[], metavar="KEY=DIR",
                    help="frigidaire_dependency_clips.py --out (with clips.json) per key")
    ap.add_argument("--state", type=Path, help="the --sample scheduler's state (scheduler_state.json): the yield")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    records, clip_dirs = keyed(a.records, "--records"), keyed(a.clips, "--clips")
    state = json.loads(a.state.read_text()) if a.state and a.state.is_file() else None
    return physical_page(a, records, clip_dirs, state)


# The page's CSS, inherited from the retired swap-test page: the published page carries it byte for byte (its
# unused rules included), and PHYSICAL_STYLE follows it.
STYLE = r"""
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
"""

if __name__ == "__main__":
    main()
