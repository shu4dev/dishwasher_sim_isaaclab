#!/usr/bin/env python3
"""Page of the limits round (plans/2026-10-23-hardware-limits-mouth-up.md): the grasp gate (G3) per object, the
reach sweep (G0-G2) as rack maps, the placement trials (G4), the Warp replays. Standard library only (runs on the Mac).

    python3 code/execution/frigidaire/frigidaire_robot_limits_page.py --notes data/media/robot/limits_v1/notes.json
"""
from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RES = ROOT / "data/results/robot/limits_v1"
OUT = ROOT / "data/media/robot/limits_v1"
GATES = ("none", "G0", "G1", "G2")
GATE_COLOR = {"none": "var(--bad)", "G0": "var(--warn)", "G1": "var(--mid)", "G2": "var(--good)"}
RACK_BOX = {"LowerRack": ((-.28, .28), (-.30, .30)), "UpperRack": ((-.26, .26), (-.29, .29))}


def rows(path):
    p = Path(path)
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.is_file() else []


def cp(k, n, a=.05):
    """Clopper-Pearson interval by bisection on the binomial tail (stdlib)."""
    if n == 0:
        return (0., 1.)

    def tail_ge(p):
        return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k, n + 1))

    def tail_le(p):
        return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(0, k + 1))

    def solve(f, target, increasing):
        lo, hi = 0., 1.
        for _ in range(60):
            mid = (lo + hi) / 2
            if (f(mid) < target) == increasing:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2
    lower = 0. if k == 0 else solve(tail_ge, a / 2, True)
    upper = 1. if k == n else solve(tail_le, a / 2, False)
    return lower, upper


def med(v):
    v = sorted(x for x in v if x is not None)
    return None if not v else v[len(v) // 2] if len(v) % 2 else (v[len(v) // 2 - 1] + v[len(v) // 2]) / 2


def fmt(x, d=1):
    return "–" if x is None else f"{x:.{d}f}"


def gate_table(runs):
    out = ["<table><thead><tr><th>object</th><th>grasp</th><th>racks</th><th>G3 held k/n</th><th>95 % CI</th><th>outcomes</th>"
           "<th>jaw rad (median)</th><th>pivot ° (median / max)</th><th>5 s drift mm (max)</th><th>pad N max (median L / R)</th>"
           "<th>invariants</th><th>clean trials</th></tr></thead><tbody>"]
    for label, rs in runs:
        if not rs:
            out.append(f"<tr><td>{label}</td><td colspan=11>not run</td></tr>")
            continue
        n, k = len(rs), sum(r["g3"] for r in rs)
        lo, hi = cp(k, n)
        oc = {}
        for r in rs:
            oc[r["outcome"]] = oc.get(r["outcome"], 0) + 1
        holds = [r["hold"] for r in rs if r.get("hold")]
        inv = {}
        for r in rs:
            for i in r["invariants"]:
                inv[i] = inv.get(i, 0) + 1
        padded = [r for r in rs if r.get("g3") and max(r["pad_N_max"].values()) > 0]
        pl = med([r["pad_N_max"].get("pad_L") for r in padded])
        pr = med([r["pad_N_max"].get("pad_R") for r in padded])
        out.append(
            f"<tr><td><b>{label}</b></td><td>rim pinch{' (approach not collision-checked: the 15 mm clearance rejects every plate approach)' if rs[0].get('waive_dish_clearance') else ''}</td>"
            f"<td>{html.escape(rs[0].get('racks', ''))}</td><td><b>{k}/{n}</b></td><td>{lo:.2f}–{hi:.2f}</td>"
            f"<td>{', '.join(f'{o} {c}' for o, c in sorted(oc.items()))}</td>"
            f"<td>{fmt(med([h.get('jaw_rad') for h in holds]), 3)}</td>"
            f"<td>{fmt(med([h.get('settle_rotation_deg') for h in holds]))} / {fmt(max([h.get('settle_rotation_deg') or 0 for h in holds], default=None))}</td>"
            f"<td>{fmt(max([h.get('drift_mm') or 0 for h in holds if h.get('held')], default=None), 2)}</td>"
            f"<td>{fmt(pl)} / {fmt(pr)} <span class=muted>({len(padded)} of {k} held trials report pad force)</span></td><td>{', '.join(f'{i} {c}/{n}' for i, c in sorted(inv.items())) or 'none'}</td>"
            f"<td>{sum(r['counted'] for r in rs)}/{n}</td></tr>")
    out.append("</tbody></table>")
    return "\n".join(out)


def reach_summary(reach, rr):
    """{(kind, target): {site: {"gate", "n", "g0", "g1", "g2", "branch", "why"}}}"""
    order = {g: i for i, g in enumerate(GATES)}
    out = {}
    for r in rr:
        cell = out.setdefault((r["kind"], r["target"], r.get("waived", False)), {}).setdefault(
            r["site"], {"gate": "none", "n": 0, "G0": 0, "G1": 0, "G2": 0, "branch": None, "why": {}, "rack": r["rack"]})
        cell["n"] += 1
        for g in ("G0", "G1", "G2"):
            cell[g] += order[r["gate"]] >= order[g]
        if order[r["gate"]] > order[cell["gate"]]:
            cell["gate"] = r["gate"]
        if r["gate"] == "G2":
            cell["branch"] = r.get("branch")
        if r.get("why"):
            w = r["why"].split(" (")[0].split(":")[0] + ": " + r["why"].split(": ", 1)[-1].split(" (")[0][:60]
            cell["why"][w] = cell["why"].get(w, 0) + 1
    return out


def rack_svg(kind, target, cells, sites, rack, title):
    (x0, x1), (y0, y1) = RACK_BOX[rack]
    W, H, pad = 300, 320, 14
    sx = lambda x: pad + (x - x0) / (x1 - x0) * (W - 2 * pad)
    sy = lambda y: pad + (y1 - y) / (y1 - y0) * (H - 2 * pad)        # rack +y (rear) up
    parts = [f'<svg viewBox="0 0 {W} {H + 26}" class="rack" role="img" aria-label="{html.escape(title)}">',
             f'<rect x="{pad}" y="{pad}" width="{W - 2 * pad}" height="{H - 2 * pad}" rx="6" class="rackbox"/>',
             f'<text x="{W / 2}" y="{H + 18}" text-anchor="middle" class="cap">{html.escape(title)} · front (door) at the bottom</text>']
    for s in sites:
        if s["rack"] != rack:
            continue
        c = cells.get(s["site"])
        if c is None:
            continue
        x, y = s["local"]["position_m"][0], s["local"]["position_m"][1]
        tip = f'{s["site"]}: best gate {c["gate"]}; yaws passing G0 {c["G0"]}/{c["n"]}, G1 {c["G1"]}/{c["n"]}, G2 {c["G2"]}/{c["n"]}' + \
              (f'; branch {c["branch"]}' if c["branch"] else "") + "; " + "; ".join(f"{k} ×{v}" for k, v in c["why"].items())
        parts.append(f'<g><title>{html.escape(tip)}</title><circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="9" fill="{GATE_COLOR[c["gate"]]}" '
                     f'stroke="var(--fg)" stroke-width=".6"/><text x="{sx(x):.1f}" y="{sy(y) + 3.5:.1f}" text-anchor="middle" '
                     f'class="lab">{c["G2"] if c["gate"] == "G2" else c["G1"] if c["gate"] == "G1" else ""}</text></g>')
    parts.append("</svg>")
    return "".join(parts)


def reach_block(reach, summ):
    if not reach:
        return "<p>The reach sweep did not run.</p>"
    out = []
    labels = {"family": "the packer's pose at the site", "level": "the dish as held (mouth up, level) at the site"}
    for (kind, target, waived), cells in sorted(summ.items(), key=lambda kv: (["bowl", "cup", "plate"].index(kv[0][0]), kv[0][1], kv[0][2])):
        sites = reach["sites"][kind]
        n_site = len(cells)
        cnt = {g: sum(c["gate"] == g for c in cells.values()) for g in GATES}
        reasons = {}
        for c in cells.values():
            for k, v in c["why"].items():
                reasons[k] = reasons.get(k, 0) + v
        top = sorted(reasons.items(), key=lambda kv: -kv[1])[:3]
        racks = sorted({c["rack"] for c in cells.values()})
        svgs = "".join(rack_svg(kind, target, cells, sites, r, f"{kind} · {target} · {r}") for r in racks)
        jobs = sum(c["n"] for c in cells.values())
        out.append(f"<h3>{kind} · {labels[target]}{' · counter-grasp clearance waived' if waived else ''}</h3>"
                   f"<p>{n_site} sites × up to 12 yaws ({jobs} cells done). Best gate per site: "
                   f"{', '.join(f'{g} {cnt[g]}' for g in GATES)}. Most common stops: "
                   f"{'; '.join(f'{html.escape(k)} ×{v}' for k, v in top) or '–'}.</p><div class='racks'>{svgs}</div>")
    return "\n".join(out)


def diag_block():
    p = RES / "r2_place_bowl_down" / "diag_hover_compare.json"
    if not p.is_file():
        return ""
    d = json.loads(p.read_text())
    rows_ = "".join(
        f"<tr><td>{float(hv) * 1e3:.0f} mm</td>"
        f"<td>{sum(r['isaac']['approach_ok'] for r in rs)}/{len(rs)}</td><td>{sum(r['kitfree']['approach_ok'] for r in rs)}/{len(rs)}</td>"
        f"<td>{sum(r['isaac']['free_hover'] == r['kitfree']['free_hover'] and r['isaac']['ik'] == r['kitfree']['ik'] and r['isaac']['approach_ok'] == r['kitfree']['approach_ok'] for r in rs)}/{len(rs)}</td>"
        f"<td>{', '.join(str(r['isaac']['free_hover']) for r in rs)}</td></tr>"
        for hv, rs in d["hovers"].items())
    xd = max(d["X_link_maxabs_diff"].values())
    sp = RES / "r2_place_bowl_down" / "diag_hover_seed.json"
    seed = json.loads(sp.read_text()) if sp.is_file() else {}
    seed_txt = "; ".join(f"{k.split()[0]} seed at {float(k.split()[1]) * 1e3:.0f} mm: {v['approach_ok_yaws']}/12 "
                         f"({min(v['ik_per_yaw'])}–{max(v['ik_per_yaw'])} IK solutions per grasp)" for k, v in seed.items())
    return (f"<h3>Round 2 diagnostic · the Isaac-versus-Kit-free hover disagreement</h3><p>Same bowl (mouth up on the counter), "
            f"same IK seed, same 12 rim grasps, three approach hovers. Isaac's model (live colliders) and the Kit-free model "
            f"(<code>arm_model.json</code>) agree in every cell, and their link calibrations differ by at most {xd:.1e}. The "
            f"round-1 gap was the <b>IK seed</b>: <code>ik_all</code> unwraps each solution toward the seed and then drops anything "
            f"outside the joint limits, so the solution set depends on the seed. Isaac seeds its current joints, while the sweep "
            f"seeds HOME_Q. Measured on the Kit-free model: {html.escape(seed_txt) or 'pending'}. Isaac's pick therefore loses "
            f"the wrist-up approach above 0.10 m. The sweep's G0–G2 counts are HOME_Q-seeded numbers (expanding the 2π wraps, "
            f"<code>kin.expand_2pi_wraps</code>, would remove the dependence).</p><table><thead><tr><th>hover</th>"
            f"<th>Isaac: yaws with a free approach</th><th>Kit-free, same seed</th><th>cells identical (IK, free hover IK, "
            f"approach)</th><th>free hover IK per yaw (Isaac)</th></tr></thead><tbody>{rows_}</tbody></table>")


SITE_CLASS = (("upper_bowl", "upper slots"), ("lower_rearright", "lower rear-right"), ("lower_rear@", "lower rear"),
              ("lower_frontright", "lower front-right"), ("lower_mid", "lower middle"), ("glass_left", "glass left"),
              ("glass_right", "glass right"))


def site_class(site):
    return next((c for k, c in SITE_CLASS if site.startswith(k)), site)


def tilt_cause(lst):
    """Why an Isaac tilt cell failed (the commonest stop over its trials)."""
    c = {}
    for r in lst:
        if r.get("g4"):
            continue
        if not r.get("g3"):
            w = "no pick: " + (r.get("why") or "")[:40]
        elif r.get("place_why"):
            pw = r["place_why"]
            w = "dropped in transit" if "dropped" in pw else "path / IK at the hover" if ("hover" in pw or "path" in pw) else pw.split(" {")[0][:50]
        else:
            d = (r.get("judge") or {}).get("distance") or [0, 0, 0]
            st = ((r.get("judge") or {}).get("settle") or {})
            w = ("tilt over tolerance" if d[2] > 15 else "not at goal" if (d[0] > 15 or d[1] > 20) else
                 "settle rule" if not st.get("passed") else "not at goal")
        c[w] = c.get(w, 0) + 1
    return max(c.items(), key=lambda kv: kv[1])[0] if c else None


def orient_block():
    p = RES / "orient.json"
    if not p.is_file():
        return ""
    d = json.loads(p.read_text())
    tilts = [int(t) for t in d["tilts"]]
    isaac, stops = {}, {}
    for run in sorted(RES.glob("r3_tilt_*")):
        kind = run.name.split("_")[2]
        for r in rows(run / "trials.jsonl"):
            isaac.setdefault((r["kind"], r["site"], int(r["tilt_deg"])), []).append(r)
        for st in rows(run / "tilt_stops.jsonl"):
            stops[(kind, st["site"])] = st
    out = [f"<p>Kit-free gates per site and target tilt (12 yaws each; done {d.get('done')} of {d.get('of')} cells) and, where Isaac ran, "
           f"its G4 count with the contact release at the target tilt (n = 5 per tilt; every G2 tilt in order, where time allowed). "
           f"In-hand poses as measured after the lift: bowl pivot {d['measured_M']['bowl']['pivot_deg']}°, cup "
           f"{d['measured_M']['cup']['pivot_deg']}°.</p>"]
    summary = []
    for kind in ("bowl", "cup"):
        cells = {(c["site"], int(c["tilt_deg"])): c for c in d["cells"].get(kind, [])}
        sites = sorted({s for s, _ in cells}, key=lambda s: (site_class(s), s))
        head = "".join(f"<th>{t}°</th>" for t in tilts)
        body = []
        for site in sites:
            tds, placed, first_fail = [], None, None
            for t in tilts:
                c = cells.get((site, t))
                if c is None:
                    tds.append("<td>–</td>")
                    continue
                lst = isaac.get((kind, site, t), [])
                k = sum(bool(r.get("g4")) for r in lst)
                if lst and k:
                    placed = t
                if first_fail is None and lst and k == 0:
                    first_fail = (t, tilt_cause(lst))
                txt = f"{c['gate']}{' ' + str(c['G2']) + '/' + str(c['n']) if c['gate'] == 'G2' else ''}"
                if lst:
                    txt += f"<br><b>Isaac {k}/{len(lst)}</b>"
                tip = f"{site} {t}°: G0 {c['G0']}/{c['n']}, G1 {c['G1']}/{c['n']}, G2 {c['G2']}/{c['n']}; stop: {c.get('why')}"
                tds.append(f"<td title='{html.escape(tip)}' style='background:color-mix(in srgb, {GATE_COLOR[c['gate']]} 28%, transparent)'>{txt}</td>")
            ran = any(isaac.get((kind, site, t)) for t in tilts)
            first_fail = None
            for t in tilts:                            # the first tilt that fails, in Isaac or already Kit-free
                c, lst = cells.get((site, t)), isaac.get((kind, site, t), [])
                if lst and not any(r.get("g4") for r in lst):
                    first_fail = (t, f"Isaac 0/{len(lst)}: {tilt_cause(lst)}")
                    break
                if ran and c is not None and c["gate"] != "G2" and not lst:
                    first_fail = (t, f"Kit-free {c['gate']}: {(c.get('why') or '').split(' (')[0]}")
                    break
            summ = (f"{'–' if not ran else (str(placed) + '°' if placed is not None else 'none')}"
                    f"{'' if not first_fail else ' · first fail ' + str(first_fail[0]) + '° (' + html.escape(str(first_fail[1])) + ')'}")
            if ran:
                summary.append((kind, site_class(site), site, placed, first_fail))
            body.append(f"<tr><td>{html.escape(site)}</td><td>{site_class(site)}</td>{''.join(tds)}<td>{summ}</td></tr>")
        out.append(f"<h3>{kind}</h3><table><thead><tr><th>site</th><th>class</th>{head}<th>Isaac: largest tilt placed · first failing tilt "
                   f"(cause)</th></tr></thead><tbody>{''.join(body)}</tbody></table>")
    return "\n".join(out), summary


def gallery(items):
    if not items:
        return "<p>none rendered</p>"
    return "<div class=vids>" + "".join(
        f"<figure><{'video controls muted playsinline preload=metadata' if v['file'].endswith('.mp4') else 'img alt=\"' + html.escape(v['caption']) + '\"'} "
        f"src=\"{html.escape(v['file'])}\">{'</video>' if v['file'].endswith('.mp4') else ''}<figcaption>{html.escape(v['caption'])}</figcaption></figure>"
        for v in items) + "</div>"


def place_why(r):
    if not r.get("g3"):
        return "pick: " + (r.get("why") or "")[:60]
    if r.get("place_why"):
        return "place: " + r["place_why"].split(" {")[0][:70]
    j = r.get("judge") or {}
    st = j.get("settle") or {}
    out = []
    if not st.get("passed"):
        out.append(f"settle rule (contact depth {st.get('contact_depth_mm')} mm, drift {st.get('drift_mm')} mm, "
                   f"dev {st.get('settle_dev_mm')} mm)")
    if not j.get("at_target"):
        out.append(f"off the target (lateral, dz mm, tilt deg) {j.get('distance')}")
    return "; ".join(out)


def place_table(rs, racked, release=None):
    rs = [r for r in rs if release is None or r.get("release", "air") == release]
    if not rs:
        return "<p>No placement trials of this kind ran.</p>"
    by = {}
    for r in rs:
        by.setdefault((r["kind"], r["site"], r.get("target", "family") + (" (detector fixed)" if r.get("run") == "r3_cup_right_fix" else ""),
                       r.get("release", "air")), []).append(r)
    out = ["<table><thead><tr><th>object</th><th>site</th><th>target</th><th>racks</th><th>n</th><th>released</th>"
           "<th>in the intended rack</th><th>at target</th><th>settle rule</th><th>G4 k/n</th><th>95 % CI</th>"
           "<th>round 1 · 2 · 3</th><th>stops (count)</th></tr></thead><tbody>"]
    for (kind, site, target, rel_), lst in sorted(by.items()):
        rel = [r for r in lst if r.get("g3") and not r.get("place_why")]
        k = sum(bool(r.get("g4")) for r in lst)
        lo, hi = cp(k, len(lst))
        r1 = [r for r in lst if not r.get("run", "").startswith(("r2_", "r3_"))]
        r2 = [r for r in lst if r.get("run", "").startswith("r2_")]
        r3 = [r for r in lst if r.get("run", "").startswith("r3_")]
        stops = {}
        for r in lst:
            w = place_why(r)
            if w:
                w = w.split(" (")[0].split(" [")[0]
                stops[w] = stops.get(w, 0) + 1
        out.append(f"<tr><td>{kind}</td><td>{html.escape(site)}</td><td>{target}{', lowered until contact' if rel_ == 'down' else ''}</td>"
                   f"<td>{html.escape(lst[0].get('racks', ''))}</td><td>{len(lst)}</td><td>{len(rel)}</td>"
                   f"<td>{sum((racked.get(r.get('traj')) or {}).get('racked_in') == r['rack'] for r in rel)}</td>"
                   f"<td>{sum(bool((r.get('judge') or {}).get('at_target')) for r in rel)}</td>"
                   f"<td>{sum(bool(((r.get('judge') or {}).get('settle') or {}).get('passed')) for r in rel)}</td>"
                   f"<td><b>{k}/{len(lst)}</b></td><td>{lo:.2f}–{hi:.2f}</td>"
                   f"<td>{sum(bool(r.get('g4')) for r in r1)}/{len(r1)} · {sum(bool(r.get('g4')) for r in r2)}/{len(r2)} · "
                   f"{sum(bool(r.get('g4')) for r in r3)}/{len(r3)}</td>"
                   f"<td>{'; '.join(f'{html.escape(w)} ×{c}' for w, c in sorted(stops.items(), key=lambda kv: -kv[1])) or '–'}</td></tr>")
    out.append("</tbody></table>")
    return "\n".join(out)


CSS = """
:root{--bg:#f7f8fa;--card:#fff;--fg:#17202a;--muted:#5b6876;--line:#d9dee5;--good:#2e9d5b;--mid:#9cc95a;--warn:#e9a23b;--bad:#d1544b;--accent:#2563eb}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#0f141a;--card:#161d25;--fg:#e6edf3;--muted:#9aa7b4;--line:#2a3440;--good:#3fb871;--mid:#a6cf63;--warn:#f0b04f;--bad:#e2685f;--accent:#6ea8ff}}
:root[data-theme="dark"]{--bg:#0f141a;--card:#161d25;--fg:#e6edf3;--muted:#9aa7b4;--line:#2a3440;--good:#3fb871;--mid:#a6cf63;--warn:#f0b04f;--bad:#e2685f;--accent:#6ea8ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:1100px;margin:0 auto;padding:20px 16px 60px}h1{font-size:1.6rem;margin:.2em 0}h2{margin-top:1.8em;border-bottom:1px solid var(--line);padding-bottom:.2em}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 16px;margin:12px 0}
.muted{color:var(--muted)}table{border-collapse:collapse;width:100%;font-size:13px;display:block;overflow-x:auto}
th,td{border-bottom:1px solid var(--line);padding:5px 7px;text-align:left;vertical-align:top}th{color:var(--muted);font-weight:600}
.racks{display:flex;flex-wrap:wrap;gap:10px}.rack{width:300px;max-width:100%;height:auto}
.rackbox{fill:none;stroke:var(--muted);stroke-dasharray:4 3}.cap{fill:var(--muted);font-size:11px}.lab{fill:#111;font-size:9px;font-weight:700}
.legend span{display:inline-block;margin-right:12px}.dot{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:4px;vertical-align:-1px}
video,img{max-width:100%;border-radius:8px;border:1px solid var(--line)}.vids{display:grid;grid-template-columns:1fr;gap:14px}
code{font-size:12.5px}ul{padding-left:1.2em}
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--notes", type=Path, required=True, help="JSON: conclusions, not_done, videos [{file, caption}], litreview, window")
    ap.add_argument("--out", type=Path, default=OUT / "index.html")
    a = ap.parse_args()
    notes = json.loads(a.notes.read_text())
    runs = [(k, rows(RES / f"gate2_{k}" / "trials.jsonl")) for k in ("bowl", "cup", "plate")]
    slide = rows(RES / "r3_plate_slide" / "trials.jsonl")
    if slide:
        runs.append(("plate (surface slide, round 3)", slide))
    reach_p = RES / "reach.json"
    reach = json.loads(reach_p.read_text()) if reach_p.is_file() else None
    summ = reach_summary(reach, rows(RES / "reach.jsonl") + rows(RES / "reach_waived.jsonl")) if reach else {}
    place = []
    for d in sorted(p for p in RES.glob("*place_*") if p.is_dir() and not p.name.startswith("_")):
        place += [dict(r, run=d.name) for r in rows(d / "trials.jsonl")]
    place += [dict(r, run="r3_cup_right_fix") for r in rows(RES / "r3_cup_right_fix" / "trials.jsonl")]
    ob = orient_block()
    orient_html = ob[0] if ob else ""
    v3 = a.out.parent / "videos_r3.json"
    r3v = json.loads(v3.read_text()) if v3.is_file() else []
    rk = RES / "place_racked.json"
    racked = json.loads(rk.read_text()) if rk.is_file() else {}
    vj = a.out.parent / "videos.json"
    if vj.is_file() and not notes.get("videos"):
        notes["videos"] = json.loads(vj.read_text())
    vids = "".join(
        f"<figure><{'video controls muted playsinline preload=metadata' if v['file'].endswith('.mp4') else 'img alt=\"' + html.escape(v['caption']) + '\"'} "
        f"src=\"{html.escape(v['file'])}\">{'</video>' if v['file'].endswith('.mp4') else ''}<figcaption class=muted>{html.escape(v['caption'])}</figcaption></figure>"
        for v in notes.get("videos", []))
    legend = "".join(f"<span><i class=dot style='background:{GATE_COLOR[g]}'></i>{t}</span>" for g, t in
                     (("G2", "G2 path found (number = yaws with a path)"), ("G1", "G1 collision-free IK at every waypoint (number = yaws)"),
                      ("G0", "G0 IK only"), ("none", "no IK at the release")))
    body = f"""<main>
<h1>UR5e + 2F-85: what a mouth-up rim pinch can grasp and where it can place it</h1>
<p class=muted>Limits round v1, rounds 1–3 (page Version 4) · {html.escape(notes.get('date', ''))} · plan <code>plans/2026-10-23-hardware-limits-mouth-up.md</code> ·
Isaac Sim 4.5 on the L4 box, headless (no cameras); videos are Warp replays of the logged trials.</p>
<div class=card><b>Question.</b> One HOTEC dish at a time (bowl, plate, cup), lying mouth-up on the counter: (1) can the UR5e with the
Robotiq 2F-85 grasp it, (2) which rack positions can the arm place it into?</div>
<h2>Conclusions at the checkpoint</h2><div class=card><ul>{''.join(f'<li>{c}</li>' for c in notes['conclusions'])}</ul></div>
<h2>Setup</h2><div class=card><ul>{''.join(f'<li>{s}</li>' for s in notes['setup'])}</ul></div>
<h2>Experiment 1 · grasp gate on the counter (G3)</h2>
<p>G3 = the pick's one best-facing reachable rim-pinch candidate (12 yaws generated), close, lift 0.15 m, 0.75 s in-hand settle,
then a 5 s window with the dish touching only the gripper and drifting under 5 mm in the TCP frame. Outcomes: <i>hold</i> (G3, pivot under 10°),
<i>pivot</i> (G3, the dish rotated 10° or more in the hand first), <i>slip</i> (not held), <i>miss</i> (jaws closed off the wall),
<i>no-reach</i> (no collision-free approach). A trial is “clean” when no harness invariant fired in it.</p>
{gate_table(runs)}
<h2>Experiment 2 · reach sweep (G0–G2, Kit-free)</h2>
<p>Per (object, rack site, yaw): G0 = IK on any of the 8 branches at the release (site pose + 6 mm); G1 = collision-free IK at pre-grasp,
grasp, lift, pre-place (10 cm back along the approach) and release with the dish carried; G2 = approach and lift lines, an RRT-Connect
path lift → pre-place within 3 s, and the straight insertion line. Hover a site for the per-yaw counts and the stop reasons.</p>
<p class=legend>{legend}</p>
{reach_block(reach, summ)}
{diag_block()}
<h2>Experiment 2 · placement in Isaac (G4)</h2>
<p>G4 = pick from the counter (up to 6 candidates), carry, insert, release, the benchmark's 150-tick settle, then the at-goal tolerance
(<code>frigidaire_bench.within_goal</code>, the Frigidaire stand-in for <code>evaluate_placement</code>) against the site pose.</p>
<h3>Released at the target height (round 1 method; bowls pooled over rounds 1 and 2)</h3>
{place_table(place, racked, "air")}
<h3>Lowered until contact, then opened (round 2: the upright3 release rule; judged against the dish pose at the release; round 3: the cup right slots with the contact detector fixed)</h3>
{place_table(place, racked, "down")}
<h2>Round 3 · orientation sweep: can a bowl or cup go in tilted?</h2>
<p>The target tilts the dish 0–150° about its horizontal axis toward the site's lean side (the horizontal part of the packer pose's
dish axis). 0° is level and mouth up; 150° is near the packer's angle. The rim pinch is unchanged and the dish sits in the hand as
measured after the lift. G0–G2 are the Kit-free gates; the release height is the lowest clear height at or above the level pose's.
Isaac then lowers the dish at the target tilt until contact (contact lag 0.15 rad) and judges G4 against the dish pose at the release.</p>
{orient_html}
<h2>Failure modes (Warp replays and one Kit-free still)</h2>
{gallery([v for v in r3v if v.get('mode')])}
<h2>Videos (Warp replays of the logged Isaac trials)</h2><div class=vids>{vids or '<p>none</p>'}</div>
{('<h3>Round 3 passes</h3>' + gallery([v for v in r3v if not v.get('mode')])) if [v for v in r3v if not v.get('mode')] else ''}
<h2>Literature in three lines</h2><div class=card><ul>{''.join(f'<li>{l}</li>' for l in notes['litreview'])}</ul></div>
<h2>Not done ({html.escape(notes.get('date', ''))})</h2><div class=card><ul>{''.join(f'<li>{l}</li>' for l in notes['not_done'])}</ul></div>
<p class=muted>Records: <code>data/results/robot/limits_v1/</code> (trials.jsonl per run, reach.json / reach.jsonl); box window {html.escape(notes.get('window', ''))}.</p>
</main>"""
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(f"<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
                     f"<title>Robot Hardware Limits</title><style>{CSS}</style></head><body>{body}</body></html>\n")
    print(f"[RESULT] PASS page {a.out}")


if __name__ == "__main__":
    main()
