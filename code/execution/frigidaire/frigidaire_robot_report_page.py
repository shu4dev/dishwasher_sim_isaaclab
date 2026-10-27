#!/usr/bin/env python3
"""Hardware Limits Report page (plans/2026-10-26-hardware-exhaustive.md, Decisions "Report"): per object the grasp rate
per counter spot, the Kit-free reach per rack site and tilt, the Isaac placement per site at level with a rack map, the
tilt limit per site, a success and a failure clip, stills of the passing poses and the failure modes. Reads one or more
record folders (Isaac trials pool over them; a later folder's Kit-free sweep replaces an earlier one's per kind and
target) and their media folders; a folder that does not exist yet reads "not run". Kit-free, on the Mac; the rack maps
need matplotlib, which this Mac's python3 lacks, so run it with /opt/anaconda3/bin/python3:

    /opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py \\
        --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 \\
        --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 --out data/media/robot/report_v1
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import date
import hashlib
import html
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import frigidaire_robot_limits_page as LP  # noqa: E402  (cp, med, fmt, rows, reach_summary, CSS, GATE_COLOR, RACK_BOX)

KINDS = ("bowl", "cup", "plate")
SKIP = ({("limits_v1", f"gate_{k}") for k in KINDS}               # superseded: results/2026-10-23-hardware-limits-mouth-up.md
        | {("exhaustive_w1", "w1_knuckle_dump")})                 # the knuckle diagnostic: under Verdicts, not a rate trial
SKIP_ROWS = (("limits_v1/r2_place_cup_down", "glass_right_"),)    # early contact fires there; re-run as r3_cup_right_fix
FAIL_TOKENS = ("fail", "fm_", "blocked", "slip", "miss", "drop")  # a clip whose file name has one is a failure
TARGET = {"level": "level", "family": "packer pose", "family-inhand": "upright gap pose, plate in hand"}
REL = {"air": "released in the air", "down": "lowered until contact", "contact": "lowered until contact", "none": ""}
MB = 1024 * 1024
CLIMB_DEG = 15.                       # the window-2 climb starts here: a lower tilt (round 3's 0°) never sets the limit
PLANNER = ("IK branch jump", "line collides", "no collision-free")   # a planner refusal, not a physical failure
CSS = LP.CSS + (".grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,320px),1fr));gap:14px}"
                "figure{margin:0}figcaption{font-size:13px;margin-top:4px}ul.fm{list-style:none;padding:0}"
                "ul.fm li{margin:0 0 18px}.lead{font-size:16px;max-width:75ch}h4{margin:1.2em 0 .4em}")


def table(head, body):
    return (f"<table><thead><tr>{''.join(f'<th>{h}</th>' for h in head)}</tr></thead><tbody>"
            f"{''.join(body) or f'<tr><td colspan={len(head)} class=muted>not run</td></tr>'}</tbody></table>")


def load(dirs):
    """Isaac trial rows (pooled), rack sites by kind, Kit-free cells {(kind, target, waived): {site: cell}}, reach meta,
    Kit-free counter grid {kind: {(i, j): spot}}, the box runner's verdicts (verdicts.json; later folders override keys)."""
    trials, sites, summ, meta, grid, verdicts, windows = [], {}, {}, {}, {}, {}, []
    for d in dirs:
        runs = sorted(p for p in d.iterdir() if p.is_dir() and not p.name.startswith("_") and (d.name, p.name) not in SKIP)
        for f in [d] + runs:                        # sweeps and verdicts sit at the folder root or in a run folder
            rep = next(iter(sorted(f.glob("limits_*.json"))), None)
            rep = json.loads(rep.read_text()) if rep else {}
            lag, run = (rep.get("flags") or {}).get("place_down_contact_lag"), f"{d.name}/{f.name}"
            trials += [dict(r, run=run, lag=lag) for r in LP.rows(f / "trials.jsonl") or rep.get("trials", [])
                       if not any(run == a and str(r.get("site")).startswith(b) for a, b in SKIP_ROWS)]
            if (f / "reach.json").is_file():
                meta = json.loads((f / "reach.json").read_text())
                for k, ss in meta["sites"].items():
                    sites.setdefault(k, {}).update({s["site"]: s for s in ss})
            if (f / "counter_grid.json").is_file():             # the Kit-free 5 x 5 pick spots per kind
                for k, v in json.loads((f / "counter_grid.json").read_text())["kinds"].items():
                    grid[k] = {(s["i"], s["j"]): s for s in v["spots"]}
            if (f / "verdicts.json").is_file():
                v = json.loads((f / "verdicts.json").read_text())
                windows += [(d.name, v["window"])] if v.get("window") else []
                verdicts.update(v)
            rr = LP.rows(f / "reach.jsonl") + LP.rows(f / "reach_waived.jsonl") + LP.rows(f / "inhand.jsonl")
            rr += [dict(r, target=float(r["tilt_deg"])) for r in LP.rows(f / "orient.jsonl")]   # tilts as reach targets
            summ.update(LP.reach_summary(None, rr))
    return trials, sites, summ, meta, grid, verdicts, windows


def grasp_block(rs, meta, grid):
    """Isaac G3 per counter spot and grasp; the Kit-free grid's spots without Isaac trials read "not run"."""
    xy, by = {None: meta.get("counter_xy")}, {}
    for r in rs:
        sp = r.get("spot")                                # counter-grid rows: {"i", "j", "xy", "kitfree_gate"}; else None
        key = (sp["i"], sp["j"]) if sp else None
        xy[key] = sp["xy"] if sp else xy[None]
        g = (r.get("grasp_family") or "rim pinch") + (" · planner clearance waived" if r.get("waive_dish_clearance") else "")
        by.setdefault((key, g), []).append(r)
    for key, sp in grid.items():
        xy[key] = sp["xy"]

    def label(key):
        p = xy[key]
        return ((f"({p[0]:.2f}, {p[1]:.2f}) m" if p else "counter")
                + ("" if key is None else f" · grid {key[0]},{key[1]} · Kit-free {(grid.get(key) or {}).get('gate', '–')}"))
    body = []
    for (key, g), lst in sorted(by.items(), key=lambda kv: (kv[0][0] is not None, kv[0][0] or (0, 0), kv[0][1])):
        n, k = len(lst), sum(bool(r["g3"]) for r in lst)
        lo, hi = LP.cp(k, n)
        inv = Counter(i for r in lst for i in r.get("invariants", []))
        fails = Counter(r["outcome"] for r in lst if not r["g3"])
        body.append(f"<tr><td>{label(key)}</td><td>{g}</td><td>{n}</td><td><b>{k}/{n}</b></td><td>{lo:.2f}–{hi:.2f}</td>"
                    f"<td>{LP.fmt(LP.med([(r.get('hold') or {}).get('settle_rotation_deg') for r in lst if r['g3']]))}</td>"
                    f"<td>{sum(bool(r['counted']) for r in lst)}/{n}"
                    f"{' (fired: ' + ', '.join(f'{i} {c}' for i, c in sorted(inv.items())) + ')' if inv else ''}</td>"
                    f"<td>{', '.join(f'{o} {c}' for o, c in fails.most_common()) or 'none'}</td></tr>")
    ran = {key for key, _ in by}
    body += [f"<tr><td>{label(key)}</td><td>rim pinch</td><td colspan=6 class=muted>not run</td></tr>"
             for key in sorted(grid) if key not in ran]
    on = [r for key, lst in by.items() if key[0] is not None for r in lst]
    isaac = (f"Isaac G3 {sum(bool(r['g3']) for r in on)}/{len(on)} over {len({k_ for k_, _ in by if k_})} spots" if on
             else "Isaac G3 not run")
    tot = (f"<p>Counter grid: Kit-free G2 (a collision-free pick path) at {sum(s_['gate'] == 'G2' for s_ in grid.values())}/"
           f"{len(grid)} spots; {isaac}.</p>" if grid else "")
    return tot + table(("counter spot", "grasp", "n", "passes (G3)", "95 % CI", "median pivot °", "counted (D4)",
                  "failure modes"), body)


def col_key(c):
    t, w = c
    return (isinstance(t, float), t if isinstance(t, float) else (t != "level", t), w)


def col_name(c):
    t, w = c
    return (f"{t:.0f}°" if isinstance(t, float) else TARGET.get(t, t)) + (" (clearance waived)" if w else "")


def reach_block(kind, summ, names):
    cols = sorted({(t, w) for k, t, w in summ if k == kind} | {("level", False)}, key=col_key)
    body = []
    for s in names:
        tds = ""
        for col in cols:
            c = summ.get((kind,) + col, {}).get(s)
            if c is None:
                tds += "<td class=muted>not run</td>"
                continue
            tip = "; ".join(f"{w} ×{n}" for w, n in sorted(c["why"].items(), key=lambda kv: -kv[1])[:3])
            txt = f"G2 {c['G2']}/{c['n']}" if c["gate"] == "G2" else c["gate"]
            tds += (f"<td title='{html.escape(tip)}' style='background:color-mix(in srgb, {LP.GATE_COLOR[c['gate']]} 28%, "
                    f"transparent)'>{txt}</td>")
        body.append(f"<tr><td>{html.escape(s)}</td>{tds}</tr>")
    tot = []
    for c in cols:
        cells = summ.get((kind,) + c, {})
        tot.append(f"{col_name(c)} " + (f"{sum(x['gate'] == 'G2' for x in cells.values())}/{len(cells)}" if cells else "not run"))
    return f"<p>Sites with a Kit-free path (G2): {' · '.join(tot)}.</p>" + table(["site"] + [col_name(c) for c in cols], body)


def release(r):
    if r.get("yaw_retry"):                                     # window 2: the planner-blocked sites at another yaw
        return f"released in the air, other yaws (yaw {r.get('yaw', 'not recorded')})"
    return "released in the air" if r.get("release", "air") == "air" else "lowered until contact"


def variant(r):
    """How a contact release was run (pooled in one row per site, as results/2026-10-26-hardware-exhaustive-w1.md does)."""
    how = "turned to the target" if r.get("down_orient") == "target" else "as it hangs"
    return how + ("" if r.get("lag") is None else f", contact lag {r['lag']:.2f} rad")


def final_tilt(r):
    """Settled tilt from vertical; rows before round 3 lack it: then the at-goal tilt, equal to it for a level air release.
    None when the dish was never released at the site (no pick, or the place stopped: place_why)."""
    if r.get("place_why") or not (r.get("g3") or r.get("grasp_family") == "in hand"):
        return None
    if r.get("final_tilt_deg") is not None:
        return r["final_tilt_deg"]
    d = (r.get("judge") or {}).get("distance")
    return d[2] if d and r.get("release", "air") == "air" else None


def place_block(rs, rack_of, names, level):
    by = {}
    for r in rs:
        by.setdefault(r["site"], {}).setdefault(release(r), []).append(r)
    body, best, tot, groups = [], {}, {}, {}
    for s in names:
        rack = rack_of[s].replace("Rack", "").lower()
        if s not in by:
            why = " · no Kit-free path" if s in level and level[s]["gate"] != "G2" else ""
            body.append(f"<tr><td>{html.escape(s)}</td><td>{rack}</td><td colspan=5 class=muted>not run{why}</td></tr>")
            continue
        for rel, lst in sorted(by[s].items()):
            n, k = len(lst), sum(bool(r.get("g4")) for r in lst)
            lo, hi = LP.cp(k, n)
            if s not in best or k / n > best[s][0] / best[s][1]:
                best[s] = (k, n)
            tot[rel] = (tot.get(rel, (0, 0))[0] + k, tot.get(rel, (0, 0))[1] + n)
            groups.setdefault(s, []).append((k, n))
            how = "" if not rel.startswith("lowered") else " <span class=muted>(" + "; ".join(
                f"{v} ×{c}" for v, c in sorted(Counter(variant(r) for r in lst).items())) + ")</span>"
            body.append(f"<tr><td>{html.escape(s)}</td><td>{rack}</td><td>{rel}{how}</td><td>{n}</td><td><b>{k}/{n}</b></td>"
                        f"<td>{lo:.2f}–{hi:.2f}</td><td>{LP.fmt(LP.med([final_tilt(r) for r in lst]))}</td></tr>")
    head = f"<p>Pooled over the sites: {'; '.join(f'{rel} {k}/{n}' for rel, (k, n) in sorted(tot.items()))}.</p>" if tot else ""
    return head + table(("site", "rack", "release", "n", "passes (G4)", "95 % CI", "median final tilt °"), body), best, groups


def rack_map(sites, best, unreachable, path, where):
    """Top view of the sites in the rack frame (front at the bottom), coloured by the best release mode's pass rate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, Normalize
    from matplotlib.lines import Line2D
    ink, muted = "#17202a", "#8a94a0"
    cmap = LinearSegmentedColormap.from_list("pass", ["#86b6ef", "#3987e5", "#0d366b"])   # one-hue sequential blue
    fig, axes = plt.subplots(1, 2, figsize=(10, 6.4), dpi=110, facecolor="#fcfcfb")
    for ax, rack in zip(axes, ("UpperRack", "LowerRack")):
        (x0, x1), (y0, y1) = LP.RACK_BOX[rack]
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls="--", lw=.8, ec=muted))
        pts = {v["site"]: v["local"]["position_m"][:2] for v in sites.values() if v["rack"] == rack}
        done = []
        for s, (x, y) in pts.items():
            dx, dy = min(((abs(x - a), abs(y - b)) for t, (a, b) in pts.items() if t != s), key=sum, default=(1, 1))
            near = any(abs(x - a) < .02 and abs(y - b) < .02 for a, b in done)  # ponytail: one close neighbour flips side
            done.append((x, y))
            if dx > dy:                                   # a row of sites: the label stands up (down for the neighbour)
                rot, ha, off = (-90, "left", (0, -6)) if near else (90, "left", (0, 6))
            else:                                         # a column: the label to the right (left for the neighbour)
                rot, ha, off = (0, "right", (-6, 0)) if near else (0, "left", (6, 0))
            kn, ms = best.get(s), 5 if near else 8
            if kn:
                ax.plot(x, y, "o", ms=ms, mfc=cmap(kn[0] / kn[1]), mec=ink, mew=.6)
            elif s in unreachable:
                ax.plot(x, y, "x", ms=ms * .8, mec=muted, mew=1.2)
            else:
                ax.plot(x, y, "o", ms=ms, mfc="none", mec=muted, mew=1)
            ax.annotate(s + (f"  {kn[0]}/{kn[1]}" if kn else ""), (x, y), xytext=off, textcoords="offset points",
                        rotation=rot, rotation_mode="anchor", fontsize=6.5, color=ink, ha=ha, va="center")
        ax.set_xlim(x0 - .03, x1 + .14)
        ax.set_ylim(y0 - .22, y1 + .17)
        ax.set_aspect("equal")
        ax.set_title(f"{rack.replace('Rack', '')} rack · front (door) at the bottom", fontsize=9, color=ink)
        ax.set_xlabel("rack x (m)", fontsize=7, color=muted)
        ax.tick_params(labelsize=7, colors=muted)
        for sp in ax.spines.values():
            sp.set_color("#d9dee5")
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(0, 1), cmap=cmap), ax=axes, shrink=.6, pad=.02)
    cb.ax.tick_params(labelsize=7)
    cb.set_label(
        f"placement pass rate {where} (best release mode)", fontsize=8, color=ink)
    fig.legend(handles=[Line2D([], [], marker="o", ls="", mfc="none", mec=muted, label="Isaac not run"),
                        Line2D([], [], marker="x", ls="", mec=muted, label="not run: no Kit-free path")],
               loc="lower center", ncol=2, fontsize=7.5, frameon=False)
    fig.savefig(path, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def tilt_cell(t, cell):
    """k/n at one tilt; a cell pooling several record folders lists each folder's share."""
    by = {}
    for src, g in cell:
        by.setdefault(src, []).append(g)
    parts = f" ({', '.join(f'{src} {sum(v)}/{len(v)}' for src, v in by.items())})" if len(by) > 1 else ""
    kn = f"{sum(g for _, g in cell)}/{len(cell)}{parts}"
    return f"{t:.0f}° {kn}" if t >= CLIMB_DEG else f"{kn} ({t:.0f}°, round 3)"


def tilt_block(rs, summ, kind):
    isaac = {}
    for r in rs:
        isaac.setdefault(r["site"], {}).setdefault(float(r["tilt_deg"]), []).append((r["run"].split("/")[0], bool(r.get("g4"))))
    kf = {t: cells for (k, t, w), cells in summ.items() if k == kind and isinstance(t, float) and not w}
    body, limits = [], {}
    for s in sorted(set(isaac) | {s for cells in kf.values() for s in cells}):
        ts = sorted(t for t in kf if s in kf[t] and t >= 0)      # the opposite direction (negative tilts) apart
        lim = next((t for t in ts if kf[t][s]["gate"] != "G2"), None)
        cells = isaac.get(s, {})
        its = sorted(cells)
        climb = [t for t in its if t >= CLIMB_DEG]
        ilim = next((t for t in climb if not any(g for _, g in cells[t])), None)
        if climb:
            limits[s] = ilim                                   # None: no tilt scored 0/n
        body.append(f"<tr><td>{html.escape(s)}</td>"
                    f"<td>{'not run' if not ts else f'{lim:.0f}°' if lim is not None else f'none up to {ts[-1]:.0f}°'}</td>"
                    f"<td>{' · '.join(tilt_cell(t, cells[t]) for t in its) or 'not run'}</td>"
                    f"<td>{'not run' if not climb else f'<b>{ilim:.0f}°</b>' if ilim is not None else f'above {climb[-1]:.0f}° (none reached)'}"
                    f"</td></tr>")
    return table(("site", "Kit-free limit (first tilt without a path)", "Isaac placed k/n per tilt",
                  "Isaac limit (first 0/n from 15°)"), body), limits


def manifest_caption(e):
    """A structured videos.json entry's caption, with the object, site, tilt and release named (the runner's caption as
    is when it already names the site). An IK branch jump is marked as the planner's limit (w2 note: not a tip-over)."""
    jump = " · the planner's IK branch jump, not a tip-over" if "IK branch jump" in str(e.get("caption")) else ""
    if e.get("caption") and (not e.get("site") or str(e["site"]) in e["caption"]):
        return e["caption"] + jump
    where = f"at {e['site']}" if e.get("site") else "at the counter"
    tilt = ("" if not e.get("site") else ("upright gap pose" if e.get("kind") == "plate" else "level")
            if e.get("tilt_deg") is None else f"tilt {float(e['tilt_deg']):.0f}°")
    parts = (f"{e.get('kind') or ''} {where}".strip(), tilt, REL.get(e.get("release"), e.get("release") or ""),
             "" if e.get("trial") is None else f"trial {e['trial']}", e.get("outcome") or "", e.get("caption") or "")
    return " · ".join(str(x) for x in parts if x) + jump


def media_index(dirs):
    """One entry per clip; files of one object with identical bytes are one clip with every name in its caption. An mp4
    is keyed by its six-frame strip PNG: the replay burns the label into every mp4 frame, so relabelled copies differ.
    A structured videos.json entry (file, kind, site, tilt_deg, release, trial, outcome, caption) sets the kind, the
    caption and success or failure; else the file name does (older caption-only manifests give the caption)."""
    clips, seen = [], {}
    for i, d in enumerate(dirs):
        man = {v["file"]: v for m in sorted(d.glob("videos*.json")) for v in json.loads(m.read_text())}
        for f in sorted(d.iterdir()):
            e = man.get(f.name) or {}
            kind = e.get("kind") if e.get("kind") in KINDS else next((k for k in KINDS if k in f.stem), None)
            if f.suffix not in (".mp4", ".png") or not kind:
                continue
            strip = f.with_suffix(".png")
            key = (kind, f.suffix, hashlib.sha256((strip if strip.is_file() else f).read_bytes()).hexdigest())
            full = "outcome" in e or "site" in e
            cap = manifest_caption(e) if full else (e.get("caption") or (man.get(f.stem + ".mp4") or {}).get("caption")
                                                    or f.stem.replace("_", " "))
            bad = (any(t in str(e["outcome"]).lower() for t in FAIL_TOKENS + ("no-reach", "false")) if "outcome" in e
                   else any(t in f.stem for t in FAIL_TOKENS))
            if key not in seen:
                seen[key] = {"src": f, "i": i, "kind": kind, "mp4": f.suffix == ".mp4", "names": [], "caps": [],
                             "good": True, "full": full}
                clips.append(seen[key])
            c = seen[key]
            c["names"].append(f.stem)
            c["good"] = c["good"] and not bad
            if cap not in c["caps"]:
                c["caps"].append(cap)
    for c in clips:
        c["caption"] = " · ".join(c["caps"]) + (f" (one clip, files {', '.join(c['names'])})" if len(c["names"]) > 1 else "")
    return clips


def use(c, out):
    """Copy a clip to <out>/media/<its media folder>/; PNGs above 10 MiB palettised (as the Dot Score Rules page does)."""
    rel = f"media/{c['src'].parent.name}/{c['src'].name}"
    dst = out / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(c["src"], dst)
    if dst.suffix == ".png" and dst.stat().st_size > 10 * MB:
        from PIL import Image
        Image.open(dst).convert("RGB").quantize(colors=256, method=Image.Quantize.MEDIANCUT).save(dst, optimize=True)
    return rel


def figure_html(rel, caption):
    tag = (f"<video controls muted playsinline preload=metadata src='{html.escape(rel)}'></video>" if rel.endswith(".mp4")
           else f"<img loading=lazy alt='{html.escape(caption)}' src='{html.escape(rel)}'>")
    return f"<figure>{tag}<figcaption class=muted>{html.escape(caption)}</figcaption></figure>"


def rank(c, good):
    """A physical failure before a planner refusal; then the newest media folder; then by name: a placement (level)
    pass or fail before any other, before the rest."""
    pref = ("place_pass", "pass", "ok") if good else ("place_fail", "level_fail", "fail", "fm_")
    physical = good or not any(t in c["caption"] for t in PLANNER)
    return physical, c["i"], -min(next((j for j, t in enumerate(pref) if t in n), len(pref)) for n in c["names"])


def pose_caption(c, names):
    if c["full"]:                                              # the manifest names the site, tilt and release
        return c["caption"]
    text = f"{c['caption']} {' '.join(c['names'])}"
    site = next((s for s in sorted(names, key=len, reverse=True) if s in text or s.replace("@", "_") in text), None)
    if site is None:
        how = "held after the pick from the counter" if "gate" in text or "grid" in text else "placement, site not in the name"
        return f"{c['kind']} {how} · {c['caption']}"
    m = re.search(r"tilt (\d+)", text)
    rel = "lowered until contact" if "contact" in text or "_down" in text else "released in the air"
    return f"{c['kind']} at {site} · {'tilt ' + m.group(1) + '°' if m else 'level'} · {rel} · {c['caption']}"


def media_blocks(kind, clips, names, out):
    mine = [c for c in clips if c["kind"] == kind]
    good = [c for c in mine if c["mp4"] and c["good"]]
    newer = [c for c in good if c["i"] > 0]                   # every success clip of the window folders, else round 1's best
    oks = sorted(newer, key=lambda c: rank(c, True), reverse=True) or ([max(good, key=lambda c: rank(c, True))] if good else [])
    bad = max((c for c in mine if c["mp4"] and not c["good"]), key=lambda c: rank(c, False), default=None)
    vids = "".join(figure_html(use(c, out), f"{tag} · {c['caption']}") for c, tag in
                   [(c, "Success") for c in oks] + [(bad, "Failure")] if c)
    stills = "".join(figure_html(use(c, out), pose_caption(c, names)) for c in mine if c["good"] and not c["mp4"])
    mp4s = {n for c in mine if c["mp4"] for n in c["names"]}
    fm = "".join(f"<li>{figure_html(use(c, out), c['caption'])}</li>" for c in mine
                 if not c["good"] and c is not bad and (c["mp4"] or not set(c["names"]) & mp4s))
    return (f"<h3>Videos</h3><div class=grid2>{vids or '<p class=muted>not run</p>'}</div>"
            f"<h4>Stills of the passing poses</h4><div class=vids>{stills or '<p class=muted>not run</p>'}</div>"
            f"<h3>Failure modes</h3>{f'<ul class=fm>{fm}</ul>' if fm else '<p class=muted>not run</p>'}")


def verdicts_block(v, round1):
    """The box runner's verdicts.json: knuckle contact, the robot-free tines test, the seed-free reach at level."""
    if not v:
        return "<section id=verdicts><h2>Verdicts</h2><p class=muted>not run</p></section>"
    k = v.get("knuckle")
    knuckle = (f"<p><b>Knuckle contact: {html.escape(str(k.get('verdict')))}.</b> {html.escape(str(k.get('evidence') or ''))} "
               f"<span class=muted>(max knuckle force {LP.fmt(k.get('max_knuckle_force_N'))} N, visual gap "
               f"{LP.fmt(k.get('visual_gap_mm'))} mm; trial <code>{html.escape(str(k.get('trial')))}</code>)</span></p>"
               if k else "<p><b>Knuckle contact:</b> <span class=muted>not run</span></p>")
    tines = table(("site", "n", "median final tilt °", "verdict"),
                  [f"<tr><td>{html.escape(s)}</td><td>{t.get('n')}</td><td>{LP.fmt(t.get('median_tilt_deg'))}</td>"
                   f"<td>{html.escape(str(t.get('verdict')))}</td></tr>" for s, t in sorted((v.get("tines") or {}).items())])
    rs = v.get("reach_seedfree") or {}

    def g2_round1(kind):
        c = round1.get((kind, "level", False), {})
        return f"{sum(x['gate'] == 'G2' for x in c.values())}/{len(c)}" if c else "–"
    reach = table(("kind", "G1 seed-free", "G2 seed-free", "G2 round 1 (HOME_Q seed)", "sites that changed"),
                  [f"<tr><td>{kind}</td><td>{rs[kind].get('g1')}</td><td>{rs[kind].get('g2')}</td><td>{g2_round1(kind)}</td>"
                   f"<td>{html.escape(', '.join(rs[kind].get('changed_sites') or [])) or 'none'}</td></tr>"
                   for kind in ("bowl", "cup") if isinstance(rs.get(kind), dict)])
    note = f"<p class=muted>{html.escape(str(rs['note']))}</p>" if rs.get("note") else ""

    def txt(x):
        return "–" if x is None else html.escape(", ".join(map(str, x)) if isinstance(x, list) else str(x))
    yaws = table(("site", "yaws tried", "released at yaw", "G4", "verdict"),
                 [f"<tr><td>{html.escape(s)}</td>" + "".join(f"<td>{txt(y.get(k_))}</td>" for k_ in
                  ("yaws_tried", "released_at_yaw", "g4", "verdict")) + "</tr>" for s, y in sorted((v.get("yaw_retry") or {}).items())])
    ph = v.get("plate_in_hand")
    plate = table(("Kit-free G2 gaps", "Isaac G4", "note"),
                  [f"<tr><td>{txt(ph.get('g2_gaps'))}</td><td>{txt(ph.get('g4'))}</td><td>{txt(ph.get('note'))}</td></tr>"] if ph else [])
    return (f"<section id=verdicts><h2>Verdicts</h2><h3>Knuckle contact</h3>{knuckle}"
            f"<h3>Robot-free tines test</h3><p class=muted>A level bowl teleported onto the site and settled with no robot "
            f"(verdict rack: the rack alone tilts it; robot: the tilt comes with the robot's release).</p>{tines}"
            f"<h3>Seed-free IK reach at level</h3><p class=muted>Sites with a Kit-free G1 / G2 at level when the IK expands "
            f"the 2π wraps, beside round 1's HOME_Q-seeded G2.</p>{reach}{note}"
            f"<h3>Other yaws at the planner-blocked sites</h3><p class=muted>In-air level release at the sites whose yaw-0 "
            f"pose Isaac's planner refused.</p>{yaws}"
            f"<h3>Plates in hand</h3><p class=muted>The plate starts in the gripper (no grasp) and goes to the upright gap "
            f"pose.</p>{plate}</section>")


def knuckle_note(kind, measured, in_lead):
    """The kind the knuckle-contact check measured (verdict real) gets that verdict; another kind only the invariant."""
    if kind == measured:
        return (", resting on the inner knuckle (the harness invariant flags this; measured a real contact)" if in_lead
                else "; knuckle contact measured real")
    return ((", the rim resting on the knuckles and fingers" if in_lead else "; the rim rests on the knuckles and fingers")
            + (f" (contact check done on the {measured} only)" if measured else " (the harness invariant flags this)"))


def lead(kind, grasp, groups, limits, where, measured):
    """One generated sentence per object: the physical grasp outcome (the counted column stays in the table), the sites
    placed at 9/10 or better, the sites at 0/n, the Isaac tilt limit range."""
    def held(rs):
        return f"{sum(bool(r['g3']) for r in rs)}/{len(rs)}"
    one = {}
    for r in grasp:
        if not r.get("spot"):
            one.setdefault((r.get("grasp_family") or "rim pinch")
                           + (", planner clearance waived" if r.get("waive_dish_clearance") else ""), []).append(r)
    on = [r for r in grasp if r.get("spot")]
    g = ((" and ".join(f"{held(rs)} ({fam})" if len(one) > 1 else held(rs) for fam, rs in sorted(one.items()))
          + " held at the counter spot") if one else "not run at the counter spot")
    g += f" and {held(on)} over the {len({(r['spot']['i'], r['spot']['j']) for r in on})} grid spots" if on else ""
    kept = [r for r in grasp if r["g3"]]
    if kept and all("carried_contact" in r.get("invariants", []) for r in kept):
        g += knuckle_note(kind, measured, True)
    hi = sorted(s for s, kn in groups.items() if any(n >= 10 and k >= .9 * n for k, n in kn))
    zero = sum(not any(k for k, _ in kn) for kn in groups.values())
    def sites_(k):
        return f"{k} site{'s' * (k != 1)}"
    p = (f"{where}, {sites_(len(hi))} placed at 9/10 or better{' (' + ', '.join(hi) + ')' if hi else ''} and {zero} at 0/n"
         if groups else f"placement {where} not run")
    lim = sorted(t for t in limits.values() if t is not None)
    none = len(limits) - len(lim)
    t = "; Isaac tilt limit " + ", ".join(x for x in (
        (f"{lim[0]:.0f}°" if lim[0] == lim[-1] else f"{lim[0]:.0f}–{lim[-1]:.0f}°") + f" at {sites_(len(lim))}" if lim else "",
        f"none reached at {sites_(none)}" if none else "") if x) if limits else ""
    return f"<p class=lead>{html.escape(f'{kind.capitalize()}: grasp {g}; {p}{t}.')}</p>"


def part_a_rows(root):
    """Part A grasp runs kept beside the placing loads (placing_v1/partA_*, the gate2 format)."""
    out = []
    for f in (sorted(root.glob("partA_*")) if root and root.is_dir() else []):
        rep = next(iter(sorted(f.glob("limits_*.json"))), None)
        rs = LP.rows(f / "trials.jsonl") or (json.loads(rep.read_text()).get("trials", []) if rep else [])
        out += [dict(r, run=f"{root.name}/{f.name}") for r in rs]
    return out


def grasping_block(trials, grid, measured):
    """Part A (plans/2026-10-26-placing-goal.md): the measured grasps at the counter spot, one row per object and grasp
    (a main grasp first, "not run" until its run lands); the counter grid as a footnote."""
    by, on = {}, {}
    for r in trials:
        if r.get("mode") != "grasp_gate":
            continue
        if r.get("spot"):
            on.setdefault(r["kind"], []).append(r)
            continue
        g = (r.get("grasp_family") or "rim pinch") + (", planner clearance waived" if r.get("waive_dish_clearance") else "")
        by.setdefault((KINDS.index(r["kind"]), r["kind"], g), []).append(r)
    for k, g in MAIN_GRASP.items():
        by.setdefault((KINDS.index(k), k, g), [])
    body = []
    for (_, kind, g), rs in sorted(by.items(), key=lambda kv: (kv[0][0], kv[0][2] != MAIN_GRASP.get(kv[0][1]), kv[0][2])):
        label = f"{g} (main)" if g == MAIN_GRASP.get(kind) else EARLIER.get((kind, g), g)
        if not rs:
            body.append(f"<tr><td>{kind}</td><td>{html.escape(label)}</td><td colspan=4 class=muted>not run</td></tr>")
            continue
        held = [r for r in rs if r["g3"]]
        cc = [r for r in rs if "carried_contact" in r.get("invariants", [])]
        knuckle = (knuckle_note(kind, measured, False)
                   if g.startswith("rim pinch") and held and all(r in cc for r in held) else "")   # the rim on the knuckles
        body.append(f"<tr><td>{kind}</td><td>{html.escape(label)}</td><td><b>{len(held)}/{len(rs)}</b></td>"
                    f"<td>{LP.fmt(LP.med([(r.get('hold') or {}).get('settle_rotation_deg') for r in held]))}</td>"
                    f"<td>carried_contact fired {len(cc)}/{len(rs)}{knuckle}</td>"
                    f"<td>{html.escape(', '.join(sorted({r['run'] for r in rs})))}</td></tr>")
    foot = ""
    if on:
        ks = sorted(on, key=KINDS.index)
        spots = {(r["spot"]["i"], r["spot"]["j"]) for k in ks for r in on[k]}
        kf = [f"{sum(x['gate'] == 'G2' for x in grid[k].values())}/{len(grid[k])}" for k in ks if grid.get(k)]
        kft = (f"{kf[0]} spots Kit-free" if len(set(kf)) == 1 and len(kf) == len(ks) else
               ", ".join(f"{k} {v} spots Kit-free" for k, v in zip(ks, kf))) if kf else "Kit-free not run"
        held = " and ".join(f"{k} {sum(bool(r['g3']) for r in on[k])}/{len(on[k])}" for k in ks)
        foot = (f"<p class=muted>Pick position does not matter: {held} over the "
                f"{len({i for i, _ in spots})}x{len({j for _, j in spots})} counter grid, {kft}.</p>")
    return ("<section id=grasping><h2>Grasping: summary</h2>"
            + table(("object", "grasp", "held k/n", "median pivot °", "invariant", "source run"), body) + foot + "</section>")


STEPS = ("R1", "R2", "S1", "S2", "S3", "S4", "S5", "S6")
VARIANTS = ("W", "R", "R-ideal")              # R-ideal: the flip onto the counter idealized by a teleport, then R
MAIN_GRASP = {"cup": "wall pinch"}                          # the user's decision, 2026-10-26: the cup's main grasp
EARLIER = {("cup", "rim pinch"): "rim pinch (earlier; re-test after round 3)"}
LEGEND = ("Steps, first failure wins: S1 reorient in hand (IK), S2 pre-place reachable, S3 path and insertion, S4 insert "
          "and release, S5 settled within 15 mm / 20 mm / 15°, S6 no earlier dish disturbed; R1 flip and set down "
          "mouth-down, R2 re-pick (foot ring, or the cup's base). W keeps the hold and turns the wrist; R regrasps, run "
          "where W failed at S1 or S2.")


def step_cell(r):
    if r is None:
        return "<span class=muted>not run</span>"
    if r.get("failed_at"):
        return f"<b>{html.escape(str(r['failed_at']))}</b>: {html.escape(str(r.get('why') or ''))}"
    e = r.get("error") or {}
    return (f"placed ({LP.fmt(e.get('lateral_mm'))} mm lateral, {LP.fmt(e.get('dz_mm'))} mm height, "
            f"{LP.fmt(e.get('tilt_deg'))}° tilt)")


def later_cell(w, y):
    """R or R-ideal: "not needed" when W placed the dish; else its first failing step, with the runner's note."""
    if w and not w.get("failed_at"):
        return "not needed"
    note = f" <span class=muted>({html.escape(str(y['note']))})</span>" if y and y.get("note") else ""
    return step_cell(y) + note


def placing_block(root, mdir, out):
    """Part B: per load, each dish placed at its goal pose from an in-hand start (variant W, then R where W failed at
    S1 or S2). Reads <load>/load.json, dishes.jsonl, summary.json and the media's videos.json; missing = "not run"."""
    head = "<section id=placing><h2>Placing the goal, dish already in hand</h2>"
    if root is None or not root.is_dir():
        return head + "<p class=muted>not run</p></section>"
    man = json.loads((mdir / "videos.json").read_text()) if mdir and (mdir / "videos.json").is_file() else []
    loads = ["easy_bowls", "easy_plates", "easy_cups"]
    loads += sorted(p.name for p in root.iterdir() if (p / "load.json").is_file() and p.name not in loads)
    parts, hist = [], Counter()
    for name in loads:
        d = root / name
        load = json.loads((d / "load.json").read_text()) if (d / "load.json").is_file() else {}
        rows = LP.rows(d / "dishes.jsonl")
        summ = json.loads((d / "summary.json").read_text()) if (d / "summary.json").is_file() else {}
        src = f" <span class=muted>({html.escape(str(load['source_pair']))})</span>" if load.get("source_pair") else ""
        if not (load or rows):
            parts.append(f"<h3>{html.escape(name)}</h3><p class=muted>not run</p>")
            continue
        V = {v: {r["dish"]: r for r in rows if r.get("variant") == v} for v in VARIANTS}
        W = V["W"]
        hist.update((v, r.get("failed_at") or "placed") for v in VARIANTS for r in V[v].values())
        dishes = sorted(load.get("dishes") or [{"name": r["dish"], "kind": r.get("kind"), "order_index": r.get("order_index")}
                                               for r in W.values()], key=lambda x: (x.get("order_index") is None, x.get("order_index")))
        ok_w = sum(1 for x in dishes if x["name"] in W and not W[x["name"]].get("failed_at"))
        placed = ", ".join(f"{sum(not r.get('failed_at') for r in V[v].values())}/{len(V[v])} with {v}" if V[v] else f"{v} not run"
                           for v in VARIANTS[1:])
        holds = sorted({x.get("grasp") for x in (load.get("dishes") or []) + rows if x.get("grasp")})
        by_ = "in hand by " + " / ".join(("an " if h[0] in "aeiou" else "a ") + h for h in holds) + "; " if holds else ""
        ss = summ.get("subset_settle")
        stands = ((("yes" if ss.get("ok") else "no") + f" (max drift {LP.fmt(ss.get('max_drift_mm'))} mm, "
                   f"{LP.fmt(ss.get('max_drift_deg'))}°)") if ss else "not run")
        body = []
        for x in dishes:
            ys = [V[v].get(x["name"]) for v in VARIANTS]
            w = ys[0]
            moved = sorted({m for y in ys if y for m in (y.get("disturbed") or [])})
            later = "".join(f"<td>{later_cell(w, y)}</td>" for y in ys[1:])
            body.append(f"<tr><td>{html.escape(str(x['name']))}</td><td>{html.escape(str(x.get('kind')))}</td><td>{step_cell(w)}</td>"
                        f"{later}<td>{html.escape(', '.join(moved)) or 'none'}</td></tr>")
        figs = ""
        for want in ("pass", "fail"):                       # one pass and one fail per load, an mp4 before a still
            es = [e for e in man if e.get("load") == name and (mdir / e["file"]).is_file()
                  and (str(e.get("outcome")).lower() == "pass") == (want == "pass")]
            e = min(es, key=lambda e: not e["file"].endswith(".mp4"), default=None)
            if e:
                cap = e.get("caption") or ""
                if str(e.get("dish")) not in cap:
                    cap = " · ".join(str(v) for v in (e.get("dish"), f"variant {e.get('variant')}", e.get("step"),
                                                         e.get("outcome"), cap) if v)
                figs += figure_html(use({"src": mdir / e["file"]}, out), cap)
        parts.append(f"<h3>{html.escape(name)}{src}</h3><p class=lead>{html.escape(name)}: {html.escape(by_)}placed {ok_w}/{len(dishes)} with W, "
                     f"{placed}; subset stands: {stands}.</p>"
                     + table(("dish", "kind", "W: first failing step (or placed, with the errors)", "R", "R-ideal (flip assumed)",
                              "disturbed"), body)
                     + (f"<div class=grid2>{figs}</div>" if figs else "<p class=muted>clips: not run</p>"))
    hist_rows = [f"<tr><td>{st}</td>" + "".join(f"<td>{hist[(v, st)]}</td>" for v in VARIANTS) + "</tr>"
                 for st in STEPS + ("placed",) if any(hist[(v, st)] for v in VARIANTS)]
    return (head + f"<p class=muted>{LEGEND}</p>" + "".join(parts) + "<h3>Failing step across the loads</h3>"
            + table(("step", "W count", "R count", "R-ideal count"), hist_rows) + "</section>")


class Srcs(HTMLParser):
    """Every src of an <img>, <video> or <source> in the page."""
    def __init__(self):
        super().__init__()
        self.srcs = []

    def handle_starttag(self, tag, attrs):
        self.srcs += [v for k, v in attrs if k == "src" and tag in ("img", "video", "source")]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--records", type=Path, nargs="+", required=True, help="record folders, oldest first")
    ap.add_argument("--media", type=Path, nargs="*", default=[], help="media folders, oldest first (clips, strips, videos*.json)")
    ap.add_argument("--placing", type=Path, default=None, help="Part B records (default: placing_v1 beside the first --records)")
    ap.add_argument("--placing-media", type=Path, default=None, help="Part B media (default: placing_v1 beside the first --media)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    for d in a.records + a.media:
        if not d.is_dir():
            print(f"[INFO] {d} not found: its cells read 'not run'", flush=True)
    recs = [d for d in a.records if d.is_dir()]
    trials, sites, summ, meta, grid, verdicts, windows = load(recs)
    kn = verdicts.get("knuckle") or {}                         # the kind the knuckle check ran on, from its trial path
    measured = next((k for k in KINDS if k in str(kn.get("trial"))), None) if kn.get("verdict") == "real" else None
    clips = media_index([d for d in a.media if d.is_dir()])
    (a.out / "media").mkdir(parents=True, exist_ok=True)
    secs = []
    for kind in KINDS:
        inhand = [r for r in trials if r.get("kind") == kind and r.get("grasp_family") == "in hand"]   # window 2 plates
        mine = [r for r in trials if r.get("kind") == kind and r.get("mode") in ("grasp_gate", "place_slot")]
        grasp = [r for r in mine if r["mode"] == "grasp_gate" and r.get("grasp_family") != "in hand"]
        place = [r for r in mine if r["mode"] == "place_slot" and r.get("target") == "level"] + inhand
        tilt = [r for r in mine if r["mode"] == "place_slot" and r.get("target") == "tilt"]
        where = "from an in-hand start" if kind == "plate" else "at level"
        rack_of = {**{r["site"]: r["rack"] for r in place + tilt}, **{s: v["rack"] for s, v in sites.get(kind, {}).items()}}
        names = sorted(rack_of, key=lambda s: (rack_of[s] != "UpperRack", s))
        level = summ.get((kind, "family-inhand" if kind == "plate" else "level", False), {})
        ptab, best, groups = place_block(place, rack_of, names, level)
        rack_map(sites.get(kind, {}), best, {s for s, c in level.items() if c["gate"] != "G2"}, a.out / "media" / f"rack_{kind}.png",
                 where)
        thtml, limits = tilt_block(tilt, summ, kind)
        for s, v in ((verdicts.get("tilt_limits") or {}).get(kind) or {}).items():   # the runner's limits cross-check the rows
            theirs = v.get("limit_deg")
            num = float(theirs) if isinstance(theirs, (int, float)) and not isinstance(theirs, bool) else None
            if limits.get(s) != num:
                print(f"[INFO] tilt limit {kind} {s}: rows {limits.get(s, 'not run')}, verdicts.json {theirs!r}", flush=True)
        opp = [(s, o) for s, o in (verdicts.get("opposite_tilt") or {}).items() if o.get("kind") == kind]
        sites_ = ", ".join(f"{s} {o.get('tilt_deg')}° {o.get('kitfree_gate')}" for s, o in opp)
        isaac_ = "; ".join(sorted({str(o.get("isaac")) for _, o in opp}))
        opp = f"<p class=muted>Opposite tilt, Kit-free only: {html.escape(sites_)}; Isaac: {html.escape(isaac_)}.</p>" if opp else ""
        tol = next((r["judge"]["tolerance"] for r in place if r.get("judge")), None)
        tol = f"{tol[0] * 1e3:.0f} mm lateral, {tol[1] * 1e3:.0f} mm height and {tol[2]:.0f}° tilt" if tol else "the at-goal tolerance"
        secs.append(
            f"<section id={kind}><h2>{kind.capitalize()}</h2>{lead(kind, grasp, groups, limits, where, measured)}"
            f"<h3>Grasp</h3><p class=muted>G3: the rim pinch closes, lifts 0.15 m, settles 0.75 s, then holds the dish 5 s "
            f"touching only the gripper with under 5 mm drift. Pivot: the dish's rotation in the hand. Counted (D4): no harness "
            f"invariant fired; carried_contact fires when the dish rests on the gripper outside the pads (the knuckles).</p>"
            f"{grasp_block(grasp, meta, grid.get(kind, {}))}"
            f"<h3>Reach</h3><p class=muted>Kit-free gates per site over 12 rim yaws: G0 IK at the release, G1 collision-free "
            f"IK at every waypoint with the dish carried, G2 a collision-free path and insertion line. A cell is the site's "
            f"best gate; G2 k/n counts the yaws with a path. Tilts: the round-3 sweep, the dish tilted about one horizontal "
            f"axis toward the site's lean side. Hover a cell for the commonest stops.</p>{reach_block(kind, summ, names)}"
            f"<h3>Placement</h3><p class=muted>Isaac G4 "
            f"{'placed from an in-hand start (no grasp)' if kind == 'plate' else 'at the level target: picked from the counter'}, carried, inserted, "
            f"released, 150 settle ticks, then within {tol} of the target. Every run pooled, release modes apart.</p>{ptab}"
            f"<figure><img loading=lazy src='media/rack_{kind}.png' alt='Top view of the {kind} sites in both racks coloured "
            f"by placement pass rate'><figcaption class=muted>Rack sites from reach.json in the rack frame; colour = the best "
            f"release mode's pass rate {where}, k/n beside the site.</figcaption></figure>"
            f"<h3>Rotation</h3><p class=muted>Isaac lowers the dish until contact at the target tilt (n per tilt as run). "
            f"Limit = the first tilt scoring 0/n, beside the first tilt with no Kit-free path.</p>{opp}{thtml}"
            f"{media_blocks(kind, clips, names, a.out)}</section>")
    mount = meta.get("mount") or {}
    where = (f"one fixed pedestal mount at ({', '.join(f'{v:.2f}' for v in mount['base'])}) m, yaw {mount['yaw_rad']:.2f} rad"
             if mount.get("base") else "one fixed pedestal mount")
    absent = [d.name for d in a.records if not d.is_dir()]
    proot = a.placing or a.records[0].parent / "placing_v1"          # Part B and the Part A runs beside it
    notes = sorted(LP.ROOT.glob("results/*hardware*.md"))
    win = "".join(html.escape(f" · window {n}: {w.get('start_edt')}–{w.get('stop_edt')} EDT, cut: {w.get('cut')}")
                  for n, w in windows)
    round1 = LP.reach_summary(None, LP.rows(recs[0] / "reach.jsonl")) if recs else {}   # the first folder: round 1
    body = (f"<main><h1>Hardware Limits Report</h1><p class=muted>{date.today().isoformat()} · records: "
            f"{', '.join(d.name for d in recs)}{' · not landed yet: ' + ', '.join(absent) if absent else ''}{win}</p>"
            f"<p class=lead>UR5e with a Robotiq 2F-85 in the Frigidaire FDPC4221AS twin, simulated headless in Isaac Sim 4.5 "
            f"on the Brev box: {where}, one HOTEC dish at a time picked mouth up from the counter and placed into an empty "
            f"rack. Simulation only, no hardware ran; videos are Warp replays of the logged trials.</p>"
            f"{grasping_block(trials + part_a_rows(proot), grid, measured)}"
            f"{placing_block(proot, a.placing_media or (a.media[0].parent / 'placing_v1' if a.media else None), a.out)}"
            f"{''.join(secs)}"
            f"{verdicts_block(verdicts, round1)}"
            f"<section id=sources><h2>Sources</h2><ul>"
            + "".join(f"<li>records <code>{html.escape(str(d))}</code>{'' if d.is_dir() else ' (not found yet)'}</li>" for d in a.records)
            + "".join(f"<li>media <code>{html.escape(str(d))}</code>{'' if d.is_dir() else ' (not found yet)'}</li>" for d in a.media)
            + "".join(f"<li>run note <code>{p.relative_to(LP.ROOT)}</code></li>" for p in notes)
            + "<li>plan <code>plans/2026-10-26-hardware-exhaustive.md</code></li></ul></section></main>")
    page = a.out / "index.html"
    page.write_text(f"<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport "
                    f"content='width=device-width,initial-scale=1'><title>Hardware Limits Report</title><style>{CSS}</style>"
                    f"</head><body>{body}</body></html>\n")
    p = Srcs()
    p.feed(page.read_text())
    srcs = sorted(set(p.srcs))
    missing = [s for s in srcs if not (a.out / s).is_file()]
    assert not missing, f"page sources missing under {a.out}: {missing}"
    assert page.stat().st_size < 15 * MB, f"index.html is {page.stat().st_size} bytes"
    total = page.stat().st_size + sum((a.out / s).stat().st_size for s in srcs)
    print(f"[RESULT] PASS {body.count('<h2>')} sections {len(srcs)} media files {total} bytes", flush=True)


if __name__ == "__main__":
    main()
