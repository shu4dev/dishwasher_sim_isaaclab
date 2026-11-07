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

Version 18 (plans/2026-10-27-bowls-lip-plates-fix.md): tables and media only. The setup table, the Part B table (one row per
round), "Grasping: summary", then one section per round (ROUNDS): replay, landed still, goal still, per-dish table in a
collapsed <details>. The per-kind sections, the drop test, Verdicts and Sources are no longer built (their builders stay).
Version 19 (plans/2026-10-27-carry-rules.md): the bowls carried from the nearest home branch (D-lip2) and the plates with the
checked retreat (I3) on top; round 9's two sections dropped (TABLE_ONLY: their Part B rows stay); the bowl table compares
round 9 and 10 per bowl (start branch, spin, height, largest travel, travel sum, pan swing, legs).
Version 20 (plans/2026-10-28-perch-release.md): the bowls carried tilted steeper than their goal, lowered until the emulated
flange force sensor trips and let go there (D-perch) on top, with the four Kit-free checks of 2026-10-28 in a details block
(data/results/robot/kitfree_v1); round 10's bowl section dropped (TABLE_ONLY: its Part B row stays).
Version 21 (plans/2026-10-28-plate-seat.md window 2, plans/2026-10-29-bowl-last.md): a verdict table on top of Part B (one row
per kind); the bowls carried to the combo with the lowest predicted release, lowered on the sensor, the hand taken away before
the settle (D-seat) on top, then the plates in the mirrored base-face hold lowered along the tine axis on the sensor (I4); round
11's bowl section and round 10's plate section dropped (TABLE_ONLY: their Part B rows stay); the Kit-free block (now in the bowl
section) gains (e) the bowl insertion scan, (f) the plate stops, (g) the mount search and (h) the plate hold study.
Version 21 also carries round 14 (plans/2026-10-29-sampler-release.md): the verdict table's columns feasible final pose, highest
seating drop and lowest hand-clear release; the drop curve (data/results/robot/drop_curve: every dish alone dropped from the
sampler's release orientation, 20-150 mm above the rack stop); the final-pose table (kitfree_v1/sampler_release/
final_pose_table.json); the plates inserted the sampler's way (I5) and bowl_01 released the sampler's way (D-sampler) on top.
Version 22 (2026-10-29, the user's request): the verdict, drop-curve and final-pose tables came off the page (too hard to read;
their builders stay); in their place "Best placement so far": the replay of the best plate run (I4) and the best bowl run
(D-perch) with one sentence each on how they did it (BEST, best_block).
Version 23 (plans/2026-10-29-perch-roll.md): the bowls released on the tine-tip perch at the dish-alone perch-roll curve's best
tilt with a jaw check (D-roll) on top, its section carrying the curve table (data/results/robot/perch_roll) and the jaw table;
the bowls' best clip and sentence in "Best placement so far" switch to D-roll only when it seats more than 3 of 7 (seated on the
goal's wires and mouth down).
Version 24 (plans/2026-10-29-foot-ring-grasp.md): the bowl mouth down on the counter, held by its measured foot ring (the v3
asset): in "Grasping: summary" two Part A rows (F-out, the pads close on the ring's outer face; F-in, the closed fingertips open
on its inner face: held k/n, the contacts, the in-hand pose; placing_v1/partA_bowl_foot_<side>/inhand.json), one gate clip, and
the Kit-free goal table (kitfree_v1/foot_goal/foot_goal.json) in a details block; nothing else moves.
Version 25 (plans/2026-10-29-ring-place.md): the setup table's hold row is the user's Part B assumptions (cups: side grasp,
mouth up or down; bowls: mouth down, foot-ring grasp; plates: face up, rim pinch; held, never dropped during the movement); a
new round section on top (D-ring: the v4 bowl picked mouth down by the foot ring, carried mouth down with the hand above,
lowered onto its refreshed goal until the flange sensor trips, let go, the hand straight up): the clip and a per-bowl table
(stop vs prediction, jaw contact after the open, mouth, seated, AT_GOAL, the no-reach blockers, bowl_06's flagged 10 mm try);
the bowls' best clip and sentence on top switch to D-ring only when at least one of its bowls seats.
Version 26 (2026-10-30, the user's cut, ``--minimal``): two things only. "Grasping": one table, the default grasp per kind (the
Part B assumption), its start, held k/n and the other grasps tried with their k/n (the counter-spot gate records), and the bowl
and cup gate clips; "Best placement": per kind the best run's clip and one sentence with its counts from the rows. The full
builders stay in this file, unused by --minimal.
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
                "ul.fm li{margin:0 0 18px}.lead{font-size:16px;max-width:75ch}h4{margin:1.2em 0 .4em}"
                "details{margin:10px 0}summary{cursor:pointer;color:var(--accent);font-weight:600}"
                "table.setup th{width:9.5em;white-space:nowrap}"
                ".grid3{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,260px),1fr));gap:14px}"
                ".grid3 figcaption{font-size:14px}")


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
            f"<h3>Robot-free tines test</h3><p class=muted>A level bowl set straight onto the site and settled with no robot "
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
        if f.name.endswith("_m2"):                     # the posture fix's gate at the new mount: its own section
            continue
        rep = next(iter(sorted(f.glob("limits_*.json"))), None)
        rs = LP.rows(f / "trials.jsonl") or (json.loads(rep.read_text()).get("trials", []) if rep else [])
        out += [dict(r, run=f"{root.name}/{f.name}") for r in rs]
    return out


PART_A = (          # the user's simple grasp table (2026-10-27): object, grasp, the grasp_family rows it pools, the note after k/n
    ("bowl", "rim pinch", ("rim pinch",), "rests on a knuckle, pivots {pivot}°"),
    ("cup", "wall pinch (top-down)", ("wall pinch",), ""),
    ("cup", "side pinch, 11 mm or 45 mm under the rim",
     ("side pinch, pads 11 mm under the rim", "side pinch, pads 45 mm under the rim"), "pads only"),
    ("cup", "side pinch, 80 mm under the rim", ("side pinch, pads 80 mm under the rim",), "gripper hits the counter"),
    ("cup", "side pinch, wrist toward the base, 45 mm under the rim, counter edge -0.17",
     ("side pinch, pads 45 mm under the rim, wrist toward the base",), "no collision-free grasp: the forearm and wrist hit "
     "the counter or the cabinet"),
    ("plate", "rim pinch", ("rim pinch, planner clearance waived",), "and only with the clearance check off"),
    ("plate", "slide", ("slide",), ""),
    ("bowl", "foot ring, mouth down: the pads close on the ring's outer face (F-out)", ("foot ring, out",), "{foot_out}"),
    ("bowl", "foot ring, mouth down: the closed fingertips open on the ring's inner face (F-in)", ("foot ring, in",), "{foot_in}"),
)
FOOT_GOAL = Path(__file__).resolve().parents[3] / "data/results/robot/kitfree_v1/foot_goal/foot_goal.json"


def foot_note(proot, side):
    """Version 24: the contact facts and the in-hand pose of a foot ring gate (placing_v1/partA_bowl_foot_<side>/inhand.json)."""
    f = (proot / f"partA_bowl_foot_{side}/inhand.json") if proot else None
    if not f or not f.is_file():
        return ""
    s = json.loads(f.read_text())
    parts = [("pads only" if not s["knuckle_contact_trials"] else f"knuckle contact in {s['knuckle_contact_trials']} trials")]
    touched = sorted({x for k, v in (s.get("bowl_parts_touched_by") or {}).items() if k.startswith("pad") for x in v})
    if touched:
        parts.append("on the " + " and ".join(touched))
    if not s["held"] and s.get("jaw_rad"):
        pc = s.get("pad_contact") or {}
        parts.append(f"the jaws stop at {min(s['jaw_rad']):.2f}-{max(s['jaw_rad']):.2f} rad (planned contact "
                     f"{s['planned']['theta_contact_rad']:.2f}), the pads leave the ring after "
                     f"{pc.get('ticks_range', [0, 0])[1] / 120:.1f} s at most, the bowl stays on the counter")
    if s.get("pad_N_hold_median"):
        parts.append("pads " + " / ".join(f"{v:.0f}" for v in s["pad_N_hold_median"].values()) + " N at the hold")
    ih = s.get("inhand")
    if ih:
        o = ih["origin_mm"]
        parts.append(f"in hand: bowl origin ({o[0]:.1f}, {o[1]:.1f}, {o[2]:.1f}) mm from the TCP (planned (0, 0, "
                     f"{s['planned']['bowl_origin_below_tcp_mm']:.1f})), axis {ih['axis_off_tcp_z_deg']:.1f}° off")
    miss = {k: v for k, v in s["outcomes"].items() if k != "hold"}
    if miss:
        parts.append("not held: " + ", ".join(f"{k} {v}" for k, v in miss.items()))
    return "; ".join(parts)


def foot_goal_block():
    """Version 24: the Kit-free goal table of the foot holds (one row per bowl x hold x target x pose: as recorded, and lifted out
    of the bowl's penetration into the rack, a Kit-free approximation of a goal re-settled with the v3 bowl), collapsed."""
    if not FOOT_GOAL.is_file():
        return ""
    g = json.loads(FOOT_GOAL.read_text())
    body = []
    for r in g["rows"]:
        b = r["best"]
        clear = ("–" if b.get("min_clear_mm") is None else
                 f"{b['min_clear_mm']:.0f} mm{' (touching)' if b.get('overlap') else ''}, {html.escape(str(b.get('blocker')))}")
        why = "" if r["verdict"] == "pass" else html.escape(str(b.get("why") or ""))[:110]
        note = "; ".join(x for x in (r.get("pose_note"), b.get("held_descent_note")) if x)
        body.append(f"<tr><td>{r['bowl']}</td><td>{r['hold']}</td><td>{r['target']}</td><td>{r['variant']}</td>"
                    f"<td>{r['spins_reach']}/{g['spins']}</td><td>{clear}</td><td>{b['tcp_above_centroid_mm']:.0f} mm</td>"
                    f"<td><b>{r['verdict']}</b>{(': ' + why) if why else ''}</td><td class=muted>{html.escape(note)}</td></tr>")
    head = " · ".join(f"{h}{'' if g['holds'][h].get('measured') else ' (planned hold, not held in the gate)'}: "
                      + ", ".join(f"{k} {v}" for k, v in d.items()) for h, d in g["summary"].items())
    return (f"<details><summary>Foot ring holds at the easy_bowls goals (Kit-free): {html.escape(head)}</summary>"
            + table(("bowl", "hold", "target", "pose", "reach (spins)", "min clearance, blocker", "TCP above centroid", "verdict",
                     "note"), body)
            + "</details>")


def grasping_block(trials, grid, measured, man=(), mdir=None, out=None, proot=None):
    """Part A: the user's simple grasp table (PART_A; k/n from the counter-spot gate records, "not run" until a run lands;
    the cup's earlier rim pinch is left out), the counter grid as a footnote, the cup side-pinch gate clips
    (videos.json entries of placing_v1/partA_cup_side_*)."""
    by, on = {}, {}
    for r in trials:
        if r.get("mode") != "grasp_gate":
            continue
        if r.get("spot"):
            on.setdefault(r["kind"], []).append(r)
            continue
        g = (r.get("grasp_family") or "rim pinch") + (", planner clearance waived" if r.get("waive_dish_clearance") else "")
        by.setdefault((r["kind"], g), []).append(r)
    body = []
    for kind, label, fams, note in PART_A:
        groups = [by.get((kind, f), []) for f in fams]
        if not any(groups):
            body.append(f"<tr><td>{kind}</td><td>{html.escape(label)}</td><td class=muted>not run</td></tr>")
            continue
        kn = [f"{sum(bool(r['g3']) for r in rs)}/{len(rs)}" for rs in groups if rs]
        held = f"{kn[0]} each" if len(kn) > 1 and len(set(kn)) == 1 else " and ".join(kn)
        piv = LP.med([(r.get("hold") or {}).get("settle_rotation_deg") for rs in groups for r in rs if r["g3"]])
        txt = note.format(pivot=f"{piv:.0f}" if piv is not None else "–", foot_out=foot_note(proot, "out"),
                          foot_in=foot_note(proot, "in"))
        body.append(f"<tr><td>{kind}</td><td>{html.escape(label)}</td><td><b>{held}</b>{', ' + html.escape(txt) if txt else ''}</td></tr>")
    foot = ""
    if on:
        ks = sorted(on, key=KINDS.index)
        spots = {(r["spot"]["i"], r["spot"]["j"]) for k in ks for r in on[k]}
        kf = [f"{sum(x['gate'] == 'G2' for x in grid[k].values())}/{len(grid[k])}" for k in ks if grid.get(k)]
        kft = (f"{kf[0]} spots Kit-free" if len(set(kf)) == 1 and len(kf) == len(ks) else
               ", ".join(f"{k} {v} spots Kit-free" for k, v in zip(ks, kf))) if kf else "Kit-free not run"
        held = " and ".join(f"{k} {sum(bool(r['g3']) for r in on[k])}/{len(on[k])}" for k in ks)
        never = [k for k in KINDS if k not in on and grid.get(k) and not any(x["gate"] == "G2" for x in grid[k].values())]
        foot = (f"<p class=muted>Pick spot does not matter: {held} over the "
                f"{len({i for i, _ in spots})}x{len({j for _, j in spots})} counter grid, {kft}."
                + "".join(f" {k.capitalize()}s never pick: 0/{len(grid[k])} spots Kit-free." for k in never) + "</p>")
    clips = [e for e in man if str(e.get("load")).startswith("partA_cup_side") and (mdir / e["file"]).is_file()]
    foot = [e for e in man if str(e.get("load")).startswith("partA_bowl_foot") and (mdir / e["file"]).is_file()]
    clips += sorted(foot, key=lambda e: (e.get("outcome") != "pass", e["load"].endswith("_in")))[:1]   # Version 24: one clip
    figs = "".join(figure_html(use({"src": mdir / e["file"]}, out), str(e.get("caption") or e["file"]).split(":")[0])
                   for e in clips)                     # Version 18: one-line captions, no paragraph (the grid footnote went)
    return ("<section id=grasping><h2>Grasping: summary</h2>"
            + table(("object", "grasp", "held"), body)
            + (f"<div class=grid2>{figs}</div>" if figs else "") + foot_goal_block() + "</section>")


STEPS = ("S1", "S2", "S3", "S4", "S5", "S6")
VARIANTS = ("W", "S")                         # S: the cup side pinch (plans/2026-10-26-side-pinch.md), its column only where run;
                                              # rows and clips of any other variant (R, R-ideal) stay on disk, off the page (user,
                                              # 2026-10-26/27: the dish is assumed grasped in hand and never let go, so no regrasp)
HOLD_NAME = {"idealized rim hold": "assumed rim hold"}      # the plate's hold (no plate grasp exists), as the page names it
MOUTH_DOWN = ("The goals are mouth-down by design: the certified loads invert the bowls and cups, because the fair score "
              "gives 0 to a dish that pools.")
LEGEND = ("Steps, first failure wins: S1 reorient in hand (IK), S2 pre-place reachable, S3 path and insertion, S4 insert "
          "and release, S5 settled within 15 mm / 20 mm / 15°, S6 no earlier dish disturbed. The dish is assumed held "
          "and never let go. W keeps the counter hold and turns the wrist. S (cups) holds the cup by the side pinch, the gripper beside it, at the measured "
          "in-hand pose of its best candidate (pad height x approach yaw with a collision-free IK at the counter pick and "
          "at the goal); a cup without one fails at S1 or S2 on the Kit-free check.")


def step_cell(r):
    if r is None:
        return "<span class=muted>not run</span>"
    if r.get("failed_at"):
        return f"<b>{html.escape(str(r['failed_at']))}</b>: {html.escape(str(r.get('why') or ''))}"
    e = r.get("error") or {}
    return (f"placed ({LP.fmt(e.get('lateral_mm'))} mm lateral, {LP.fmt(e.get('dz_mm'))} mm height, "
            f"{LP.fmt(e.get('tilt_deg'))}° tilt)")


def s_cell(r):
    """Variant S: an S1 reason with an empty hits list means no IK solution exists at all at +0.10 m."""
    if r and r.get("failed_at") == "S1" and "commonest hits []" in str(r.get("why")):
        n = (r.get("candidates") or {}).get("pass_pick")
        return f"<b>S1</b>: no IK solution at +0.10 m for any of the {n} candidates"
    return step_cell(r)


DROP_LEAD = ("Each dish starts welded in the gripper at its counter hold (bowl rim pinch, cup wall pinch, plate assumed rim "
             "hold), is carried with that hold unchanged (TCP down, mouth up) to the lowest collision-free pose above its goal, "
             "and let go there; the benchmark's settle judges where it comes to rest.")
DROP_LEGEND = ("Settled: the dish's drift over the last 60 of the 150 settle ticks is at most 5 mm and 3°; nothing else gates "
               "it. Drop height: the dish centroid above the goal centroid when the jaws open, the lowest of 25-350 mm in 25 mm "
               "steps (12 yaws about the vertical) with a collision-free IK and a path, else no reach: no IK (none at any height "
               "or yaw) or no path (transit_to found no path to the first three such poses); the dish is skipped. "
               "Landed on: the dish's contact partners at rest. Distance to goal: the centroid's lateral and height offsets and "
               "the tilt between the dish axes. Mouth: the sign of the dish axis's z. Dishes go in the witness order and stay "
               "where they land; an earlier dish moved more than 10 mm or 20° by a later drop is listed, never a failure. The "
               "plates' hold is assumed (no plate pick exists): the plate gate's median hold, pivoted 38° in the hand. Bowls and "
               "the plate retry ran with the limits rounds' carried-dish settle tolerance (0.08 rad: a welded bowl lags wrist_3 "
               "by 0.044 rad, which the default 0.03 rad check reads as a blocked move); cups needed none. The plates without a "
               "path were retried with a 45 s RRT-Connect budget per joint plan (15 s otherwise). The counter's front edge was at "
               "y = -0.30 m (the benchmark's slab) in this run.")
SIDE_LEAD = ("Each cup starts welded in the gripper at the base-side pinch's hold (the side pinch with the wrist toward the "
             "cup's base, pads 45 mm under the rim), is carried flipped mouth down (cup axis straight down, the gripper "
             "horizontal beside it, the wrist above) to the lowest collision-free pose above its goal, and let go there.")
SIDE_LEGEND = ("Success: settled (drift over the last 60 of the 150 settle ticks at most 5 mm and 3°) and mouth down (the cup "
               "axis's z below 0); settled-any counts every settled cup. The counter's front edge is at y = -0.17 m in this run "
               "(pulled back from the benchmark's -0.30, so the extended racks end in front of it). Drop height: the cup "
               "centroid above the goal centroid when the jaws open, the lowest of 25-350 mm in 25 mm steps (12 yaws about the "
               "vertical) with a collision-free IK and a path, else no reach after six such poses without a path. Cups go in the "
               "witness order with the lower rack pushed in and stay where they land; an earlier cup moved more than 10 mm or 20° "
               "is listed, never a failure.")
LANDED = {"Cabinet": "tub (Cabinet)", "Ground": "room floor", "Counter": "counter", "Door": "door"}


def no_reach(r):
    """The kind of "no reach": no collision-free IK at any drop height or yaw, or IK but no path to any tried pose."""
    return "no IK" if "no collision-free IK" in str(r.get("why")) else "no path"


def drop_row(x, r):
    """One dish of the drop-test table: dish, kind, drop height, landed on, settled, distance to goal, mouth, moved."""
    cells = [html.escape(str(x["name"])), html.escape(str(x.get("kind")))]
    if r is None:
        return "<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "<td colspan=6 class=muted>not run</td></tr>"
    if r.get("failed_at") == "no reach":
        return ("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + f"<td><b>no reach</b> ({no_reach(r)})</td><td colspan=5 class=muted>"
                f"{html.escape(re.sub(r' None$', '', str(r.get('why') or '')))}</td></tr>")
    st, e = r.get("settle") or {}, r.get("error") or {}
    cells += [f"{r['drop_height_m'] * 1e3:.0f} mm",
              html.escape(", ".join(LANDED.get(n, n) for n in r.get("landed_on") or []) or "nothing"),
              "<b>yes</b>" if r.get("settled") else
              f"<b>no</b> (drift {LP.fmt(st.get('drift_mm'))} mm, {LP.fmt(st.get('drift_deg'))}°)",
              f"{LP.fmt(e.get('lateral_mm'))} mm lateral, {LP.fmt(e.get('dz_mm'))} mm height, {LP.fmt(e.get('tilt_deg'))}° tilt",
              f"{html.escape(str(r.get('mouth')))} (axis z {r.get('axis_z', 0.):.2f})",
              html.escape(", ".join(f"{m[0]} ({m[1]} mm, {m[2]}°)" for m in r.get("moved_earlier") or [])) or "none"]
    return "<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>"


FLIP_LEAD = ("Each bowl starts welded in the gripper at the bowl gate's measured rim pinch (its median hold) and is carried "
             "turned about the gripper's closing axis so that it hangs mouth down. Turned all the way (the full flip), the bowl "
             "falls onto the palm when the jaws open, so the carry tilts it partway: the bowl axis is set 0, 15, 30, 45, 60 or "
             "75° from straight down, both turn directions. A Kit-free check releases the bowl at each tilt and moves it 0-50 mm "
             "straight down in 5 mm steps. It must not touch the open gripper or wrist 3, nor wrists 1 and 2 at the drop "
             "posture. The smallest tilt that clears is used for every height and yaw of that bowl; when none clears, the "
             "bowl goes from the full flip and is flagged palm. The rest is the cups' drop: the bowl centroid over the goal "
             "centroid, 25-350 mm above it in 25 mm steps, 12 yaws about the vertical, the lowest free pose, the yaw with the "
             "smallest joint move, six poses before no reach, let go and settled.")
FLIP_LEGEND = ("Success: settled (drift over the last 60 of the 150 settle ticks at most 5 mm and 3°) and mouth down (the bowl "
               "axis's z below 0); settled-any counts every settled bowl. Tilt: the bowl axis's angle from straight down in "
               "the carry. The measured hold pivoted 21° about the TCP's x axis, so the axis is not square to the closing axis: "
               "the full flip leaves it 24° from straight down, and tilts 0 and 15 both give the full flip. Clearance: the "
               "smallest gap between the falling bowl and the gripper or wrists over the 50 mm, Kit-free (0 = touching). "
               "Carried-dish settle tolerance 0.08 rad (a welded bowl lags wrist 3 by 0.044 rad). The counter's front edge is "
               "at y = -0.17 m. Bowls go in the witness order and stay where they land; the rack not in use is pushed in.")
INSERT_LEAD = ("The hold is assumed: no plate pick exists (the plate rim pinch held 3/10 only with the clearance check off, "
               "the surface slide 0/25). Each plate starts welded at grasp.py's plate rim pinch: TCP 10 mm under the rim top, "
               "pads across the rim wall, zero pivot. The pinch sits at the rim point that is highest at the goal, so the "
               "plate hangs below the TCP at its goal lean and slot yaw. The gripper body is on the plate's mouth face, or on "
               "its base face with wrist 3 turned 180°. The plate is carried at its exact goal orientation (no yaw scan) to "
               "the lowest collision-free pose 25-350 mm above the goal, both gripper sides tried per height, the smaller "
               "joint move first. It is then lowered straight down in 5 mm steps until it touches the rack or a dish (the "
               "harness contact report), the arm is blocked, or it reaches the goal height. The jaws open there and the hand "
               "backs out along its approach.")
INSERT_LEGEND = ("Success: settled (drift over the last 60 of the 150 settle ticks at most 5 mm and 3°). Stop: contact (the "
                 "plate touched the rack or a dish), blocked (the arm stopped following), goal (lowered the full height). "
                 "Lean from goal: the angle between the final and goal plate axes. In slot: every plate point within 5 mm of "
                 "the slot's two tine rows and below the tine tops lies between the slot's two tine columns, each column "
                 "taken at the tine axis at that height, ± the tine radius. All 8 goal poses pass this test. "
                 "Carried-dish settle tolerance 0.08 rad. Plates go in the witness order with the upper rack pushed in and stay "
                 "where they land; a side switch re-welds the plate at home (one teleport).")
NEW_MOUNT = ("<p class=muted>Mount: the pedestal at the posture fix's searched mount ({m}), the start posture the "
             "collision-free elbow-up home IK with the smallest max |q| (ties: nearest HOME_Q).</p>")


def mount_line(rs):
    m = next((r.get("mount") for r in rs.values() if r.get("mount")), None)
    where = f"({', '.join(f'{v:.2f}' for v in m[:3])}) m, yaw {m[3]:.2f} rad" if m and len(m) > 3 else "posture_v1.json"
    return NEW_MOUNT.format(m=where)


def start_w3(rs):
    v = sorted({round(r["reset"]["q_start_deg"][5]) for r in rs.values() if (r.get("reset") or {}).get("q_start_deg")})
    return ", ".join(f"{x}°" for x in v) if v else "–"


def contact_events(rs, names=("arm_or_palm_contact",)):
    """{dish: [(phase, partner pair, force N)]} of the harness invariants in the rows' trial log (trials/trial.jsonl beside
    the rows' trajectories), first occurrence per pair."""
    out = {}
    paths = {(LP.ROOT / r["traj"]).parent / "trial.jsonl" for r in rs.values() if r.get("traj")}
    for path in paths:
        if not path.is_file():
            continue
        with open(path) as f:
            for line in f:
                if '"event": "invariant"' not in line:
                    continue
                e = json.loads(line)
                pl = e.get("payload") or {}
                if pl.get("name") in names and e.get("dish") in rs:
                    d = pl.get("detail") or {}
                    out.setdefault(e["dish"], []).append((e.get("phase"), "-".join(d.get("pair") or []), d.get("force_N")))
    return out


def flip_scan_cell(r):
    """Per tilt the smallest clearance of the fall check (Kit-free), both turn directions; the chosen tilt in bold."""
    by = {}
    for s_ in r.get("flip_scan") or []:
        if "same_as_tilt_deg" in s_:
            continue
        fc = s_.get("at_drop_posture") if isinstance(s_.get("at_drop_posture"), dict) else s_.get("gripper_wrist3") or {}
        by.setdefault(s_["tilt_deg"], []).append(f"{LP.fmt(fc.get('min_mm'))} mm")
    return "; ".join((f"<b>{t:.0f}°</b>" if t == r.get("flip_tilt_deg") else f"{t:.0f}°") + ": " + " / ".join(v)
                     for t, v in sorted(by.items()))


def media_of(man, mdir, out, load, var, prefix):
    es = {e["file"].split("_")[0]: e for e in man if e.get("load") == load and mdir and (mdir / e["file"]).is_file()
          and e.get("variant") == ("D" if e["file"].startswith("goal_") else var)}
    figs = "".join(figure_html(use({"src": mdir / es[t]["file"]}, out), es[t].get("caption") or es[t]["file"])
                   for t in ("goal", "landed") if t in es)
    clip = (figure_html(use({"src": mdir / es[prefix]["file"]}, out), es[prefix].get("caption") or "") if prefix in es
            else "<p class=muted>replay: none recorded</p>")
    return (f"<div class=grid2>{figs}</div>" if figs else "<p class=muted>stills: none recorded</p>") + clip


def flip_block(root, mdir, out, man):
    """Part B's first section (plans/2026-10-27-bowls-plates-down.md): easy_bowls' D-flip rows (the last per bowl)."""
    head = "<section id=bowl-flip><h2>Bowls: rim pinch, partial flip, mouth-down drop</h2>"
    d = root / "easy_bowls" if root is not None else None
    rs = {r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "D-flip"} if d and d.is_dir() else {}
    if not rs:
        return head + "<p class=muted>not run</p></section>"
    dishes = sorted(json.loads((d / "load.json").read_text())["dishes"], key=lambda x: x.get("order_index", 0))
    k = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in rs.values())
    m = sum(bool(r.get("settled")) for r in rs.values())
    gone = Counter(no_reach(r) for r in rs.values() if r.get("failed_at") == "no reach")
    palm = sum(r.get("flip_clear") is False for r in rs.values())
    tilts = ", ".join(f"{x['name']} {rs[x['name']].get('flip_tilt_deg', 0):.0f}°" for x in dishes if x["name"] in rs)
    lead_ = (f"Settled mouth-down {k}/{len(rs)}, settled-any {m}/{len(rs)}"
             + (f"; no reach {sum(gone.values())} (" + ", ".join(f"{v} {w}" for w, v in sorted(gone.items())) + ")" if gone else "")
             + f". Tilt used: {tilts}" + (f"; {palm} flagged palm (no tilt cleared)" if palm else "")
             + f". Wrist 3 at the start: {start_w3(rs)}.")
    hold = next((r.get("in_hand") or {} for r in rs.values()), {})
    hold_s = (f"Hold: {hold.get('source')} (trial {hold.get('trial')}, pivot {LP.fmt(hold.get('pivot_deg'))}°), turned "
              f"about its closing axis in the carry." if hold else "")
    ev = contact_events(rs)
    own = {n: [e for e in v if n in e[1].split("-")] for n, v in ev.items()}
    own = {n: v for n, v in own.items() if v}
    other = [f"{n} {ph}: {pr} ({LP.fmt(fN, 0)} N)" for n, v in sorted(ev.items()) for ph, pr, fN in v if n not in pr.split("-")]
    if own:
        phases = sorted({ph for v in own.values() for ph, _, _ in v})
        parts_ = sorted({x for n, v in own.items() for _, pr, _ in v for x in pr.split("-") if x != n})
        lead_ += (f" The harness logged the bowl touching the robot (arm_or_palm_contact) for {len(own)}/{len(rs)} bowls, "
                  f"in the {' and '.join(phases)} phases: {', '.join(parts_)}. The Kit-free check moved the bowl straight "
                  f"down against the gripper and wrists only; it did not model the bowl tipping as the jaws open, nor the "
                  f"forearm.")
    if other:
        lead_ += f" Other robot contacts: {'; '.join(other)}."
    body = []
    for x in dishes:
        r = rs.get(x["name"])
        cells = [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "<td colspan=8 class=muted>not run</td></tr>")
            continue
        tilt = (f"{r.get('flip_tilt_deg', 0):.0f}° (axis {LP.fmt(r.get('flip_axis_tilt_deg'))}°, turn "
                f"{LP.fmt(r.get('flip_turn_deg'))}°)" + ("" if r.get("flip_clear") else " <b>palm</b>"))
        cells += [tilt, flip_scan_cell(r)]
        if r.get("failed_at") == "no reach":
            body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + f"<td><b>no reach</b> ({no_reach(r)})</td><td colspan=5 "
                        f"class=muted>{html.escape(re.sub(r' None$', '', str(r.get('why') or '')))}</td></tr>")
            continue
        st, e = r.get("settle") or {}, r.get("error") or {}
        p_ = r.get("posture") or {}
        cells += [f"{r['drop_height_m'] * 1e3:.0f} mm",
                  html.escape(", ".join(LANDED.get(n, n) for n in r.get("landed_on") or []) or "nothing"),
                  "<b>yes</b>" if r.get("settled") else f"<b>no</b> (drift {LP.fmt(st.get('drift_mm'))} mm, {LP.fmt(st.get('drift_deg'))}°)",
                  f"{html.escape(str(r.get('mouth')))} (axis z {r.get('axis_z', 0.):.2f})",
                  f"{LP.fmt(e.get('lateral_mm'))} mm lateral, {LP.fmt(e.get('dz_mm'))} mm height, {LP.fmt(e.get('tilt_deg'))}° tilt",
                  deg_cells(p_["max_abs_deg"]) if p_.get("max_abs_deg") else "–"]
        body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>")
    moved = [f"{n} moved {', '.join(f'{m_[0]} ({m_[1]} mm, {m_[2]}°)' for m_ in r.get('moved_earlier'))}"
             for n, r in rs.items() if r.get("moved_earlier")]
    return (head + mount_line(rs) + f"<p class=lead>{FLIP_LEAD}</p><p class=lead>{html.escape(lead_)}</p>"
            f"<p class=muted>{FLIP_LEGEND} {html.escape(hold_s)}</p>"
            + media_of(man, mdir, out, "easy_bowls", "D-flip", "drop")
            + table(("bowl", "slot", "tilt used", "clearance per tilt (both directions)", "drop height", "landed on", "settled",
                     "mouth", "distance to goal", "max |q|: pan, lift, elbow, w1, w2, w3"), body)
            + (f"<p class=muted>Earlier bowls moved by a later drop: {html.escape('; '.join(moved))}.</p>" if moved else "")
            + "</section>")


def insert_block(root, mdir, out, man):
    """Part B's second section (plans/2026-10-27-bowls-plates-down.md): easy_plates' I rows (the last per plate)."""
    head = "<section id=plate-insert><h2>Plates: inserted from the top edge, assumed hold</h2>"
    d = root / "easy_plates" if root is not None else None
    rs = {r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "I"} if d and d.is_dir() else {}
    if not rs:
        return head + "<p class=muted>not run</p></section>"
    dishes = sorted(json.loads((d / "load.json").read_text())["dishes"], key=lambda x: x.get("order_index", 0))
    k = sum(bool(r.get("settled")) for r in rs.values())
    sl = sum(bool(r.get("in_slot")) for r in rs.values())
    gone = Counter(no_reach(r) for r in rs.values() if r.get("failed_at") == "no reach")
    stops = Counter(r.get("stop_reason") for r in rs.values() if r.get("stop_reason"))
    lead_ = (f"With the assumed hold: settled {k}/{len(rs)}, in slot {sl}/{len(rs)}"
             + (f"; no reach {sum(gone.values())} (" + ", ".join(f"{v} {w}" for w, v in sorted(gone.items())) + ")" if gone else "")
             + (f". Stops: {', '.join(f'{w} {v}' for w, v in sorted(stops.items()))}" if stops else "")
             + f". Wrist 3 at the start: {start_w3(rs)}.")
    ev = contact_events(rs)
    if ev:
        lead_ += " Robot contacts logged by the harness: " + "; ".join(
            f"{n}: " + ", ".join(f"{pr} ({ph}, {LP.fmt(fN, 0)} N)" for ph, pr, fN in v) for n, v in sorted(ev.items())) + "."
    body = []
    for x in dishes:
        r = rs.get(x["name"])
        cells = [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "<td colspan=9 class=muted>not run</td></tr>")
            continue
        if r.get("failed_at") == "no reach":
            body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + f"<td><b>no reach</b> ({no_reach(r)})</td><td colspan=8 "
                        f"class=muted>{html.escape(re.sub(r' None$', '', str(r.get('why') or '')))}</td></tr>")
            continue
        st, e, p_ = r.get("settle") or {}, r.get("error") or {}, r.get("posture") or {}
        ld = r.get("lean_deg") or {}
        sw = r.get("side_switches") or 0
        cells += [html.escape(str(r.get("side"))) + (f" ({sw} side switch{'es' if sw > 1 else ''})" if sw else ""),
                  f"{r['start_height_m'] * 1e3:.0f} mm",
                  f"{r.get('lowered_mm')} mm, <b>{html.escape(str(r.get('stop_reason')))}</b>"
                  + (f" ({html.escape(', '.join(r.get('stop_partners') or []))})" if r.get("stop_partners") else ""),
                  "<b>yes</b>" if r.get("settled") else f"<b>no</b> (drift {LP.fmt(st.get('drift_mm'))} mm, {LP.fmt(st.get('drift_deg'))}°)",
                  f"{LP.fmt(r.get('lean_from_goal_deg'))}° (lean {LP.fmt(ld.get('final'))}° vs {LP.fmt(ld.get('goal'))}°)",
                  f"{LP.fmt(e.get('lateral_mm'))} mm lateral, {LP.fmt(e.get('dz_mm'))} mm height",
                  html.escape(", ".join(LANDED.get(n, n) for n in r.get("landed_on") or []) or "nothing"),
                  "<b>yes</b>" if r.get("in_slot") else "no",
                  deg_cells(p_["max_abs_deg"]) if p_.get("max_abs_deg") else "–"]
        body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>")
    moved = [f"{n} moved {', '.join(f'{m_[0]} ({m_[1]} mm, {m_[2]}°)' for m_ in r.get('moved_earlier'))}"
             for n, r in rs.items() if r.get("moved_earlier")]
    return (head + mount_line(rs) + f"<p class=lead>{INSERT_LEAD}</p><p class=lead>{html.escape(lead_)}</p>"
            f"<p class=muted>{INSERT_LEGEND}</p>" + media_of(man, mdir, out, "easy_plates", "I", "insert")
            + table(("plate", "slot", "gripper side", "start height", "lowered, stop", "settled", "lean from goal",
                     "distance to goal", "landed on", "in slot", "max |q|: pan, lift, elbow, w1, w2, w3"), body)
            + (f"<p class=muted>Earlier plates moved by a later insertion: {html.escape('; '.join(moved))}.</p>" if moved else "")
            + "</section>")


POSTURE_LEAD = ("The cup drop of the next section again, under the posture rules of plans/2026-10-27-posture-fix.md: every IK "
                "solution wrapped to ±180° and unwound toward the current joints only inside ±190° (the elbow ±180°), so no "
                "joint winds past a turn; elbow-up solutions only; within the lowest free drop height, the yaw with the smallest "
                "joint move; RRT-Connect shortcutting 10 s (2 s before); and the pedestal picked by a Kit-free search of 25 "
                "mounts. Everything else as the next section: the counter edge at y = -0.17 m, 12 yaws, 25-350 mm, six poses "
                "before no reach, the witness order, the lower rack pushed in, the same home TCP and 1.25 m safe height.")
JOINTS = ("pan", "lift", "elbow", "wrist 1", "wrist 2", "wrist 3")
OLD_MOUNT = "<p class=muted>Old mount: the pedestal at (0.45, -0.55, 0.90) m, yaw 2.60 rad.</p>"
MOUNT_FILE = "data/results/robot/mount/posture_v1.json"
POSTURE_GATE = "partA_cup_side_base45_m2"


def deg_cells(v, flag=180.):
    return ", ".join(f"<b>{x:.0f}°</b>" if x > flag else f"{x:.0f}°" for x in v)


def trial_max_q(path):
    """Max |q| per arm joint (deg) over a trial log's samples (round 6 logged no posture dict)."""
    mx = None
    if not path.is_file():
        return None
    with open(path) as f:
        for line in f:
            if '"event": "sample"' not in line:
                continue
            q = (json.loads(line).get("payload") or {}).get("q")
            if q:
                a = [abs(float(v)) * 180. / 3.141592653589793 for v in q[:6]]
                mx = a if mx is None else [max(x, y) for x, y in zip(mx, a)]
    return mx


def posture_block(root, mdir, out, man):
    """Part B's first section (plans/2026-10-27-posture-fix.md): easy_cups' D-side-m2 rows (the last per cup), the mount
    table of posture_v1.json, the joint range and carry travel per cup, the drop table, the replay and the landed still."""
    mf = LP.ROOT / MOUNT_FILE
    mt = json.loads(mf.read_text()) if mf.is_file() else None
    best = (mt or {}).get("best") or {}
    low = bool(best) and not best.get("baseline")
    head = (f"<section id=posture-drop><h2>Cups: side pinch, mouth-down drop at the "
            f"{'lower' if low or not best else 'searched (today’s)'} mount</h2>")
    d = root / "easy_cups" if root is not None else None
    rs = {r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "D-side-m2"} if d and d.is_dir() else {}
    if not rs and not mt:
        return head + "<p class=muted>not run</p></section>"
    parts = [f"<p class=lead>{POSTURE_LEAD}</p>"]
    if rs:
        dishes = sorted(json.loads((d / "load.json").read_text())["dishes"], key=lambda x: x.get("order_index", 0))
        k = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in rs.values())
        m = sum(bool(r.get("settled")) for r in rs.values())
        gone = Counter(no_reach(r) for r in rs.values() if r.get("failed_at") == "no reach")
        ps = [r["posture"] for r in rs.values() if r.get("posture")]
        mx = [max(p["max_abs_deg"][j] for p in ps) for j in range(6)] if ps else None
        past = [JOINTS[j] for j in range(6) if mx and mx[j] > 180.]
        up = all(p["elbow_up"] for p in ps) if ps else None
        lead_ = (f"Settled mouth-down {k}/{len(rs)}, settled-any {m}/{len(rs)}"
                 + (f"; no reach {sum(gone.values())} (" + ", ".join(f"{v} {w}" for w, v in sorted(gone.items())) + ")" if gone else "")
                 + (f". Max |q| over the run: {', '.join(f'{n} {v:.0f}°' for n, v in zip(JOINTS, mx))}; "
                    f"{'past 180°: ' + ', '.join(past) if past else 'no joint past 180°'}; elbow up throughout: "
                    f"{'yes' if up else 'no'}" if mx else "") + ".")
        parts.append(f"<p class=lead>{html.escape(lead_)}</p>")
    if best:
        tb = (mt.get("baseline") or {})
        parts.append(f"<p class=muted>Mount: the search picked ({', '.join(f'{v:.2f}' for v in best['base'])}) m, yaw "
                     f"{best['yaw_rad']:.2f} rad, score {best['score']} (goals {best['goals_ok']}/23, picks {best['picks_ok']}/2); "
                     f"today's mount ({', '.join(f'{v:.2f}' for v in tb.get('base') or [])}) m, yaw "
                     f"{tb.get('yaw_rad', 0.):.2f} rad, scored {tb.get('score', '–')}. Rule: {html.escape(mt['rule'])}. Searched "
                     f"in {mt['seconds'] / 60:.0f} min ({mt['workers']} workers, cap {mt['cap_s'] / 60:.0f} min).</p>")
        body = []
        for r in mt["table"]:
            tag = (" <b>picked</b>" if r.get("picked") else "") + (" (today)" if r.get("baseline") else "")
            mnt = f"({', '.join(f'{v:.2f}' for v in r['base'])}){tag}"
            if r.get("status") != "done":
                body.append(f"<tr><td>{mnt}</td><td>{r['yaw_rad']:.2f}</td><td colspan=6 class=muted>"
                            f"{html.escape(str(r.get('status')))}</td></tr>")
                continue
            gl = r.get("goals_per_load") or {}
            pk = r.get("picks") or {}
            cells = [mnt, f"{r['yaw_rad']:.2f}", f"<b>{r['score']}</b>" if r.get("picked") else str(r["score"]),
                     " / ".join(str(gl.get(L, "–")) for L in ("easy_bowls", "easy_plates", "easy_cups")),
                     " / ".join("yes" if (pk.get(n) or {}).get("ok") else "no" for n in ("bowl rim pinch", "cup wall pinch")),
                     f"{len(r.get('base_side_yaws') or [])}/12",
                     "yes" if r.get("start_free_with_cup") else "<b>no</b>" + ("" if r.get("home_ik") else " (no home IK)"),
                     f"{r.get('seconds', 0.):.0f} s"]
            body.append("<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>")
        parts.append("<h3>Mount search (Kit-free)</h3>" + table(
            ("mount (x, y, z) m", "yaw rad", "score", "goals with a drop IK: bowls / plates / cups", "picks: bowl rim / cup wall",
             "base-side yaws", "episode start", "time"), body))
    gate = root / POSTURE_GATE if root is not None else None
    grs = LP.rows(gate / "trials.jsonl") if gate and (gate / "trials.jsonl").is_file() else []
    hold = next((r.get("in_hand") or {} for r in rs.values()), {}).get("source")
    parts.append("<p class=muted>Base-side gate at this mount: "
                 + (f"held {sum(bool(r.get('g3')) for r in grs)}/{len(grs)} (n = 10 as round 6)" if grs else
                    "not run (the Kit-free pre-check passed no yaw at the picked mount)")
                 + (f". Hold: {html.escape(str(hold))}." if hold else ".") + "</p>")
    if rs:
        body = []
        for x in dishes:
            r = rs.get(x["name"])
            p = (r or {}).get("posture")
            if not p:
                body.append(f"<tr><td>{html.escape(x['name'])}</td><td colspan=4 class=muted>not run</td></tr>")
                continue
            body.append(f"<tr><td>{html.escape(x['name'])}</td><td>{deg_cells(p['max_abs_deg'])}</td>"
                        f"<td>{'yes' if p['elbow_up'] else '<b>no</b> (' + str(p['elbow_down_ticks']) + ' ticks)'}</td>"
                        f"<td>{deg_cells(p['carry_travel_deg'], flag=1e9)}</td><td>{p['carry_travel_sum_deg']:.0f}°</td></tr>")
        if ps:
            body.append(f"<tr><td><b>whole run</b></td><td>{deg_cells(mx)}</td><td>{'yes' if up else '<b>no</b>'}</td>"
                        f"<td colspan=2 class=muted>–</td></tr>")
        old = trial_max_q(root / "c1_easy_cups_Dside" / "trials" / "trial.jsonl")
        if old:
            body.append(f"<tr><td>the next section's run (old mount and rules)</td><td>{deg_cells(old)}</td>"
                        f"<td colspan=3 class=muted>from its trial log's joint samples (no posture record then)</td></tr>")
        parts.append("<h3>Joint range and travel</h3><p class=muted>Max |q| per joint over each cup's trial (reset at home, "
                     "carry, release, retreat; joints in the order shoulder pan, shoulder lift, elbow, wrist 1, wrist 2, wrist "
                     "3; bold = past 180°). Carry travel: the summed |Δq| per joint from the drop's start to the release, every "
                     "try. Elbow up: shoulder lift in (-180°, 0°) and elbow in (0°, 180°) at every tick.</p>"
                     + table(("cup", "max |q|: pan, lift, elbow, w1, w2, w3", "elbow up", "carry travel per joint", "travel sum"),
                             body))
        es = {e["file"].split("_")[0]: e for e in man if e.get("load") == "easy_cups" and mdir and (mdir / e["file"]).is_file()
              and e.get("variant") == ("D" if e["file"].startswith("goal_") else "D-side-m2")}
        figs = "".join(figure_html(use({"src": mdir / es[t]["file"]}, out), es[t].get("caption") or es[t]["file"])
                       for t in ("goal", "landed") if t in es)
        clip = (figure_html(use({"src": mdir / es["drop"]["file"]}, out), es["drop"].get("caption") or "") if "drop" in es
                else "<p class=muted>replay: none recorded</p>")
        parts.append("<h3>Drops</h3>" + (f"<div class=grid2>{figs}</div>" if figs else "<p class=muted>stills: none recorded</p>")
                     + clip + table(("dish", "kind", "drop height", "landed on", "settled", "distance to goal", "mouth",
                                     "earlier cups moved"), [drop_row(x, rs.get(x["name"])) for x in dishes]))
    return head + "".join(parts) + "</section>"


def side_drop_block(root, mdir, out, man):
    """Part B's first section (plans/2026-10-27-cup-side-drop.md): easy_cups' D-side rows (the last per cup), the drop test's
    goal still beside the D-side landed still, the D-side replay and the per-cup table."""
    head = "<section id=side-drop><h2>Cups: side pinch, mouth-down drop</h2>"
    d = root / "easy_cups" if root is not None else None
    rs = {r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "D-side"} if d and d.is_dir() else {}
    if not rs:
        return head + "<p class=muted>not run</p></section>"
    dishes = sorted(json.loads((d / "load.json").read_text())["dishes"], key=lambda x: x.get("order_index", 0))
    k = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in rs.values())
    m = sum(bool(r.get("settled")) for r in rs.values())
    gone = Counter(no_reach(r) for r in rs.values() if r.get("failed_at") == "no reach")
    lead_ = (f"Settled mouth-down {k}/{len(rs)}, settled-any {m}/{len(rs)}"
             + (f"; no reach {sum(gone.values())} (" + ", ".join(f"{v} {w}" for w, v in sorted(gone.items())) + ")" if gone else "") + ".")
    es = {e["file"].split("_")[0]: e for e in man if e.get("load") == "easy_cups" and mdir and (mdir / e["file"]).is_file()
          and e.get("variant") == ("D" if e["file"].startswith("goal_") else "D-side")}
    figs = "".join(figure_html(use({"src": mdir / es[t]["file"]}, out), es[t].get("caption") or es[t]["file"])
                   for t in ("goal", "landed") if t in es)
    clip = (figure_html(use({"src": mdir / es["drop"]["file"]}, out), es["drop"].get("caption") or "") if "drop" in es
            else "<p class=muted>replay: none recorded</p>")
    hold = next((r.get("in_hand") or {} for r in rs.values()), {}).get("source")
    return (head + f"<p class=lead>{SIDE_LEAD}</p><p class=lead>{html.escape(lead_)}</p><p class=muted>{SIDE_LEGEND}"
            + (f" Hold: {html.escape(str(hold))}." if hold else "") + "</p>"
            + (f"<div class=grid2>{figs}</div>" if figs else "<p class=muted>stills: none recorded</p>") + clip
            + table(("dish", "kind", "drop height", "landed on", "settled", "distance to goal", "mouth", "earlier cups moved"),
                    [drop_row(x, rs.get(x["name"])) for x in dishes]) + "</section>")


def placing_block(root, mdir, out):
    """Part B = the drop test (plans/2026-10-27-drop-test.md, variant D): per load the lead "settled k/n", the goal still
    beside the as-landed still, the load replay and the per-dish table. Reads <load>/load.json, the variant D rows of
    dishes.jsonl (the last row per dish) and the media's videos.json; the W and S rows stay on disk, off the page."""
    head = "<section id=placing><h2>Drop test: the dish carried above its goal and let go</h2>"
    if root is None or not root.is_dir():
        return head + "<p class=muted>not run</p></section>"
    man = json.loads((mdir / "videos.json").read_text()) if mdir and (mdir / "videos.json").is_file() else []
    loads = ["easy_bowls", "easy_plates", "easy_cups"]
    loads += sorted(p.name for p in root.iterdir() if (p / "load.json").is_file() and p.name not in loads)
    parts, tot = [], Counter()
    for name in loads:
        d = root / name
        load = json.loads((d / "load.json").read_text()) if (d / "load.json").is_file() else {}
        D = {r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "D"}
        src = f" <span class=muted>({html.escape(str(load['source_pair']))})</span>" if load.get("source_pair") else ""
        dishes = sorted(load.get("dishes") or [], key=lambda x: x.get("order_index", 0))
        if not D:
            parts.append(f"<h3>{html.escape(name)}{src}</h3><p class=muted>not run</p>")
            continue
        k = sum(bool(r.get("settled")) for r in D.values())
        gone = Counter(no_reach(r) for r in D.values() if r.get("failed_at") == "no reach")
        tot.update(settled=k, dishes=len(D), **gone)
        lead = (f"{name}: settled {k}/{len(D)}" + (f", no reach {sum(gone.values())} ("
                                                   + ", ".join(f"{v} {w}" for w, v in sorted(gone.items())) + ")" if gone else "")
                + (f", dropped but not settled {len(D) - k - sum(gone.values())}" if len(D) - k - sum(gone.values()) else "") + ".")
        es = {e["file"].split("_")[0]: e for e in man if e.get("load") == name and e.get("variant") == "D"
              and (mdir / e["file"]).is_file()}
        figs = "".join(figure_html(use({"src": mdir / es[t]["file"]}, out), es[t].get("caption") or es[t]["file"])
                       for t in ("goal", "landed") if t in es)
        clip = figure_html(use({"src": mdir / es["drop"]["file"]}, out), es["drop"].get("caption") or "") if "drop" in es else \
            "<p class=muted>replay: none recorded</p>"
        parts.append(f"<h3>{html.escape(name)}{src}</h3><p class=lead>{html.escape(lead)}</p>"
                     + (f"<div class=grid2>{figs}</div>" if figs else "<p class=muted>stills: none recorded</p>") + clip
                     + table(("dish", "kind", "drop height", "landed on", "settled", "distance to goal", "mouth",
                              "earlier dishes moved"), [drop_row(x, D.get(x["name"])) for x in dishes]))
    total = (f"<p class=lead>Settled {tot['settled']}/{tot['dishes']} dishes over the loads"
             + (f"; no reach {tot['no IK'] + tot['no path']}: {tot['no IK']} without a collision-free IK at any drop height or "
                f"yaw, {tot['no path']} with one but no path to it" if tot["no IK"] + tot["no path"] else "") + ".</p>") if tot["dishes"] else ""
    return (head + f"<p class=lead>{DROP_LEAD}</p>{total}<p class=muted>{DROP_LEGEND}</p>" + "".join(parts)
            + "<p class=muted>The earlier placing attempts (the dish set at its exact goal pose, variants W and S: none placed) "
              "are in results/2026-10-26-placing-goal.md and results/2026-10-26-side-pinch.md.</p></section>")


# ---------------------------------------------------------------------------- Version 18 (plans/2026-10-27-bowls-lip-plates-fix.md):
# tables and media only. A setup table, a Part B table with one row per round, then per round the replay, the landed and goal
# stills and the per-dish table in a collapsed <details> block; one-line captions.
ROUNDS = (   # (section id, title, load, rows' variant, media prefix, hold, one-phrase note)
    ("bowl-ring", "Bowls: foot-ring hold, mouth-down carry, sensed stop on the refreshed goal", "easy_bowls", "D-ring", "drop",
     "foot ring, pads on the ring's outer face (the F-out gate's measured hold), the bowl mouth down",
     "v4 bowl, goals refreshed on v4 (each bowl alone, released 2 mm above its recorded goal); picked at the gate's counter "
     "spot, lifted 0.15 m, carried mouth down with the hand above, lowered along the rack's up axis at 20 mm/s until the "
     "emulated flange sensor reads > 3.5 N on 2 ticks, let go, 0.5 s, the hand straight up 100 mm, the rest wait, judged"),
    ("bowl-roll", "Bowls: released on the tine-tip perch at the curve's best tilt, jaw check", "easy_bowls", "D-roll", "drop",
     "rim pinch at the lip (measured hold), carried at the perch-roll curve's best tilt",
     "only bowls whose best tilt seated mouth down on >= 4 of 5 dish-alone drops; spins whose open hand the recorded rolling "
     "bowl never meets (2 mm); lowered at 20 mm/s until the emulated flange sensor reads > 3.5 N on 2 ticks, let go there, "
     "backed out, the rest wait, then judged"),
    ("plate-sampler", "Plates: the sampler's release, upright, sensed stop", "easy_plates", "I5", "insert",
     "assumed top-edge rim hold, mirrored (hand body on the base side), rim 6 mm into the pads, plate upright",
     "carried to 50 mm above the sampler's release pose, lowered at 20 mm/s until the emulated flange sensor reads > 3.5 N on "
     "2 ticks, raised 20 mm, let go, the hand straight up 0.10 m"),
    ("bowl-sampler", "Bowl_01: the sampler's release, sensed stop", "easy_bowls", "D-sampler", "drop",
     "rim pinch at the lip (measured hold), the bowl in its packed pose",
     "carried to 50 mm above the sampler's release pose, lowered at 20 mm/s until the emulated flange sensor trips, raised "
     "20 mm, let go, the hand straight up 0.10 m; the other six bowls placed at their goals (infeasible there)"),
    ("bowl-seat", "Bowls: lowest predicted release, sensed stop, hand away", "easy_bowls", "D-seat", "drop",
     "rim pinch at the lip (measured hold), carried at the tilt with the lowest predicted stop",
     "lowered at 20 mm/s until the emulated flange sensor reads > 3.5 N on 2 ticks, let go there, the hand taken away before "
     "the settle"),
    ("plate-seat", "Plates: seated along the tine axis, sensed stop", "easy_plates", "I4", "insert",
     "assumed top-edge rim hold, base face (mirrored across the rim wall), rim 6 mm into the pads",
     "lowered along the tine axis at 20 mm/s until the emulated flange sensor reads > 3.5 N on 2 ticks, let go there, the "
     "hand backing out"),
    ("bowl-perch", "Bowls: over-lean carry, sensed stop, perch-and-drop", "easy_bowls", "D-perch", "drop",
     "rim pinch at the lip (measured hold), carried steeper than the goal",
     "lowered at 20 mm/s until the emulated flange sensor reads > 3.5 N on 2 ticks, let go there, the hand backing out"),
    ("bowl-lip2", "Bowls: carried from the nearest home branch", "easy_bowls", "D-lip2", "drop",
     "rim pinch at the lip (measured hold)", "home branch and rim spin with the smallest largest-joint travel"),
    ("plate-insert-3", "Plates: inserted, checked retreat", "easy_plates", "I3", "insert",
     "assumed top-edge rim hold", "back-off through a checked line; rise only clear of the rim"),
    ("bowl-lip", "Bowls: lip pinch, dropped at the goal lean", "easy_bowls", "D-lip", "drop",
     "rim pinch at the lip (measured hold)", "2 mm held-dish margin; each bowl tipped on the finger as the jaws opened"),
    ("plate-insert-2", "Plates: inserted from the top edge, bugs fixed", "easy_plates", "I2", "insert",
     "assumed top-edge rim hold", "bugs fixed; the rise after the 25 mm back-off lifted plate_07 by its rim and laid it flat"),
    ("bowl-flip", "Bowls: rim pinch, partial flip, mouth-down drop", "easy_bowls", "D-flip", "drop",
     "rim pinch turned about the closing axis", "75° tilt; the bowls hit the palm or wrists as the jaws opened"),
    ("plate-insert", "Plates: inserted from the top edge, assumed hold", "easy_plates", "I", "insert",
     "assumed top-edge rim hold", "a face switch dropped the plate 0.75 s; plate_06's no reach was the quaternion bug"),
    ("posture-drop", "Cups: side pinch, mouth-down drop at the lower mount", "easy_cups", "D-side-m2", "drop",
     "side pinch, wrist toward the base, flipped", "posture fix: searched mount, joints within ±190°"),
    ("side-drop", "Cups: side pinch, mouth-down drop", "easy_cups", "D-side", "drop",
     "side pinch, wrist toward the base, flipped", "old mount (0.45, -0.55, 0.90)"),
)
DOWN = ("D-perch", "D-lip2", "D-lip", "D-flip", "D-side-m2", "D-side")          # settled and mouth down (D-lip2: recorded, the metric
                                                                        # is the joint travel)
TABLE_ONLY = ("D-lip2", "D-lip", "I2", "I3", "D-perch")   # Version 19: round 9's sections dropped, Version 20: round 10's
                                                 # bowls, Version 21: round 10's plates and round 11's bowls; Part B rows kept
PAIR = {"Cabinet": "tub", "UpperRack": "upper rack", "LowerRack": "lower rack", "SilverwareBasket": "basket",
        "Counter": "counter", "Pedestal": "pedestal", "Floor": "floor", "Door": "door"}


def pair_name(p):
    """'forearm_link|Cabinet' -> 'forearm vs tub'; 'held|forearm_link' -> 'held dish vs forearm'."""
    a, b = p.split("|")
    nm = lambda x: PAIR.get(x, x.replace("_link", "").replace("_", " "))
    return f"{'held dish' if a == 'held' else nm(a)} vs {nm(b)}"


def top_pairs(r, k=2):
    """A no-reach row's commonest blocking pairs (D-lip: one list; I2: per face)."""
    b = r.get("blocking") or {}
    if "pairs" in b:
        return ", ".join(f"{pair_name(p)} ({n}/{b['ik_solutions']})" for p, n in b["pairs"][:k])
    return "; ".join(f"{side}: " + ", ".join(f"{pair_name(p)} ({n}/{v['ik_solutions']})" for p, n in v["pairs"][:k])
                     for side, v in b.items())


def rows_of(root, load, var):
    d = root / load if root is not None else None
    return ({r["dish"]: r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == var} if d and d.is_dir() else {},
            sorted(json.loads((d / "load.json").read_text())["dishes"], key=lambda x: x.get("order_index", 0))
            if d and (d / "load.json").is_file() else [])


def result_of(var, rs):
    n = len(rs)
    k = sum(bool(r.get("settled")) for r in rs.values())
    if var in DOWN:
        dn = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in rs.values())
        return f"mouth down {dn}/{n}, settled {k}/{n}"
    if var == "I4":
        se = sum(bool((r.get("seated") or {}).get("seated")) for r in rs.values())
        return f"seated on the goal's wires {se}/{n}, settled {k}/{n}"
    if var == "D-seat":
        se = sum(bool((r.get("seated") or {}).get("seated")) for r in rs.values())
        aw = sum(bool(r.get("hand_away")) for r in rs.values())
        return f"seated mouth down {se}/{n}, hand away {aw}/{n}, settled {k}/{n}"
    if var == "D-roll":
        ran = [r for r in rs.values() if not r.get("placed_at_goal")]
        se = sum(bool((r.get("seated") or {}).get("seated")) for r in ran)
        dn = sum(r.get("mouth") == "down" for r in ran)
        jc = sum(r.get("failed_at") == "jaw contact" for r in ran)
        return (f"seated mouth down {se}/{len(ran)} lowered (mouth down {dn}), jaw contact after the open {jc}; "
                f"{n - len(ran)} not run (placed at the goal)")
    if var == "D-ring":
        ran = [r for r in rs.values() if not r.get("placed_at_goal")]
        se = sum(bool((r.get("seated") or {}).get("seated")) for r in ran)
        dn = sum(r.get("mouth") == "down" for r in ran)
        ag = sum(bool(r.get("at_goal")) for r in ran)
        jc = sum(bool((r.get("jaw_contact") or {}).get("after_open")) for r in ran)
        return (f"seated mouth down at their refreshed goals {se}/{len(ran)} carried, {se}/{n} overall (mouth down {dn}, "
                f"AT_GOAL {ag}); jaw watch clean {len(ran) - jc}/{len(ran)}; {n - len(ran)} no reach (placed at the goal)")
    if var in ("I5", "D-sampler"):
        ran = [r for r in rs.values() if not r.get("placed_at_goal")]
        se = sum(bool((r.get("seated") or {}).get("seated")) for r in ran)
        return (f"{'seated mouth down' if var == 'D-sampler' else 'seated on the goal wires'} {se}/{len(ran)}, settled "
                f"{sum(bool(r.get('settled')) for r in ran)}/{len(ran)}; {n - len(ran)} infeasible (placed at the goal)")
    return f"settled {k}/{n}, in slot {sum(bool(r.get('in_slot')) for r in rs.values())}/{n}"


def failures_of(rs):
    c = Counter(r.get("failed_at") for r in rs.values() if r.get("failed_at"))
    if "no perch" in c:                                # D-perch: the no-perch rows' commonest blocking pair
        blk = Counter(x["blocking"]["pairs"][0][0] for x in rs.values() if x.get("failed_at") == "no perch"
                      and (x.get("blocking") or {}).get("pairs"))
        n = c.pop("no perch")
        return "; ".join([f"no perch {n}" + (f" ({pair_name(blk.most_common(1)[0][0])})" if blk else "")]
                         + [x for x in [failures_of({k: v for k, v in rs.items() if v.get("failed_at") != "no perch"})]
                            if x != "none"])
    if any(r.get("placed_at_goal") for r in rs.values()):   # I5 / D-sampler / D-roll: not run, placed at the goal
        pl = Counter("infeasible at the sampler's release" if r.get("failed_at") == "no reach" else r.get("failed_at")
                     for r in rs.values() if r.get("placed_at_goal"))
        return "; ".join([f"{k_} {v_}" for k_, v_ in pl.most_common()]
                         + [x for x in [failures_of({a: b for a, b in rs.items() if not b.get("placed_at_goal")})] if x != "none"])
    gone = Counter(no_reach(r) for r in rs.values() if r.get("failed_at") == "no reach")
    out = []
    for f, n in sorted(c.items(), key=lambda x: -x[1]):
        if f == "no reach":
            blk = Counter(p for r in rs.values() if r.get("failed_at") == "no reach"
                          for p in ([x[0] for x in (r.get("blocking") or {}).get("pairs", [])[:1]] if "pairs" in (r.get("blocking") or {})
                                    else [v["pairs"][0][0] for v in (r.get("blocking") or {}).values() if v.get("pairs")][:1]))
            out.append(f"no reach {n} (" + ", ".join(f"{w} {v}" for w, v in sorted(gone.items()))
                       + (f"; {pair_name(blk.most_common(1)[0][0])}" if blk else "") + ")")
        else:
            out.append(f"{f} {n}")
    return "; ".join(out) or "none"


def setup_block(mt, starts):
    best = (mt or {}).get("best") or {}
    where = (f"({', '.join(f'{v:.2f}' for v in best['base'])}) m, yaw {best['yaw_rad']:.2f} rad (searched); the cup side drop "
             f"and grasping: (0.45, -0.55, 0.90) m, yaw 2.60 rad" if best.get("base") else "posture_v1.json")
    st = "; ".join(f"{k} {v}" for k, v in starts.items()) or "–"
    rows_ = (("arm", "UR5e on a pedestal, Isaac Sim 4.5 headless, simulation only"),
             ("gripper", "Robotiq 2F-85"),
             ("mount", where),
             ("posture rules", "elbow up, joints within ±190° (elbow ±180°); start = the free home IK with the smallest max "
                               "|q|, ties the smallest Σ|q − HOME_Q| (bowls since round 10: the free home branch of the best "
                               f"carry combo): {st}"),
             ("hold assumptions (Part B)", "cups start mouth up or mouth down with the side grasp (pads 11 mm under the "
                                           "rim); bowls start mouth down only, with the foot-ring grasp (pads on the ring's "
                                           "outer face); plates start face up only, with the rim pinch (top edge, hand on the "
                                           "base side, rim 6 mm in); every dish is held and never dropped during the movement"),
             ("counter", "edge y = -0.17 m"))
    return ("<table class=setup><tbody>" + "".join(f"<tr><th>{k}</th><td>{html.escape(v)}</td></tr>" for k, v in rows_)
            + "</tbody></table>")


def travel_down(root):
    """D-lip2 vs D-lip per reached bowl: (bowls reached in both, largest travel down, down by 30 deg or more)."""
    r10, _ = rows_of(root, "easy_bowls", "D-lip2")
    r9, _ = rows_of(root, "easy_bowls", "D-lip")
    pairs = [(max(r9[n]["posture"]["carry_travel_deg"]), r["carry_cost_deg"]) for n, r in r10.items()
             if r.get("carry_cost_deg") and n in r9 and r9[n].get("drop_height_m") is not None]
    return len(pairs), sum(b < a for a, b in pairs), sum(a - b >= 30. for a, b in pairs)


def partb_block(root):
    body = []
    for sid, title, load, var, _, hold, note in ROUNDS:
        rs, _ = rows_of(root, load, var)
        ex = []
        if var == "D-ring":                            # Version 25: the 7 rows of the load; the flagged try in the note
            rs, ex = ring_rows(root)
        name = html.escape(title) if var in TABLE_ONLY else f"<a href='#{sid}'>{html.escape(title)}</a>"
        if not rs and var == "D-roll" and (PERCH_ROOT / "drops.jsonl").is_file():   # Version 23: the gate skipped the robot
            dr = [json.loads(x) for x in (PERCH_ROOT / "drops.jsonl").read_text().splitlines() if x.strip()]
            dr = [x for x in dr if x.get("dropped")]
            body.append(f"<tr><td>{name}</td><td>{load}</td><td>{html.escape(hold)}</td>"
                        f"<td><b>robot round not run</b>: the dish-alone curve seated mouth down "
                        f"{sum(bool(x.get('seated')) for x in dr)}/{len(dr)} drops (mouth down "
                        f"{sum(x.get('mouth') == 'down' for x in dr)})</td><td>no bowl gated (best tilt under 4/5 for "
                        f"every bowl)</td><td>{html.escape(note)}</td></tr>")
            continue
        if not rs:
            body.append(f"<tr><td>{name}</td><td>{load}</td><td>{html.escape(hold)}</td>"
                        f"<td colspan=3 class=muted>not run</td></tr>")
            continue
        if var == "D-lip2":
            n, dn, d30 = travel_down(root)
            note = f"{note}: largest travel down on {dn}/{n} bowls, by 30° or more on {d30}/{n}"
        if var == "D-perch":
            n, k = perch_ok(rs)
            note = f"{note}; the stop within 2 mm of the predicted perch on {k}/{n} lowered bowls"
        if var in ("I4", "D-seat", "I5", "D-sampler", "D-roll", "D-ring"):
            kinds_ = Counter((r.get("stop") or {}).get("kind") for r in rs.values() if (r.get("stop") or {}).get("kind"))
            ok2 = sum(bool((r.get("stop_check") or {}).get("within_2mm")) for r in rs.values())
            note = (f"{note}; stops: " + ", ".join(f"{k} {v}" for k, v in sorted(kinds_.items()))
                    + f"; within 2 mm of the Kit-free prediction on {ok2}/{sum(kinds_.values())}" if kinds_ else
                    f"{note}; no plate reached its descent start, so the sensor never ran")
        if ex:
            note = f"{note}; " + "; ".join(f"flagged: {x['dish']} at the {x.get('arm_margin_mm'):.0f} mm arm margin "
                                          f"({x.get('racks')}): {ring_outcome(x)}" for x in ex)
        fails = ring_failures(rs) if var == "D-ring" else failures_of(rs)
        body.append(f"<tr><td>{name}</td><td>{load}</td><td>{html.escape(hold)}</td>"
                    f"<td><b>{result_of(var, rs)}</b></td><td>{html.escape(fails)}</td><td>{html.escape(note)}</td></tr>")
    return ("<h3>Part B: one row per round</h3>"
            + table(("round", "load", "hold", "result", "failure mode", "note"), body))


BEST = (   # Version 22: (load, rows' variant, replay prefix, how it was done) of the best placement so far per kind
    ("easy_plates", "I4", "insert", "Plates: held by the top edge (an assumed grip, no plate pick from the counter works), "
     "lowered along the tines' lean at 20 mm/s until the wrist sensed more than 3.5 N, then let go"),
    ("easy_bowls", "D-perch", "drop", "Bowls: pinched by the rim, carried tilted steeper than the goal, lowered at 20 mm/s "
     "until the wrist sensed more than 3.5 N on the tine tip, then let go"),
)


ROLL_BEST = ("easy_bowls", "D-roll", "drop", "Bowls: pinched by the rim, carried at the tilt where the bowl alone rolls "
             "into its seat, lowered at 20 mm/s until the wrist sensed more than 3.5 N on the tine tip, then let go and backed out")


RING_BEST = ("easy_bowls", "D-ring", "drop", "Bowls: picked mouth down by the foot ring, carried mouth down with the hand "
             "above, lowered onto the goal at 20 mm/s until the wrist sensed more than 3.5 N, then let go and the hand rose")


def best_rows(root):
    """BEST with the bowls' entry switched to D-roll when it seats more than 3 of 7 (Version 23), to D-ring when at least one
    of its bowls seats (Version 25; the flagged extra try not counted)."""
    rs, _ = rows_of(root, "easy_bowls", "D-roll")
    se = sum(bool((r.get("seated") or {}).get("seated")) for r in rs.values() if not r.get("placed_at_goal"))
    ring, _ = ring_rows(root)
    se_r = sum(bool((r.get("seated") or {}).get("seated")) for r in ring.values() if not r.get("placed_at_goal"))
    return tuple(RING_BEST if b[0] == "easy_bowls" and se_r >= 1 else ROLL_BEST if b[0] == "easy_bowls" and se > 3 else b
                 for b in BEST)


def best_block(root, mdir, out, man):
    """Version 22: the best placement so far per kind (BEST), its replay and one sentence: how it was done, then the count
    from the rows (I4: plates fully in their slots among the reachable ones; D-perch: bowls settled mouth down)."""
    figs = []
    for load, var, prefix, how in best_rows(root):
        rs, _ = rows_of(root, load, var)
        e = next((e for e in man if e.get("load") == load and e.get("variant") == var and e["file"].split("_")[0] == prefix
                  and e["file"].endswith(".mp4") and mdir and (mdir / e["file"]).is_file()), None)
        if not rs or e is None:
            continue
        if var == "D-ring":
            rs, _ = ring_rows(root)
        n = len(rs)
        if var == "D-ring":
            ran = [r for r in rs.values() if not r.get("placed_at_goal")]
            se = sum(bool((r.get("seated") or {}).get("seated")) for r in ran)
            jc = sum(bool((r.get("jaw_contact") or {}).get("after_open")) for r in ran)
            nr = [(d, r) for d, r in rs.items() if r.get("failed_at") == "no reach"]
            res = (f"{se} of {n} bowls seated mouth down at their refreshed goals: {se} of the {len(ran)} carried "
                   f"({', '.join(d for d, r in rs.items() if not r.get('placed_at_goal'))}), "
                   + ("no gripper contact after the open on any" if not jc else f"gripper contact after the open on {jc}")
                   + f"; {', '.join(d for d, _ in nr)} out of reach ("
                   + "; ".join(ring_blockers(r) for _, r in nr) + ")")
        elif var == "D-roll":
            ran = [r for r in rs.values() if not r.get("placed_at_goal")]
            se = sum(bool((r.get("seated") or {}).get("seated")) for r in ran)
            res = f"{se} of {n} bowls seated on their goal's wires mouth down ({len(ran)} gated by the roll curve)"
        elif var == "I4":
            reach = sum(r.get("failed_at") != "no reach" for r in rs.values())
            se = sum(bool((r.get("seated") or {}).get("seated")) for r in rs.values())
            res = (f"{se} of the {reach} plates the arm can reach ended fully in their slots; the other {n - reach} are out of "
                   f"reach (the forearm hits the tub)")
        else:
            dn = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in rs.values())
            res = f"{dn} of {n} landed mouth down (two of them resting on the open finger), none fully in its slot"
        rel = use({"src": mdir / e["file"]}, out)
        figs.append(f"<figure><video controls muted playsinline preload=metadata src='{html.escape(rel)}'></video>"
                    f"<figcaption>{html.escape(how + ': ' + res + '.')}</figcaption></figure>")
    return f"<h3>Best placement so far</h3><div class=grid2>{''.join(figs)}</div>" if figs else ""


DROP_ROOT = LP.ROOT / "data/results/robot/drop_curve"
AT_GOAL = {"bowl": (.015, .020, 15.), "cup": (.015, .020, 15.), "plate": (.018, .020, 16.)}   # frigidaire_bench.AT_GOAL
FINAL_POSE_JSON = LP.ROOT / "data/results/robot/kitfree_v1/sampler_release/final_pose_table.json"
LOADS = (("plate", "easy_plates"), ("bowl", "easy_bowls"), ("cup", "easy_cups"))


def drop_data(load):
    """The drop curve of one load: (rows of drops.jsonl, highest.json, report.json) or None when not run."""
    d = DROP_ROOT / load
    if not (d / "drops.jsonl").is_file():
        return None
    rows = [json.loads(x) for x in (d / "drops.jsonl").read_text().splitlines() if x.strip()]
    hi = json.loads((d / "highest.json").read_text()) if (d / "highest.json").is_file() else {}
    rp = json.loads((d / "report.json").read_text()) if (d / "report.json").is_file() else {}
    return rows, hi, rp


def final_pose():
    return json.loads(FINAL_POSE_JSON.read_text()) if FINAL_POSE_JSON.is_file() else None


def short(n):
    return n.split("_")[-1]


def drop_verdict(kind, load):
    """The verdict cell 'highest seating drop' of one kind: dishes seated at the sampler's 20 mm (every repeat / any) and the
    highest drop with every repeat seated."""
    dd = drop_data(load)
    if dd is None:
        return "not run"
    rows, hi, _ = dd
    n = len(hi)
    at20 = [d for d in hi if (hi[d].get("seated_by_gap") or {}).get("20", "0/0").split("/")[0] not in ("0",)]
    all20 = [d for d in hi if (lambda v: v and v.split("/")[0] == v.split("/")[1] and v.split("/")[1] != "0")(
        (hi[d].get("seated_by_gap") or {}).get("20"))]
    best = max(((v.get("highest_all_seated_mm") or 0, d) for d, v in hi.items()), default=(0, None))
    anyh = [d for d, v in hi.items() if v.get("highest_any_seated_mm") is not None]
    return (f"{len(all20)}/{n} seated on every repeat at the sampler's 20 mm, {len(at20)}/{n} on some; {len(anyh)}/{n} seat at "
            f"some height" + (f"; every repeat seated up to {best[0]} mm ({best[1]})" if best[1] and best[0] else ""))


def release_verdict(kind, fp):
    """The verdict cell 'lowest hand-clear release' of one kind from the final-pose table."""
    if fp is None:
        return "not run"
    rows = [r for r in fp["rows"] if r["kind"] == kind]
    ok = [r for r in rows if r["feasible"]]
    blk = Counter(re.sub(r", [0-9.]+ mm deep", "", re.sub(r"^hand blocked \((.*)\)$", r"\1", r["verdict"])) for r in rows
                  if not r["feasible"])
    lo_, hi_ = (min(r["release_above_goal_mm"] for r in ok), max(r["release_above_goal_mm"] for r in ok)) if ok else (0, 0)
    txt = (f"the sampler's release (20 mm above the rack stop, {lo_:.0f}" + (f"-{hi_:.0f}" if round(hi_) != round(lo_) else "")
           + f" mm above the goal) for {len(ok)}/{len(rows)}: {', '.join(short(r['dish']) for r in ok)}" if ok else
           f"none at the sampler's release (0/{len(rows)})")
    return txt + ("; blocked: " + ", ".join(f"{k} {v}" for k, v in blk.most_common()) if blk else "")


def verdict_block(root):
    """Version 21 (round 14): the verdict, one row per kind: graspable, reachable, feasible final pose (the hand at the
    sampler's release pose, Kit-free), highest seating drop (the drop curve, the dish alone), lowest hand-clear release, best
    result, limiting part."""
    fp = final_pose()
    feas = {k: (f"{sum(r['feasible'] for r in fp['rows'] if r['kind'] == k)}/{sum(r['kind'] == k for r in fp['rows'])}"
                if fp else "not run") for k in KINDS}
    ps5, _ = rows_of(root, "easy_plates", "I5")
    bs5, _ = rows_of(root, "easy_bowls", "D-sampler")
    ps4, _ = rows_of(root, "easy_plates", "I4")
    bs, _ = rows_of(root, "easy_bowls", "D-seat")
    cs, _ = rows_of(root, "easy_cups", "D-side-m2")
    ran = lambda rs: [r for r in rs.values() if not r.get("placed_at_goal")]                     # noqa: E731
    seat = lambda rs: sum(bool((r.get("seated") or {}).get("seated")) for r in rs)              # noqa: E731
    n_ps4 = seat(ps4.values())
    n_cd = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in cs.values())
    p5, b5 = ran(ps5), ran(bs5)
    best_p = "; ".join(x for x in (f"I5 (the sampler's way): seated on the goal wires {seat(p5)}/{len(p5)}" if p5 else "",
                                   f"I4 (tine axis, window 2): seated {n_ps4}/{sum(bool(r.get('stop')) for r in ps4.values())} "
                                   f"lowered" if ps4 else "") if x)
    best_b = "; ".join(x for x in (f"D-sampler (bowl_01): seated mouth down {seat(b5)}/{len(b5)}" if b5 else "",
                                   f"D-seat (round 13): seated mouth down {seat(bs.values())}/{len(bs)}" if bs else "") if x)
    rows_ = [("plate", "only with the planner's clearance check off: the rim pinch held 3/10, the slide 0/25 (the in-hand pose "
                       "is assumed)", "the front bank; the rear bank: the forearm in the tub at every release", feas["plate"],
              drop_verdict("plate", "easy_plates"), release_verdict("plate", fp), best_p or "not run",
              "rear bank: the forearm (80-93 mm into the tub at the sampler's release); front bank: the plate's own fall "
              "from the upright release (the sampler drops it 121-123 mm above the goal)"),
             ("bowl", "yes: the rim pinch held 10/10 (it rests on a knuckle, 21° pivot)", "6/7 to their descent starts (round 13)",
              feas["bowl"], drop_verdict("bowl", "easy_bowls"), release_verdict("bowl", fp), best_b or "not run",
              "the jaws and wrists (rigid with the TCP) in the racks, the basket or the neighbour bowl at the sampler's "
              "release for 6/7; the axis-aligned rim pinch puts the hand under any mouth-down bowl"),
             ("cup", "yes: the wall pinch 10/10, the side pinch 10/10 at 11 or 45 mm under the rim",
              "8/8 goals at the posture-fix mount", feas["cup"], drop_verdict("cup", "easy_cups"),
              release_verdict("cup", fp) + "; round 7: let go 100-125 mm above the goal (side pinch, flipped mouth down)",
              f"settled mouth down {n_cd}/{len(cs)} (D-side-m2, dropped 100-125 mm above the goal)" if cs else "not run",
              "the gripper in the upper rack at the sampler's 34-36 mm release; the side pinch drops from 100-125 mm")]
    return ("<h3>Verdict</h3>" + table(("kind", "graspable", "reachable", "feasible final pose (hand at the sampler's release)",
                                        "highest seating drop (dish alone)", "lowest hand-clear release", "best result",
                                        "limiting part"), [tr([html.escape(c) for c in r]) for r in rows_]))


def drop_curve_block():
    """Version 21 (round 14): the drop curve, one row per dish: seated k/n per drop height above the rack stop (bowls and cups:
    also mouth down), the highest drop with every repeat / some repeat seated."""
    gaps = (20, 40, 60, 80, 100, 150)
    body, notes = [], []
    for kind, load in LOADS:
        dd = drop_data(load)
        if dd is None:
            body.append(tr([load, "", "<span class=muted>not run</span>"] + [""] * (len(gaps) + 1)))
            continue
        rows, hi, rp = dd
        per = {}
        for x in rows:
            per.setdefault((x["dish"], x["gap_mm"]), []).append(x)
        notes.append(f"{load}: {rp.get('drops')} drops" + (f", cut: {rp['cut']}" if rp.get("cut") else "")
                     + (f", re-judged ({rp['rejudged']['changed']} verdicts changed)" if rp.get("rejudged") else ""))
        for dn, v in hi.items():
            cells = [html.escape(dn), html.escape(str(v.get("slot"))),
                     html.escape(str(next((x.get("stop") for x in rows if x["dish"] == dn), "")))]
            for g in gaps:
                xs = per.get((dn, g), [])
                dr = [x for x in xs if x.get("dropped")]
                if not xs:
                    cells.append("–")
                    continue
                if not dr:
                    cells.append("<span class=muted>not dropped</span>")
                    continue
                k = sum(bool(x.get("seated")) for x in dr)
                c = f"{'<b>' if k == len(dr) else ''}{k}/{len(dr)}{'</b>' if k == len(dr) else ''}"
                lat, dz, tl = AT_GOAL[kind]                # the benchmark's at-goal tolerance (frigidaire_bench.AT_GOAL)
                ag = sum(x["error"]["lateral_mm"] <= lat * 1e3 and abs(x["error"]["dz_mm"]) <= dz * 1e3 and
                         x["error"]["tilt_deg"] <= tl and (kind == "plate" or x.get("mouth") == "down") for x in dr)
                c += f" <span class=muted>at goal {ag}</span>"
                if len(dr) < len(xs):
                    c += f" <span class=muted>+{len(xs) - len(dr)} blocked</span>"
                cells.append(c)
            cells.append(f"{v.get('highest_all_seated_mm') or '–'} / {v.get('highest_any_seated_mm') or '–'} mm")
            body.append(tr(cells))
    return ("<section id=drop-curve><h2>Drop curve: the dish alone from the sampler's release orientation</h2>"
            + table(("dish", "slot", "sampler stop", *[f"{g} mm: seated / at goal" for g in gaps],
                     "highest seated: every / some repeat"), body)
            + "<p class=muted>" + html.escape("; ".join(notes)) + "</p></section>")


def final_pose_block():
    """Version 21 (round 14): the final-pose table (kitfree_v1/sampler_release/final_pose_table.json), one row per dish."""
    fp = final_pose()
    if fp is None:
        return "<section id=final-pose><h2>Final pose: the hand at the sampler's release pose</h2><p class=muted>not run</p></section>"
    body = []
    for r in fp["rows"]:
        b = r.get("best") or {}
        hv = ("–" if not b.get("ik") else f"{LP.fmt(b.get('open_vs_held_mm'))} / " + (
            f"{LP.fmt(b.get('closed_nonpad_vs_held_mm'))}" if (b.get("closed_nonpad_vs_held_mm") or -1) > 0
            else f"touch {LP.fmt(b.get('closed_nonpad_pen_mm'))} deep"))
        mc = ("–" if not b.get("ik") else
              f"touching: {html.escape(str(b.get('deepest_pair')).replace(' vs ', ' in '))} {LP.fmt(b.get('deepest_mm'))} mm deep"
              if b.get("deepest_mm") else f"{LP.fmt(b.get('min_clear_mm'))} mm, "
              f"{html.escape(pair_name(str(b.get('blocking') or '?|?').replace(' | ', '|')))}")
        body.append(tr([html.escape(r["dish"]), html.escape(str(r["slot"])), f"{LP.fmt(r['release_above_goal_mm'])} mm",
                        f"{LP.fmt(r['release_lift_mm'])} ({LP.fmt(r['lowest_clear_lift_mm'])})", html.escape(str(r["hold"])),
                        f"{b.get('ik', 0)} ({r['holds_with_ik']}/{r['holds']} holds)", hv, mc,
                        "free" if b.get("descent_ok") else "–" if b.get("descent_ok") is None else "blocked",
                        html.escape(", ".join(b.get("leg1_homes") or []) or "–"),
                        ("<b>feasible</b>" if r["feasible"] else html.escape(r["verdict"]))]))
    sm = fp.get("summary_feasible", {})
    return ("<section id=final-pose><h2>Final pose: the hand at the sampler's release pose</h2>"
            + f"<p>Feasible: plates {sm.get('plate')}, bowls {sm.get('bowl')}, cups {sm.get('cup')} (Kit-free IK + FCL).</p>"
            + table(("dish", "slot", "sampler release above the goal (centroid)", "release lift above the move pose (lowest "
                     "clear), mm", "hold", "IK branches", "hand vs held dish: open / closed, mm", "min clearance, blocker",
                     "50 mm descent", "leg 1 from", "verdict"), body) + "</section>")


PERCH_ROOT = LP.ROOT / "data/results/robot/perch_roll/easy_bowls"


def perch_curve_table():
    """Version 23: the dish-alone perch-roll curve, one row per bowl: per tilt seated mouth down k/n, mouth down, at goal, the
    roll vs the static prediction; the best tilt and the gate (>= 4 of 5)."""
    f = PERCH_ROOT / "highest.json"
    if not f.is_file():
        return "<p class=muted>perch-roll curve: not run</p>"
    hi = json.loads(f.read_text())
    bt = json.loads((PERCH_ROOT / "best_tilt.json").read_text())["bowls"]
    tilts = sorted({float(t) for v in hi.values() for t in v["tilts"]})
    body = []
    for dn, v in hi.items():
        cells = [html.escape(dn), html.escape(str(v.get("slot")))]
        for t in tilts:
            c = v["tilts"].get(str(t))
            if not c:
                cells.append("<span class=muted>–</span>")
                continue
            k, n = c["seated_and_down"], c["drops"]
            cells.append(f"{'<b>' if k >= 4 else ''}{k}/{n}{'</b>' if k >= 4 else ''} <span class=muted>down {c['mouth_down']}, "
                         f"at goal {c['at_goal']}, perch {c['perch_mm']} mm, roll {c['roll_pred']} (right {c['roll_agrees']}/{n})"
                         f"</span>")
        b = bt.get(dn) or {}
        cells.append(f"{b.get('best_tilt')}° ({b.get('seated_down')}/{b.get('drops')}): "
                     + ("<b>gated</b>" if b.get("gate") else "not gated"))
        body.append(tr(cells))
    return ("<h4>Dish-alone perch roll: seated on the goal's wires and mouth down</h4>"
            + table(("bowl", "slot", *[f"tilt {t:.0f}°" for t in tilts], "best tilt, gate"), body))


def jaw_table(rs, dishes):
    """Version 23: per gated bowl the jaw check at its best tilt: spins with a passing release pose, the best clearance."""
    body = []
    for x in dishes:
        r = rs.get(x["name"])
        if r is None or r.get("failed_at") == "not gated":
            continue
        jt = r.get("jaw_check") or {}
        ok = [k for k, v in jt.items() if v.get("passing")]
        best = max(jt.items(), key=lambda kv: -1e9 if kv[1].get("best_min_mm") is None else kv[1]["best_min_mm"]) if jt else None
        body.append(tr([html.escape(x["name"]), f"{(r.get('gate') or {}).get('best_tilt')}°",
                        f"{len(ok)}/{len(jt)}: {html.escape(', '.join(ok)) or 'none'}",
                        "–" if best is None else f"{LP.fmt(best[1].get('best_min_mm'))} mm (spin {best[0]})",
                        html.escape(str(r.get("spin_deg"))) if r.get("stop") else html.escape(str(r.get("failed_at")))]))
    return ("<h4>Jaw check: the open hand vs the recorded rolling bowl (2 mm)</h4>"
            + table(("bowl", "tilt", "spins with a passing release pose", "best clearance", "spin used / outcome"), body))


def roll_table(rs, dishes):
    """Version 23: per bowl the D-roll row: the gate, the tilt and spin, the stop vs the predicted perch, the jaw contact
    after the open, mouth, seated, at goal, the roll vs the prediction."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 6))
            continue
        if r.get("placed_at_goal") or not r.get("stop"):
            body.append(tr(cells + [f"<b>{html.escape(str(r.get('failed_at')))}</b>"])[:-5]
                        + f"<td colspan=6 class=muted>{html.escape(str(r.get('why'))[:300])}</td></tr>")
            continue
        st, pm, td, sd, jc = (r.get("stop") or {}, r.get("perch_mm") or {}, r.get("tilt_deg") or {}, r.get("seated") or {},
                              r.get("jaw_contact") or {})
        pair = (st.get("pairs") or [["none"]])[0][0]
        body.append(tr(cells + [f"{LP.fmt(td.get('carried'))}° / {r.get('spin_deg', 0):+.0f}°",
                                f"{LP.fmt(pm.get('measured'))} vs {pm.get('pred')} mm ({html.escape(pair_name(pair) if '|' in pair else pair)}, "
                                f"{LP.fmt(st.get('force_N'))} N)",
                                ("<b>yes</b>: " + html.escape(", ".join(jc["after_open"])) if jc.get("after_open") else "none"),
                                f"{r.get('mouth')} / " + ("<b>seated</b>" if sd.get("seated") else "not seated")
                                + (" / at goal" if r.get("at_goal") else ""),
                                f"{html.escape(str(r.get('roll_measured')))} (pred {html.escape(str((r.get('combo') or {}).get('roll')))})",
                                html.escape(str(r.get("failed_at") or "PLACED"))]))
    return details(("bowl", "slot", "tilt / spin", "stop: measured vs predicted perch (pair, force)", "jaw contact after the open",
                    "mouth / seated", "roll vs prediction", "verdict"), body)


def sampler_table(rs, dishes):
    """Version 21 (round 14): per dish the sampler-style release (rows I5 / D-sampler): the hold, the sampler's release height
    and where the robot let go, the Kit-free stop vs the sensed stop, the rise, contacts after the release, settled / seated,
    the offsets from the goal; the dishes infeasible at the sampler's release with the Kit-free blocker (placed at the goal)."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 7))
            continue
        if r.get("placed_at_goal"):
            body.append(tr(cells + ["<b>infeasible</b>"])[:-5] + f"<td colspan=7 class=muted>"
                        f"{html.escape(str((r.get('sampler_table') or {}).get('verdict')))} (Kit-free; placed at its goal "
                        f"pose)</td></tr>")
            continue
        if r.get("failed_at") in ("no reach", "no seat"):
            body.append(tr(cells + [f"<b>{html.escape(r['failed_at'])}</b>"])[:-5]
                        + f"<td colspan=7 class=muted>{html.escape(str(r.get('why'))[:300])}</td></tr>")
            continue
        rel, st, ck, sd = r.get("sampler_release") or {}, r.get("stop") or {}, r.get("stop_check") or {}, r.get("seated") or {}
        rt, up = r.get("retreat") or {}, r.get("up_before_release") or {}
        pair = (st.get("pairs") or [[None, None]])[0]
        stop = ("no contact (bound)" if st.get("kind") in (None, "none") else
                f"{html.escape(str(st.get('kind')))} {LP.fmt(ck.get('measured_mm'))} mm: {html.escape(pair_name(pair[0]))}, "
                f"{LP.fmt(st.get('force_N'))} N")
        if ck.get("within_2mm") is not None:
            stop += f" ({'within' if ck['within_2mm'] else '<b>off</b>:'} {ck['measured_mm'] - ck['pred_mm']:+.1f} mm of {LP.fmt(ck['pred_mm'])})"
        e = r.get("error") or {}
        body.append(tr(cells + [html.escape(str(rel.get("hold"))),
                                f"{LP.fmt(rel.get('above_goal_mm'))} / <b>{LP.fmt(r.get('release_h_mm'))}</b> mm", stop,
                                f"up {LP.fmt(up.get('tcp_up_mm'))} mm, rise {LP.fmt(rt.get('rise_mm'))} mm",
                                html.escape(", ".join(pair_name(p) for p in rt.get("hand_touched_dish_after_release") or []) or "none"),
                                ("settled" if r.get("settled") else "NOT settled") + " / " + ("<b>seated</b>" if sd.get("seated")
                                                                                            else "not seated")
                                + (f" (mouth {r.get('mouth')})" if r.get("mouth") else ""),
                                f"{LP.fmt(e.get('lateral_mm'))} / {LP.fmt(e.get('dz_mm'))} mm, {LP.fmt(e.get('tilt_deg'))}°"]))
    return details(("dish", "slot", "hold", "release above the goal: sampler / robot let go", "stop: kind, height, pair, force",
                    "up before the release, rise after", "hand touched the dish after the release", "settled / seated",
                    "lateral / height, tilt from the goal"), body)


def round_media(man, mdir, out, load, var, prefix):
    """The replay, the landed still and the goal still (the first drop test's), one-line captions."""
    es = {e["file"].split("_")[0]: e for e in man if e.get("load") == load and mdir and (mdir / e["file"]).is_file()
          and e.get("variant") == ("D" if e["file"].startswith("goal_") else var)}
    kind = load.split("_")[1][:-1]
    cap = {prefix: f"Replay: every {kind} in order (Warp, from the logged Isaac run)",
           "landed": f"As landed: {(es.get('landed') or {}).get('outcome', '')}", "goal": f"Goal: every {kind} at its goal pose"}
    figs = [figure_html(use({"src": mdir / es[t]["file"]}, out), cap[t]) for t in (prefix, "landed", "goal") if t in es]
    return ((figs[0] if prefix in es else "<p class=muted>replay: none recorded</p>")
            + f"<div class=grid2>{''.join(figs[1:] if prefix in es else figs)}</div>")


def details(head, body):
    return f"<details><summary>Per-dish table</summary>{table(head, body)}</details>"


def tr(cells):
    return "<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>"


def nr_row(cells, r, span):
    """A no-reach row: why (no IK / no path) and the blocking pairs when the arm model has them."""
    why = top_pairs(r) if r.get("blocking") else re.sub(r" None$", "", str(r.get("why") or ""))
    return tr(cells + [f"<b>no reach</b> ({no_reach(r)})"]) [:-5] + f"<td colspan={span} class=muted>{html.escape(why)}</td></tr>"


def err3(e):
    return f"{LP.fmt(e.get('lateral_mm'))} / {LP.fmt(e.get('dz_mm'))} mm, {LP.fmt(e.get('tilt_deg'))}°"


def lip_table(rs, dishes):
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 8))
            continue
        if r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 8))
            continue
        p_, ld, fc = r.get("posture") or {}, r.get("lean_deg") or {}, r.get("fall_clearance") or {}
        body.append(tr(cells + [f"{r.get('spin_deg', 0):+.0f}°", f"{r['drop_height_m'] * 1e3:.0f} mm",
                                "<b>yes</b>" if r.get("settled") else "no", f"{r.get('mouth')} ({r.get('axis_z', 0.):.2f})",
                                err3(r.get("error") or {}), f"{LP.fmt(ld.get('final'))}° vs {LP.fmt(ld.get('goal'))}°",
                                f"{LP.fmt(fc.get('min_mm'))} mm ({fc.get('link', '–')})",
                                deg_cells(p_["max_abs_deg"]) if p_.get("max_abs_deg") else "–",
                                f"{p_['carry_travel_deg'][0]:.0f}°" if p_.get("carry_travel_deg") else "–"]))
    return details(("bowl", "slot", "spin", "drop height", "settled", "mouth (axis z)", "lateral / height, tilt", "lean vs goal",
                    "fall clearance", "max |q|: pan, lift, elbow, w1, w2, w3", "pan swing"), body)


def lip2_table(rs, dishes, r9):
    """Version 19: per bowl round 9 (D-lip) vs round 10 (D-lip2): start branch (wrist 2 / wrist 3), spin, height, largest
    joint travel (round 10: measured, predicted), travel sum, pan swing, path type per leg; then round 10's drop result."""
    def ar(a, b):
        return f"{a} → <b>{b}</b>"

    def w23(r):
        q = (r.get("reset") or {}).get("q_start_deg")
        return f"{q[4]:.0f}° / {q[5]:.0f}°" if q else "–"

    def tv(r, f):
        t = (r.get("posture") or {}).get("carry_travel_deg")
        return f"{f(t):.0f}°" if t and r.get("drop_height_m") is not None else "–"

    def sp(r):
        return f"{r['spin_deg']:+.0f}°" if r.get("spin_deg") is not None else "–"

    def ht(r):
        return f"{r['drop_height_m'] * 1e3:.0f} mm" if r.get("drop_height_m") is not None else "–"
    body = []
    for x in dishes:
        r, o, cells = rs.get(x["name"]), r9.get(x["name"]) or {}, [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 8))
            continue
        if r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 8))
            continue
        lg = r.get("legs") or {}
        legs = " / ".join(str(lg.get(k)).split(" (")[0] for k in ("leg1", "leg2"))
        if (r.get("legs_pred") or {}) and [r["legs_pred"].get(k) for k in ("leg1", "leg2")] != [lg.get(k) for k in ("leg1", "leg2")]:
            legs += f" (predicted {r['legs_pred'].get('leg1')} / {str(r['legs_pred'].get('leg2')).split(' (')[0]})"
        body.append(tr(cells + [ar(w23(o), w23(r)), ar(sp(o), sp(r)), ar(ht(o), ht(r)),
                                ar(tv(o, max), f"{r.get('carry_cost_deg', 0):.0f}°") + f" (pred {r.get('carry_cost_pred', 0):.0f}°)",
                                ar(tv(o, sum), tv(r, sum)), ar(tv(o, lambda t: t[0]), tv(r, lambda t: t[0])), html.escape(legs),
                                ("<b>yes</b>" if r.get("settled") else "no") + f", {r.get('mouth')} ({r.get('axis_z', 0.):.2f})"]))
    return details(("bowl", "slot", "start wrist 2 / wrist 3 (round 9 → 10)", "spin", "drop height",
                    "largest joint travel", "travel sum", "pan swing", "legs (round 10)", "settled, mouth"), body)


def perch_table(rs, dishes):
    """Version 20: per bowl the over-lean perch-and-drop (rows D-perch): tilt (goal -> carried), spin, perch predicted /
    measured, the sensed stop (first pair, force, which body), the static roll predicted vs the outcome, settled, mouth, the
    largest joint travel (measured, predicted)."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 8))
            continue
        if r.get("failed_at") in ("no reach", "no perch"):
            why = top_pairs(r) if r.get("blocking") else re.sub(r" None$", "", str(r.get("why") or ""))
            body.append(tr(cells + [f"<b>{r['failed_at']}</b>"])[:-5] + f"<td colspan=8 class=muted>{html.escape(why)}</td></tr>")
            continue
        td, pm, st, rp = r.get("tilt_deg") or {}, r.get("perch_mm") or {}, r.get("stop") or {}, r.get("roll_pred") or {}
        pair = (st.get("pairs") or [[None, None]])[0]
        stop = ("no contact (bound)" if st.get("kind") == "none" else
                f"{html.escape(pair_name(pair[0]))}, {LP.fmt(st.get('force_N'))} N ({st.get('kind')})")
        diff = pm.get("diff")
        body.append(tr(cells + [f"{LP.fmt(td.get('goal'))}° → {LP.fmt(td.get('carried'))}°", f"{r.get('spin_deg', 0):+.0f}°",
                                f"{pm.get('pred')} / <b>{LP.fmt(pm.get('measured'))}</b> mm"
                                + (f" ({diff:+.1f})" if isinstance(diff, (int, float)) else ""), stop,
                                f"{html.escape(str(rp.get('sign')))} ({LP.fmt(rp.get('com_offset_mm'))} mm) → {r.get('mouth')}",
                                "<b>yes</b>" if r.get("settled") else "no", f"{r.get('mouth')} ({r.get('axis_z', 0.):.2f})",
                                f"{r.get('carry_cost_deg', 0):.0f}° (pred {r.get('carry_cost_pred', 0):.0f}°)"]))
    return details(("bowl", "slot", "tilt (goal → carried)", "spin", "perch pred / measured", "stop: pair, force",
                    "roll pred → mouth", "settled", "mouth (axis z)", "largest joint travel"), body)


WIRE = ((r"tine base rail \(row (\d)\)", r"rail \1"), (r"tine row(\d) col(\d)", r"r\1c\2"), (r"floor long wire ", "long "),
        (r"floor cross wire ", "cross "))


def wire_short(w):
    """'tine base rail (row 0)' -> 'rail 0', 'tine row0 col5' -> 'r0c5', 'floor long wire 02' -> 'long 02'."""
    for a, b in WIRE:
        w = re.sub(a, b, w)
    return w


def seat_table(rs, dishes):
    """Version 21: per plate the seated insertion along the tine axis (rows I4): the hold depth, the Kit-free seat vs the
    plate's height at the sensed stop, the Kit-free hand stop (the closed gripper's first touch on the way down) and the
    clearance at the seat, the stop (kind, first pair, force; within 2 mm of the matching prediction), the wires the settled
    plate touches vs the goal's, seated, lateral / height / lean, the retreat."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 8))
            continue
        if r.get("failed_at") in ("no reach", "no seat"):
            ss, cl = r.get("seat_search") or {}, (r.get("hand_clear_at_seat_mm") or {}).get("closed") or [None, None, None]
            seat = (f"{ss['seat_mm'] + 0.:+.0f} mm on the tine axis ({', '.join(wire_short(w) for w in ss.get('supports') or [])})"
                    if ss.get("seat_mm") is not None else "none")
            why = top_pairs(r) if r.get("blocking") else re.sub(r" None$", "", str(r.get("why") or ""))
            if r.get("carry_fails"):
                fl = r["carry_fails"]
                hs_ = [re.match(r"^\+(\d+) mm", w).group(1) for w in fl if re.match(r"^\+(\d+) mm", w)]
                why = (f"carry: {len(fl)} descent starts tried (+{hs_[0]}..+{hs_[-1]} mm), "
                       + ("every one: RRT-Connect found no path from the home branch to above the start"
                          if all("rrt: none" in w for w in fl) else "the last: " + re.sub(r"^\+\d+ mm: ", "", fl[-1])))
            body.append(tr(cells + [f"{LP.fmt(r.get('hold_depth_mm'))} mm", html.escape(seat),
                                    f"closed jaws {LP.fmt(cl[0])} mm vs {html.escape(PAIR.get(str(cl[2]), str(cl[2])))}",
                                    f"<b>no reach</b> ({no_reach(r)})"])[:-5]
                        + f"<td colspan=4 class=muted>{html.escape(why)}</td></tr>")
            continue
        sm, hs, st, sd = r.get("seat_mm") or {}, r.get("hand_stop_mm") or {}, r.get("stop") or {}, r.get("seated") or {}
        chk, cl = r.get("stop_check") or {}, (r.get("hand_clear_at_seat_mm") or {}).get("closed") or [None, None, None]
        pair = (st.get("pairs") or [[None, None]])[0]
        stop = ("no contact (bound)" if st.get("kind") in (None, "none") else
                f"{html.escape(str(st.get('kind')))}: {html.escape(pair_name(pair[0]))}, {LP.fmt(st.get('force_N'))} N")
        if chk.get("within_2mm") is not None:
            stop += f" ({'within' if chk['within_2mm'] else '<b>off</b>:'} {chk['measured_mm'] - chk['pred_mm']:+.1f} mm)"
        e, ld, rt = r.get("error") or {}, r.get("lean_deg") or {}, r.get("retreat") or {}
        rise = rt.get("rise")
        wf = ", ".join(wire_short(w) for w in sd.get("wires_final") or []) or "none"
        wg = ", ".join(wire_short(w) for w in sd.get("wires_goal") or []) or "none"
        body.append(tr(cells + [f"{LP.fmt(r.get('hold_depth_mm'))} mm",
                                f"{LP.fmt(sm.get('pred'))} / <b>{LP.fmt(sm.get('measured'))}</b> mm",
                                (f"{LP.fmt(hs.get('pred'))} mm" if hs.get("pred") is not None else "none")
                                + f" ({LP.fmt(cl[0])} mm vs {html.escape(PAIR.get(str(cl[2]), str(cl[2])))})", stop,
                                f"{html.escape(wf)} <span class=muted>vs</span> {html.escape(wg)}",
                                ("<b>yes</b>" if sd.get("seated") else "no") + (" / settled" if r.get("settled") else " / NOT settled"),
                                f"{LP.fmt(e.get('lateral_mm'))} / {LP.fmt(e.get('dz_mm'))} mm, {LP.fmt(ld.get('final'))}° "
                                f"(goal {LP.fmt(ld.get('goal'))}°)",
                                f"back {LP.fmt(rt.get('back_mm'))} mm, " + ("rose" if rise is True else "rise blocked" if rise is False
                                                                          else html.escape(str(rise)))]))
    return details(("plate", "slot", "rim depth", "seat pred / at the stop", "hand stop pred (closed jaws at the seat)",
                    "stop: kind, pair, force", "wires: settled vs goal", "seated / settled", "lateral / height, lean",
                    "retreat"), body)


def seat_bowl_table(rs, dishes):
    """Version 21: per bowl the D-seat round: tilt (goal -> carried), spin, the predicted stop (planned; re-predicted at the
    measured start: body and height above the seat) vs the measured stop (body, pair, force, height; within 2 mm and on the
    predicted body), the hand-away retreat, settled, seated on the goal's wires, mouth, the largest joint travel."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 7))
            continue
        if r.get("failed_at") == "no reach":
            why = top_pairs(r) if r.get("blocking") else re.sub(r" None$", "", str(r.get("why") or ""))[:300]
            body.append(tr(cells + ["<b>no reach</b>"])[:-5] + f"<td colspan=7 class=muted>{html.escape(why)}</td></tr>")
            continue
        td, sp, st, ck = r.get("tilt_deg") or {}, r.get("stop_pred") or {}, r.get("stop") or {}, r.get("stop_check") or {}
        rp, rt, sd = sp.get("re_pred") or {}, r.get("retreat") or {}, r.get("seated") or {}
        pair = (st.get("pairs") or [[None, None]])[0]
        stop = ("no contact (bound)" if st.get("kind") in (None, "none") else
                f"{html.escape(str(st.get('kind')))} {LP.fmt(st.get('height_mm'))} mm: {html.escape(pair_name(pair[0]))}, "
                f"{LP.fmt(st.get('force_N'))} N")
        if ck.get("within_2mm") is not None:
            stop += (f" ({'within' if ck['within_2mm'] else '<b>off</b>:'} {ck['measured_mm'] - ck['pred_mm']:+.1f} mm"
                     + ("" if ck.get("body_match") else ", <b>other body</b>") + ")")
        ret = (f"{html.escape(str(rt.get('direction')))}, bowl moved {LP.fmt(rt.get('moved_mm'))} mm" if rt.get("hand_away")
               else "<b>hand stuck</b>")
        body.append(tr(cells + [f"{LP.fmt(td.get('goal'))}° → {LP.fmt(td.get('carried'))}°", f"{r.get('spin_deg', 0):+.0f}°",
                                f"{html.escape(str(sp.get('kind')))} {sp.get('height_mm')} / {html.escape(str(rp.get('kind')))} "
                                f"{LP.fmt(rp.get('height_mm'))} mm", stop, ret,
                                ("<b>yes</b>" if r.get("settled") else "no") + " / " + ("<b>yes</b>" if sd.get("seated") else "no")
                                + f" (mouth {r.get('mouth')})",
                                f"{LP.fmt((sd.get('bottom_nearest') or [None])[0])} / {LP.fmt((sd.get('tine_nearest') or [None])[0])} mm",
                                f"{r.get('carry_cost_deg', 0):.0f}°"]))
    return details(("bowl", "slot", "tilt (goal → carried)", "spin", "stop predicted: planned / at the measured start",
                    "stop measured", "hand away", "settled / seated (mouth)", "goal floor wire / tine distance",
                    "largest joint travel"), body)


def kitfree_gh(kdir):
    """Version 21: (g) the mount search (kitfree_v1/mount_seats/table.json, tool_only.json), (h) the plate hold study
    (kitfree_v1/insert_plate/depth5.json, hold_base/hold_base.json, hold_base/offset/hold_offset.json)."""
    def jl(p):
        f = kdir / p
        return json.loads(f.read_text()) if f.is_file() else {}
    t, to = jl("mount_seats/table.json"), jl("mount_seats/tool_only.json")
    g_rows = []
    if t:
        def mrow(label, m, n=None):
            return tr([label + (f" ({n})" if n is not None else ""), html.escape(str(m.get("mount"))), f"{m.get('bowls')}/7",
                       f"{m.get('plates')}/8 ({', '.join(m.get('which_plates') or []) or '–'})", f"{m.get('cups')}/8",
                       "yes" if m.get("bowl_pick") and m.get("cup_pick") else html.escape(str(m.get("loses") or "no"))])
        g_rows.append(mrow("current mount", t["current"]))
        if t.get("top"):
            low = min(t["top"], key=lambda m: float(re.findall(r"[-\d.]+", m["mount"])[2]))
            g_rows.append(mrow("lowest constraint-passing mount", low, f"of {t.get('n_candidates')}"))
        if t.get("fail_top"):
            g_rows.append(mrow("fails a constraint (best)", t["fail_top"][0], f"of {t.get('n_fail')}"))
    tb = []
    for n, recs in sorted((to.get("bowls") or {}).items()):
        best = max(recs, key=lambda x: x.get("tool_min_mm", -1e9)) if recs else None
        if best:
            tb.append(tr([n, "touching (no clearance at any seat and spin)" if best["tool_min_mm"] <= 0 else f"{best['tool_min_mm']} mm",
                          html.escape(f"{best.get('seat')} spin {best.get('spin'):+.0f}"),
                          html.escape(str(best.get("path_open_by") or best.get("path_wrist_3_by")))]))
    d5, hb, ho = jl("insert_plate/depth5.json"), jl("insert_plate/hold_base/hold_base.json"), \
        jl("insert_plate/hold_base/offset/hold_offset.json")
    h_rows = []
    for n in ("plate_07", "plate_03", "plate_06", "plate_02"):
        old = next((x for x in ((d5.get("goal") or {}).get(n) or {}).get("rows") or [] if x.get("side") == "base" and x.get("depth") == 6), {})
        p1 = ((hb.get("checks") or {}).get("part1_held_plate_overlap") or {}).get("mirror", {}).get(n, {}).get("6", {})
        p4 = ((hb.get("checks") or {}).get("part4_pad_clearance") or {}).get("mirror", {}).get(n, {}).get("6", {})
        p2 = ((hb.get("checks") or {}).get("part2_seat") or {}).get("mirror", {}).get(n, {}).get("6", {})
        so = p2.get("seat_open") or {}
        nb = (p4.get("closed") or {}).get("hand") or (so.get("neighbours") or [None, None, None])
        rows_o = [x for x in ((ho.get("checks") or {}).get(n) or []) if isinstance(x, dict)] if isinstance((ho.get("checks") or {}).get(n), list) else []
        h_rows.append(tr([n, f"{LP.fmt((old.get('held_overlap_open') or [None])[0])} / {LP.fmt((p1.get('open') or [None])[0])} mm",
                          (f"{LP.fmt(nb[0])} mm ({html.escape(str(nb[-1]))})" if nb and nb[0] is not None and nb[-1] is not None
                           and float(nb[0]) < 1e6 else "no neighbour in the bank yet"),
                          ("none: every rim offset 30-90° (both sides, both holds, 6 and 10 mm) has the closed finger on the "
                           "neighbour; from 60° the gripper also reaches the tines" + ("; no planner-free seat anywhere"
                                                                                        if n == "plate_02" else ""))
                          if n != "plate_07" else "–",
                          "yes" if n == "plate_07" else "no"]))
    return ("<h4>(g) Mount search: 825 pedestal x yaw candidates, scored on hand-clear bowl seats, plate seats, cup drops "
            "and the two counter picks</h4>"
            + table(("class", "mount", "bowls seated, hand clear", "plates", "cups", "both counter picks"), g_rows)
            + table(("bowl", "best tool clearance anywhere (mount-independent)", "at", "blocker"), tb)
            + "<h4>(h) Plate hold study: the coded base-face hold vs the mirrored one, the top edge at 6 mm, rim offsets</h4>"
            + table(("plate", "hand vs the held plate, open jaws: coded base face / mirrored", "closed jaws vs the "
                     "neighbour at the seat (mirrored, 6 mm)", "best rim-offset pinch", "seatable with the hand clear"), h_rows))


def kitfree_ef(kdir, root):
    """Version 21: (e) the bowl insertion scan (kitfree_v1/insert_bowl/final.json), (f) the plate stops per front plate
    (insert_plate/task2_r10.json straight down, round 10's I3 rows, plate_seat_smoke/smoke_seat_d6.json, insert_plate/
    depth5.json)."""
    def jl(p):
        f = kdir / p
        return json.loads(f.read_text()) if f.is_file() else {}
    fb, t2, sm, d5 = (jl("insert_bowl/final.json"), jl("insert_plate/task2_r10.json"), jl("plate_seat_smoke/smoke_seat_d6.json"),
                      jl("insert_plate/depth5.json"))
    e_rows = []
    for n, v in fb.items():
        c = v.get("best_touch_free") or v.get("smallest_residual")
        if not c:
            e_rows.append(tr([n, f"{v.get('spin_seats', 0)}", "none", "–", "–", "no"]))
            continue
        clr = c.get("seat_clear_mm")
        hand = (f"open {clr['open']} / closed {clr['closed']} / links {clr['links']} mm" if clr else
                f"no IK at the seat (lowest IK {c.get('ik_lowest_mm')} mm)" if not c.get("ik_at_seat") else "–")
        tch = c.get("touch") or {}
        blk = (c.get("blocker_15mm") or (f"{tch.get('body')} vs {c.get('touch_part')} at {tch.get('h_mm')} mm" if tch else "–"))
        e_rows.append(tr([n, f"{v.get('spin_seats', 0)} ({v.get('seated_rows', 0)} rows)",
                          f"{c['theta']}°, dh {c['dh']:+d} / dt {c['dt']:+d} mm, spin {c['spin']:+.0f}° (seat {c['seat_mm']} mm)",
                          html.escape(hand), html.escape(str(blk).replace("_link", "").replace("_", " ")),
                          "yes" if v.get("free_whole_way") else "no"]))
    r10, _ = rows_of(root, "easy_plates", "I3") if root is not None else ({}, [])
    f_rows = []
    for n in ("plate_07", "plate_03", "plate_06", "plate_02"):
        a = (t2.get(n) or {}).get("no_plates") or {}
        wires = sorted({c["part"] for c in a.get("contacts") or []})
        r = r10.get(n) or {}
        meas = (f"+{r['start_height_m'] * 1e3 - r['lowered_mm']:.0f} mm ({', '.join(r.get('stop_partners') or []) or r.get('stop_reason')})"
                if r.get("start_height_m") is not None and r.get("lowered_mm") is not None else "–")
        s_ = sm.get(n) or {}
        rows5 = {(x.get("side"), x.get("depth")): x for x in ((d5.get("goal") or {}).get(n) or {}).get("rows") or []}

        def clr(d):
            x = rows5.get(("base", d)) or {}
            if not x.get("reach"):
                return "no IK" if x else "–"
            o, c = x.get("seat_open") or {}, x.get("seat_closed") or {}
            return f"closed {c.get('mm')} ({PAIR.get(c.get('body'), c.get('body'))}) / open {o.get('mm')} mm"
        f_rows.append(tr([n, f"+{a.get('first_contact_mm')} mm ({', '.join(wire_short(w) for w in wires)})" if a else "–", meas,
                          (f"{s_.get('seat_z_mm') + 0.:+.1f} mm ({', '.join(wire_short(w) for w in (s_.get('seat') or {}).get('supports') or [])})"
                           if s_ else "–"), clr(19), clr(6)]))
    return ("<h4>(e) Bowl insertion scan: lowered at tilts goal..90°, offsets ±40 mm, every spin (6,237 descents)</h4>"
            + table(("bowl", "seats found", "best candidate: tilt, offsets, spin", "hand at the seat", "blocker",
                     "insertable with the hand free"), e_rows)
            + "<h4>(f) Plate stops: straight down vs along the tine axis (goal scene), hand at the seat</h4>"
            + table(("plate", "straight down: first contact", "round 10: measured stop", "tine axis: seat",
                     "hand at the seat, rim 19 mm", "rim 6 mm"), f_rows))


def kitfree_block(kdir, dishes, root=None):
    """Version 20: the four Kit-free checks of 2026-10-28 (data/results/robot/kitfree_v1, README there), one row per bowl;
    Version 21 adds kitfree_ef's (e) and (f)."""
    def jl(p):
        f = kdir / p
        return json.loads(f.read_text()) if f.is_file() else {}
    names = [x["name"] for x in dishes]
    short = lambda p: re.sub(r"@z_loc([+-]\d+)mm", r" z\1", str(p)).replace("LowerRack:", "lower rack ").replace(
        "UpperRack:", "upper rack ").replace("coupler/palm (base_link)", "palm").replace("_link", "").replace("_", " ")
    g, f, o, t, sp = (jl("guarded/guarded_all.json"), jl("pinchdepth/fall_out.json"), jl("overlean/overlean_all.json"),
                      jl("tangent/task1_fixed.json"), jl("tangent/selfpen_report.json"))
    a_rows = []
    for n in names:
        r = g.get(n)
        if not r:
            a_rows.append(tr([n, "–", "–", "–", "–"]))
            continue
        m, bt = r.get("M") or {}, r.get("bowl_touch") or {}
        h0 = m.get("hand0") or {}
        first = "bowl" if m.get("bowl_first") else "hand"
        a_rows.append(tr([n, f"{bt.get('h_mm')} mm ({html.escape(short(bt.get('pair')))})" if bt else "none",
                          f"{h0.get('h_mm')} mm ({html.escape(short(h0.get('body')))} vs {html.escape(short(h0.get('scene')))})"
                          if h0 else "none", first, f"{h0.get('vertical_fall_mm')} mm" if first == "hand" and h0 else "0"]))
    b_rows = []
    for n in names:
        r = f.get(n)
        if not r:
            b_rows.append(tr([n] + ["–"] * 4))
            continue
        by = {round(x["d_mm"]): x for x in r.get("rows") or []}
        b_rows.append(tr([n] + [f"{by[d]['min_mm']} mm ({html.escape(short(by[d]['blocking']))})" if d in by else "–"
                              for d in (0, 10, 20, 30)]))
    c_rows = []
    for n in names:
        r = o.get(n) or {}
        ok = sorted(((float(th), v["chosen"]) for th, v in r.items() if v.get("chosen")), key=lambda x: x[0])
        if not ok:
            c_rows.append(tr([n, "none", "–", "–", "–", "–", "–", "–"]))
            continue
        th, c = min(ok, key=lambda x: (not (x[1].get("com_mouth_mm") or 0) > 0, x[1]["bowl"]["h_mm"], x[0]))
        c_rows.append(tr([n, f"{ok[0][0]:.1f}–{ok[-1][0]:.1f}° ({len(ok)})", f"{th:.1f}°", f"{c['spin']:+.0f}°",
                          f"{c['bowl']['h_mm']} mm", html.escape(short(c["bowl"].get("part"))),
                          f"{'mouth-down' if (c.get('com_mouth_mm') or 0) > 0 else 'mouth-up'} ({c.get('com_mouth_mm'):+.0f} mm)",
                          f"{c.get('travel_deg')}° (J{c.get('travel_joint')})"]))
    d_rows = []
    for n in names:
        r = t.get(n)
        if not r:
            d_rows.append(tr([n] + ["–"] * 4))
            continue
        cs = r.get("cands") or []
        fl = [x for x in cs if x.get("free_lowest_mm") is not None]
        best = min(fl, key=lambda x: x["free_lowest_mm"]) if fl else None
        if best is None:
            d_rows.append(tr([n, f"{sum(bool(x.get('free0')) for x in cs)}/{len(cs)}", "none", "–", "–"]))
            continue
        lw = (best.get("lowering") or {}).get("open") or {}
        d_rows.append(tr([n, f"{sum(bool(x.get('free0')) for x in cs)}/{len(cs)}",
                          f"{best['free_lowest_mm']} mm (d {best['depth_mm']} mm)",
                          ("bowl first" if best.get("bowl_first") else
                           f"hand first at {best.get('hand_first_touch_mm')} mm ({html.escape(short(lw.get('part')))})"),
                          f"{(best.get('bowl_touch_below_free_mm') or ['–'])[0]} mm"]))
    sp_rows = [tr([f"{d} mm", f"{a} mm", f"{b} mm"]) for d, a, b in zip(sp.get("depth_mm") or [], sp.get("closed_outer_finger_mm") or [],
                                                                         sp.get("inner_knuckle_mm") or [])]
    return ("<details><summary>Kit-free checks (2026-10-28)</summary>"
            "<h4>(a) Hand under the lip: the bowl lowered straight down at the measured pinch</h4>"
            + table(("bowl", "bowl touch", "hand touch", "first", "residual fall"), a_rows)
            + "<h4>(b) Fall clearance vs pinch depth d (released bowl vs gripper and wrists)</h4>"
            + table(("bowl", "d 0 mm", "d 10 mm", "d 20 mm", "d 30 mm"), b_rows)
            + "<h4>(c) Over-lean: passing tilts, the rule's pick (mouth-down roll, lowest perch)</h4>"
            + table(("bowl", "passing tilts", "tilt", "spin", "perch", "contact", "roll", "joint move from the round-10 start"),
                    c_rows)
            + "<h4>(d) Tangential rim hold: free at the goal, lowest free height, first touch</h4>"
            + table(("bowl", "free at goal", "lowest free", "first touch", "bowl touch"), d_rows)
            + table(("pinch depth", "closed outer finger in the wall", "inner knuckle in the wall"), sp_rows)
            + kitfree_ef(kdir, root) + kitfree_gh(kdir) + "</details>")


def perch_ok(rs):
    """(reached, stop within 2 mm of the predicted perch)."""
    got = [r for r in rs.values() if (r.get("perch_mm") or {}).get("measured") is not None]
    return len(got), sum(abs(r["perch_mm"]["measured"] - r["perch_mm"]["pred"]) <= 2. for r in got)


def insert3_table(rs, dishes):
    """Version 19: per plate the insertion and the checked retreat (back-off, what stopped it, rim clearance, rise)."""
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 7))
            continue
        if r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 7))
            continue
        rt = r.get("retreat") or {}
        bb = rt.get("back_blocked_by")
        bb = (f"{bb['at_mm']} mm: " + ", ".join(pair_name(p) for p in bb["pairs"][:2])) if isinstance(bb, dict) else str(bb or "–")
        rise = rt.get("rise")
        body.append(tr(cells + [html.escape(str(r.get("side"))), f"{r.get('lowered_mm')} mm, {html.escape(str(r.get('stop_reason')))}",
                                f"{LP.fmt(rt.get('back_mm'))} mm", html.escape(bb), f"{LP.fmt(rt.get('rim_clear_mm'))} mm",
                                "yes" if rise is True else "blocked" if rise is False else html.escape(str(rise)),
                                ("<b>yes</b>" if r.get("settled") else "no") + " / " + ("<b>yes</b>" if r.get("in_slot") else "no")]))
    return details(("plate", "slot", "face", "lowered, stop", "back", "back stopped by", "rim clearance", "rise",
                    "settled / in slot"), body)


def insert2_table(rs, dishes):
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None:
            body.append(tr(cells + ["<span class=muted>not run</span>"] + [""] * 4))
            continue
        if r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 4))
            continue
        body.append(tr(cells + [html.escape(str(r.get("side"))), f"{r.get('lowered_mm')} mm",
                                html.escape(str(r.get("stop_reason"))) + (f" ({html.escape(', '.join(r.get('stop_partners') or []))})"
                                                                          if r.get("stop_partners") else ""),
                                "<b>yes</b>" if r.get("settled") else "no", "<b>yes</b>" if r.get("in_slot") else "no"]))
    return details(("plate", "slot", "face", "lowered", "stop", "settled", "in slot"), body)


def flip_table(rs, dishes):
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None or r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 6) if r else tr(cells + ["<span class=muted>not run</span>"] + [""] * 6))
            continue
        p_ = r.get("posture") or {}
        body.append(tr(cells + [f"{r.get('flip_tilt_deg', 0):.0f}°", f"{r['drop_height_m'] * 1e3:.0f} mm",
                                "<b>yes</b>" if r.get("settled") else "no", f"{r.get('mouth')} ({r.get('axis_z', 0.):.2f})",
                                err3(r.get("error") or {}), deg_cells(p_["max_abs_deg"]) if p_.get("max_abs_deg") else "–",
                                f"{p_['carry_travel_deg'][0]:.0f}°" if p_.get("carry_travel_deg") else "–"]))
    return details(("bowl", "slot", "tilt", "drop height", "settled", "mouth (axis z)", "lateral / height, tilt",
                    "max |q|: pan, lift, elbow, w1, w2, w3", "pan swing"), body)


def insert1_table(rs, dishes):
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"]), html.escape(str(x.get("slot")))]
        if r is None or r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, 4) if r else tr(cells + ["<span class=muted>not run</span>"] + [""] * 4))
            continue
        body.append(tr(cells + [html.escape(str(r.get("side"))), f"{r.get('lowered_mm')} mm", html.escape(str(r.get("stop_reason"))),
                                "<b>yes</b>" if r.get("settled") else "no", "<b>yes</b>" if r.get("in_slot") else "no"]))
    return details(("plate", "slot", "face", "lowered", "stop", "settled", "in slot"), body)


def cup_table(rs, dishes, posture=False):
    body = []
    for x in dishes:
        r, cells = rs.get(x["name"]), [html.escape(x["name"])]
        span = 4 if posture else 3
        if r is None or r.get("failed_at") == "no reach":
            body.append(nr_row(cells, r, span) if r else tr(cells + ["<span class=muted>not run</span>"] + [""] * span))
            continue
        p_ = r.get("posture") or {}
        body.append(tr(cells + [f"{r['drop_height_m'] * 1e3:.0f} mm", "<b>yes</b>" if r.get("settled") else "no",
                                f"{r.get('mouth')} ({r.get('axis_z', 0.):.2f})", err3(r.get("error") or {})]
                       + ([deg_cells(p_["max_abs_deg"]) if p_.get("max_abs_deg") else "–"] if posture else [])))
    return details(("cup", "drop height", "settled", "mouth (axis z)", "lateral / height, tilt")
                   + (("max |q|: pan, lift, elbow, w1, w2, w3",) if posture else ()), body)


def ring_rows(root):
    """Version 25: the D-ring rows of easy_bowls: ({dish: row} of the load's rows, [the flagged extra tries])."""
    d = root / "easy_bowls" if root is not None else None
    if not (d and (d / "dishes.jsonl").is_file()):
        return {}, []
    rows_ = [r for r in LP.rows(d / "dishes.jsonl") if r.get("variant") == "D-ring"]
    return {r["dish"]: r for r in rows_ if not r.get("extra")}, [r for r in rows_ if r.get("extra")]


def ring_outcome(r):
    """One phrase: what a D-ring row did (no reach, or the judged outcome)."""
    if r.get("placed_at_goal") or not r.get("stop"):
        return f"{r.get('failed_at')} ({str(r.get('why') or '')[:160]})"
    sd = r.get("seated") or {}
    return (f"mouth {r.get('mouth')}, {'seated' if sd.get('seated') else 'not seated'}, "
            f"{'AT_GOAL' if r.get('at_goal') else 'off goal'}, jaw contact after the open "
            f"{'yes' if (r.get('jaw_contact') or {}).get('after_open') else 'none'}"
            + (f" ({r.get('failed_at')})" if r.get("failed_at") else ""))


def ring_blockers(r):
    """The scene bodies of a no-reach row's Kit-free blocker (round 16's goal table: 'hand not clear: [link|body, ...]')."""
    m = re.search(r"hand not clear: \[([^\]]*)\]", str(r.get("why") or ""))
    names = []
    for p in (m.group(1).split(",") if m else []):
        b = p.strip(" '").split("|")[-1]
        b = PAIR.get(b, b)
        if b not in names:
            names.append(b)
    return ", ".join(names) or "Kit-free"


def ring_failures(rs):
    """Version 25: the D-ring failure modes; each no-reach bowl with its Kit-free blocker."""
    c = Counter(r.get("failed_at") for r in rs.values() if r.get("failed_at"))
    out = []
    for f, n in sorted(c.items(), key=lambda x: -x[1]):
        if f == "no reach":
            out.append(f"no reach {n}: " + "; ".join(f"{d} ({ring_blockers(r)})" for d, r in rs.items()
                                                       if r.get("failed_at") == "no reach"))
        else:
            out.append(f"{f} {n}")
    return "; ".join(out) or "none"


def ring_table(root, dishes):
    """Version 25: per bowl the D-ring row: spin and the carry (mouth down throughout, hand above), the stop vs the prediction
    (pair, force), the jaw contact after the open, mouth / seated / AT_GOAL, the verdict; no reach with its Kit-free blocker;
    the flagged extra try (bowl_06 at the 10 mm arm margin) right after its bowl."""
    main, ex = ring_rows(root)
    body = []

    def one(name, slot, r):
        cells = [name, html.escape(str(slot))]
        if r is None:
            return tr(cells + ["<span class=muted>not run</span>"] + [""] * 5)
        if r.get("placed_at_goal") or not r.get("stop"):
            return (tr(cells + [f"<b>{html.escape(str(r.get('failed_at')))}</b>"])[:-5]
                    + f"<td colspan=5 class=muted>{html.escape(str(r.get('why'))[:560])}</td></tr>")
        st, sc, sd, jc, ca = (r.get("stop") or {}, r.get("stop_check") or {}, r.get("seated") or {}, r.get("jaw_contact") or {},
                              r.get("carry") or {})
        ss = r.get("seated_as_run") or {}
        gp = sd.get("goal_pose_test") or {}
        pair = (st.get("pairs") or [["none"]])[0][0]
        return tr(cells + [f"{r.get('spin_deg', 0):+.0f}° · mouth down throughout "
                           f"{'yes' if ca.get('mouth_down_throughout') else '<b>no</b>'} (axis z ≤ {LP.fmt(ca.get('max_axis_z'), 3)}), "
                           f"hand above {'yes' if ca.get('hand_above_throughout') else '<b>no</b>'} (TCP ≥ "
                           f"{LP.fmt(ca.get('min_tcp_above_centroid_mm'))} mm above the bowl's centroid)",
                           f"{LP.fmt(sc.get('measured_mm'))} vs {LP.fmt(sc.get('pred_mm'))} mm "
                           f"({html.escape(pair_name(pair) if '|' in pair else pair)}, {LP.fmt(st.get('force_N'))} N)",
                           ("<b>yes</b>: " + html.escape(", ".join(jc["after_open"])) if jc.get("after_open") else "none"),
                           f"{r.get('mouth')} / " + ("<b>seated</b>" if sd.get("seated") else "not seated")
                           + (" / AT_GOAL" if r.get("at_goal") else " / off goal")
                           + (f" (no bottom wire: centroid {LP.fmt(gp.get('centroid_mm'), 2)} mm, axis "
                              f"{LP.fmt(gp.get('axis_deg'))}°" + ("; re-judged by the axis rule, as run the full rotation "
                              "at spin 0 failed it" if ss else "") + ")" if gp else ""),
                           html.escape(str(r.get("failed_at") or "PLACED"))])
    for x in dishes:
        body.append(one(html.escape(x["name"]), x.get("slot"), main.get(x["name"])))
        for e in [e for e in ex if e["dish"] == x["name"]]:
            body.append(one(f"{html.escape(x['name'])} <b>(flagged: arm margin {e.get('arm_margin_mm'):.0f} mm, "
                            f"{html.escape(str(e.get('racks')))})</b>", x.get("slot"), e))
    return details(("bowl", "slot", "spin · carry", "stop: measured vs predicted (pair, force)", "jaw contact after the open",
                    "mouth / seated / AT_GOAL", "verdict"), body)


TABLES = {"D-roll": lambda rs, ds: perch_curve_table() + jaw_table(rs, ds) + roll_table(rs, ds),
          "I5": lambda rs, ds: sampler_table(rs, ds), "D-sampler": lambda rs, ds: sampler_table(rs, ds),
          "I4": lambda rs, ds: seat_table(rs, ds), "D-seat": lambda rs, ds: seat_bowl_table(rs, ds), "I3": insert3_table, "D-lip": lip_table, "I2": insert2_table, "D-flip": flip_table, "I": insert1_table,
          "D-side-m2": lambda rs, ds: cup_table(rs, ds, posture=True), "D-side": cup_table}


def round_block(root, mdir, out, man, sid, title, load, var, prefix, kdir=None):
    rs, dishes = rows_of(root, load, var)
    head = f"<section id={sid}><h2>{html.escape(title)}</h2>"
    if not rs and var == "D-roll" and (PERCH_ROOT / "highest.json").is_file():   # the gate skipped the robot round
        return head + perch_curve_table() + "<p class=muted>robot round: not run (no bowl gated)</p></section>"
    if not rs:
        return head + "<p class=muted>not run</p></section>"
    tab = (lip2_table(rs, dishes, rows_of(root, load, "D-lip")[0]) if var == "D-lip2" else
           perch_table(rs, dishes) if var == "D-perch" else ring_table(root, dishes) if var == "D-ring" else
           TABLES[var](rs, dishes))
    if var == "D-seat" and kdir:                       # Version 21: the Kit-free block moves to the newest bowl section
        tab += kitfree_block(kdir, dishes, root)
    return head + round_media(man, mdir, out, load, var, prefix) + tab + "</section>"


def starts_of(root):
    """The start posture per kind from the newest rows (reset q_start_deg), one phrase each; the cup ran no row since round 8
    and shares the plate's start (Kit-free preview, data/logs/robot/r9_preview.log: same branch at the 2 mm margin). Round 10's
    bowls start at the home branch of their best carry combo: every distinct start with its bowl count."""
    out = {}
    rs, _ = rows_of(root, "easy_bowls", "D-seat")
    cnt = Counter(tuple(round(v) for v in r["reset"]["q_start_deg"]) for r in rs.values()
                  if (r.get("reset") or {}).get("q_start_deg") and r.get("stop"))
    if cnt:
        out["bowl (round 13, per bowl: the home branch of its best combo)"] = "; ".join(
            f"[{', '.join(str(v) for v in q)}]° on {n}" for q, n in cnt.most_common())
    for load, var, kind in (("easy_plates", "I4", "plate (round 12, base face, rim 6 mm)"), ("easy_plates", "I3", "cup (as the "
                            "round-10 plate)")):
        rs, _ = rows_of(root, load, var)
        qs = sorted({tuple(round(v) for v in r["reset"]["q_start_deg"]) for r in rs.values()
                     if (r.get("reset") or {}).get("q_start_deg")})
        if qs:
            w3 = qs[0][5]
            out[kind] = (f"[{', '.join(str(v) for v in qs[0])}]° (wrist 3 at {w3}"
                         + (", wrist 1 10.6 mm from the bowl)" if kind == "bowl" and abs(w3) < 90 else
                            ", the wrist-3-at-0 branches overlap the dish)" if abs(w3) >= 90 else ")"))
    return out


WORDS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six", 7: "seven"}
SETUP_LINE = ("UR5e + Robotiq 2F-85 (hand side-mounted, as built in the twin) on a pedestal at (0.45, -0.70, 0.60) m, "
              "Frigidaire twin, HOTEC dishes, Isaac Sim 4.5, simulation only; the dish is held and never dropped during the carry.")


def grasp_counts(trials):
    """{(kind, grasp family as the gate rows name it): (held, trials)} of the counter-spot grasp gates (grasping_block's
    grouping; G3 = held after the lift and the hold)."""
    by = {}
    for r in trials:
        if r.get("mode") != "grasp_gate" or r.get("spot"):
            continue
        g = (r.get("grasp_family") or "rim pinch") + (", planner clearance waived" if r.get("waive_dish_clearance") else "")
        by.setdefault((r["kind"], g), []).append(r)
    return {k: (sum(bool(r["g3"]) for r in rs), len(rs)) for k, rs in by.items()}


def minimal_body(trials, proot, mdir, out):
    """Version 26: the title, the date, the setup line, "Grasping" (one table and the two gate clips) and "Best placement"
    (three clips, one sentence each; the counts from the rows). Nothing else."""
    kn = grasp_counts(trials)

    def f(kind, fam):
        return "{}/{}".format(*kn[(kind, fam)]) if (kind, fam) in kn else "not run"
    rows = (("bowl", "foot ring, pads on the ring's outer face", "mouth down on the counter", f("bowl", "foot ring, out"),
             f"rim pinch, mouth up: {f('bowl', 'rim pinch')} (rests on a knuckle; it leaves the hand under a mouth-down bowl); "
             f"foot ring from inside: {f('bowl', 'foot ring, in')} (slips)"),
            ("plate", "top-edge rim pinch, hand on the base side, rim 6 mm in", "face up",
             "assumed: no measured pick (no clip)",
             f"rim pinch: {f('plate', 'rim pinch, planner clearance waived')}, only with the planner's clearance check off; "
             f"slide: {f('plate', 'slide')}"),
            ("cup", "side pinch 11 mm under the rim", "mouth up or mouth down", f("cup", "side pinch, pads 11 mm under the rim"),
             f"wall pinch: {f('cup', 'wall pinch')}; side pinch 45 mm under the rim: "
             f"{f('cup', 'side pinch, pads 45 mm under the rim')}; side pinch 80 mm under the rim: "
             f"{f('cup', 'side pinch, pads 80 mm under the rim')} (the gripper hits the counter); side pinch with the wrist "
             f"toward the base: {f('cup', 'side pinch, pads 45 mm under the rim, wrist toward the base')} (no collision-free grasp)"))
    grasp = table(("kind", "default grasp (Part B assumption)", "start", "held", "other grasps tried (held)"),
                  [tr([html.escape(c) for c in r]) for r in rows])
    clips = "".join(figure_html(use({"src": mdir / fn}, out), cap) for fn, cap in (
        ("partA_bowl_foot_out_pass_t00.mp4", "bowl: foot ring, outer face"),
        ("partA_cup_side_h11_pass_t00.mp4", "cup: side pinch 11 mm under the rim")))
    ring, _ = ring_rows(proot)
    ran = [r for r in ring.values() if not r.get("placed_at_goal")]
    se = [n[-2:] for n, r in ring.items() if (r.get("seated") or {}).get("seated")]      # the witness order
    nr = sum(r.get("failed_at") == "no reach" for r in ring.values())
    pl, _ = rows_of(proot, "easy_plates", "I4")
    reach = [r for r in pl.values() if r.get("failed_at") != "no reach"]
    pse = sum(bool((r.get("seated") or {}).get("seated")) for r in pl.values())
    cu, _ = rows_of(proot, "easy_cups", "D-side-m2")
    cdn = sum(bool(r.get("settled")) and r.get("mouth") == "down" for r in cu.values())
    hs = sorted(round(r["drop_height_m"] * 1e3) for r in cu.values() if r.get("drop_height_m") is not None)
    best = (("drop_easy_bowls_ring.mp4",
             f"Picked mouth-down by the foot ring, carried mouth-down with the hand above, lowered onto the goal until the "
             f"wrist sensed the rack, let go, hand up: {len(se)} of {len(ring)} seated at their goals ({', '.join(se)}); the "
             f"other {WORDS.get(nr, nr)} are out of reach (tub face, neighbour, tines, a bad goal)."),
            ("insert_easy_plates_4.mp4",
             f"Held by the top edge (assumed grip), lowered along the tines' lean until the wrist sensed 3.5 N, let "
             f"go: {pse} of the {len(reach)} reachable front plates seated; the rear {len(pl) - len(reach)} are out of reach "
             f"(forearm in the tub)."),
            ("drop_easy_cups_side_m2.mp4",
             f"Side pinch, flipped mouth-down, let go {hs[0]} to {hs[-1]} mm above the goal: {cdn} of {len(cu)} settled "
             f"mouth down."))
    assert len(ran) == len(ring) - nr, "D-ring rows: carried + no reach != 7"
    figs = "".join(f"<figure><video controls muted playsinline preload=metadata src='{html.escape(use({'src': mdir / fn}, out))}'>"
                   f"</video><figcaption>{html.escape(cap)}</figcaption></figure>" for fn, cap in best)
    return (f"<main><h1>Hardware Limits Report</h1><p class=muted>{date.today().isoformat()}</p><p>{html.escape(SETUP_LINE)}</p>"
            f"<section id=grasping><h2>Grasping</h2>{grasp}<div class=grid2>{clips}</div></section>"
            f"<section id=placement><h2>Best placement</h2><div class=grid3>{figs}</div></section></main>")


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
    ap.add_argument("--kitfree", type=Path, default=LP.ROOT / "data/results/robot/kitfree_v1",
                    help="Version 20: the Kit-free checks' JSON copies (README there)")
    ap.add_argument("--minimal", action="store_true",
                    help="Version 26: the grasp table + clips and the best placement per kind only (the full builders unused)")
    a = ap.parse_args()
    for d in a.records + a.media:
        if not d.is_dir():
            print(f"[INFO] {d} not found: its cells read 'not run'", flush=True)
    recs = [d for d in a.records if d.is_dir()]
    trials, sites, summ, meta, grid, verdicts, windows = load(recs)
    kn = verdicts.get("knuckle") or {}                         # the kind the knuckle check ran on, from its trial path
    measured = next((k for k in KINDS if k in str(kn.get("trial"))), None) if kn.get("verdict") == "real" else None
    (a.out / "media").mkdir(parents=True, exist_ok=True)
    proot = a.placing or a.records[0].parent / "placing_v1"          # Part B and the Part A runs beside it
    pmdir = a.placing_media or (a.media[0].parent / "placing_v1" if a.media else None)
    pman = json.loads((pmdir / "videos.json").read_text()) if pmdir and (pmdir / "videos.json").is_file() else []
    mf = LP.ROOT / MOUNT_FILE
    mt = json.loads(mf.read_text()) if mf.is_file() else None
    body = (minimal_body(trials + part_a_rows(proot), proot, pmdir, a.out) if a.minimal else
            f"<main><h1>Hardware Limits Report</h1><p class=muted>{date.today().isoformat()}</p>"
            + setup_block(mt, starts_of(proot)) + best_block(proot, pmdir, a.out, pman) + partb_block(proot)
            + grasping_block(trials + part_a_rows(proot), grid, measured, pman, pmdir, a.out, proot)
            + "".join(round_block(proot, pmdir, a.out, pman, sid, title, load, var, prefix, a.kitfree)
                      for sid, title, load, var, prefix, _, _ in ROUNDS if var not in TABLE_ONLY) + "</main>")
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
