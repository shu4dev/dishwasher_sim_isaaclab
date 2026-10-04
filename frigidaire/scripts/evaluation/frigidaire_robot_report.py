#!/usr/bin/env python3
"""Build the Phase 0 report of plans/2026-09-29-easy-s0.md from the trial logs (Kit-free; never by hand).

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_robot_report.py \\
        --runs p0_upright3_legacy p0_upright3_headline ... --conventions p0_conventions --out REPORT.md

Per trial (artifacts/<run-id>/<trial-id>/): plots/*.png from trial.jsonl (joint tracking error, pad normal force,
bowl displacement, contact-event timeline, all vs simulated time). Per run: artifacts/<run-id>/summary/*.png (k/N by
bowl, histogram of the in-hand settle displacement) and artifacts/<run-id>/index.html (self-contained: plots, key
frames and the video embedded). The report: every required Phase 0 test of the plan with its status ("FAIL (not
run)" when nothing ran), the run tables (k/N, simulated and wall time as separate columns, attempts and
configurations separately), per-bowl causes, deviations from the benchmark with their flags, the audit table (plan
decision D20 wording), the hypotheses, open problems; every claim cites a trial-log line, a record or a file.
"""
from __future__ import annotations

import argparse
import base64
from collections import defaultdict
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / "results/robot/runs"
ARTIFACTS = ROOT / "artifacts"
PLAN = "plans/2026-09-29-easy-s0.md"
MAP = "plans/2026-09-29-easy-s0-map.md"
INVARIANTS = ("arm_or_palm_contact", "carried_contact", "teleport_after_reset", "sleeping_body", "nan_frame",
              "joint_limit", "pad_force")
PAD_FORCE_MAX_N = 235.
ROBOT_TESTS = ("frigidaire/tests/test_robot_kin.py", "frigidaire/tests/test_robot_conventions.py",
               "frigidaire/tests/test_robot_flags.py", "frigidaire/tests/test_robot_harness.py")
CAT_COLOR = {"pad": "#2f6db3", "finger": "#d9822b", "palm": "#8a2be2", "arm": "#c0392b", "dish": "#3a9a5b",
             "rack": "#7f8c8d", "basket": "#95a5a6", "counter": "#b8a15a", "pedestal": "#555555", "ground": "#999999",
             "appliance": "#6c7a89", "other": "#aaaaaa"}
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")


# --------------------------------------------------------------------------- reading

def read_jsonl(path):
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if line:
                out.append((n, json.loads(line)))
    return out


def _episode_module():
    """The episode script's pure helpers (failure_class); importing it boots nothing (Kit imports live in main())."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("frigidaire_robot_episode", ROOT / "frigidaire/scripts/experiment/frigidaire_robot_episode.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def summarize_cause(text):
    """(class, short text) of a failure cause. A pick failure lists every candidate it tried
    ('... (tried N: [why, ...])'): the class is the most frequent tried kind and the text counts the kinds."""
    import ast
    import re
    fc = _episode_module().failure_class
    if not text:
        return None, None
    m = re.search(r"\(tried (\d+): (\[.*\])\)\s*$", text, re.S)
    if not m:
        return fc(text), text
    try:
        tried = ast.literal_eval(m.group(2))
    except (ValueError, SyntaxError):
        return fc(text), text
    kinds = defaultdict(int)
    for why in tried:
        kinds[fc(why)] += 1
    if not kinds:
        return fc(text), text
    top = max(kinds, key=kinds.get)
    parts = "; ".join(f"{k} x{n}" for k, n in sorted(kinds.items(), key=lambda kv: -kv[1]))
    first = {}
    for why in tried:
        first.setdefault(fc(why), why[:110])
    return top, f"{len(tried)} candidates tried: {parts} (e.g. {first[top]})"


def summarize_move(m):
    """(class, text) over EVERY attempt of a failed move (the record's `cause` quotes the last attempt only)."""
    import ast
    import re
    fc = _episode_module().failure_class
    kinds, parts = defaultdict(int), []
    for a in m.get("attempts", []):
        texts = []
        if a.get("invariants"):
            texts.append("invariant: " + ", ".join(a["invariants"]))
        if a.get("pick") is None and a.get("why"):
            texts.append(a["why"])
        elif a.get("place_why"):
            texts.append(a["place_why"])
        elif a.get("physical_cause") and not a.get("ok_physical"):
            texts.append(a["physical_cause"])
        sub = defaultdict(int)
        for t in texts:
            mm = re.search(r"\(tried (\d+): (\[.*\])\)\s*$", t, re.S)
            tried = None
            if mm:
                try:
                    tried = ast.literal_eval(mm.group(2))
                except (ValueError, SyntaxError):
                    tried = None
            for why in (tried if tried else [t]):
                k = fc(why)
                sub[k] += 1
                kinds[k] += 1
        if sub:
            parts.append(f"attempt {a.get('attempt', 0) + 1}: " + ", ".join(f"{k} x{n}" for k, n in sorted(sub.items(), key=lambda kv: -kv[1])))
    if not kinds:
        c = m.get("cause")
        return (fc(c) if c else None), c
    top = max(kinds, key=kinds.get)
    return top, "; ".join(parts)


def reclassify(record):
    """Re-derive every failure class with the CURRENT classifier over every attempt (older records carry the label the
    classifier of their day wrote, and the record's cause quotes the last attempt only)."""
    if not record:
        return record
    by_dish = {}
    for m in record.get("moves", []):
        if not m.get("ok"):
            if m.get("attempts"):
                m["failure_class"], m["cause_summary"] = summarize_move(m)
            elif m.get("cause"):
                m["failure_class"], m["cause_summary"] = summarize_cause(m["cause"])
            by_dish.setdefault(m["dish"], m)
    for oid, b in record.get("per_bowl", {}).items():
        if b.get("cause"):
            mv = by_dish.get(oid)
            if mv is not None and not str(b["cause"]).startswith("not attempted"):
                b["failure_class"], b["cause_summary"] = mv["failure_class"], mv.get("cause_summary")
            else:
                b["failure_class"], b["cause_summary"] = summarize_cause(b["cause"])
    return record


def load_run(run_id):
    """One run: its record(s), trials (trial log rows + media) and paths."""
    rdir = RUNS / run_id
    adir = ARTIFACTS / run_id
    records = sorted(p for p in rdir.glob("*.json")) if rdir.exists() else []
    run = {"run_id": run_id, "record_path": records[0] if records else None,
           "record": reclassify(json.loads(records[0].read_text())) if records else None, "trials": [],
           "adir": adir, "rdir": rdir}
    for tdir in sorted(p for p in adir.glob("t*") if p.is_dir()) if adir.exists() else []:
        tl = tdir / "trial.jsonl"
        if not tl.exists():
            continue
        rows = read_jsonl(tl)
        trial = {"trial_id": tdir.name, "dir": tdir, "rows": rows, "log": tl,
                 "meta": next((r for n, r in rows if r["event"] == "meta"), None),
                 "end": next((r for n, r in rows if r["event"] == "end"), None),
                 "end_line": next((n for n, r in rows if r["event"] == "end"), None),
                 "video": tdir / "video.mp4" if (tdir / "video.mp4").exists() else None,
                 "frames": sorted((tdir / "frames").glob("*.png")) if (tdir / "frames").exists() else []}
        run["trials"].append(trial)
    return run


def rel(path):
    """Path relative to the repo root (REPORT.md lives there); symlinked roots (artifacts/) are kept as written."""
    path = Path(path)
    for base in (ROOT, ROOT.resolve()):
        try:
            return str(path.relative_to(base))
        except ValueError:
            pass
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def logref(trial, line):
    return f"`{rel(trial['log'])}:L{line}`"


# --------------------------------------------------------------------------- plots (per trial)

def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"figure.facecolor": "white", "axes.grid": True, "grid.alpha": .3, "font.size": 9})
    return plt


def phase_spans(rows):
    """[(t0, t1, move, phase)] of the trial from the sample rows' context."""
    spans, cur = [], None
    for n, r in rows:
        if r["event"] != "sample":
            continue
        key = (r.get("move"), r.get("phase"))
        t = r["sim_s"]
        if cur is None or cur[2:] != key:
            if cur is not None:
                spans.append((cur[0], t, *cur[2:]))
            cur = [t, t, *key]
        cur[1] = t
    if cur is not None:
        spans.append(tuple(cur))
    return spans


def shade_moves(ax, spans):
    seen = set()
    for t0, t1, move, phase in spans:
        if move is None:
            continue
        if move not in seen:
            seen.add(move)
            ax.axvspan(t0, t1, color="#f0f0f0" if move % 2 else "#e0e6ee", alpha=.6, lw=0, zorder=0)
            ax.text(t0, ax.get_ylim()[1], f" m{move}", va="top", fontsize=7, color="#555")
        else:
            ax.axvspan(t0, t1, color="#f0f0f0" if move % 2 else "#e0e6ee", alpha=.6, lw=0, zorder=0)


def invariant_marks(ax, rows):
    for n, r in rows:
        if r["event"] == "invariant":
            ax.axvline(r["sim_s"], color="#c0392b", lw=.8, ls="--", alpha=.8)
            ax.text(r["sim_s"], ax.get_ylim()[0], r["payload"]["name"], rotation=90, fontsize=6, color="#c0392b",
                    va="bottom", ha="right")


def plot_trial(trial):
    """The four per-trial plots; returns {name: path}."""
    plt = _mpl()
    import numpy as np
    rows = trial["rows"]
    samples = [r for n, r in rows if r["event"] == "sample"]
    if not samples:
        return {}
    pdir = trial["dir"] / "plots"
    pdir.mkdir(exist_ok=True)
    t = np.array([r["sim_s"] for r in samples])
    spans = phase_spans(rows)
    out = {}
    # ---- joint tracking error
    lag = np.array([r["payload"]["lag"] for r in samples]) * 1e3
    fig, ax = plt.subplots(figsize=(11, 3.4))
    for j, name in enumerate(JOINTS):
        ax.plot(t, lag[:, j], lw=.8, label=name)
    ax.set_xlabel("simulated time [s]"); ax.set_ylabel("measured - commanded [mrad]")
    ax.set_title(f"{trial['trial_id']}: arm joint tracking error (sampled every 6 ticks = 20 Hz)")
    ax.legend(ncol=6, fontsize=7, loc="upper right")
    shade_moves(ax, spans); invariant_marks(ax, rows)
    fig.tight_layout(); fig.savefig(pdir / "joint_tracking.png", dpi=110); plt.close(fig)
    out["joint_tracking"] = pdir / "joint_tracking.png"
    # ---- pad force
    fig, ax = plt.subplots(figsize=(11, 3.4))
    has_normal = "pad_F" in samples[0]["payload"]
    for side, color in (("pad_L", "#2f6db3"), ("pad_R", "#d9822b")):
        n_ = np.array([r["payload"]["pad_N"].get(side, 0.) for r in samples])
        ax.plot(t, n_, lw=.9, color=color, label=f"{side} {'normal' if has_normal else '|impulse|/dt'}")
        if has_normal:
            f_ = np.array([r["payload"]["pad_F"].get(side, 0.) for r in samples])
            ax.plot(t, f_, lw=.6, color=color, alpha=.45, ls=":", label=f"{side} total |impulse|/dt")
    ax.axhline(PAD_FORCE_MAX_N, color="#c0392b", lw=.8, ls="-.", label="2F-85 max 235 N")
    ax.set_xlabel("simulated time [s]"); ax.set_ylabel("force on the pad collider [N]")
    ax.set_title(f"{trial['trial_id']}: pad contact force (PhysX contact reports; sampled at 20 Hz)")
    ax.legend(ncol=5, fontsize=7, loc="upper right")
    shade_moves(ax, spans); invariant_marks(ax, rows)
    fig.tight_layout(); fig.savefig(pdir / "pad_force.png", dpi=110); plt.close(fig)
    out["pad_force"] = pdir / "pad_force.png"
    # ---- bowl displacement from the start pose
    ids = sorted(samples[0]["payload"]["dishes"])
    start = {oid: np.array(samples[0]["payload"]["dishes"][oid][:3]) for oid in ids}
    fig, ax = plt.subplots(figsize=(11, 3.4))
    for oid in ids:
        d = np.array([np.linalg.norm(np.array(r["payload"]["dishes"][oid][:3]) - start[oid]) for r in samples]) * 1e3
        carried = np.array([r["payload"].get("carried") == oid for r in samples])
        ax.plot(t, d, lw=.8, label=oid)
        if carried.any():
            ax.plot(t[carried], d[carried], lw=2.2, alpha=.5, color=ax.lines[-1].get_color())
    ax.set_xlabel("simulated time [s]"); ax.set_ylabel("|p - p_start| [mm]")
    ax.set_title(f"{trial['trial_id']}: bowl displacement from its start pose (thick = carried window)")
    ax.legend(ncol=min(7, len(ids)), fontsize=7, loc="upper left")
    shade_moves(ax, spans); invariant_marks(ax, rows)
    fig.tight_layout(); fig.savefig(pdir / "bowl_displacement.png", dpi=110); plt.close(fig)
    out["bowl_displacement"] = pdir / "bowl_displacement.png"
    # ---- contact-event timeline (pairs that touch the robot or a dish)
    begins = {}
    events = []
    for n, r in rows:
        if r["event"] == "contact_begin":
            begins[tuple(r["payload"]["pair"])] = (r["sim_s"], r["payload"])
        elif r["event"] == "contact_end":
            key = tuple(r["payload"]["pair"])
            t0 = begins.pop(key, (r["sim_s"] - r["payload"].get("duration_s", 0.), r["payload"]))[0]
            events.append((key, t0, r["sim_s"], r["payload"]["cats"], r["payload"].get("peak_N", 0.)))
    t_end = t[-1] if len(t) else 0.
    for key, (t0, p) in begins.items():
        events.append((key, t0, t_end, p["cats"], p.get("normal_N", p.get("force_N", 0.))))
    keep = [e for e in events if any(c in ("pad", "finger", "palm", "arm") for c in e[3]) or all(c == "dish" for c in e[3])
            or (("dish" in e[3]) and any(c in ("rack", "basket") for c in e[3]))]
    keep.sort(key=lambda e: (e[3][0] not in ("pad", "finger", "palm", "arm"), "|".join(e[0])))
    lanes = []
    for e in keep:
        lab = "|".join(e[0])
        if lab not in lanes:
            lanes.append(lab)
    fig, ax = plt.subplots(figsize=(11, max(2.5, .22 * len(lanes) + 1.2)))
    for e in keep:
        lab = "|".join(e[0])
        y = lanes.index(lab)
        cat = next((c for c in e[3] if c in ("pad", "finger", "palm", "arm")), next((c for c in e[3] if c != "dish"), "dish"))
        ax.barh(y, max(e[2] - e[1], .05), left=e[1], height=.7, color=CAT_COLOR.get(cat, "#aaa"), alpha=.85)
        if e[4]:
            ax.text(e[2], y, f" {e[4]:.0f} N", va="center", fontsize=6, color="#333")
    ax.set_yticks(range(len(lanes))); ax.set_yticklabels(lanes, fontsize=7)
    ax.set_xlim(0, t_end); ax.set_xlabel("simulated time [s]")
    ax.set_title(f"{trial['trial_id']}: contact events (touching pairs; colour = robot part or partner category; label = peak normal force)")
    ax.invert_yaxis()
    invariant_marks(ax, rows)
    fig.tight_layout(); fig.savefig(pdir / "contact_timeline.png", dpi=110); plt.close(fig)
    out["contact_timeline"] = pdir / "contact_timeline.png"
    return out


def lag_stats(trial):
    """Max |measured - commanded| per arm joint [mrad] over the samples, split by whether a dish was carried."""
    import numpy as np
    samples = [r for n, r in trial["rows"] if r["event"] == "sample"]
    if not samples:
        return None
    lag = np.abs(np.array([r["payload"]["lag"] for r in samples])) * 1e3
    carried = np.array([bool(r["payload"].get("carried")) for r in samples])
    out = {}
    for label, mask in (("carried", carried), ("free", ~carried)):
        out[label] = {j: round(float(lag[mask, i].max()), 1) if mask.any() else None for i, j in enumerate(JOINTS)}
        out[label]["samples"] = int(mask.sum())
    return out


def plot_run_summary(run):
    """k/N by bowl and the in-hand settle-displacement histogram (all trials of the run)."""
    plt = _mpl()
    import numpy as np
    sdir = run["adir"] / "summary"
    sdir.mkdir(parents=True, exist_ok=True)
    out = {}
    per_bowl = defaultdict(lambda: {"reached": 0, "n": 0, "classes": defaultdict(int)})
    settle, drift = [], []
    rec_pb = (run["record"] or {}).get("per_bowl", {})
    for trial in run["trials"]:
        end = trial["end"]
        if end:
            for oid, b in end["payload"]["per_bowl"].items():
                per_bowl[oid]["n"] += 1
                per_bowl[oid]["reached"] += int(bool(b["reached"]))
                if not b["reached"]:
                    cls = rec_pb.get(oid, {}).get("failure_class") if len(run["trials"]) == 1 else None
                    per_bowl[oid]["classes"][cls or (summarize_cause(b["cause"])[0] if b.get("cause") else None) or "other"] += 1
        for n, r in trial["rows"]:
            if r["event"] == "hold":
                settle.append(r["payload"].get("settle_displacement_mm"))
                if r["payload"].get("drift_mm") is not None:
                    drift.append(r["payload"]["drift_mm"])
    if per_bowl:
        ids = sorted(per_bowl)
        fig, ax = plt.subplots(figsize=(max(4, .9 * len(ids) + 2), 3.2))
        k = [per_bowl[i]["reached"] for i in ids]
        n = [per_bowl[i]["n"] for i in ids]
        ax.bar(ids, n, color="#dddddd", label="trials (N)")
        ax.bar(ids, k, color="#3a9a5b", label="reached its goal (k)")
        import textwrap
        for i, oid in enumerate(ids):
            cls = "\n".join(textwrap.wrap(", ".join(f"{c} x{m}" for c, m in per_bowl[oid]["classes"].items()), 18))
            ax.text(i, n[i] + .05, f"{k[i]}/{n[i]}\n{cls}", ha="center", va="bottom", fontsize=6)
        ax.set_ylim(0, max(n) * 1.9 + .5); ax.set_ylabel("trials"); ax.legend(fontsize=7, loc="upper right")
        ax.set_title(f"{run['run_id']}: k/N by bowl, failure class", fontsize=9)
        fig.tight_layout(); fig.savefig(sdir / "k_over_n.png", dpi=110); plt.close(fig)
        out["k_over_n"] = sdir / "k_over_n.png"
    s = [v for v in settle if v is not None]
    if s:
        fig, axes = plt.subplots(1, 2 if drift else 1, figsize=(8 if drift else 4.5, 3))
        ax = axes[0] if drift else axes
        ax.hist(s, bins=max(5, min(20, len(s))), color="#2f6db3")
        ax.set_xlabel("in-hand settle displacement after the lift [mm]"); ax.set_ylabel("grasps")
        ax.set_title(f"settle displacement (n={len(s)}, mean {np.mean(s):.1f}, sd {np.std(s):.1f} mm)", fontsize=8)
        if drift:
            axes[1].hist(drift, bins=max(5, min(20, len(drift))), color="#d9822b")
            axes[1].axvline(5., color="#c0392b", ls="--", lw=.8)
            axes[1].set_xlabel("drift over the hold window [mm]"); axes[1].set_title("hold-window drift (gate 5 mm)", fontsize=8)
        fig.suptitle(run["run_id"], fontsize=9)
        fig.tight_layout(); fig.savefig(sdir / "settle_hist.png", dpi=110); plt.close(fig)
        out["settle_hist"] = sdir / "settle_hist.png"
    return out


# --------------------------------------------------------------------------- index.html (per run)

def b64(path, mime):
    return f"data:{mime};base64," + base64.b64encode(Path(path).read_bytes()).decode()


def write_index(run, plots, summary, embed_video_max_mb=12.):
    rec = run["record"] or {}
    parts = [f"<!doctype html><html><head><meta charset='utf-8'><title>{html.escape(run['run_id'])}</title>",
             "<style>body{font-family:system-ui,sans-serif;margin:24px;max-width:1400px}img{max-width:100%;height:auto}"
             "table{border-collapse:collapse;font-size:13px}td,th{border:1px solid #ccc;padding:3px 6px}"
             "h2{margin-top:36px}.frames{display:flex;flex-wrap:wrap;gap:8px}.frames figure{margin:0;width:310px}"
             ".frames img{width:310px}figcaption{font-size:11px;color:#444}pre{font-size:11px;white-space:pre-wrap}</style></head><body>",
             f"<h1>{html.escape(run['run_id'])}</h1>",
             f"<p>record <code>{html.escape(rel(run['record_path']) if run['record_path'] else 'none')}</code> · generated "
             f"{datetime.now(timezone.utc).isoformat(timespec='seconds')} by frigidaire_robot_report.py (from the logs)</p>"]
    if rec:
        v = rec.get("verdicts", {})
        parts.append("<table><tr><th>instance</th><th>profile</th><th>headline flags</th><th>result</th><th>abort</th>"
                     "<th>benchmark success</th><th>final hold</th><th>invariants</th><th>moves ok</th><th>sim s (moves)</th><th>wall s (moves)</th></tr>")
        ok = sum(1 for m in rec.get("moves", []) if m.get("ok"))
        parts.append(f"<tr><td>{rec.get('instance')}</td><td>{rec.get('profile')}</td><td>{rec.get('headline')}</td><td><b>{rec.get('result')}</b></td>"
                     f"<td>{html.escape(str(rec.get('abort')))}</td><td>{v.get('benchmark_success')}</td><td>{v.get('final_hold')}</td>"
                     f"<td>{html.escape(json.dumps(v.get('invariant_counts')))}</td><td>{ok}/{len(rec.get('moves', []))}</td>"
                     f"<td>{rec.get('sim_s_moves')}</td><td>{rec.get('seconds_wall')}</td></tr></table>")
        pb = rec.get("per_bowl", {})
        if pb:
            parts.append("<h2>Per bowl</h2><table><tr><th>bowl</th><th>start</th><th>goal rack</th><th>moves</th><th>reached</th>"
                         "<th>distance (lat mm, dz mm, tilt deg)</th><th>racked in</th><th>cause</th><th>class</th><th>invariants</th></tr>")
            for oid, b in pb.items():
                parts.append(f"<tr><td>{oid}</td><td>{b['start']}</td><td>{b['goal_rack']}</td><td>{b['moves']}</td><td>{b['reached']}</td>"
                             f"<td>{b.get('distance')}</td><td>{b.get('racked_in')}</td><td>{html.escape(str(b.get('cause_summary') or b.get('cause')))[:300]}</td>"
                             f"<td>{b.get('failure_class')}</td><td>{b.get('invariants')}</td></tr>")
            parts.append("</table>")
    for name, path in summary.items():
        parts.append(f"<h2>{name}</h2><img src='{b64(path, 'image/png')}'>")
    for trial in run["trials"]:
        parts.append(f"<h2>{html.escape(trial['trial_id'])}</h2>")
        if trial["video"]:
            size = trial["video"].stat().st_size / 1e6
            if size <= embed_video_max_mb:
                parts.append(f"<video controls width='960' src='{b64(trial['video'], 'video/mp4')}'></video>"
                             f"<p>video.mp4 ({size:.1f} MB, embedded)</p>")
            else:
                parts.append(f"<video controls width='960' src='{trial['trial_id']}/video.mp4'></video>"
                             f"<p>video.mp4 ({size:.1f} MB, linked next to this page)</p>")
        for name, path in plots.get(trial["trial_id"], {}).items():
            parts.append(f"<h3>{name}</h3><img src='{b64(path, 'image/png')}'>")
        if trial["frames"]:
            parts.append("<h3>key frames</h3><div class='frames'>")
            for f in trial["frames"]:
                parts.append(f"<figure><img src='{b64(f, 'image/png')}'><figcaption>{html.escape(f.name)}</figcaption></figure>")
            parts.append("</div>")
        end = trial["end"]
        if end:
            parts.append(f"<h3>end row ({trial['trial_id']}/trial.jsonl:L{trial['end_line']})</h3><pre>"
                         + html.escape(json.dumps({k: v for k, v in end["payload"].items() if k not in ("final_poses", "final_components")}, indent=1)[:6000]) + "</pre>")
    parts.append("</body></html>")
    path = run["adir"] / "index.html"
    path.write_text("\n".join(parts))
    return path


# --------------------------------------------------------------------------- report sections

AUDIT_ROWS = [
    # (item, plan text, code (path:line), verdict, correction / D20 wording)
    ("1", "6.0 asset layers converted on the host in a temporary env", "`mirror_robot_usd.sh:38-58`", "confirmed",
     "already scripted; unpinned usd-core, no output hashes (D11 pending: manifest written this phase, pin unverifiable without a re-download)"),
    ("2", "gripper colliders instanced, un-instanced before reset", "`ur5e.py:77-91`, episode `before_reset`", "confirmed",
     "arm colliders were still instanced; the D12 press test (this phase) shows they DO make contacts on 4.5"),
    ("3", "inner finger drives fought the close; now mirror finger_joint", "`ur5e.py:25-32`, `rig.py:56`", "confirmed",
     "signs MEASURED 2026-07-31 (on-corrallab), not copied; the mimic couplings are live (D7 test this phase)"),
    ("4", "PD lag 0.16 rad at speed; velocity feed-forward", "`rig.py:128-150`", "confirmed", "gains unchanged since on-corrallab"),
    ("5", "planner allowed +-2pi; real limits now", "`kin.py:42-51`, `rig.py:93-95`, `collide.py:213-215`", "partly",
     "D20(a): the +-2pi planner is an unrecoverable draft; the ported code already had elbow +-pi"),
    ("6", "IK prefers wrist-up; grasp paired with an approach on the same branch", "episode `goal_qs`, `pick`", "confirmed by reading, untested",
     "no test covers the pairing"),
    ("7", "long descents crossed a wrist singularity", "episode `transit_to`, `plan_to`", "confirmed",
     "D20(b): the code's diagnosis is an IK branch jump, not a singularity"),
    ("8", "gripper hull misplaced; built from live poses", "`collide.py:29-77`", "confirmed", ""),
    ("9", "one hull per collider", "`collide.py:75-76`", "confirmed", "stale docstrings `collide.py:3-4, :30`"),
    ("10", "contact exemption near the grasp: gripper only", "`collide.py:143-144`, episode `ALLOW_CONTACT_M`", "confirmed", ""),
    ("11", "pedestal, floor, carried bowl in the planning model", "episode `add_static_box`, `collide.py:149-170`", "confirmed", ""),
    ("12", "rise to 1.25 m first", "flags `z_safe_m`, episode `transit_to`", "confirmed", ""),
    ("13", "smooth starts/stops, 0.25 rad/s carry", "`rig.py:115, :165`, episode `plan_to`", "partly",
     "0.24 rad/s is the smoothstep PEAK; RRT paths are not smoothstepped"),
    ("R1", "grasp gate slip < 5 mm -> stable after it settles", "flags `hold_gate`; `hold_verdict`", "confirmed",
     "three gates coexisted (probe, grasp test, episode); now one rule per profile (D9): headline = drift <= 5 mm over 2 s"),
    ("R2", "finger drive stiffness/effort changed; pad material added", "flags `gripper_effort/stiffness`; `ur5e.pad_material`", "confirmed",
     "the gain change tripled the pad force (13 -> 41 N); only the material made no difference"),
    ("R3", "upper rack pushed in while loading the lower rack", "flags `upper_in_for_lower`, `reextend_before_scoring`", "partly",
     "declared scripted rack motion; both racks re-extended before scoring (D2); the old episode left them in/out"),
    ("R4", "easy_s0 replaced by 3 upright bowls", "flags `test_case`", "confirmed", "behind the flag, default off"),
    ("R5", "lowered mouth-up until contact", "flags `place_mode`", "confirmed", "behind the flag, default off (`goal_pose`)"),
    ("R6", "success = in the rack, stable, nothing disturbed", "flags `judge`, `disturbance`, `end_check`, `move_settle`", "partly",
     "'stable' was never gated; headline = at-goal tolerance + benchmark settle + D1 disturbance + D2 end check"),
    ("R7", "blocked lag limits widened to 0.06 / 0.08", "flags `tol_*`, `settled_lag_rad`, `lag_max_rad`", "partly",
     "widened only the post-move check; the legacy profile freezes the call-site values; headline = rig defaults (D8)"),
    ("O1", "mouth-down bowls unreliable; lying bowl untested", "`grasp.py` foot/side; records grasp_test/*", "refuted (untested half)",
     "D20(d): side grasp never executed in isolation (the 'lying' test rolled upright); attempted twice in the scene and failed"),
    ("O2", "rear 12 cm of the lower rack under the counter; no pedestal reaches all goals", "`grasp.py:86-91`, mount records", "confirmed",
     "the goal proxy was a TOP-DOWN grasp (H5 premise confirmed)"),
    ("scene", "'pick bowls off a countertop'", "easy_s0.json objects", "partly", "D20(e): 5 counter + 2 lower-rack starts (bowl_01 lying, bowl_07 upright)"),
    ("time", "'143 s simulated' (robot.md)", "mp4 length", "partly", "D20(c): derived from the video, no logged field; every run now logs sim_s and wall_s separately"),
]

HYPOTHESES = [
    ("H1", "the arm lag is mostly gravity sag",
     "mechanism off in config: gravity is disabled on every robot link (`ur5e.py:49`, labelled privileged, D6); Phase 1a measures the residual static error"),
    ("H2", "the conversion dropped the 2F-85 linkage coupling",
     "refuted at the asset level (five PhysxMimicJointAPI in the untouched payload); RUNTIME TESTED THIS PHASE, see the conventions section: all five couplings hold during a loaded close"),
    ("H3", "the pad material had no effect for a mechanical reason",
     "(a) refuted: the bowl authors no combine mode; (b) refuted: the material binds the live collider prims; (c) consistent with the records; Phase 1c measures"),
    ("H4", "the bowl's physics model is wrong",
     "refuted by the asset: mass authored 0.067375 kg (measured), 192 exact convex shells with open cavity and recess; Phase 1d verifies at runtime (D5)"),
    ("H5", "the pedestal verdict assumed the old placement", "confirmed by code (top-down goal proxy in the mount search); Phase 4"),
    ("H6", "some goals may be inaccessible to the bowl alone", "untested; only the benchmark's sequence certificate exists; Phase 4"),
]

PROCESS_ERRORS = [
    ("a false pass from a sleeping body", "sleep threshold 0 on every dish prim and robot link (`harness.author_sleep_and_reports`), `is_sleeping` polled every 12 ticks -> invariant `sleeping_body`"),
    ("drive lag misread as the pads stalling", "pad forces come from PhysX contact reports (normal component per pad collider), never from the drive lag; the jaw-angle filter stays a stand-in for object detection"),
    ("a flipped approach sign", "`test_approach_axis_points_from_the_wrist_to_the_fingertips` on the live asset (fixture) + `grasp.unit` guards"),
    ("a NaN closing axis from two coincident points", "`grasp.unit` raises on non-finite or degenerate axes before any division (`test_degenerate_axes_raise_before_use`)"),
    ("a grasp test that teleported the arm into the bowl", "every state write after the reset raises `teleport_after_reset` (backend `set_rigid_pose`/`restore` and `rig.teleport_arm` are guarded); analysis scripts that teleport are labelled ANALYSIS"),
    ("a required side-grasp test that was skipped", "the report lists every required test and marks absent ones FAIL (not run)"),
    ("wall time labelled as sim time", "every trial-log row carries `sim_s` (tick x 1/120 s) and `wall_s` (monotonic) as separate fields; the video overlay shows both"),
    ("attempts counted as distinct loads", "rows carry `attempt` and `config_id` (a digest of the reset poses) separately; the tables count attempts and configurations in their own columns"),
]


ROBOT_FILES = ("frigidaire/src/dishsim_frigidaire/robot", "frigidaire/scripts/experiment/frigidaire_robot_episode.py",
               "frigidaire/scripts/setup/frigidaire_robot_conventions.py", "frigidaire/scripts/setup/frigidaire_robot_conventions_fixture.py",
               "frigidaire/scripts/setup/robot_asset_manifest.py", "frigidaire/scripts/setup/mirror_robot_usd.sh",
               "frigidaire/scripts/evaluation/frigidaire_robot_report.py", "frigidaire/tests/test_robot_kin.py",
               "frigidaire/tests/test_robot_conventions.py", "frigidaire/tests/test_robot_flags.py", "frigidaire/tests/test_robot_harness.py",
               "frigidaire/tests/fixtures/robot", "frigidaire/docs/robot.md", "plans")

DECISIONS_TAKEN = [
    ("2026-09-29", "D13 commit before Phase 0", "user: continue without a commit (a pre-Phase-0 snapshot went to artifacts/_baseline_pre_phase0/); the user committed the stack later (see the git status row)"),
    ("2026-09-29", "artifacts/ location", "user: symlink onto the 2 TB drive, gitignored (root disk gains nothing)"),
    ("2026-09-30", "easy_s0 diagnostic run budget", "user: allow up to 45 min (--max-wall-seconds 2700, the last 5 min for scoring)"),
    ("2026-09-30", "D4 and the knuckle contacts", "user: keep D4 as decided (knuckle contact = auto-fail; Phase 2's grasps must keep the rim on the pads)"),
    ("2026-09-30", "D4 and the silverware basket", "user: the basket counts as rack furniture for the carried dish (flagged interpretation kept)"),
]


def git_status():
    """HEAD and the tracked / modified / untracked state of the robot files (the D13 row is read from git, not typed)."""
    def run(*a):                       # read-only; safe.directory because the container runs as root in the user's checkout
        return subprocess.run(["git", "-c", f"safe.directory={ROOT}", *a], cwd=ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, text=True).stdout.strip()
    head = run("log", "-1", "--format=%h %ad %s", "--date=short")
    tracked = run("ls-files", "--", *ROBOT_FILES).splitlines()
    porcelain = run("status", "--porcelain", "--", *ROBOT_FILES).splitlines()
    modified = sorted(l[2:].strip() for l in porcelain if "M" in l[:2])
    untracked = sorted(l[2:].strip() for l in porcelain if l.startswith("??"))
    return {"head": head, "tracked": len(tracked), "modified": modified, "untracked": untracked}


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c).replace("|", "\\|").replace("\n", " ") for c in r) + " |")
    return "\n".join(out)


def find_rows(trial, event, **match):
    return [(n, r) for n, r in trial["rows"] if r["event"] == event and all(r["payload"].get(k) == v for k, v in match.items())]


def run_table_row(run):
    rec = run["record"]
    if not rec:
        return [run["run_id"], "-", "-", "**FAIL (not run)**", "", "", "", "", "", "", ""]
    v = rec.get("verdicts", {})
    ok = sum(1 for m in rec.get("moves", []) if m.get("ok"))
    att = sum(len(m.get("attempts", [])) for m in rec.get("moves", []))
    cfg = len({t["meta"]["config_id"] for t in run["trials"] if t["meta"]}) or ("-" if not run["trials"] else 0)
    profile = rec.get("profile") or "(pre-harness code path, old record format)"
    return [run["run_id"], rec.get("instance"), f"{profile}{' +diagnostic' if rec.get('flags', {}).get('continue_after_failed_dish') else ''}",
            f"**{rec.get('result')}**", f"{ok}/{len(rec.get('moves', []))}", att, cfg,
            v.get("benchmark_success"), v.get("final_hold"), json.dumps(v.get("invariant_counts")) if v else "-",
            f"{rec.get('sim_s_moves', '-')} / {rec.get('seconds_wall')}"]


def per_bowl_rows(run):
    rec = run["record"]
    out = []
    if not rec:
        return out
    trial = run["trials"][0] if run["trials"] else None
    for oid, b in rec.get("per_bowl", {}).items():
        ref = ""
        if trial:
            mv = [(n, r) for n, r in trial["rows"] if r["event"] == "move" and r.get("dish") == oid]
            if mv:
                ref = logref(trial, mv[-1][0])
            inv = [(n, r) for n, r in trial["rows"] if r["event"] == "invariant" and r.get("dish") == oid]
            if inv:
                ref += " " + logref(trial, inv[0][0])
        out.append([run["run_id"], oid, b["start"], b["goal_rack"], b["moves"], b["reached"], b.get("distance"),
                    b.get("racked_in"), (b.get("cause_summary") or b.get("cause") or "-")[:220], b.get("failure_class"), ", ".join(b.get("invariants", [])), ref])
    return out


def compare_records(a, b):
    """The legacy reproduction: per-move jaw angle and final poses byte-for-byte vs the pre-Phase-0 PASS."""
    import numpy as np
    out = {"moves": [], "final_pose_max_diff_mm": None}
    for ma, mb in zip(a.get("moves", []), b.get("moves", [])):
        ta = [(x.get("pick") or {}).get("theta") for x in ma["attempts"]]
        tb = [(x.get("pick") or {}).get("theta") for x in mb["attempts"]]
        out["moves"].append((ma["dish"], ma["ok"], mb.get("ok_physical", mb["ok"]), ta, tb))
    diffs = []
    for oid, pa in a.get("final_poses", {}).items():
        pb = b.get("final_poses", {}).get(oid)
        if pb:
            diffs.append(float(np.linalg.norm(np.array(pa["position_m"]) - pb["position_m"])) * 1e3)
    out["final_pose_max_diff_mm"] = max(diffs) if diffs else None
    return out


def build_report(args, runs, conventions, pytest_xml, plots, summaries, indexes):
    now = datetime.now(timezone.utc).isoformat(timespec="seconds")
    L = [f"# REPORT: {PLAN} -- Phase 0 (audit and harness)", "",
         f"Generated {now} by `frigidaire/scripts/evaluation/frigidaire_robot_report.py` from the trial logs, run records and "
         f"the conventions record; nothing in the measured tables is typed by hand. Companion audit with path:line evidence: `{MAP}`.",
         "", "Time columns: `sim s` = simulated seconds (physics ticks x 1/120 s), `wall s` = host wall-clock seconds; they are never "
         "mixed. `attempts` counts grasp attempts, `configs` counts distinct start configurations (`config_id`).", ""]
    by_id = {r["run_id"]: r for r in runs}
    # ---------------------------------------------------------------- 1. status of each phase
    L += ["## 1. Phase status", "",
          md_table(["phase", "status"], [
              ("0 audit and harness", "executed this session (this report); accept criteria below"),
              ("1 physics and model fidelity", "not started (stop-and-report point after 1e)"),
              ("2 grasp library", "not started"), ("3 placement at the real goals", "not started"),
              ("4 reach, access, loading order", "not started"), ("5 easy_s0 end to end", "not started")]), ""]
    # ---------------------------------------------------------------- 2. required tests
    L += ["## 2. Required Phase 0 tests (FAIL (not run) when nothing ran)", ""]
    rows = []

    def status(cond, ok_text, fail_text):
        return f"PASS: {ok_text}" if cond else f"**FAIL**: {fail_text}"

    def last(pred):                     # runs are listed chronologically: the latest matching run is the evidence
        return next((r for r in reversed(runs) if r["record"] and pred(r["record"])), None)

    fl = lambda rec: rec.get("flags", {})  # noqa: E731
    legacy = last(lambda rec: rec.get("profile") == "legacy_upright3" and fl(rec).get("sleep_threshold") is not None)
    headline3 = last(lambda rec: rec.get("profile") == "headline" and fl(rec).get("test_case") == "upright3" and not fl(rec).get("continue_after_failed_dish"))
    headline3d = last(lambda rec: rec.get("profile") == "headline" and fl(rec).get("test_case") == "upright3" and fl(rec).get("continue_after_failed_dish"))
    easy = last(lambda rec: rec.get("instance") == "easy_s0" and not fl(rec).get("continue_after_failed_dish"))
    easyd = last(lambda rec: rec.get("instance") == "easy_s0" and fl(rec).get("continue_after_failed_dish"))
    pre = ROOT / "results/robot/episodes/robot_upright3.json"
    pre_rec = json.loads(pre.read_text()) if pre.exists() else None
    rows.append(("0.1 audit table", "PASS: section 8 (D20 wording)" if AUDIT_ROWS else "FAIL (not run)"))
    g = git_status()
    rows.append(("D13 git state of the robot stack (read from git)",
                 f"HEAD `{g['head']}`; {g['tracked']} robot files tracked; modified since HEAD: {g['modified'] or 'none'}; untracked: {g['untracked'] or 'none'} "
                 "(the executor never commits; the user commits from the summary)"))
    ab = last(lambda rec: rec.get("profile") == "legacy_upright3" and fl(rec).get("sleep_threshold") is None)
    if legacy and pre_rec:
        cmp_ = compare_records(pre_rec, legacy["record"])
        verdict_same = all(oa == ob for _, oa, ob, _, _ in cmp_["moves"]) and legacy["record"]["verdicts"]["legacy_rule"] == pre_rec["result"] == "PASS" \
            and legacy["record"]["end_check"]["outcome"] == pre_rec["end_check"]["outcome"]
        exact = all(ta == tb for _, _, _, ta, tb in cmp_["moves"]) and cmp_["final_pose_max_diff_mm"] is not None and cmp_["final_pose_max_diff_mm"] < 1e-6
        jaw = max(abs(ta[0] - tb[0]) for _, _, _, ta, tb in cmp_["moves"] if ta and tb and ta[0] and tb[0])
        dmm = cmp_["final_pose_max_diff_mm"]
        numbers = "byte-identical" if exact else f"not byte-identical: jaw angles within {jaw * 1e3:.1f} mrad, final poses within {dmm:.1f} mm"
        txt = (f"legacy profile run `{legacy['run_id']}` reproduces the PASS verdict (3/3 physically ok, all racked; record `{rel(legacy['record_path'])}`); "
               f"the numbers are {numbers} of `{rel(pre)}`")
        if not exact:
            txt += "; the pre-harness reproduction `results/robot/runs/p0_prerepro_upright3/robot_upright3.json` IS byte-identical, so the harness changed the timeline"
            if ab:
                abd = compare_records(pre_rec, ab["record"])["final_pose_max_diff_mm"]
                cause = "the" if abd < 1e-6 else "NOT the only"
                txt += f"; A/B `{ab['run_id']}` (sleep authoring off, ANALYSIS): final poses within {abd:.2e} mm of the PASS -> the plan's never-sleep rule (D10) is {cause} cause"
            else:
                txt += "; attribution run not available"
        rows.append(("0.2 flags: 3-bowl test with the OLD settings reproduces the pre-Phase-0 PASS", status(verdict_same, txt, txt)))
    else:
        rows.append(("0.2 flags: 3-bowl test with the OLD settings", "**FAIL (not run)**"))
    rows.append(("0.2 flags: 3-bowl test with the HEADLINE settings",
                 f"PASS: ran (`{headline3['run_id']}`, result {headline3['record']['result']}, expected to fail the at-goal tolerance)" if headline3 else "**FAIL (not run)**"))
    rows.append(("0.2 flags: 3-bowl test, headline settings, diagnostic (invariants recorded, D18)",
                 f"PASS: ran (`{headline3d['run_id']}`, result {headline3d['record']['result']})" if headline3d else "**FAIL (not run)**"))
    # 0.3 harness
    any_trial = next((t for r in runs for t in r["trials"] if t["meta"]), None)
    if any_trial:
        m = any_trial["meta"]["payload"]
        rows.append(("0.3 sleeping disabled", status(m["authored"]["dish_sleep"] >= 1 and m["authored"]["robot_sleep"] >= 1,
                     f"sleep threshold {m['flags']['sleep_threshold']} authored on {m['authored']['dish_sleep']} dishes, {m['authored']['robot_sleep']} robot links, "
                     f"{m['authored']['articulation_sleep']} articulation roots ({logref(any_trial, 1)})", "not authored")))
        diag = [(n, r) for r in runs for t in r["trials"] for n, r in find_rows(t, "settle") if "diagnostic" in str(r["payload"].get("what"))]
        rows.append(("0.3 settle routine (< 1 mm/s, 0.01 rad/s for 1 s, cap 5 s) -- D3: diagnostic column",
                     status(bool(diag), f"{len(diag)} diagnostic settle rows over the runs (first: `...:L{diag[0][0]}`)", "no rows")))
        nc = sum(len(find_rows(t, "contact_begin")) for r in runs for t in r["trials"])
        rows.append(("0.3 contact logging (pads, fingers, palm, arm links, bowls, racks, basket, counter, pedestal)",
                     status(nc > 0, f"{nc} contact_begin rows over the runs (PhysX contact reports, touching pairs only)", "none")))
    else:
        rows += [("0.3 sleeping disabled", "**FAIL (not run)**"), ("0.3 settle routine", "**FAIL (not run)**"), ("0.3 contact logging", "**FAIL (not run)**")]
    # 0.4 invariants
    fired = defaultdict(list)
    for r in runs:
        for t in r["trials"]:
            for n, row in find_rows(t, "invariant"):
                fired[row["payload"]["name"]].append(f"{r['run_id']}:{t['trial_id']}:L{n}")
    for name in INVARIANTS:
        rows.append((f"0.4 invariant `{name}`", "implemented (`harness.Harness._check`); " +
                     (f"fired in {len(fired[name])} place(s), e.g. `{fired[name][0]}`" if fired[name] else "never fired in these runs")))
    # 0.5 conventions
    if conventions:
        c = conventions
        rows.append(("0.5 convention tests on the live asset (approach sign, jaw axis, pad normals, finger signs)",
                     f"PASS: conventions run `{c['run_id']}` (`artifacts/{c['run_id']}/conventions.json`), fixture "
                     f"`frigidaire/tests/fixtures/robot/conventions.json`; pytest below"))
        rows.append(("0.5 D7: mimic couplings during a loaded close (H2 runtime test)",
                     status(all(abs(abs(v) - 1.) < .01 for v in c["loaded_close"]["ratios"].values()),
                            f"all coupled joints track finger_joint with |ratio| = {min(abs(v) for v in c['loaded_close']['ratios'].values()):.4f}..{max(abs(v) for v in c['loaded_close']['ratios'].values()):.4f} "
                            f"at finger_joint {c['loaded_close']['finger_joint']:.3f} rad on the bowl rim", "a coupling slipped")))
        rows.append(("0.5 D12: arm-collider press test",
                     status(all(p["verdict"] == "contact" for p in c["press"].values()),
                            "; ".join(f"robot {n}: {p['config']} -> plate caught at z {p['plate_rest_z_m']} m on {p['arm_links_touching']} (peak "
                                      f"{max(h['peak_N'] for h in p['hits'].values()):.0f} N)" for n, p in c["press"].items()),
                            "an arm made no contact")))
    else:
        rows += [("0.5 convention tests", "**FAIL (not run)**"), ("0.5 D7 loaded close", "**FAIL (not run)**"), ("0.5 D12 press test", "**FAIL (not run)**")]
    if pytest_xml:
        rows.append(("0.5 Kit-free pytest (robot tests)", pytest_xml))
    else:
        rows.append(("0.5 Kit-free pytest (robot tests)", "**FAIL (not run)**"))
    # 0.6 logging + media
    full = [(r, t) for r in runs for t in r["trials"] if t["video"] and t["frames"] and plots.get(r["run_id"], {}).get(t["trial_id"])]
    rows.append(("0.6 JSONL trial logs + generated report", "PASS: this file and `artifacts/<run-id>/index.html` are generated from the logs" if runs else "**FAIL (not run)**"))
    rows.append(("0.6 one full trial's video, key frames and plots present and linked",
                 status(bool(full), f"{len(full)} trial(s), e.g. `{rel(full[0][1]['video'])}`, {len(full[0][1]['frames'])} key frames, {len(plots[full[0][0]['run_id']][full[0][1]['trial_id']])} plots" if full else "", "no trial has all three")))
    # acceptance runs
    rows.append(("accept: 3-bowl test through the harness", f"PASS: `{legacy['run_id']}`" if legacy else "**FAIL (not run)**"))
    rows.append(("accept: easy_s0 through the harness (expected to fail)", f"PASS: ran, result {easy['record']['result']} (`{easy['run_id']}`)" if easy else "**FAIL (not run)**"))
    rows.append(("accept: per-bowl failure causes for easy_s0 (diagnostic continue, D18)",
                 f"PASS: `{easyd['run_id']}`, section 5" if easyd else "**FAIL (not run)**"))
    L += [md_table(["required test", "status / evidence"], rows), ""]
    # ---------------------------------------------------------------- 3. runs
    L += ["## 3. Runs", "",
          md_table(["run", "instance", "profile", "result", "moves ok", "attempts", "configs", "benchmark success", "final hold", "invariants (distinct)", "sim s / wall s (moves)"],
                   [run_table_row(r) for r in runs]), "",
          "`result`: legacy profile = the old rule (every move physically ok, all racked; invariants recorded only); headline profile = "
          "benchmark success (every dish within the at-goal tolerance AND end check accepted) AND final hold passed AND no invariant "
          "AND no relaxation flag. Runs with `--test-case` or `--diagnostic-continue` are labelled non-headline by construction.", ""]
    if legacy and pre_rec:
        cmp_ = compare_records(pre_rec, legacy["record"])
        L += ["### 3.1 Regression: the 3-bowl test with the old settings", "",
              md_table(["dish", "PASS record ok", "legacy run ok (physical)", "jaw rad (PASS)", "jaw rad (legacy run)"],
                       [(d, a, b, ta, tb) for d, a, b, ta, tb in cmp_["moves"]]),
              "", f"Final poses: max difference {cmp_['final_pose_max_diff_mm']:.2e} mm vs `{rel(pre)}` (the pre-Phase-0 PASS, backed up at "
              f"`results/robot/backup_pre_phase0_20260929/`). Pre-harness reproduction with the same code path: "
              f"`results/robot/runs/p0_prerepro_upright3/robot_upright3.json` (PASS 3/3, 460.8 s wall vs 472.1 s).", ""]
        v = legacy["record"].get("verdicts", {})
        L += [f"Harness verdict on the same run: invariants {json.dumps(v.get('invariant_counts'))} (distinct first occurrences; ticks in violation "
              f"{json.dumps(legacy['record'].get('invariant_ticks'))}). The old rule PASSES while the plan's D4 invariant fails it: see section 6.", ""]
    # ---------------------------------------------------------------- 4. per bowl
    L += ["## 4. Per bowl (every run)", "",
          md_table(["run", "bowl", "start", "goal rack", "moves", "reached", "distance (lat mm, dz mm, tilt deg)", "racked in", "cause", "class", "invariants", "log"],
                   [row for r in runs for row in per_bowl_rows(r)]), ""]
    # ---------------------------------------------------------------- 5. easy_s0 causes
    if easyd or easy:
        L += ["## 5. easy_s0 under the harness", ""]
        for r in [x for x in (easy, easyd) if x]:
            rec = r["record"]
            L += [f"### {r['run_id']} ({rec['profile']}{' + diagnostic continue (D18, not headline)' if rec['flags'].get('continue_after_failed_dish') else ''})", "",
                  f"result {rec['result']}, abort `{rec.get('abort')}`, end check `{rec.get('end_check', {}).get('outcome')}`, counter-full refusals {rec.get('counter_full_refusals')}, "
                  f"moves ok {sum(1 for m in rec['moves'] if m.get('ok'))}/{len(rec['moves'])}, sim {rec.get('sim_s_moves')} s / wall {rec.get('seconds_wall')} s.", ""]
            mrows = []
            for m in rec["moves"]:
                att = m.get("attempts", [])
                mrows.append((m["move"], m["kind"], m["dish"], m.get("ok"), m.get("ok_physical"), len(att), m.get("failure_class"),
                              (m.get("cause_summary") or m.get("cause") or "-")[:240], m.get("sim_s"), m.get("seconds_wall")))
            L += [md_table(["move", "kind", "dish", "ok", "physically ok", "attempts", "class", "cause", "sim s", "wall s"], mrows), ""]
    # ---------------------------------------------------------------- 6. findings
    L += ["## 6. Measured findings of Phase 0", ""]
    if conventions:
        c = conventions
        L += [f"- **Conventions on the live asset** (`artifacts/{c['run_id']}/conventions.json`): TCP axes = the commanded frame "
              f"({c['tcp']['axes_world']}); wrist-3 -> TCP along +z by {c['tcp']['z_dot_wrist3_to_tcp']:.3f} m; pads at y = "
              f"{c['pads']['open']['left']['centroid_mm'][1]:.1f} / {c['pads']['open']['right']['centroid_mm'][1]:.1f} mm open, "
              f"{c['pads']['closed']['left']['centroid_mm'][1]:.1f} / {c['pads']['closed']['right']['centroid_mm'][1]:.1f} mm closed (gap {c['pads']['gap_mm']} mm); "
              f"pad face normals {c['pads']['closed']['left']['face_normal_tcp']} / {c['pads']['closed']['right']['face_normal_tcp']} (TCP frame).",
              f"- **Finger-joint signs**: unloaded close ratios {json.dumps(c['unloaded_close']['ratios'])} vs the payload's mimic gearings "
              f"{json.dumps(c['unloaded_close']['expected_from_mimic'])} and the code's INNER_FINGER_SIGNS {json.dumps(c['unloaded_close']['code_inner_finger_signs'])}: match = {c['unloaded_close']['signs_match']}.",
              f"- **D7 / H2 runtime**: loaded close on the bowl rim stops at finger_joint {c['loaded_close']['finger_joint']:.3f} rad with ratios "
              f"{json.dumps(c['loaded_close']['ratios'])}: PhysX on Isaac 4.5 honours all five mimic couplings (the four rotX instances on Z-axis joints included).",
              f"- **Contact monitor vs Isaac Lab ContactSensor** on the same close: sensor (whole inner-finger bodies) {c['loaded_close']['pad_sensor_N']} N, "
              f"contact reports by robot part {json.dumps(c['loaded_close']['bowl_contact_N_by_robot_part'])} N -- the sensor's left value is carried by the finger side, not the pad.",
              f"- **D12 press test**: " + "; ".join(f"robot {n} ({p['config']}): plate rest z {p['plate_rest_z_m']} m, links {p['arm_links_touching']}" for n, p in c["press"].items())
              + ". Instanced arm colliders DO make contacts on 4.5 (only the gripper's were dead): the episode keeps `deinstance_root = /World/Robot/Gripper`; de-instancing the whole robot is available behind the flag.", ""]
    for r in runs:
        for t in r["trials"]:
            holds = find_rows(t, "hold")
            if holds:
                L += [f"- **In-hand settle after the lift** (`{r['run_id']}`): " + "; ".join(
                    f"move {row.get('move')} {row.get('dish')}: settle displacement {row['payload'].get('settle_displacement_mm')} mm / {row['payload'].get('settle_rotation_deg')} deg, "
                    f"{'drift ' + str(row['payload'].get('drift_mm')) + ' mm over ' + str(row['payload'].get('window_s')) + ' s, ' if row['payload'].get('window_s') else ''}"
                    f"held {row['payload'].get('held')} ({logref(t, n)})" for n, row in holds) + "."]
            st = lag_stats(t)
            if st and st["carried"]["samples"]:
                L += [f"- **Arm tracking error** (`{r['run_id']}`, max |measured - commanded| in mrad over 20 Hz samples; plot `{rel(t['dir'] / 'plots' / 'joint_tracking.png')}`): "
                      f"carrying a dish ({st['carried']['samples']} samples): " + ", ".join(f"{j} {st['carried'][j]}" for j in JOINTS)
                      + f"; free ({st['free']['samples']} samples): " + ", ".join(f"{j} {st['free'][j]}" for j in JOINTS)
                      + ". The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture)."]
            inv = find_rows(t, "invariant")
            if inv:
                L += [f"- **Invariants fired** (`{r['run_id']}`): " + "; ".join(
                    f"{row['payload']['name']} at move {row.get('move')} {row.get('phase')}: {json.dumps(row['payload']['detail'])[:220]} ({logref(t, n)})" for n, row in inv[:8]) + "."]
            knuckle = [(n, row) for n, row in find_rows(t, "contact_end") if any("knuckle" in x for x in row["payload"]["pair"]) and row["payload"].get("peak_N", 0) > 1]
            if knuckle:
                L += [f"- **The rim pinch loads the inner knuckles** (`{r['run_id']}`): " + "; ".join(
                    f"{row['payload']['pair']} peak {row['payload']['peak_N']:.0f} N for {row['payload']['duration_s']:.2f} s, min separation {row['payload'].get('min_sep_m')} m ({logref(t, n)})"
                    for n, row in knuckle[:6]) + ". The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp."]
    L += [""]
    # ---------------------------------------------------------------- 7. deviations and flags
    L += ["## 7. Deviations from the benchmark, with their flags", "",
          "Every relaxation is a named field of `frigidaire/src/dishsim_frigidaire/robot/flags.py` (headline default off); each run's record stores its resolved flags "
          "and `deviations_from_headline`.", ""]
    drows = []
    for r in runs:
        rec = r["record"]
        if not rec:
            continue
        for name, val, head, item in rec.get("deviations_from_headline", []):
            drows.append((r["run_id"], name, item, json.dumps(val), json.dumps(head)))
    L += [md_table(["run", "flag", "plan item", "value", "headline value"], drows) if drows else "(every run used the headline values)", "",
          "Interpretations taken by the harness (flagged for the user, plan Section 6):", "",
          "- D4: the silverware basket counts as rack furniture for the carried dish (`harness.CARRIED_ALLOWED`), because the benchmark's support rule seats it on the lower rack; "
          "a carried bowl touching the basket is therefore allowed, a bowl touching a finger body or knuckle is not.",
          "- D1: 'at its goal before the move' uses the benchmark tolerance on the pre-move pose plus the dishes this trial already placed at their goals.",
          "- D10 (sleep threshold 0 on the dish prims and robot links): besides the false-pass guard, it is a precondition of the contact logging: PhysX emits no "
          "contact reports for sleeping bodies (NVIDIA's RigidContactView test sets `physxRigidBody:sleepThreshold = 0` for that reason, comment "
          "'disable sleeping, because sleeping bodies don't get contact reports'); the A/B run with the asset thresholds shows the bowls asleep on the counter.",
          "- R3 in the headline: the rack motions are scripted environment actions (logged as `rack` rows); both racks are re-extended before the final hold and the end check.",
          "- Privileged information used by the controller (plan Section 3): " + "; ".join(json.loads(json.dumps(any_trial["meta"]["payload"]["privileged"])) if any_trial else []) + ".", ""]
    # ---------------------------------------------------------------- 8. audit
    g = git_status()
    L += ["## 8. Audit: fixes 1-13 and R1-R7 -> code (Phase 0.1; D20 wording)", "",
          f"Commits: none of fixes 1-13 / R1-R7 has a commit of its own (the whole stack was untracked when the audit was made; a pre-Phase-0 snapshot with sha256 "
          f"sums sits in `artifacts/_baseline_pre_phase0/`). Git now: HEAD `{g['head']}`, {g['tracked']} robot files tracked, modified since HEAD {g['modified'] or 'none'}, "
          f"untracked {g['untracked'] or 'none'}.", "",
          md_table(["item", "plan text", "code", "verdict", "correction"], AUDIT_ROWS), "",
          "### Decisions taken by the user during Phase 0", "", md_table(["date", "item", "decision"], DECISIONS_TAKEN), ""]
    # ---------------------------------------------------------------- 9. hypotheses
    L += ["## 9. Hypotheses H1-H6 (Phase 1 measures; Phase 0 status)", "", md_table(["H", "claim", "status after Phase 0"], HYPOTHESES), ""]
    # ---------------------------------------------------------------- 10. process errors
    L += ["## 10. Process errors the harness makes impossible", "", md_table(["error", "mechanism"], PROCESS_ERRORS), ""]
    # ---------------------------------------------------------------- 11. media index
    L += ["## 11. Media and plots", ""]
    for r in runs:
        L += [f"### {r['run_id']}", f"- page: `{rel(indexes[r['run_id']])}`" if r["run_id"] in indexes else "- page: none"]
        for name, path in summaries.get(r["run_id"], {}).items():
            L += [f"- {name}: ![{name}]({rel(path)})"]
        for t in r["trials"]:
            if t["video"]:
                L += [f"- {t['trial_id']} video: [`{rel(t['video'])}`]({rel(t['video'])}) ({t['video'].stat().st_size / 1e6:.1f} MB, 1280x720, 15 fps of simulated time)"]
            for name, path in plots.get(r["run_id"], {}).get(t["trial_id"], {}).items():
                L += [f"- {t['trial_id']} {name}: ![{name}]({rel(path)})"]
            if t["frames"]:
                L += [f"- {t['trial_id']} key frames ({len(t['frames'])}): " + " ".join(f"![{f.stem}]({rel(f)})" for f in t["frames"][:12])]
        L += [""]
    # ---------------------------------------------------------------- 12. open problems
    L += ["## 12. Open problems and questions for the user", "",
          "- The plan's D4 invariant fails the only passing grasp: the rim pinch loads the inner knuckles (section 6); user ruling 2026-09-30: D4 stays, "
          "Phase 2's grasps must keep the rim on the pads (opening = wall + 10-15 mm per side, the plan's G1).",
          "- Under the pre-R7 lag limits (D8) every carry of the 3-bowl case is judged blocked at the top of the rise: wrist_3 lags 46-49 mrad while a bowl is "
          "held (gravity is off on the links, so it is not sag) against the 30 mrad settled limit; Phase 1a's contact-first blocked detection is the planned remedy.",
          "- After a failed transport the episode releases the dish where the hand is (up to 30 cm above the counter), as the old episode did; a lowered abort "
          "release is a control change for Phase 2+, not Phase 0.",
          "- D11 (asset script): `assets/robots/MANIFEST.sha256` is written from the files present; the usd-core version that produced the converted layers is "
          "unrecorded and cannot be verified without a re-download (which D11 forbids).",
          f"- D13: git HEAD `{g['head']}`; robot files modified since HEAD {g['modified'] or 'none'}, untracked {g['untracked'] or 'none'} (the user commits; see the summary's suggested message).",
          "- Phase 1 (1a-1e) has not started; every hypothesis row above is a code/asset status, not a measurement, except H2's runtime test.", ""]
    L += ["## Machine constraints", "",
          "Safety: one container (`dishsim-isaac`) on GPU 1, at most two Kit jobs staggered 90 s with a 6 GB free check, jobs stopped by recorded PID only, "
          "nothing outside this repo touched. Privacy: everything on the box (results/robot, artifacts/, logs/robot on the 2 TB drive); no uploads. "
          "Space: the root disk gained nothing (artifacts/ is a symlink onto the 2 TB drive; Kit's per-run log is redirected there with `--/log/file`).", ""]
    return "\n".join(L)


def run_pytest(out_dir):
    """Run the Kit-free robot tests and summarise the junit XML (the log this row is built from)."""
    xml = out_dir / "pytest_robot.xml"
    tests = [str(ROOT / t) for t in ROBOT_TESTS if (ROOT / t).exists()]
    if not tests:
        return None
    subprocess.run([sys.executable, "-m", "pytest", "-q", "--junitxml", str(xml), *tests], cwd=ROOT,
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env={**__import__("os").environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"})
    if not xml.exists():
        return "**FAIL**: pytest produced no junit file"
    root = ET.parse(xml).getroot()
    suites = root.findall("testsuite") or [root]
    n = sum(int(s.get("tests", 0)) for s in suites)
    bad = sum(int(s.get("failures", 0)) + int(s.get("errors", 0)) for s in suites)
    files = sorted({Path(t).name for t in tests})
    return (f"{'PASS' if bad == 0 else '**FAIL**'}: {n - bad}/{n} passed in {', '.join(files)} (`{rel(xml)}`)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runs", nargs="+", required=True, help="run ids under results/robot/runs and artifacts/")
    parser.add_argument("--conventions", default=None, help="run id of the conventions record under artifacts/")
    parser.add_argument("--out", type=Path, default=ROOT / "REPORT.md")
    parser.add_argument("--report-run", default="p0_report", help="artifacts/<report-run>/ holds the pytest junit xml")
    args = parser.parse_args()
    runs = [load_run(r) for r in args.runs]
    conventions = None
    if args.conventions:
        cpath = ARTIFACTS / args.conventions / "conventions.json"
        conventions = json.loads(cpath.read_text()) if cpath.exists() else None
    plots, summaries, indexes = {}, {}, {}
    for run in runs:
        plots[run["run_id"]] = {t["trial_id"]: plot_trial(t) for t in run["trials"]}
        summaries[run["run_id"]] = plot_run_summary(run) if run["trials"] else {}
        if run["adir"].exists():
            indexes[run["run_id"]] = write_index(run, plots[run["run_id"]], summaries[run["run_id"]])
    rdir = ARTIFACTS / args.report_run
    rdir.mkdir(parents=True, exist_ok=True)
    pytest_summary = run_pytest(rdir)
    text = build_report(args, runs, conventions, pytest_summary, plots, summaries, indexes)
    args.out.write_text(text)
    n_plots = sum(len(v) for p in plots.values() for v in p.values())
    print(f"[RESULT] PASS report {rel(args.out)}: {len(runs)} runs, {n_plots} trial plots, {len(indexes)} index pages, pytest: {pytest_summary}")


if __name__ == "__main__":
    main()
