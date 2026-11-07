"""report stage (Kit-free): the bench page over every sample of a results folder (plans/2026-11-01-merged-benchmark.md):
the samples its index.jsonl lists, each with its runs/ record (plans/2026-11-02-sample-schema.md); tables, stills and
videos, no paragraphs. Writes <media root>/index.html; media paths are relative to it.

Rosters: labels (kinds, count, allowance), dishes, n on the counter, cap, S_ref, how far the sample got. Episodes: per
roster and algorithm solved, raw moves, S / S_ref, the failure. Then per sample with stills or a run its initial, goal
and finished stills and the episode videos.
"""
import html
import json
import os
from pathlib import Path

from . import ROOT
from .sample import ledger, read_run

ORDER = ("greedy_offline", "rrt_connect", "first_fit", "mcts")


def samples(out):
    """(path, sample, runs record or None) of every sample <out>/index.jsonl lists, in its order (roster, then seed)."""
    index = Path(out) / "index.jsonl"
    found = []
    for line in index.read_text().splitlines() if index.is_file() else []:
        if not line.strip():
            continue
        p = Path(out) / json.loads(line)["file"]
        try:
            found.append((p, json.loads(p.read_text()), read_run(p)))
        except (OSError, ValueError):
            continue
    return found


def status(d, run=None):
    """'through <stage>' for a sample whose last ledger entry (its own, then its runs file's) passed, else
    '<stage> FAIL: <note>'."""
    stages = ledger(d, run)
    if not stages:
        return "no stage ran"
    last = stages[-1]
    return f"through {last['stage']}" if last["result"] != "FAIL" else f"{last['stage']} FAIL: {last.get('note') or ''}"


def f3(v):
    return "-" if v is None else f"{v:.3f}"


def cell(v):
    return html.escape("-" if v is None else str(v))


def table(head, rows):
    out = ["<table><thead><tr>" + "".join(f"<th>{html.escape(h)}</th>" for h in head) + "</tr></thead><tbody>"]
    out += ["<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows]
    return "\n".join(out + ["</tbody></table>"])


def media_rel(path, page_dir):
    p = Path(path) if Path(path).is_absolute() else ROOT / path
    return os.path.relpath(p, page_dir) if p.is_file() else None


def write_page(out, media):
    """Write <media>/index.html over the samples of ``out``; returns its path."""
    media = Path(media)
    media.mkdir(parents=True, exist_ok=True)
    found = samples(out)
    roster_rows, episode_rows, sections = [], [], []
    for p, d, run in found:
        meta = d["metadata"]
        lv = meta["params"].get("levels") or {}
        init = d.get("initial") or {}
        counter = init.get("counter") or {}
        score = (d.get("goal") or {}).get("score") or {}
        dishes = sum(meta["params"]["count"].values())
        roster_rows.append([cell(p.stem), cell(lv.get("kinds")), cell(lv.get("count")), cell(lv.get("allowance")),
                            cell(dishes), cell(counter.get("start_count")), cell(counter.get("cap")), f3(score.get("S_ref")),
                            cell(status(d, run))])
        run, stills_of = run or {}, meta.get("media") or {}
        for r in sorted(run.get("episodes") or [], key=lambda r: ORDER.index(r["algorithm"]) if r["algorithm"] in ORDER else 9):
            episode_rows.append([cell(p.stem), cell(r["algorithm"]), cell("A" if r["track"] == "goal" else "B"),
                                 cell(None if r.get("record") is None else "yes" if r.get("solved") else "no"),
                                 cell(r.get("moves")), f3(r.get("S_over_S_ref")), cell(r.get("failure")),
                                 cell(r.get("at_goal")), cell(r.get("racked")), cell(r.get("nudges")),
                                 cell(None if r.get("planning_s") is None else round(r["planning_s"], 1))])
        if not (run.get("episodes") or stills_of):
            continue
        stills = [("initial", stills_of.get("initial")), ("goal", stills_of.get("goal"))]
        stills += [(f"finished, {r['algorithm']}", r.get("still")) for r in run.get("episodes") or []]
        figs = [f'<figure><img src="{html.escape(src)}" alt="{html.escape(cap)}" loading="lazy"><figcaption>'
                f'{html.escape(cap)}</figcaption></figure>' for cap, path in stills if path
                for src in [media_rel(path, media)] if src]
        vids = [f'<figure><video src="{html.escape(src)}" controls preload="metadata" muted playsinline></video>'
                f'<figcaption>{html.escape(r["algorithm"])}</figcaption></figure>'
                for r in run.get("episodes") or [] if r.get("video") for src in [media_rel(r["video"], media)] if src]
        sections.append(f'<section><h2>{html.escape(p.stem)}</h2><p class="tags">' + " · ".join(
            html.escape(f"{k} {v}") for k, v in lv.items()) + f' · S_ref {f3(score.get("S_ref"))}</p>'
            + f'<div class="grid">{"".join(figs)}</div><div class="grid">{"".join(vids)}</div></section>')
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Merged benchmark {html.escape(Path(out).name)}</title>
<style>
:root {{ --bg: #fbfbfa; --fg: #1d1f21; --muted: #5f6368; --line: #d9dadc; --head: #f0f1f2; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg: #17181a; --fg: #e8e9ea; --muted: #a2a5a9;
  --line: #3a3c40; --head: #232427; }} }}
:root[data-theme="dark"] {{ --bg: #17181a; --fg: #e8e9ea; --muted: #a2a5a9; --line: #3a3c40; --head: #232427; }}
body {{ background: var(--bg); color: var(--fg); font: 14px/1.45 system-ui, sans-serif; margin: 0 auto; max-width: 1280px;
  padding: 16px; }}
h1 {{ font-size: 20px; margin: 4px 0 2px; }} h2 {{ font-size: 16px; margin: 28px 0 4px; }}
.tags {{ color: var(--muted); margin: 0 0 8px; }}
.scroll {{ overflow-x: auto; }}
table {{ border-collapse: collapse; margin: 8px 0 16px; font-variant-numeric: tabular-nums; }}
th, td {{ border: 1px solid var(--line); padding: 4px 8px; text-align: left; white-space: nowrap; }}
th {{ background: var(--head); }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 10px; }}
figure {{ margin: 0; }} img, video {{ width: 100%; height: auto; border: 1px solid var(--line); }}
figcaption {{ color: var(--muted); font-size: 12px; }}
</style></head><body>
<h1>Merged benchmark {html.escape(Path(out).name)}</h1>
<p class="tags">Frigidaire FDPC4221AS twin · HOTEC set · Isaac Sim 6.2.0 + Isaac Lab 3.0.0 · S = fair score, 6.2 ceilings · unracked dish = 0</p>
<h2>Rosters</h2>
<div class="scroll">{table(["sample", "kinds", "count", "allowance", "dishes", "on counter", "cap", "S_ref", "status"], roster_rows)}</div>
<h2>Episodes</h2>
<div class="scroll">{table(["sample", "algorithm", "track", "solved", "moves", "S / S_ref", "failure", "at goal", "racked",
                            "nudges", "planning s"], episode_rows)}</div>
{"".join(sections)}
</body></html>
"""
    path = media / "index.html"
    path.write_text(page)
    return path


def stage_report(state, params):
    path = write_page(params.get("out") or Path(params["sample"]).parent, params.get("media") or ROOT / "data/media/bench/v1")
    n = len(samples(params.get("out") or Path(params["sample"]).parent))
    return state, "PASS", f"page {os.path.relpath(path, ROOT)} over {n} samples"
