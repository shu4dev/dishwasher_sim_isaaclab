#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Results page of the HOTEC benchmark (frigidaire_bench.py): one self-contained folder to publish.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_bench_page.py [--label "pilot, 1 instance per tier"]

Writes data/media/benchmark/frigidaire_hotec/page/: index.html (tables and the per-episode browser, data inline),
sheets/<instance>.jpg (the Isaac stills of one instance as a 4 x 2 sprite: initial, goal, the four finished
episodes), analysis/<instance>.jpg (the four score-detail figures stacked), video/*.mp4 (copied) and files.json
(the relative paths to publish). Sprites keep the file count inside the artifact limits (255 files, 64 MB).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PANEL = (640, 480)                              # one Isaac still in the sprite (4:3, the render aspect)
ANALYSIS_W = 1200                               # score-detail figure width in its sprite
EPISODES = (("goal", "greedy_offline"), ("goal", "rrt_connect"), ("open", "baseline"), ("open", "mcts"))
PANELS = ["initial", "goal"] + [f"finished__{t}__{a}" for t, a in EPISODES]
COLS, ROWS = 4, 2


def bench():
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sheet(paths, out):
    """Stills -> one 4 x 2 JPEG sprite; a missing still leaves its cell dark."""
    from PIL import Image
    canvas = Image.new("RGB", (PANEL[0] * COLS, PANEL[1] * ROWS), (30, 34, 38))
    for i, path in enumerate(paths):
        if path is not None and path.is_file():
            canvas.paste(Image.open(path).convert("RGB").resize(PANEL, Image.LANCZOS), ((i % COLS) * PANEL[0], (i // COLS) * PANEL[1]))
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out, quality=84, optimize=True)


def stack(paths, out):
    """Score-detail figures -> one vertical JPEG sprite at ANALYSIS_W; returns the cell height."""
    from PIL import Image
    images = [Image.open(p).convert("RGB") for p in paths if p is not None and p.is_file()]
    if not images:
        return None
    h = round(ANALYSIS_W * images[0].height / images[0].width)
    canvas = Image.new("RGB", (ANALYSIS_W, h * len(paths)), (255, 255, 255))
    for i, p in enumerate(paths):
        if p is not None and p.is_file():
            canvas.paste(Image.open(p).convert("RGB").resize((ANALYSIS_W, h), Image.LANCZOS), (0, i * h))
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out, quality=82, optimize=True)
    return h


def build(label, out=None, media=None):
    B = bench()
    out, media = Path(out or B.OUT), Path(media or B.MEDIA)
    B.collect(out)
    summary = json.loads((out / "compare" / "summary.json").read_text())
    rows = B.episode_rows(out)
    page = media / "page"
    page.mkdir(parents=True, exist_ok=True)
    files, instances, episodes = ["index.html"], {}, []
    for inst_path in sorted((out / "instances").glob("*/*_s*.json")):
        inst = json.loads(inst_path.read_text())
        iid, tier = inst["instance_id"], inst["tier"]
        stills = media / "stills" / tier / iid
        sheet([stills / f"{name}.png" for name in PANELS], page / "sheets" / f"{iid}.jpg")
        figures = [media / "analysis" / tier / f"{iid}__{t}__{a}.png" for t, a in EPISODES]
        cell = stack(figures, page / "analysis" / f"{iid}.jpg")
        files.append(f"sheets/{iid}.jpg")
        if cell:
            files.append(f"analysis/{iid}.jpg")
        videos = {}
        for t, a in EPISODES:
            src = media / "video" / tier / f"{iid}__{t}__{a}.mp4"
            if src.is_file():
                dst = page / "video" / src.name
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(src, dst)
                videos[f"{t}__{a}"] = f"video/{src.name}"
                files.append(videos[f"{t}__{a}"])
        instances[iid] = {"tier": tier, "seed": inst["seed"], "n": inst["n_objects"], "counter_start": inst["counter"]["start_count"],
                          "cap": inst["counter"]["cap"], "S_ref": inst["goal"]["S_ref"], "S_planned": inst["goal"]["S_planned"],
                          "lower_bound": inst["goal"]["lower_bound"], "kept": len(inst["goal"]["keep"]),
                          "certificate": inst.get("certificate", {}).get("moves"), "stacked": len({b for _, b in inst.get("support", [])}),
                          "order_constraints": len(inst["goal"].get("order", [])),
                          "analysis": bool(cell), "videos": videos}
    for track in ("goal", "open"):
        for r in rows[track]:
            src_track, src_tier, stem = r["source_episode"].split("/")
            src_algo = stem.split("__", 1)[1]
            episodes.append({"track": track, "tier": r["tier"], "instance": r["instance"], "algorithm": r["algorithm"],
                             "shared_from": r.get("shared_from"), "success": bool(r["success"]), "abort": r.get("abort"),
                             "end": r.get("end_check", {}).get("outcome"), "moves": r.get("moves_used"), "gap": r.get("gap"),
                             "lower_bound": r.get("lower_bound"), "failed_settles": r.get("failed_settles", 0),
                             "refused": r.get("infeasible_commands", 0), "counter_full": r.get("counter_full_refusals", 0),
                             "nudges": r.get("nudges", 0),
                             "planning_wall_s": r.get("planning_time_total_s"), "planning_cpu_s": r.get("planning_cpu_s"),
                             "S_final": r.get("S_final"), "S_ref": r.get("S_ref"),
                             "panel": PANELS.index(f"finished__{src_track}__{src_algo}"),
                             "figure": [f"{t}__{a}" for t, a in EPISODES].index(f"{src_track}__{src_algo}"),
                             "source": f"{src_track}__{src_algo}"})
    method = None
    if (media / "mcts_method.png").is_file():               # the track-B MCTS method figure (frigidaire_mcts_figure.py)
        (page / "method").mkdir(parents=True, exist_ok=True)
        shutil.copyfile(media / "mcts_method.png", page / "method" / "mcts_method.png")
        method = "method/mcts_method.png"
        files.append(method)
    data = {"label": label, "method": method, "generated_utc": summary["generated_utc"], "tables": summary["tables"], "instances": instances,
            "episodes": episodes, "tiers": {t: {"inventory": v["inventory"], "slack": v["slack"]} for t, v in B.TIERS.items()},
            "sprite": {"cols": COLS, "rows": ROWS, "panels": PANELS}, "analysis_aspect": [ANALYSIS_W, cell or 706],
            "figures": len(EPISODES),
            "at_goal": B.AT_GOAL}
    (page / "index.html").write_text(TEMPLATE.replace("__DATA__", json.dumps(data).replace("</", "<\\/")))   # raw-text <script>
    (page / "files.json").write_text(json.dumps(files, indent=1) + "\n")
    size = sum((page / f).stat().st_size for f in files)
    print(f"[RESULT] PASS page: {len(files)} files, {size / 1e6:.1f} MB, {len(episodes)} table rows, {len(instances)} instances", flush=True)


TEMPLATE = r"""<title>HOTEC Load Benchmark</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Archivo:wdth,wght@87.5,600;87.5,700&family=IBM+Plex+Mono:wght@400;500&family=Public+Sans:wght@400;600&display=swap">
<style>
:root {
  --ground: #eef1f3; --surface: #ffffff; --sunken: #e3e8ec; --ink: #15202a; --muted: #56636e; --line: #cdd5db;
  --accent: #1c6aa3; --accent-soft: #d8e7f3; --ok: #2b7a4b; --ok-soft: #dcefe3; --bad: #a93d36; --bad-soft: #f5dfdc;
  --neutral-soft: #e6eaee; --shadow: 0 1px 2px rgba(21, 32, 42, .08), 0 6px 24px rgba(21, 32, 42, .06);
  --display: "Archivo", "Arial Narrow", "Helvetica Neue", Arial, sans-serif;
  --body: "Public Sans", "Helvetica Neue", Arial, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --ground: #0e1317; --surface: #161d23; --sunken: #1d262e; --ink: #e2e8ed; --muted: #93a0aa; --line: #2a353f;
    --accent: #69ade3; --accent-soft: #1b3348; --ok: #5cb883; --ok-soft: #17321f; --bad: #e27b72; --bad-soft: #3a1e1b;
    --neutral-soft: #222c35; --shadow: 0 1px 2px rgba(0, 0, 0, .4), 0 6px 24px rgba(0, 0, 0, .3);
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --ground: #0e1317; --surface: #161d23; --sunken: #1d262e; --ink: #e2e8ed; --muted: #93a0aa; --line: #2a353f;
  --accent: #69ade3; --accent-soft: #1b3348; --ok: #5cb883; --ok-soft: #17321f; --bad: #e27b72; --bad-soft: #3a1e1b;
  --neutral-soft: #222c35; --shadow: 0 1px 2px rgba(0, 0, 0, .4), 0 6px 24px rgba(0, 0, 0, .3);
}
body { background: var(--ground); color: var(--ink); font: 15px/1.55 var(--body); }
.wrap { max-width: 1180px; margin: 0 auto; padding-inline: 20px; }
header.wrap { padding-block: 40px 8px; }
.eyebrow { font: 500 12px/1.4 var(--mono); letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }
h1, h2, h3 { font-family: var(--display); font-stretch: 87.5%; text-wrap: balance; margin: 0; }
h1 { font-size: clamp(30px, 5vw, 46px); font-weight: 700; line-height: 1.05; margin-top: 10px; }
h2 { font-size: 24px; font-weight: 700; }
h3 { font-size: 17px; font-weight: 600; }
.lede { max-width: 68ch; color: var(--muted); margin: 14px 0 0; }
.label-chip { display: inline-block; margin-left: 10px; vertical-align: middle; font: 500 12px/1 var(--mono); padding: 6px 9px;
  border-radius: 999px; background: var(--accent-soft); color: var(--accent); }
section.wrap { padding-block: 26px; }
.facts { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 12px; margin-top: 22px; }
.fact { background: var(--surface); border: 1px solid var(--line); border-radius: 10px; padding: 12px 14px; }
.fact dt { font: 500 11px/1.3 var(--mono); letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }
.fact dd { margin: 4px 0 0; }
.formula { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1.3fr); gap: 22px; align-items: start; }
.eq { background: var(--surface); border: 1px solid var(--line); border-radius: 12px; padding: 18px 20px; box-shadow: var(--shadow); }
.eq .math { font: 500 clamp(19px, 3vw, 24px)/1.4 var(--mono); }
.eq .math sub { font-size: .6em; }
.eq p, .rules p { margin: 8px 0 0; color: var(--muted); }
.rules { display: grid; gap: 10px; }
.rules div { border-left: 3px solid var(--line); padding-left: 12px; }
.rules b { color: var(--ink); font-weight: 600; }
.tables { display: grid; gap: 22px; margin-top: 14px; }
.scroll { overflow-x: auto; border: 1px solid var(--line); border-radius: 10px; background: var(--surface); }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; }
th, td { padding: 9px 12px; text-align: left; border-bottom: 1px solid var(--line); white-space: nowrap; }
th { font: 500 11px/1.3 var(--mono); letter-spacing: .05em; text-transform: uppercase; color: var(--muted); background: var(--sunken); }
td.num { font-family: var(--mono); text-align: right; }
tr:last-child td { border-bottom: 0; }
tr.tier-start td { border-top: 2px solid var(--line); }
.shared { color: var(--muted); font-size: 12px; }
.controls { display: flex; flex-wrap: wrap; gap: 14px 22px; margin-top: 14px; align-items: end; }
.seg { display: flex; flex-direction: column; gap: 6px; }
.seg span { font: 500 11px/1 var(--mono); letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }
.seg div { display: flex; flex-wrap: wrap; gap: 4px; background: var(--sunken); padding: 3px; border-radius: 9px; }
.seg button { font: 600 13px/1 var(--body); border: 0; background: transparent; color: var(--ink); padding: 8px 12px; border-radius: 7px; cursor: pointer; }
.seg button[aria-pressed="true"] { background: var(--surface); color: var(--accent); box-shadow: var(--shadow); }
button:focus-visible, .shot:focus-visible, select:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.seg select { font: 500 13px/1 var(--mono); padding: 8px 10px; border-radius: 7px; border: 1px solid var(--line); background: var(--surface); color: var(--ink); }
.episode { margin-top: 18px; background: var(--surface); border: 1px solid var(--line); border-radius: 14px; padding: 18px; box-shadow: var(--shadow); }
.ep-head { display: flex; flex-wrap: wrap; gap: 10px 14px; align-items: center; }
.pill { font: 600 12px/1 var(--body); padding: 7px 10px; border-radius: 999px; }
.pill.ok { background: var(--ok-soft); color: var(--ok); }
.pill.bad { background: var(--bad-soft); color: var(--bad); }
.pill.neutral { background: var(--neutral-soft); color: var(--muted); font-weight: 500; }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(128px, 1fr)); gap: 1px; background: var(--line);
  border: 1px solid var(--line); border-radius: 10px; overflow: hidden; margin-top: 14px; }
.stat { background: var(--surface); padding: 10px 12px; }
.stat b { display: block; font: 500 18px/1.2 var(--mono); }
.stat span { font-size: 12px; color: var(--muted); }
.shots { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; margin-top: 16px; }
figure { margin: 0; }
figcaption { font-size: 13px; color: var(--muted); margin-top: 6px; }
figcaption b { color: var(--ink); font-weight: 600; }
.shot { display: block; width: 100%; max-width: 100%; aspect-ratio: 4 / 3; border-radius: 8px; border: 1px solid var(--line);
  background-color: var(--sunken); background-repeat: no-repeat; cursor: zoom-in; padding: 0; }
.media { display: grid; grid-template-columns: minmax(0, 1.55fr) minmax(0, 1fr); gap: 14px; margin-top: 16px; align-items: start; }
.analysis { width: 100%; max-width: 100%; border-radius: 8px; border: 1px solid var(--line); background-color: #fff; background-repeat: no-repeat; cursor: zoom-in; padding: 0; }
video { width: 100%; max-width: 100%; border-radius: 8px; border: 1px solid var(--line); background: #000; }
.note { font-size: 13px; color: var(--muted); }
dialog { border: 0; padding: 0; background: transparent; max-width: 96vw; }
dialog::backdrop { background: rgba(8, 12, 16, .82); }
dialog .big { width: min(94vw, 1500px); max-width: 100%; border-radius: 8px; background-repeat: no-repeat; }
dialog button { position: absolute; top: 8px; right: 8px; font: 600 13px/1 var(--body); padding: 8px 12px; border-radius: 7px; border: 0; background: var(--surface); color: var(--ink); cursor: pointer; }
footer.wrap { padding-block: 10px 40px; color: var(--muted); font-size: 13px; }
.paths { margin: 12px 0 0; padding-left: 18px; }
.paths li { margin-bottom: 8px; }
.paths code { font: 13px/1.5 var(--mono); overflow-wrap: anywhere; }
@media (max-width: 820px) {
  .formula, .media { grid-template-columns: minmax(0, 1fr); }
  .shots { grid-template-columns: minmax(0, 1fr); }
}
@media (prefers-reduced-motion: no-preference) { .seg button { transition: background .15s, color .15s; } }
</style>

<header class="wrap">
  <div class="eyebrow">Frigidaire FDPC4221AS twin &middot; HOTEC wheat-straw dinnerware &middot; Isaac Sim 4.5 / Isaac Lab 2.1</div>
  <h1>HOTEC Load Benchmark <span class="label-chip" id="label"></span></h1>
  <p class="lede">Dishes start partly in a stacked pile on the counter and partly dropped into the racks. An algorithm moves
  one dish at a time by teleport, and Isaac settles every move. No goal is given by hand: the target load is the
  highest-exposure arrangement found by sampling with the spray-exposure score.</p>
  <dl class="facts" id="facts"></dl>
</header>

<section class="wrap">
  <h2>How a load is scored</h2>
  <div class="formula" style="margin-top:14px">
    <div class="eq">
      <div class="math">S = &Sigma;<sub>o</sub> A<sub>o</sub>E<sub>o</sub> / &Sigma;<sub>o</sub> A<sub>o</sub></div>
      <p><b>E<sub>o</sub></b>: share of spray-arm rays that reach the food-contact surface of dish <i>o</i>, averaged
      over its samples (revision 5: lower and upper arms, the appliance and every other dish occlude).
      <b>A<sub>o</sub></b>: its food-contact area. A dish that would pool water makes the load infeasible.</p>
    </div>
    <div class="rules">
      <div><b>Track A, reach the goal.</b> Every dish must end within the at-goal tolerance of the sampled goal
      (bowls and cups 15 mm lateral, 20 mm height, 15&deg; tilt; plates 18 mm / 20 mm / 16&deg;). Gap = moves &minus; lower
      bound, where the bound counts the counter dishes plus the rack dishes the goal does not keep in place.</div>
      <div><b>Track B, open.</b> Success means every dish is racked and passes the end check; the algorithm chooses its
      own load, and S is reported against S<sub>ref</sub>, the settled sampled goal.</div>
      <div><b>Every move.</b> Teleport, then 150 + 60 physics ticks at 120 Hz. Moving another dish more than 10 mm or 20&deg;
      is fatal, as is moving a dish that another rests on. A dish that drifts past 8 cm is put back (failed settle). End check:
      tub-wall clearance, both racks retracted, every dish contained.</div>
    </div>
  </div>
</section>

<section class="wrap" id="methodSection" hidden>
  <h2>Track B planner: move-level MCTS</h2>
  <p class="note">Each action moves one dish: onto one of its best free rack poses, kept in place, or parked on the
  counter. A rollout repairs the parent's complete load around the moved dish and scores it. The search starts from
  the best of a pool of complete packings and has the same 60 s budget as every other planner.</p>
  <img id="methodImg" alt="MCTS method: the search loop, the best S found over the 60 s, and S against a longer budget" style="width:100%;border-radius:8px;border:1px solid var(--line);background:#fff">
</section>

<section class="wrap">
  <h2>Results</h2>
  <div class="tables" id="tables"></div>
</section>

<section class="wrap">
  <h2>Episodes</h2>
  <div class="controls" id="controls"></div>
  <article class="episode" id="episode" aria-live="polite"></article>
</section>

<section class="wrap">
  <h2>Files on disk</h2>
  <p class="note">Full-resolution sources of every image and video on this page, relative to the repository root on the
  lab workstation (results, media and logs live on its data drive).</p>
  <ul class="paths" id="paths"></ul>
</section>

<footer class="wrap" id="footer"></footer>

<dialog id="zoom"><div class="big" id="zoomImg" role="img"></div><button type="button" id="zoomClose">Close</button></dialog>

<script type="application/json" id="data">__DATA__</script>
<script>
(function () {
  const D = JSON.parse(document.getElementById("data").textContent);
  const $ = (id) => document.getElementById(id);
  const el = (tag, attrs, ...kids) => {
    const n = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) {
      if (k === "class") n.className = v; else if (k === "style") n.style.cssText = v; else if (k === "text") n.textContent = v;
      else n.setAttribute(k, v);
    }
    for (const k of kids) if (k != null) n.append(k);
    return n;
  };
  const f = (v, d = 1) => (v == null || Number.isNaN(v)) ? "–" : Number(v).toFixed(d);
  const ALGO = { greedy_offline: "Greedy (offline)", rrt_connect: "RRT-Connect", planner: "Exposure planner (retired)", mcts: "MCTS", baseline: "First-fit baseline" };
  const TRACK = { goal: "Track A: goal", open: "Track B: open" };
  const WHY = { disturbed: "disturbed a neighbour", "give-up": "gave up", "refusal-loop": "refusal loop", "settle-loop": "settle loop",
    "init-mismatch": "start did not reproduce", not_all_racked: "not every dish racked", tub_wall: "crossed the tub wall",
    closure_failure: "racks would not close", aborted: "aborted", "time-budget": "planning budget spent" };
  const tierText = (t) => Object.entries(D.tiers[t].inventory).map(([k, n]) => n + " " + k + "s").join(" + ");

  $("label").textContent = D.label;
  const facts = [
    ["Tiers", Object.keys(D.tiers).map((t) => t + ": " + tierText(t) + ", allowance n+" + D.tiers[t].slack).join("; ")],
    ["n", "dishes on the counter at the start, uniform in [N/4, 3N/4] per instance"],
    ["Goal", "coordinate ascent over the HOTEC rack families, until no gain or 2 sweeps; rack dishes may stay"],
    ["Budget", "no move budget; 60 s of planning per episode"],
  ];
  for (const [k, v] of facts) $("facts").append(el("div", { class: "fact" }, el("dt", { text: k }), el("dd", { text: v })));

  // ---- result tables
  const cols = {
    goal: [["success", (r) => r.solved + "/" + r.n], ["moves", (r) => f(r.moves)], ["gap", (r) => f(r.gap)],
           ["planning s (wall / CPU)", (r) => f(r.planning_wall_s) + " / " + f(r.planning_cpu_s)], ["failed settles", (r) => r.failed_settles]],
    open: [["all racked", (r) => r.solved + "/" + r.n], ["moves", (r) => f(r.moves)], ["S", (r) => f(r.S_final, 3)],
           ["S / S_ref", (r) => f(r.S_over_S_ref, 3)], ["planning s (wall / CPU)", (r) => f(r.planning_wall_s) + " / " + f(r.planning_cpu_s)]],
  };
  for (const track of ["goal", "open"]) {
    const t = el("table");
    const head = el("tr", {}, el("th", { text: "tier" }), el("th", { text: "algorithm" }));
    for (const [name] of cols[track]) head.append(el("th", { text: name, style: "text-align:right" }));
    head.append(el("th", { text: "failures" }));
    t.append(el("thead", {}, head));
    const body = el("tbody");
    let last = null;
    for (const r of D.tables[track] || []) {
      const tr = el("tr", { class: r.tier !== last && last !== null ? "tier-start" : "" });
      last = r.tier;
      tr.append(el("td", { text: r.tier }));
      tr.append(el("td", {}, ALGO[r.algorithm] || r.algorithm,
        r.shared_from ? el("span", { class: "shared", text: " · shares the " + (r.shared_from === "planner" ? "planner's run" : "track-A run") }) : null));
      for (const [, get] of cols[track]) tr.append(el("td", { class: "num", text: String(get(r)) }));
      tr.append(el("td", { text: Object.entries(r.aborts).map(([k, n]) => (WHY[k] || k) + " " + n).join(", ") || "–" }));
      body.append(tr);
    }
    t.append(body);
    $("tables").append(el("div", {}, el("h3", { text: TRACK[track], style: "margin-bottom:8px" }), el("div", { class: "scroll" }, t)));
  }

  // ---- episode browser
  const state = { tier: null, track: "goal", algorithm: "rrt_connect", instance: null };
  const tiers = Object.keys(D.tiers).filter((t) => D.episodes.some((e) => e.tier === t));
  state.tier = tiers[0];
  const segment = (label, key, options) => {
    const box = el("div");
    for (const [value, text] of options) {
      const b = el("button", { type: "button", "aria-pressed": String(state[key] === value), text });
      b.addEventListener("click", () => { state[key] = value; render(); });
      box.append(b);
    }
    return el("div", { class: "seg" }, el("span", { text: label }), box);
  };
  const spritePos = (i, cols, rows) => {
    const c = i % cols, r = Math.floor(i / cols);
    return (cols > 1 ? c / (cols - 1) * 100 : 0) + "% " + (rows > 1 ? r / (rows - 1) * 100 : 0) + "%";
  };
  const panel = (iid, index, caption) => {
    const b = el("button", { type: "button", class: "shot", "aria-label": caption + " (enlarge)" });
    b.style.backgroundImage = "url('sheets/" + iid + ".jpg')";
    b.style.backgroundSize = (D.sprite.cols * 100) + "% " + (D.sprite.rows * 100) + "%";
    b.style.backgroundPosition = spritePos(index, D.sprite.cols, D.sprite.rows);
    b.addEventListener("click", () => zoom(b.style.backgroundImage, b.style.backgroundSize, b.style.backgroundPosition, "4 / 3", caption));
    return b;
  };
  const zoom = (image, size, pos, ratio, caption) => {
    const z = $("zoomImg");
    z.style.backgroundImage = image; z.style.backgroundSize = size; z.style.backgroundPosition = pos; z.style.aspectRatio = ratio;
    z.setAttribute("aria-label", caption);
    $("zoom").showModal();
  };
  $("zoomClose").addEventListener("click", () => $("zoom").close());
  $("zoom").addEventListener("click", (e) => { if (e.target === $("zoom")) $("zoom").close(); });

  function render() {
    const inTier = D.episodes.filter((e) => e.tier === state.tier);
    const ids = [...new Set(inTier.map((e) => e.instance))].sort((a, b) => D.instances[a].seed - D.instances[b].seed);
    if (!ids.includes(state.instance)) state.instance = ids[0];
    const c = $("controls");
    c.replaceChildren(
      segment("Tier", "tier", tiers.map((t) => [t, t])),
      segment("Track", "track", [["goal", "A: goal"], ["open", "B: open"]]),
      segment("Algorithm", "algorithm", Object.entries(ALGO)));
    const pick = el("select", { id: "instancePick", "aria-label": "Instance" });
    for (const id of ids) {
      const o = el("option", { value: id, text: id + " · n " + D.instances[id].counter_start + " on the counter" });
      if (id === state.instance) o.selected = true;
      pick.append(o);
    }
    pick.addEventListener("change", () => { state.instance = pick.value; render(); });
    c.append(el("div", { class: "seg" }, el("span", { text: "Instance" }), pick));

    const ep = D.episodes.find((e) => e.tier === state.tier && e.track === state.track && e.algorithm === state.algorithm && e.instance === state.instance);
    const box = $("episode");
    box.replaceChildren();
    if (!ep) { box.append(el("p", { class: "note", text: "No record for this combination yet." })); return; }
    const inst = D.instances[ep.instance];
    const okText = ep.track === "goal" ? "Solved" : "All racked";
    const why = ep.abort || ep.end;
    box.append(el("div", { class: "ep-head" },
      el("h3", { text: ep.instance + " · " + TRACK[ep.track] + " · " + (ALGO[ep.algorithm] || ep.algorithm) }),
      el("span", { class: "pill " + (ep.success ? "ok" : "bad"), text: ep.success ? okText : "Failed: " + (WHY[why] || why || "unknown") }),
      ep.shared_from ? el("span", { class: "pill neutral", text: ep.shared_from === "planner" ? "same run as the planner" : "the track-A run, scored as open" }) : null));
    const stats = [
      [ep.moves ?? "–", "moves executed"], [ep.lower_bound ?? "–", "lower bound"], [ep.gap == null ? "–" : ep.gap, "gap"],
      [f(ep.S_final, 3), "S of the racked dishes"], [f(ep.S_ref, 3), "S_ref (sampled goal)"],
      [f(ep.planning_wall_s) + " / " + f(ep.planning_cpu_s), "planning s, wall / CPU"],
      [ep.failed_settles + " / " + ep.refused, "failed settles / FCL refusals"],
      [ep.nudges ?? 0, "start dishes nudged (non-fatal)"],
      [inst.counter_start + " / " + inst.cap, "on the counter / allowance"],
    ];
    const s = el("div", { class: "stats" });
    for (const [v, k] of stats) s.append(el("div", { class: "stat" }, el("b", { text: String(v) }), el("span", { text: k })));
    box.append(s);
    const shots = el("div", { class: "shots" });
    shots.append(el("figure", {}, panel(ep.instance, 0, "Initial configuration"),
      el("figcaption", {}, el("b", { text: "Initial. " }), inst.counter_start + " dishes piled on the counter (" + inst.stacked + " resting on another), " + (inst.n - inst.counter_start) + " dropped in the racks")));
    shots.append(el("figure", {}, panel(ep.instance, ep.panel, "Finished configuration"),
      el("figcaption", {}, el("b", { text: "Finished. " }), (ep.moves ?? 0) + " moves" + (why ? ", stopped: " + (WHY[why] || why) : ""))));
    shots.append(el("figure", {}, panel(ep.instance, 1, "Sampled goal configuration"),
      el("figcaption", {}, el("b", { text: "Goal. " }), "S_ref " + f(inst.S_ref, 3) + " as built (" + f(inst.S_planned, 3) + " proposed); " + inst.kept + " kept in place, " + inst.order_constraints + " order constraints")));
    box.append(shots);
    const media = el("div", { class: "media" });
    if (inst.analysis) {
      const [w, h] = D.analysis_aspect;
      const a = el("button", { type: "button", class: "analysis", "aria-label": "Score detail (enlarge)" });
      a.style.aspectRatio = w + " / " + h;
      a.style.backgroundImage = "url('analysis/" + ep.instance + ".jpg')";
      a.style.backgroundSize = "100% " + (100 * D.figures) + "%";
      a.style.backgroundPosition = spritePos(ep.figure, 1, D.figures);
      a.addEventListener("click", () => zoom(a.style.backgroundImage, a.style.backgroundSize, a.style.backgroundPosition, w + " / " + h, "Score detail"));
      media.append(el("figure", {}, a, el("figcaption", {}, el("b", { text: "Score detail. " }),
        "Exposure of every food-contact sample in the finished load, per-dish exposure, and S after each move (a low-resolution re-score)")));
    }
    const video = inst.videos[ep.source];
    if (video) {
      media.append(el("figure", {}, el("video", { controls: "", preload: "metadata", src: video, muted: "" }),
        el("figcaption", {}, el("b", { text: "Start to finish. " }), "Isaac RTX stop-motion; the caption shows S and the racked count after each move")));
    } else {
      media.append(el("p", { class: "note", text: "Videos are recorded for the first instance of each tier." }));
    }
    box.append(media);
  }
  render();
  if (D.method) { $("methodImg").src = D.method; $("methodSection").hidden = false; }
  const M = "data/media/benchmark/frigidaire_hotec/", R = "data/results/benchmark/frigidaire_hotec/";
  for (const [what, path] of [
    ["Isaac stills", M + "stills/<tier>/<instance>/{initial,goal,finished__<track>__<algorithm>}.png"],
    ["Score-detail figures", M + "analysis/<tier>/<instance>__<track>__<algorithm>.png"],
    ["Result tables", R + "compare/summary.{md,json}"],
    ["Episode records", R + "episodes/{goal,open}/<tier>/<instance>__<algorithm>.json (+ .analysis.json)"],
    ["Instances", R + "instances/<tier>/<instance>.json"],
    ["MCTS method figure", M + "mcts_method.png"],
    ["MCTS budget sweep", R + "mcts_budget_sweep/<instance>__<budget>s.json"],
    ["Logs", "data/logs/benchmark_frigidaire_hotec/<unit>.log"]])
    $("paths").append(el("li", {}, el("b", { text: what + ": " }), el("code", { text: path })));
  const vids = [];
  for (const inst of Object.values(D.instances))
    for (const v of Object.values(inst.videos || {})) vids.push(M + "video/" + inst.tier + "/" + v.replace(/^video\//, ""));
  if (vids.length)
    $("paths").append(el("li", {}, el("b", { text: "Videos: " }), ...vids.flatMap((v, i) => [i ? el("br") : null, el("code", { text: v })])));
  $("footer").textContent ="Generated " + D.generated_utc.replace("T", " ").slice(0, 16) + " UTC from the benchmark records " +
    "(data/results/benchmark/frigidaire_hotec on the lab workstation). Images: Isaac RTX renders; score detail: Kit-free Warp re-score.";
})();
</script>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--label", default="pilot: 1 instance per tier")
    ap.add_argument("--out", type=Path, help="benchmark records (default data/results/benchmark/frigidaire_hotec)")
    ap.add_argument("--media", type=Path, help="benchmark media (default data/media/benchmark/frigidaire_hotec)")
    args = ap.parse_args()
    build(args.label, args.out, args.media)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
