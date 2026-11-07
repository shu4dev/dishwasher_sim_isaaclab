"""The sample pipeline (plans/2026-11-02-sample-schema.md): one sample file per (roster, seed) with four blocks,
metadata, initial, goal and dag (schema/sample.json); the algorithms' results beside it in <out>/runs/<sample>.json
(schema/run.json); <out>/index.jsonl lists the folder's samples.

Stages, each stage(state, params) -> (state, result, note) over the sample, which is rewritten after every stage with a
ledger entry (stage, params, code SHA, UTC time, PASS | FAIL | SKIP) in metadata.stages; the entries of run, video and
report go to the runs file:
    load -> initial -> gate_initial -> fit -> build -> gate_goal -> certify -> score -> graph -> stills -> run -> video -> report
Kit stages run the existing Kit scripts through code/util/run_kit.sh; build and score cast rays on the GPU, so the whole
loop runs inside the container. The CLI is code/sampler/scripts/bench.py:
    code/util/run_py.sh code/sampler/scripts/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report
Rerun stages on stored samples (poses are stored, never re-derived from the seed):
    code/util/run_py.sh code/sampler/scripts/bench.py --from score --to report --input <sample.json>
A --variant run writes a new sample file tagged with the variant (with a copy of its runs file); the input sample is never
overwritten. Schema-1 pair files convert in place with python -m sampler.convert <folder>.
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_VERSION = 2
STAGES = ("load", "initial", "gate_initial", "fit", "build", "gate_goal", "certify", "score", "graph", "stills", "run", "video",
          "report")
RUN_STAGES = ("run", "video", "report")      # their ledger entries go to runs/<sample>.json
STAGE_FN = {"load": ("sampler.loader", "stage_load"), "initial": ("sampler.initial", "stage_initial"),
            "gate_initial": ("sampler.gate", "stage_gate_initial"), "fit": ("sampler.build", "stage_fit"),
            "build": ("sampler.build", "stage_build"), "gate_goal": ("sampler.gate", "stage_gate_goal"),
            "certify": ("sampler.goal", "stage_certify"), "score": ("sampler.scorer.score", "stage_score"),
            "graph": ("sampler.graph", "stage_graph"), "stills": ("sampler.stills", "stage_stills"),
            "run": ("paths.episodes", "stage_run"), "video": ("paths.video", "stage_video"),
            "report": ("paths.report", "stage_report")}
DEFAULT_COUNT = "plate=8,bowl=8,cup=8"
ALGORITHMS = ("greedy_offline", "rrt_connect", "first_fit", "mcts")   # track A, then track B
MAKER_KEYS = ("method", "level", "S_first_fit", "S_ascent", "sweeps")  # the maker summary; the rest stays in _work


def kv(text):
    return {k: v for k, v in (item.split("=", 1) for item in text.split(",") if item)} if text else {}


def rel_root(path):
    """``path`` repo-relative (absolute from the cwd first) when under the repo, else absolute."""
    path = Path(path).absolute()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def code_sha():
    import subprocess
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip() or None
    except OSError:
        return None


def utc():
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(value, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    os.replace(tmp, path)


def write(path, state):
    """The sample, checked against schema/sample.json, then the folder's index.jsonl."""
    from sampler import loader
    loader.validate(state, "sample")
    dump(path, state)
    write_index(Path(path).parent)


def run_path(path):
    """<out>/runs/<sample>.json of the sample file ``path``."""
    return Path(path).parent / "runs" / Path(path).name


def read_run(path):
    """The runs file of the sample file ``path``, or None."""
    p = run_path(path)
    return json.loads(p.read_text()) if p.is_file() else None


def write_run(path, run):
    """The runs file of the sample file ``path``, checked against schema/run.json."""
    from sampler import loader
    loader.validate(run, "run")
    dump(run_path(path), run)


def ledger(sample, run=None):
    """The sample's ledger, then its runs file's."""
    return [*sample["metadata"]["stages"], *((run or {}).get("stages") or [])]


def write_index(out):
    """<out>/index.jsonl, rewritten whole: one line per sample file of the folder (file, sample_id, params, levels,
    S_ref, the dag's edge count, the last ledger entry's stage and result)."""
    lines = []
    for p in sorted(Path(out).glob("*.json")):
        try:
            d = json.loads(p.read_text())
        except (OSError, ValueError):
            continue
        if not (isinstance(d, dict) and "metadata" in d):
            continue
        m, last = d["metadata"], (ledger(d, read_run(p)) or [None])[-1]
        lines.append(json.dumps({"file": p.name, "sample_id": m["sample_id"], "params": m["params"],
                                 "levels": m["params"].get("levels"),
                                 "S_ref": ((d.get("goal") or {}).get("score") or {}).get("S_ref"),
                                 "dag_edges": None if d.get("dag") is None else len(d["dag"]["edges"]),
                                 "last_stage": last and {"stage": last["stage"], "result": last["result"]}}))
    tmp = Path(out) / "index.jsonl.tmp"
    tmp.write_text("".join(line + "\n" for line in lines))
    os.replace(tmp, Path(out) / "index.jsonl")


def initial_of(start):
    """The initial block of a start record (frigidaire_bench_kit.py --start): every dish's settled world pose, the rack
    joints and the counter (slab size, centre and top, start count, cap, stacks)."""
    snap, c = start["initial_snapshot"], start["counter"]
    return {"poses": {o["object_id"]: snap["poses"][o["object_id"]] for o in start["objects"]}, "rack_joints": snap["joints"],
            "counter": {"size_m": c["size_m"], "center_m": c["center_m"], "top_z_m": c["top_z_m"],
                        "start_count": c["start_count"], "cap": c["cap"], "stacks": c["pile"]["stacks"]}}


def goal_of(dep, start):
    """The goal block of a build record (frigidaire_dependency_kit.py --load --order): every dish's built world pose
    (targets_world), the joints of the empty-machine baseline the build ran in (the start record's copy; the build record
    stores rack frames, no joints), per dish its rack, slot, variant and commanded rack-local pose; score null until the
    score stage."""
    return {"poses": dep["targets_world"], "rack_joints": start["baseline"]["joints"],
            "slots": {e["id"]: {"rack": e["rack"], "slot": e.get("slot"), "variant": e.get("variant"),
                                "rack_local_pose": {"position_m": list(e["position"]),
                                                    "quaternion_xyzw": list(e["quaternion_xyzw"])}}
                      for e in dep.get("entries_chosen") or dep.get("entries") or []},
            "score": None}


def dag_of(graph, order):
    """The dag block of a certified graph (graph.physical_graph): nodes, edges {a, b, kind}, layers, the build order."""
    return {"nodes": list(graph["layers"]), "edges": [{k: e[k] for k in ("a", "b", "kind")} for e in graph["edges"]],
            "layers": graph["layers"], "order": list(order)}


def new_sample(args, seed):
    """A fresh sample of the roster given by the explicit flags (the levels are labels, difficulty.resolve); run_stages
    names it after its file."""
    from sampler import difficulty
    count = {k: int(v) for k, v in kv(args.count or DEFAULT_COUNT).items()}
    params = difficulty.resolve(args.kinds.split(",") if args.kinds else None, count, args.on_counter,
                                args.counter_allowance)
    return {"metadata": {"schema_version": SCHEMA_VERSION, "sample_id": None, "machine": args.machine,
                         "dishes": args.dishes, "seed": seed, "variant": None, "params": params, "stages": [],
                         "maker": None, "provenance": {}, "media": None},
            "initial": None, "goal": None, "dag": None}


def copy_start(state, src_path):
    """--start-from (plans/2026-11-05-hardware-aware-sampler.md): ``state`` with the metadata (the roster, seed, start
    provenance, provenance.start_from) and the initial block of the sample ``src_path``; its load, initial and gate_initial
    ledger entries copied (marked "copied"). The goal side starts empty."""
    from sampler import rel
    src = json.loads(Path(src_path).read_text())
    sm, m, r = src["metadata"], state["metadata"], rel(src_path)
    for k in ("machine", "dishes", "seed", "params"):
        m[k] = sm[k]
    m["provenance"] = {k: v for k, v in sm["provenance"].items() if k in ("load", "initial_record", "settle", "gate_initial")}
    m["provenance"]["start_from"] = r
    first = {}
    for e in sm["stages"]:
        if e["stage"] in ("load", "initial", "gate_initial"):
            first.setdefault(e["stage"], dict(e, copied=r, note=f"copied from {r}: {e.get('note')}"))
    m["stages"] = [first[s] for s in ("load", "initial", "gate_initial") if s in first]
    m["maker"], m["media"] = None, None
    state["initial"] = src["initial"]
    return state


def sample_path(out, state, tag=""):
    """<out>/<machine>__<dishes>__<roster>__s<seed>[__<variant>].json: three rosters share one --out folder."""
    from sampler import difficulty
    m = state["metadata"]
    return Path(out) / (f"{m['machine']}__{m['dishes']}__{difficulty.tag(m['params'])}__s{m['seed']}"
                        + (f"__{tag}" if tag else "") + ".json")


def run_stages(state, path, stages, variants, max_attempts=6, extra=None):
    """Run ``stages`` in order on ``state``, writing ``path`` (and for run, video and report the runs file) after each;
    stops at the first FAIL. Returns True when none failed. ``extra`` joins the stage params (algorithms, the media
    folder, the results folder)."""
    import importlib
    meta = state["metadata"]
    meta["sample_id"] = path.stem
    if "initial.on_counter" in variants:
        meta["params"]["on_counter"] = int(variants["initial.on_counter"])
    params = {**variants, "work": str(path.parent / "_work" / path.stem), "sample": str(path),
              "max_attempts": max_attempts, **(extra or {})}
    for stage in stages:
        module, name = STAGE_FN[stage]
        fn = getattr(importlib.import_module(module), name)
        try:
            state, result, note = fn(state, params)
        except (Exception, SystemExit) as exc:                   # noqa: BLE001  the ledger records it; a SystemExit (e.g.
            result, note = "FAIL", f"exception {exc!r}"          # stale packer masks) fails this seed, not the whole run
        entry = {"stage": stage, "params": {k: v for k, v in params.items() if k not in ("work", "sample")},
                 "code_sha": code_sha(), "utc": utc(), "result": result, "note": note}
        if stage in RUN_STAGES:
            run = read_run(path) or {"schema_version": SCHEMA_VERSION, "sample_id": path.stem, "stages": []}
            run["stages"].append(entry)
            write_run(path, run)
        else:
            meta["stages"].append(entry)
        write(path, state)
        print(f"[{stage}] {result} {note}", flush=True)
        if result == "FAIL":
            return False
    return True
