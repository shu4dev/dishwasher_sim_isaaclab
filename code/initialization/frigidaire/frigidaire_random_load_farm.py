#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Host-side farm for frigidaire_random_load_kit.py: launch, supervise, harvest and report (python3 stdlib only).

Each worker is one long-lived Kit process in the dishsim-isaac container, pinned to one physical core (taskset
"j,j+16" with --cores-per-worker 1), niced, BLAS capped to 1 thread, PhysX numThreads via --kit_args. Launches are
staggered and wait for free GPU memory. The target worker count follows --ramp ('4@0,8@420,12@840': W by elapsed
seconds) unless <run>/target_workers holds an integer; lowering W writes STOP.wJJ for the highest workers. Workers
that recycle (exit 3), need a fresh Kit (exit 4) or crash are relaunched with gen+1 (3 bad exits in 15 min park a
worker); a worker whose status file is older than --hang-s is killed by its container pid. The harvester writes
harvest.json and accepted.jsonl every loop and writes STOP once n_accepted >= --target-accepted. Physical cores 14-15
(vCPUs 14, 15, 30, 31) are reserved for the GPU track and never used.

    nohup python3 code/initialization/frigidaire/frigidaire_random_load_farm.py --run-dir data/results/random_loads/rl_20261009 \\
        --ramp 4@0,8@420,12@840 --cores-per-worker 1 --physx-threads 2 --seed 20261009 \\
        --hard-stop-utc 2026-10-09T10:30:00Z --start-cutoff-min 8 --target-accepted 249 --recycle-loads 20 \\
        --min-free-gpu-mb 6000 --stagger-s 20 > data/results/random_loads/rl_20261009/farm.out 2>&1 &
    python3 code/initialization/frigidaire/frigidaire_random_load_farm.py --run-dir <run> --report
    python3 code/initialization/frigidaire/frigidaire_random_load_farm.py --run-dir <run> --harvest | --stop | --kill
    python3 code/initialization/frigidaire/frigidaire_random_load_farm.py --run-dir <run> --single 13 --cores 12,13 \\
        --physx-threads 4 --max-loads 1          # a probe worker (foreground)
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
CONTAINER = "dishsim-isaac"
SCRIPT = "code/initialization/frigidaire/frigidaire_random_load_kit.py"
RESERVED_CPUS = {14, 15, 30, 31}
SMT = 16                                   # vCPU i and i+16 share a physical core
STOPPED_REASONS = {"stop_file", "hard_stop", "start_cutoff"}
GATED = {"accepted", "settle_failure", "penetration_failure", "closure_failure", "outside_dishwasher"}
ABANDONED = {"abandoned_tries_exhausted", "abandoned_no_free_pose", "abandoned_restore_failure", "abandoned_deep_contact"}


def utc_now():
    return datetime.now(timezone.utc)


def iso(t):
    return t.isoformat(timespec="seconds")


def parse_utc(text):
    return datetime.fromisoformat(text.replace("Z", "+00:00"))


def atomic_write(path, text):
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def rel(path):
    path = os.path.abspath(str(path))
    if path == str(ROOT) or path.startswith(str(ROOT) + os.sep):
        return os.path.relpath(path, str(ROOT))
    raise ValueError(f"path not visible inside the container: {path}")


def log(msg):
    print(f"[{iso(utc_now())}] {msg}", flush=True)


def gpu_free_used_mb():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.free,memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=30)
        free, used = (int(v) for v in r.stdout.strip().splitlines()[0].split(","))
        return free, used
    except Exception:                                   # noqa: BLE001
        return None, None


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except Exception:                                   # noqa: BLE001
        return None


def index_lines(run):
    lines = []
    for f in sorted((run / "index").glob("*.jsonl")):
        try:
            text = f.read_text()
        except OSError:
            continue
        for raw in text.splitlines():
            try:
                lines.append(json.loads(raw))
            except json.JSONDecodeError:                 # a partial last line
                pass
    return lines


def harvest(run, target_accepted=None, started_utc=None):
    lines = index_lines(run)
    counts = Counter(l.get("outcome") for l in lines)
    accepted = sorted((l for l in lines if l.get("outcome") == "accepted"), key=lambda l: l.get("finished_utc") or "")
    n_acc, attempted = len(accepted), len(lines)
    gated = sum(counts[o] for o in GATED)
    abandoned = sum(counts[o] for o in ABANDONED)
    now = utc_now()
    state = read_json(run / "farm_state.json") or {}
    start = started_utc or state.get("started_utc") or (min((l.get("started_utc") for l in lines if l.get("started_utc")),
                                                            default=None))
    hours = max(1e-9, (now - parse_utc(start)).total_seconds() / 3600.) if start else None
    rate_h = n_acc / hours if hours else None
    out = {"counts": dict(counts), "n_accepted": n_acc, "attempted": attempted, "gated": gated, "abandoned": abandoned,
           "rates": {"acceptance_per_attempted": n_acc / attempted if attempted else None,
                     "acceptance_per_gated": n_acc / gated if gated else None,
                     "abandon_rate": abandoned / attempted if attempted else None,
                     "accepted_per_h": rate_h, "loads_per_h": attempted / hours if hours else None},
           "five_over_n_plus_1": 5. / (n_acc + 1), "target_accepted": target_accepted,
           "eta_utc_target": (iso(now + timedelta(hours=(target_accepted - n_acc) / rate_h))
                              if target_accepted and rate_h and n_acc < target_accepted else None),
           "precheck_loads": sum(bool(l.get("precheck_enabled")) for l in lines),
           "failfast_contact_loads": sum(bool(l.get("failfast_contact_enabled")) for l in lines),
           "contact_undo_loads": sum(bool(l.get("contact_undo_enabled")) for l in lines),
           "start_utc": start, "now_utc": iso(now)}
    atomic_write(run / "accepted.jsonl", "".join(json.dumps({k: l.get(k) for k in ("load_id", "path", "finished_utc",
                                                                                     "wall_s", "order")}) + "\n"
                                                 for l in accepted))
    atomic_write(run / "harvest.json", json.dumps(out, indent=2) + "\n")
    return out


class Farm:
    def __init__(self, args):
        self.args = args
        self.run = (ROOT / args.run_dir).resolve() if not Path(args.run_dir).is_absolute() else Path(args.run_dir)
        self.rel = rel(ROOT / args.run_dir) if not Path(args.run_dir).is_absolute() else rel(args.run_dir)
        (self.run / "logs").mkdir(parents=True, exist_ok=True)
        self.hard_stop = parse_utc(args.hard_stop_utc)
        self.cutoff = self.hard_stop - timedelta(minutes=args.start_cutoff_min)
        self.ramp = sorted((float(t), int(w)) for w, t in (item.split("@") for item in args.ramp.split(",")))
        self.workers = {}                           # j -> {proc, gen, tag, launched, log}
        self.next_gen = defaultdict(int)
        self.bad_exits = defaultdict(list)
        self.parked = set()
        self.last_launch = 0.
        self.started_mono = time.monotonic()
        previous = read_json(self.run / "farm_state.json") or {}
        for tag in previous.get("gens_used", []):     # a restarted farm never reuses a (worker, gen) pair
            j, g = tag
            self.next_gen[j] = max(self.next_gen[j], g + 1)
        self.state = {"started_utc": previous.get("started_utc") or iso(utc_now()), "args": vars(args),
                      "hard_stop_utc": iso(self.hard_stop), "start_cutoff_utc": iso(self.cutoff),
                      "launches": previous.get("launches", []), "exits": previous.get("exits", []),
                      "w_timeline": previous.get("w_timeline", []), "running_timeline": previous.get("running_timeline", []),
                      "gpu": previous.get("gpu", []), "gens_used": previous.get("gens_used", []), "parked": [],
                      "stop_written": previous.get("stop_written")}
        atomic_write(self.run / "config.json", json.dumps({"args": vars(args), "written_utc": iso(utc_now())}, indent=2) + "\n")
        self.last_gpu = 0.

    def cpus(self, j, c=None):
        c = c or self.args.cores_per_worker
        phys = [self.args.first_core + j * c + i for i in range(c)]
        cpus = phys + [p + SMT for p in phys]
        if set(cpus) & RESERVED_CPUS or max(phys) >= SMT:
            return None
        return ",".join(str(x) for x in cpus)

    def target_w(self):
        override = self.run / "target_workers"
        if override.exists():
            try:
                return max(0, min(self.args.max_workers, int(override.read_text().strip())))
            except ValueError:
                pass
        elapsed = time.monotonic() - self.started_mono
        w = 0
        for t, n in self.ramp:
            if elapsed >= t:
                w = n
        return min(w, self.args.max_workers)

    def command(self, j, gen, cpus, physx_threads, max_loads):
        argv = [SCRIPT, "--run-dir", self.rel, "--worker", str(j), "--gen", str(gen), "--seed", str(self.args.seed),
                "--max-loads", str(max_loads), "--stop-file", f"{self.rel}/STOP",
                "--hard-stop-utc", iso(self.hard_stop), "--start-cutoff-utc", iso(self.cutoff),
                "--load-budget-s", str(self.args.load_budget_s), "--tries-per-dish", str(self.args.tries_per_dish),
                "--headless", "--device", "cpu"]
        if physx_threads:
            argv.append(f"--kit_args=--/persistent/physics/numThreads={physx_threads}")
        return ["docker", "exec", "-w", "/workspace/dishsim", "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1",
                "-e", "MKL_NUM_THREADS=1", CONTAINER, "taskset", "-c", cpus, "nice", "-n", "10",
                "/workspace/dishsim/code/util/run_kit.sh", *argv]

    def launch(self, j, cpus=None, physx_threads=None, max_loads=None):
        cpus = cpus or self.cpus(j)
        gen = self.next_gen[j]
        self.next_gen[j] += 1
        tag = f"w{j:02d}_g{gen}"
        cmd = self.command(j, gen, cpus, physx_threads if physx_threads is not None else self.args.physx_threads,
                           max_loads or self.args.recycle_loads)
        logf = self.run / "logs" / f"{tag}.log"
        fh = open(logf, "w")
        fh.write(f"# {iso(utc_now())} cpus {cpus}\n# {' '.join(cmd)}\n")
        fh.flush()
        proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
        fh.close()
        self.workers[j] = {"proc": proc, "gen": gen, "tag": tag, "launched": time.monotonic(), "log": str(logf), "cpus": cpus}
        self.state["launches"].append({"utc": iso(utc_now()), "worker": j, "gen": gen, "cpus": cpus})
        self.state["gens_used"].append([j, gen])
        self.last_launch = time.monotonic()
        log(f"launch {tag} cpus {cpus}")
        return proc

    def status(self, tag):
        return read_json(self.run / "workers" / f"{tag}.status.json")

    def container_kill(self, pid):
        subprocess.run(["docker", "exec", CONTAINER, "kill", "-9", str(pid)], capture_output=True, timeout=60)

    def stopped(self):
        return (self.run / "STOP").exists()

    def poll_workers(self):
        now = time.monotonic()
        for j, w in list(self.workers.items()):
            rc = w["proc"].poll()
            st = self.status(w["tag"]) or {}
            if rc is None:
                path = self.run / "workers" / f"{w['tag']}.status.json"
                age = now - w["launched"]
                try:
                    age = min(age, time.time() - path.stat().st_mtime)
                except OSError:
                    pass
                if age > self.args.hang_s and st.get("pid"):
                    log(f"hang: {w['tag']} status {age:.0f}s old, killing container pid {st['pid']}")
                    self.container_kill(st["pid"])
                    self.state["exits"].append({"utc": iso(utc_now()), "worker": j, "gen": w["gen"], "kind": "hang_kill",
                                                "pid": st.get("pid")})
                continue
            reason = st.get("exit_reason")
            code = st.get("exit_code")
            kind = ("stopped" if reason in STOPPED_REASONS else "recycle" if code == 3
                    else "fresh_kit" if code == 4 else "crash")
            self.state["exits"].append({"utc": iso(utc_now()), "worker": j, "gen": w["gen"], "rc": rc, "exit_code": code,
                                        "exit_reason": reason, "kind": kind, "loads_done": st.get("loads_done"),
                                        "accepted": st.get("accepted"), "rss_mb": st.get("rss_mb")})
            log(f"exit {w['tag']} rc={rc} code={code} reason={reason} kind={kind} loads={st.get('loads_done')}")
            del self.workers[j]
            if kind in ("fresh_kit", "crash"):
                self.bad_exits[j] = [t for t in self.bad_exits[j] if now - t < 900] + [now]
                if len(self.bad_exits[j]) >= 3:
                    self.parked.add(j)
                    self.state["parked"] = sorted(self.parked)
                    log(f"park worker {j}: 3 bad exits within 15 min")

    def scale(self, W):
        for j in range(self.args.max_workers):
            f = self.run / f"STOP.w{j:02d}"
            if j >= W and j in self.workers and not f.exists():
                f.write_text(iso(utc_now()) + "\n")
                log(f"scale down: STOP.w{j:02d}")
            elif j < W and f.exists():
                f.unlink()

    def can_start(self):
        return not self.stopped() and utc_now() < self.cutoff

    def maybe_launch(self, W):
        if not self.can_start() or time.monotonic() - self.last_launch < self.args.stagger_s:
            return
        free = [j for j in range(W) if j not in self.workers and j not in self.parked and self.cpus(j)
                and not (self.run / f"STOP.w{j:02d}").exists()]
        if not free:
            return
        gpu_free, _ = gpu_free_used_mb()
        if gpu_free is not None and gpu_free < self.args.min_free_gpu_mb:
            log(f"launch held: GPU free {gpu_free} MiB < {self.args.min_free_gpu_mb}")
            self.last_launch = time.monotonic()              # re-check after one stagger interval
            return
        self.launch(free[0])

    def save_state(self):
        self.state["running"] = {str(j): w["tag"] for j, w in self.workers.items()}
        self.state["parked"] = sorted(self.parked)
        self.state["updated_utc"] = iso(utc_now())
        atomic_write(self.run / "farm_state.json", json.dumps(self.state, indent=1) + "\n")

    def loop(self):
        log(f"farm start: run {self.rel}, hard stop {iso(self.hard_stop)}, cutoff {iso(self.cutoff)}, ramp {self.ramp}")
        last_w, killed = None, False
        while True:
            try:
                now_utc = utc_now()
                W = self.target_w()
                if W != last_w:
                    self.state["w_timeline"].append([iso(now_utc), W])
                    log(f"target W = {W}")
                    last_w = W
                self.poll_workers()
                h = harvest(self.run, self.args.target_accepted, self.state["started_utc"])
                if h["n_accepted"] >= self.args.target_accepted and not self.stopped():
                    (self.run / "STOP").write_text(f"n_accepted {h['n_accepted']} >= {self.args.target_accepted} "
                                                   f"at {iso(now_utc)}\n")
                    self.state["stop_written"] = iso(now_utc)
                    log(f"STOP written: n_accepted {h['n_accepted']}")
                self.scale(W)
                self.maybe_launch(W)
                if time.monotonic() - self.last_gpu >= 60:
                    free, used = gpu_free_used_mb()
                    self.state["gpu"].append([iso(now_utc), used, free, len(self.workers)])
                    self.last_gpu = time.monotonic()
                self.state["running_timeline"].append([iso(now_utc), len(self.workers)])
                if now_utc >= self.hard_stop + timedelta(seconds=60) and self.workers and not killed:
                    log("hard stop + 60 s: pkill remaining workers")
                    subprocess.run(["docker", "exec", CONTAINER, "pkill", "-f", f"{SCRIPT} --run-dir {self.rel}"],
                                   capture_output=True, timeout=60)
                    killed = True
                self.save_state()
                if not self.workers and (self.stopped() or now_utc >= self.cutoff):
                    harvest(self.run, self.args.target_accepted, self.state["started_utc"])
                    log(f"farm done: n_accepted {h['n_accepted']} of {h['attempted']} loads")
                    self.save_state()
                    return
            except Exception as exc:                   # noqa: BLE001
                log(f"farm loop error: {exc!r}")
            time.sleep(10)

    def single(self):
        cpus = self.args.cores
        phys = [int(x) for x in cpus.split(",")]
        allc = sorted(set(phys + [p + SMT for p in phys if p < SMT]))
        if set(allc) & RESERVED_CPUS:
            raise SystemExit(f"refusing reserved cpus {sorted(set(allc) & RESERVED_CPUS)}")
        cpus = ",".join(str(x) for x in allc)
        proc = self.launch(self.args.single, cpus=cpus, physx_threads=self.args.physx_threads,
                           max_loads=self.args.max_loads or self.args.recycle_loads)
        self.save_state()
        rc = proc.wait()
        log(f"single worker exited rc={rc}")
        self.poll_workers()
        self.save_state()


# --------------------------------------------------------------------------------------------- report

def _median(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 3) if xs else None


def w_at(timeline, t):
    w = None
    for stamp, n in timeline:
        if stamp <= t:
            w = n
        else:
            break
    return w


def report(run):
    h = harvest(run)
    state = read_json(run / "farm_state.json") or {}
    lines = index_lines(run)
    print(f"run {run}")
    print(f"loads {h['attempted']}  accepted {h['n_accepted']}  gated {h['gated']}  abandoned {h['abandoned']}  "
          f"5/(n+1) {h['five_over_n_plus_1']:.4f}")
    r = h["rates"]
    fmt = lambda v: "-" if v is None else f"{v:.3f}"
    print(f"acceptance/attempted {fmt(r['acceptance_per_attempted'])}  acceptance/gated {fmt(r['acceptance_per_gated'])}  "
          f"abandon rate {fmt(r['abandon_rate'])}  accepted/h {fmt(r['accepted_per_h'])}  loads/h {fmt(r['loads_per_h'])}  "
          f"ETA {h['eta_utc_target']}")
    print("outcomes:", dict(sorted(h["counts"].items(), key=lambda kv: -kv[1])))
    print(f"switches: closure precheck {h['precheck_loads']} loads, contact fail-fast {h['failfast_contact_loads']}, "
          f"contact undo {h['contact_undo_loads']}  (touch <run>/CLOSURE_PRECHECK | FAILFAST_CONTACT | CONTACT_UNDO)")
    deep = Counter((l.get("outcome"), l.get("first_deep_contact_k") is not None) for l in lines)
    print("deep contact (a passing drop window failing the gate contact check) x outcome:",
          {f"{o}|{'deep' if d else 'clean'}": n for (o, d), n in sorted(deep.items(), key=lambda kv: -kv[1])})
    ks = [l["first_deep_contact_k"] for l in lines if l.get("first_deep_contact_k") is not None]
    if ks:
        print(f"first deep-contact k: median {_median(ks)}  values {sorted(ks)[:40]}")
    reasons = Counter((l.get("outcome"), (l.get("reason") or "")[:70]) for l in lines if l.get("outcome") != "accepted")
    print("failure reasons:")
    for (o, why), n in reasons.most_common(12):
        print(f"  {n:4d}  {o}: {why}")
    for label, sel in (("all", lambda l: True), ("gated", lambda l: l.get("outcome") in GATED),
                       ("abandoned", lambda l: l.get("outcome") in ABANDONED)):
        sub = [l for l in lines if sel(l)]
        if sub:
            print(f"seconds per load [{label}, n={len(sub)}]: wall {_median([l.get('wall_s') for l in sub])}  "
                  f"sim {_median([l.get('sim_s') for l in sub])}  build wall {_median([l.get('build_wall_s') for l in sub])}  "
                  f"gate wall {_median([l.get('gate_wall_s') for l in sub])}  build ticks "
                  f"{_median([l.get('build_ticks') for l in sub])}  gate ticks {_median([l.get('gate_ticks') for l in sub])}  "
                  f"tries {_median([l.get('tries') for l in sub])}  undos {_median([l.get('undos') for l in sub])}")
    # per-try records from the result files
    timeline = state.get("running_timeline", [])
    tries_by_k, try_results, drift, undo_fail, undos = defaultdict(list), Counter(), [], 0, 0
    ms_w_bin = defaultdict(list)
    gate_ms_w = defaultdict(list)
    first_load_ms = defaultdict(list)
    for l in lines:
        rec = read_json(ROOT / l["path"]) if l.get("path") else None
        if not rec:
            continue
        mid = l.get("finished_utc") or ""
        W = w_at(timeline, mid) if timeline else None
        for d in rec.get("drops", []):
            tries_by_k[d["k"]].append(len([t for t in d["tries"] if t.get("physics")]))
            for t in d["tries"]:
                ph = t.get("physics")
                if ph:
                    try_results[ph["result"]] += 1
                u = t.get("undo")
                if u:
                    undos += 1
                    undo_fail += not u.get("passed")
                    drift.append(u.get("max_dp_mm"))
        for k, ms in (rec.get("timings", {}).get("ms_per_tick_by_k") or {}).items():
            k = int(k)
            kb = "0-3" if k <= 3 else "8-12" if 8 <= k <= 12 else "20-23" if k >= 20 else None
            if kb:
                ms_w_bin[(W, kb)].append(ms)
        ph = rec.get("timings", {}).get("phases", {})
        for g in ("gate_0", "gate_1"):
            if ph.get(g, {}).get("ticks"):
                gate_ms_w[W].append(ph[g]["wall_s"] * 1e3 / ph[g]["ticks"])
        if rec.get("load_index") == 0:
            first_load_ms[W].append((rec.get("timings", {}).get("ms_per_tick_by_k") or {}).get("0"))
    if tries_by_k:
        print("physics tries per dish by k (mean):",
              " ".join(f"{k}:{sum(v)/len(v):.2f}" for k, v in sorted(tries_by_k.items())))
    print("per-try results:", dict(try_results.most_common()))
    print(f"undos {undos}  restore failures {undo_fail}  undo drift mm median {_median(drift)} max "
          f"{max([d for d in drift if d is not None], default=None)}")
    print("ms/tick by W (running workers at load finish) and k bin (build), and gate:")
    for W in sorted({w for w, _ in ms_w_bin} | set(gate_ms_w), key=lambda x: (x is None, x)):
        bins = "  ".join(f"k{kb} {_median(ms_w_bin.get((W, kb), []))}" for kb in ("0-3", "8-12", "20-23"))
        print(f"  W={W}: {bins}  gate {_median(gate_ms_w.get(W, []))}  first-load k0 {_median(first_load_ms.get(W, []))}")
    exits = Counter(e.get("kind") for e in state.get("exits", []))
    print("worker exits:", dict(exits), " parked:", state.get("parked"))
    rss = [e.get("rss_mb") for e in state.get("exits", []) if e.get("rss_mb")]
    if rss:
        print(f"RSS at exit MB median {_median(rss)} max {max(rss)}")
    gpu = state.get("gpu", [])
    if gpu:
        print("GPU used MiB (utc, used, free, workers), last 5:", gpu[-5:])
    running = state.get("running", {})
    print(f"running now: {len(running)} {sorted(running.values())}")
    # projection
    if lines and r["loads_per_h"]:
        A = r["acceptance_per_attempted"] or 0.
        print(f"projection: {r['loads_per_h']:.1f} loads/h x A {A:.3f} = {r['loads_per_h']*A:.1f} accepted/h")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", required=True)
    p.add_argument("--ramp", default="4@0,8@420,12@840")
    p.add_argument("--max-workers", type=int, default=14)
    p.add_argument("--cores-per-worker", type=int, default=1)
    p.add_argument("--first-core", type=int, default=0)
    p.add_argument("--physx-threads", type=int, default=2)
    p.add_argument("--seed", type=int, default=20261009)
    p.add_argument("--hard-stop-utc", default="2026-10-09T10:30:00Z")
    p.add_argument("--start-cutoff-min", type=float, default=8.)
    p.add_argument("--target-accepted", type=int, default=249)
    p.add_argument("--recycle-loads", type=int, default=20)
    p.add_argument("--load-budget-s", type=float, default=2400.)
    p.add_argument("--tries-per-dish", type=int, default=8)
    p.add_argument("--min-free-gpu-mb", type=int, default=6000)
    p.add_argument("--stagger-s", type=float, default=20.)
    p.add_argument("--hang-s", type=float, default=420.)
    p.add_argument("--single", type=int, default=None, help="launch one probe worker J in the foreground")
    p.add_argument("--cores", default=None, help="--single: physical cores, e.g. 12,13 (SMT siblings added)")
    p.add_argument("--max-loads", type=int, default=None, help="--single: loads before exit")
    p.add_argument("--harvest", action="store_true")
    p.add_argument("--report", action="store_true")
    p.add_argument("--stop", action="store_true")
    p.add_argument("--kill", action="store_true")
    args = p.parse_args()
    run = (ROOT / args.run_dir) if not Path(args.run_dir).is_absolute() else Path(args.run_dir)
    if not run.is_dir():
        raise SystemExit(f"create the run dir on the host first: mkdir -p {args.run_dir}")
    if args.report:
        report(run)
    elif args.harvest:
        print(json.dumps(harvest(run, args.target_accepted), indent=2))
    elif args.stop:
        (run / "STOP").write_text(f"manual stop {iso(utc_now())}\n")
        print("STOP written")
    elif args.kill:
        subprocess.run(["docker", "exec", CONTAINER, "pkill", "-f", f"{SCRIPT} --run-dir {rel(run)}"])
    elif args.single is not None:
        if not args.cores:
            raise SystemExit("--single needs --cores")
        Farm(args).single()
    else:
        Farm(args).loop()


if __name__ == "__main__":
    main()
