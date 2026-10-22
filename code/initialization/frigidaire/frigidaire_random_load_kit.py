#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Completely random, Isaac-settled 24-dish HOTEC loads: one long-lived Kit worker builds loads in a loop.

Each load drops the 24 dishes (8 plates, 8 bowls, 8 cups) one at a time in a random order. A release draws the rack
in proportion to its box volume, an orientation uniform on SO(3) and an actor position uniform over the positions
that keep the WHOLE visual mesh inside the rack box; it is redrawn until the 1 mm FCL preflight clears the appliance
and the settled dishes. The dish is released at rest and must come to rest within 6 s (one passing 1 s window at the
gate's thresholds over every placed dish and the basket, every dish connected to a rack, basket seated); else the
drop is undone exactly (tensor restore of the pre-drop capture, then a re-settle window) and redrawn, K tries per
dish. A dish's rack is where it rests. The finished load is gated in the same process with the joint gate's own
calls: the loaded hold (12 s allowance, 5 s observation, 5 mm peak, 1 mm median), close_and_contain upper first,
and one lower-first retry from the settled capture. With <run>/CLOSURE_PRECHECK present at load start, every
passing drop is also checked Kit-free (tub wall, closed-rack containment, retraction sweep) and undone on a hit.

    code/util/run_kit.sh code/initialization/frigidaire/frigidaire_random_load_kit.py \\
        --run-dir data/results/random_loads/<run> --worker 0 --gen 0 --seed 20261009 --headless --device cpu

Outputs (repo-relative run dir): loads/<id>/result.json, index/wJJ_gG.jsonl, workers/wJJ_gG.status.json.
The worker's exit reason is in its status file and the '[RESULT] EXIT <code> <reason>' line (isaaclab.sh -p
swallows exit codes): 0 stopped (STOP file, start cutoff, hard stop), 3 recycle (--max-loads reached), 4 needs a
fresh Kit (reset sanity, baseline failure or simulation error).
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
KINDS = ("plate", "bowl", "cup")
USER_BOXES = {"LowerRack": ([-.2625, -.2805, .002], [.2625, .2805, .3563]),        # rack-local release boxes [m]
              "UpperRack": ([-.240, -.2575, -.0149], [.240, .2575, .227])}
WINDOW_ROWS = 121                       # 1 s at 120 Hz, the hold() window
FELL_Z_M = .15                          # visual centroid below this: off the rack (door/ground)
EXIT_STOPPED, EXIT_RECYCLE, EXIT_FRESH = 0, 3, 4
SCHEMA = "dishsim.random_load.v1"


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def parse_utc(text):
    return datetime.fromisoformat(text.replace("Z", "+00:00")) if text else None


def atomic_json(path, obj, indent=None):
    path = Path(path)
    tmp = path.with_name(path.name + f".tmp{os.getpid()}")
    tmp.write_text(json.dumps(obj, indent=indent, allow_nan=False, default=_json_default) + "\n")
    os.replace(tmp, path)


def _json_default(value):
    import numpy as np
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (set, tuple)):
        return sorted(value) if isinstance(value, set) else list(value)
    raise TypeError(f"not JSON serializable: {type(value)}")


def rss_mb():
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return round(int(line.split()[1]) / 1024., 1)
    except OSError:
        pass
    return None


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def park_pose(i):
    """Parked dishes rest on the ground far from the appliance (HOTEC visual z_min is 0 at the foot plane)."""
    return {"position_m": [3. + .4 * i, 0., .001], "quaternion_xyzw": [0., 0., 0., 1.]}


def reach_sets(pairs, active, seated, racks):
    """support_paths reachability, one BFS per rack: the active dishes connected to rack R through active dishes
    (plus the basket for the lower rack when it is seated). Other appliance bodies are never transit nodes."""
    act = set(active)
    adjacency = {}
    for a, b in pairs:
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)
    out = {}
    for rack in racks:
        allowed = act | {rack}
        if rack == "LowerRack" and seated:
            allowed.add("SilverwareBasket")
        seen, stack = {rack}, [rack]
        while stack:
            node = stack.pop()
            for other in adjacency.get(node, ()):
                if other in allowed and other not in seen:
                    seen.add(other)
                    stack.append(other)
        out[rack] = seen & act
    return out


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", type=Path, required=True, help="repo-relative run directory (host-created)")
    p.add_argument("--worker", type=int, required=True)
    p.add_argument("--gen", type=int, default=0)
    p.add_argument("--seed", type=int, default=20261009)
    p.add_argument("--max-loads", type=int, default=20, help="loads per process before a recycle exit")
    p.add_argument("--stop-file", type=Path, default=None, help="default <run-dir>/STOP")
    p.add_argument("--hard-stop-utc", default=None)
    p.add_argument("--start-cutoff-utc", default=None, help="no new load starts after this time")
    p.add_argument("--load-budget-s", type=float, default=2400.)
    p.add_argument("--tries-per-dish", type=int, default=8)
    p.add_argument("--fcl-max-proposals", type=int, default=4000)
    p.add_argument("--drop-max-ticks", type=int, default=720)
    p.add_argument("--resettle-max-ticks", type=int, default=360)
    p.add_argument("--eval-every", type=int, default=6)
    p.add_argument("--peak-penetration-m", type=float, default=.005)
    p.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    p.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    p.add_argument("--no-hull-motion-points", action="store_true")
    p.add_argument("--trace-final", action="store_true")
    p.add_argument("--n-per-kind", type=int, default=8, help="smoke runs only")
    p.add_argument("--debug-force-retry", action="store_true", help="smoke runs only: run the lower-first attempt anyway")
    p.add_argument("--debug-force-undo", action="store_true", help="smoke runs only: undo every first try of k >= 1")
    p.add_argument("--debug-drop-contact-limit-m", type=float, default=None,
                   help="smoke runs only: per-drop contact check threshold (default: the gate's LIMITS)")
    return p


class Worker:
    def __init__(self, args, app):
        self.args, self.app = args, app
        self.boot_started = time.monotonic()
        self.run = (ROOT / args.run_dir) if not args.run_dir.is_absolute() else args.run_dir
        self.tag = f"w{args.worker:02d}_g{args.gen}"
        for sub in ("loads", "index", "workers"):
            (self.run / sub).mkdir(parents=True, exist_ok=True)
        self.status_path = self.run / "workers" / f"{self.tag}.status.json"
        self.index_path = self.run / "index" / f"{self.tag}.jsonl"
        self.stop_files = [args.stop_file or (self.run / "STOP"), self.run / f"STOP.w{args.worker:02d}"]
        now_utc, now_mono = datetime.now(timezone.utc), time.monotonic()
        hard = parse_utc(args.hard_stop_utc)
        cutoff = parse_utc(args.start_cutoff_utc)
        self.hard_stop = now_mono + (hard - now_utc).total_seconds() if hard else float("inf")
        self.cutoff = now_mono + (cutoff - now_utc).total_seconds() if cutoff else self.hard_stop
        self.status = {"pid": os.getpid(), "worker": args.worker, "gen": args.gen, "load_id": None,
                       "phase": "boot", "k": None, "try": None, "tick_index": 0, "ms_per_tick_recent": None,
                       "loads_done": 0, "accepted": 0, "rss_mb": rss_mb(), "utc": utc_now(),
                       "cpus": sorted(os.sched_getaffinity(0)), "exit_code": None, "exit_reason": None}
        self._hb_last = (time.monotonic(), 0)
        self.write_status()

    # ----------------------------------------------------------------------------------------- status
    def write_status(self, **kw):
        self.status.update(kw)
        self.status["utc"] = utc_now()
        try:
            atomic_json(self.status_path, self.status)
        except OSError:
            pass

    def heartbeat(self, force=False):
        now = time.monotonic()
        last_t, last_tick = self._hb_last
        if not force and now - last_t < 15.:
            return
        tick = self.backend.tick_index if hasattr(self, "backend") else 0
        if tick > last_tick:
            self.status["ms_per_tick_recent"] = round((now - last_t) * 1e3 / (tick - last_tick), 2)
        self._hb_last = (now, tick)
        self.write_status(tick_index=tick, rss_mb=rss_mb())

    def stop_reason(self):
        if any(Path(p).exists() for p in self.stop_files):
            return "stop_file"
        now = time.monotonic()
        if now >= self.hard_stop:
            return "hard_stop"
        if now >= self.cutoff:
            return "start_cutoff"
        return None

    # ----------------------------------------------------------------------------------------- boot
    def boot(self):
        import numpy as np
        import torch
        import carb.settings
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        import frigidaire_bench as B
        from scipy.spatial import ConvexHull
        from dishsim_frigidaire.initial_state_runtime import (IsaacInitialStateBackend, all_vertex_step_speed,
                                                              window_motion_metrics, OBSERVATION_SECONDS)
        from dishsim_frigidaire.random_pose_runtime import COMPONENTS
        from dishsim_frigidaire.random_poses import (LIMITS, RACKS, compose_pose, relative_pose, motion_is_settled,
                                                     quaternion_matrix_xyzw, uniform_quaternion_xyzw,
                                                     source_geometry_domains)
        from dishsim_frigidaire.random_pose_experiment import check_deadline, BudgetExpired
        torch.set_num_threads(1)
        LIMITS["peak_penetration_m"] = float(self.args.peak_penetration_m)   # validate.py's override; read at call time
        self.np, self.torch, self.B = np, torch, B
        self.all_vertex_step_speed, self.window_motion_metrics = all_vertex_step_speed, window_motion_metrics
        self.OBSERVATION_SECONDS, self.COMPONENTS, self.LIMITS, self.RACKS = OBSERVATION_SECONDS, COMPONENTS, LIMITS, RACKS
        self.compose_pose, self.relative_pose, self.motion_is_settled = compose_pose, relative_pose, motion_is_settled
        self.qmat, self.uniform_q = quaternion_matrix_xyzw, uniform_quaternion_xyzw
        self.check_deadline, self.BudgetExpired = check_deadline, BudgetExpired

        n = self.args.n_per_kind
        self.ids = [(f"{k}_{i:02d}", k) for k in KINDS for i in range(1, n + 1)]
        self.kind_of = dict(self.ids)
        baseline = json.loads(self.args.baseline.read_text())
        self.baseline = baseline.get("snapshot", baseline)
        worker = self

        class DropBackend(IsaacInitialStateBackend):
            """Park, capture and exact tensor restore for undo; snapshots carry the contact pairs; tick heartbeats."""

            def park_all(self):
                for i, obj in enumerate(self.objects.values()):
                    self.set_rigid_pose(obj, park_pose(i))
                self.tracker.clear()
                self.previous_frames = None

            def capture(self):
                return {"joint_pos": self.dishwasher.data.joint_pos.clone(), "target": self.target.clone(),
                        "poses": self.poses(), "tick": self.tick_index}

            def restore_state(self, cap):
                torch = self.torch
                self.target.copy_(cap["target"])
                self.dishwasher.write_joint_state_to_sim(cap["joint_pos"], torch.zeros_like(cap["joint_pos"]))
                self.dishwasher.write_root_velocity_to_sim(torch.zeros((1, 6), device=self.sim.device))
                self.dishwasher.reset()
                self.dishwasher.set_joint_position_target(self.target)
                self.set_rigid_pose(self.basket, cap["poses"]["SilverwareBasket"])
                for name, obj in self.objects.items():
                    self.set_rigid_pose(obj, cap["poses"][name])
                self.tracker.clear()             # safe: every body was just teleported (and woken)
                self.previous_frames = None

            def snapshot(self):
                s = super().snapshot()
                s["contact_pairs"] = sorted(list(p) for p in self.latest_contact["pairs"])
                return s

            def tick(self):
                out = super().tick()
                if self.tick_index % 120 == 0:
                    worker.heartbeat()
                return out

        candidates = [{"object_id": oid, "kind": kind, "rack": "LowerRack", "pose_world": park_pose(i)}
                      for i, (oid, kind) in enumerate(self.ids)]
        self.domains = source_geometry_domains()
        self.backend = DropBackend(self.args.usd, self.run / "workers", device="cpu", domains=self.domains,
                                   deadline=float("inf"), app=self.app, candidates=candidates, tableware=B.tableware())
        self.backend.runtime_settings.update(fresh_scene_per_arrangement=False, scene_reset="park+baseline restore")
        self.checker = B.bench_checker()
        self.dt = self.backend.dt
        # release boxes: the source-derived sampling bounds, checked against the user's numbers
        boxes = self.domains["sampling_bounds_by_rack"]
        self.box = {r: (np.asarray(boxes[r]["lower_m"], float), np.asarray(boxes[r]["upper_m"], float)) for r in RACKS}
        for r, (lo, hi) in USER_BOXES.items():
            if np.abs(self.box[r][0] - lo).max() > 1e-3 or np.abs(self.box[r][1] - hi).max() > 1e-3:
                raise RuntimeError(f"sampling box of {r} {self.box[r]} differs from the user's {lo} {hi}")
        vol = {r: float(np.prod(self.box[r][1] - self.box[r][0])) for r in RACKS}
        self.p_lower = vol["LowerRack"] / (vol["LowerRack"] + vol["UpperRack"])
        # closed-rack containment for the precheck: the gate's cabinet interior (no tolerance) at the racks-in frames
        from dishsim_frigidaire.asset import BODY_POSITIONS
        inner = self.domains["interior_bounds"]
        self.contain_box = (np.asarray(inner["lower_m"], float), np.asarray(inner["upper_m"], float))
        self.closed_origin = {r: np.asarray(BODY_POSITIONS[r], float) for r in RACKS}
        self.points = self.backend.points
        self.hull = {k: self.points[k][ConvexHull(self.points[k]).vertices] for k in self.points}
        self.centroid = {k: self.points[k].mean(0) for k in self.points}
        rng = np.random.default_rng(12345)
        for k in self.hull:                                  # (i) every orientation fits both boxes
            for r in RACKS:
                for _ in range(1000):
                    rot = self.hull[k] @ self.qmat(self.uniform_q(rng)).T
                    if ((self.box[r][1] - rot.max(0)) - (self.box[r][0] - rot.min(0)) <= 0).any():
                        raise RuntimeError(f"empty feasible actor box for {k} in {r}")
        for k in self.hull:                                  # (ii) hull points give the exact all-vertex max speed
            for _ in range(50):
                a = {"position_m": rng.normal(size=3).tolist(), "quaternion_xyzw": self.uniform_q(rng).tolist()}
                b = {"position_m": rng.normal(size=3).tolist(), "quaternion_xyzw": self.uniform_q(rng).tolist()}
                full = all_vertex_step_speed(self.points[k], a, b, 1.)
                hull = all_vertex_step_speed(self.hull[k], a, b, 1.)
                if abs(full - hull) > 1e-12 * max(1., full):
                    raise RuntimeError(f"hull speed mismatch for {k}: {full} vs {hull}")
        self.motion_point_counts = {}
        if not self.args.no_hull_motion_points:
            for oid, kind in self.ids:
                self.backend.motion_points[oid] = self.hull[kind]
            basket = self.backend.motion_points["SilverwareBasket"]
            self.backend.motion_points["SilverwareBasket"] = basket[ConvexHull(basket).vertices]
            self.backend.runtime_settings["motion_points"] = "convex-hull vertices of every authored visual mesh (exact max)"
        self.motion_point_counts = {k: [len(self.points[k]), len(self.hull[k])] for k in self.points}
        self.physx_threads = carb.settings.get_settings().get("/persistent/physics/numThreads")
        self.boot_s = time.monotonic() - self.boot_started
        self.isaac_version = Path("/isaac-sim/VERSION").read_text().strip() if Path("/isaac-sim/VERSION").exists() else None
        src = [Path(__file__), ROOT / "code/frigidaire/src/dishsim_frigidaire/initial_state_runtime.py",
               ROOT / "code/frigidaire/src/dishsim_frigidaire/initial_state_candidates.py",
               ROOT / "code/frigidaire/src/dishsim_frigidaire/random_poses.py",
               ROOT / "code/planner/frigidaire/frigidaire_bench.py"]
        self.source_hashes = {str(p.relative_to(ROOT)): sha256(p) for p in src}
        self.provenance = {"baseline": str(self.args.baseline.relative_to(ROOT)) if self.args.baseline.is_absolute()
                           and str(self.args.baseline).startswith(str(ROOT)) else str(self.args.baseline),
                           "baseline_sha256": sha256(self.args.baseline),
                           "tableware": {k: str(Path(v).relative_to(ROOT)) for k, v in B.tableware().items()},
                           "tableware_sha256": {k: sha256(v) for k, v in B.tableware().items()},
                           "usd": str(self.args.usd), "execution_source_hashes": self.source_hashes}
        self.sweep_cache = {}
        print(f"[BOOT] {self.tag} boot {self.boot_s:.1f}s physx numThreads={self.physx_threads} cpus="
              f"{sorted(os.sched_getaffinity(0))} p_lower={self.p_lower:.4f} hull points {self.motion_point_counts}",
              flush=True)
        self.write_status(phase="booted", boot_s=round(self.boot_s, 1), physx_threads=self.physx_threads)

    # ----------------------------------------------------------------------------------------- helpers
    def pose_distance(self, a, b):
        np = self.np
        dp = float(np.linalg.norm(np.asarray(a["position_m"]) - b["position_m"]))
        qa, qb = np.asarray(a["quaternion_xyzw"]), np.asarray(b["quaternion_xyzw"])
        c = abs(float(qa @ qb) / (np.linalg.norm(qa) * np.linalg.norm(qb)))
        return dp, float(np.degrees(2 * np.arccos(min(1., c))))

    def centroid_z(self, oid, pose):
        return float(self.qmat(pose["quaternion_xyzw"])[2] @ self.centroid[self.kind_of[oid]] + pose["position_m"][2])

    def rack_frames(self, frames):
        return {r: frames[r] for r in self.RACKS}

    # ----------------------------------------------------------------------------------------- release sampler
    def propose(self, oid, placed, frames, rng):
        np, B, checker = self.np, self.B, self.checker
        kind = self.kind_of[oid]
        t0 = time.monotonic()
        checker.update_components({n: frames[n] for n in self.COMPONENTS})
        bodies = [checker._body(self.kind_of[d], frames[d], d) for d in placed]
        rejects = {}
        for n in range(1, self.args.fcl_max_proposals + 1):
            if n % 100 == 0:
                self.check_deadline(self.backend.deadline)
                self.heartbeat()
            rack = "LowerRack" if rng.random() < self.p_lower else "UpperRack"
            q = self.uniform_q(rng)
            rot = self.hull[kind] @ self.qmat(q).T
            lo, hi = self.box[rack][0] - rot.min(0), self.box[rack][1] - rot.max(0)
            local = rng.uniform(lo, hi)
            pos, quat = self.compose_pose(frames[rack]["position_m"], frames[rack]["quaternion_xyzw"], local, q)
            pose = {"position_m": pos.tolist(), "quaternion_xyzw": quat.tolist()}
            body = checker._body(kind, pose, oid)
            bad = None
            for other in bodies:
                result = checker.pair(body, other)
                if not result["valid"]:
                    bad = "unresolved_contact_query" if result["reason"] == "unresolved_contact_query" else "dish"
                    break
            if bad is None:
                result = checker.against_components(body)
                if not result["valid"]:
                    if result["reason"] == "unresolved_contact_query":
                        bad = "unresolved_contact_query"
                    else:
                        bad = next((p["bodies"][1] for p in result["pairs"] if not p["valid"]), "component")
            if bad is None:
                return {"rack": rack, "local": {"position_m": local.tolist(), "quaternion_xyzw": q.tolist()},
                        "world": pose, "feasible_box": [lo.tolist(), hi.tolist()]}, \
                       {"proposals": n, "rejects": rejects, "wall_s": round(time.monotonic() - t0, 3)}
            rejects[bad] = rejects.get(bad, 0) + 1
        return None, {"proposals": self.args.fcl_max_proposals, "rejects": rejects,
                      "wall_s": round(time.monotonic() - t0, 3)}

    # ----------------------------------------------------------------------------------------- rest window
    def drop_rest(self, active, new, max_ticks, rack_of):
        """Tick until one 1 s window passes the gate thresholds over the active dishes and the basket."""
        np, backend, dt = self.np, self.backend, self.dt
        racks = self.RACKS
        window = deque(maxlen=WINDOW_ROWS)
        prev = None
        t0, tick0 = time.monotonic(), backend.tick_index
        names = list(active) + ["SilverwareBasket"]
        out = {"result": "no_rest", "reason": "no passing rest window before the drop timeout", "ids": []}
        for t in range(1, max_ticks + 1):
            frames, joints = backend.tick()
            speeds = {}
            for d in active:
                if prev is None or frames[d] == prev[d]:
                    speeds[d] = 0.
                else:
                    speeds[d] = self.all_vertex_step_speed(backend.motion_points[d], prev[d], frames[d], dt)
            speeds["SilverwareBasket"] = backend.latest_speeds.get("SilverwareBasket", 0.)
            pairs = backend.latest_contact["pairs"]
            seated = ("LowerRack", "SilverwareBasket") in pairs
            window.append({"poses": frames, "speeds": speeds, "S": reach_sets(pairs, active, seated, racks),
                           "seated": seated, "peak_m": backend.latest_contact["peak_m"], "joints": joints})
            prev = frames
            if t % self.args.eval_every:
                continue
            if t > 30:
                fell = [d for d in active if self.centroid_z(d, frames[d]) < FELL_Z_M]
                if fell:
                    out = {"result": "fell_off", "reason": "visual centroid below 0.15 m", "ids": fell}
                    break
            if len(window) < WINDOW_ROWS:
                continue
            metrics = {n: self.window_motion_metrics(window, n, dt) for n in names}
            settled_each = {n: self.motion_is_settled(m) for n, m in metrics.items()}
            seated_all = all(row["seated"] for row in window)
            if not any(row["seated"] for row in window):
                out = {"result": "basket_unseated", "reason": "basket unseated over a full window", "ids": []}
                break
            cand = {d: [r for r in racks if all(d in row["S"][r] for row in window)] for d in active}
            lost = [d for d in active if settled_each[d] and not any(d in row["S"][r] for row in window for r in racks)]
            if lost:
                out = {"result": "rests_unsupported" if new in lost else "placed_rests_unsupported",
                       "reason": "settled with no contact path to a rack", "ids": lost}
                break
            if all(settled_each.values()) and seated_all and all(cand[d] for d in active):
                peaks = [row["peak_m"] for row in window]
                rf = self.rack_frames(frames)
                racks_now = {}
                for d in active:
                    c = cand[d]
                    if len(c) == 1:
                        r = c[0]
                    elif rack_of.get(d) in c:
                        r = rack_of[d]
                    else:
                        r = self.B.racked_in(self.points[self.kind_of[d]], frames[d], rf)
                        r = r if r in c else "LowerRack"
                    racks_now[d] = r
                deepest = sorted(backend.latest_contact["depths_m"].items(), key=lambda kv: -kv[1])[:3]
                out = {"result": "pass", "reason": None, "ids": [], "racks": racks_now,
                       "candidates": {d: c for d, c in cand.items() if len(c) > 1},
                       "window": {"max_peak_m": float(max(peaks)), "median_peak_m": float(np.median(peaks)),
                                  "contacts_ok": bool(max(peaks) < (self.args.debug_drop_contact_limit_m
                                                                    or self.LIMITS["peak_penetration_m"])
                                                      and np.median(peaks) < (self.args.debug_drop_contact_limit_m
                                                                    or self.LIMITS["median_max_penetration_m"])),
                                  "deepest_contacts_mm": [[k_, round(v_ * 1e3, 3)] for k_, v_ in deepest],
                                  "endpoints_ok": all(backend.endpoint_ok(row["joints"], backend.goal(racks))
                                                      for row in window)}}
                break
        else:
            moving, unsupported = [], []
            if len(window) == WINDOW_ROWS:
                metrics = {n: self.window_motion_metrics(window, n, dt) for n in names}
                moving = [n for n, m in metrics.items() if not self.motion_is_settled(m)]
                unsupported = [d for d in active if not any(all(d in row["S"][r] for row in window) for r in racks)]
                out["window"] = {"moving": moving, "unsupported": unsupported,
                                 "seated_all": all(row["seated"] for row in window)}
            out["ids"] = moving + [d for d in unsupported if d not in moving]
        ticks = backend.tick_index - tick0
        wall = time.monotonic() - t0
        out.update(ticks=ticks, sim_s=round(ticks * dt, 4), wall_s=round(wall, 3),
                   ms_per_tick=round(wall * 1e3 / max(1, ticks), 2), frames=backend.poses())
        return out

    def closure_precheck(self, racks_now, frames):
        """Kit-free: every placed dish inside the tub side walls and clear of the cabinet (and the closed upper rack)
        along its rack's retraction sweep; rack-local poses against the rack's current frame."""
        B = self.B
        hits = []
        for d, r in racks_now.items():
            kind = self.kind_of[d]
            p, q = self.relative_pose(frames[d]["position_m"], frames[d]["quaternion_xyzw"],
                                      frames[r]["position_m"], frames[r]["quaternion_xyzw"])
            if not B.tub_clear(self.points[kind], p, q):
                hits.append([d, "tub_wall"])
                continue
            closed = B.posed_points(self.points[kind], p, q) + self.closed_origin[r]
            if (closed < self.contain_box[0]).any() or (closed > self.contain_box[1]).any():
                hits.append([d, "contain"])
                continue
            key = (d, r, tuple(self.np.round(p, 4)), tuple(self.np.round(q, 4)))
            if key not in self.sweep_cache:
                self.sweep_cache[key] = bool(B.sweep_hits({"rack": r, "kind": kind, "position": p.tolist(),
                                                           "quaternion_xyzw": q.tolist()}))
            if self.sweep_cache[key]:
                hits.append([d, "sweep"])
        return hits

    # ----------------------------------------------------------------------------------------- one load
    def reset(self, record):
        backend = self.backend
        t0, tick0 = time.monotonic(), backend.tick_index
        backend.deadline = float("inf")
        backend.trace = None
        backend.assignments = {}
        rec = backend.prepare_baseline(self.baseline)
        backend.park_all()
        for _ in range(12):
            backend.tick()
        now = backend.poses()
        joints = backend.joints()
        dev = {}
        ok = rec["result"] == "PASS"
        for name in self.COMPONENTS:
            dp, dr = self.pose_distance(now[name], self.baseline["poses"][name])
            dev[name] = [round(dp * 1e3, 4), round(dr, 4)]
            if dp > (.001 if name == "SilverwareBasket" else .0005) or dr > .1:
                ok = False
        jdev = {}
        for name, value in self.baseline["joints"].items():
            d = abs(joints[name] - value)
            jdev[name] = round(math.degrees(d), 4) if name == "door_hinge" else round(d * 1e3, 4)
            if d > (math.radians(.05) if name == "door_hinge" else .0005):
                ok = False
        record["reset"] = {"hold_passed": rec["result"] == "PASS", "hold_reason": rec["hold"].get("reason"),
                           "component_dev_mm_deg": dev, "joint_dev_mm_or_deg": jdev, "sanity_passed": ok,
                           "ticks": backend.tick_index - tick0, "wall_s": round(time.monotonic() - t0, 3)}
        record["timings"]["phases"]["reset"] = {"wall_s": round(time.monotonic() - t0, 3), "ticks": backend.tick_index - tick0}
        return ok

    def build(self, record, rng, order, precheck, failfast=False, contact_undo=False):
        np, backend = self.np, self.backend
        args = self.args
        placed, rack_of = [], {}
        drops = record["drops"]
        totals = {"tries": 0, "undos": 0, "fcl_proposals": 0, "abandoned_at": None, "abandon_reason": None,
                  "first_deep_contact_k": None}
        ph = record["timings"]["phases"]
        for key in ("fcl", "drops", "undos"):
            ph[key] = {"wall_s": 0., "ticks": 0}
        ms_by_k = record["timings"]["ms_per_tick_by_k"]
        for k, oid in enumerate(order):
            kind = self.kind_of[oid]
            cap = backend.capture()
            drop = {"k": k, "object_id": oid, "kind": kind, "rack": None, "rack_changes": [], "tries": []}
            drops.append(drop)
            self.status.update(k=k)
            success = False
            k_wall, k_ticks = 0., 0
            for attempt in range(args.tries_per_dish):
                self.status.update(**{"try": attempt})
                frames = backend.poses()
                release, fcl = self.propose(oid, placed, frames, rng)
                totals["fcl_proposals"] += fcl["proposals"]
                ph["fcl"]["wall_s"] += fcl["wall_s"]
                entry = {"try": attempt, "release": release, "fcl": fcl}
                drop["tries"].append(entry)
                if release is None:
                    totals.update(abandoned_at=k, abandon_reason="abandoned_no_free_pose")
                    print(f"[DROP] {record['load_id']} k={k} {oid} try={attempt} no free pose in "
                          f"{fcl['proposals']} proposals", flush=True)
                    return placed, rack_of, totals
                totals["tries"] += 1
                backend.set_rigid_pose(backend.objects[oid], release["world"])
                backend.previous_frames = None
                phys = self.drop_rest(placed + [oid], oid, args.drop_max_ticks, rack_of)
                frames_after = phys.pop("frames")
                ph["drops"]["wall_s"] += phys["wall_s"]
                ph["drops"]["ticks"] += phys["ticks"]
                k_wall += phys["wall_s"]
                k_ticks += phys["ticks"]
                disturbance = max((self.pose_distance(cap["poses"][d], frames_after[d])[0] * 1e3 for d in placed),
                                  default=0.)
                phys["disturbance_mm"] = round(disturbance, 3)
                if phys["result"] == "pass" and contact_undo and not phys["window"]["contacts_ok"]:
                    phys.update(result="deep_contact", reason="the gate's contact check failed on the rest window")
                if phys["result"] == "pass" and args.debug_force_undo and attempt == 0 and k >= 1:
                    phys.update(result="debug_undo", reason="--debug-force-undo")
                if phys["result"] == "pass" and precheck:
                    hits = self.closure_precheck(phys["racks"], frames_after)
                    phys["precheck_hits"] = hits
                    if hits:
                        phys.update(result="closure_precheck", reason="tub wall or retraction sweep hit",
                                    ids=[h[0] for h in hits])
                entry["physics"] = phys
                print(f"[DROP] {record['load_id']} k={k} {oid} try={attempt} {release['rack']} -> {phys['result']} "
                      f"{phys['ticks']} ticks {phys['wall_s']:.1f}s {phys['ms_per_tick']} ms/tick "
                      f"fcl {fcl['proposals']}p {fcl['wall_s']:.2f}s", flush=True)
                if phys["result"] == "pass":
                    new_racks = phys["racks"]
                    drop["rack"] = new_racks[oid]
                    for d, r in new_racks.items():
                        if d in rack_of and rack_of[d] != r:
                            drop["rack_changes"].append([d, rack_of[d], r])
                    rack_of.update(new_racks)
                    placed.append(oid)
                    success = True
                    if not phys["window"]["contacts_ok"] and totals["first_deep_contact_k"] is None:
                        totals["first_deep_contact_k"] = k
                    break
                # undo: exact restore of the pre-drop capture, then a full passing re-settle window
                totals["undos"] += 1
                backend.restore_state(cap)
                undo = self.drop_rest(list(placed), None, args.resettle_max_ticks, rack_of)
                frames_undo = undo.pop("frames")
                ph["undos"]["wall_s"] += undo["wall_s"]
                ph["undos"]["ticks"] += undo["ticks"]
                k_wall += undo["wall_s"]
                k_ticks += undo["ticks"]
                drift = [self.pose_distance(cap["poses"][d], frames_undo[d]) for d in placed]
                entry["undo"] = {"ticks": undo["ticks"], "wall_s": undo["wall_s"], "passed": undo["result"] == "pass",
                                 "result": undo["result"], "ids": undo.get("ids", []),
                                 "max_dp_mm": round(max((x[0] for x in drift), default=0.) * 1e3, 3),
                                 "max_drot_deg": round(max((x[1] for x in drift), default=0.), 3)}
                if undo["result"] != "pass":
                    totals.update(abandoned_at=k, abandon_reason="abandoned_restore_failure")
                    print(f"[DROP] {record['load_id']} k={k} undo failed: {undo['result']} {undo.get('ids')}", flush=True)
                    return placed, rack_of, totals
                for d, r in undo["racks"].items():
                    if rack_of.get(d) != r:
                        drop["rack_changes"].append([d, rack_of.get(d), r])
                rack_of.update(undo["racks"])
            ms_by_k[str(k)] = round(k_wall * 1e3 / max(1, k_ticks), 2)
            self.write_status()
            if not success:
                totals.update(abandoned_at=k, abandon_reason="abandoned_tries_exhausted")
                return placed, rack_of, totals
            if failfast and totals["first_deep_contact_k"] is not None:
                totals.update(abandoned_at=k, abandon_reason="abandoned_deep_contact")
                print(f"[DROP] {record['load_id']} k={k} fail-fast: rest window fails the gate contact check "
                      f"{drop['tries'][-1]['physics']['window']['deepest_contacts_mm']}", flush=True)
                return placed, rack_of, totals
        return placed, rack_of, totals

    def gate(self, record, placed, rack_of):
        """The joint gate's own acceptance: loaded hold, close_and_contain upper first, one lower-first retry."""
        backend, LIMITS = self.backend, self.LIMITS
        backend.assignments = {d: rack_of[d] for d in placed}
        backend.loaded = True
        backend.previous_frames = None
        backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None
        # no tracker.clear(): sleeping pairs are retained only by the tracker
        attempts, settled_cap, deciding = [], None, None
        prepared = [{"object_id": d, "kind": self.kind_of[d]} for d in placed]
        for attempt, order in enumerate((("UpperRack", "LowerRack"), ("LowerRack", "UpperRack"))):
            t0, tick0 = time.monotonic(), backend.tick_index
            g = {"order": list(order), "loaded_rack_motions": [], "restore_drift": None}
            self.write_status(phase=f"gate_{attempt}")
            if attempt == 1:
                backend.restore_state(settled_cap)
                backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None
            settled = backend.hold(phase=f"loaded_settle_{attempt}", goal=backend.goal(self.RACKS), timeout=12.,
                                   observation=self.OBSERVATION_SECONDS, abort_on_peak=True)
            g["settled"] = settled
            g["maximum_settle_penetration_m"] = backend.maximum_penetration_m
            if backend.maximum_penetration_m >= LIMITS["peak_penetration_m"]:
                g.update(outcome="penetration_failure", reason="Excessive penetration during joint initial settling")
            elif not settled["passed"]:
                g.update(outcome="settle_failure", reason=settled.get("reason"))
            else:
                g["initial_snapshot"] = backend.snapshot()
                if attempt == 0:
                    settled_cap = backend.capture()
                else:
                    g["restore_drift"] = {d: [round(x * (1e3 if i == 0 else 1.), 3) for i, x in enumerate(
                        self.pose_distance(settled_cap["poses"][d], g["initial_snapshot"]["poses"][d]))] for d in placed}
                backend.close_and_contain(g, order, prepared)
            ticks = backend.tick_index - tick0
            g["ticks"], g["wall_s"] = ticks, round(time.monotonic() - t0, 3)
            record["timings"]["phases"][f"gate_{attempt}"] = {"wall_s": g["wall_s"], "ticks": ticks}
            attempts.append(g)
            deciding = g
            print(f"[GATE] {record['load_id']} attempt {attempt} {'/'.join(order)} -> {g['outcome']} "
                  f"({g.get('reason')}) {ticks} ticks {g['wall_s']:.1f}s", flush=True)
            if settled_cap is None:
                break
            if g["outcome"] != "closure_failure" and not (self.args.debug_force_retry and attempt == 0):
                break
        return attempts, deciding

    def summarize_attempt(self, g):
        s = g.get("settled") or {}
        motions = [{k: m.get(k) for k in ("rack", "passed", "reason", "failed_step", "duration_s", "maximum_penetration_m",
                                          "peak_measured_rack_speed_m_s", "unsupported_object_ids", "basket_supported")}
                   | {"hold_passed": (m.get("hold") or {}).get("passed"), "hold_reason": (m.get("hold") or {}).get("reason")}
                   for m in g.get("loaded_rack_motions", [])]
        return {"order": g["order"], "outcome": g.get("outcome"), "reason": g.get("reason"),
                "settled": {k: s.get(k) for k in ("passed", "reason", "simulated_seconds", "peak_penetration_m",
                                                  "median_max_penetration_m", "unsupported_object_ids",
                                                  "qualification_simulated_seconds", "early_abort")},
                "motions": motions, "maximum_settle_penetration_m": g.get("maximum_settle_penetration_m"),
                "maximum_cycle_penetration_m": g.get("maximum_cycle_penetration_m"),
                "ticks": g.get("ticks"), "wall_s": g.get("wall_s"), "restore_drift": g.get("restore_drift")}

    def run_load(self, load_index):
        np, backend, args = self.np, self.backend, self.args
        load_id = f"{self.tag}_l{load_index:04d}"
        out_dir = self.run / "loads" / load_id
        out_dir.mkdir(parents=True, exist_ok=True)
        started, tick_start = time.monotonic(), backend.tick_index
        precheck = (self.run / "CLOSURE_PRECHECK").exists()
        failfast = (self.run / "FAILFAST_CONTACT").exists()
        contact_undo = (self.run / "CONTACT_UNDO").exists()
        seed_seq = [args.seed, args.worker, args.gen, load_index]
        rng = np.random.default_rng(np.random.SeedSequence(seed_seq))
        order = [self.ids[i][0] for i in rng.permutation(len(self.ids))]
        record = {"schema": SCHEMA, "load_id": load_id, "worker": args.worker, "gen": args.gen,
                  "load_index": load_index, "seed": {"run_seed": args.seed, "seed_sequence": seed_seq},
                  "started_utc": utc_now(), "finished_utc": None, "outcome": "simulation_error", "reason": None,
                  "order": None, "precheck_enabled": precheck, "failfast_contact_enabled": failfast,
                  "contact_undo_enabled": contact_undo, "limits": dict(self.LIMITS),
                  "params": {"tries_per_dish": args.tries_per_dish, "drop_max_ticks": args.drop_max_ticks,
                             "resettle_max_ticks": args.resettle_max_ticks, "window_rows": WINDOW_ROWS,
                             "eval_every": args.eval_every, "fell_z_m": FELL_Z_M,
                             "fcl_max_proposals": args.fcl_max_proposals, "load_budget_s": args.load_budget_s,
                             "p_lower": self.p_lower,
                             "boxes": {r: [self.box[r][0].tolist(), self.box[r][1].tolist()] for r in self.RACKS},
                             "orientation": "uniform SO(3)",
                             "position": "uniform feasible actor box (whole visual mesh inside)",
                             "n_per_kind": args.n_per_kind, "debug_force_retry": args.debug_force_retry,
                             "debug_force_undo": args.debug_force_undo,
                             "closure_precheck": "tub_clear + sweep_hits on every passing drop" if precheck else None},
                  "dish_order": order, "racks": {}, "racked_in": {}, "objects": [],
                  "initial_snapshot": None, "final_snapshot": None, "settled": None, "loaded_rack_motions": [],
                  "final_hold": None, "final_containment": None, "maximum_settle_penetration_m": None,
                  "maximum_cycle_penetration_m": None, "gate_attempts": [], "drops": [], "build": None, "reset": None,
                  "timings": {"phases": {}, "ms_per_tick_by_k": {}},
                  "worker_info": {"pid": os.getpid(), "cpus": sorted(os.sched_getaffinity(0)),
                                  "physx_threads": self.physx_threads, "rss_mb": rss_mb(),
                                  "loads_in_process": load_index + 1, "boot_s": round(self.boot_s, 2),
                                  "isaac_sim_version": self.isaac_version, "hull_motion_points": not args.no_hull_motion_points,
                                  "motion_point_counts": self.motion_point_counts},
                  "provenance": {**self.provenance, "runtime_settings": backend.runtime_settings}}
        self.write_status(load_id=load_id, phase="reset", k=None, **{"try": None})
        exit_code = None
        trace = None
        try:
            if not self.reset(record):
                record.update(outcome="baseline_failure", reason="between-load reset hold or sanity check failed")
                exit_code = EXIT_FRESH
                return record, exit_code
            backend.deadline = min(self.hard_stop, time.monotonic() + args.load_budget_s)
            self.write_status(phase="build")
            placed, rack_of, totals = self.build(record, rng, order, precheck, failfast, contact_undo)
            record["build"] = totals
            frames = backend.poses()
            record["racks"] = {d: rack_of[d] for d in placed}
            record["objects"] = [{"object_id": d, "kind": self.kind_of[d], "rack": rack_of[d], "pose_world": frames[d]}
                                 for d in placed]
            if totals["abandon_reason"]:
                record.update(outcome=totals["abandon_reason"], reason=f"abandoned at k={totals['abandoned_at']}")
                return record, exit_code
            if args.trace_final:
                trace = (out_dir / "trace.jsonl").open("w", buffering=1)
                backend.trace = trace
            attempts, g = self.gate(record, placed, rack_of)
            record["gate_attempts"] = [self.summarize_attempt(a) for a in attempts]
            record.update(outcome=g["outcome"], reason=g.get("reason"), order=g["order"],
                          settled=g.get("settled"), loaded_rack_motions=g.get("loaded_rack_motions", []),
                          final_hold=g.get("final_hold"), final_containment=g.get("final_containment"),
                          maximum_settle_penetration_m=g.get("maximum_settle_penetration_m"),
                          maximum_cycle_penetration_m=g.get("maximum_cycle_penetration_m"),
                          initial_snapshot=g.get("initial_snapshot"), final_snapshot=g.get("final_snapshot"))
            snap = g.get("initial_snapshot")
            if snap is not None:
                poses = snap["poses"]
                rf = self.rack_frames(poses)
                objects = []
                for d in placed:
                    r = rack_of[d]
                    p, q = self.relative_pose(poses[d]["position_m"], poses[d]["quaternion_xyzw"],
                                              poses[r]["position_m"], poses[r]["quaternion_xyzw"])
                    objects.append({"object_id": d, "kind": self.kind_of[d], "rack": r, "pose_world": poses[d],
                                    "rack_local_pose": {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()},
                                    "final_pose_world": (g.get("final_snapshot") or {}).get("poses", {}).get(d)})
                record["objects"] = objects
                record["racked_in"] = {d: self.B.racked_in(self.points[self.kind_of[d]], poses[d], rf) for d in placed}
        except self.BudgetExpired as exc:
            record.update(outcome="timeout", reason=str(exc))
        except Exception as exc:                                     # noqa: BLE001
            traceback.print_exc()
            record.update(outcome="simulation_error", reason=repr(exc), traceback=traceback.format_exc())
            exit_code = EXIT_FRESH
        finally:
            backend.trace = None
            if trace is not None:
                trace.close()
            backend.deadline = float("inf")
            ticks = backend.tick_index - tick_start
            record.update(finished_utc=utc_now(), wall_seconds=round(time.monotonic() - started, 3),
                          ticks=ticks, sim_seconds=round(ticks * self.dt, 3))
            record["worker_info"]["rss_mb"] = rss_mb()
        return record, exit_code

    def finish_load(self, record):
        out = self.run / "loads" / record["load_id"] / "result.json"
        atomic_json(out, record)
        ph = record["timings"]["phases"]
        build = record.get("build") or {}
        line = {"load_id": record["load_id"], "outcome": record["outcome"], "reason": record.get("reason"),
                "order": record.get("order"), "wall_s": record.get("wall_seconds"), "sim_s": record.get("sim_seconds"),
                "ticks": record.get("ticks"),
                "build_ticks": sum(ph.get(k, {}).get("ticks", 0) for k in ("drops", "undos")),
                "gate_ticks": sum(ph.get(k, {}).get("ticks", 0) for k in ("gate_0", "gate_1")),
                "build_wall_s": round(sum(ph.get(k, {}).get("wall_s", 0.) for k in ("drops", "undos", "fcl")), 2),
                "gate_wall_s": round(sum(ph.get(k, {}).get("wall_s", 0.) for k in ("gate_0", "gate_1")), 2),
                "tries": build.get("tries"), "undos": build.get("undos"), "fcl_proposals": build.get("fcl_proposals"),
                "abandoned_at": build.get("abandoned_at"), "precheck_enabled": record.get("precheck_enabled"),
                "failfast_contact_enabled": record.get("failfast_contact_enabled"),
                "contact_undo_enabled": record.get("contact_undo_enabled"),
                "first_deep_contact_k": build.get("first_deep_contact_k"),
                "gate_contacts_ok": (record.get("settled") or {}).get("contacts_ok"),
                "n_placed": len(record.get("racks") or {}), "started_utc": record.get("started_utc"),
                "finished_utc": record.get("finished_utc"), "worker": record["worker"], "gen": record["gen"],
                "path": str(out.relative_to(ROOT)) if str(out).startswith(str(ROOT)) else str(out)}
        with self.index_path.open("a") as fh:
            fh.write(json.dumps(line, allow_nan=False, default=_json_default) + "\n")
            fh.flush()
        print(f"[LOAD] {record['load_id']} {record['outcome']} ({record.get('reason')}) wall {record.get('wall_seconds')}s "
              f"sim {record.get('sim_seconds')}s tries {build.get('tries')} undos {build.get('undos')}", flush=True)

    def loop(self):
        accepted = 0
        for load_index in range(self.args.max_loads):
            reason = self.stop_reason()
            if reason:
                return EXIT_STOPPED, reason
            record, exit_code = self.run_load(load_index)
            self.finish_load(record)
            accepted += record["outcome"] == "accepted"
            self.write_status(loads_done=load_index + 1, accepted=accepted, phase="between_loads", load_id=None)
            if exit_code is not None:
                return exit_code, record["outcome"]
            if record["outcome"] == "timeout" and time.monotonic() >= self.hard_stop:
                return EXIT_STOPPED, "hard_stop"
        return EXIT_RECYCLE, "max_loads"


def main():
    parser = build_parser()
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    if args.device != "cpu":
        raise SystemExit("[RESULT] EXIT 4 this sampler requires --device cpu")
    app, worker, code, reason = None, None, EXIT_FRESH, "crash"
    try:
        worker = Worker(args, None)
        early = worker.stop_reason()
        if early:
            code, reason = EXIT_STOPPED, early
        else:
            app = AppLauncher(args).app
            worker.app = app
            worker.boot()
            worker.backend.app = app
            code, reason = worker.loop()
    except Exception as exc:                                         # noqa: BLE001
        traceback.print_exc()
        code, reason = EXIT_FRESH, f"exception {exc!r}"
    finally:
        if worker is not None:
            worker.write_status(phase="exit", exit_code=code, exit_reason=reason, rss_mb=rss_mb())
        print(f"[RESULT] EXIT {code} {reason}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
