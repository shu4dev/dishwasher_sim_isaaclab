#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac dependency graph of one 24-dish load (boot-first; one load per Kit process).

--edges pairs (the default; procedure v20, 2026-10-14, research/2026-10-14-newj-curve-and-leaning-pairs.md; the
first version, v18, wrote pairs/, pairs_more/, pairs_rerun/, pairs_perturbed/ and is in git history): ordered-pair
tests on the goal load alone. Candidate pairs: every dish pair whose collision pieces (frigidaire_bench.bench_checker's
convex parts) come within PAIR_M = 20 mm at the target poses, any rack (FCL distance; overlap recorded as 0 mm).
A test parks its absent set S off-scene (as build_sequence parks), teleports every other dish to its exact target
and re-settles that context 150 ticks; a context dish then outside its at-goal tolerance of its target has fallen:
it is parked too and the context re-settled again (FALL_ROUNDS at most; recorded per test as parked_fallen, with every
dish's lateral / dz / tilt from its target per round). The dropped dish B is released at its LOWEST CLEAR HOVER
(frigidaire_bench.clear_release: its target raised in 1 mm steps along its rack's up axis until the pose passes the
bench checker's 1 mm penetration allowance against every context dish at its re-settled pose and the appliance; past
60 mm "no clear release": not landed, nothing dropped) and settled 150 + 60 ticks: landed = within the at-goal
tolerance of its target and contact depth < 2 mm; disturbed = context dishes that end outside their own at-goal
tolerance of their targets. L(B): S = {B}, for every dish; R(A, B): S = {A, B}, both orientations of every candidate
pair. Each drop stores its margin to the five thresholds (lateral, height, tilt, contact, disturbance = the moved
context dishes' goal distances); a drop within MARGIN = 20 % of any is rerun under 3 perturbed releases (3 mm / 3 deg,
--perturb-* mechanism, seeds 1, 2, 3; 10 S + 1..3 with --perturb-seed S) and the majority is its verdict. Edges
(a before b): need (B, X) for every X fallen in the first re-settle of L(B)'s context; knock (B, X) for every X the
verdict of L(B) disturbed; block (B, A) when L(B)'s verdict did not land and R(A, B)'s did; one edge per (a, b) with
its reasons. A two-way pair with need both ways is mutual support ("place together or re-roll"); any other two-way
pair has its drops (L(a), L(b), R(a, b), R(b, a)) retested under the 3 perturbed releases and, if it stays, is a
cycle. Status: cycle (a cycle pair, or a cycle once each mutual pair is one node; no build); mutual_support (a
diagnostic build_sequence with each mutual pair released together at its targets + one clear hover, the rest one at a
time: not a benchmark move); otherwise build_sequence started from the edges certifies the graph: ok | no_order |
landing | moved, a failed build names the failing dish. Every constraint a build learns beyond the edges joins the
graph with reason "learned" (extra_constraints). Writes <out>/pairs/<load_id>.json (the random sampler: pairs_random/).

--goal built (with --edges pairs, 2026-10-14; the default gate tests the targets as they are): a jointly settled goal
may hold a two-way pair, two dishes that only settle that way together, so it cannot be built one dish at a time.
The targets are first re-settled by build_sequence under the own-load rule (settle deviation <= DEV_MAX_M of the
commanded pose = target + the goal hover, contact depth < 2 mm, no placed dish moved > 10 mm / 20 deg; up to
SEQUENCE_BUILDS builds), its learned set started from the edges of the load's <out>/pairs/<load_id>.json minus its
two-way pairs (and any edge still on a longer cycle), else empty. No build = status 'resettle_failed' (the builds
kept, no tests). Otherwise the built poses become the targets (targets_source 'sequential_build'; goal_delta =
B.goal_distance from the previous target to the built pose per dish, and its max) and everything above runs on them,
the certifying build under the at-goal tolerance of the built poses. Writes <out>/pairs_resettled/<load_id>.json
(pairs_resettled_random/): the pairs record plus goal_source 'sequential_build', goal_delta, resettle_builds,
resettle_initial, resettle_rule and original (the pairs record read, or null); entries_settled = the built goal.
--out-dir PATH (any --edges and --goal): the record and _kit/<load_id>/ go under PATH instead.

--edges swaps (2026-10-15, plans/2026-10-15-sequential-goals-dependency-graphs.md; implies --goal built): the sampler is
the --goal built re-settle started from no constraints (lowest centre first, up to SEQUENCE_BUILDS builds learning the
order); its last build is the WITNESS, its built poses the goal, and the placed dishes' poses at each of its releases
are the states the swaps start from. No build = 'resettle_failed', as above. Candidate pairs: the v20 rule (collision
pieces within PAIR_M at the goal) plus the sampler's learned pairs. Each pair (a before b in the witness) is swapped once
in the state the witness was in when a went in (those dishes at their witness poses, every later dish parked,
context() re-settles it; a context dish then outside its tolerance of that pose is parked as fallen): step 1 drops b,
step 2 (b landed) drops a with b at b's landed pose. A failed swap writes the edge a before b with one reason: need (step
1: b does not land), knock (a drop moves a placed dish: at its goal before, it ends outside its at-goal tolerance; not at
its goal, it moves > 10 mm / 20 deg, build_sequence's rule), block (step 2: a has no clear release or does not land once
b is in); support = b rests on a in the witness's final contacts. Margins and reruns as v20 (MARGIN, 3 perturbed
releases, majority). Library (Kit-free, LIBRARY_SEED): the witness, per edge the witness with b moved to just before a,
--library N random obeying orders and N random single-violation orders (an obeying order, one random edge reversed the
same way); each predicts the dish it fails at (the earliest failing dish of its violated edges: b for a step-1 edge, a
for a step-2 one) and stops two dishes after it. Tick poses every TRAJ_EVERY ticks of the witness and every drop go to
_kit/<load_id>/traj/<name>.json (frigidaire_dependency_clips.py). Writes <out>/sequential/<load_id>.json; the library
orders are built afterwards, in any number of Kit sessions: --order-index K[,K...] builds those orders through
build_sequence (fixed order, nothing learned, at-goal tolerance of the built goal) into _kit/<load_id>/library/<K>.json,
and --merge-library (Kit-free) writes their outcomes into the record.

--release insertion (with --edges swaps; 2026-10-15 b, plans/2026-10-15-reposing-sampler-five-samples.md; the first
version, released --height-mm above the target, is in git history; the default --release clear is the lowest clear hover
above): every move of the run (the sampler, both swap steps, the confirmations, the library) is
frigidaire_bench.insertion_release toward the dish's MOVE POSE (a plate upright in its slot, 3 mm toward its packed lean's
side; a bowl or cup at its packed pose): it starts with its lowest collision point 20 mm above its rack's rim wire centre
(its highest collision point under the tub ceiling), descends along the rack's up axis in 5 mm steps, each checked with
the bench checker's 1 mm allowance against every placed dish and the appliance; a placed dish in the way or a blocked
start blocks the move (no release: reason block), the appliance alone stops the descent, and the dish is released at rest
20 mm above the lowest clear step, then settles 150 + 60 ticks. Each drop records start_lift_mm, path_steps,
lowest_lift_mm, release_lift_mm, stop and, when blocked, blocked, blocked_step and blocked_lift_mm; the descent is the
first frames of each recorded trajectory (geometric: no ticks). The sampler is the re-posing sampler (build_sequence's
``repose``): a seeded random order; a dish that does not go in (blocked, not within DEV_MAX_M of its pose, a placed dish
disturbed) is undone and tried at up to REPOSE_TRIES other free poses of its kind with a clear descent (Reposer), else
it goes to the back of the order; the load fails only when no dish left goes in. The record adds reposing {seed, order,
attempts, reposed, repose_calls}, entries_chosen and moves_world (the pose and move each dish went in with), and
_kit/<load_id>/states.json keeps the swap states. --order-index builds with the record's release and moves.
--merge-library also writes the verdict (valid | invalid | refine | pending: every library order must behave as the
graph predicts); --refine (one Kit session) makes each edge of a violation that still built conditional (b needs one of
{a, C...}, each C confirmed by a drop) and appends FRESH_ORDERS orders obeying the refined graph; --sample (host python
on the box) runs the whole loop over the packings (sample_loop).
--plate-order lean-side (the default since plans/2026-10-16-lean-side-plate-order.md; random = the order above): an
upright plate settles leaning ~30 deg toward its lean side and overhangs the next 3-4 slots there, so the sampler's
permutation (the same seed) has each plate bank's positions refilled by that bank's plates in chain order, the slot
furthest on the lean side first (frigidaire_bench_kit.chain_order); bowls and cups stay where the permutation put them.
A plate that does not go in is re-posed along its chain only (its bank's slots past its own, nearest first), and with no
slot left it goes to the end of the plates still queued. The witness records plate_order and chains {plate: bank,
index, slot, placed_index, placed_slot}.
--backtrack N --learn-rounds R (plans/2026-10-16-backtracking-sampler-learning-loop.md): the sampler backtracks
(build_sequence's ``backtrack``: a dish that does not go in undoes up to N of its most recently placed blockers, goes
in, and they go back; reposing.backtracks); validity = every library order that obeys the graph lands all 24 and every
order that breaks an edge fails somewhere, the dish it failed at and the predicted one both recorded (verdict rows
failed_at, predicted_at, as_predicted); --merge-library runs up to R learning rounds: each obeying order that failed
adds a learned edge from its build's diagnosis and FRESH_ORDERS fresh obeying orders re-check (learn_round; the record's
learning {max_rounds, rounds, edges_added}).
--tolerance library (plans/2026-10-16-doubled-tolerance-rerun.md; default bench): the swaps (landing, the fallen context,
disturbance), the confirmations and the library builds judge against frigidaire_bench.AT_GOAL_LIBRARY (AT_GOAL doubled:
bowl and cup 30 mm lateral / 40 mm height / 30 deg, plate 36 / 40 / 32 deg) with contact depth < 4 mm; the sampler's
own-load rule (DEV_MAX_M, 2 mm, AT_GOAL for what it disturbs) is unchanged. The record keeps it as ``tolerance``, and
--order-index / --refine use the record's. --rejudge --records PATH... (Kit-free) rewrites saved samples' swap edges
from their saved drops under --tolerance into --out-dir (rejudge); --sample --resume-from DIR continues those samples'
libraries first and samples new packings only once all of them are valid.

--edges physical (plans/2026-10-16-physical-cause-graph.md; implies --release insertion and the re-posing sampler, no swaps
and no library): a valid sample is (1) built poses that pass the bench's joint gate, (2) a witness order that rebuilds all 24
in a fresh Kit process and (3) a graph whose every edge is a physical cause in the finished load. Per load, one Kit process
each: the sampler (unchanged) writes <out>/<load_id>.json with procedure 'physical'; --manifest-out writes the joint-gate
manifest of its built poses (entries_settled); frigidaire_initial_state_validate.py gates them (--peak-penetration-m 0.005,
lower first once on closure_failure); --certify --gate DIR rebuilds the witness order from empty racks in a fresh process
(build_sequence, fixed order, the at-goal tolerance of the built pose: certified = 24 of 24 land), then: SUPPORT from the
rebuild's resting contacts (the net contact normal force on b from a, sign calibrated on the dish-rack contacts, pointing up
by more than FORCE_MIN_N; the higher-centre rule when the calibration fails), one candidate set per supported dish; ACCESS
(geometric, Kit-free in itself): b before a when a's final pose meets b's own insertion path (insertion_release against a
alone and the appliance, start down to the lowest clear step); CONFIRMATION drops per support candidate set S of b, each b
dropped by the move into the finished load (also removed: the dishes that cover b's path and everything resting,
transitively, on b or a removed support; anything that falls in the re-settle is parked; landed = the at-goal tolerance of
b's rebuilt pose, contact under the limit): without S (it must not land, else 'free'), with each support alone (which ones
suffice, kept as the disjunction), and with several supports, with all of S (the control) and with only one removed. Only
HARD requirements become edges (coordinator, 2026-10-16): a single support when b fails without it and lands with it; one
of several when b fails without that one alone. Graph (physical_graph): hard supports and every access hit; a pair with
arcs both ways loses both (conflicts), an edge the witness breaks is not drawn (against_witness), a dish in no edge is
free, layers for the DAG drawing. Writes gate, certify, contacts, access, confirmations, graph, valid into the record and
the drops' trajectories to _kit/<load_id>/traj/ (certify_witness, conf_<b>_<none|a|all|no_a>). --sample --edges physical
(physical_loop) schedules it all; --resume-from DIR imports built --edges swaps samples' sampler output first.

--perturb-mm X --perturb-deg Y --perturb-seed S (--edges pairs; default 0 0 0 = unperturbed): the settle is
deterministic (a rerun reproduced every drop bit for bit), so a plain rerun measures nothing. With X or Y > 0 each L
and R drop commands the dropped dish at its release pose (v20: the lowest clear hover found at the unperturbed
target; v18: target + the goal hover), yawed U(-Y, Y) deg about the world Z axis
through its visual centroid, then shifted by a point uniform in the disc of radius X mm in its rack's plane (numpy
default_rng((S, test number)), reproducible); the context dishes, the resettle and the certifying build are never
perturbed, and landing is judged against the unperturbed target (and command) with the same tolerances. Per test
dx_mm, dy_mm (rack axes) and yaw_deg as applied; per record perturbation {lateral_mm, yaw_deg, seed} (zeros when off).

--edges build (2026-10-13): the load is built one dish at a time in the baseline's extended racks by the bench's
sequence gate (frigidaire_bench_kit.build_sequence: lowest centre first, 150 + 60 ticks per dish, up to 8 builds while
it learns the order). Edges (a before b): support (final contacts, the higher dish rests on the lower), learned (from
failed builds) and neighbour (same-rack pairs within 15 cm keep the successful build's order,
frigidaire_bench.neighbour_order). Writes <out>/<sampler>/<load_id>.json.
Waves (both modes) = longest-path layers (sources at 0).
Targets (2026-10-14: r05_00014's packed poses did not build, its shingled plates landed 10-13 mm deep): with --gate
the load has passed the top-5 joint gate (all 24 teleported and settled at once, 5 mm momentary peak, both racks
retracted, containment) and every dish must land within the at-goal tolerance of its JOINTLY SETTLED pose, the
benchmark's goal path (frigidaire_bench_kit.sequence_plan --gate); a load that failed its gate gets status 'gate',
no edges and no waves, written without booting Kit. Without --gate (the fallback) restricted loads are built toward
their commanded packed poses (lifted up to 60 mm) within DEV_MAX_M of the command (the own-load rule), random farm
loads toward their farm-settled poses within the at-goal tolerance.

Per load, from the repo root (<id> e.g. r05_00014):
(a) the joint-gate manifest, Kit-free (isaaclab is never imported; run it through run_py.sh, inside the container,
    since frigidaire_hotec_top5.manifest_of writes the container's absolute tableware paths):
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --manifest-out data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json
(b) the joint gate with frigidaire_hotec_top5.gate_unit's flags; it prints [RESULT] <outcome> (accepted passes):
    code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json --out-dir data/results/dot_rules/dependency/restricted/_gate/<id> --max-wall-seconds 600 --peak-penetration-m 0.005 --headless --device cpu
    on closure_failure (the rack-speed flake) gate_unit reruns it once lower first: the same command with
    --order lower_first --out-dir data/results/dot_rules/dependency/restricted/_gate/<id>_lower_first, then give
    (c) that directory if it is accepted
(c) the graph (24 + 2 x pairs drops of 150 + 210 ticks, the fallen-context, marginal and two-way reruns, then the
    certifying build: about 35-70 min of Kit per load):
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --edges pairs --max-wall-seconds 7200 --headless
    (--goal built: the goal re-settled one dish at a time first, 1-8 more builds; --edges build --force: the
    2026-10-13 build-learned graph)
(d) --edges swaps (no gate; the restricted packing's commanded poses are the sampler's targets), then the library in
    parallel sessions (one physical core each), then the merge, Kit-free:
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --edges swaps --library 10 --out-dir data/results/dot_rules/dependency/sequential --max-wall-seconds 14400 --headless
    code/util/run_kit.sh ... the same ... --order-index 0,13,26
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --edges swaps --out-dir data/results/dot_rules/dependency/sequential --merge-library

Writes the <load_id>.json above and <folder>/_kit/<load_id>/report.json.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import random
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
RACK_FRAMES = ("LowerRack", "UpperRack", "SilverwareBasket")
PAIR_M = .02                      # --edges pairs: candidate pairs' collision pieces within 20 mm at the target poses
REASONS = ("knock", "need", "block", "learned")
MARGIN = .2                       # a drop within 20 % of any of its five thresholds is rerun under 3 perturbed releases
RERUN_MM, RERUN_DEG, RERUN_SEEDS = 3., 3., (1, 2, 3)   # seeds 10 S + 1..3 with --perturb-seed S (1, 2, 3 unperturbed)
MOVED_MM, MOVED_DEG = 1., 1.      # a context dish the drop moved by more: its goal distance enters the disturbance margin
FALL_ROUNDS = 4                   # context re-settles: the fallen are parked and the context re-settled, at most 4 rounds
LANDING_TAIL = ("contact depth < 2 mm; released at the lowest clear hover; no placed dish at its goal ends outside its "
                "at-goal tolerance (a placed dish not at its goal: moved > 10 mm / 20 deg)")
TRAJ_EVERY, PARK_X = 4, 2.        # --edges swaps: tick poses every 4 ticks; a dish beyond x = 2 m is parked
LIBRARY_SEED = 0                  # --edges swaps: the random library orders
REPOSE_TRIES = 5                  # --release insertion: other free poses with a clear descent tried per failed dish
FRESH_ORDERS = 10                 # --refine: fresh random orders obeying the refined graph
CONFIRM_INDEX = 500               # --refine: trajectory numbers of the confirmation drops start here
FORCE_MIN_N = .05                 # --edges physical: a dish-dish contact force whose up component exceeds 0.05 N is support
CALIBRATE_MIN = .8                # its sign: >= 80 % of the dish-rack contacts must push their dish up, else the centre rule
GATE_PEAK_M = .005                # the joint gate's peak penetration (frigidaire_hotec_top5.gate_unit's flag)


class Refused(Exception):
    """An expected stop (no traceback): a non-accepted farm load, a baseline that does not hold."""


def rel(path):
    p = Path(path).absolute()
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


def plain_kind(kind):
    return kind[6:] if kind.startswith("hotec_") else kind


def read_load(args):
    """(record path, entries): rack-local entries (racks out = the rack body frame), kinds plate | bowl | cup."""
    if args.sampler == "restricted":
        path = Path(args.restricted_dir) / f"{args.load}.json"
        data = json.loads(path.read_text())
        entries = [{"id": e["id"], "kind": plain_kind(e["kind"]), "rack": e["rack"], "slot": e.get("slot"),
                    "variant": e.get("variant"), "position": [float(v) for v in e["position"]],
                    "quaternion_xyzw": [float(v) for v in e["quaternion_xyzw"]], "keep": bool(e.get("keep", False))}
                   for e in data["entries"]]
    else:
        path = Path(args.farm_dir) / "loads" / args.load / "result.json"
        data = json.loads(path.read_text())
        if data.get("outcome") != "accepted":
            raise Refused(f"farm load outcome {data.get('outcome')!r}, not accepted")
        entries = [{"id": o["object_id"], "kind": plain_kind(o["kind"]), "rack": data["racks"][o["object_id"]],
                    "slot": "random", "variant": "random",
                    "position": [float(v) for v in o["rack_local_pose"]["position_m"]],
                    "quaternion_xyzw": [float(v) for v in o["rack_local_pose"]["quaternion_xyzw"]], "keep": False}
                   for o in data["objects"]]
    return path, entries


def read_gate(args, entries):
    """The load's finished joint gate (frigidaire_initial_state_validate.py --out-dir), read Kit-free."""
    path = Path(args.gate) / "result.json"
    if not path.is_file():
        raise Refused(f"no joint gate result {rel(path)}")
    gate = json.loads(path.read_text())
    planned = {o["object_id"]: o["rack_local_pose"]["position_m"] for o in gate.get("candidate_objects") or []}
    if planned and planned != {e["id"]: e["position"] for e in entries}:    # every load has the same 24 dish ids
        raise Refused(f"the joint gate {rel(args.gate)} ran on another load")
    return gate


def gate_info(args, gate):
    return {"dir": rel(args.gate), "outcome": gate.get("outcome"), "reason": gate.get("reason") or gate.get("error"),
            "peak_penetration_limit_m": (gate.get("limits") or {}).get("peak_penetration_m"),
            **{k: gate[k] for k in ("maximum_settle_penetration_m", "maximum_cycle_penetration_m", "wall_seconds")
               if k in gate}}


def layers(ids, edges):
    """Longest-path layer of every dish over (a before b) edges, sources at 0; None when the edges hold a cycle."""
    succ, indeg = {i: [] for i in ids}, {i: 0 for i in ids}
    for a, b in set(map(tuple, edges)):
        succ[a].append(b)
        indeg[b] += 1
    wave, todo, done = {i: 0 for i in ids}, [i for i in ids if indeg[i] == 0], 0
    while todo:
        i = todo.pop()
        done += 1
        for j in succ[i]:
            wave[j] = max(wave[j], wave[i] + 1)
            indeg[j] -= 1
            if indeg[j] == 0:
                todo.append(j)
    return wave if done == len(ids) else None


def local_entries(P, entries, frames, poses):
    """``entries`` at the world ``poses``, rack-local in the rack ``frames`` (the score script's input)."""
    out = []
    for e in entries:
        local = P.local_from_world(frames[e["rack"]], poses[e["id"]])
        out.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
    return out


def pairs_folder(args, name="pairs"):
    return args.out / (name if args.sampler == "restricted" else f"{name}_{args.sampler}")


def record_folder(args):
    """--out-dir, else <out>/<sampler>/ (--edges build), <out>/pairs/ or, --goal built, <out>/pairs_resettled/ (the
    random sampler: pairs_random/, pairs_resettled_random/)."""
    if args.out_dir:
        return args.out_dir
    if args.edges == "build":
        return args.out / args.sampler
    if args.edges == "swaps":
        return pairs_folder(args, "sequential")
    return pairs_folder(args, "pairs_resettled" if args.goal == "built" else "pairs")


def write_record(args, out_path, dep):
    if out_path.exists() and not args.force:
        raise Refused(f"refusing to overwrite {rel(out_path)} (--force)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(out_path.name + ".tmp")
    tmp.write_text(json.dumps(dep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    os.replace(tmp, out_path)


def goal_setup(args, entries, gate, B, K, P):
    """The module docstring's targets: (baseline snapshot, its rack frames, {id: world target pose}, landing(kind,
    dist, dev), the rule's text, targets_source, entries_settled). Kit-free."""
    base = json.loads(Path(args.baseline).read_text())
    snapshot = base.get("snapshot", base)
    frames = {r: snapshot["poses"][r] for r in RACK_FRAMES}
    targets = {e["id"]: P.world_from_local(frames[e["rack"]], {"position_m": e["position"],
                                                               "quaternion_xyzw": e["quaternion_xyzw"]}) for e in entries}
    tail = LANDING_TAIL
    if args.sampler == "restricted":
        landing = lambda kind, dist, dev: dev <= K.DEV_MAX_M                          # noqa: E731
        rule = (f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the commanded packed pose "
                f"(released {B.GOAL_HOVER_M * 1e3:.0f} mm above it), {tail}")
    else:
        landing = lambda kind, dist, dev: B.within_goal(kind, dist)                   # noqa: E731
        tol = ", ".join(f"{k} {v[0] * 1e3:.0f} mm lateral / {v[1] * 1e3:.0f} mm dz / {v[2]:.0f} deg"
                        for k, v in B.AT_GOAL.items())
        rule = f"at-goal tolerance of the settled pose ({tol}), {tail}"
    targets_source = "commanded" if args.sampler == "restricted" else "farm_settled"
    entries_settled = []
    if gate is not None:          # accepted joint gate: its settled world poses (racks extended, the same baseline)
        joint = gate["initial_snapshot"]["poses"]
        targets = {e["id"]: joint[e["id"]] for e in entries}
        landing = lambda kind, dist, dev: B.within_goal(kind, dist)                   # noqa: E731
        targets_source, rule = "joint_gate", f"at-goal tolerance of the jointly settled pose, {tail}"
        entries_settled = local_entries(P, entries, joint, joint)    # rack-local in the gate's measured racks
    return snapshot, frames, targets, landing, rule, targets_source, entries_settled


def dependency(args, source, entries, gate, out_path, kit_dir, report, started):
    """--edges build: the 2026-10-13 graph (support + learned + neighbour edges of the build)."""
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points

    snapshot, frames, targets, landing, rule, targets_source, entries_settled = goal_setup(args, entries, gate, B, K, P)
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    print(f"building {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes; "
          f"landing = {rule}", flush=True)
    backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    by_id = {e["id"]: e for e in entries}
    built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids, landing)
    builds = record["builds"]
    for k, b in enumerate(builds):
        print(f"build {k + 1}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)

    ids = [e["id"] for e in entries]
    learned = sorted(tuple(x) for x in record["learned"])
    ok = built == "ok"
    if ok:
        status = "ok"
        support = sorted(set(map(tuple, record["goal_order"])) - set(learned))
        neighbour = [tuple(x) for x in B.neighbour_order(record["order"], record["poses_world"],
                                                          {e["id"]: e["rack"] for e in entries})]
        poses_world = record["poses_world"]
        built_entries = local_entries(P, entries, frames, poses_world)
    else:
        support, neighbour, poses_world, built_entries = [], [], {}, []
        status = ("cycle" if layers(ids, learned) is None else "no_order" if len(builds) >= K.SEQUENCE_BUILDS
                  else builds[-1]["status"])
    deps = sorted(set(support) | set(learned))
    waves = layers(ids, sorted(set(deps) | set(neighbour)) if ok else learned)
    dep_waves = layers(ids, deps) if ok else None
    n_waves = max(waves.values()) + 1 if waves is not None else None
    reason = record["reason"] + ("; the learned order constraints form a cycle" if status == "cycle" else "")
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "status": status, "reason": reason,
           "builds": builds, "learned": [list(x) for x in learned], "support": [list(x) for x in support],
           "neighbour": [list(x) for x in neighbour], "goal_order": [list(x) for x in deps],
           "order": record.get("order") or builds[-1]["order"], "entries": entries, "targets_source": targets_source,
           "targets_world": targets, "poses_world": poses_world, "entries_settled": entries_settled,
           "built_entries": built_entries, "frames": frames,
           **({"waves": waves} if waves is not None else {}), "n_waves": n_waves,
           "n_waves_dependencies": max(dep_waves.values()) + 1 if dep_waves is not None else None,
           "acyclic": waves is not None, "failed_dish": None if ok else builds[-1]["dish"], "landing_rule": rule,
           "gate": gate_info(args, gate) if gate is not None else None,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "builds_run": len(builds)},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    report.update(result="PASS", status=status, dependency=rel(out_path),
                  reason=f"status {status} builds {len(builds)} support {len(support)} learned {len(learned)} "
                         f"neighbour {len(neighbour)} waves {n_waves if n_waves is not None else 'none'} "
                         f"targets {targets_source}")


def candidate_pairs(checker, entries, targets, max_m=PAIR_M):
    """[a, b, distance_mm] (a < b, sorted) for every dish pair whose collision pieces come within ``max_m`` at the
    world ``targets``, any rack: the checker's convex parts per dish (InitialCollisionChecker._body), an AABB prefilter
    on the authored collision bounds, then one FCL distance over the two dishes' pieces (overlap, FCL's -1, is 0).
    Kit-free (pxr reads the parts: run_py.sh, or after the Kit boot)."""
    import numpy as np
    fcl = checker.fcl
    bodies = [checker._body(e["kind"], targets[e["id"]], e["id"]) for e in sorted(entries, key=lambda e: e["id"])]
    out = []
    for n, a in enumerate(bodies):
        for b in bodies[n + 1:]:
            if np.any(a["bounds"][0] > b["bounds"][1] + max_m) or np.any(b["bounds"][0] > a["bounds"][1] + max_m):
                continue
            data = fcl.DistanceData(request=fcl.DistanceRequest(), result=fcl.DistanceResult())
            a["manager"].distance(b["manager"], data, fcl.defaultDistanceCallback)
            d = max(float(data.result.min_distance), 0.)
            if d < max_m:
                out.append([a["id"], b["id"], round(d * 1e3, 2)])
    return out


def graph_edges(tests):
    """{(a, b): [reasons]} from the v20 tests (module docstring): knock (B, X) for every X the verdict of L(B) disturbed
    (ended outside its at-goal tolerance), need (B, X) for every X outside its tolerance after the first re-settle of
    L(B)'s context, block (B, A) when L(B)'s verdict did not land and R(A, B)'s did."""
    last = {t["dish"]: t for t in tests if not t["removed"]}
    reasons = {}
    for b, t in last.items():
        for x in t["verdict"]["disturbed"]:
            reasons.setdefault((b, x), set()).add("knock")
        for x in t["context_rounds"][0]["fallen"]:
            reasons.setdefault((b, x), set()).add("need")
    for t in tests:
        if t["removed"]:
            a, b = t["removed"][0], t["dish"]
            if not last[b]["verdict"]["landed"] and t["verdict"]["landed"]:
                reasons.setdefault((b, a), set()).add("block")
    return {e: [r for r in REASONS if r in rs] for e, rs in sorted(reasons.items())}


def two_way(edges):
    """[[a, b], class] for every pair with edges both ways (a < b): mutual_support when both carry need, else cycle."""
    return [[[a, b], "mutual_support" if "need" in edges[(a, b)] and "need" in edges[(b, a)] else "cycle"]
            for a, b in sorted(edges) if a < b and (b, a) in edges]


def groups_of(pairs):
    """{dish: representative} merging the given pairs (union-find; representative = smallest id)."""
    rep = {}

    def find(i):
        while rep.get(i, i) != i:
            i = rep[i]
        return i
    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb:
            rep[max(ra, rb)] = min(ra, rb)
    return {i: find(i) for i in {x for p in pairs for x in p}}


def first_order(path):
    """--goal built: (the (a before b) edges of the load's --edges pairs record at ``path`` minus its two-way pairs and
    any edge still on a longer cycle (b reaches a), so build_sequence starts acyclic; the record's relative path),
    ([], None) without a record."""
    if not path.is_file():
        return [], None
    edges = {(e["a"], e["b"]) for e in json.loads(path.read_text()).get("edges") or []}
    edges -= {(b, a) for a, b in edges}
    succ = {}
    for a, b in edges:
        succ.setdefault(a, set()).add(b)

    def reach(i):
        seen, todo = set(), [i]
        while todo:
            for j in succ.get(todo.pop(), ()):
                if j not in seen:
                    seen.add(j)
                    todo.append(j)
        return seen
    return sorted((a, b) for a, b in edges if a not in reach(b)), rel(path)


def goal_delta(B, P, entries, centroids, built, targets):
    """--goal built: {per_dish: {id: {lateral_mm, dz_mm, tilt_deg}}, max: the same over dishes} from the previous
    target to the built pose of every dish (B.goal_distance, as the drops judge landing)."""
    per = {}
    for e in entries:
        d = B.goal_distance(centroids[e["kind"]], P.pose_T(built[e["id"]]), P.pose_T(targets[e["id"]]))
        per[e["id"]] = {"lateral_mm": round(d[0] * 1e3, 1), "dz_mm": round(d[1] * 1e3, 1), "tilt_deg": round(d[2], 1)}
    return {"per_dish": per, "max": {k: max(v[k] for v in per.values()) for k in ("lateral_mm", "dz_mm", "tilt_deg")}}


def perturbation(args):
    return {"lateral_mm": args.perturb_mm, "yaw_deg": args.perturb_deg, "seed": args.perturb_seed}


def release_info(args, B):
    """The record's release: {mode: clear} (the lowest clear hover) or the insertion move (2026-10-15 b)."""
    if args.release != "insertion":
        return {"mode": "clear"}
    return {"mode": "insertion", "version": "2026-10-15b", "step_mm": round(B.INSERT_STEP_M * 1e3, 3),
            "above_rim_mm": round(B.INSERT_ABOVE_RIM_M * 1e3, 3), "gap_mm": round(B.INSERT_GAP_M * 1e3, 3),
            "lean_bias_mm": round(B.INSERT_LEAN_BIAS_M * 1e3, 3),
            "move_pose": "a plate upright in its slot (lean 0, the gap's mid x) moved 3 mm toward its packed lean's side; "
                         "a bowl or cup at its packed pose",
            "start": "the move pose raised until its lowest collision point is 20 mm above the rack's rim wire centre, "
                     "its highest collision point capped under the tub ceiling (geometry.py TubCeiling), never below "
                     "the move pose",
            "check": "each 5 mm step down passes the bench checker's 1 mm allowance against all placed dishes and the "
                     "appliance; a placed dish in the way (or a blocked start) blocks the move, the appliance alone "
                     "stops the descent",
            "release": "at rest 20 mm above the lowest clear pose (never above the start), then 150 + 60 ticks"}


def tolerance_of(B, name, limit=.002):
    """(at-goal table, contact-depth limit m, text) of --tolerance (plans/2026-10-16-doubled-tolerance-rerun.md): bench =
    B.AT_GOAL with the gates' contact limit ``limit`` (LIMITS, 2 mm), library = B.AT_GOAL_LIBRARY with
    B.CONTACT_LIBRARY_M (4 mm)."""
    table, contact = (B.AT_GOAL_LIBRARY, B.CONTACT_LIBRARY_M) if name == "library" else (B.AT_GOAL, limit)
    text = ", ".join(f"{k} {v[0] * 1e3:g} mm lateral / {v[1] * 1e3:g} mm dz / {v[2]:g} deg" for k, v in table.items())
    return table, contact, text


def landing_tail(rinfo, contact_m=None):
    """LANDING_TAIL with the record's release (and, given, its contact-depth limit)."""
    tail = LANDING_TAIL if contact_m is None else LANDING_TAIL.replace("< 2 mm", f"< {contact_m * 1e3:g} mm")
    if rinfo["mode"] != "insertion":
        return tail
    return tail.replace("released at the lowest clear hover", (
        f"lowered from {rinfo['above_rim_mm']:g} mm above the rim in {rinfo['step_mm']:g} mm steps (plates upright) "
        f"until the appliance stops it, a placed dish in the way blocking it, released {rinfo['gap_mm']:g} mm above that"))


def perturb(T_cmd, centroid, R_rack, lateral_m, yaw_deg, seed, index):
    """--perturb-*: (``T_cmd`` yawed U(-yaw_deg, yaw_deg) about the world Z axis through the dish's visual ``centroid``,
    then shifted by a point uniform in the disc of radius ``lateral_m`` in the rack plane (``R_rack``'s x/y axes),
    (dx mm, dy mm, yaw deg) as applied); numpy default_rng((seed, index)), so a run is reproducible."""
    import numpy as np
    rng = np.random.default_rng((seed, index))
    r, th, yaw = lateral_m * np.sqrt(rng.random()), 2. * np.pi * rng.random(), float(rng.uniform(-yaw_deg, yaw_deg))
    dx, dy, a = float(r * np.cos(th)), float(r * np.sin(th)), np.radians(yaw)
    Rz = np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
    c = T_cmd[:3, :3] @ np.asarray(centroid, dtype=float) + T_cmd[:3, 3]     # the centroid stays put under the yaw
    T = T_cmd.copy()
    T[:3, :3] = Rz @ T_cmd[:3, :3]
    T[:3, 3] = c + Rz @ (T_cmd[:3, 3] - c) + np.asarray(R_rack, dtype=float) @ np.array([dx, dy, 0.])
    return T, (dx * 1e3, dy * 1e3, yaw)


def parked(pose):
    return float(pose["position_m"][0]) > PARK_X


class Recorder:
    """--edges swaps: every dish's world pose (x y z qx qy qz qw, flat per frame) each TRAJ_EVERY ticks while a segment
    is open, written to <folder>/<name>.json (with ``empty_resets`` the segment restarts whenever every dish is parked:
    only the last build is kept); while ``capture`` is on, each release (a parked dish teleported into the racks) keeps
    the placed dishes' poses at that moment in ``states[dish]``. Wraps backend.tick and backend.set_rigid_pose."""

    def __init__(self, backend, ids, folder):
        self.ids, self.folder, self.frames, self.empty_resets, self.capture, self.states = \
            list(ids), Path(folder), None, False, False, {}
        self.backend = backend
        names = {id(obj): i for i, obj in backend.objects.items()}
        tick, place = backend.tick, backend.set_rigid_pose

        def recorded_tick():
            tick()
            now = backend.previous_frames
            if self.frames is not None and backend.tick_index % TRAJ_EVERY == 0:
                if self.empty_resets and all(parked(now[i]) for i in self.ids):
                    self.frames = []
                self.frames.append([round(float(v), 5) for i in self.ids
                                    for v in (*now[i]["position_m"], *now[i]["quaternion_xyzw"])])

        def recorded_place(obj, pose):
            i, now = names.get(id(obj)), backend.previous_frames
            if self.capture and i is not None and now is not None and parked(now[i]) and not parked(pose):
                self.states[i] = {j: now[j] for j in self.ids if j != i and not parked(now[j])}
            place(obj, pose)
        backend.tick, backend.set_rigid_pose = recorded_tick, recorded_place

    def begin(self, empty_resets=False):
        self.frames, self.empty_resets = [], empty_resets

    def mark(self):
        """The open segment's length (build_sequence's ``trace``: an undone attempt is cut back to it)."""
        return len(self.frames) if self.frames is not None else 0

    def cut(self, n):
        if self.frames is not None and len(self.frames) > n:
            del self.frames[n:]

    def descent(self, dishes, steps, now=None):
        """--release insertion: one frame per descent step appended to the open segment, ``dishes`` at that step's
        poses (``steps`` = [[pose per dish] per step], start first), every other dish where it is ``now`` (default: the
        last tick). The descent is geometric (no ticks), so these frames are its only trace."""
        if self.frames is None:
            return
        now = now or self.backend.previous_frames
        for poses in steps:
            at = {**now, **dict(zip(dishes, poses))}
            self.frames.append([round(float(v), 5) for i in self.ids
                                for v in (*at[i]["position_m"], *at[i]["quaternion_xyzw"])])

    def save(self, name, poses=None, **meta):
        """Write the open segment (or, with ``poses``, that one frame) as <name>.json; returns the file name."""
        frames = self.frames or []
        if poses is not None:
            frames = [[round(float(v), 5) for i in self.ids
                       for v in (*poses[i]["position_m"], *poses[i]["quaternion_xyzw"])]]
        self.frames, self.empty_resets = None, False
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / f"{name}.json").write_text(json.dumps({"ids": self.ids, "every": TRAJ_EVERY, "hz": 120,
                                                             "frames": frames, **meta}) + "\n")
        return f"{name}.json"


def seed_of(*parts):
    """A stable 32-bit seed from strings (the sampler's order and re-posing draws per load)."""
    import hashlib
    return int(hashlib.sha256("|".join(map(str, parts)).encode()).hexdigest()[:8], 16)


class Reposer:
    """build_sequence's ``repose`` for the --release insertion sampler (plans/2026-10-15-reposing-sampler-five-samples.md,
    Kit-free): up to REPOSE_TRIES other poses of the dish's kind from the packing's candidate list (the restricted
    sampler's: frigidaire_hotec_top5 level-0 families, masks and gate bans), drawn in a seeded random order, each FREE as
    the packing defines it, against the dishes placed so far (free_iter: no collision past the 0.8 mm touch with the
    placed dishes at their settled poses, one plate or cup per slot among them, no gate ban; the unplaced dishes' poses
    are not reserved, so a dish whose pose gets taken is re-posed in its turn) and with a CLEAR DESCENT under the
    insertion move against the placed dishes at their settled poses and the appliance (no placed dish blocks it, the
    start is clear). A plate's move depends only on its slot, so a plate gets at most one candidate per slot, never its
    own slot. ``calls`` keeps one line per call (dish, placed, screened, found, seconds). ``chain`` (the lean-side order,
    plans/2026-10-16-lean-side-plate-order.md: entry -> (bank, rank), frigidaire_bench_kit.plate_bank): a plate walks its
    chain instead, only the slots of its own bank past its own along the chain, nearest first (a random variant per slot)."""

    def __init__(self, B, P, frames, seed, chain=None):
        import importlib.util
        spec = importlib.util.spec_from_file_location("top5_repose", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
        T = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(T)    # its private frigidaire_bench copy: TolerantDishSet, nesting allowed (the packing's rules)
        self.T, self.B, self.P, self.frames, self.chain = T, B, P, frames, chain
        self.fam, self.parts, self.points = T.level_families(0)
        self.masks = T.level_masks(self.fam, self.parts, self.points, 0)
        self.bans = T.all_bans()
        self.rng, self.calls = random.Random(seed), []

    def target_move(self, entry):
        """(world target, world move pose) of a rack-local entry."""
        f = self.frames[entry["rack"]]
        target = self.P.world_from_local(f, {"position_m": list(entry["position"]),
                                             "quaternion_xyzw": list(entry["quaternion_xyzw"])})
        return target, self.P.world_from_local(f, self.B.move_local(entry))

    def __call__(self, dish, entries, placed, now):
        B, T, P, t0 = self.B, self.T, self.P, time.monotonic()
        own = entries[dish]
        kind, others = own["kind"], []
        for i in placed:                                     # the placed dishes at their settled poses, rack-local
            e = entries[i]
            local = P.local_from_world(self.frames[e["rack"]], now[i])
            others.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
        dishes = T.TolerantDishSet(self.parts, self.points)
        for e in others:
            dishes.add(e["id"], e)
        used = {(e["rack"], e["slot"]) for e in others}
        checker = B.release_checker()
        appliance = B.appliance_bodies(checker, now)
        bodies = [checker._body(entries[i]["kind"], now[i], i) for i in placed]
        heights = {r: B.rack_heights(now, r) for r in ("LowerRack", "UpperRack")}
        seen, out, screened = ({own["slot"]} if kind == "plate" else set()), [], 0
        index = list(range(len(self.fam[kind])))
        self.rng.shuffle(index)
        walk = self.chain is not None and kind == "plate"
        if walk:                                             # its bank's slots past its own, nearest first (stable sort:
            bank, rank = self.chain(own)                     # the shuffled variants stay random within a slot)
            ranks = {k: self.chain(self.fam[kind][k][0]) for k in index}
            index = sorted((k for k in index if ranks[k][0] == bank and ranks[k][1] > rank), key=lambda k: ranks[k][1])
        for k in index:
            if len(out) >= REPOSE_TRIES:
                break
            c = self.fam[kind][k][0]
            if (not self.masks[kind][k] or (kind != "bowl" and (c["rack"], c["slot"]) in used)
                    or (kind == "plate" and c["slot"] in seen) or B.cand_key(c) == B.cand_key(own)
                    or T.B.banned(c, others, self.bans) or dishes.collides(c, skip=(dish,))):
                continue
            seen.add(c["slot"])
            entry = T.B.entry_of(dish, c)
            target, move = self.target_move(entry)
            up, rim_z, top_z = heights[c["rack"]]
            screened += 1
            if B.insertion_release(checker, [(kind, move, dish)], bodies, appliance, up, rim_z, top_z)["poses"] is None:
                continue
            out.append({"entry": entry, "target": target, "move": move})
        self.calls.append({"dish": dish, "placed": len(placed), "screened": screened, "found": len(out),
                           "seconds": round(time.monotonic() - t0, 2), **({"chain": True} if walk else {})})
        print(f"  repose {dish} ({len(placed)} placed{', chain walk' if walk else ''}): {len(out)} clear of {screened} free poses screened "
              f"({time.monotonic() - t0:.1f} s): " + ", ".join(f"{o['entry']['slot']} {o['entry']['variant']}" for o in out),
              flush=True)
        return out


def any_of(e, edge):
    """The dishes any one of which, placed before b, obeys the edge e = (a, b): [a], or a conditional edge's any_of
    (plans/2026-10-15-reposing-sampler-five-samples.md: b needs one of {a, C...})."""
    return list(edge.get("any_of") or [e[0]]) if isinstance(edge, dict) else [e[0]]


def obeying_order(ids, edges, rng):
    """A random order obeying every edge in ``edges`` ({(a, b): edge} or [(a, b)]; a conditional edge is obeyed when any
    member precedes b): the next dish drawn uniformly from the ready ones (never none: the witness obeys every edge)."""
    req = {i: [] for i in ids}
    for e in sorted(edges):
        req[e[1]].append(any_of(e, edges[e] if isinstance(edges, dict) else None))
    placed, out, left = set(), [], list(ids)
    while left:
        ready = sorted(i for i in left if all(any(m in placed for m in ms) for ms in req[i]))
        if not ready:
            raise ValueError("obeying_order: the edges admit no order")
        i = ready[rng.randrange(len(ready))]
        left.remove(i)
        placed.add(i)
        out.append(i)
    return out


def violate(order, a, b):
    """``order`` with b moved to just before a (the edge a before b reversed, everything else kept)."""
    out = [i for i in order if i != b]
    out.insert(out.index(a), b)
    return out


def predict(order, edges):
    """What the graph says of ``order``: its violated edges (a conditional edge: no member before b), the dish it fails
    at (per violated edge the edge's failing dish, b for a step-1 failure and a for a step-2 one; the earliest in
    ``order``), the edges naming that dish, and the build's stop (two dishes after it; None = build all)."""
    pos = {i: n for n, i in enumerate(order)}
    broken = sorted(e for e, v in edges.items() if all(pos[m] > pos[e[1]] for m in any_of(e, v)))
    first = min((pos[edges[e]["fails"]] for e in broken), default=None)
    dish = None if first is None else order[first]
    return {"violated": [list(e) for e in broken], "predicted": dish, "predicted_position": first,
            "predicted_edges": [list(e) for e in broken if edges[e]["fails"] == dish],
            "stop": None if first is None else first + 3}


def library_orders(ids, witness, edges, n, seed=LIBRARY_SEED):
    """The evidence library (module docstring): the witness, one violating order per edge, ``n`` random obeying and
    ``n`` random single-violation orders, each with predict()'s fields and outcome None (filled by --merge-library)."""
    rng = random.Random(seed)
    E = sorted(edges)
    lib = [("witness", list(witness), None)] + [("edge_violation", violate(witness, a, b), [a, b]) for a, b in E]
    lib += [("obeying", obeying_order(ids, edges, rng), None) for _ in range(n)]
    for _ in range(n if E else 0):
        a, b = rng.choice(E)
        lib.append(("random_violation", violate(obeying_order(ids, edges, rng), a, b), [a, b]))
    return [{"index": k, "kind": kind, "order": order, "edge": edge, **predict(order, edges), "outcome": None}
            for k, (kind, order, edge) in enumerate(lib)]


def judge_order(lib, edges):
    """(prediction under ``edges``, as_predicted, still_builds) of one library order under the validity of
    plans/2026-10-16-backtracking-sampler-learning-loop.md: as_predicted None while it is not built; an order that obeys
    every edge (predicted None) is as predicted when no dish failed (built all 24, or to its stop), an order that breaks
    one when it failed at ANY dish: the dish it failed at (failed_at) and the predicted dish (predicted_at) are both
    recorded and shown, never required to match (a cascade in a new context is physics, not a wrong edge); still_builds
    = it breaks an edge and nothing failed up to its stop (the conditional refinement's case). An order is judged by the
    graph it was drawn from: the edges learned up to its round (``round``: 0 for the first library, r for round r's fresh
    orders; a learned edge carries the round that added it)."""
    rnd = lib.get("round") or 0
    o, pr = lib.get("outcome"), predict(lib["order"], {k: e for k, e in edges.items() if (e.get("round") or 0) <= rnd})
    if o is None:
        return pr, None, False
    failed = o.get("failed_dish")
    ok = (failed is None) if pr["predicted"] is None else (failed is not None)
    return pr, ok, pr["predicted"] is not None and failed is None


def verdict(dep):
    """The sample's validity (plans/2026-10-16-backtracking-sampler-learning-loop.md): every library order that obeys
    the graph lands all 24 and every order that breaks an edge fails somewhere. {stage: pending (orders not built yet) |
    refine (a violation still built: its edges go to --refine) | learn (an obeying order failed that has not taught an
    edge yet: the learning loop, see learn_round) | valid | invalid, reason, counts, rows [{index, kind, round,
    predicted_at, failed_at, as_predicted, taught}], refine [{index, edges}], failed_obeying, still_building}. Each order
    is judged by the graph of its round (judge_order); an obeying order that failed and taught a learned edge is re-checked
    by the next round's fresh orders, so the sample is valid when the last round passes. Invalid: the refinement failed or
    a violation still built after it, a failed obeying order no edge along the witness order excludes, or an obeying
    order failed in the last of learning.max_rounds rounds."""
    E = {(e["a"], e["b"]): e for e in dep["edges"]}
    rows = []
    for lib in dep["library"]:
        pr, ok, still = judge_order(lib, E)
        o = lib.get("outcome") or {}
        rows.append({"index": lib["index"], "kind": lib["kind"], "round": lib.get("round"), "ok": ok, "still": still,
                     "obeying": pr["predicted"] is None, "predicted_at": pr["predicted"], "failed_at": o.get("failed_dish"),
                     "as_predicted": ok, "predicted_edges": pr["predicted_edges"], "outcome": o.get("status")})
    built = [r for r in rows if r["ok"] is not None]
    bad = [r for r in built if not r["ok"]]
    broke = [r for r in bad if r["obeying"]]
    still = [r for r in bad if r["still"]]
    ref, learn = dep.get("refinement"), dep.get("learning") or {}
    rounds, cap = len(learn.get("rounds") or []), int(learn.get("max_rounds") or 0)
    taught = {x["order"]: x["edge"] for r in learn.get("rounds") or [] for x in r["diagnoses"] if x["edge"]}
    for r in rows:
        r["taught"] = taught.get(r["index"])
    broke = [r for r in broke if r["taught"] is None]              # not yet learned from
    out = {"orders": len(rows), "built": len(built), "as_predicted": len(built) - len(bad),
           "at_predicted_dish": sum(r["failed_at"] is not None and r["failed_at"] == r["predicted_at"] for r in built),
           "mismatches": bad, "refine": [], "refined": ref is not None, "rounds": rounds, "taught": len(taught),
           "failed_obeying": [r["index"] for r in broke], "still_building": [r["index"] for r in still],
           "rows": [{k: r[k] for k in ("index", "kind", "round", "predicted_at", "failed_at", "as_predicted", "taught")}
                    for r in rows]}

    def said(r):
        return (f"order {r['index']} ({r['kind']}{', round ' + str(r['round']) if r['round'] else ''}): "
                + (f"obeys every edge, Isaac failed at {r['failed_at']}" if r["obeying"] else
                   f"breaks {', '.join('>'.join(e) for e in r['predicted_edges'])}, Isaac "
                   + ("built through its stop" if r["outcome"] == "stopped" else "landed every dish")))
    wit = [r for r in rows if r["kind"] == "witness" and r["ok"] is False and r["obeying"]]
    why = [f"the witness order did not rebuild in the library: it failed at {r['failed_at']}" for r in wit] \
        + ([ref["reason"]] if ref is not None and ref.get("ok") is False else []) \
        + ([said(r) + " after the refinement" for r in still] if ref is not None else []) \
        + ([learn["unlearnable"]] if learn.get("unlearnable") else []) \
        + ([said(r) + f" after {rounds} learning round(s)" for r in broke] if rounds >= cap and broke else [])
    if why:
        return {**out, "stage": "invalid", "reason": "; ".join(why)}
    if len(built) < len(rows):
        return {**out, "stage": "pending", "reason": f"{len(rows) - len(built)} orders not built"}
    if still:
        out["refine"] = [{"index": r["index"], "edges": r["predicted_edges"]} for r in still]
        return {**out, "stage": "refine", "reason": f"{len(still)} violation(s) still build: orders "
                + ", ".join(str(r["index"]) for r in still)}
    if broke:
        return {**out, "stage": "learn", "reason": f"{len(broke)} obeying order(s) failed: "
                + "; ".join(said(r) for r in broke)}
    return {**out, "stage": "valid",
            "reason": f"{len(rows)} orders: {sum(r['obeying'] and r['ok'] for r in rows)} obeying orders built all 24, "
                      f"{sum(not r['obeying'] for r in rows)} violations failed ({out['at_predicted_dish']} at the "
                      f"predicted dish)" + (" (refined graph)" if ref is not None else "")
                      + (f"; {len(taught)} obeying order(s) failed and taught an edge, {rounds} learning round(s), the "
                         f"last one passed" if rounds else "")}


def diagnose(lib, dep):
    """Candidate learned edges (a, b, why) for an obeying library order that failed at X, in the order of the plan's
    diagnosis (plans/2026-10-16-backtracking-sampler-learning-loop.md): the dishes X hit -- it moved them, or they
    blocked its descent -- go after X (X, Y); the dishes X needed before it -- the witness placed them before X and this
    order had not: what X rests on in the witness's final contacts first, then the rest nearest first (built-goal
    centres; first version: only its 20 mm neighbours) -- go before X (D, X); with none named, the dish placed just
    before X in this order goes after X
    (X, Z), walking further back while the witness itself placed that dish before X. Every candidate is broken by this
    order, so the next obeying orders avoid what failed."""
    order, o = lib["order"], lib["outcome"]
    x, fpos = o["failed_dish"], o["failed_position"]
    prefix = order[:fpos]
    step = next((s for s in reversed(o.get("steps") or []) if s.get("id") == x), {})
    out = []
    hit = list(o.get("moved") or []) if o["status"] == "moved" else \
        [y for y in step.get("blockers") or [] if y in prefix] if step.get("blocked") else []
    for y in hit:
        out.append((x, y, f"{x} {'moved' if o['status'] == 'moved' else 'was blocked by'} {y}, placed before it"))
    if o["status"] == "landing" and not step.get("blocked"):
        support = {tuple(p) for p in dep["witness"].get("support") or []}
        w = {i: n for n, i in enumerate(dep["witness"]["order"])}
        goal = dep.get("targets_world") or {}

        def gap(d):                                          # built-goal centre distance [mm] (unknown: last)
            a, b = goal.get(d), goal.get(x)
            return math.dist(a["position_m"], b["position_m"]) * 1e3 if a and b else 1e9
        # what X had before it in the witness and lacks here: what it rests on first, then the rest nearest first
        # (2026-10-16: r07_00012's bowls missed context 200-400 mm away, never their 20 mm neighbours)
        lacked = sorted((d for d in dep["witness"]["order"][:w[x]] if d not in prefix), key=lambda d: ((d, x) not in support, gap(d)))
        for d in lacked:
            out.append((d, x, f"{x} did not land; it rests on {d} in the witness" if (d, x) in support else
                        f"{x} did not land; {d} ({gap(d):.0f} mm away) went in before it in the witness, not here"))
    w = {i: n for n, i in enumerate(dep["witness"]["order"])}
    for z in reversed(prefix):
        if w[z] > w[x]:
            out.append((x, z, f"{z} went in just before {x} in this order (the latest such dish the witness placed "
                              f"after {x})"))
            break
    return out


def learn_round(dep):
    """One round of the learning loop (plans/2026-10-16-backtracking-sampler-learning-loop.md): every obeying library
    order that failed and has not taught an edge yet (verdict failed_obeying) adds one learned edge, reason 'learned',
    the first diagnose() candidate that is not an edge already and runs along the witness order (an edge against it would
    make the order that built the goal a violation, a cycle with the witness: it is dropped and reported, and the next
    candidate is taken), then FRESH_ORDERS random orders obeying the graph join the library (kind learned_obeying, round
    r) for --order-index. A learned edge carries its round; earlier orders keep being judged by the graph they were drawn
    from (judge_order). A failed obeying order with no candidate left makes the sample invalid (learning.unlearnable).
    Kit-free."""
    E = {(e["a"], e["b"]): e for e in dep["edges"]}
    v = verdict(dep)
    learn = dep["learning"]
    rnd = len(learn["rounds"]) + 1
    w = {i: n for n, i in enumerate(dep["witness"]["order"])}
    support = {tuple(p) for p in dep["witness"].get("support") or []}
    seen = {tuple(p) for p in dep["witness"].get("learned") or []}
    added, unlearnable, report = {}, [], []
    for idx in v["failed_obeying"]:
        lib = dep["library"][idx]
        x, rejected, pick = lib["outcome"]["failed_dish"], [], None
        for a, b, why in diagnose(lib, dep):
            if (a, b) in added:                                    # learned from another order of this round
                pick = (a, b, why)
                break
            if (a, b) in E:
                rejected.append([a, b, why, "already an edge (conditional, obeyed through another dish)"])
                continue
            if w[a] > w[b]:
                rejected.append([a, b, why, f"against the witness (it placed {b} before {a} and built)"])
                continue
            pick = (a, b, why)
            break
        report.append({"order": idx, "failed_at": x, "status": lib["outcome"]["status"],
                       "edge": list(pick[:2]) if pick else None, "why": pick[2] if pick else None, "rejected": rejected})
        if pick is None:
            unlearnable.append(f"order {idx} ({lib['kind']}) obeys every edge and failed at {x}; no edge along the "
                               f"witness order excludes it ({len(rejected)} candidate(s) rejected)")
        elif pick[:2] not in added and pick[:2] not in E:
            a, b = pick[:2]
            added[(a, b)] = {"a": a, "b": b, "reason": "learned", "step": 2 if a == x else 1, "fails": x,
                             "support": (a, b) in support, "learned": (a, b) in seen or (b, a) in seen,
                             "detail": f"obeying order {idx} failed at {x}: {pick[2]}", "placed_before": None,
                             "affected": list(lib["outcome"].get("moved") or []),
                             "margin": {"min": None, "source": "learning", "margins": {}}, "swap": None, "traj": [],
                             "round": rnd, "from_order": idx}
    E.update(added)
    fresh = []
    if not unlearnable:
        ids = list(dep["witness"]["order"])
        rng, n0 = random.Random(LIBRARY_SEED + 100 + rnd), len(dep["library"])
        for j in range(FRESH_ORDERS):
            o = obeying_order(ids, E, rng)
            fresh.append({"index": n0 + j, "kind": "learned_obeying", "round": rnd, "order": o, "edge": None,
                          **predict(o, E), "outcome": None})
        dep["library"] += fresh
    dep["edges"] = [E[x] for x in sorted(E)]
    learn["rounds"].append({"round": rnd, "from_orders": v["failed_obeying"], "edges": [[a, b] for a, b in added],
                            "diagnoses": report, "orders": [f["index"] for f in fresh], "seed": LIBRARY_SEED + 100 + rnd})
    learn["edges_added"] = sum(len(r["edges"]) for r in learn["rounds"])
    if unlearnable:
        learn["unlearnable"] = "; ".join(unlearnable)
    return added, unlearnable


def library_outcome(lib, status, build):
    """One built library order: status (ok | stopped | landing | moved), the failing dish and its position, the dishes
    placed, and whether it is what the graph predicted (built-or-failed: an order obeying every edge built, one breaking
    an edge failed somewhere; predicted_at and failed_at are both kept)."""
    failed = build["dish"] if status in ("landing", "moved") else None
    placed = len(build["steps"]) - (1 if failed else 0)
    match = failed is None if lib["predicted"] is None else failed is not None
    return {"status": status, "failed_dish": failed, "failed_position": lib["order"].index(failed) if failed else None,
            "moved": build["moved"], "placed": placed, "matches_prediction": match, "predicted_at": lib["predicted"],
            "failed_at": failed, "steps": build["steps"]}


def pairs(args, source, entries, gate, out_path, kit_dir, report, started):
    """--edges pairs: the v20 ordered-pair test (module docstring), its graph and its certifying (or, for mutual
    support, diagnostic) build; with --goal built the goal is first re-settled one dish at a time and the tests run on
    the built poses."""
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim import rearrange as R
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS

    snapshot, frames, targets, landing, rule, targets_source, entries_settled = goal_setup(args, entries, gate, B, K, P)
    ids = [e["id"] for e in entries]
    by_id = {e["id"]: e for e in entries}
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    backend, resettle, kit_resettle, head, rec, witness = None, {}, {}, "", None, None
    rinfo = release_info(args, B)                                   # --release: the clear hover or the insertion move
    release = args.release
    tail = landing_tail(rinfo)
    moves, states, dep0 = {}, {}, None                              # the insertion move's poses; the swap states
    tol_name = args.tolerance                                       # --tolerance: the swaps' and the library's table
    table, climit, tol_text = tolerance_of(B, tol_name, LIMITS["peak_penetration_m"])
    if args.refine:                                                 # --refine: the record's goal, moves and states
        dep0 = json.loads(out_path.read_text())
        if (dep0.get("verdict") or {}).get("stage") != "refine":
            raise Refused(f"{rel(out_path)} has no verdict 'refine' (run --merge-library first)")
        tol_name = dep0.get("tolerance") or "bench"                # the record's tolerance, whatever --tolerance says
        table, climit, tol_text = tolerance_of(B, tol_name, LIMITS["peak_penetration_m"])
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir / "refine_scene", report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
        targets, witness, rinfo = dep0["targets_world"], dep0["witness"], dep0["release"]
        moves.update(dep0.get("moves_world") or {})
        by_id = {e["id"]: e for e in dep0.get("entries_chosen") or entries}     # a re-posed dish may change rack
        states.update(json.loads((kit_dir / "states.json").read_text()))
        landing = lambda kind, dist, dev: B.within_goal(kind, dist, table)            # noqa: E731
        rule, targets_source, entries_settled = dep0["landing_rule"], "sequential_build", dep0["entries_settled"]
        rec = Recorder(backend, ids, kit_dir / "traj")
    elif args.goal == "built":                                      # the goal re-settled one dish at a time first
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
        if args.edges in ("swaps", "physical"):     # the sampler: nothing given; the last build's ticks kept
            initial, original = [], None
            rec = Recorder(backend, ids, kit_dir / "traj")
            rec.capture = True
            rec.begin(empty_resets=True)
        else:
            initial, original = first_order(pairs_folder(args) / f"{args.load}.json")
        own = (f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the commanded pose (the {targets_source} "
               f"pose + {B.GOAL_HOVER_M * 1e3:.0f} mm), {LANDING_TAIL}" if release == "clear" else
               f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the {targets_source} pose (the target, not "
               f"the release pose), {tail}")
        print(f"resettle {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes one at a time, "
              f"{len(initial)} initial constraints from {original or 'no pairs record'}; landing = {own}", flush=True)
        t_resettle = time.monotonic()
        sampler, chosen = None, entries
        if args.edges in ("swaps", "physical") and release == "insertion":       # the re-posing sampler (2026-10-15 b)
            seed = seed_of("reposing", args.load)
            order0 = list(ids)
            random.Random(seed).shuffle(order0)
            chains, chain = None, None
            if args.plate_order == "lean-side":   # plans/2026-10-16-lean-side-plate-order.md: the same permutation,
                order0, chains = K.chain_order(order0, by_id, B.lean_side)   # each bank's plates refilled in chain order
                chain = lambda e: K.plate_bank(e, B.lean_side)                          # noqa: E731
            moves = {i: P.world_from_local(frames[by_id[i]["rack"]], B.move_local(by_id[i])) for i in ids}
            reposer = Reposer(B, P, frames, seed + 1, chain=chain)
            links = ", ".join(f"{i} {c['bank']} {c['index']}" for i, c in (chains or {}).items())
            print(f"re-posing sampler: candidates ready ({time.monotonic() - t_resettle:.0f} s); {args.plate_order} order "
                  f"{' '.join(order0)}" + (f"; chains {links}" if chains else ""), flush=True)
            built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids,
                                                lambda kind, dist, dev: dev <= K.DEV_MAX_M, release=release,
                                                on_path=rec.descent, moves=moves, order=order0, repose=reposer, trace=rec,
                                                chains=chains, backtrack=args.backtrack)
            for a in record["attempts"]:
                s = a["step"]
                print(f"  attempt {a['dish']} try {a['try']} {a['slot']} {a['variant']} ({a['placed']} placed)"
                      + (f" [{a['phase']}]" if a.get("phase") else "") + f": "
                      f"{a['status']}" + (f" blocked ({s.get('blocked')}) at {s.get('blocked_lift_mm')} mm by "
                                          f"{s.get('blockers')}" if s.get("no_clear_release") else
                                          f" dev {s.get('settle_dev_mm')} mm contact {s.get('contact_mm')} mm stop "
                                          f"{s.get('stop')} {s.get('lowest_lift_mm')} mm moved {a['moved'] or '-'}"
                                          + (f" overlap {s['overlap']}" if s.get("overlap") else "")),
                      flush=True)
            for b in record["backtracks"]:
                print(f"  backtrack {b['n']} of {b['dish']} ({b['placed']} placed): undid {b['undone']} -> {b['outcome']}"
                      + (f", back {b['back']}, deferred {b['deferred']}" if b["outcome"] == "placed" else "")
                      + (f", moved {b['moved']}" if b.get("moved") else ""), flush=True)
            moves, chosen = record["moves"], [record["entries"][i] for i in ids]
            by_id = {e["id"]: e for e in chosen}                   # the swaps use each dish's chosen rack and move
            sampler = {"seed": seed, "order": order0, "plate_order": args.plate_order, "attempts": record["attempts"],
                       "reposed": record["reposed"], "repose_calls": reposer.calls, "entries_chosen": chosen,
                       "backtrack": args.backtrack, "backtracks": record["backtracks"]}
            if built == "ok" and any(b["outcome"] == "placed" for b in record["backtracks"]):
                # A backtrack leaves dishes that went in while an undone dish was in the racks, so the final order is
                # not how they went in (r08_00034's witness did not rebuild, 2026-10-16): the witness order is built
                # once more from empty racks (fixed order, the sampler's landing rule) and that build is the goal, its
                # releases the swap states and its frames the witness clip; a dish that fails there fails the sample.
                rec.states.clear()
                built, rb, _ = K.build_sequence(backend, B, P, by_id, record["targets"], centroids,
                                                lambda kind, dist, dev: dev <= K.DEV_MAX_M, fixed=list(record["order"]),
                                                release=release, on_path=rec.descent, moves=moves)
                last = rb["builds"][-1]
                sampler["rebuild"] = {"status": built, "dish": last["dish"], "moved": last["moved"],
                                      "placed": len(last["steps"]) - (1 if last["dish"] else 0), "reason": rb["reason"]}
                print(f"witness rebuilt from empty racks after {sum(b['outcome'] == 'placed' for b in record['backtracks'])} "
                      f"placed backtrack(s): {built} {last['dish'] or ''} {last['moved'] or ''} ({rb['reason']})", flush=True)
                record["builds"] = record["builds"] + rb["builds"]
                if built == "ok":
                    record.update(poses_world=rb["poses_world"], support=rb["support"], order=rb["order"])
                else:
                    record["reason"] = (f"the sampler built all {len(ids)} with backtracking, but its order did not "
                                        f"rebuild from empty racks: {rb['reason']}")
            if chains:                    # per plate: its bank, chain index, slot planned, slot and place in its bank built
                ok, per = {a["dish"] for a in record["attempts"] if a["status"] == "ok"}, {}
                for i in record["builds"][-1]["order"]:             # the placed dishes in placement order
                    if i in chains and i in ok:
                        chains[i]["placed_index"] = per.get(chains[i]["bank"], 0)
                        per[chains[i]["bank"]] = chains[i]["placed_index"] + 1
                for i, c in chains.items():
                    c["placed_slot"] = record["entries"][i]["slot"] if "placed_index" in c else None
                sampler["chains"] = chains
        else:
            built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids,
                                                lambda kind, dist, dev: dev <= K.DEV_MAX_M, initial=initial,
                                                release=release, on_path=rec.descent if rec is not None else None)
        for k, b in enumerate(record["builds"], 1):
            print(f"resettle build {k}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)
        resettle = {"goal_source": "sequential_build", "goal_delta": None, "resettle_builds": record["builds"],
                    "resettle_initial": [list(x) for x in initial], "resettle_rule": own, "original": original}
        if sampler is not None:
            resettle.update(reposing=sampler, moves_world=moves, entries_chosen=chosen)
        kit_resettle = {"resettle_wall_seconds": round(time.monotonic() - t_resettle, 1),
                        "resettle_builds_run": len(record["builds"])}
        if rec is not None:
            rec.capture = False
            last = record["builds"][-1]
            witness = {"order": record.get("order") or last["order"], "status": last["status"],
                       "learned": [list(x) for x in record["learned"]],
                       "support": [list(x) for x in record.get("support", [])], "steps": last["steps"],
                       "traj": rec.save("witness", order=last["order"], status=last["status"])}
            if sampler is not None:       # the order rule, and per plate its bank and chain index (the page's chains)
                witness.update(plate_order=sampler["plate_order"], chains=sampler.get("chains"))
            resettle["witness"] = witness
            states = rec.states
            (kit_dir / "states.json").write_text(json.dumps(states) + "\n")
        if built != "ok":
            return gate_failed(args, source, entries, gate, out_path, report, "resettle_failed",
                               "the goal did not build one dish at a time: " + record["reason"],
                               targets_source=targets_source, targets_world=targets, entries_settled=entries_settled,
                               frames=frames, failed_dish=record["builds"][-1]["dish"], **resettle, release=rinfo,
                               kit={"wall_seconds": round(time.monotonic() - started, 1), **kit_resettle})
        resettle["goal_delta"] = goal_delta(B, P, entries, centroids, record["poses_world"],
                                            record["targets"] if sampler is not None else targets)
        top = resettle["goal_delta"]["max"]
        head = (("release insertion " if release != "clear" else "")
                + f"resettle builds {len(record['builds'])} goal delta max lateral {top['lateral_mm']} "
                f"dz {top['dz_mm']} mm tilt {top['tilt_deg']} deg ")
        print(f"goal built from the {targets_source} pose: {head}({kit_resettle['resettle_wall_seconds']:.0f} s)",
              flush=True)
        targets, targets_source = record["poses_world"], "sequential_build"
        landing = lambda kind, dist, dev: B.within_goal(kind, dist, table)            # noqa: E731
        rule = (f"at-goal tolerance of the sequentially built pose ({tol_name}: {tol_text}), "
                f"{landing_tail(rinfo, climit)}")
        entries_settled = local_entries(P, chosen, frames, targets)
        if args.edges == "physical":           # plans/2026-10-16-physical-cause-graph.md: the sampler's build only; the
            dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "procedure": "physical",  # gate
                   "status": "ok", "reason": f"goal built one dish at a time in {len(resettle['resettle_builds'])} "  # and
                   "build(s) by the re-posing sampler", "release": rinfo, **resettle, "entries": entries,       # --certify
                   "targets_source": "sequential_build", "targets_world": targets, "entries_settled": entries_settled,
                   "frames": frames, "landing_rule": rule, "tolerance": tol_name, "gate": None, "certify": None,
                   "valid": None, "kit": {"wall_seconds": round(time.monotonic() - started, 1), **kit_resettle},
                   "generated_utc": datetime.now(timezone.utc).isoformat()}
            write_record(args, out_path, dep)
            report.update(result="PASS", status="ok", dependency=rel(out_path), reason=head + "status ok (sampler only)")
            return
    pair_list = candidate_pairs(B.bench_checker(), entries, targets)
    runs = [(i, []) for i in ids]                                   # L(B), then R(A, B) both ways per candidate pair
    for a, b, _ in pair_list:
        runs += [(b, [a]), (a, [b])]
    on = bool(args.perturb_mm or args.perturb_deg)                 # --perturb-*: the L and R drops
    shaken = (f" perturb {args.perturb_mm:g} mm {args.perturb_deg:g} deg seed {args.perturb_seed}" if on else "")
    seeds = tuple(10 * args.perturb_seed + s for s in RERUN_SEEDS)
    if args.edges == "pairs":
        print(f"pairs {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes, {len(pair_list)} "
              f"candidate pairs within {PAIR_M * 1e3:.0f} mm, {len(runs)} drops; landing = {rule}; release = lowest "
              f"clear hover ({B.RELEASE_RULE}, {B.RELEASE_STEP_M * 1e3:g} mm steps, cap {B.RELEASE_CAP_M * 1e3:g} mm); "
              f"marginal reruns {RERUN_MM:g} mm {RERUN_DEG:g} deg seeds {list(seeds)}"
              + (f";{shaken} (offset uniform in the disc, yaw uniform)" if on else ""), flush=True)
    if backend is None:
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
    park = {i: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, i in enumerate(by_id)}
    limit = climit                                                  # --tolerance: the contact-depth limit
    checker = B.release_checker()
    kind = {i: by_id[i]["kind"] for i in ids}
    goals = {i: P.pose_T(targets[i]) for i in ids}
    tol = {i: table[kind[i]] for i in ids}
    rack_R = {i: P.pose_T(frames[by_id[i]["rack"]])[:3, :3] for i in ids}
    state = {}

    def gd(i, T):
        return B.goal_distance(centroids[kind[i]], T, goals[i])

    def mm3(d):
        return [round(d[0] * 1e3, 1), round(d[1] * 1e3, 1), round(d[2], 1)]

    def context(absent0, at=None):
        """Park ``absent0``, every other dish at its target (or at its pose in ``at``, --edges swaps), re-settle 150
        ticks; while a context dish ends outside its at-goal tolerance of that pose (fallen), park it too and re-settle
        again (FALL_ROUNDS at most). (poses, absent, rounds)."""
        absent, rounds, at = set(absent0), [], at or {}
        ref = {i: P.pose_T(p) for i, p in at.items()}
        while True:
            for i in ids:
                backend.set_rigid_pose(backend.objects[i], park[i] if i in absent else at.get(i, targets[i]))
            for _ in range(K.SETTLE_TICKS):
                backend.tick()
            now = backend.poses()
            drift = {i: B.goal_distance(centroids[kind[i]], P.pose_T(now[i]), ref.get(i, goals[i]))
                     for i in ids if i not in absent}
            fallen = sorted(i for i, d in drift.items() if not B.within_goal(kind[i], d, table))
            rounds.append({"parked": sorted(absent - set(absent0)), "fallen": fallen,
                           "drift": {i: mm3(d) for i, d in drift.items()}})
            if not fallen or len(rounds) >= FALL_ROUNDS:
                return now, absent, rounds
            absent |= set(fallen)

    def drop(dish, removed, index, seed=None, mm=0., deg=0., at=None):
        """One drop of ``dish`` with ``removed`` absent (module docstring), released at its lowest clear hover above
        the target (with ``seed``: that release pose perturbed by ``mm`` / ``deg``, as --perturb-* does). --edges swaps
        (``at`` = the context dishes' start poses): disturbed = build_sequence's rule, the drop's ticks are recorded
        (``traj``) and the dish's final pose kept (``pose``). --release insertion: the insertion move toward the dish's
        move pose (frigidaire_bench.insertion_release; a plate upright in its slot), released 20 mm above where the
        appliance stops it, its descent the first frames of ``traj``; a descent a context dish blocks (or a blocked
        start) = not landed (no_clear_release, blocked, blocked_step, blocked_lift_mm, blockers)."""
        now, absent, rounds = context({dish, *removed}, at)
        ctx = [i for i in ids if i not in absent]
        pre = {i: P.pose_T(now[i]) for i in ctx}
        if "appliance" not in state:
            state["appliance"] = B.appliance_bodies(checker, now)
        off = (0., 0., 0.)
        close = (B.near_poses({i: now[i] for i in ctx}, kind, targets[dish]["position_m"]) if release == "clear"
                 else {i: now[i] for i in ctx})                     # the insertion path reaches further
        bodies = [checker._body(kind[i], pose, i) for i, pose in close.items()]
        path, lifts, up, mv = {}, None, None, moves.get(dish, targets[dish])
        if release == "clear":
            rpose, hover, blockers = B.clear_release(checker, [(kind[dish], targets[dish], dish)], bodies,
                                                   state["appliance"], rack_R[dish][:, 2])
        else:
            up, rim_z, top_z = B.rack_heights(now, by_id[dish]["rack"])
            m = B.insertion_release(checker, [(kind[dish], mv, dish)], bodies, state["appliance"], up, rim_z, top_z)
            rpose, hover, blockers, path, lifts = m["poses"], m["lift"], m["blockers"], B.path_info(m), B.shown_lifts(m)
        if rpose is not None and seed is not None:                    # the release pose perturbed (the --perturb-*
            T, off = perturb(P.pose_T(rpose[0]), centroids[kind[dish]], rack_R[dish], mm * 1e-3, deg, seed, index)
            rpose = [P.pose_dict(T)]                                  # mechanism); landing judged against the target
        out = {"dish": dish, "removed": list(removed), "index": index, "seed": seed,
               "parked_fallen": sorted(absent - {dish, *removed}), "context_rounds": rounds,
               "context_valid": not rounds[-1]["fallen"],
               "context_drift_mm": round(max((float(np.linalg.norm(pre[i][:3, 3] - goals[i][:3, 3])) * 1e3 for i in ctx),
                                             default=0.), 1),
               "dx_mm": round(off[0], 2), "dy_mm": round(off[1], 2), "yaw_deg": round(off[2], 2), **path}
        name = f"t{index:03d}" + (f"_s{seed}" if seed is not None else "")
        descent = [[B.lifted(mv, h, up)] for h in lifts] if lifts else []
        if rpose is None:                                             # no clear release: not landed, nothing dropped
            out.update(landed=False, no_clear_release=True, hover_mm=None, blockers=blockers, lateral_mm=None,
                       dz_mm=None, tilt_deg=None, contact_mm=None, disturbed=[], moved={}, margins={}, marginal=[])
            if rec is not None and not descent:                     # the state it could not be released into
                out["traj"] = rec.save(name, poses=now, dish=dish, no_clear_release=True)
            elif rec is not None:                                   # the descent until the blocked step
                rec.begin()
                rec.descent([dish], descent, now)
                out["traj"] = rec.save(name, dish=dish, no_clear_release=True, path_blocked=True, blockers=blockers,
                                       **path)
            return out
        if rec is not None:
            rec.begin()
            rec.descent([dish], descent, now)                       # (insertion) the descent, then the fall
        backend.set_rigid_pose(backend.objects[dish], rpose[0])
        for _ in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
        if rec is not None:
            out["traj"] = rec.save(name, dish=dish, release=rpose[0], descent_frames=len(descent))
        now = backend.poses()
        post = {i: P.pose_T(now[i]) for i in [*ctx, dish]}
        dist = gd(dish, post[dish])
        dev = float(np.linalg.norm(post[dish][:3, 3] - np.asarray(rpose[0]["position_m"], dtype=float)))
        deep = float(max((d for name, d in backend.latest_contact["depths_m"].items() if dish in name.split("|")),
                         default=0.))
        after = {i: gd(i, post[i]) for i in ctx}
        moved = {}
        for i in ctx:
            dp, dr = float(np.linalg.norm(post[i][:3, 3] - pre[i][:3, 3])) * 1e3, R.rot_angle_deg(pre[i], post[i])
            if dp > MOVED_MM or dr > MOVED_DEG:
                moved[i] = {"moved_mm": round(dp, 1), "moved_deg": round(dr, 1), "goal": mm3(after[i])}
        t = tol[dish]
        margins = {"lateral": (t[0] - dist[0]) / t[0], "height": (t[1] - dist[1]) / t[1],
                   "tilt": (t[2] - dist[2]) / t[2], "contact": (limit - deep) / limit}
        near = [(tol[i][c] - after[i][c]) / tol[i][c] for i in moved for c in range(3)]
        margins["disturbance"] = min(near, key=abs) if near else None
        disturbed = sorted(i for i in ctx if not B.within_goal(kind[i], after[i], table))
        if at is not None:            # build_sequence's rule: at its goal before, outside it after; else > 10 mm / 20 deg
            ingoal = {i for i in ctx if B.within_goal(kind[i], gd(i, pre[i]), table)}
            old = R.disturbed_items(pre, {i: post[i] for i in ctx}, None)
            disturbed = sorted({i for i in ingoal if i in disturbed} | {i for i in old if i not in ingoal})
            out["pose"] = now[dish]
        out.update(landed=bool(landing(kind[dish], dist, dev)) and deep < limit, no_clear_release=False,
                   hover_mm=round(hover * 1e3), blockers=[], lateral_mm=round(dist[0] * 1e3, 1),
                   dz_mm=round(dist[1] * 1e3, 1), tilt_deg=round(dist[2], 1), contact_mm=round(deep * 1e3, 2),
                   disturbed=disturbed, moved=moved,
                   margins={k: None if v is None else round(v, 3) for k, v in margins.items()},
                   marginal=[k for k, v in margins.items() if v is not None and abs(v) <= MARGIN])
        return out

    def verdict(t):
        """The single drop's verdict, or the majority of its 3 perturbed reruns."""
        rs = t.get("reruns")
        if not rs:
            t["verdict"] = {"landed": t["landed"], "disturbed": t["disturbed"], "source": "single"}
        else:
            hits = [x for r in rs for x in r["disturbed"]]
            t["verdict"] = {"landed": sum(r["landed"] for r in rs) * 2 > len(rs),
                            "disturbed": sorted(x for x in set(hits) if hits.count(x) * 2 > len(rs)),
                            "source": f"majority of {len(rs)} perturbed releases"}
        return t

    def label(t):
        if len(t["removed"]) > 1:                                   # --edges swaps: a partial load
            return f"{t['dish']} into {len(ids) - 1 - len(t['removed'])} placed"
        return f"{t['dish']} " + (f"without {t['removed'][0]}" if t["removed"] else "last")

    def show(t, head):
        print(f"{head} {label(t)}" + (f" seed {t['seed']} offset {t['dx_mm']} {t['dy_mm']} mm yaw {t['yaw_deg']} deg"
                                      if t["seed"] is not None else "")
              + (f" parked fallen {t['parked_fallen']}" if t["parked_fallen"] else "")
              + (f": path blocked at step {t['blocked_step']} ({t['blocked_lift_mm']} mm of {t['start_lift_mm']} mm) "
                 f"(blockers {t['blockers']})" if t.get("blocked_step") else
                 f": no clear release (blockers {t['blockers']})" if t["no_clear_release"] else
                 f": hover {t['hover_mm']} mm landed {t['landed']} lateral {t['lateral_mm']} dz {t['dz_mm']} mm tilt "
                 f"{t['tilt_deg']} deg contact {t['contact_mm']} mm disturbed {t['disturbed'] or '-'} marginal "
                 f"{t['marginal'] or '-'}")
              + f" need {t['context_rounds'][0]['fallen'] or '-'} drift {t['context_drift_mm']} mm "
                f"({time.monotonic() - t0:.0f} s)", flush=True)

    def rerun(t, why):
        if t.get("reruns"):
            return False
        t["reruns"], t["rerun_reason"] = [], why
        for sd in seeds:
            r = drop(t["dish"], t["removed"], t["index"], sd, RERUN_MM, RERUN_DEG, at=ats.get(t["index"]))
            t["reruns"].append(r)
            show(r, f"  rerun ({why})")
        verdict(t)
        print(f"  verdict {label(t)}: landed {t['verdict']['landed']} disturbed {t['verdict']['disturbed'] or '-'} "
              f"({t['verdict']['source']})", flush=True)
        return True

    thresholds = {"tolerance": tol_name,
                  "at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3), "tilt_deg": v[2]}
                              for k, v in table.items()},
                  "disturbed": "a context dish outside its at-goal tolerance of its target after the drop"
                               + ("; swaps: build_sequence's rule (at its goal before: outside it after; else moved > "
                                  "10 mm / 20 deg)" if args.edges == "swaps" else ""),
                  "contact_limit_mm": round(limit * 1e3, 3),
                  "release": {"rule": B.RELEASE_RULE, "allowance_mm": round(checker.penetration_limit_m * 1e3, 3),
                              "step_mm": round(B.RELEASE_STEP_M * 1e3, 3), "cap_mm": round(B.RELEASE_CAP_M * 1e3, 3),
                              "near_m": B.RELEASE_NEAR_M} if release == "clear" else
                             {"rule": B.RELEASE_RULE, "allowance_mm": round(checker.penetration_limit_m * 1e3, 3),
                              **rinfo},
                  "margin": MARGIN, "rerun": {"lateral_mm": RERUN_MM, "yaw_deg": RERUN_DEG, "seeds": list(seeds)},
                  "moved_mm": MOVED_MM, "moved_deg": MOVED_DEG, "fall_rounds": FALL_ROUNDS,
                  "context_ticks": K.SETTLE_TICKS, "drop_ticks": K.SETTLE_TICKS + K.WINDOW_TICKS,
                  "physics_hz": 120, "candidate_mm": round(PAIR_M * 1e3, 3)}
    tests, t0, ats = [], time.monotonic(), {}
    if args.refine:
        return refine_graph(args, dep0, out_path, report, started, ids, targets, states, drop, verdict, rerun, show, ats)
    if args.edges == "swaps":
        return swap_graph(args, source, entries, out_path, report, started, B, P, ids, targets, entries_settled, frames,
                          rule, pair_list, witness, rec.states, resettle, kit_resettle, head, thresholds, drop, verdict,
                          rerun, show, ats, rinfo)
    for k, (dish, removed) in enumerate(runs, 1):
        t = drop(dish, removed, k, args.perturb_seed if on else None, args.perturb_mm, args.perturb_deg)
        tests.append(verdict(t))
        show(t, f"test {k}/{len(runs)}")
    drop_seconds = time.monotonic() - t0
    for t in tests:                                                 # margins: 3 perturbed releases, majority
        if t["marginal"]:
            rerun(t, "margin " + "+".join(t["marginal"]))
    margin_seconds = time.monotonic() - t0 - drop_seconds
    edges = graph_edges(tests)
    for _ in range(3):                                              # two-way pairs without need both ways: retested
        todo = [p for p, cls in two_way(edges) if cls == "cycle"]
        fresh = [rerun(t, "two-way " + "-".join(p)) for p in todo for t in tests
                 if (not t["removed"] and t["dish"] in p) or (t["removed"] and {t["dish"], t["removed"][0]} == set(p))]
        if not any(fresh):
            break
        edges = graph_edges(tests)
    retest_seconds = time.monotonic() - t0 - drop_seconds - margin_seconds
    pairs2 = two_way(edges)
    mutual = [p for p, cls in pairs2 if cls == "mutual_support"]
    cycles = [p for p, cls in pairs2 if cls == "cycle"]
    goal_order = sorted(edges)
    count = {r: sum(r in rs for rs in edges.values()) for r in REASONS[:3]}
    rep = groups_of(mutual)
    node = {i: rep.get(i, i) for i in ids}
    contracted = sorted({(node[a], node[b]) for a, b in goal_order if node[a] != node[b]})
    open_order = [e for e in goal_order if node[e[0]] != node[e[1]]]   # the graph minus the mutual pairs' own edges
    waves = layers(ids, open_order) if layers(sorted(set(node.values())), contracted) is not None else None
    n_waves = max(waves.values()) + 1 if waves is not None else None
    print(f"graph: {len(edges)} edges ({', '.join(f'{r} {n}' for r, n in count.items())}), cycles {cycles or '-'}, "
          f"mutual support {mutual or '-'}, waves {n_waves if n_waves is not None else 'none'}", flush=True)

    builds, extra, order, poses_world, built_entries, failed, diagnostic = [], [], [], targets, [], None, False
    if waves is None:
        status = "cycle"
        reason = "no valid one-at-a-time order: " + (
            "two-way pairs " + ", ".join(f"{a} <-> {b}" for a, b in cycles) if cycles
            else "the edges form a cycle through more than two dishes") + (
            f"; mutual support {', '.join(f'{a} <-> {b}' for a, b in mutual)}" if mutual else "")
    else:
        together = [sorted(i for i in ids if node[i] == r) for r in sorted(set(rep.values()))]
        diagnostic = bool(together)
        built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids, landing, initial=open_order,
                                            together=together)
        builds = record["builds"]
        for k, b in enumerate(builds):
            print(f"{'diagnostic ' if diagnostic else ''}build {k + 1}: {b['status']} {b['dish'] or ''} "
                  f"{b['moved'] or ''}", flush=True)
        learned = sorted(tuple(x) for x in record["learned"])
        extra = sorted(set(learned) - set(open_order))
        order = record.get("order") or builds[-1]["order"]
        if built == "ok":
            poses_world = record["poses_world"]
            built_entries = local_entries(P, entries, frames, poses_world)
        else:
            failed = builds[-1]["dish"]
        tail = (f"; {len(extra)} constraint(s) learned beyond the graph's {len(open_order)} edges"
                + ("; the learned constraints close a cycle" if layers(ids, learned) is None else ""))
        if diagnostic:
            status = "mutual_support"
            reason = (f"place together or re-roll: mutual support {', '.join(f'{a} <-> {b}' for a, b in mutual)}; "
                      f"diagnostic build (each pair released together, not a benchmark move): {record['reason']}{tail}")
        else:
            status = "ok" if built == "ok" else "no_order" if len(builds) >= K.SEQUENCE_BUILDS else builds[-1]["status"]
            reason = record["reason"] + tail
        for a, b in extra:                                          # learned constraints join the graph
            edges[(a, b)] = edges.get((a, b), []) + ["learned"]
        if built == "ok" and layers(ids, open_order + extra) is not None:
            waves = layers(ids, open_order + extra)
            n_waves = max(waves.values()) + 1
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "procedure": "v20", "status": status,
           "reason": reason, "pairs_tested": pair_list, "tests": tests,
           "edges": [{"a": a, "b": b, "reasons": rs} for (a, b), rs in sorted(edges.items())],
           "two_way": [{"pair": p, "class": cls, "reasons_ab": edges.get(tuple(p), []),
                        "reasons_ba": edges.get(tuple(p[::-1]), [])} for p, cls in pairs2],
           "cycles": cycles, "mutual_support": mutual, "goal_order": [list(x) for x in goal_order], "order": order,
           "builds": builds, "diagnostic": diagnostic, "extra_constraints": [list(x) for x in extra], "waves": waves,
           "n_waves": n_waves, "acyclic": waves is not None, "failed_dish": failed, "entries": entries,
           "targets_source": targets_source, "targets_world": targets, "poses_world": poses_world,
           "entries_settled": entries_settled, "built_entries": built_entries, "frames": frames, "landing_rule": rule,
           "thresholds": thresholds, "perturbation": perturbation(args),
           "gate": gate_info(args, gate) if gate is not None else None, **resettle,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "drops": len(tests),
                   "reruns": sum(len(t.get("reruns") or []) for t in tests),
                   "drop_wall_seconds": round(drop_seconds, 1), "margin_wall_seconds": round(margin_seconds, 1),
                   "retest_wall_seconds": round(retest_seconds, 1), "builds_run": len(builds), **kit_resettle},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    reruns = sum(bool(t.get("reruns")) for t in tests)
    report.update(result="PASS", status=status, dependency=rel(out_path),
                  reason=head + f"status {status} pairs {len(pair_list)} drops {len(tests)} reruns {reruns} edges "
                         f"{len(edges)} (knock {count['knock']} need {count['need']} block {count['block']} learned "
                         f"{len(extra)}) cycles {len(cycles)} mutual {len(mutual)} builds {len(builds)}"
                         f"{' diagnostic' if diagnostic else ''} extra {len(extra)} waves "
                         f"{n_waves if n_waves is not None else 'none'} targets {targets_source}{shaken}")


def swap_graph(args, source, entries, out_path, report, started, B, P, ids, targets, entries_settled, frames, rule,
               pair_list, witness, states, resettle, kit_resettle, head, thresholds, drop, verdict, rerun, show, ats,
               rinfo):
    """--edges swaps after the sampler (module docstring): one swap per candidate pair in the witness state, the edges,
    the (unbuilt) library orders, the record. ``drop`` ... ``ats`` are pairs()' closures and their rerun state."""
    order = witness["order"]
    pos = {i: n for n, i in enumerate(order)}
    support = {tuple(x) for x in witness["support"]}
    learned = {tuple(x) for x in witness["learned"]}
    cands = {}
    for a, b, d in pair_list:
        cands[tuple(sorted((a, b), key=pos.get))] = {"distance_mm": d, "source": ["near"]}
    for x, y in sorted(learned):
        cands.setdefault(tuple(sorted((x, y), key=pos.get)), {"distance_mm": None, "source": []})["source"].append("learned")
    todo = sorted(cands, key=lambda p: (pos[p[0]], pos[p[1]]))
    print(f"swaps {args.load}: witness {' '.join(order)}; {len(todo)} candidate pairs ({len(pair_list)} within "
          f"{PAIR_M * 1e3:.0f} mm, {sum('learned' in c['source'] for c in cands.values())} learned); landing = {rule}",
          flush=True)
    swaps, edges, t_swaps = [], {}, time.monotonic()
    partial = out_path.parent / "_kit" / args.load / "swaps_partial.json"

    def decide(t):
        """The drop (the single one, else a rerun) whose outcome is the verdict's: the clip's source."""
        v = t["verdict"]
        return next((d for d in [t, *(t.get("reruns") or [])]
                     if d["landed"] == v["landed"] and bool(d["disturbed"]) == bool(v["disturbed"])), t)

    def judged(t, head):
        show(verdict(t), head)
        if t["marginal"]:
            rerun(t, "margin " + "+".join(t["marginal"]))
        return t["verdict"]

    for k, (a, b) in enumerate(todo, 1):
        prefix = order[:pos[a]]
        state = {i: states.get(a, {}).get(i, targets[i]) for i in prefix}
        later = [i for i in ids if i not in prefix]
        t1 = drop(b, [i for i in later if i != b], 2 * k - 1, at=state)
        ats[t1["index"]] = state
        v1 = judged(t1, f"swap {k}/{len(todo)} {b} before {a} (step 1)")
        t2, reason, step = None, None, None
        if not v1["landed"]:
            reason, step = ("block" if t1["no_clear_release"] else "need"), 1
        elif v1["disturbed"]:
            reason, step = "knock", 1
        else:
            at2 = {**state, b: decide(t1).get("pose", targets[b])}
            t2 = drop(a, [i for i in later if i not in (a, b)], 2 * k, at=at2)
            ats[t2["index"]] = at2
            v2 = judged(t2, f"swap {k}/{len(todo)} {a} after {b} (step 2)")
            if not v2["landed"]:
                reason, step = "block", 2
            elif v2["disturbed"]:
                reason, step = "knock", 2
        swaps.append({"swap": k, "pair": [a, b], "placed_before": len(prefix), **cands[(a, b)], "step1": t1,
                      "step2": t2, "failed": reason is not None, "reason": reason, "step": step,
                      "context_valid": t1["context_valid"] and (t2 is None or t2["context_valid"])})
        if reason is not None:
            t, dropped = (t1, b) if step == 1 else (t2, a)
            d, v = decide(t), t["verdict"]
            if not v["landed"] and d["no_clear_release"] and d.get("blocked_step"):
                detail = (f"{dropped}'s descent is blocked " + ("at its start, " if d.get("blocked") == "start" else "")
                          + f"{d['blocked_lift_mm']:g} mm above its move pose (step {d['blocked_step']} of the descent "
                          f"from {d['start_lift_mm']:g} mm) by {', '.join(d['blockers'])}")
            elif not v["landed"] and d["no_clear_release"]:
                detail = f"{dropped} has no clear release (blocked by {', '.join(d['blockers'])})"
            elif not v["landed"]:
                detail = (f"{dropped} lands {d['lateral_mm']} mm lateral / {d['dz_mm']} mm dz / {d['tilt_deg']} deg off "
                          f"its goal, contact {d['contact_mm']} mm")
            else:
                detail = f"{dropped} moves {', '.join(v['disturbed'])}"
            mins = [abs(x) for x in t["margins"].values() if x is not None]
            edges[(a, b)] = {"a": a, "b": b, "reason": reason, "step": step, "fails": dropped,
                             "support": (a, b) in support, "learned": (a, b) in learned, "detail": detail,
                             "placed_before": len(prefix), "affected": v["disturbed"],
                             "margin": {"min": min(mins) if mins else None, "source": v["source"],
                                        "margins": t["margins"]},
                             "swap": k, "traj": [decide(t1)["traj"]] + ([decide(t2)["traj"]] if t2 else [])}
        print(f"swap {k}/{len(todo)} {a} > {b}: " + (f"EDGE {reason} (step {step}) {edges[(a, b)]['detail']}"
                                                     if reason else "passes (no edge)")
              + f" ({time.monotonic() - t_swaps:.0f} s)", flush=True)
        partial.write_text(json.dumps({"swaps": swaps}, default=str) + "\n")
    E = {e: edges[e] for e in sorted(edges)}
    waves = layers(ids, list(E))                                    # acyclic: every edge follows the witness order
    count = {r: sum(e["reason"] == r for e in E.values()) for r in ("need", "block", "knock")}
    library = library_orders(ids, order, E, args.library)
    drops = [d for s in swaps for t in (s["step1"], s["step2"]) if t for d in [t, *(t.get("reruns") or [])]]
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "procedure": "swaps", "status": "ok",
           "reason": (f"goal built one dish at a time in {len(resettle['resettle_builds'])} build(s); {len(todo)} swaps, "
                      f"{len(E)} edge(s)"), "release": rinfo, **resettle,
           "pairs_tested": [{"pair": list(p), **cands[p]} for p in todo], "swaps": swaps, "edges": list(E.values()),
           "support_only": sorted(list(p) for p in support if p not in E), "waves": waves,
           "n_waves": max(waves.values()) + 1, "library": library, "library_summary": None, "entries": entries,
           "targets_source": "sequential_build", "targets_world": targets, "entries_settled": entries_settled,
           "frames": frames, "landing_rule": rule, "thresholds": thresholds, "tolerance": thresholds["tolerance"],
           "valid": None, "verdict": None,
           "refinement": None, "learning": {"max_rounds": args.learn_rounds, "rounds": [], "edges_added": 0},
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "drops": len(drops),
                   "reruns": sum(len(t.get("reruns") or []) for s in swaps for t in (s["step1"], s["step2"]) if t),
                   "swap_wall_seconds": round(time.monotonic() - t_swaps, 1), **kit_resettle},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    report.update(result="PASS", status="ok", dependency=rel(out_path),
                  reason=head + f"status ok witness {len(order)} of {len(ids)} swaps {len(todo)} drops {len(drops)} edges "
                         f"{len(E)} (need {count['need']} block {count['block']} knock {count['knock']}) support "
                         f"{sum(e['support'] for e in E.values())} waves {max(waves.values()) + 1} library {len(library)}")


def refine_graph(args, dep, out_path, report, started, ids, targets, states, drop, verdict_of, rerun, show, ats):
    """--refine (plans/2026-10-15-reposing-sampler-five-samples.md): every edge (a, b) of a library violation that still
    built (the record's verdict 'refine') becomes conditional, b needs one of {a, C...}: C = the dishes that order placed
    before b and the edge's swap context lacked (the witness's dishes before a), each confirmed by one drop of b in that
    context with C added at its goal pose and a absent (a step-2 edge: then a's drop after b, as its swap did); confirmed
    = that failing step now passes. An edge with no confirmed C fails the refinement (the sample is invalid). Then
    FRESH_ORDERS random orders obeying the refined graph join the library (kind refined_obeying) for --order-index."""
    order = dep["witness"]["order"]
    pos = {i: n for n, i in enumerate(order)}
    E = {(e["a"], e["b"]): e for e in dep["edges"]}
    todo = {}
    for item in dep["verdict"]["refine"]:
        lib = dep["library"][item["index"]]
        lpos = {d: n for n, d in enumerate(lib["order"])}
        for a, b in map(tuple, item["edges"]):
            prefix = set(order[:pos[a]])
            t = todo.setdefault((a, b), {"orders": [], "C": []})
            t["orders"].append(item["index"])
            t["C"] += [d for d in lib["order"][:lpos[b]] if d not in prefix and d != a and d not in t["C"]]

    def judged(t, head):
        show(verdict_of(t), head)
        if t["marginal"]:
            rerun(t, "margin " + "+".join(t["marginal"]))
        return t["verdict"]

    def decide(t):
        v = t["verdict"]
        return next((d for d in [t, *(t.get("reruns") or [])]
                     if d["landed"] == v["landed"] and bool(d["disturbed"]) == bool(v["disturbed"])), t)

    print(f"refine {args.load}: {len(todo)} edge(s) from orders {[i['index'] for i in dep['verdict']['refine']]}: "
          + "; ".join(f"{a} > {b} C {t['C']}" for (a, b), t in todo.items()), flush=True)
    confirmations, k, ok_all, why = [], 0, True, []
    for (a, b), t in todo.items():
        e = E[(a, b)]
        prefix = order[:pos[a]]
        state = {i: states.get(a, {}).get(i, targets[i]) for i in prefix}
        later = [i for i in ids if i not in prefix]
        conf = []
        for c in t["C"]:
            k += 1
            t1 = drop(b, [i for i in later if i not in (b, c)], CONFIRM_INDEX + 2 * k - 1, at=state)
            ats[t1["index"]] = state
            v1 = judged(t1, f"confirm {a} > {b} with {c}: {b} (step 1)")
            ok, t2 = v1["landed"] and not v1["disturbed"], None
            if ok and e["step"] == 2:
                at2 = {**state, b: decide(t1).get("pose", targets[b])}
                t2 = drop(a, [i for i in later if i not in (a, b, c)], CONFIRM_INDEX + 2 * k, at=at2)
                ats[t2["index"]] = at2
                v2 = judged(t2, f"confirm {a} > {b} with {c}: {a} after {b} (step 2)")
                ok = v2["landed"] and not v2["disturbed"]
            confirmations.append({"edge": [a, b], "c": c, "confirmed": ok, "step1": t1, "step2": t2,
                                  "context_valid": t1["context_valid"] and (t2 is None or t2["context_valid"]),
                                  "traj": [decide(t1)["traj"]] + ([decide(t2)["traj"]] if t2 else [])})
            conf += [c] if ok else []
            print(f"confirm {a} > {b} with {c}: {'CONFIRMED' if ok else 'not confirmed'} "
                  f"({time.monotonic() - started:.0f} s)", flush=True)
        t["confirmed"] = conf
        if conf:
            e.update(any_of=[a] + conf, conditional=True, refined_from=t["orders"])
        else:
            ok_all = False
            why.append(f"{a} > {b}: the violation built (orders {t['orders']}) but none of {t['C'] or 'no dish'} "
                       f"lets {b} go in without {a}")
    fresh = []
    if ok_all:
        rng, n0 = random.Random(LIBRARY_SEED + 1), len(dep["library"])
        for j in range(FRESH_ORDERS):
            o = obeying_order(ids, E, rng)
            fresh.append({"index": n0 + j, "kind": "refined_obeying", "order": o, "edge": None, **predict(o, E),
                          "outcome": None, "round": len((dep.get("learning") or {}).get("rounds") or [])})
        dep["library"] += fresh
    dep["edges"] = [E[x] for x in sorted(E)]
    dep["refinement"] = {"ok": ok_all, "reason": "; ".join(why) or None,
                         "edges": [{"a": a, "b": b, "orders": t["orders"], "C": t["C"], "confirmed": t["confirmed"]}
                                   for (a, b), t in todo.items()],
                         "confirmations": confirmations, "fresh": [f["index"] for f in fresh],
                         "wall_seconds": round(time.monotonic() - started, 1)}
    if not ok_all:
        dep["verdict"] = verdict(dep)
        dep["valid"] = False
    args.force = True
    write_record(args, out_path, dep)
    report.update(result="PASS", status="refined" if ok_all else "invalid", dependency=rel(out_path),
                  reason=f"refine {len(todo)} edge(s), {len(confirmations)} confirmation drop pairs, "
                         f"{sum(c['confirmed'] for c in confirmations)} confirmed; "
                         + (f"{len(fresh)} fresh orders appended" if ok_all else "refinement failed: " + "; ".join(why)))


def library_run(args, source, entries, gate, out_path, kit_dir, report, started):
    """--order-index: build the record's library orders K one after another in this Kit session (build_sequence, the
    fixed order, nothing learned, the at-goal tolerance of the built goal, stopping where the order says); each outcome
    goes to _kit/<load_id>/library/<K>.json."""
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS
    dep = json.loads(out_path.read_text()) if out_path.is_file() else {}
    if dep.get("procedure") != "swaps" or not dep.get("library"):
        raise Refused(f"no --edges swaps record with a library at {rel(out_path)}")
    tol_name = dep.get("tolerance") or "bench"                     # the record's tolerance, whatever --tolerance says
    table, climit, _ = tolerance_of(B, tol_name, LIMITS["peak_penetration_m"])
    snapshot = goal_setup(args, entries, None, B, K, P)[0]
    entries = dep.get("entries_chosen") or entries                  # the re-posing sampler's chosen entries
    rinfo = dep.get("release") or {"mode": "clear"}                # the record's move, whatever --release says
    release = "insertion" if rinfo["mode"] == "insertion" else "clear"
    if release == "insertion" and rinfo.get("version") != "2026-10-15b":
        raise Refused(f"{rel(out_path)} was made with the retired insertion move (released H above the target)")
    by_id = {e["id"]: e for e in entries}
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    folder = kit_dir / "library"
    backend = K._sequence_scene(args, B, entries, snapshot, folder / f"scene_{args.order_index[0]}", report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    done = []
    for k in args.order_index:
        lib, t = dep["library"][k], time.monotonic()
        status, record, _ = K.build_sequence(backend, B, P, by_id, dep["targets_world"], centroids,
                                             lambda kind, dist, dev: B.within_goal(kind, dist, table),
                                             fixed=lib["order"], stop=lib["stop"], release=release,
                                             moves=dep.get("moves_world"), goal_table=table, contact_m=climit)
        out = {**library_outcome(lib, status, record["builds"][-1]), "wall_seconds": round(time.monotonic() - t, 1),
               "tolerance": tol_name}
        folder.mkdir(parents=True, exist_ok=True)
        (folder / f"{k}.json").write_text(json.dumps({"index": k, "kind": lib["kind"], "edge": lib["edge"],
                                                      "predicted": lib["predicted"], "outcome": out}, indent=1) + "\n")
        print(f"library {k} {lib['kind']}{' ' + ' > '.join(lib['edge']) if lib['edge'] else ''} ({tol_name}): {status} "
              f"{out['failed_dish'] or ''} placed {out['placed']}, predicted {lib['predicted'] or 'all land'}, "
              f"matches {out['matches_prediction']} ({out['wall_seconds']:.0f} s)", flush=True)
        done.append(f"{k}:{status}{'+' if out['matches_prediction'] else '-'}")
    report.update(result="PASS", status="library", reason=f"library {' '.join(done)}")


def merge_library(args):
    """--merge-library (Kit-free): every built library order's outcome (_kit/<load_id>/library/<K>.json) into the
    record, library_summary {kind: orders, built, matches (as the rule asks), at_predicted_dish} and the verdict; a
    verdict 'learn' runs a learning round (learn_round, at most --learn-rounds per sample, recorded in learning {max_rounds,
    rounds, edges_added}) and the verdict again (pending: its fresh orders are not built yet)."""
    folder = record_folder(args)
    out_path = folder / f"{args.load}.json"
    dep = json.loads(out_path.read_text())
    for lib in dep["library"]:
        path = folder / "_kit" / args.load / "library" / f"{lib['index']}.json"
        if path.is_file():
            lib["outcome"] = json.loads(path.read_text())["outcome"]
    learn = dep.get("learning") or {"rounds": [], "edges_added": 0}
    learn["max_rounds"] = args.learn_rounds
    dep["learning"] = learn
    v = verdict(dep)
    if v["stage"] == "learn":
        added, unlearnable = learn_round(dep)
        print(f"learning round {len(learn['rounds'])}: {len(added)} edge(s) "
              + ", ".join(f"{a}>{b}" for a, b in added) + (f"; {'; '.join(unlearnable)}" if unlearnable else "")
              + f"; {len(learn['rounds'][-1]['orders'])} fresh obeying orders", flush=True)
        v = verdict(dep)
    E = {(e["a"], e["b"]): e for e in dep["edges"]}
    summary = {}
    for lib in dep["library"]:
        s = summary.setdefault(lib["kind"], {"orders": 0, "built": 0, "matches": 0, "at_predicted_dish": 0})
        s["orders"] += 1
        if lib["outcome"] is not None:
            pr, ok, _ = judge_order(lib, E)
            s["built"] += 1
            s["matches"] += bool(ok)
            s["at_predicted_dish"] += bool(pr["predicted"]) and lib["outcome"].get("failed_dish") == pr["predicted"]
    dep["library_summary"] = summary
    dep["verdict"] = v
    dep["valid"] = True if v["stage"] == "valid" else False if v["stage"] == "invalid" else None
    args.force = True
    write_record(args, out_path, dep)
    built = sum(s["built"] for s in summary.values())
    print(f"[RESULT] PASS {args.load} merged {built} of {len(dep['library'])} library orders: "
          + ", ".join(f"{k} {s['matches']} of {s['built']} as the rule asks" for k, s in summary.items())
          + f"; learning rounds {len(learn['rounds'])} of {args.learn_rounds}; verdict {v.get('stage')}: {v.get('reason')}",
          flush=True)


def rejudge(args):
    """--rejudge (Kit-free; plans/2026-10-16-doubled-tolerance-rerun.md): every --records sample's swap edges re-judged
    from its saved drops under --tolerance, written to --out-dir/<load_id>.json with a fresh library drawn from those
    edges (the learned edges, the refinement, the library outcomes and the verdict go: the library and the learning loop
    run again under the new rule), its Kit files (states.json, traj/) and S (.scores.json) copied. A saved drop: landed =
    it had a release and its saved lateral / dz / tilt lie within the table with contact below the table's limit;
    disturbed = a placed dish it moved (> 1 mm / 1 deg, saved with its goal distance after) that ends outside the table
    (the "at its goal before" branch; a dish not at its goal that moved > 10 mm / 20 deg cannot be told from the saved
    fields, and neither sample has one). Its verdict as the live rule: a drop within MARGIN of a threshold of the table
    takes the majority of its saved perturbed reruns, or with none saved (they ran only near the old thresholds) its own
    outcome, flagged marginal_unrerun; any other drop decides alone. A swap whose step 1 now passes but failed before has
    no saved step 2: its pair gets no edge (untested_step2: an obeying library order tests it, and the learning loop adds
    an edge if it fails). Every swap and drop keeps its old judgment under ``before``."""
    import shutil
    sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
    import frigidaire_bench as B
    for path in args.records:
        rec = json.loads(path.read_text())
        lid = rec["load_id"]
        out_path = args.out_dir / f"{lid}.json"
        if out_path.exists() and not args.force:
            print(f"[RESULT] FAIL {lid} refusing to overwrite {rel(out_path)} (--force)", flush=True)
            continue
        if rec.get("procedure") != "swaps" or rec.get("status") != "ok":
            print(f"[RESULT] FAIL {lid} {rel(path)} is not a built --edges swaps sample", flush=True)
            continue
        old = rec.get("tolerance") or "bench"
        table, limit, text = tolerance_of(B, args.tolerance, rec["thresholds"]["contact_limit_mm"] / 1e3)
        kind = {e["id"]: e["kind"] for e in rec.get("entries_chosen") or rec["entries"]}

        def judge(t):
            """(landed, disturbed, margins) of one saved drop under the table."""
            if t.get("no_clear_release"):
                return False, [], {}
            tl, dish = table[kind[t["dish"]]], t["dish"]
            lat, dz, tilt, deep = t["lateral_mm"] / 1e3, t["dz_mm"] / 1e3, t["tilt_deg"], t["contact_mm"] / 1e3
            after = {i: (m["goal"][0] / 1e3, m["goal"][1] / 1e3, m["goal"][2]) for i, m in (t.get("moved") or {}).items()}
            margins = {"lateral": (tl[0] - lat) / tl[0], "height": (tl[1] - dz) / tl[1], "tilt": (tl[2] - tilt) / tl[2],
                       "contact": (limit - deep) / limit}
            near = [(table[kind[i]][c] - d[c]) / table[kind[i]][c] for i, d in after.items() for c in range(3)]
            margins["disturbance"] = min(near, key=abs) if near else None
            return (B.within_goal(kind[dish], (lat, dz, tilt), table) and deep < limit,
                    sorted(i for i, d in after.items() if not B.within_goal(kind[i], d, table)), margins)

        def verdict_of(t):
            """(the live rule's verdict of a saved drop under the table, its marginal thresholds when unrerun)."""
            landed, disturbed, margins = judge(t)
            marginal = [k for k, v in margins.items() if v is not None and abs(v) <= MARGIN]
            rs = t.get("reruns") or []
            if marginal and rs:
                js = [judge(r) for r in rs]
                hits = [x for j in js for x in j[1]]
                return {"landed": sum(j[0] for j in js) * 2 > len(js),
                        "disturbed": sorted(x for x in set(hits) if hits.count(x) * 2 > len(js)),
                        "source": f"majority of {len(js)} perturbed releases"}, None
            return {"landed": landed, "disturbed": disturbed, "source": "single"}, (marginal or None)

        def decide(t, v):
            return next((d for d in [t, *(t.get("reruns") or [])]
                         if judge(d)[0] == v["landed"] and bool(judge(d)[1]) == bool(v["disturbed"])), t)

        def redo(t, v=None):
            """The saved drop with its fields under the table, the old ones under ``before``."""
            landed, disturbed, margins = judge(t)
            out = {**t, "landed": landed, "disturbed": disturbed,
                   "margins": {k: None if x is None else round(x, 3) for k, x in margins.items()},
                   "marginal": [k for k, x in margins.items() if x is not None and abs(x) <= MARGIN],
                   "before": {"tolerance": old, **{k: t.get(k) for k in ("landed", "disturbed", "margins", "marginal",
                                                                         "verdict")}}}
            if t.get("reruns"):
                out["reruns"] = [redo(r) for r in t["reruns"]]
            if v is not None:
                out["verdict"] = v
            return out

        order = rec["witness"]["order"]
        support = {tuple(x) for x in rec["witness"]["support"]}
        learned = {tuple(x) for x in rec["witness"]["learned"]}
        swaps, edges, untested, unrerun, changed = [], {}, [], [], []
        for s in rec["swaps"]:
            a, b = s["pair"]
            t1, t2 = s["step1"], s["step2"]
            v1, f1 = verdict_of(t1)
            v2 = f2 = reason = step = None
            if not v1["landed"]:
                reason, step = ("block" if t1["no_clear_release"] else "need"), 1
            elif v1["disturbed"]:
                reason, step = "knock", 1
            elif t2 is None:                                      # step 1 failed under the old rule: no step 2 saved
                untested.append([a, b])
            else:
                v2, f2 = verdict_of(t2)
                if not v2["landed"]:
                    reason, step = "block", 2
                elif v2["disturbed"]:
                    reason, step = "knock", 2
            unrerun += [{"swap": s["swap"], "step": n, "dish": t["dish"], "near": f}
                        for n, t, f in ((1, t1, f1), (2, t2, f2)) if f]
            if reason is not None:
                t, v, dropped = (t1, v1, b) if step == 1 else (t2, v2, a)
                d = decide(t, v)
                if not v["landed"] and d["no_clear_release"] and d.get("blocked_step"):
                    detail = (f"{dropped}'s descent is blocked " + ("at its start, " if d.get("blocked") == "start" else "")
                              + f"{d['blocked_lift_mm']:g} mm above its move pose (step {d['blocked_step']} of the "
                              f"descent from {d['start_lift_mm']:g} mm) by {', '.join(d['blockers'])}")
                elif not v["landed"] and d["no_clear_release"]:
                    detail = f"{dropped} has no clear release (blocked by {', '.join(d['blockers'])})"
                elif not v["landed"]:
                    detail = (f"{dropped} lands {d['lateral_mm']} mm lateral / {d['dz_mm']} mm dz / {d['tilt_deg']} deg "
                              f"off its goal, contact {d['contact_mm']} mm")
                else:
                    detail = f"{dropped} moves {', '.join(v['disturbed'])}"
                margins = judge(t)[2]
                mins = [abs(x) for x in margins.values() if x is not None]
                edges[(a, b)] = {"a": a, "b": b, "reason": reason, "step": step, "fails": dropped,
                                 "support": (a, b) in support, "learned": (a, b) in learned, "detail": detail,
                                 "placed_before": s["placed_before"], "affected": v["disturbed"],
                                 "margin": {"min": min(mins) if mins else None, "source": v["source"],
                                            "margins": {k: None if x is None else round(x, 3) for k, x in margins.items()}},
                                 "swap": s["swap"],
                                 "traj": [decide(t1, v1)["traj"]] + ([decide(t2, v2)["traj"]] if step == 2 else [])}
            if (s["reason"], s["step"]) != (reason, step):
                changed.append([a, b, f"{s['reason'] or 'none'} (step {s['step']}) -> {reason or 'none'} (step {step})"])
            swaps.append({**s, "failed": reason is not None, "reason": reason, "step": step, "step1": redo(t1, v1),
                          "step2": None if t2 is None else redo(t2, v2),
                          "untested_step2": [a, b] in untested,
                          "before": {"tolerance": old, "failed": s["failed"], "reason": s["reason"], "step": s["step"]}})
        E = {e: edges[e] for e in sorted(edges)}
        ids = [e["id"] for e in rec["entries"]]
        waves = layers(ids, list(E))
        library = library_orders(ids, order, E, args.library)
        before = [e for e in rec["edges"] if e.get("swap") is not None]
        count = {r: sum(e["reason"] == r for e in E.values()) for r in ("need", "block", "knock")}
        dropped = [[e["a"], e["b"], e["reason"], e["step"]] for e in before if (e["a"], e["b"]) not in E]
        info = {"from": rel(path), "tolerance": args.tolerance, "from_tolerance": old,
                "utc": datetime.now(timezone.utc).isoformat(), "edges_before": len(before), "edges_after": len(E),
                "dropped": dropped, "changed": changed, "untested_step2": untested, "marginal_unrerun": unrerun,
                "learned_dropped": sum(e.get("reason") == "learned" for e in rec["edges"]),
                "refinement_dropped": rec.get("refinement") is not None, "library_before": len(rec["library"]),
                "verdict_before": (rec.get("verdict") or {}).get("stage")}
        rule = (rec.get("landing_rule") or "").replace(
            "at-goal tolerance of the sequentially built pose",
            f"at-goal tolerance of the sequentially built pose ({args.tolerance}: {text})").replace(
            "contact depth < 2 mm", f"contact depth < {limit * 1e3:g} mm")
        dep = {**rec, "swaps": swaps, "edges": list(E.values()),
               "support_only": sorted(list(p) for p in support if p not in E), "waves": waves,
               "n_waves": max(waves.values()) + 1, "library": library, "library_summary": None, "valid": None,
               "verdict": None, "refinement": None,
               "learning": {"max_rounds": args.learn_rounds, "rounds": [], "edges_added": 0},
               "tolerance": args.tolerance, "landing_rule": rule,
               "thresholds": {**rec["thresholds"], "tolerance": args.tolerance, "contact_limit_mm": round(limit * 1e3, 3),
                              "at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3),
                                              "tilt_deg": v[2]} for k, v in table.items()}},
               "reason": (f"goal built one dish at a time; {len(swaps)} swaps re-judged under the {args.tolerance} "
                          f"tolerance from {rel(path)}: {len(E)} edge(s) (were {len(before)})"),
               "rejudged": info, "generated_utc": datetime.now(timezone.utc).isoformat()}
        write_record(args, out_path, dep)
        src, dst = path.parent / "_kit" / lid, args.out_dir / "_kit" / lid
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src / "states.json", dst / "states.json")
        shutil.copytree(src / "traj", dst / "traj", dirs_exist_ok=True)
        if (path.parent / f"{lid}.scores.json").is_file():
            shutil.copy2(path.parent / f"{lid}.scores.json", args.out_dir / f"{lid}.scores.json")
        print(f"[RESULT] PASS {lid} rejudged ({old} -> {args.tolerance}): {len(before)} swap edges -> {len(E)} (need "
              f"{count['need']} block {count['block']} knock {count['knock']}); dropped {dropped or '-'}; changed "
              f"{changed or '-'}; untested step 2 {untested or '-'}; marginal, no rerun saved "
              f"{[(u['swap'], u['step'], u['dish'], u['near']) for u in unrerun] or '-'}; {info['learned_dropped']} "
              f"learned edge(s) and the refinement ({info['refinement_dropped']}) dropped; library {len(library)} orders "
              f"-> {rel(out_path)}", flush=True)


SAMPLE_CHUNK, SAMPLE_STAGGER_S, SAMPLE_PILOT = 2, 10., "r07_00012"
KIT_CORES, PY_CORES = tuple(range(14)), (14, 15)


def judge_log(path):
    """(ok, text) of a job log: [RESULT] PASS, no Traceback, no free(): invalid pointer (the exit code says nothing)."""
    text = path.read_text(errors="replace") if path.is_file() else ""
    return ("[RESULT] PASS" in text and "Traceback" not in text and "free(): invalid pointer" not in text), text


def sample_loop(args):
    """--sample (host python on the box, stdlib only; plans/2026-10-15-reposing-sampler-five-samples.md): the packings of
    data/logs/dot_rules/sequential_fresh_order.txt from r07_00012 on (at most --max-packings), one Kit session each (the
    re-posing sampler, the swaps, the library list), the library orders SAMPLE_CHUNK per session, --merge-library, then
    --refine and the fresh orders when the verdict asks for them, or a learning round's fresh orders (the merge appends
    them: --learn-rounds), until --target-valid samples are valid (every other Kit
    job is then killed) or the stop point (--max-packings resolved, or --max-wall-seconds from now: every job killed). Kit
    jobs one per physical core 0-13 (taskset j,j+16 inside dishsim-isaac, SAMPLE_STAGGER_S apart; a packing's pipeline
    jobs go before new packings, earlier packings first; each gets the time left as its --max-wall-seconds), a valid
    sample's clips and score on cores 14-15 (an invalid built sample's after them). A job is judged by its log; a failed
    job or merge halts new launches. Resumes from
    the records and library outcomes already on disk. Log and state: data/logs/dot_rules/<--out-dir's name>/, clips
    data/media/dot_rules/dependency_<--out-dir's name>/."""
    import subprocess
    t_end = time.time() + args.max_wall_seconds
    out = rel(args.out_dir) if args.out_dir else "data/results/dot_rules/dependency/samples"
    tag = Path(out).name                                  # samples -> the 2026-10-15 b folders, samples_v2 -> its own
    logs, media = ROOT / "data/logs/dot_rules" / tag, f"data/media/dot_rules/dependency_{tag}"
    logs.mkdir(parents=True, exist_ok=True)
    order = (ROOT / "data/logs/dot_rules/sequential_fresh_order.txt").read_text().split()
    todo = order[order.index(SAMPLE_PILOT):][:args.max_packings]
    resumed = []
    if args.resume_from:              # plans/2026-10-16-doubled-tolerance-rerun.md: the rejudged samples first, then the
        rdir = ROOT / args.resume_from                    # packings of the fresh list their first run had not attempted
        if rdir.resolve() != (ROOT / out).resolve():
            raise SystemExit("--resume-from must be the --out-dir (the folder --rejudge wrote)")
        recs = {p.name[:-5]: json.loads(p.read_text()) for p in sorted(rdir.glob("r*.json"))
                if not p.name.endswith(".scores.json")}
        resumed = [k for k in order if k in recs]
        seen = set(resumed)
        for d in recs.values():
            src = (d.get("rejudged") or {}).get("from")
            if src:
                seen |= {p.name.split(".")[0] for p in (ROOT / src).parent.glob("r*.json")}
        todo = (resumed + [k for k in order[order.index(SAMPLE_PILOT):] if k not in seen])[:args.max_packings]
    kit = "code/planner/frigidaire/frigidaire_dependency_kit.py"
    base = [kit, "--sampler", "restricted", "--edges", "swaps", "--release", "insertion", "--library", str(args.library),
            "--plate-order", args.plate_order, "--backtrack", str(args.backtrack), "--learn-rounds", str(args.learn_rounds),
            "--tolerance", args.tolerance, "--out-dir", out]
    st = {lid: {"stage": "queued", "reason": None, "started": None, "finished": None, "orders": None} for lid in todo}
    earlier = [Path(x) for x in args.retry_of.split(",")] if args.retry_of else []    # --retry-of A[,B...]
    first = earlier[0] if earlier else None

    def eligible(lid):
        """--retry-of A[,B...]: a packing goes in once A's record exists and says its sampler failed (and no later run B
        failed it again); one A built is skipped (its outcome stands) unless --rerun-backtracked and a backtrack placed a
        dish there (its witness order is not how the dishes went in). --resume-from: a new packing waits until every
        resumed sample is valid and is skipped once one is not."""
        if resumed and lid not in resumed:
            bad = [k for k in resumed if st[k]["stage"] in ("invalid", "failed", "error", "killed", "skipped")]
            if bad:
                st[lid].update(stage="skipped", reason=f"not started: {', '.join(bad)} did not validate (new packings "
                                                       f"only once every rerun sample is valid)")
                return False
            if any(st[k]["stage"] != "valid" for k in resumed):
                return False
        if first is None:
            return True
        p = ROOT / first / f"{lid}.json"
        if not p.is_file():
            return False
        d = json.loads(p.read_text())
        again = [b for b in earlier[1:] if (ROOT / b / f"{lid}.json").is_file()
                 and json.loads((ROOT / b / f"{lid}.json").read_text()).get("status") != "ok"]
        if again:
            st[lid].update(stage="skipped", reason=f"failed in its sampler again in {again[0].name}: that outcome stands")
            return False
        if d.get("status") == "ok":
            backed = any(b["outcome"] == "placed" for b in (d.get("reposing") or {}).get("backtracks") or [])
            if args.rerun_backtracked and backed:
                return True
            st[lid].update(stage="skipped", reason=f"built in {first.name}: its outcome stands")
            return False
        return True
    kit_q, py_q, running, free_kit, free_py = [], [], [], list(KIT_CORES), list(PY_CORES)
    flags = {"halt": False, "last_kit": 0., "seq": 0}

    def say(msg):
        line = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()) + " " + msg
        with (logs / "scheduler.log").open("a") as f:
            f.write(line + "\n")
        print(line, flush=True)

    def save_state():
        (logs / "scheduler_state.json").write_text(json.dumps({
            "updated_utc": datetime.now(timezone.utc).isoformat(), "deadline_utc": datetime.fromtimestamp(
                t_end, timezone.utc).isoformat(), "plate_order": args.plate_order, "tolerance": args.tolerance,
            "backtrack": args.backtrack, "resumed": resumed,
            "valid": [k for k, s in st.items() if s["stage"] == "valid"],
            "packings": st, "running": [[j.kind, j.load, j.tag, j.core] for j in running],
            "queued": [[j.kind, j.load, j.tag] for j in kit_q + py_q]}, indent=1) + "\n")

    class Job:
        def __init__(self, kind, load, argv, tag="", wrapper="code/util/run_kit.sh", pattern=None):
            flags["seq"] += 1
            self.kind, self.load, self.argv, self.tag, self.wrapper = kind, load, argv, tag, wrapper
            self.pattern, self.seq, self.proc, self.core, self.t0 = pattern, flags["seq"], None, None, None
            self.killed = False
            self.log = logs / f"{load}_{kind}{'_' + tag if tag else ''}.log"

        def start(self, core):
            self.core, self.t0 = core, time.time()
            argv = list(self.argv)
            if self.wrapper.endswith("run_kit.sh"):     # every Kit session ends by the deadline
                argv += ["--max-wall-seconds", str(int(max(120, t_end - time.time() + 300))), "--headless"]
            cmd = ["docker", "exec", "-w", "/workspace/dishsim", "-e", "OMP_NUM_THREADS=2", "dishsim-isaac", "taskset",
                   "-c", f"{core},{core + 16}", "nice", "-n", "10", self.wrapper, *argv]
            self.proc = subprocess.Popen(cmd, cwd=ROOT, stdout=self.log.open("w"), stderr=subprocess.STDOUT,
                                         start_new_session=True)
            say(f"start {self.kind} {self.load} {self.tag} core {core}")

        def kill(self):
            if self.killed:
                return False
            self.killed = True
            if self.pattern:
                subprocess.run(["docker", "exec", "dishsim-isaac", "pkill", "-KILL", "-f", "--", self.pattern],
                               capture_output=True)
            if self.proc is not None and self.proc.poll() is None:
                self.proc.kill()
            return True

    def kit_job(kind, lid, extra, tag=""):
        pattern = f"--load {lid} " + "".join(f"{x} " for x in extra) + "--max-wall-seconds"
        return Job(kind, lid, base + ["--load", lid, *extra], tag, pattern=pattern)

    def covered(lid):
        """Library indices queued or running for ``lid``."""
        return {k for j in kit_q + running if j.load == lid and j.kind in ("library", "fresh")
                for k in map(int, j.tag.split("-"))}

    def cancel(lid, why):
        kit_q[:] = [j for j in kit_q if j.load != lid]
        for j in running:
            if j.load == lid and j.wrapper.endswith("run_kit.sh") and j.kill():
                say(f"killed {j.kind} {lid} {j.tag}: {why}")

    def advance(lid):
        """Queue what ``lid`` needs next from its record and library outcomes on disk (also the resume)."""
        s, path = st[lid], ROOT / out / f"{lid}.json"
        if not path.is_file() or s["stage"] in ("valid", "invalid", "failed", "error"):
            return
        dep = json.loads(path.read_text())
        s["backtracks"] = len((dep.get("reposing") or {}).get("backtracks") or [])
        if dep.get("status") != "ok":
            s.update(stage="failed", reason=dep.get("reason"), finished=time.time())
            say(f"{lid} FAILED in the sampler: {dep.get('reason')}")
            return
        for lib in dep["library"]:
            p = ROOT / out / "_kit" / lid / "library" / f"{lib['index']}.json"
            if lib.get("outcome") is None and p.is_file():
                lib["outcome"] = json.loads(p.read_text())["outcome"]
        if not dep.get("learning"):
            dep["learning"] = {"rounds": []}
        dep["learning"]["max_rounds"] = args.learn_rounds              # (the verdict needs the cap)
        v = verdict(dep)
        s["orders"] = f"{v['as_predicted']}/{v['built']} built of {v['orders']}"
        s["rounds"] = v["rounds"]
        if v["stage"] == "invalid":
            s.update(stage="invalid", reason=v["reason"], finished=time.time())
            say(f"{lid} INVALID: {v['reason']}")
            cancel(lid, "invalid")
            return
        missing = [lib["index"] for lib in dep["library"] if lib.get("outcome") is None]
        if missing:
            new = [k for k in missing if k not in covered(lid)]
            for c in range(0, len(new), SAMPLE_CHUNK):
                ks = new[c:c + SAMPLE_CHUNK]
                kind = "fresh" if dep["library"][ks[0]]["kind"] in ("refined_obeying", "learned_obeying") else "library"
                kit_q.append(kit_job(kind, lid, ["--order-index", ",".join(map(str, ks))], "-".join(map(str, ks))))
            s["stage"] = ("learn" if (dep.get("learning") or {}).get("rounds") else "fresh" if dep.get("refinement")
                          else "library")
            if new:
                say(f"{lid} queued {len(new)} {s['stage']} orders in {math.ceil(len(new) / SAMPLE_CHUNK)} sessions")
            return
        if any(j.load == lid for j in kit_q + running):
            return
        m = subprocess.run(["code/util/run_py.sh", *base, "--load", lid, "--merge-library"], cwd=ROOT,
                           capture_output=True, text=True, timeout=1200)
        (logs / f"{lid}_merge.log").open("a").write(m.stdout + m.stderr)
        if "[RESULT] PASS" not in m.stdout or "Traceback" in m.stdout + m.stderr:
            s.update(stage="error", reason=f"merge failed: see {rel(logs / (lid + '_merge.log'))}", finished=time.time())
            flags["halt"] = True
            say(f"ERROR merge {lid}: see {rel(logs / (lid + '_merge.log'))}; HALT (no new launches)")
            return
        dep = json.loads(path.read_text())
        v = dep.get("verdict") or {}
        s["rounds"] = v.get("rounds")
        say(f"{lid} merged: verdict {v.get('stage')}: {v.get('reason')}")
        if v.get("stage") in ("valid", "invalid"):     # clips and S for every built sample (valid ones first)
            s.update(stage=v["stage"], reason=v["reason"], finished=time.time())
            clips = ["code/planner/frigidaire/frigidaire_dependency_clips.py", "--record", f"{out}/{lid}.json", "--out",
                     f"{media}/{lid}"]
            score = ["code/planner/frigidaire/frigidaire_dependency_score.py", "--dependency-dir", out, "--loads", lid]
            jobs = [Job("clips", lid, clips, wrapper="code/util/run_py.sh", pattern=" ".join(clips)),
                    Job("score", lid, score, wrapper="code/util/run_py.sh", pattern=" ".join(score))]
            if v["stage"] == "valid":
                py_q[:0] = jobs
            else:
                py_q.extend(jobs)
        elif v.get("stage") == "refine" and not dep.get("refinement"):
            s["stage"] = "refine"
            kit_q.append(kit_job("refine", lid, ["--refine"]))
        elif v.get("stage") == "pending":               # a learning round appended fresh orders: queue them
            return advance(lid)
        else:
            s.update(stage="invalid", reason=v.get("reason"), finished=time.time())

    for lid in todo:                                         # resume: records already on disk
        if (ROOT / out / f"{lid}.json").is_file():
            st[lid]["started"] = time.time()
            advance(lid)
    say(f"--sample up: {len(todo)} packings ({' .. '.join(todo[::max(len(todo) - 1, 1)]) or '-'}), target {args.target_valid} valid, deadline "
        f"{datetime.fromtimestamp(t_end, timezone.utc):%H:%M:%SZ}; resumed "
        f"{sum(s['started'] is not None for s in st.values())}")
    while True:
        now = time.time()
        for j in list(running):
            if j.proc.poll() is None:
                continue
            running.remove(j)
            (free_kit if j.core in KIT_CORES else free_py).append(j.core)
            ok, text = judge_log(j.log)
            res = [ln for ln in text.splitlines() if ln.startswith("[RESULT]")]
            say(f"done {j.kind} {j.load} {j.tag} ({(now - j.t0) / 60:.1f} min) log_ok {ok}: {res[-1] if res else 'no RESULT'}")
            s = st[j.load]
            if s["stage"] in ("invalid", "failed", "killed") and j.kind not in ("clips", "score"):
                continue
            if not ok:
                if j.kind in ("clips", "score"):
                    say(f"ERROR {j.kind} {j.load}: see {rel(j.log)} (the page needs it)")
                    continue
                s.update(stage="error", reason=f"{j.kind} {j.tag} failed: see {rel(j.log)}", finished=now)
                flags["halt"] = True
                say(f"ERROR {j.kind} {j.load} {j.tag}: see {rel(j.log)}; HALT (no new launches)")
                continue
            if j.kind in ("clips", "score"):
                s[j.kind] = True
                continue
            advance(j.load)
        valid = [k for k, s in st.items() if s["stage"] == "valid"]
        done_target = len(valid) >= args.target_valid
        if done_target or now >= t_end:
            why = f"{len(valid)} valid" if done_target else "deadline"
            for j in [x for x in running if x.wrapper.endswith("run_kit.sh") or now >= t_end]:
                if j.kill():
                    st[j.load]["stage"] = st[j.load]["stage"] if st[j.load]["stage"] == "valid" else "killed"
                    say(f"killed {j.kind} {j.load} {j.tag}: {why}")
            kit_q.clear()
            if now >= t_end:
                py_q.clear()
            for k, s in st.items():                              # in progress when the run stopped
                if s["started"] is not None and s["stage"] in ("main", "library", "refine", "fresh", "learn"):
                    s.update(stage="killed", reason=f"stopped ({why}) while in stage {s['stage']}", finished=now)
        if not flags["halt"] and not done_target and now < t_end and free_kit and now - flags["last_kit"] >= SAMPLE_STAGGER_S:
            job = next(iter(sorted((j for j in kit_q if j.kind != "main"), key=lambda j: (todo.index(j.load), j.seq))),
                       None)
            if job is None:
                fresh_lid = next((k for k in todo if st[k]["stage"] == "queued" and eligible(k)), None)
                if fresh_lid is not None:
                    st[fresh_lid].update(stage="main", started=now)
                    job = kit_job("main", fresh_lid, [])
                    kit_q.append(job)
            if job is not None:
                kit_q.remove(job)
                free_kit.sort()
                job.start(free_kit.pop(0))
                running.append(job)
                flags["last_kit"] = now
        while py_q and free_py:
            free_py.sort()
            job = py_q.pop(0)
            job.start(free_py.pop(0))
            running.append(job)
        save_state()
        active = running or kit_q or py_q
        if first is not None or resumed:                     # (a packing the first run built is skipped; --resume-from:
            for k in todo:                                   # new ones once a resumed sample is not valid)
                if st[k]["stage"] == "queued":
                    eligible(k)
        resolved = all(s["stage"] in ("valid", "invalid", "failed", "error", "killed", "skipped") for s in st.values())
        if not active and (done_target or resolved or flags["halt"] or now >= t_end):
            say(f"[RESULT] {'PASS' if done_target else 'STOP'} --sample: {len(valid)} valid of "
                f"{sum(s['started'] is not None for s in st.values())} attempted ({', '.join(valid) or 'none'}); "
                + ", ".join(f"{k} {s['stage']}" for k, s in st.items() if s["started"] is not None))
            return
        time.sleep(5)


def gate_failed(args, source, entries, gate, out_path, report, status="gate", reason=None, **fields):
    """A record without a graph: status 'gate' (the load failed its joint gate; no Kit boot) or, with --goal built,
    'resettle_failed' (the goal did not build one dish at a time; ``fields`` = what the run measured)."""
    info = gate_info(args, gate) if gate is not None else None
    reason = reason or f"joint gate {info['outcome']}: {info['reason']}"
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "status": status, "reason": reason,
           "builds": [], "learned": [], "support": [], "neighbour": [], "goal_order": [], "order": [],
           "entries": entries, "targets_source": "joint_gate", "targets_world": {}, "poses_world": {},
           "entries_settled": [], "built_entries": [], "frames": None, "n_waves": None, "n_waves_dependencies": None,
           "acyclic": None, "failed_dish": None, "landing_rule": None, "gate": info, "kit": None,
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    if args.edges == "pairs":     # the pairs record's keys, empty (no build classes)
        dep = {k: v for k, v in dep.items() if k not in ("learned", "support", "neighbour", "n_waves_dependencies")}
        dep.update(pairs_tested=[], tests=[], edges=[], cycles=[], extra_constraints=[], waves=None, thresholds=None,
                   perturbation=perturbation(args))
    if args.goal == "built":
        dep.update(goal_source="sequential_build", goal_delta=None, resettle_builds=[], resettle_initial=[],
                   resettle_rule=None, original=None)
    dep.update(fields)
    write_record(args, out_path, dep)
    report.update(result="PASS", status=status, dependency=rel(out_path), reason=f"status {status} {reason}")


def write_manifest(args):
    """--manifest-out: the load's joint-gate manifest, frigidaire_hotec_top5.manifest_of(entries) (rack-local poses,
    the benchmark baseline = the default --baseline, the HOTEC v2 tableware). Kit-free: isaaclab is never imported."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("top5", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)    # Kit-free at import: numpy, dishsim and a private copy of frigidaire_bench
    try:
        if args.edges == "physical":           # the built poses (plans/2026-10-16-physical-cause-graph.md), rack-local in
            path = record_folder(args) / f"{args.load}.json"     # the baseline's extended racks
            dep = json.loads(path.read_text()) if path.is_file() else {}
            if dep.get("status") != "ok" or not dep.get("entries_settled"):
                raise Refused(f"no built --edges physical record at {rel(path)}")
            entries = dep["entries_settled"]
        else:
            _, entries = read_load(args)
    except Refused as exc:
        print(f"[RESULT] FAIL {args.load} {exc}", flush=True)
        return
    args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_out.write_text(json.dumps(T.manifest_of(entries), indent=1) + "\n")
    print(f"[RESULT] PASS manifest {rel(args.manifest_out)}", flush=True)


# --------------------------------------------------------------------------- --edges physical

class ForceLog:
    """Per ordered body pair (sensor, filter) of the backend's contact view, the net contact normal force sum(f n) of its
    rows at the last tick they were reported (world N, sign convention as PhysX reports it; support_candidates calibrates
    it): wraps the backend's contact tracker. A sleeping pair is no longer reported, and the tracker keeps it only while
    neither surface moved, so a resting pair's last rows are its resting force."""

    def __init__(self, backend):
        import numpy as np
        self.raw = {}
        tracker = backend.tracker
        names, update = tracker.names, tracker.update

        def logged(data, poses):
            out = update(data, poses)
            arr = [v.detach().cpu().numpy() if hasattr(v, "detach") else np.asarray(v) for v in data]
            f, n = arr[0].reshape(-1), arr[2].reshape(-1, 3)
            counts = arr[4].astype(np.int64).reshape(len(names), -1)
            starts = arr[5].astype(np.int64).reshape(len(names), -1)
            for i, j in np.argwhere(counts > 0):
                if i != j:
                    b, c = starts[i, j], counts[i, j]
                    self.raw[(names[i], names[j])] = (f[b:b + c, None] * n[b:b + c]).sum(axis=0)
            return out
        tracker.update = logged


def support_candidates(raw, pairs, poses, ids, racks=("LowerRack", "UpperRack", "SilverwareBasket")):
    """(supports {b: [a...]}, contacts [{pair, on_b_from_a_n, on_a_from_b_n}], calibration) of the finished load, standard
    library only. ``raw`` = ForceLog.raw, ``pairs`` = the resting contact pairs (sorted name tuples), ``poses`` = the final
    world poses. The sign that turns a row into the force ON its sensor is calibrated on the dish-rack contacts (a rack
    pushes its dishes up): calibration {sign, agree, n, rule}; with no such contact or fewer than CALIBRATE_MIN agreeing,
    rule 'centre' (planner.support_edges: the dish whose centre is 5 mm higher rests on the other) decides instead of the
    force. Rule 'force': a supports b when the force on b from a has an up (world z) component above FORCE_MIN_N."""
    dish = set(ids)
    votes = [float(v[2]) for (i, j), v in raw.items() if i in dish and j in racks and tuple(sorted((i, j))) in pairs]
    votes += [-float(v[2]) for (i, j), v in raw.items() if j in dish and i in racks and tuple(sorted((i, j))) in pairs]
    sign = 1. if sum(votes) >= 0 else -1.
    agree = sum(sign * v > 0 for v in votes) / len(votes) if votes else 0.
    cal = {"sign": sign, "agree": round(agree, 3), "n": len(votes),
           "rule": "force" if votes and agree >= CALIBRATE_MIN else "centre"}

    def force_on(b, a):                                       # on b from a [N]
        if (b, a) in raw:
            return [sign * float(x) for x in raw[(b, a)]]
        if (a, b) in raw:
            return [-sign * float(x) for x in raw[(a, b)]]
        return None
    supports, contacts = {i: [] for i in ids}, []
    for a, b in sorted(p for p in pairs if p[0] in dish and p[1] in dish and p[0] != p[1]):
        fb, fa = force_on(b, a), force_on(a, b)
        contacts.append({"pair": [a, b], "on_b_from_a_n": fb and [round(x, 4) for x in fb],
                         "on_a_from_b_n": fa and [round(x, 4) for x in fa]})
        if cal["rule"] == "force":
            if fb is not None and fb[2] > FORCE_MIN_N:
                supports[b].append(a)
            if fa is not None and fa[2] > FORCE_MIN_N:
                supports[a].append(b)
        else:                                                 # planner.SUPPORT_MIN_DZ_M = 5 mm
            dz = float(poses[b]["position_m"][2]) - float(poses[a]["position_m"][2])
            if dz > .005:
                supports[b].append(a)
            elif -dz > .005:
                supports[a].append(b)
    return {b: sorted(s) for b, s in supports.items() if s}, contacts, cal


def access_hits(B, checker, kinds, racks, moves, poses, ids):
    """(hits, paths) of the finished load (Kit-free, FCL): per dish a, its insertion path (insertion_release against the
    appliance alone at the final ``poses``: the start down to the lowest clear step), then every other dish b whose final
    pose fails the checker's 1 mm allowance against a at some step: hit {a, b, step, steps, lift_mm, start_mm, pose (a's
    world pose at the first such step)}, i.e. a must go in before b. paths {a: start_mm, steps, stop, blocked}."""
    appliance = B.appliance_bodies(checker, poses)
    bodies = {i: checker._body(kinds[i], poses[i], i) for i in ids}
    hits, paths = [], {}
    for a in ids:
        up, rim_z, top_z = B.rack_heights(poses, racks[a])
        m = B.insertion_release(checker, [(kinds[a], moves[a], a)], [], appliance, up, rim_z, top_z)
        clear = [] if m["blocked"] == "start" else m["lifts"][:-1] if m["blockers"] else m["lifts"]
        paths[a] = {"start_mm": round(m["start"] * 1e3, 1), "steps": len(clear), "stop": m["stop"],
                    "blocked": m["blocked"], "lowest_mm": None if m["lowest"] is None else round(m["lowest"] * 1e3, 1)}
        left = [b for b in ids if b != a]
        for k, h in enumerate(clear):
            pose = B.lifted(moves[a], h, up)
            body = checker._body(kinds[a], pose, a)
            for b in list(left):
                if not checker.pair(body, bodies[b])["valid"]:
                    hits.append({"a": a, "b": b, "step": k + 1, "steps": len(clear), "lift_mm": round(h * 1e3, 1),
                                 "start_mm": paths[a]["start_mm"], "pose": pose})
                    left.remove(b)
            if not left:
                break
    return hits, paths


def physical_graph(order, confirmations, hits):
    """The drawn graph of a certified load (plans/2026-10-16-physical-cause-graph.md; only HARD requirements, coordinator
    2026-10-16), standard library only. Support: a -> b for each a in a confirmation's ``hard`` (b fails without a: its
    only support removed, or with several, that one removed and the others in, while it lands with them all); a dish whose
    supports are needed only as a disjunction (outcome disjunctive: it fails without all of them, no single one is
    necessary) gets no edge, kept under ``disjunctions`` {b, supports, suffice}. Access: a -> b for every hit (b's final
    pose covers a's insertion path). A pair with arcs both ways loses both (conflicts: nothing is claimed about it); an edge
    the witness order breaks is not drawn (against_witness: the witness built the load that way). So every drawn edge
    follows the witness order and the graph is a DAG: layers = 0 for a dish with no prerequisite, else 1 + the deepest
    prerequisite's layer. Edges {a, b, kind support | access, conf | hit (index)}: a must be in place before b."""
    pos = {i: n for n, i in enumerate(order)}
    support = {(a, c["dish"]): k for k, c in enumerate(confirmations) if c.get("outcome") == "hard" for a in c["hard"]}
    access = {(h["a"], h["b"]): k for k, h in enumerate(hits)}
    arcs = set(support) | set(access)
    conflicts = sorted({tuple(sorted(p)) for p in arcs if p[::-1] in arcs})
    gone = {p for c in conflicts for p in (c, c[::-1])}
    edges, against = [], []
    for kind, found, key in (("support", support, "conf"), ("access", access, "hit")):
        for (a, b), k in sorted(found.items(), key=lambda x: (pos[x[0][1]], pos[x[0][0]])):
            if (a, b) in gone:
                continue
            (against if pos[a] > pos[b] else edges).append({"a": a, "b": b, "kind": kind, key: k})
    used = {i for e in edges for i in (e["a"], e["b"])}
    layer = {}
    for i in order:                                          # every drawn edge goes forward in the witness order
        layer[i] = 1 + max((layer[e["a"]] for e in edges if e["b"] == i), default=-1)
    counts = {"support": sum(e["kind"] == "support" for e in edges), "access": sum(e["kind"] == "access" for e in edges),
              "conflicts": len(conflicts), "against_witness": len(against),
              "disjunctive": sum(c.get("outcome") == "disjunctive" for c in confirmations)}
    return {"edges": edges, "conflicts": [list(c) for c in conflicts], "against_witness": against,
            "disjunctions": [{"b": c["dish"], "supports": c["supports"], "suffice": c.get("suffice") or []}
                             for c in confirmations if c.get("outcome") == "disjunctive"],
            "free": [i for i in order if i not in used], "layers": layer, "counts": counts}


def physical_gate(gate_dir, dep):
    """The record's joint-gate numbers from ``gate_dir``/result.json (frigidaire_initial_state_validate.py on the built
    poses); a lower-first rerun also names the upper-first outcome. Standard library only."""
    path = Path(gate_dir) / "result.json"
    if not path.is_file():
        raise Refused(f"no joint gate result {rel(path)}")
    g = json.loads(path.read_text())
    planned = {o["object_id"]: [round(float(v), 6) for v in o["rack_local_pose"]["position_m"]]
               for o in g.get("candidate_objects") or []}
    if planned and planned != {e["id"]: [round(float(v), 6) for v in e["position"]] for e in dep["entries_settled"]}:
        raise Refused(f"the joint gate {rel(gate_dir)} ran on other poses than this record's built goal")
    mm = lambda v: None if v is None else round(float(v) * 1e3, 2)                 # noqa: E731
    settled, lim = g.get("settled") or {}, g.get("limits") or {}
    out = {"dir": rel(gate_dir), "outcome": g.get("outcome"), "reason": g.get("reason") or g.get("error"),
           "order": g.get("order"), "peak_limit_mm": mm(lim.get("peak_penetration_m")),
           "median_limit_mm": mm(lim.get("median_max_penetration_m")),
           "settle_peak_mm": mm(g.get("maximum_settle_penetration_m")),
           "cycle_peak_mm": mm(g.get("maximum_cycle_penetration_m")),
           "window_peak_mm": mm(settled.get("peak_penetration_m")),
           "window_median_mm": mm(settled.get("median_max_penetration_m")),
           "contained": (all(v["contained"] for v in g["final_containment"].values())
                         if g.get("final_containment") else None),
           "wall_seconds": round(float(g.get("wall_seconds") or 0.), 1)}
    first = Path(str(gate_dir)[:-len("_lower_first")]) / "result.json" if str(gate_dir).endswith("_lower_first") else None
    if first is not None and first.is_file():
        f = json.loads(first.read_text())
        out["first_try"] = {"order": f.get("order"), "outcome": f.get("outcome"), "reason": f.get("reason")}
    return out


def physical_certify(args, source, entries, gate, out_path, kit_dir, report, started):
    """--certify (module docstring, --edges physical): the record's gate, the fresh-process rebuild of its witness, the
    support candidates from the rebuild's resting contact forces, the access hits, the support confirmation drops and the
    graph, all written into the record."""
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS
    dep = json.loads(out_path.read_text()) if out_path.is_file() else {}
    if dep.get("procedure") != "physical" or dep.get("status") != "ok":
        raise Refused(f"no built --edges physical record at {rel(out_path)}")
    if not args.gate:
        raise Refused("--certify needs --gate DIR (the joint gate of the built poses)")
    args.force = True
    dep["gate"] = ginfo = physical_gate(args.gate, dep)
    if ginfo["outcome"] != "accepted":
        dep.update(valid=False, verdict={"stage": "invalid", "reason": f"joint gate {ginfo['outcome']}: {ginfo['reason']}"})
        write_record(args, out_path, dep)
        report.update(result="PASS", status="gate", dependency=rel(out_path), reason=dep["verdict"]["reason"])
        return
    tol_name = dep.get("tolerance") or "bench"
    table, climit, tol_text = tolerance_of(B, tol_name, LIMITS["peak_penetration_m"])
    rule = f"at-goal tolerance of the sequentially built pose ({tol_name}: {tol_text}), {landing_tail(dep['release'], climit)}"
    snapshot = goal_setup(args, entries, None, B, K, P)[0]
    chosen = dep["entries_chosen"]
    by_id = {e["id"]: e for e in chosen}
    ids, order, moves = list(by_id), list(dep["witness"]["order"]), dep["moves_world"]
    kind, rack = {i: by_id[i]["kind"] for i in ids}, {i: by_id[i]["rack"] for i in ids}
    centroids = B.kind_centroids({k: visual_points(B.ASSETS / f"{k}.usda") for k in B.KINDS})
    backend = K._sequence_scene(args, B, chosen, snapshot, kit_dir / "certify_scene", report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    forces = ForceLog(backend)
    rec = Recorder(backend, ids, kit_dir / "traj")
    t0 = time.monotonic()
    print(f"certify {args.load}: gate {ginfo['outcome']} ({ginfo['order']}, settle peak {ginfo['settle_peak_mm']} mm, cycle "
          f"peak {ginfo['cycle_peak_mm']} mm); rebuilding the witness {' '.join(order)}; landing = {rule}", flush=True)
    rec.begin(empty_resets=True)
    status, rb, _ = K.build_sequence(backend, B, P, by_id, dep["targets_world"], centroids,
                                     lambda k_, dist, dev: B.within_goal(k_, dist, table), fixed=order,
                                     release="insertion", on_path=rec.descent, moves=moves, goal_table=table,
                                     contact_m=climit)
    final = backend.poses()
    last = rb["builds"][-1]
    certified = status == "ok"
    cert = {"status": status, "certified": certified, "dish": last["dish"], "moved": last["moved"],
            "placed": len(last["steps"]) - (1 if last["dish"] else 0), "reason": rb["reason"], "steps": last["steps"],
            "traj": rec.save("certify_witness", order=order, status=status), "tolerance": tol_name,
            "poses_world": {i: final[i] for i in ids} if certified else None,
            "rebuild_seconds": round(time.monotonic() - t0, 1)}
    print(f"certify {args.load}: rebuild {status} {last['dish'] or ''} {last['moved'] or ''} ({cert['placed']} of "
          f"{len(ids)} landed, {cert['rebuild_seconds']:.0f} s): {rb['reason']}", flush=True)
    thresholds = {"tolerance": tol_name, "contact_limit_mm": round(climit * 1e3, 3),
                  "at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3), "tilt_deg": v[2]}
                              for k, v in table.items()}, "force_min_n": FORCE_MIN_N}
    dep.update(certify=cert, landing_rule=rule, thresholds=thresholds)
    if not certified:
        dep.update(valid=False, verdict={"stage": "invalid", "reason": (
            f"the witness did not rebuild in a fresh Kit process: {status} at {last['dish']}"
            + (f" (moved {', '.join(last['moved'])})" if last["moved"] else "") + f", {cert['placed']} of {len(ids)} landed")})
        write_record(args, out_path, dep)
        report.update(result="PASS", status="not_certified", dependency=rel(out_path), reason=dep["verdict"]["reason"])
        return
    pairs = set()
    for _ in range(30):                                      # the resting contacts (they flicker: a union over 0.25 s)
        backend.tick()
        pairs |= {tuple(p) for p in backend.latest_contact["pairs"]}
    supports, contacts, cal = support_candidates(forces.raw, pairs, final, ids)
    print(f"certify {args.load}: {len(contacts)} dish-dish contacts, force sign {cal['sign']:+.0f} from {cal['n']} dish-rack "
          f"contacts ({cal['agree']:.0%} agree, rule {cal['rule']}); support candidates "
          + ("; ".join(f"{b} on {'+'.join(s)}" for b, s in supports.items()) or "none"), flush=True)
    checker = B.release_checker()
    t1 = time.monotonic()
    hits, paths = access_hits(B, checker, kind, rack, moves, final, ids)
    print(f"certify {args.load}: {len(hits)} access hits ({time.monotonic() - t1:.0f} s): "
          + ("; ".join(f"{h['a']} before {h['b']} (step {h['step']} of {h['steps']}, {h['lift_mm']} mm)" for h in hits)
             or "none"), flush=True)
    cover, resting = {}, {}
    for h in hits:
        cover.setdefault(h["a"], set()).add(h["b"])
    for b, s in supports.items():
        for a in s:
            resting.setdefault(a, set()).add(b)

    def above(start):                                        # everything resting, transitively, on ``start``
        out, todo = set(), list(start)
        while todo:
            for y in resting.get(todo.pop(), ()):
                if y not in out:
                    out.add(y)
                    todo.append(y)
        return out
    park = {i: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, i in enumerate(by_id)}
    goals = {i: P.pose_T(final[i]) for i in ids}

    def settle(absent):
        """Park ``absent``, every other dish at its final pose, 150 ticks; a dish then outside its tolerance of its final
        pose has fallen: parked too, re-settled (FALL_ROUNDS at most). (poses, absent, fallen per round)."""
        absent, rounds = set(absent), []
        while True:
            for i in ids:
                backend.set_rigid_pose(backend.objects[i], park[i] if i in absent else final[i])
            for _ in range(K.SETTLE_TICKS):
                backend.tick()
            now = backend.poses()
            fallen = sorted(i for i in ids if i not in absent
                            and B.off_goal(kind[i], centroids[kind[i]], P.pose_T(now[i]), goals[i], table))
            rounds.append(fallen)
            if not fallen or len(rounds) >= FALL_ROUNDS:
                return now, absent, rounds
            absent |= set(fallen)

    def drop(b, removed, name):
        """b by the move into the finished load without ``removed``: {landed, lateral_mm, dz_mm, tilt_deg, contact_mm
        (from b's final pose), disturbed, blocked, ...}; its ticks to traj/<name>.json."""
        now, absent, rounds = settle({b, *removed})
        ctx = [i for i in ids if i not in absent]
        bodies = [checker._body(kind[i], now[i], i) for i in ctx]
        up, rim_z, top_z = B.rack_heights(now, rack[b])
        m = B.insertion_release(checker, [(kind[b], moves[b], b)], bodies, B.appliance_bodies(checker, now), up, rim_z,
                                top_z)
        out = {"dish": b, "removed": sorted(removed), "fallen": sorted(absent - {b, *removed}), "rounds": rounds,
               **B.path_info(m)}
        rec.begin()
        rec.descent([b], [[B.lifted(moves[b], h, up)] for h in B.shown_lifts(m)], now)
        if m["poses"] is None:                                # a context dish (or the start) blocks the descent
            out.update(landed=False, blocked_by=m["blockers"], disturbed=[],
                       traj=rec.save(name, dish=b, path_blocked=True, blockers=m["blockers"], removed=sorted(removed)))
            return out
        pre = {i: P.pose_T(now[i]) for i in ctx}
        backend.set_rigid_pose(backend.objects[b], m["poses"][0])
        for _ in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
        out["traj"] = rec.save(name, dish=b, release=m["poses"][0], removed=sorted(removed))
        now = backend.poses()
        d = B.goal_distance(centroids[kind[b]], P.pose_T(now[b]), goals[b])
        deep = float(max((v for k_, v in backend.latest_contact["depths_m"].items() if b in k_.split("|")), default=0.))
        out.update(landed=bool(B.within_goal(kind[b], d, table) and deep < climit), lateral_mm=round(d[0] * 1e3, 1),
                   dz_mm=round(d[1] * 1e3, 1), tilt_deg=round(d[2], 1), contact_mm=round(deep * 1e3, 2),
                   disturbed=sorted(i for i in ctx if B.off_goal(kind[i], centroids[kind[i]], P.pose_T(now[i]), pre[i],
                                                                 table)), pose=now[b])
        return out

    def said(x):
        return ("blocked by " + ", ".join(x.get("blocked_by") or []) if x.get("blocked_by") is not None else
                f"{'LANDS' if x['landed'] else 'fails'} {x['lateral_mm']} mm / {x['dz_mm']} mm / {x['tilt_deg']} deg, "
                f"contact {x['contact_mm']} mm") + (f", fell {x['fallen']}" if x["fallen"] else "")
    confirmations = []
    for b in [i for i in order if i in supports]:
        S, cov = supports[b], cover.get(b, set())

        def removed_for(present):                            # (closes over this b's S and cov)
            gone = set(S) - set(present)
            return sorted((gone | cov | above({b} | gone)) - {b} - set(present))
        c = {"dish": b, "supports": S, "kind": "plain" if len(S) == 1 else "several", "coverers": sorted(cov),
             "without": drop(b, removed_for(()), f"conf_{b}_none"), "alone": {}, "all": None, "loo": {}, "hard": [],
             "suffice": []}
        print(f"  confirm {b} on {'+'.join(S)}: without them {said(c['without'])} "
              f"({time.monotonic() - t0:.0f} s)", flush=True)
        if c["without"]["landed"]:
            c["outcome"] = "free"                            # b lands without its supports: no physical cause found
        elif c["without"].get("blocked_by") is not None:
            c["outcome"] = "blocked"
        else:
            for a in S:                                      # each support alone present: which ones suffice
                c["alone"][a] = drop(b, removed_for((a,)), f"conf_{b}_{a}")
                print(f"  confirm {b} with {a} alone: {said(c['alone'][a])}", flush=True)
            c["suffice"] = [a for a in S if c["alone"][a]["landed"]]
            if len(S) == 1:                                  # hard: b fails without a and lands with it
                c["hard"] = list(c["suffice"])
                c["outcome"] = "hard" if c["hard"] else "control_failed"
            else:                                            # hard: b fails without that support alone (the others in)
                c["all"] = drop(b, removed_for(S), f"conf_{b}_all")
                print(f"  confirm {b} with all of {'+'.join(S)}: {said(c['all'])}", flush=True)
                for a in S:
                    rest = [x for x in S if x != a]
                    c["loo"][a] = (c["alone"][rest[0]] if len(rest) == 1 else
                                   drop(b, removed_for(rest), f"conf_{b}_no_{a}"))
                    if len(rest) > 1:
                        print(f"  confirm {b} without {a} only: {said(c['loo'][a])}", flush=True)
                c["hard"] = [a for a in S if not c["loo"][a]["landed"] and c["loo"][a].get("blocked_by") is None]
                c["outcome"] = ("control_failed" if not c["all"]["landed"] else "hard" if c["hard"]
                                else "disjunctive")          # needs one of c["suffice"] (or several): not drawn
        confirmations.append(c)
    graph = physical_graph(order, confirmations, hits)
    n = graph["counts"]
    dep.update(contacts={"pairs": contacts, "calibration": cal, "supports": supports}, access={"hits": hits, "paths": paths},
               confirmations=confirmations, graph=graph, valid=True,
               verdict={"stage": "valid", "reason": (
                   f"joint gate accepted ({ginfo['order']}), witness rebuilt 24 of 24 in a fresh Kit process; "
                   f"{n['support']} hard support and {n['access']} access edge(s) drawn; {len(confirmations)} support "
                   f"candidate(s): " + ", ".join(f"{k} {sum(c['outcome'] == k for c in confirmations)}" for k in
                                                 ("hard", "disjunctive", "free", "blocked", "control_failed")))})
    dep["kit"] = {**(dep.get("kit") or {}), "certify_wall_seconds": round(time.monotonic() - started, 1),
                  "confirmation_drops": len({x["traj"] for c in confirmations
                                             for x in [c["without"], c["all"], *c["alone"].values(), *c["loo"].values()]
                                             if x})}
    write_record(args, out_path, dep)
    report.update(result="PASS", status="valid", dependency=rel(out_path),
                  reason=f"{dep['verdict']['reason']}; conflicts {n['conflicts']}, against the witness "
                         f"{n['against_witness']}, free {len(graph['free'])}")


PHYSICAL_KEEP = ("load_id", "sampler", "source", "release", "goal_source", "goal_delta", "resettle_builds",
                 "resettle_initial", "resettle_rule", "original", "reposing", "moves_world", "entries_chosen", "witness",
                 "entries", "targets_source", "targets_world", "entries_settled", "frames")


def import_sample(src, out_dir, tolerance):
    """--sample --edges physical --resume-from: a built --edges swaps sample's sampler output (witness, built goal, moves,
    chosen entries; no swaps, library or verdict) as an --edges physical record in ``out_dir``, with the sampler's
    witness trajectory and S (.scores.json). Standard library only; an existing record is kept (False)."""
    import shutil
    src = Path(src)
    rec = json.loads(src.read_text())
    lid = rec["load_id"]
    dst = Path(out_dir) / f"{lid}.json"
    if dst.is_file():
        return False
    dep = {k: rec[k] for k in PHYSICAL_KEEP if k in rec}
    dep.update(procedure="physical", status="ok", reason=f"the re-posing sampler's build, imported from {rel(src)}",
               landing_rule=None, tolerance=tolerance, gate=None, certify=None, valid=None, imported_from=rel(src),
               kit={"resettle_wall_seconds": (rec.get("kit") or {}).get("resettle_wall_seconds")},
               generated_utc=datetime.now(timezone.utc).isoformat())
    traj = Path(out_dir) / "_kit" / lid / "traj"
    traj.mkdir(parents=True, exist_ok=True)
    w = src.parent / "_kit" / lid / "traj" / rec["witness"]["traj"]
    if w.is_file():
        shutil.copy2(w, traj / w.name)
    if (src.parent / f"{lid}.scores.json").is_file():
        shutil.copy2(src.parent / f"{lid}.scores.json", Path(out_dir) / f"{lid}.scores.json")
    tmp = dst.with_name(dst.name + ".tmp")
    tmp.write_text(json.dumps(dep, indent=1) + "\n")
    os.replace(tmp, dst)
    return True


def physical_loop(args):
    """--sample --edges physical (host python on the box, stdlib only; plans/2026-10-16-physical-cause-graph.md): the
    packings of sequential_fresh_order.txt from r07_00012 (at most --max-packings; --resume-from DIR imports that folder's
    built samples first and skips every packing their parent runs attempted). Per packing, Kit jobs: main (the sampler; not
    for an imported sample), gate (--manifest-out, then frigidaire_initial_state_validate.py at GATE_PEAK_M), gate_lower
    (the same lower rack first, once, on closure_failure), certify (one fresh Kit process, --gate the accepted dir); then
    clips (every certify that ran, valid first) and the score (valid, unscored) on cores 14-15. Kit jobs on cores 0-13, a
    packing's pipeline jobs before new packings, SAMPLE_STAGGER_S apart. Stops at --target-valid valid samples (Kit jobs
    killed, clips finish), when every packing is resolved, or --max-wall-seconds from now (everything killed). A Kit job
    is judged by its log (judge_log; a gate by its result.json); a failed one halts new launches. Log and state:
    data/logs/dot_rules/<--out-dir name>/, clips data/media/dot_rules/dependency_<name>/<load_id>/."""
    import shlex
    import subprocess
    t_end = time.time() + args.max_wall_seconds
    out = rel(args.out_dir)
    tag = Path(out).name
    logs, media = ROOT / "data/logs/dot_rules" / tag, f"data/media/dot_rules/dependency_{tag}"
    logs.mkdir(parents=True, exist_ok=True)
    (ROOT / out).mkdir(parents=True, exist_ok=True)
    order = (ROOT / "data/logs/dot_rules/sequential_fresh_order.txt").read_text().split()
    fresh = order[order.index(SAMPLE_PILOT):]
    todo, resumed = fresh[:args.max_packings], []
    if args.resume_from:
        rdir = ROOT / args.resume_from
        recs = {p.name[:-5]: json.loads(p.read_text()) for p in sorted(rdir.glob("r*.json"))
                if not p.name.endswith(".scores.json")}
        resumed = [k for k in order if (recs.get(k) or {}).get("status") == "ok"]
        seen = set(recs)
        for d in recs.values():
            src = (d.get("rejudged") or {}).get("from")
            if src:
                seen |= {p.name.split(".")[0] for p in (ROOT / src).parent.glob("r*.json")}
        todo = (resumed + [k for k in fresh if k not in seen])[:args.max_packings]
        for k in resumed:
            import_sample(rdir / f"{k}.json", ROOT / out, args.tolerance)
    kit = "code/planner/frigidaire/frigidaire_dependency_kit.py"
    base = [kit, "--sampler", "restricted", "--edges", "physical", "--release", "insertion", "--plate-order",
            args.plate_order, "--backtrack", str(args.backtrack), "--tolerance", args.tolerance, "--out-dir", out]
    st = {k: {"stage": "queued", "reason": None, "started": None, "finished": None, "resumed": k in resumed}
          for k in todo}
    kit_q, py_q, running, free_kit, free_py = [], [], [], list(KIT_CORES), list(PY_CORES)
    flags = {"halt": False, "last_kit": 0., "seq": 0}
    rank = {"certify": 0, "gate_lower": 1, "gate": 2, "main": 3}

    def say(msg):
        line = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()) + " " + msg
        with (logs / "scheduler.log").open("a") as f:
            f.write(line + "\n")
        print(line, flush=True)

    def save_state():
        (logs / "scheduler_state.json").write_text(json.dumps({
            "procedure": "physical", "updated_utc": datetime.now(timezone.utc).isoformat(),
            "deadline_utc": datetime.fromtimestamp(t_end, timezone.utc).isoformat(), "plate_order": args.plate_order,
            "tolerance": args.tolerance, "backtrack": args.backtrack, "resumed": resumed, "target_valid": args.target_valid,
            "max_packings": args.max_packings, "valid": [k for k, s in st.items() if s["stage"] == "valid"],
            "packings": st, "running": [[j.kind, j.load, j.core] for j in running],
            "queued": [[j.kind, j.load] for j in kit_q + py_q]}, indent=1) + "\n")

    class Job:
        def __init__(self, kind, load, argv, wrapper="code/util/run_kit.sh", pattern=None, deadline=False):
            flags["seq"] += 1
            self.kind, self.load, self.argv, self.wrapper, self.pattern = kind, load, argv, wrapper, pattern
            self.deadline, self.seq, self.proc, self.core, self.t0, self.killed = deadline, flags["seq"], None, None, None, False
            self.log = logs / f"{load}_{kind}.log"

        def start(self, core):
            self.core, self.t0 = core, time.time()
            argv = list(self.argv)
            if self.deadline:                               # a dependency-kit Kit session ends by the deadline
                argv += ["--max-wall-seconds", str(int(max(120, t_end - time.time() + 300))), "--headless"]
            cmd = ["docker", "exec", "-w", "/workspace/dishsim", "-e", "OMP_NUM_THREADS=2", "dishsim-isaac", "taskset",
                   "-c", f"{core},{core + 16}", "nice", "-n", "10", self.wrapper, *argv]
            self.proc = subprocess.Popen(cmd, cwd=ROOT, stdout=self.log.open("w"), stderr=subprocess.STDOUT,
                                         start_new_session=True)
            say(f"start {self.kind} {self.load} core {core}")

        def kill(self):
            if self.killed:
                return False
            self.killed = True
            if self.pattern:
                subprocess.run(["docker", "exec", "dishsim-isaac", "pkill", "-KILL", "-f", "--", self.pattern],
                               capture_output=True)
            if self.proc is not None and self.proc.poll() is None:
                self.proc.kill()
            return True

    def gdir(lid, lower=False):
        return f"{out}/_gate/{lid}" + ("_lower_first" if lower else "")

    def kit_job(kind, lid, extra=()):
        return Job(kind, lid, base + ["--load", lid, *extra], deadline=True,
                   pattern=f"--load {lid} " + "".join(f"{x} " for x in extra) + "--max-wall-seconds")

    def gate_job(lid, lower=False):
        g = gdir(lid, lower)
        val = ["code/initialization/frigidaire/frigidaire_initial_state_validate.py", "--manifest",
               f"{gdir(lid)}/manifest.json", "--out-dir", g, "--max-wall-seconds", "600", "--peak-penetration-m",
               str(GATE_PEAK_M), "--headless", "--device", "cpu"] + (["--order", "lower_first"] if lower else [])
        if lower:
            return Job("gate_lower", lid, val, pattern=f"--out-dir {g} ")
        man = ["code/util/run_py.sh", *base, "--load", lid, "--manifest-out", f"{g}/manifest.json"]
        cmd = " ".join(map(shlex.quote, man)) + " && code/util/run_kit.sh " + " ".join(map(shlex.quote, val))
        return Job("gate", lid, ["-c", cmd], wrapper="bash", pattern=f"--out-dir {g} ")

    def py_jobs(lid, valid):
        clips = ["code/planner/frigidaire/frigidaire_dependency_clips.py", "--record", f"{out}/{lid}.json", "--out",
                 f"{media}/{lid}"]
        jobs = [Job("clips", lid, clips, wrapper="code/util/run_py.sh", pattern=" ".join(clips))]
        if valid and not (ROOT / out / f"{lid}.scores.json").is_file():
            score = ["code/planner/frigidaire/frigidaire_dependency_score.py", "--dependency-dir", out, "--loads", lid]
            jobs.append(Job("score", lid, score, wrapper="code/util/run_py.sh", pattern=" ".join(score)))
        if valid:
            py_q[:0] = jobs
        else:
            py_q.extend(jobs)

    def record(lid):
        p = ROOT / out / f"{lid}.json"
        return json.loads(p.read_text()) if p.is_file() else {}

    def done(j, ok, text):
        """What ``j`` finishing means for its packing, and the next job."""
        lid, s = j.load, st[j.load]
        if j.kind in ("clips", "score"):
            s[j.kind] = ok
            if not ok:
                say(f"ERROR {j.kind} {lid}: see {rel(j.log)} (the page needs it)")
            return
        if s["stage"] in ("valid", "invalid", "failed", "killed", "error"):
            return
        if j.kind in ("gate", "gate_lower"):
            res = ROOT / gdir(lid, j.kind == "gate_lower") / "result.json"
            g = json.loads(res.read_text()) if res.is_file() else None
            if g is None:
                s.update(stage="error", reason=f"{j.kind}: no result.json (see {rel(j.log)})", finished=time.time())
                flags["halt"] = True
                say(f"ERROR {j.kind} {lid}: no result.json, see {rel(j.log)}; HALT (no new launches)")
                return
            s.setdefault("gates", []).append(g.get("outcome"))
            if g.get("outcome") == "accepted":
                s["stage"] = "certify"
                kit_q.append(kit_job("certify", lid, ["--certify", "--gate", gdir(lid, j.kind == "gate_lower")]))
            elif g.get("outcome") == "closure_failure" and j.kind == "gate":
                s["stage"] = "gate_lower"
                kit_q.append(gate_job(lid, lower=True))
            else:
                s.update(stage="invalid", reason=f"joint gate {g.get('outcome')}: {g.get('reason') or g.get('error')}",
                         finished=time.time())
            say(f"{lid} {j.kind}: {g.get('outcome')} -> {s['stage']}")
            return
        if not ok:
            s.update(stage="error", reason=f"{j.kind} failed: see {rel(j.log)}", finished=time.time())
            flags["halt"] = True
            say(f"ERROR {j.kind} {lid}: see {rel(j.log)}; HALT (no new launches)")
            return
        dep = record(lid)
        if j.kind == "main":
            if dep.get("status") != "ok":
                s.update(stage="failed", reason=dep.get("reason"), finished=time.time())
                say(f"{lid} FAILED in the sampler: {dep.get('reason')}")
                return
            s["stage"] = "gate"
            kit_q.append(gate_job(lid))
            say(f"{lid} built all {len(dep.get('targets_world') or {})}: gate queued")
            return
        cert = dep.get("certify") or {}                       # certify
        s.update(certified=cert.get("certified"), edges=(dep.get("graph") or {}).get("counts"),
                 gate=(dep.get("gate") or {}).get("outcome"), finished=time.time(),
                 stage="valid" if dep.get("valid") else "invalid", reason=(dep.get("verdict") or {}).get("reason"))
        say(f"{lid} {s['stage'].upper()}: {s['reason']}")
        if cert:
            py_jobs(lid, s["stage"] == "valid")

    decided = []
    for lid in todo:                                         # a restart: packings already decided on disk stay decided
        dep, gates = record(lid), [ROOT / gdir(lid, low) / "result.json" for low in (True, False)]
        g = next((json.loads(p.read_text()) for p in gates if p.is_file()), None)
        if dep.get("valid") is not None:
            st[lid].update(stage="valid" if dep["valid"] else "invalid", started=time.time(), finished=time.time(),
                           reason=(dep.get("verdict") or {}).get("reason"), gate=(dep.get("gate") or {}).get("outcome"),
                           certified=(dep.get("certify") or {}).get("certified"),
                           edges=(dep.get("graph") or {}).get("counts"))
        elif dep.get("status") not in (None, "ok"):
            st[lid].update(stage="failed", reason=dep.get("reason"), started=time.time(), finished=time.time())
        elif g is not None and g.get("outcome") not in ("accepted", "closure_failure"):
            st[lid].update(stage="invalid", started=time.time(), finished=time.time(), gates=[g.get("outcome")],
                           reason=f"joint gate {g.get('outcome')}: {g.get('reason') or g.get('error')}")
        else:
            continue
        decided.append(lid)
    for lid in resumed:
        if lid not in decided:
            st[lid].update(stage="gate", started=time.time())
            kit_q.append(gate_job(lid))
    dtext = ", ".join(k + " " + st[k]["stage"] for k in decided) or "-"
    say(f"--sample --edges physical up: {len(todo)} packings ({', '.join(todo)}), resumed {resumed or '-'}, decided on "
        f"disk {dtext}, target {args.target_valid} valid, deadline {datetime.fromtimestamp(t_end, timezone.utc):%H:%M:%SZ}")
    while True:
        now = time.time()
        for j in list(running):
            if j.proc.poll() is None:
                continue
            running.remove(j)
            (free_kit if j.core in KIT_CORES else free_py).append(j.core)
            ok, text = judge_log(j.log)
            res = [ln for ln in text.splitlines() if ln.startswith("[RESULT]")]
            say(f"done {j.kind} {j.load} ({(now - j.t0) / 60:.1f} min) log_ok {ok}: {res[-1] if res else 'no RESULT'}")
            if not j.killed:
                done(j, ok, text)
        valid = [k for k, s in st.items() if s["stage"] == "valid"]
        target = len(valid) >= args.target_valid
        if target or now >= t_end:
            why = f"{len(valid)} valid" if target else "deadline"
            for j in [x for x in running if x.deadline or x.kind.startswith("gate") or now >= t_end]:
                if j.kill():
                    say(f"killed {j.kind} {j.load}: {why}")
            kit_q.clear()
            if now >= t_end:
                py_q.clear()
            for k, s in st.items():
                if s["started"] is not None and s["stage"] in ("main", "gate", "gate_lower", "certify"):
                    s.update(stage="killed", reason=f"stopped ({why}) in stage {s['stage']}", finished=now)
        if not flags["halt"] and not target and now < t_end and free_kit and now - flags["last_kit"] >= SAMPLE_STAGGER_S:
            job = min(kit_q, key=lambda j: (rank[j.kind], todo.index(j.load), j.seq), default=None)
            if job is None:
                lid = next((k for k in todo if st[k]["stage"] == "queued"), None)
                if lid is not None:
                    st[lid].update(stage="main", started=now)
                    job = kit_job("main", lid)
                    kit_q.append(job)
            if job is not None:
                kit_q.remove(job)
                free_kit.sort()
                job.start(free_kit.pop(0))
                running.append(job)
                flags["last_kit"] = now
        while py_q and free_py and now < t_end:
            free_py.sort()
            job = py_q.pop(0)
            job.start(free_py.pop(0))
            running.append(job)
        save_state()
        resolved = all(s["stage"] in ("valid", "invalid", "failed", "error", "killed") for s in st.values())
        if not (running or kit_q or py_q) and (target or resolved or flags["halt"] or now >= t_end):
            n_att = sum(s["started"] is not None for s in st.values())
            say(f"[RESULT] {'PASS' if target else 'STOP'} --sample --edges physical: {len(valid)} valid of {n_att} "
                f"attempted ({', '.join(valid) or 'none'}); "
                + ", ".join(f"{k} {s['stage']}" for k, s in st.items() if s["started"] is not None))
            return
        time.sleep(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sampler", choices=("restricted", "random"), required=True)
    parser.add_argument("--load", "--loads", dest="load", metavar="ID", help="one load id per Kit process (required but with --sample)")
    parser.add_argument("--restricted-dir", type=Path, default=ROOT / "data/results/dot_rules/restricted/loads")
    parser.add_argument("--farm-dir", type=Path, default=ROOT / "data/results/random_loads/rl_20261009")
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out", type=Path, default=ROOT / "data/results/dot_rules/dependency")
    parser.add_argument("--out-dir", type=Path, metavar="PATH",
                        help="write <load_id>.json and _kit/<load_id>/ under PATH instead of the folder under --out")
    parser.add_argument("--gate", type=Path, metavar="DIR",
                        help="the load's finished joint gate (its --out-dir): build toward the jointly settled poses")
    parser.add_argument("--edges", choices=("pairs", "build", "swaps", "physical"), default="pairs",
                        help="pairs: the ordered-pair drop-test graph, <out>/pairs/ (default); "
                             "build: the 2026-10-13 build-learned graph, <out>/<sampler>/; swaps: the sequential "
                             "goal's swap-tested graph and its library, <out>/sequential/ (implies --goal built); "
                             "physical: the sampler only, then --manifest-out, the joint gate and --certify (support "
                             "and access edges of the finished load, plans/2026-10-16-physical-cause-graph.md)")
    parser.add_argument("--certify", action="store_true",
                        help="--edges physical (one fresh Kit process): rebuild the record's witness from empty racks, "
                             "then its support and access edges and the support confirmation drops; needs --gate DIR "
                             "(the accepted joint gate of the built poses)")
    parser.add_argument("--library", type=int, default=10, metavar="N",
                        help="--edges swaps: N random obeying and N random single-violation library orders (10)")
    parser.add_argument("--order-index", type=lambda s: [int(x) for x in s.split(",")], metavar="K[,K...]",
                        help="--edges swaps: build these library orders of the existing record (one Kit session)")
    parser.add_argument("--merge-library", action="store_true",
                        help="--edges swaps, Kit-free: write the built library orders' outcomes into the record")
    parser.add_argument("--release", choices=("clear", "insertion"), default="clear",
                        help="--edges swaps: clear = the lowest clear hover (default); insertion = the re-posing "
                             "sampler and the insertion move (lowered from above the rim, plates upright, released "
                             "20 mm above where the appliance stops it)")
    parser.add_argument("--refine", action="store_true",
                        help="--edges swaps --release insertion: the conditional-edge refinement of a record whose "
                             "--merge-library verdict is 'refine' (one Kit session)")
    parser.add_argument("--sample", action="store_true",
                        help="host python on the box (stdlib): the sampling loop over the packings of "
                             "sequential_fresh_order.txt from r07_00012, Kit sessions on cores 0-13, clips on 14-15")
    parser.add_argument("--plate-order", choices=("lean-side", "random"), default="lean-side",
                        help="--release insertion: the sampler's order, lean-side = a random permutation whose plate "
                             "positions are refilled per bank in chain order from the lean side, plates re-posed along "
                             "the chain (plans/2026-10-16-lean-side-plate-order.md, default); random = the 2026-10-15 b "
                             "uniformly random order")
    parser.add_argument("--backtrack", type=int, default=0, metavar="N",
                        help="--release insertion: the sampler's backtracking depth (plans/2026-10-16-backtracking-"
                             "sampler-learning-loop.md): a dish with no pose that goes in undoes up to N of its most "
                             "recently placed blockers, goes in, and they go back (at most 2 backtracks per dish; 0 = off)")
    parser.add_argument("--learn-rounds", type=int, default=0, metavar="N",
                        help="--merge-library: up to N learning rounds (a failed obeying order adds a learned edge, "
                             "FRESH_ORDERS fresh obeying orders re-check; 0 = an obeying order that fails is invalid)")
    parser.add_argument("--retry-of", metavar="DIR",
                        help="--sample: rerun only the packings whose record in DIR (an earlier --sample run's --out-dir, "
                             "relative to the repo root) failed in the sampler, each once that record exists; the ones it "
                             "built are skipped")
    parser.add_argument("--rerun-backtracked", action="store_true",
                        help="--sample --retry-of: also rerun the packings the first run built with a backtrack that "
                             "placed its dish (their witness order is not how the dishes went in)")
    parser.add_argument("--tolerance", choices=("bench", "library"), default="bench",
                        help="the at-goal table of the swaps, the library and learning (plans/2026-10-16-doubled-tolerance-"
                             "rerun.md): bench = AT_GOAL with contact < 2 mm (default); library = AT_GOAL_LIBRARY (doubled) "
                             "with contact < 4 mm; the sampler's own-load rule is unchanged")
    parser.add_argument("--rejudge", action="store_true",
                        help="Kit-free: re-judge the --records samples' saved swap drops under --tolerance into --out-dir "
                             "(fresh library, learning and refinement dropped)")
    parser.add_argument("--records", nargs="+", type=Path, metavar="PATH", help="--rejudge: --edges swaps records")
    parser.add_argument("--resume-from", type=Path, metavar="DIR",
                        help="--sample: the folder --rejudge wrote (= --out-dir): its samples' libraries first, new packings "
                             "of the fresh list (ones their first run had not attempted) only once all of them are valid")
    parser.add_argument("--target-valid", type=int, default=5, metavar="N", help="--sample: stop at N valid samples")
    parser.add_argument("--max-packings", type=int, default=20, metavar="N", help="--sample: attempt at most N packings")
    parser.add_argument("--goal", choices=("gate", "built"), default="gate",
                        help="--edges pairs: gate = test the targets as they are (default); built = re-settle the "
                             "goal one dish at a time first, then test the built poses, <out>/pairs_resettled/")
    parser.add_argument("--perturb-mm", type=float, default=0., metavar="X",
                        help="--edges pairs: release each L/R drop's dish at a random lateral offset, uniform in the "
                             "disc of radius X mm in its rack plane (default 0: unperturbed)")
    parser.add_argument("--perturb-deg", type=float, default=0., metavar="Y",
                        help="--edges pairs: and at a random yaw about world Z through its centroid, uniform in "
                             "+-Y deg (default 0)")
    parser.add_argument("--perturb-seed", type=int, default=0, metavar="S",
                        help="the offsets' numpy default_rng((S, test number)) (default 0)")
    parser.add_argument("--manifest-out", type=Path, metavar="PATH", help="Kit-free: write the joint-gate manifest, exit")
    parser.add_argument("--max-wall-seconds", type=float, default=3600.)
    parser.add_argument("--force", action="store_true", help="overwrite an existing output")
    args, _ = parser.parse_known_args()                       # --manifest-out exits before isaaclab is imported
    if args.manifest_out:
        return write_manifest(args)
    if args.rejudge:
        if not args.records or not args.out_dir:
            parser.error("--rejudge needs --records and --out-dir")
        return rejudge(args)
    if args.goal == "built" and args.edges == "build":
        parser.error("--goal built needs --edges pairs")
    if (args.order_index or args.merge_library or args.refine) and args.edges != "swaps":
        parser.error("--order-index / --merge-library / --refine need --edges swaps")
    if args.sample and args.edges not in ("swaps", "physical"):
        parser.error("--sample needs --edges swaps or physical")
    if args.certify and args.edges != "physical":
        parser.error("--certify needs --edges physical")
    if (args.release == "insertion" or args.refine or args.sample) and (args.edges not in ("swaps", "physical")
                                                                        or args.release != "insertion"):
        parser.error("--release insertion, --refine and --sample need --edges swaps|physical --release insertion")
    if args.edges in ("swaps", "physical"):
        args.goal = "built"
    if args.sample:
        return physical_loop(args) if args.edges == "physical" else sample_loop(args)
    if not args.load:
        parser.error("--load is required")
    if args.merge_library:
        return merge_library(args)
    if (args.perturb_mm or args.perturb_deg) and args.edges != "pairs":
        parser.error("--perturb-mm / --perturb-deg need --edges pairs")
    if min(args.perturb_mm, args.perturb_deg, args.perturb_seed) < 0:
        parser.error("--perturb-mm, --perturb-deg and --perturb-seed must be >= 0")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    if args.edges in ("swaps", "physical"):                   # (a fresh namespace: set again)
        args.goal = "built"
    folder = record_folder(args)
    out_path = folder / f"{args.load}.json"
    if out_path.exists() and not args.force and not args.order_index and not args.refine and not args.certify:
        print(f"[RESULT] FAIL {args.load} refusing to overwrite {rel(out_path)} (--force)", flush=True)
        return
    kit_dir = folder / "_kit" / args.load
    report_name = ("report_refine.json" if args.refine else "report_certify.json" if args.certify else "report.json"
                   if not args.order_index else f"report_library_{'_'.join(map(str, args.order_index))}.json")
    started = time.monotonic()
    report = {"result": "FAIL", "reason": None, "load_id": args.load, "sampler": args.sampler,
              "started_utc": datetime.now(timezone.utc).isoformat()}
    app = None
    try:
        source, entries = read_load(args)
        report["source"] = rel(source)
        gate = read_gate(args, entries) if args.gate and not args.certify else None     # (--certify reads its own)
        if gate is not None and gate.get("outcome") != "accepted":
            return gate_failed(args, source, entries, gate, out_path, report)      # (finally: report, [RESULT])
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        (physical_certify if args.certify else library_run if args.order_index else dependency if args.edges == "build"
         else pairs)(args, source, entries, gate, out_path, kit_dir, report, started)
    except Refused as exc:
        report.update(result="FAIL", reason=str(exc))
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=time.monotonic() - started, finished_utc=datetime.now(timezone.utc).isoformat())
        kit_dir.mkdir(parents=True, exist_ok=True)
        (kit_dir / report_name).write_text(json.dumps(report, indent=2, default=str) + "\n")
        goal = "goal built " if args.goal == "built" else ""
        print(f"[RESULT] {report['result']} {args.load} {goal}{report['reason']}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
