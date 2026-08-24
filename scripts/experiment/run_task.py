# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase 2 — multi-object pick-and-place: spawn a random countertop, clear it into the machine.

Where ``run_trials.py`` runs one object into one slot per trial, this runs an EPISODE: N objects
are spawned at reproducible random poses, settled under physics, and then cleared one at a time
in an order the task sequencer decides. ``run_trials.py`` is untouched and remains the
single-object baseline.

An episode may start from a machine that is still shut. With ``--scenario both_in`` the racks
begin stowed, where NO slot is reachable, so the robot pulls the lower rack out before the first
pick and loads the machine in the resulting state. Such an episode spans two collision-cache
states; see ``POST_STATE`` below.

Every episode's recorded trajectory begins and ends at ``config.HOME_Q``, so any two are
comparable and any one can be chained from a known state.

Layering (enforced by ``tests/test_layer_boundary.py``):

    TaskSequencer  which object next  ->  PickPlace  one object's choreography
                                             -> MotionService  move A to B
                                                  -> Planner  configuration-space search

Artifacts are written so Phase 3 needs no changes: per-pick records use the EXISTING trial
schema under ``trials/``, per-pick recordings under ``trajectories/``, and the episode-level
record — pick order, costs, blocked reasons, support graph — under ``episodes/``.

One invocation runs ONE episode: the scene is built for a fixed object set, so a different
draw needs a fresh process. Sweep seeds from the shell.

Run with:
    scripts/run_kit.sh scripts/experiment/run_task.py --headless --seed 0
    scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
        --cost_fn shortest_ik --run_id demo
    for s in 0 1 2; do scripts/run_kit.sh scripts/experiment/run_task.py \
        --headless --seed $s --run_id "sweep_$s"; done
"""

import argparse
import json
import os
import sys
import traceback

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Multi-object pick-and-place episodes.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement (see config.BASE_PLACEMENTS); default: the machine's.")
parser.add_argument("--seed", type=int, default=0, help="Layout seed (reproduces the scene).")
parser.add_argument("--n_objects", type=int, default=None, help="Objects per episode.")
parser.add_argument("--classes", type=str, default=None,
                    help="Comma list of object classes to draw from (default: config.TASK).")
parser.add_argument("--spawn", type=str, default=None, metavar="SPEC",
                    help="Explicit composition, e.g. 'cup=2-4,mug=1'. A range is drawn per "
                         "episode from --seed. Replaces the uniform draw over --classes.")
parser.add_argument("--scenario", type=str, default=None,
                    help="Machine state name. An internal state ('placement') starts with the "
                         "racks already positioned; a scenario with a rack action ('both_in') "
                         "makes the robot open the machine first. Overrides "
                         "config.TASK['rack_state'].")
parser.add_argument("--rack_lower_m", type=float, default=None,
                    help="Lower-rack extension [m], 0 = stowed, -0.20 = out. With --rack_upper_m "
                         "this selects the machine state by geometry instead of by name.")
parser.add_argument("--rack_upper_m", type=float, default=None,
                    help="Upper-rack extension [m]; see --rack_lower_m.")
parser.add_argument("--planner", type=str, default=None, help="Planner name (see registry).")
parser.add_argument("--planner_param", type=str, action="append", default=[], metavar="K=V",
                    help="Override one planner parameter (repeatable).")
parser.add_argument("--cost_fn", type=str, default=None,
                    help="Pick-order heuristic (see dishsim.task.cost.available_costs()).")
parser.add_argument("--allow_stacking", action="store_true",
                    help="Stage B: permit overlapping/stacked spawn poses.")
parser.add_argument("--full_load", type=str, nargs="?", const="__default__", default=None,
                    metavar="PLAN",
                    help="Run the multi-phase FULL-LOAD episode from a capacity-plan artifact "
                         "(default path: results/capacity/<machine>/<placement>/"
                         "full_load_plan.json; generate with scripts/setup/plan_full_load.py). "
                         "Items spawn on the counter in restock waves; racks move by scripted "
                         "drive transitions between loading phases.")
parser.add_argument("--phases", type=str, default=None, metavar="SPEC",
                    help="Explicit multi-phase composition (debug path, slots assigned at "
                         "runtime), e.g. 'placement:cup=2;third_out:fork=1'. States must be "
                         "internal (no rack action).")
parser.add_argument("--orbit", action="store_true",
                    help="Write a closing 360-degree orbit clip (phase mode, needs "
                         "--enable_cameras).")
parser.add_argument("--out", type=str, default=None, help="Run directory.")
parser.add_argument("--run_id", type=str, default=None, help="Run name.")
parser.add_argument("--media", type=str, default=None, help="Media directory.")
parser.add_argument("--video_stride", type=int, default=None,
                    help="Capture every Nth physics step (default: 2, or 4 in phase mode — "
                         "full-load episodes run 5-10x longer).")
parser.add_argument("--video_camera", type=str, default=None,
                    help="Camera the episode MP4 is written from (default: config.TASK).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

_enable_cameras = args_cli.enable_cameras  # 2.1's AppLauncher pops this off the namespace
app_launcher = AppLauncher(args_cli)  # boot Kit BEFORE importing dishsim/isaaclab scene modules
args_cli.enable_cameras = _enable_cameras
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from dishsim import config  # noqa: E402
from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz  # noqa: E402
from dishsim.task import phases as tphases  # noqa: E402  (Kit-free: config + numpy only)

if args_cli.machine:
    config.apply_machine(args_cli.machine)  # first: it resets scenario + base placement
if args_cli.placement:
    config.apply_base_placement(args_cli.placement)


def _resolve_phases():
    """Phase mode: the ordered loading phases, resolved before the scenario is."""
    if args_cli.full_load is None and not args_cli.phases:
        return None, None
    if args_cli.full_load is not None and args_cli.phases:
        raise SystemExit("[FAIL] --full_load and --phases are mutually exclusive")
    clashes = [f for f, v in (("--spawn", args_cli.spawn), ("--n_objects", args_cli.n_objects),
                              ("--classes", args_cli.classes),
                              ("--rack_lower_m", args_cli.rack_lower_m),
                              ("--rack_upper_m", args_cli.rack_upper_m)) if v is not None]
    if clashes:
        raise SystemExit(f"[FAIL] {clashes} are meaningless in phase mode — the plan/spec "
                         f"fixes the composition and the phase states fix the racks")
    if args_cli.full_load is not None:
        path = args_cli.full_load
        if path == "__default__":
            path = os.path.join(PROJECT_ROOT, "results", "capacity", config.MACHINE,
                                config.BASE_PLACEMENT, "full_load_plan.json")
        if not os.path.exists(path):
            raise SystemExit(
                f"[FAIL] no full-load plan at {path} — generate it with:\n"
                f"  scripts/setup/plan_full_load.py --machine {config.MACHINE} "
                f"--placement {config.BASE_PLACEMENT}")
        return tphases.load_full_load_plan(
            path, machine=config.MACHINE, base_placement=config.BASE_PLACEMENT), path
    try:
        spec = tphases.parse_phases_arg(args_cli.phases)
    except ValueError as exc:
        raise SystemExit(f"[FAIL] {exc}")
    counters: dict = {}
    out = []
    for state, counts in spec:
        items = []
        for cls in sorted(counts):
            for _ in range(counts[cls]):
                n = counters.get(cls, 0)
                counters[cls] = n + 1
                fam = config.OBJECTS[cls].grasp.family
                items.append(tphases.PhaseItem(
                    item_id=f"{cls}_{n:02d}", object_class=cls, instance=n, slot_id=-1,
                    slot_name="", mode=config.effective_placement_mode(cls),
                    acquire="weld" if fam in config.WELD_ACQUIRE_FAMILIES else "pick"))
        out.append(tphases.Phase(state, items))
    return out, None


#: Phase mode: ordered loading phases (None = the classic single-phase episode), and the plan
#: artifact they came from (None for --phases).
PHASE_LIST, FULL_LOAD_PLAN_PATH = _resolve_phases()
if args_cli.video_stride is None:
    args_cli.video_stride = 4 if PHASE_LIST is not None else 2


def _resolve_scenario() -> str:
    """Machine state from --scenario, per-rack metres, or config — resolved before Kit boots."""
    if (args_cli.rack_lower_m is None) != (args_cli.rack_upper_m is None):
        raise SystemExit("[FAIL] --rack_lower_m and --rack_upper_m must be given together")
    if args_cli.rack_lower_m is not None:
        spec = {"lower_m": args_cli.rack_lower_m, "upper_m": args_cli.rack_upper_m}
    else:
        spec = args_cli.scenario or config.TASK["rack_state"]
    try:
        return config.resolve_rack_state(spec)
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(f"[FAIL] {exc}")


if PHASE_LIST is not None and args_cli.scenario is None:
    # phase mode starts the machine directly in the first loading phase's state unless an
    # explicit prologue scenario (e.g. both_in for the robot rack pull) is requested
    SCENARIO = PHASE_LIST[0].state
else:
    SCENARIO = _resolve_scenario()


def _post_action_state(scenario: str) -> str:
    """The machine state the episode PLACES in, after any rack action has run.

    Derived from the action rather than hardcoded to ``PLACEMENT_STATE``: both shipped
    scenarios happen to converge there (``both_in`` pulls the lower rack out, ``both_out``
    pushes the upper one in), but a new scenario that did not would otherwise silently load the
    wrong caches.
    """
    sc = config.state_params(scenario)
    action = sc.get("rack_action")
    if action is None:
        return scenario
    ext = {"lower_m": sc["rack_lower_m"], "upper_m": sc["rack_upper_m"]}
    ext["upper_m" if action["joint"].endswith("_up") else "lower_m"] = float(action["to"])
    try:
        return config.resolve_rack_state(ext)
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(f"[FAIL] scenario {scenario!r} acts into an unbaked state: {exc}")


#: Where the rack action leaves the machine — the state every pick and place plans against.
POST_STATE = _post_action_state(SCENARIO)
#: The rack action to run before any picking, or ``None`` when the racks start pre-positioned.
RACK_ACTION = config.state_params(SCENARIO).get("rack_action")

if PHASE_LIST is not None and POST_STATE != PHASE_LIST[0].state:
    raise SystemExit(
        f"[FAIL] --scenario {SCENARIO!r} leaves the machine in {POST_STATE!r} but the first "
        f"loading phase is {PHASE_LIST[0].state!r} — a prologue scenario must act into the "
        f"first phase's state")

config.apply_scenario(SCENARIO)

from dishsim import placement  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim import trajectory as dtraj  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.geometry import config_hash as geometry_config_hash  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.planners import available as available_planners  # noqa: E402
from dishsim.planners import make_planner  # noqa: E402
from dishsim.task import layout as tlayout  # noqa: E402
from dishsim.task import rack as task_rack  # noqa: E402
from dishsim.task import recovery as trecovery  # noqa: E402
from dishsim.task import slotting as tslotting  # noqa: E402
from dishsim.task import support as tsupport  # noqa: E402
from dishsim.task.grasp import GraspFinder  # noqa: E402
from dishsim.task.episode import (  # noqa: E402
    EPISODE_SCHEMA_VERSION, EpisodeResult, full_load_status, merge_wave_results, write_episode,
)
from dishsim.task.motion import MotionService  # noqa: E402
from dishsim.task.primitives import GraspProfile, PickPlace  # noqa: E402
from dishsim.task.sequencer import TaskItem, TaskSequencer  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402
from dishsim.ur5e_kin import ik_wrist3_all  # noqa: E402

TRIAL_SCHEMA_VERSION = 1


def main() -> int:
    if PHASE_LIST is not None:
        return _run_full_load_episode()
    if RACK_ACTION is not None:
        print(f"[INFO] scenario {SCENARIO!r} starts with a rack action "
              f"({RACK_ACTION['mode']} {RACK_ACTION['joint']} -> {RACK_ACTION['to']} m); "
              f"the episode places in {POST_STATE!r}")

    # Explicit composition wins over the uniform draw. When it is set the class POOL is derived
    # from it — otherwise a class named only in spawn_counts would have no grasp profile or goal
    # set loaded, and would fail with a KeyError deep inside the layout rejection loop.
    spawn_spec = None
    if args_cli.spawn:
        try:
            spawn_spec = tlayout.parse_spawn_arg(args_cli.spawn)
        except ValueError as exc:
            print(f"[FAIL] {exc}")
            return 1
    elif config.TASK["spawn_counts"]:
        spawn_spec = dict(config.TASK["spawn_counts"])
    if spawn_spec is not None and args_cli.n_objects is not None:
        print("[FAIL] --n_objects and an explicit composition are mutually exclusive: the "
              "composition already fixes the total. Drop one.")
        return 1

    classes_pool = (sorted(spawn_spec) if spawn_spec is not None
                    else [c.strip() for c in args_cli.classes.split(",")] if args_cli.classes
                    else list(config.TASK["classes"]))
    unknown = [c for c in classes_pool if c not in config.OBJECTS]
    if unknown:
        print(f"[FAIL] unknown object class(es) {unknown}")
        return 1
    welded = [c for c in classes_pool
              if config.OBJECTS[c].grasp.family in config.WELD_ACQUIRE_FAMILIES]
    if welded:
        print(f"[INFO] {welded} have no calibrated countertop pick (grasp families "
              f"{config.WELD_ACQUIRE_FAMILIES}); they are ACQUIRED by snapping to the carry "
              f"transform at the pre-grasp hover, not picked. Calibrate with "
              f"scripts/setup/calibrate_grasp.py to make them real picks.")

    n_objects = int(args_cli.n_objects or config.TASK["n_objects"])
    planner_name = args_cli.planner or config.PLANNER
    if planner_name not in available_planners():
        print(f"[FAIL] unknown planner {planner_name!r} (choices: {available_planners()})")
        return 1
    planner_params = dict(config.PLANNER_PARAMS.get(planner_name, {}))
    for item in args_cli.planner_param:
        key, _, value = item.partition("=")
        try:
            planner_params[key.strip()] = json.loads(value)
        except json.JSONDecodeError:
            planner_params[key.strip()] = value
    planner = make_planner(planner_name, **planner_params)

    # Validate the video camera HERE, not in the capture callback. That callback first runs deep
    # inside the episode — after Kit boot, cache loads, scene build and the settle loop — so a
    # typo would otherwise cost minutes before surfacing as a bare KeyError.
    video_camera = args_cli.video_camera or config.TASK["video_camera"]
    known_cameras = sorted(set(config.CAMERAS) | {"episode"})
    if video_camera not in known_cameras:
        print(f"[FAIL] unknown --video_camera {video_camera!r} (choices: {known_cameras})")
        return 1

    run_id = args_cli.run_id or (
        f"task_{SCENARIO}_{planner_name}_"
        f"{__import__('datetime').datetime.now(__import__('datetime').timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    )
    run_dir = args_cli.out or os.path.join(PROJECT_ROOT, "results", "experiments", run_id)
    for sub in ("episodes", "trials", "trajectories"):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)
    media_dir = args_cli.media or os.path.join(PROJECT_ROOT, "media", "task", run_id)
    if args_cli.enable_cameras:
        os.makedirs(media_dir, exist_ok=True)

    # ---- cache pre-flight ---------------------------------------------------------------------
    # Rack extensions are part of the collision-cache hash and caches are keyed by state name, so
    # a state that has not been baked for these classes cannot be planned against. Check every
    # cache the run will open BEFORE Kit does any work, and say exactly how to create what is
    # missing — a bare FileNotFoundError minutes in is a poor substitute.
    # The reference class is checked unconditionally: the machine world is loaded from it
    # whatever the pool (config.reference_class: mug on v1, cup on the Bosch).
    #
    # An episode with a rack action spans TWO states and therefore two sets of caches. The rack
    # phase plans in the START state but carries nothing, so it needs only the scenario-level
    # (mug-keyed) machine cache; every pick and place happens after the action and needs the
    # per-class caches of the POST state. Checking both against one state is what would send you
    # to bake `objects/cup/both_in`, which nothing ever reads.
    ref = config.reference_class()
    wanted = [(SCENARIO, [ref])] if RACK_ACTION is not None else []
    wanted.append((POST_STATE, list(dict.fromkeys([ref, *classes_pool]))))
    missing = []
    for state, names in wanted:
        for name in names:
            cdir = config.scenario_cache_dir(state, object_name=name)
            for rel in ("scene_state.json", os.path.join("slots", "slots.json"),
                        os.path.join("slots", "goal_sets.json")):
                if not os.path.exists(os.path.join(cdir, rel)):
                    missing.append((state, name, cdir, rel))
                    break
    if missing:
        for state in dict.fromkeys(m[0] for m in missing):
            names = [m[1] for m in missing if m[0] == state]
            print(f"[FAIL] machine state {state!r} has no collision cache for {names}")
            for _, _, cdir, rel in (m for m in missing if m[0] == state):
                print(f"       missing {os.path.join(os.path.relpath(cdir, PROJECT_ROOT), rel)}")
            print("       bake it with:")
            print(f"         scripts/setup/build_state.py --state {state} "
                  f"--classes {','.join(names)}")
        return 1

    # ---- geometry: one world for every pick, plus per-class grasp profiles -------------------
    # load_manifest asserts the cache's config_hash against the CURRENT config, and the hash
    # contains the rack extensions — so a cache can only be opened while its own state is
    # applied. Flip to the post-action state to build the pick world, then flip back: the SCENE
    # was already built at SCENARIO (apply_scenario ran before the scene imports bound their rack
    # values), so this only moves the hash the loader checks, never the machine.
    rack_world = None
    if RACK_ACTION is not None:
        rack_cache = config.scenario_cache_dir(SCENARIO, object_name=config.reference_class())
        # the reference-class cache hashes against ITS object being active (mug on v1 is the
        # module default and matched implicitly; the Bosch reference is the cup)
        with config.active_object(config.reference_class()):
            rack_world = CollisionWorld(cache_dir=rack_cache, self_check=True, object_attached=False)
        print(f"[INFO] rack world:      {os.path.relpath(rack_cache, PROJECT_ROOT)} ({SCENARIO})")
        config.apply_scenario(POST_STATE)

    # Per-piece cluster whenever any pooled class needs a thin insertion: the merged gripper+
    # payload hull is a single convex wedge that provably cannot enter a cutlery bay, a 30 mm
    # tine gap, or a third-rack tray channel. Merged is 3x faster and strictly conservative, so
    # it stays the default for pools that only stand things on the rack floor. The EFFECTIVE
    # mode decides — on the Bosch a fork reroutes basket_drop -> flat_lay_third and must keep
    # the per-piece cluster through that reroute.
    merged = not any(config.effective_placement_mode(c) in
                     ("basket_drop", "plate_slot", "flat_lay_third")
                     for c in classes_pool)
    cache_dir = config.scenario_cache_dir(POST_STATE, object_name=config.reference_class())
    with config.active_object(config.reference_class()):
        world = CollisionWorld(cache_dir=cache_dir, self_check=True, object_attached=False,
                               merged_cluster=merged)
    if not merged:
        print("[INFO] per-piece payload cluster (a thin-insertion mode is in the pool)")
    T_w3_tcp = np.array(world.manifest["t_wrist3_tcp"])
    print(f"[INFO] collision world: {os.path.relpath(cache_dir, PROJECT_ROOT)} ({POST_STATE}; "
          f"nothing carried, payloads attach per pick)")

    profiles, goal_sets, slots_by_class, slot_names_by_class = {}, {}, {}, {}
    for name in classes_pool:
        profiles[name] = GraspProfile.build(name, POST_STATE)
        cdir = config.scenario_cache_dir(POST_STATE, object_name=name)
        with open(os.path.join(cdir, "slots", "slots.json")) as f:
            slots_by_class[name] = {s["slot_id"]: placement.SlotFrame.from_json(s)
                                    for s in json.load(f)["slots"]}
        with open(os.path.join(cdir, "slots", "goal_sets.json")) as f:
            goal_sets[name] = {g["slot_id"]: np.array(g["configs"])
                               for g in json.load(f)["goal_sets"]}
        slot_names_by_class[name] = placement.slot_names(list(slots_by_class[name].values()))
        inv = {v: k for k, v in slot_names_by_class[name].items()}
        usable = [inv.get(k, k) for k, v in goal_sets[name].items() if len(v)]
        print(f"[INFO] {name:<8} slots with goal configs: {usable}")

    # Back to the state the SCENE is physically in. This matters beyond bookkeeping:
    # hold_targets pins the rack drives from config.RACK_JOINT_TARGETS every step, so leaving the
    # post-action state applied would command the rack out from step one and the robot would
    # arrive at an already-open machine. After the pull, RackAction's latched override — not this
    # — is what holds the rack where it was put.
    if RACK_ACTION is not None:
        config.apply_scenario(SCENARIO)

    # ---- layout ------------------------------------------------------------------------------
    def reach_fn(object_class: str, T_base_obj: np.ndarray) -> bool:
        """Could the robot actually pick this class standing here?

        The same feasibility test the acquisition itself will run, which is not the same test for
        every class. A picked class must be reachable at the pre-grasp hover AND at the grasp
        pose: checking only the hover would accept poses the arm can reach down to but not grasp
        at. A WELD-acquired class never descends — the object is snapped to the carry transform at
        the hover — so demanding grasp-pose IK there rejects poses the episode would handle
        perfectly well. Measured: it rejected 344 of 400 fork draws, for a descent that never runs.

        The world here holds the machine and nothing else: at layout time no object has spawned
        yet. Stage B re-checks against the SETTLED scene, where neighbours exist and physics has
        had its say.
        """
        prof = profiles[object_class]
        T_grasp = np.asarray(T_base_obj) @ T_inv(prof.T_tcp_obj)
        T_hover = T_grasp.copy()
        T_hover[2, 3] += config.PICK_HOVER_M
        q_seed = np.array(config.HOME_Q)
        hover = [q for q in ik_wrist3_all(T_hover @ T_inv(T_w3_tcp), q_seed=q_seed)
                 if not world.in_collision(q)]
        if not hover:
            return False
        if prof.acquire == "weld":
            return True
        return any(not world.in_collision(q)
                   for q in ik_wrist3_all(T_grasp @ T_inv(T_w3_tcp), q_seed=hover[0]))

    rng = np.random.default_rng(args_cli.seed)
    if spawn_spec is not None:
        try:
            draw = tlayout.resolve_spawn_counts(spawn_spec, rng)
        except ValueError as exc:
            print(f"[FAIL] {exc}")
            return 1
        if not draw:
            print(f"[FAIL] composition {spawn_spec} drew 0 objects for seed {args_cli.seed}; an "
                  f"empty episode would report 'cleared' having done nothing")
            return 1
        print(f"[INFO] composition {spawn_spec} -> {len(draw)} objects: "
              f"{ {c: draw.count(c) for c in sorted(set(draw))} }")
    else:
        draw = tlayout.sample_classes(rng, n_objects, classes_pool)
    items_layout, rejection = tlayout.plan_countertop_layout(
        draw, seed=args_cli.seed, reach_fn=reach_fn, allow_stacking=args_cli.allow_stacking
    )
    print(f"[INFO] layout seed {args_cli.seed}: {[i.item_id for i in items_layout]} "
          f"({rejection.n_draws} draws; rejected {rejection.too_close} too-close, "
          f"{rejection.unreachable} unreachable)")

    # ---- scene -------------------------------------------------------------------------------
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device)
    )
    peers = ["{ENV_REGEX_NS}/" + it.item_id for it in items_layout]
    obj_specs = []
    for it in items_layout:
        spec = config.OBJECTS[it.object_class]
        pos_w, quat = _pose_w(it)
        obj_specs.append({
            "name": it.item_id, "usd_path": spec.usd_path, "pos": pos_w, "quat": quat,
            # peer filters let the support graph be read from real contacts (Stage B)
            "contact_filters": [p for p in peers if not p.endswith("/" + it.item_id)],
        })
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=False, with_robot_contacts=True, objects=obj_specs))
    welds = {
        it.item_id: dscene.author_weld(
            scene.stage, prim_name=it.item_id,
            T_wrist3_obj=T_w3_tcp @ profiles[it.object_class].T_tcp_obj,
        )
        for it in items_layout
    }
    # The episode camera is appended LAST: render_videos.py takes `next(iter(cameras))` for its
    # frame-count, variation and black-frame gates, so inserting a new name first would silently
    # move those checks onto a different view.
    ep = config.EPISODE_CAMERA
    cam_specs = {**{k: v for k, v in config.CAMERAS.items()},
                 "episode": (tuple(ep["eye"]), tuple(ep["target"]), dict(ep["lens"]))}
    rig = CameraRig(cam_specs, hw=config.CAMERA_HW) if args_cli.enable_cameras else None
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    for path in welds.values():
        dscene.set_weld_enabled(scene.stage, path, False)

    # The whole contact story — grip gates and the support graph alike — reads force_matrix_w by
    # zipping its filter axis against cfg.filter_prim_paths_expr. That ordering IS guaranteed
    # element-for-element by Isaac Lab (verified against the sensor source and by scrambling the
    # cfg order), and a filter expression matching zero or several bodies crashes at reset rather
    # than misaligning silently. Asserting it once here costs nothing and pins the invariant
    # every force reading in this file depends on.
    for it in items_layout:
        sensor = scene[f"{it.item_id}_contact"]
        n_filters = len(sensor.cfg.filter_prim_paths_expr)
        assert sensor.contact_physx_view.filter_count == n_filters, (
            f"{it.item_id}: contact filter count {sensor.contact_physx_view.filter_count} != "
            f"{n_filters} configured expressions — the force_matrix_w zip would be misaligned"
        )

    dt = sim.get_physics_dt()
    robot = scene["robot"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = robot.data.joint_pos.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]
    gripper_bodies = tuple(sensor_names[1])
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    rec = dtraj.TrajectoryRecorder()

    ctx = _Ctx(scene, sim, dt, robot, arm_ids, sensors, sensor_names, rec)

    # ---- spawn -> settle -> reachability check -> resample --------------------------------------
    def measured(item_id: str) -> np.ndarray:
        o = scene[item_id]
        return T_base_w @ make_T(o.data.root_pos_w[0].cpu().numpy(),
                                 wxyz_to_xyzw(o.data.root_quat_w[0].cpu().numpy()))

    def spawn_and_settle(layout):
        """Teleport to a layout, settle physics, and report the MEASURED poses.

        Prims are reused rather than respawned: the object set is fixed when the scene is built,
        and only the poses vary between layout attempts.
        """
        for it in layout:
            pos_w, quat = _pose_w(it)
            scene[it.item_id].write_root_pose_to_sim(
                root_pose=torch.tensor(np.concatenate([pos_w, xyzw_to_wxyz(quat)])[None],
                                       dtype=torch.float32, device=device))
            scene[it.item_id].write_root_velocity_to_sim(
                root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
        ctx.step(int(config.TASK["settle_steps"]))
        return {it.item_id: measured(it.item_id) for it in layout}

    # Reachability cannot be guaranteed analytically once physics has had its say: an object may
    # be spawned reachable and settle somewhere unreachable — and with stacking it is guaranteed
    # to move, because it drops onto whatever is beneath it. So the guarantee is enforced AFTER
    # settling, and a layout that fails it is resampled wholesale.
    max_attempts = int(config.TASK["max_layout_retries"])
    seeds_tried, attempt, poses = [], 0, None
    while True:
        attempt += 1
        seeds_tried.append(int(args_cli.seed) if attempt == 1 else int(args_cli.seed) + 10_000 * attempt)
        if attempt > 1:
            items_layout, rejection = tlayout.plan_countertop_layout(
                draw, seed=seeds_tried[-1], reach_fn=reach_fn,
                allow_stacking=args_cli.allow_stacking)
        poses = spawn_and_settle(items_layout)
        unreachable = [k for k, T in poses.items()
                       if not reach_fn(next(i.object_class for i in items_layout if i.item_id == k), T)]
        drifts = {k: float(np.linalg.norm(T[:3, 3]
                                          - next(i for i in items_layout if i.item_id == k).T_base_obj[:3, 3])) * 1e3
                  for k, T in poses.items()}
        print(f"[INFO] layout attempt {attempt}/{max_attempts} (seed {seeds_tried[-1]}): settled "
              + ", ".join(f"{k} {d:.1f} mm" for k, d in drifts.items()))
        if not unreachable:
            break
        print(f"[INFO]   unreachable after settling: {unreachable} — resampling the layout")
        if attempt >= max_attempts:
            print(f"[FAIL] no layout with all objects reachable after settling in {max_attempts} "
                  f"attempts (seeds tried: {seeds_tried}). Still unreachable: {unreachable}. "
                  f"Widen TASK['spawn_rect_w'], lower TASK['n_objects'], or raise "
                  f"TASK['max_layout_retries'].")
            return 1

    items = [TaskItem(item_id=it.item_id, object_class=it.object_class, instance=it.instance,
                      T_base_obj=poses[it.item_id], radius_m=it.radius_m)
             for it in items_layout]
    n_layout_attempts = attempt

    # every object on the counter is an obstacle for every pick — and for the rack action, which
    # sweeps the arm right past the counter on its way to the handle
    for it in items:
        world.add_object(it.item_id, profiles[it.object_class].pieces, it.T_base_obj)
        if rack_world is not None:
            rack_world.add_object(it.item_id, profiles[it.object_class].pieces, it.T_base_obj)

    # ---- the episode ----------------------------------------------------------------------------
    motion = MotionService(planner, world, ctx)
    scene_access = _SceneAccess(scene, welds, items, gripper_bodies, T_base_w, T_w3_tcp, profiles)
    capture, writer = _make_capture(rig, rec, media_dir, args_cli, dt,
                                    f"ep{args_cli.seed:03d}", video_camera)
    pick_place = PickPlace(motion, scene_access, profiles=profiles, goal_sets=goal_sets,
                           T_w3_tcp=T_w3_tcp, seed=args_cli.seed, on_step=capture)
    # Stage C: grasp availability is a property of the CURRENT world, re-derived every iteration
    # over a yaw sweep, with a rejection funnel so "unpickable" can say why.
    grasp_finder = GraspFinder(profiles, motion)
    pick_place.grasp_finder = grasp_finder

    assignment = _assign_slots(items, slots_by_class, goal_sets, slot_names_by_class)
    for it in items:
        slot = assignment.get(it.item_id)
        inv = {v: k for k, v in slot_names_by_class[it.object_class].items()}
        print(f"[INFO] slot assignment: {it.item_id:<12} -> "
              f"{inv.get(slot.slot_id, slot.slot_id) if slot is not None else 'NONE (no free feasible slot)'}")

    def allocate_slot(item):
        """Return this item's pre-assigned destination slot (see :func:`_assign_slots`)."""
        return assignment.get(item.item_id)

    def refresh():
        poses = {it.item_id: measured(it.item_id) for it in items if it.state == "pending"}
        for item_id, T in poses.items():
            it = next(i for i in items if i.item_id == item_id)
            motion.set_obstacle(item_id, profiles[it.object_class].pieces, T)
        return poses

    peer_ids = {it.item_id for it in items}
    latest_graph: dict = {}

    def support_fn(remaining):
        """Rebuild the support graph from the CURRENT measured state (Stage B).

        Called before every pick, never cached: removing one object lets its neighbours settle
        somewhere new, so a graph carried over either blocks a free object forever or clears a
        pick that pulls a pile over.
        """
        graph = tsupport.build_support_graph(
            remaining, forces=_peer_forces(scene, remaining, peer_ids))
        if graph.disagreements:
            print(f"[WARN] support backends disagree: {graph.disagreements}")
        holding = {k: sorted(v) for k, v in graph.supports.items() if v}
        if holding:
            print(f"[INFO] support graph: {holding}")
        latest_graph["g"] = graph
        return graph.supports

    rec.begin(scene, sim, {
        "episode_id": f"ep{args_cli.seed:03d}", "seed": args_cli.seed,
        "classes": draw, "scenario": SCENARIO, "post_state": POST_STATE,
        "planner": planner.describe(),
        "object_classes": {it.item_id: it.object_class for it in items},
        # legacy single-object key, used by verify_replay/render_videos to pick an active
        # object. Must be shuffle-invariant or a replay would resolve a different class
        # (and therefore a different config_hash) depending on the draw order.
        "object": sorted(set(draw))[0],
        "config_hash": geometry_config_hash(),
        # eye, target AND lens: without the lens a replay re-renders a 15 mm episode view
        # through the 24 mm default and silently reframes the shot.
        "cameras": {k: {"eye": list(v[0]), "target": list(v[1]),
                        "lens": (dict(v[2]) if len(v) > 2 else config.camera_lens(k))}
                    for k, v in cam_specs.items()},
        "camera_hw": list(config.CAMERA_HW), "camera_fps": int(config.CAMERA_FPS),
    }, object_keys=[it.item_id for it in items])
    seq = TaskSequencer(
        # the sequencer holds the FINDER itself, not a bound method wrapping it — the
        # recovery ladder's cheap rung widens the sweep through this reference
        items, motion, pick_place=pick_place, grasp_fn=grasp_finder,
        slot_fn=allocate_slot, cost_fn=args_cli.cost_fn or config.TASK["cost_fn"],
        refresh_fn=refresh, support_fn=support_fn,
        recovery=trecovery.make_recovery(),
        max_recoveries=int(config.TASK["max_recovery_attempts"]),
        on_event=lambda n, p: print(f"[INFO] {n}: {p}"),
    )
    # ---- start anchor -------------------------------------------------------------------------
    # The scene spawns the arm at HOME_Q and hold_targets re-arms the drives there, but the
    # layout spawn->settle->resample loop above steps physics an unbounded number of times before
    # the first pick. Measure the drift, correct it if it matters, and assert — an episode that
    # begins somewhere other than home is not comparable with any other episode.
    start_home_err = pick_place.home_error()
    if start_home_err >= float(config.TASK["home_tol_rad"]):
        print(f"[INFO] arm drifted {start_home_err:.4f} rad from home during layout settling "
              f"— returning before the first pick")
        pick_place.return_home(phase="episode-start-home", settle_steps=30)
        start_home_err = pick_place.home_error()
    assert start_home_err < float(config.TASK["home_tol_rad"]), (
        f"episode did not start at HOME_Q: {start_home_err:.4f} rad > "
        f"{config.TASK['home_tol_rad']} rad after a corrective retreat"
    )

    # ---- rack prologue --------------------------------------------------------------------------
    # A stowed rack has nowhere to place anything (0 of 15 slots have goal configurations,
    # measured), so the machine has to be opened before the first pick. The arm plans this against
    # the START-state world; every pick afterwards plans against the POST-state world, which is
    # only truthful once the rack has actually arrived — hence the hard gate below.
    episode_id = f"ep{args_cli.seed:03d}"
    rack_phase, rack_detail = None, None
    if RACK_ACTION is not None:
        rack_joint_ids = scene["dishwasher"].find_joints(RACK_ACTION["joint"])[0]
        rack_runner = task_rack.RackAction(
            motion, gripper_bodies=gripper_bodies, seed=args_cli.seed, on_step=capture,
            measure_ext=lambda: float(
                scene["dishwasher"].data.joint_pos[0, rack_joint_ids[0]]),
        )
        motion.world = rack_world
        try:
            rack_phase = task_rack.run_sequence(
                [task_rack.RackSpec.from_action(RACK_ACTION, rack_world)], rack_runner)
        finally:
            motion.world = world          # picks always plan against the post-action world
        for o in rack_phase.outcomes:
            print(f"[INFO] rack {o.joint}: ok={o.ok} target={o.target_m} m "
                  f"achieved={o.achieved_m} m err={o.error_m} m {o.stage or ''} {o.detail or ''}")
        if not rack_phase.ok:
            # Hard stop. Every pick plans against a world posed at the POST-state extension; if
            # the rack did not get there, that world is a fiction and so is everything built on
            # it. Better one honest error than a run of quietly invalid placements.
            bad = next(o for o in rack_phase.outcomes if not o.ok)
            rack_detail = f"{bad.stage}: {bad.detail}"
        else:
            # World-handoff seam: the disengage was validated in the RACK world (rack at its
            # cached, pre-action pose), so the parked arm can sit inside the EXTENDED rack's
            # hulls in the pick world — the first pick then dies with an uninitializable
            # start tree (measured: OMPL rejecting in 2.4 ms). Route through HOME, which the
            # mount sweep verified collision-free in the post-action world.
            how = pick_place.return_home(phase="post-rack-home", settle_steps=30)
            print(f"[INFO] post-rack home return: {how}")

    if rack_detail is not None:
        result = EpisodeResult(episode_id=episode_id, seed=int(args_cli.seed), classes=list(draw),
                               cost_fn=args_cli.cost_fn or config.TASK["cost_fn"],
                               status="error", detail=rack_detail,
                               unplaced=[it.item_id for it in items])
    else:
        try:
            result = seq.run(episode_id=episode_id, seed=args_cli.seed, classes=draw)
        except Exception as exc:  # noqa: BLE001
            # The failed episode is the interesting one: never lose the trajectory to a crash. The
            # arm is still parked below and every artifact is still written, with the cause noted.
            traceback.print_exc()
            result = EpisodeResult(episode_id=episode_id, seed=int(args_cli.seed),
                                   classes=list(draw),
                                   cost_fn=args_cli.cost_fn or config.TASK["cost_fn"],
                                   status="error", detail=f"{type(exc).__name__}: {exc}",
                                   unplaced=[it.item_id for it in items])
    result.start_home_err_rad = start_home_err
    if rack_phase is not None:
        result.rack_phase = rack_phase.to_json()
    result.layout_rejection = rejection.to_json()
    # a deadlock must be able to say WHY each object was unpickable, not merely that it was
    for item_id, why in grasp_finder.reasons().items():
        if item_id in result.blocked_reasons:
            result.blocked_reasons[item_id] = f"{result.blocked_reasons[item_id]}; {why}"
    result.n_layout_attempts = n_layout_attempts
    if latest_graph.get("g") is not None:
        result.support_graph = latest_graph["g"].supports

    # ---- closing retreat ----------------------------------------------------------------------
    # Runs BEFORE rec.end and _finish_media so the motion lands in the episode .npz and the MP4,
    # and the final stills show the arm parked rather than extended mid-scene. It runs AFTER
    # seq.run, whose placement verdicts are already decided — parking the arm can never revise a
    # pass/fail. It can still nudge a placed object if the un-collision-checked lerp fallback
    # fires, so the placed poses are snapshotted here and re-measured after.
    placed_before = {p.item_id: scene_access.object_pose_base(p.item_id)[:3, 3].copy()
                     for p in result.picks if p.success}
    try:
        result.home_return_status = pick_place.return_home(phase="episode-home")
        result.end_home_err_rad = pick_place.home_error()
        result.post_home_displacement_mm = {
            item_id: 1000.0 * float(np.linalg.norm(
                scene_access.object_pose_base(item_id)[:3, 3] - pos0))
            for item_id, pos0 in placed_before.items()
        }
        moved = {k: v for k, v in result.post_home_displacement_mm.items() if v > 1.0}
        print(f"[INFO] closing retreat: {result.home_return_status}, "
              f"home err {result.end_home_err_rad:.4f} rad"
              + (f", disturbed {moved}" if moved else ""))
    except Exception as exc:  # noqa: BLE001 — parking is evidence, not the experiment
        print(f"[WARN] closing retreat failed: {exc}")
        result.home_return_status = "failed"

    traj_path = os.path.join(run_dir, "trajectories", f"{result.episode_id}.npz")
    tmeta = rec.end(traj_path, extra_meta={"status": result.status, "n_placed": result.n_placed})
    for p in result.picks:
        p.trajectory_path = os.path.relpath(traj_path, PROJECT_ROOT)
        _write_trial(run_dir, result, p, planner, args_cli, tmeta)

    ep_path = write_episode(
        os.path.join(run_dir, "episodes", f"{result.episode_id}.json"), result,
        extra={"planner": planner.describe(), "scenario": SCENARIO, "post_state": POST_STATE,
               "run_id": run_id, "trajectory": os.path.relpath(traj_path, PROJECT_ROOT),
               "motion_stats": {"n_plans": motion.stats.n_plans,
                                "n_plan_failures": motion.stats.n_plan_failures,
                                "plan_time_s": round(motion.stats.plan_time_s, 3),
                                "n_exec_steps": motion.stats.n_exec_steps}},
    )
    with open(os.path.join(PROJECT_ROOT, "results", "experiments", "LATEST"), "w") as f:
        f.write(run_id + "\n")
    _write_manifest(run_dir, run_id, planner)

    print(f"[INFO] episode {result.episode_id}: status={result.status} "
          f"placed={result.n_placed}/{len(result.classes)} order={[p.item_id for p in result.picks]}")
    for p in result.picks:
        print(f"[INFO]   {p.item_id:<12} slot={p.goal_slot} cost={p.cost:.4f} "
              f"success={p.success} {p.failure_stage or ''} {p.failure_detail or ''}")
    if result.blocked_reasons:
        for k, v in result.blocked_reasons.items():
            print(f"[INFO]   blocked {k}: {v}")
    print(f"[INFO] wrote {os.path.relpath(ep_path, PROJECT_ROOT)}")
    if rig is not None:
        _finish_media(rig, writer, media_dir, result)
    print(f"[RESULT] {'PASS' if result.status == 'cleared' else 'FAIL'} ({result.status})")
    return 0 if result.status == "cleared" else 1


#: Rack body that a phase's placed items ride through the NEXT scripted transition — the rack
#: that was extended (and loaded) in that state.
_PHASE_RIDER_BODY = {"placement": "E_shelf_1_04", "middle_out": "E_shelf_03",
                     "third_out": "E_shelf_third", "placement_open": "E_shelf_1_04"}

#: Parking grid for not-yet-spawned wave items, well outside the workspace on the ground
#: plane (the capacity_fill idiom).
def _park_pose_w(k: int):
    return (-2.0 - 0.35 * (k % 6), -1.5 + 0.35 * (k // 6), 0.10), (0.0, 0.0, 0.0, 1.0)


def _run_full_load_episode() -> int:  # noqa: PLR0915 — one episode, top to bottom, like main()
    """The multi-phase FULL-LOAD episode: per loading phase, restock the counter in waves and
    clear it; between phases the racks move by scripted drive transitions while placed items
    ride. Composition and (in ``--full_load`` mode) destination slots come from the capacity
    plan, so the episode attempts exactly the jointly-certified load.

    Differences from the single-phase ``main()`` path, by design:

    - the episode record is FLUSHED after every wave and phase (atomic rewrite) and trials are
      written at each phase end — a 60+ minute run that dies mid-phase stays analyzable;
    - the trajectory recorder is segmented per phase (fresh ``begin``/``end`` around each) so
      the 40k-step buffer never overflows and each phase owns an ``.npz``;
    - a failed transition DEGRADES the episode (remaining phases skipped, placed items kept)
      instead of erroring — earlier phases' picks planned against their own worlds and stay
      valid evidence;
    - a wave whose layout cannot settle reachable is skipped (items reported unplaced), not
      fatal.
    """
    phase_states = [ph.state for ph in PHASE_LIST]
    all_items = [(pi, it) for pi, ph in enumerate(PHASE_LIST) for it in ph.items]
    n_planned = len(all_items)
    classes_pool = list(dict.fromkeys(it.object_class for _, it in all_items))
    print(f"[INFO] FULL-LOAD episode: {n_planned} items over {len(PHASE_LIST)} phases "
          + " -> ".join(f"{ph.state}({len(ph.items)})" for ph in PHASE_LIST))
    print(f"[INFO] rough wall-time estimate: {n_planned * 4} min of picks + "
          f"{(len(PHASE_LIST) - 1) * 2} min of transitions (measured per-item ~2-6 min)")
    welded = sorted({it.object_class for _, it in all_items if it.acquire == "weld"})
    if welded:
        print(f"[INFO] {welded} are weld-ACQUIRED (no calibrated countertop pick); their "
              f"success rate reports separately and is never summed with real picks")

    planner_name = args_cli.planner or config.PLANNER
    if planner_name not in available_planners():
        print(f"[FAIL] unknown planner {planner_name!r} (choices: {available_planners()})")
        return 1
    planner_params = dict(config.PLANNER_PARAMS.get(planner_name, {}))
    for item in args_cli.planner_param:
        key, _, value = item.partition("=")
        try:
            planner_params[key.strip()] = json.loads(value)
        except json.JSONDecodeError:
            planner_params[key.strip()] = value
    planner = make_planner(planner_name, **planner_params)

    video_camera = args_cli.video_camera or config.TASK["video_camera"]
    known_cameras = sorted(set(config.CAMERAS) | {"episode"})
    if video_camera not in known_cameras:
        print(f"[FAIL] unknown --video_camera {video_camera!r} (choices: {known_cameras})")
        return 1

    run_id = args_cli.run_id or (
        f"fullload_{config.MACHINE}_"
        f"{__import__('datetime').datetime.now(__import__('datetime').timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    )
    run_dir = args_cli.out or os.path.join(PROJECT_ROOT, "results", "experiments", run_id)
    for sub in ("episodes", "trials", "trajectories"):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)
    media_dir = args_cli.media or os.path.join(PROJECT_ROOT, "media", "task", run_id)
    if args_cli.enable_cameras:
        os.makedirs(media_dir, exist_ok=True)

    # ---- cache pre-flight: the prologue state (reference class only) + every phase state ------
    # The reference class supplies each state's WORLD (manifest + pieces) and nothing else, so
    # it needs no baked goal sets — those bakes run --skip_goals on purpose. Classes that PLACE
    # in a state need the full triplet.
    ref = config.reference_class()
    wanted = [(SCENARIO, [ref])] if RACK_ACTION is not None else []
    for ph in PHASE_LIST:
        wanted.append((ph.state, list(dict.fromkeys(
            [ref, *sorted({it.object_class for it in ph.items})]))))
    missing = []
    for state, names in wanted:
        placing = {it.object_class for ph in PHASE_LIST if ph.state == state
                   for it in ph.items}
        for name in names:
            cdir = config.scenario_cache_dir(state, object_name=name)
            needed = ["scene_state.json"]
            if name in placing:
                needed += [os.path.join("slots", "slots.json"),
                           os.path.join("slots", "goal_sets.json")]
            for rel in needed:
                if not os.path.exists(os.path.join(cdir, rel)):
                    missing.append((state, name, cdir, rel))
                    break
    if missing:
        mp = (f" --machine {config.MACHINE}" if config.MACHINE != config.MACHINE_BASELINE_NAME
              else "") + (f" --placement {config.BASE_PLACEMENT}"
                          if config.BASE_PLACEMENT != "front" else "")
        for state in dict.fromkeys(m[0] for m in missing):
            names = [m[1] for m in missing if m[0] == state]
            print(f"[FAIL] machine state {state!r} has no collision cache for {names}")
            print(f"       bake it with:\n         scripts/setup/build_state.py{mp} "
                  f"--state {state} --classes {','.join(names)}")
        return 1

    # ---- geometry: per-state worlds + per-class tables ---------------------------------------
    # load_manifest asserts each cache's hash against the CURRENT config, so every state's
    # artifacts load under its own applied scenario; the scene itself is built at SCENARIO.
    #
    # PLACEMENT plans against each class's OWN cache world, not the reference class's: a
    # cache's gripper hulls are extracted at that class's grasp aperture, and judging a
    # fork's flat-lay goals (fingers closed on the handle) in the cup world (fingers spread)
    # phantom-blocks every config against the tray rims — measured 2026-08-14, 48/48 configs
    # alive in the fork world, 0/48 in the cup world. The capacity planner certifies in
    # per-class worlds; the runner must judge with the same geometry. The reference world
    # remains the between-picks/transition view (HOME validity is measured in it).
    ref_worlds, class_worlds = {}, {}
    profiles, goal_sets, slots_by_class, slot_names_by_class = {}, {}, {}, {}
    pieces_by_class: dict = {}  # CoACD pieces are state-independent; first profile wins
    t_w3_ref = None

    def _check_t_w3(world, label):
        nonlocal t_w3_ref
        t_w3 = np.array(world.manifest["t_wrist3_tcp"])
        if t_w3_ref is None:
            t_w3_ref = t_w3
        assert np.allclose(t_w3, t_w3_ref, atol=1e-9), \
            f"t_wrist3_tcp differs between phase caches ({label})"

    for state in dict.fromkeys(phase_states):
        config.apply_scenario(state)
        ph_classes = sorted({it.object_class for pi, it in all_items
                             if PHASE_LIST[pi].state == state})
        merged = not any(config.effective_placement_mode(c) in
                         ("basket_drop", "plate_slot", "flat_lay_third") for c in ph_classes)
        with config.active_object(ref):
            ref_worlds[state] = CollisionWorld(
                cache_dir=config.scenario_cache_dir(state, object_name=ref),
                self_check=True, object_attached=False, merged_cluster=merged)
        _check_t_w3(ref_worlds[state], f"{state}/{ref}")
        class_worlds[state] = {}
        profiles[state], goal_sets[state] = {}, {}
        slots_by_class[state], slot_names_by_class[state] = {}, {}
        for name in ph_classes:
            profiles[state][name] = GraspProfile.build(name, state)
            pieces_by_class.setdefault(name, profiles[state][name].pieces)
            with config.active_object(name):
                class_worlds[state][name] = CollisionWorld(
                    cache_dir=config.scenario_cache_dir(state, object_name=name),
                    self_check=True, object_attached=False, merged_cluster=merged)
            _check_t_w3(class_worlds[state][name], f"{state}/{name}")
            cdir = config.scenario_cache_dir(state, object_name=name)
            with open(os.path.join(cdir, "slots", "slots.json")) as f:
                slots_by_class[state][name] = {s["slot_id"]: placement.SlotFrame.from_json(s)
                                               for s in json.load(f)["slots"]}
            with open(os.path.join(cdir, "slots", "goal_sets.json")) as f:
                goal_sets[state][name] = {g["slot_id"]: np.array(g["configs"])
                                          for g in json.load(f)["goal_sets"]}
            slot_names_by_class[state][name] = placement.slot_names(
                list(slots_by_class[state][name].values()))
            usable = [k for k, v in goal_sets[state][name].items() if len(v)]
            print(f"[INFO] {state}/{name:<8} slots with goal configs: {len(usable)}")
    T_w3_tcp = t_w3_ref

    def all_worlds(state):
        return [ref_worlds[state], *class_worlds[state].values()]

    def set_obstacle_all(state, item_id, pieces, T):
        for w in all_worlds(state):
            if w.has_object(item_id):
                w.set_object_pose(item_id, np.asarray(T))
            else:
                w.add_object(item_id, pieces, np.asarray(T))

    class _ClassWorldPick:
        """Run each pick against the item's OWN cache world, then restore the reference view
        and re-sync the handled item's obstacle across every world of the state (its class
        world saw the clear/re-add during the pick; the others still hold the pre-pick
        pose)."""

        def __init__(self, inner, state):
            self._inner, self._state = inner, state

        def __getattr__(self, name):
            return getattr(self._inner, name)

        def __call__(self, item, candidate):
            motion.world = class_worlds[self._state][item.object_class]
            try:
                return self._inner(item, candidate)
            finally:
                motion.world = ref_worlds[self._state]
                set_obstacle_all(self._state, item.item_id,
                                 pieces_by_class[item.object_class],
                                 measured(item.item_id))

    rack_world = None
    if RACK_ACTION is not None:
        config.apply_scenario(SCENARIO)
        rack_cache = config.scenario_cache_dir(SCENARIO, object_name=ref)
        with config.active_object(ref):
            rack_world = CollisionWorld(cache_dir=rack_cache, self_check=True,
                                        object_attached=False)
    config.apply_scenario(SCENARIO)  # back to the state the SCENE is physically built in

    def make_reach_fn(state, *, thorough=False):
        """Per-phase pick-side feasibility (same semantics as main()'s reach_fn), judged in
        the class's own cache world (its gripper hulls sit at that class's aperture).

        ``thorough`` additionally demands a collision-free hover WITH the payload attached
        for weld-acquire classes — the carry pose the acquisition snaps to. Hover-only checks
        pass spawn draws whose post-acquire carry start then dies with "carry start
        configuration is in collision" (measured on a bowl, phase_smoke7). Attach/detach per
        call is too slow for layout's rejection loop, so the thorough variant runs only in
        the once-per-item post-settle recheck, feeding the wave-resample machinery.
        """
        def reach_fn(object_class: str, T_base_obj: np.ndarray) -> bool:
            prof = profiles[state][object_class]
            world = class_worlds[state][object_class]
            T_grasp = np.asarray(T_base_obj) @ T_inv(prof.T_tcp_obj)
            T_hover = T_grasp.copy()
            T_hover[2, 3] += config.PICK_HOVER_M
            q_seed = np.array(config.HOME_Q)
            hover = [q for q in ik_wrist3_all(T_hover @ T_inv(T_w3_tcp), q_seed=q_seed)
                     if not world.in_collision(q)]
            if not hover:
                return False
            if prof.acquire == "weld":
                if not thorough:
                    return True
                world.attach_object(prof.pieces, T_w3_tcp @ prof.T_tcp_obj)
                try:
                    return any(not world.in_collision(q) for q in hover)
                finally:
                    world.detach_carried_object()
            return any(not world.in_collision(q)
                       for q in ik_wrist3_all(T_grasp @ T_inv(T_w3_tcp), q_seed=hover[0]))
        return reach_fn

    # ---- scene: every phase's items exist from the start, later waves parked off-workspace ---
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device)
    )
    obj_specs = []
    for k, (pi, it) in enumerate(all_items):
        state = PHASE_LIST[pi].state
        # contact filters: gripper prims + SAME-PHASE peers only. Support graphs only ever
        # consult items sharing the counter within a phase, and full-load item counts would
        # otherwise grow the sensor pair count O(N^2) across phases.
        peers = ["{ENV_REGEX_NS}/" + jt.item_id for jt in PHASE_LIST[pi].items
                 if jt.item_id != it.item_id]
        pos_w, quat = _park_pose_w(k)
        obj_specs.append({"name": it.item_id,
                          "usd_path": config.OBJECTS[it.object_class].usd_path,
                          "pos": pos_w, "quat": quat, "contact_filters": peers})
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=False, with_robot_contacts=True, objects=obj_specs))
    welds = {
        it.item_id: dscene.author_weld(
            scene.stage, prim_name=it.item_id,
            T_wrist3_obj=T_w3_tcp @ profiles[PHASE_LIST[pi].state][it.object_class].T_tcp_obj,
        )
        for pi, it in all_items
    }
    ep = config.EPISODE_CAMERA
    cam_specs = {**{k: v for k, v in config.CAMERAS.items()},
                 "episode": (tuple(ep["eye"]), tuple(ep["target"]), dict(ep["lens"]))}
    rig = CameraRig(cam_specs, hw=config.CAMERA_HW) if args_cli.enable_cameras else None
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    for path in welds.values():
        dscene.set_weld_enabled(scene.stage, path, False)
    for pi, it in all_items:
        sensor = scene[f"{it.item_id}_contact"]
        n_filters = len(sensor.cfg.filter_prim_paths_expr)
        assert sensor.contact_physx_view.filter_count == n_filters, (
            f"{it.item_id}: contact filter count {sensor.contact_physx_view.filter_count} != "
            f"{n_filters} configured expressions")

    dt = sim.get_physics_dt()
    robot, dw = scene["robot"], scene["dishwasher"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = robot.data.joint_pos.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]
    gripper_bodies = tuple(sensor_names[1])
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    rec = dtraj.TrajectoryRecorder()
    ctx = _Ctx(scene, sim, dt, robot, arm_ids, sensors, sensor_names, rec)
    episode_id = f"ep{args_cli.seed:03d}"

    def measured(item_id: str) -> np.ndarray:
        o = scene[item_id]
        return T_base_w @ make_T(o.data.root_pos_w[0].cpu().numpy(),
                                 wxyz_to_xyzw(o.data.root_quat_w[0].cpu().numpy()))

    rack_joint_names = sorted(config.RACK_JOINT_TARGETS)
    rack_joint_ids, _ = dw.find_joints(rack_joint_names, preserve_order=True)

    def measure_exts() -> dict:
        jp = dw.data.joint_pos[0].cpu().numpy()
        return {j: float(jp[i]) for j, i in zip(rack_joint_names, rack_joint_ids)}

    def measure_body_pos(body: str) -> np.ndarray:
        bi = dw.body_names.index(body)
        pos_w = dw.data.body_link_pos_w[0, bi].cpu().numpy()
        return (T_base_w @ np.append(pos_w, 1.0))[:3]

    # ---- episode state ------------------------------------------------------------------------
    motion = MotionService(planner, ref_worlds[PHASE_LIST[0].state], ctx)
    master = EpisodeResult(episode_id=episode_id, seed=int(args_cli.seed),
                           classes=[it.object_class for _, it in all_items],
                           cost_fn=args_cli.cost_fn or config.TASK["cost_fn"],
                           status="partial", detail=None)
    master.phases = []
    placed: dict[str, int] = {}      # item_id -> phase index it was placed in
    on_counter: dict[str, str] = {}  # item_id -> object_class, still lying on the counter
    ep_path = os.path.join(run_dir, "episodes", f"{episode_id}.json")
    phase_meta_extra = {"planner": planner.describe(), "scenario": SCENARIO,
                        "post_state": POST_STATE, "run_id": run_id,
                        "phase_states": phase_states, "mode": "full_load"}

    def flush():
        """Cumulative episode record after every wave/phase — a crash stays analyzable."""
        write_episode(ep_path + ".tmp", master, extra=phase_meta_extra)
        os.replace(ep_path + ".tmp", ep_path)

    _write_manifest(run_dir, run_id, planner, phase_states=phase_states)
    transitions_ok = True
    pick_place = None
    writer = None
    trial_backlog: list = []  # (pick, state) written at each phase end with that phase's tmeta

    try:
        for pi, phase in enumerate(PHASE_LIST):
            state = phase.state
            ph_summary = {"phase": pi, "state": state, "n_items": len(phase.items),
                          "n_placed": 0, "n_waves": 0, "transition_ok": None}
            master.phases.append(ph_summary)
            capture, writer = _make_capture(rig, rec, media_dir, args_cli,
                                            dt, f"{episode_id}_p{pi}", video_camera)
            rec.begin(scene, sim, {
                "episode_id": episode_id, "seed": args_cli.seed, "phase": pi,
                "phase_state": state, "classes": master.classes,
                "object_classes": {it.item_id: it.object_class for _, it in all_items},
                "object": sorted(set(master.classes))[0],
                "config_hash": geometry_config_hash(),
                "cameras": {k: {"eye": list(v[0]), "target": list(v[1]),
                                "lens": (dict(v[2]) if len(v) > 2 else config.camera_lens(k))}
                            for k, v in cam_specs.items()},
                "camera_hw": list(config.CAMERA_HW), "camera_fps": int(config.CAMERA_FPS),
                **phase_meta_extra,
            }, object_keys=[it.item_id for _, it in all_items])

            scene_access = _SceneAccess(scene, welds, [it for _, it in all_items],
                                        gripper_bodies, T_base_w, T_w3_tcp, profiles[state])
            inner_pick_place = PickPlace(motion, scene_access, profiles=profiles[state],
                                         goal_sets=goal_sets[state], T_w3_tcp=T_w3_tcp,
                                         seed=args_cli.seed, on_step=capture)
            grasp_finder = GraspFinder(profiles[state], motion)
            inner_pick_place.grasp_finder = grasp_finder
            pick_place = _ClassWorldPick(inner_pick_place, state)

            if pi == 0:
                # start anchor (as in main: settle stepping happens before the first pick)
                ctx.step(30)
                start_home_err = pick_place.home_error()
                if start_home_err >= float(config.TASK["home_tol_rad"]):
                    pick_place.return_home(phase="episode-start-home", settle_steps=30)
                    start_home_err = pick_place.home_error()
                assert start_home_err < float(config.TASK["home_tol_rad"]), (
                    f"episode did not start at HOME_Q: {start_home_err:.4f} rad")
                master.start_home_err_rad = start_home_err
                if RACK_ACTION is not None:
                    rj = dw.find_joints(RACK_ACTION["joint"])[0]
                    rack_runner = task_rack.RackAction(
                        motion, gripper_bodies=gripper_bodies, seed=args_cli.seed,
                        on_step=capture,
                        measure_ext=lambda: float(dw.data.joint_pos[0, rj[0]]))
                    motion.world = rack_world
                    try:
                        rack_phase = task_rack.run_sequence(
                            [task_rack.RackSpec.from_action(RACK_ACTION, rack_world)],
                            rack_runner)
                    finally:
                        motion.world = ref_worlds[state]
                    master.rack_phase = rack_phase.to_json()
                    if not rack_phase.ok:
                        bad = next(o for o in rack_phase.outcomes if not o.ok)
                        master.detail = f"{bad.stage}: {bad.detail}"
                        transitions_ok = False
                        break
                    pick_place.return_home(phase="post-rack-home", settle_steps=30)
            else:
                # ---- scripted transition into this phase's state ------------------------
                pick_place.return_home(phase="pre-transition-home", settle_steps=30)
                gates = config.TASK["transition_slip_gates"]
                riders, slip_gates = {}, {}
                for item_id, ppi in placed.items():
                    riders[item_id] = _PHASE_RIDER_BODY[PHASE_LIST[ppi].state]
                    mode = config.effective_placement_mode(
                        next(it.object_class for _, it in all_items if it.item_id == item_id))
                    slip_gates[item_id] = tuple(gates.get(mode, gates["default"]))
                transition = task_rack.ScriptedTransition(
                    motion, measure_exts=measure_exts,
                    measure_items=lambda: {i: measured(i) for i in placed},
                    measure_body_pos=measure_body_pos,
                    home_q=np.array(config.HOME_Q), on_step=capture)
                outcome = transition.run(
                    tphases.transition_targets(PHASE_LIST[pi - 1].state, state),
                    from_state=PHASE_LIST[pi - 1].state, to_state=state,
                    riders=riders, slip_gates=slip_gates)
                master.transitions.append(outcome.to_json())
                ph_summary["transition_ok"] = outcome.ok
                for item_id, s in outcome.item_slips.items():
                    print(f"[INFO] transition rider {item_id}: slip {s['slip_mm']:.1f} mm "
                          f"rot {s['rot_deg']:.1f} deg ok={s['ok']}")
                print(f"[INFO] transition {PHASE_LIST[pi - 1].state} -> {state}: ok={outcome.ok} "
                      f"{outcome.stage or ''} {outcome.detail or ''}")
                if not outcome.ok:
                    transitions_ok = False
                    master.detail = f"transition to {state}: {outcome.stage}: {outcome.detail}"
                    break
                # world swap + obstacle re-registration at MEASURED post-transition poses in
                # EVERY world of the incoming state (placed items rode their racks; refresh()
                # only ever touches pending items). Pieces come from the state-independent
                # per-class lookup: a leftover from an earlier phase may belong to a class
                # this state never bakes, and loading its cache here would trip the manifest
                # hash against the wrong applied scenario.
                motion.world = ref_worlds[state]
                for item_id in list(placed) + list(on_counter):
                    cls = next(it.object_class for _, it in all_items if it.item_id == item_id)
                    set_obstacle_all(state, item_id, pieces_by_class[cls], measured(item_id))
                pick_place.return_home(phase="post-transition-home", settle_steps=30)

            # ---- slot assignment for this phase --------------------------------------------
            if FULL_LOAD_PLAN_PATH is not None:
                assignment = {}
                for it in phase.items:
                    slot = slots_by_class[state][it.object_class].get(it.slot_id)
                    if slot is None or not len(goal_sets[state][it.object_class].get(
                            it.slot_id, ())):
                        print(f"[WARN] planned slot {it.slot_id} for {it.item_id} has no "
                              f"baked goal configs anymore — item will report no-goal-config")
                        continue
                    assignment[it.item_id] = slot
            else:
                assignment = _assign_slots(
                    [TaskItem(item_id=it.item_id, object_class=it.object_class,
                              instance=it.instance, T_base_obj=np.eye(4)) for it in phase.items],
                    slots_by_class[state], goal_sets[state], slot_names_by_class[state])

            # ---- waves ---------------------------------------------------------------------
            reach_fn = make_reach_fn(state)
            reach_fn_settled = make_reach_fn(state, thorough=True)
            waves = tphases.plan_waves(
                phase.items, rect=dict(config.TASK["spawn_rect_w"]),
                min_separation_m=float(config.TASK["min_separation_m"]),
                radius_fn=lambda c: tlayout.footprint_radius(config.OBJECTS[c]),
                fill_factor=float(config.TASK["wave_fill_factor"]),
                max_wave=config.TASK["max_wave_objects"])
            ph_summary["n_waves"] = len(waves)
            print(f"[INFO] phase {pi} ({state}): {len(phase.items)} items in "
                  f"{[len(w) for w in waves]} wave(s)")

            T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
            for wi, wave in enumerate(waves):
                # leftovers from earlier waves/phases still occupy the counter; the layout
                # rect (and _too_close) works in WORLD xy, measured() in the base frame
                occupied = [((T_w_base @ measured(i))[:2, 3],
                             tlayout.footprint_radius(config.OBJECTS[on_counter[i]]))
                            for i in on_counter]
                wave_classes = [it.object_class for it in wave]
                attempt, layout_ok = 0, False
                while attempt < int(config.TASK["max_layout_retries"]):
                    attempt += 1
                    wseed = int(args_cli.seed) + 9973 * pi + 101 * wi + 10_007 * (attempt - 1)
                    try:
                        items_layout, rejection = tlayout.plan_countertop_layout(
                            wave_classes, seed=wseed, reach_fn=reach_fn,
                            allow_stacking=args_cli.allow_stacking, occupied=occupied)
                    except RuntimeError as exc:
                        print(f"[WARN] wave {pi}/{wi} layout failed (seed {wseed}): {exc}")
                        continue
                    # positional remap: plan_countertop_layout returns one item per requested
                    # class, in request order — re-key onto the wave's global item ids
                    for it, li in zip(wave, items_layout):
                        pos_w, quat = _pose_w(li)
                        scene[it.item_id].write_root_pose_to_sim(
                            root_pose=torch.tensor(np.concatenate([pos_w, xyzw_to_wxyz(quat)])[None],
                                                   dtype=torch.float32, device=device))
                        scene[it.item_id].write_root_velocity_to_sim(
                            root_velocity=torch.zeros((1, 6), dtype=torch.float32,
                                                      device=device))
                    ctx.step(int(config.TASK["settle_steps"]))
                    unreachable = [it.item_id for it in wave
                                   if not reach_fn_settled(it.object_class,
                                                          measured(it.item_id))]
                    if not unreachable:
                        layout_ok = True
                        break
                    print(f"[INFO] wave {pi}/{wi} attempt {attempt}: unreachable after "
                          f"settling: {unreachable} — resampling")
                if not layout_ok:
                    print(f"[WARN] wave {pi}/{wi}: no reachable settled layout in "
                          f"{attempt} attempts — skipping {len(wave)} item(s)")
                    for it in wave:
                        master.unplaced.append(it.item_id)
                        master.blocked_reasons[it.item_id] = "spawn-unreachable-after-settling"
                    flush()
                    continue

                wave_items = [TaskItem(item_id=it.item_id, object_class=it.object_class,
                                       instance=it.instance,
                                       T_base_obj=measured(it.item_id),
                                       radius_m=items_layout[k].radius_m)
                              for k, it in enumerate(wave)]
                for wit in wave_items:
                    on_counter[wit.item_id] = wit.object_class
                    set_obstacle_all(state, wit.item_id,
                                     profiles[state][wit.object_class].pieces,
                                     wit.T_base_obj)

                def refresh(_items=wave_items, _state=state):
                    poses = {t.item_id: measured(t.item_id) for t in _items
                             if t.state == "pending"}
                    for item_id, T in poses.items():
                        t = next(x for x in _items if x.item_id == item_id)
                        set_obstacle_all(_state, item_id,
                                         profiles[_state][t.object_class].pieces, T)
                    return poses

                peer_ids = {it.item_id for it in phase.items}

                def support_fn(remaining):
                    graph = tsupport.build_support_graph(
                        remaining, forces=_peer_forces(scene, remaining, peer_ids))
                    return graph.supports

                seq = TaskSequencer(
                    wave_items, motion, pick_place=pick_place, grasp_fn=grasp_finder,
                    slot_fn=lambda item: assignment.get(item.item_id),
                    cost_fn=args_cli.cost_fn or config.TASK["cost_fn"],
                    refresh_fn=refresh, support_fn=support_fn,
                    recovery=trecovery.make_recovery(),
                    max_recoveries=int(config.TASK["max_recovery_attempts"]),
                    on_event=lambda n, p: print(f"[INFO] {n}: {p}"))
                wave_result = seq.run(episode_id=episode_id, seed=args_cli.seed,
                                      classes=wave_classes)
                merge_wave_results(master, wave_result, phase_index=pi, state=state,
                                   wave_index=wi,
                                   acquired_by_item={it.item_id: it.acquire for it in wave})
                for p in wave_result.picks:
                    trial_backlog.append((p, state))
                    if p.success:
                        placed[p.item_id] = pi
                        on_counter.pop(p.item_id, None)
                ph_summary["n_placed"] = sum(1 for p in master.picks
                                             if p.phase == pi and p.success)
                pick_place.return_home(phase=f"wave-{pi}-{wi}-home", settle_steps=30)
                if rig is not None:
                    rig.save_stills(media_dir, f"{episode_id}_p{pi}w{wi}")
                flush()

            # ---- phase wrap-up: close this phase's recorder segment + write its trials -----
            traj_path = os.path.join(run_dir, "trajectories", f"{episode_id}_p{pi}.npz")
            tmeta = rec.end(traj_path, extra_meta={"phase": pi, "phase_state": state,
                                                   "n_placed": ph_summary["n_placed"]})
            for p, pstate in trial_backlog:
                p.trajectory_path = os.path.relpath(traj_path, PROJECT_ROOT)
                _write_trial(run_dir, master, p, planner, args_cli, tmeta, scenario=pstate)
            trial_backlog = []
            if writer is not None:
                writer.close()
                writer = None
            flush()
    except Exception as exc:  # noqa: BLE001 — the failed episode is the interesting one
        traceback.print_exc()
        master.detail = master.detail or f"{type(exc).__name__}: {exc}"
        transitions_ok = False

    # A break (rack/transition failure) or an exception can leave the current phase's
    # recorder open and its video writer unclosed — the partial segment is still evidence.
    if rec.active:
        try:
            traj_path = os.path.join(run_dir, "trajectories", f"{episode_id}_partial.npz")
            tmeta = rec.end(traj_path, extra_meta={"partial": True})
            for p, pstate in trial_backlog:
                p.trajectory_path = os.path.relpath(traj_path, PROJECT_ROOT)
                _write_trial(run_dir, master, p, planner, args_cli, tmeta, scenario=pstate)
            trial_backlog = []
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] partial trajectory segment lost: {exc}")
    if writer is not None:
        try:
            writer.close()
        except Exception:  # noqa: BLE001
            pass

    # items whose phase never ran (transition failure / crash) are honestly unplaced
    attempted = {p.item_id for p in master.picks} | set(master.unplaced)
    for _, it in all_items:
        if it.item_id not in attempted and it.item_id not in placed:
            master.unplaced.append(it.item_id)

    # ---- closing retreat + final verdict ------------------------------------------------------
    if pick_place is not None:
        scene_access_final = _SceneAccess(scene, welds, [it for _, it in all_items],
                                          gripper_bodies, T_base_w, T_w3_tcp, {})
        placed_before = {i: scene_access_final.object_pose_base(i)[:3, 3].copy()
                         for i in placed}
        try:
            master.home_return_status = pick_place.return_home(phase="episode-home")
            master.end_home_err_rad = pick_place.home_error()
            master.post_home_displacement_mm = {
                i: 1000.0 * float(np.linalg.norm(
                    scene_access_final.object_pose_base(i)[:3, 3] - pos0))
                for i, pos0 in placed_before.items()}
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] closing retreat failed: {exc}")
            master.home_return_status = "failed"

    master.status = full_load_status(master, n_planned=n_planned,
                                     transitions_ok=transitions_ok)
    phase_meta_extra["motion_stats"] = {
        "n_plans": motion.stats.n_plans, "n_plan_failures": motion.stats.n_plan_failures,
        "plan_time_s": round(motion.stats.plan_time_s, 3),
        "n_exec_steps": motion.stats.n_exec_steps}
    flush()
    with open(os.path.join(PROJECT_ROOT, "results", "experiments", "LATEST"), "w") as f:
        f.write(run_id + "\n")

    print(f"[INFO] FULL-LOAD episode {episode_id}: status={master.status} "
          f"placed={master.n_placed}/{n_planned}")
    for ph_summary in master.phases:
        print(f"[INFO]   phase {ph_summary['phase']} ({ph_summary['state']}): "
              f"{ph_summary['n_placed']}/{ph_summary['n_items']} placed in "
              f"{ph_summary['n_waves']} wave(s), transition_ok={ph_summary['transition_ok']}")
    if rig is not None:
        try:
            rig.save_stills(media_dir, f"{episode_id}_final")
            if args_cli.orbit:
                from dishsim.media import write_orbit  # noqa: PLC0415
                center = (float(config.DISHWASHER_POS_W[0]) - 0.15,
                          float(config.DISHWASHER_POS_W[1]), 0.45)
                write_orbit(rig, scene, sim, dt,
                            os.path.join(media_dir, f"{episode_id}_orbit.mp4"),
                            center=center, radius=1.7, height=0.8)
            print(f"[INFO] media -> {os.path.relpath(media_dir, PROJECT_ROOT)}")
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] media write failed: {exc}")
    print(f"[RESULT] {'PASS' if master.status == 'cleared' else 'FAIL'} ({master.status})")
    return 0 if master.status == "cleared" else 1


def _assign_slots(items, slots_by_class, goal_sets, slot_names_by_class) -> dict:
    """Assign each item a destination slot (semantics and rationale: :mod:`dishsim.task.slotting`).

    Thin adapter binding the shared packing rule to this run's config: the EFFECTIVE placement
    mode (machine overrides applied — on the Bosch a fork's ``basket_drop`` reroutes to
    ``flat_lay_third``, and the ``slot_pools`` lookup must follow it), registry body radii, and
    the ``TASK`` preference tables.
    """
    return tslotting.assign_slots(
        [tslotting.SlotRequest(it.item_id, it.object_class) for it in items],
        slots_by_class, goal_sets, slot_names_by_class,
        type_slots=config.TASK.get("type_slots") or {},
        slot_pools=config.TASK["slot_pools"],
        margin_m=float(config.TASK["slot_separation_margin_m"]),
        mode_fn=config.effective_placement_mode,
        rim_radius_fn=lambda cls: config.OBJECTS[cls].rim_radius_m,
    )


def _pose_w(item):
    """Layout pose -> world (position, XYZW quaternion) for spawning.

    Full base transform: a yawed base rotates the spawn pose too, not just its position.
    """
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    return T_to_pos_quat(T_w_base @ item.T_base_obj)  # XYZW


class _Ctx:
    """Concrete :class:`dishsim.task.motion.ExecContext` over a live Isaac scene."""

    def __init__(self, scene, sim, dt, robot, arm_ids, sensors, sensor_names, rec):
        self.scene, self.sim, self.dt = scene, sim, dt
        self.robot, self.arm_ids = robot, arm_ids
        self.sensors, self.sensor_names = sensors, sensor_names
        self.rec = rec
        self._aperture = config.GRIPPER_APERTURE_OPEN_RAD
        self._arm_q = None
        self._payload = None  # item_id of the carried object, for the pad-reaction subtraction

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self.scene.write_data_to_sim()
            self.sim.step()
            self.scene.update(self.dt)
            self.rec.sample()

    def hold(self, arm_q=None, aperture=None) -> None:
        if arm_q is not None:
            self._arm_q = arm_q
        if aperture is not None:
            self._aperture = aperture
        dscene.hold_targets(self.scene, aperture=self._aperture, arm_q=self._arm_q)

    def arm_q(self):
        return self.robot.data.joint_pos[0, self.arm_ids].cpu().numpy().astype(float)

    def contact_peak(self, exclude=()):
        key = f"{self._payload}_contact" if self._payload else None
        return dscene.unexpected_robot_contact(self.scene, self.sensors, self.sensor_names,
                                               exclude=exclude, object_key=key)

    def set_payload(self, key):
        self._payload = key

    def set_rack_target(self, targets) -> None:
        dscene.set_rack_target_override(targets)

    def ramp_aperture(self, aperture_rad: float, steps: int, arm_q=None, on_step=None) -> float:
        self._aperture = aperture_rad
        if arm_q is not None:
            self._arm_q = arm_q
        # ramp_gripper's per_step is 0-based while every other hook here is 1-based
        per_step = None if on_step is None else (lambda i: on_step(i + 1))
        return dscene.ramp_gripper(self.scene, self.sim, aperture_rad, steps,
                                   arm_q=self._arm_q, per_step=per_step, step_fn=self.step)

    def set_phase(self, name: str) -> None:
        self.rec.set_phase(name)


class _SceneAccess:
    """Concrete :class:`dishsim.task.primitives.SceneAccess`, keyed on item_id.

    Object prims are named after their item_id, so the scene lookup, the weld and the contact
    sensor all resolve from one key with no instance bookkeeping.
    """

    def __init__(self, scene, welds, items, gripper_bodies, T_base_w, T_w3_tcp, profiles):
        self.scene, self.welds = scene, welds
        self.peer_ids = {it.item_id for it in items}
        self._gripper_bodies = gripper_bodies
        self.T_base_w, self.T_w3_tcp = T_base_w, T_w3_tcp
        self.T_w_base = T_inv(T_base_w)  # base -> world, for writing poses back to the sim
        self.profiles = profiles

    def object_pose_base(self, item_id: str) -> np.ndarray:
        o = self.scene[item_id]
        return self.T_base_w @ make_T(o.data.root_pos_w[0].cpu().numpy(),
                                      wxyz_to_xyzw(o.data.root_quat_w[0].cpu().numpy()))

    def set_weld(self, item_id: str, enabled: bool) -> None:
        dscene.set_weld_enabled(self.scene.stage, self.welds[item_id], enabled)

    def grip_forces(self, item_id: str) -> dict:
        return _grip_forces_named(self.scene, item_id, self.peer_ids)

    def grasp_origin_base(self, arm_q, object_class: str) -> np.ndarray:
        from dishsim.ur5e_kin import fk_wrist3  # noqa: PLC0415

        T = (fk_wrist3(np.asarray(arm_q, dtype=float)) @ self.T_w3_tcp
             @ self.profiles[object_class].T_tcp_obj)
        return T[:3, 3]

    def gripper_bodies(self) -> tuple:
        return self._gripper_bodies

    def place_in_gripper(self, item_id: str, arm_q, object_class: str) -> None:
        from dishsim.ur5e_kin import fk_wrist3  # noqa: PLC0415

        T_base_obj = (fk_wrist3(np.asarray(arm_q, dtype=float)) @ self.T_w3_tcp
                      @ self.profiles[object_class].T_tcp_obj)
        obj = self.scene[item_id]
        # full base transform: position AND quaternion compose with the (possibly yawed) base
        pos_w, quat = T_to_pos_quat(self.T_w_base @ T_base_obj)  # XYZW, as everywhere here
        pose = torch.tensor(np.concatenate([pos_w, xyzw_to_wxyz(quat)]), dtype=torch.float32,
                            device=obj.data.root_pos_w.device).unsqueeze(0)
        obj.write_root_pose_to_sim(root_pose=pose)
        # zero the velocity too: the object arrives carrying whatever motion its countertop
        # settle left it with, and a moving body fights the weld the instant it is enabled
        obj.write_root_velocity_to_sim(root_velocity=torch.zeros_like(pose[:, :6]))


def _peer_forces(scene, items, peer_ids) -> dict:
    """Object-to-object contact forces, ``{item_id: {peer_id: force_N}}``.

    Each object's ContactSensor filters against the gripper prims AND its peers, so the peer
    columns of ``force_matrix_w`` are exactly the object-object pairs. Entries whose filter is a
    robot prim are dropped here — only peers become support edges.
    """
    out = {}
    for it in items:
        sensor = scene[f"{it.item_id}_contact"]
        mat = getattr(sensor.data, "force_matrix_w", None)
        if mat is None:
            out[it.item_id] = {}
            continue
        mags = np.linalg.norm(mat[0, 0].cpu().numpy(), axis=-1)
        names = [p.rsplit("/", 1)[-1] for p in sensor.cfg.filter_prim_paths_expr]
        out[it.item_id] = {n: float(m) for n, m in zip(names, mags) if n in peer_ids}
    return out


def _grip_forces_named(scene, item_id: str, peer_ids=()) -> dict:
    """Grip forces for a named object, split into ROBOT partners and peer objects.

    Two corrections over a naive read of ``force_matrix_w``, both of which produce silent false
    verdicts rather than errors:

    * **Peers are not robot contact.** Stage B widened every object's filter list with its peers
      so the support graph could be read from real contacts. The grasp gates iterate "every
      partner that is not a pad" and fail the pick on any of them, so without separating the two
      an object merely resting against its neighbour on the counter would be reported as a
      grasp fault. Peers are returned separately and are the support graph's business.
    * **The external residual is the magnitude of a vector difference, not a difference of
      magnitudes.** ``||F_net|| - ||sum F_filtered||`` collapses toward zero whenever the
      unfiltered force is near-orthogonal to the pad resultant — which is exactly the geometry
      here, since a top-down rim grasp squeezes horizontally while a support reaction is
      vertical. With a 31 N pad resultant, 2.4 N of leftover countertop support computes as
      0.09 N and slips under the 0.1 N gate that exists to catch precisely that.

    Args:
        scene: Live interactive scene.
        item_id: Object whose sensor to read.
        peer_ids: Item ids that are other OBJECTS rather than robot bodies.

    Returns:
        ``{"partners", "peers", "pads_n", "external_n", "net_n"}``; ``partners`` holds robot
        bodies only.
    """
    sensor = scene[f"{item_id}_contact"]
    mat = getattr(sensor.data, "force_matrix_w", None)
    if mat is None:
        return {"partners": {}, "peers": {}, "pads_n": [0.0, 0.0], "external_n": 0.0, "net_n": 0.0}
    fm = mat[0, 0].cpu().numpy()  # [n_filters, 3]; filter order matches the cfg list
    net_vec = sensor.data.net_forces_w[0, 0].cpu().numpy()
    names = [p.rsplit("/", 1)[-1] for p in sensor.cfg.filter_prim_paths_expr]
    mags = np.linalg.norm(fm, axis=-1)
    everything = {n: float(m) for n, m in zip(names, mags)}
    for pad in config.GRIP_PAD_BODIES:
        assert pad in everything, f"pad {pad} missing from the {item_id} contact filter list"
    peers = set(peer_ids)
    return {
        "partners": {n: v for n, v in everything.items() if v > 0.01 and n not in peers},
        "peers": {n: v for n, v in everything.items() if v > 0.01 and n in peers},
        "pads_n": [everything[b] for b in config.GRIP_PAD_BODIES],
        "external_n": float(np.linalg.norm(net_vec - fm.sum(axis=0))),
        "net_n": float(np.linalg.norm(net_vec)),
    }


def _write_manifest(run_dir, run_id, planner, *, phase_states=None, extra=None):
    """Run-level provenance (new in v3; episode runs previously wrote none and Phase 3 lost
    the run id / planner / config hashes)."""
    doc = {
        "run_id": run_id, "argv": sys.argv[1:], "seed": int(args_cli.seed),
        "machine": config.MACHINE, "base_placement": config.BASE_PLACEMENT,
        "scenario": SCENARIO, "post_state": POST_STATE,
        "phase_states": list(phase_states or []),
        "planner": planner.describe(),
        "full_load_plan": (os.path.relpath(FULL_LOAD_PLAN_PATH, PROJECT_ROOT)
                           if FULL_LOAD_PLAN_PATH else None),
        "episode_schema_version": EPISODE_SCHEMA_VERSION,
        "trial_schema_version": TRIAL_SCHEMA_VERSION,
    }
    hashes = {}
    entry = config.SCENARIO_NAME
    try:
        for state in dict.fromkeys([POST_STATE, *(phase_states or [])]):
            config.apply_scenario(state)
            with config.active_object(config.reference_class()):
                hashes[state] = geometry_config_hash()
    finally:
        config.apply_scenario(entry)
    doc["config_hash_by_state"] = hashes
    doc["config_hash"] = hashes.get(POST_STATE)  # what metrics.summarize echoes
    doc.update(extra or {})
    path = os.path.join(run_dir, "manifest.json")
    with open(path + ".tmp", "w") as f:
        json.dump(doc, f, indent=2)
    os.replace(path + ".tmp", path)
    return path


def _make_capture(rig, rec, media_dir, args_cli, dt, episode_id, camera):
    """Frame counter + inline video writer, matching run_trials.py's capture discipline.

    ``mark_captured`` is called whether or not a video is being written, so the recorded
    ``captured`` mask is identical with and without cameras — Phase 3 replays the same frames
    either way.
    """
    if rig is None:
        def counter(_n=None):
            rec.mark_captured()
        return counter, None

    writer = VideoWriter(os.path.join(media_dir, f"{episode_id}.mp4"), fps=config.CAMERA_FPS)
    state = {"i": 0}

    def capture(_n=None):
        state["i"] += 1
        if state["i"] % max(1, args_cli.video_stride) != 0:
            return
        rec.mark_captured()
        rig.update(dt)
        writer.add(rig.grab_one(camera))

    return capture, writer


def _finish_media(rig, writer, media_dir, result):
    try:
        if writer is not None:
            writer.close()
        rig.save_stills(media_dir, f"{result.episode_id}_final")
        print(f"[INFO] media -> {os.path.relpath(media_dir, PROJECT_ROOT)}")
    except Exception as exc:  # media is evidence, not the experiment: never fail the run on it
        print(f"[WARN] media write failed: {exc}")


def _write_trial(run_dir, result, pick, planner, args_cli, tmeta, *, scenario=None):
    """One per-pick record in the EXISTING trial schema, so Phase 3 reads it unchanged.

    Phase mode passes ``scenario`` (the pick's phase state) and stamps two extra keys —
    absent in single-phase records, which therefore stay byte-identical.
    """
    tag = f"{result.episode_id}_{pick.item_id}"
    record = {
        "schema_version": TRIAL_SCHEMA_VERSION,
        "trial": tag, "slot": pick.goal_slot, "seed": result.seed, "repeat": pick.order,
        "object": pick.object_class,
        # "weld" means the object was SNAPPED into the hand, not picked — the honest label for a
        # grasp family with no calibrated countertop pick. Never aggregate the two as one number.
        "acquired": ("weld" if config.OBJECTS[pick.object_class].grasp.family
                     in config.WELD_ACQUIRE_FAMILIES else "pick"),
        "start_welded": config.OBJECTS[pick.object_class].grasp.family
                        in config.WELD_ACQUIRE_FAMILIES,
        "placement_mode": config.effective_placement_mode(pick.object_class),
        "planner": planner.describe(),
        "scenario": scenario or SCENARIO, "rack_action": RACK_ACTION,
        "success": pick.success, "failure_stage": pick.failure_stage,
        "failure_detail": pick.failure_detail,
        "plan_time_s": pick.plan_time_s, "path_len_rad": None,
        "exec_steps": tmeta.get("n_steps"), "goal_config_index": None,
        "grasp_force_n": None, "exec_pad_peak_n": None, "retract": None,
        "rack_final_err_m": None, "pick_weld_err_mm": None,
        "final_pose_err": None,
        "trajectory": {"path": pick.trajectory_path, "n_steps": tmeta.get("n_steps"),
                       "n_captured": tmeta.get("n_captured")},
        "media": {},
        "episode": {"episode_id": result.episode_id, "order": pick.order,
                    "cost": pick.cost, "n_candidates": pick.n_candidates},
    }
    if pick.phase is not None:  # phase-mode extras; single-phase records stay byte-identical
        record["phase"] = pick.phase
        record["phase_state"] = pick.phase_state
        record["wave"] = pick.wave
    path = os.path.join(run_dir, "trials", f"{tag}.json")
    with open(path, "w") as f:
        json.dump(record, f, indent=2)
    pick.trial_path = os.path.relpath(path, PROJECT_ROOT)
    return path


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
