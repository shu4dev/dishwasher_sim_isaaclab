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
parser.add_argument("--out", type=str, default=None, help="Run directory.")
parser.add_argument("--run_id", type=str, default=None, help="Run name.")
parser.add_argument("--media", type=str, default=None, help="Media directory.")
parser.add_argument("--video_stride", type=int, default=2, help="Capture every Nth physics step.")
parser.add_argument("--video_camera", type=str, default=None,
                    help="Camera the episode MP4 is written from (default: config.TASK).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)  # boot Kit BEFORE importing dishsim/isaaclab scene modules
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab.scene import InteractiveScene  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

from dishsim import config  # noqa: E402

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
from dishsim.task import support as tsupport  # noqa: E402
from dishsim.task.grasp import GraspFinder  # noqa: E402
from dishsim.task.episode import EpisodeResult, write_episode  # noqa: E402
from dishsim.task.motion import MotionService  # noqa: E402
from dishsim.task.primitives import GraspProfile, PickPlace  # noqa: E402
from dishsim.task.sequencer import TaskItem, TaskSequencer  # noqa: E402
from dishsim.transforms import T_inv, make_T  # noqa: E402
from dishsim.ur5e_kin import ik_wrist3_all  # noqa: E402

TRIAL_SCHEMA_VERSION = 1


def main() -> int:
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
    # The mug is checked unconditionally: the machine world is loaded from it whatever the pool.
    #
    # An episode with a rack action spans TWO states and therefore two sets of caches. The rack
    # phase plans in the START state but carries nothing, so it needs only the scenario-level
    # (mug-keyed) machine cache; every pick and place happens after the action and needs the
    # per-class caches of the POST state. Checking both against one state is what would send you
    # to bake `objects/cup/both_in`, which nothing ever reads.
    wanted = [(SCENARIO, ["mug"])] if RACK_ACTION is not None else []
    wanted.append((POST_STATE, list(dict.fromkeys(["mug", *classes_pool]))))
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
        rack_cache = config.scenario_cache_dir(SCENARIO, object_name="mug")
        rack_world = CollisionWorld(cache_dir=rack_cache, self_check=True, object_attached=False)
        print(f"[INFO] rack world:      {os.path.relpath(rack_cache, PROJECT_ROOT)} ({SCENARIO})")
        config.apply_scenario(POST_STATE)

    # Per-piece cluster whenever any pooled class needs a thin insertion: the merged gripper+
    # payload hull is a single convex wedge that provably cannot enter a cutlery bay or a 30 mm
    # tine gap. Merged is 3x faster and strictly conservative, so it stays the default for pools
    # that only stand things on the rack floor.
    merged = not any(config.OBJECTS[c].placement.mode in ("basket_drop", "plate_slot")
                     for c in classes_pool)
    cache_dir = config.scenario_cache_dir(POST_STATE, object_name="mug")
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
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg())
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
        assert sensor.contact_view.filter_count == n_filters, (
            f"{it.item_id}: contact filter count {sensor.contact_view.filter_count} != "
            f"{n_filters} configured expressions — the force_matrix_w zip would be misaligned"
        )

    dt = sim.get_physics_dt()
    robot = scene["robot"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = robot.data.joint_pos.torch.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]
    gripper_bodies = tuple(sensor_names[1])
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    rec = dtraj.TrajectoryRecorder()

    ctx = _Ctx(scene, sim, dt, robot, arm_ids, sensors, sensor_names, rec)

    # ---- spawn -> settle -> reachability check -> resample --------------------------------------
    def measured(item_id: str) -> np.ndarray:
        o = scene[item_id]
        return T_base_w @ make_T(o.data.root_pos_w.torch[0].cpu().numpy(),
                                 o.data.root_quat_w.torch[0].cpu().numpy())

    def spawn_and_settle(layout):
        """Teleport to a layout, settle physics, and report the MEASURED poses.

        Prims are reused rather than respawned: the object set is fixed when the scene is built,
        and only the poses vary between layout attempts.
        """
        for it in layout:
            pos_w, quat = _pose_w(it)
            scene[it.item_id].write_root_pose_to_sim_index(
                root_pose=torch.tensor(np.concatenate([pos_w, quat])[None],
                                       dtype=torch.float32, device=device))
            scene[it.item_id].write_root_velocity_to_sim_index(
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
                scene["dishwasher"].data.joint_pos.torch[0, rack_joint_ids[0]]),
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


def _assign_slots(items, slots_by_class, goal_sets, slot_names_by_class) -> dict:
    """Assign each item a destination slot: feasible, unused, and not overlapping another item.

    Two independent scarcities bite here, and both are measured rather than assumed.

    **Reachability.** Only a few slots have any goal configurations at all — on the current rack
    4 of 15, clustered at the front, because the rear cells sit where the carried object cannot
    be brought in without hitting the machine. Empty goal sets there are expected signal
    (docs/success_criteria.md), not a bug.

    **Overlap.** The slot grid is an *overlapping candidate* grid, laid out at
    ``SLOT_GRID_PITCH_M`` = 60 mm for a task that places ONE object per trial and wants dense
    coverage. Two objects assigned to adjacent candidates would be told to occupy the same
    space. Compatibility is therefore checked against each class's body radius
    (``rim_radius_m`` — the standing circle, not the bbox diagonal, so a mug's handle does not
    veto a neighbour it would simply be turned away from).

    **Preference.** ``TASK["type_slots"]`` gives an ORDERED list of slot NAMES per object type.
    Order is a preference, not a permission: it decides which slot a type reaches for first,
    never whether a slot is legal. Legality is still a non-empty goal set plus no overlap with an
    already-assigned slot, so a named slot that is infeasible for this class, or already taken,
    is skipped rather than failing the object.

    Items are served most-constrained-first, the standard heuristic, and that stays load-bearing
    even with explicit lists — a list makes a type MORE constrained, not less. Measured: the mug
    can use 2 slots and the cup 4, so serving the mug first gets both placed; reverse it and the
    cup takes the mug's slot and the mug gets nothing. Items left without a slot are returned
    absent, and the caller reports them rather than silently dropping them.
    """
    type_slots = config.TASK.get("type_slots") or {}
    feasible, sources = {}, {}
    for it in items:
        cls = it.object_class
        mode = config.OBJECTS[cls].placement.mode
        names = type_slots.get(cls)
        pool = config.TASK["slot_pools"].get(mode)
        # `is not None`, not truthiness: an EMPTY list is a deliberate "this type has nowhere to
        # go", and must not silently fall through to every slot in the rack.
        if names is not None:
            by_name = slot_names_by_class[cls]
            unknown = [n for n in names if n not in by_name]
            if unknown:
                raise SystemExit(
                    f"[FAIL] TASK['type_slots'][{cls!r}] names {unknown}, which do not exist for "
                    f"placement mode {mode!r}. Valid names: {sorted(by_name)}")
            ids, sources[it.item_id] = [by_name[n] for n in names], "type_slots"
        elif pool is not None:
            ids, sources[it.item_id] = list(pool), "slot_pools"
        else:
            ids, sources[it.item_id] = sorted(slots_by_class[cls]), "all"
        feasible[it.item_id] = [s for s in ids if len(goal_sets[cls].get(s, ())) > 0]

    margin = float(config.TASK["slot_separation_margin_m"])
    out, taken = {}, []  # taken: (centre, radius) of slots already handed out
    for it in sorted(items, key=lambda i: (len(feasible[i.item_id]), i.item_id)):
        r = float(config.OBJECTS[it.object_class].rim_radius_m)
        for sid in feasible[it.item_id]:
            slot = slots_by_class[it.object_class][sid]
            centre = slot.T_base_slot[:3, 3]
            if any(float(np.linalg.norm(centre - c)) < (r + rr + margin) for c, rr in taken):
                continue
            taken.append((centre, r))
            out[it.item_id] = slot
            break
    return out


def _pose_w(item):
    """Layout pose -> world (position, XYZW quaternion) for spawning."""
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    pos_w = item.T_base_obj[:3, 3] + np.asarray(config.ROBOT_BASE_POS_W)
    quat = Rotation.from_matrix(item.T_base_obj[:3, :3]).as_quat()  # XYZW
    return pos_w, quat


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
        return self.robot.data.joint_pos.torch[0, self.arm_ids].cpu().numpy().astype(float)

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
        self.profiles = profiles

    def object_pose_base(self, item_id: str) -> np.ndarray:
        o = self.scene[item_id]
        return self.T_base_w @ make_T(o.data.root_pos_w.torch[0].cpu().numpy(),
                                      o.data.root_quat_w.torch[0].cpu().numpy())

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
        from scipy.spatial.transform import Rotation  # noqa: PLC0415

        T_base_obj = (fk_wrist3(np.asarray(arm_q, dtype=float)) @ self.T_w3_tcp
                      @ self.profiles[object_class].T_tcp_obj)
        obj = self.scene[item_id]
        pos_w = T_base_obj[:3, 3] + np.asarray(config.ROBOT_BASE_POS_W)
        quat = Rotation.from_matrix(T_base_obj[:3, :3]).as_quat()  # XYZW, as everywhere here
        pose = torch.tensor(np.concatenate([pos_w, quat]), dtype=torch.float32,
                            device=obj.data.root_pos_w.torch.device).unsqueeze(0)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        # zero the velocity too: the object arrives carrying whatever motion its countertop
        # settle left it with, and a moving body fights the weld the instant it is enabled
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(pose[:, :6]))


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
        mags = np.linalg.norm(mat.torch[0, 0].cpu().numpy(), axis=-1)
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
    fm = mat.torch[0, 0].cpu().numpy()  # [n_filters, 3]; filter order matches the cfg list
    net_vec = sensor.data.net_forces_w.torch[0, 0].cpu().numpy()
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


def _write_trial(run_dir, result, pick, planner, args_cli, tmeta):
    """One per-pick record in the EXISTING trial schema, so Phase 3 reads it unchanged."""
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
        "placement_mode": config.OBJECTS[pick.object_class].placement.mode,
        "planner": planner.describe(),
        "scenario": SCENARIO, "rack_action": RACK_ACTION,
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
    path = os.path.join(run_dir, "trials", f"{tag}.json")
    with open(path, "w") as f:
        json.dump(record, f, indent=2)
    pick.trial_path = os.path.relpath(path, PROJECT_ROOT)
    return path


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
