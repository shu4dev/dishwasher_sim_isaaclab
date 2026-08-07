# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase F: rack reconfiguration -> countertop pick -> plan -> place -> evaluate, per trial.

The both_in / both_out scenarios are INITIAL machine states; the machine itself is unmodified.
Each trial does what a human does:

1. **Rack phase** — plan to the moved rack's front handle, engage it (open-jaw wrap for a
   pull, closed fist for a push), then slide the rack drive-synchronized: the joint's drive
   target ramps to ``rack_action["to"]`` while the arm tracks the moving handle along a
   collision-validated IK path (the same visible-contact + guaranteed-actuation methodology as
   the mug weld). Gates: rack lands within ``RACK_SLIDE_TOL_M``; arm links stay contact-free.
   Both scenarios converge to the shared placement state (lower rack out, upper in).
2. **Pick phase** — the mug starts free-standing on the dishwasher countertop. Plan to a
   pre-grasp hover over it, descend on a straight scripted line, verify zero contact with the
   jaws open, close to the calibrated pinch band (the C2 gate), enable the hidden weld
   (zero-error by construction: the weld anchors ARE the grasp transform), verify the weld,
   and lift clear.
3. **Place phase** — unchanged from the validated pipeline: RRT-Connect to a Phase-E goal set
   (computed on the placement-state cache), execute with contact monitoring, visible open
   before the weld releases, validated tool-axis retract, settle, evaluate
   docs/success_criteria.md against the slot frame.

Every trial writes ``results/.../trial_<id>.json``, a full MP4, and a final close-up still.

Run with (shake-out):
    scripts/run_kit.sh scripts/20_plan_and_place.py --headless --enable_cameras \
        --scenario both_in --slots 2 --seeds 0
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="OMPL plan-and-place trials with rack + pick phases.")
parser.add_argument("--slots", type=str, default="1,2,3", help="Comma-separated slot ids.")
parser.add_argument("--seeds", type=str, default="0", help="Comma list or a-b range of RNG seeds.")
parser.add_argument("--repeats", type=int, default=1, help="Trials per (slot, seed) pair.")
parser.add_argument("--out", type=str, default=None,
                    help="Trial-JSON dir (default: results/ or results/<scenario>).")
parser.add_argument("--media", type=str, default=None,
                    help="Media dir (default: media/F or media/F/<scenario>).")
parser.add_argument("--skip_existing", action="store_true", help="Skip trials whose JSON exists (resume).")
parser.add_argument("--object", type=str, default="mug", help="Carried object class (see config.OBJECTS).")
parser.add_argument("--scenario", type=str, default="both_out",
                    help="Initial rack-state scenario (see config.SCENARIOS).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

# scenario BEFORE scene/robots imports — they bind rack targets + the derived USD at import
config.set_active_object(args_cli.object)
config.apply_scenario(args_cli.scenario)
if args_cli.out is None and args_cli.object != "mug":
    args_cli.out = os.path.join(PROJECT_ROOT, "results", "demos", args_cli.object)
if args_cli.media is None and args_cli.object != "mug":
    args_cli.media = os.path.join(config.scenario_media_dir("F"), args_cli.object)
if args_cli.out is None:
    args_cli.out = (os.path.join(PROJECT_ROOT, "results") if config.SCENARIO_NAME == "both_out"
                    else os.path.join(PROJECT_ROOT, "results", config.SCENARIO_NAME))
if args_cli.media is None:
    args_cli.media = config.scenario_media_dir("F")

from scipy.spatial.transform import Rotation  # noqa: E402

from dishsim import planning, rack_ops  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.geometry import config_hash as geometry_config_hash  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.placement import SlotFrame, evaluate_placement  # noqa: E402
from dishsim.transforms import T_inv, make_T  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3, ik_wrist3_all  # noqa: E402

# Rack action of the initial scenario. Internal states (e.g. --scenario placement) have the
# racks pre-positioned and no rack_action: Phase R is skipped — used by the per-class object
# demos, where rack reconfiguration is object-independent (demonstrated by the mug campaign).
ACTION = config.state_params(args_cli.scenario).get("rack_action")

# Weld-carried classes skip Phase P and START WELDED at the carry pose — the v0 framing
# (grasp acquisition out of scope): edge-pinch discs cannot be pinched off a flat countertop
# (the jaw would have to close across the full disc), and rim-edge thin walls give a binary
# 0->60 N contact against the rigid weld (measured in the scripts/11 staircase) — for both,
# the visible close stops at first contact and the weld carries.
# handle_pinch joined the weld-carried set after measurement: the near-closed pad arc
# drags a free-lying 30 g fork ~21 mm during the close (weld gate 5 mm) — real systems
# solve this with tactile servoing, out of scope here
START_WELDED = config.active_object_spec().grasp.family in ("edge_pinch", "rim_edge", "handle_pinch")


def parse_ids(spec: str) -> list[int]:
    out = []
    for tok in spec.split(","):
        tok = tok.strip()
        if "-" in tok[1:]:
            a, b = tok.split("-")
            out.extend(range(int(a), int(b) + 1))
        elif tok:
            out.append(int(tok))
    return out


def countertop_staging_rot(yaw_deg: float) -> "Rotation":
    """World rotation of the object staged on the countertop (axis convention aware).

    Y-up objects (the legacy mug) stand via Rx(+90); Z-up objects stand as authored; X-up
    objects (cutlery/utensils) lie flat as authored. Yaw spins about world z on top.
    """
    yaw = Rotation.from_euler("z", np.radians(yaw_deg))
    if tuple(config.OBJECT_AXIS_OBJ) == (0.0, 1.0, 0.0):
        return yaw * Rotation.from_euler("x", np.pi / 2)
    return yaw


def countertop_T_base(instance: int = 0) -> np.ndarray:
    """The object's staged pose (base frame) for a trial instance."""
    (pos_w, yaw_deg) = config.OBJECT_COUNTERTOP_POSES_W[instance]
    pos_b = np.array(pos_w) - np.array(config.ROBOT_BASE_POS_W)
    return make_T(pos_b, countertop_staging_rot(yaw_deg).as_quat())


def main() -> None:
    os.makedirs(args_cli.out, exist_ok=True)
    os.makedirs(args_cli.media, exist_ok=True)

    # ---- collision worlds ---------------------------------------------------------------------
    # Placement-state cache: slots/goals + pick/place/retract planning. For internal states
    # (no rack action) the trial PLACES in that state directly — e.g. placement_open (both
    # racks out) for the rear plate bank, which the stowed upper rack otherwise blocks.
    place_cache_state = args_cli.scenario if ACTION is None else config.PLACEMENT_STATE
    placement_cache = config.scenario_cache_dir(place_cache_state)
    with open(os.path.join(placement_cache, "slots", "slots.json")) as f:
        slots_doc = json.load(f)
    slots = {s["slot_id"]: SlotFrame.from_json(s) for s in slots_doc["slots"]}
    with open(os.path.join(placement_cache, "slots", "goal_sets.json")) as f:
        goal_sets = {g["slot_id"]: np.array(g["configs"]) for g in json.load(f)["goal_sets"]}

    place_state = config.SCENARIO_NAME
    config.apply_scenario(place_cache_state)  # manifest hash of the placement cache
    if slots_doc.get("config_hash") != geometry_config_hash():
        raise SystemExit(
            f"[FAIL] slots.json is stale for object {config.ACTIVE_OBJECT!r} in the placement "
            f"state — re-run scripts/15_goal_configs.py --object {config.ACTIVE_OBJECT}"
        )
    # per-piece cluster for thin-insertion modes (the merged gripper+object hull cannot
    # enter a bay or a tine gap); merged (3x faster, strictly conservative) otherwise
    merged = config.active_object_spec().placement.mode not in ("basket_drop", "plate_slot")
    world = CollisionWorld(cache_dir=placement_cache, self_check=True, merged_cluster=merged)
    retract_world = CollisionWorld(cache_dir=placement_cache, self_check=True, merged_cluster=merged)
    pick_world = CollisionWorld(cache_dir=placement_cache, self_check=True, object_attached=False)
    config.apply_scenario(place_state)  # back to the trial's initial state
    mug_pieces = retract_world.carried_object_pieces()
    retract_world.detach_carried_object()

    if ACTION is not None:
        # initial-state cache: rack-approach + slide planning (nothing carried yet)
        rack_world = CollisionWorld(cache_dir=config.scenario_cache_dir(), self_check=True, object_attached=False)
        rack_body = ACTION["body"]
        rack_joint = ACTION["joint"]
        rack_params = config.RACK_GEN[rack_body]
        T_base_rack = np.array(rack_world.manifest["statics"][rack_body]["T_base_body"])
        state0 = rack_world.manifest["statics_state"]
        cached_ext = state0["rack_upper_m"] if rack_joint.endswith("_up") else state0["rack_lower_m"]
        e0, e1 = float(cached_ext), float(ACTION["to"])
    else:  # internal state: racks pre-positioned, no rack phase
        rack_world, rack_body, rack_joint, rack_params = None, None, None, None
        T_base_rack, e0, e1 = None, 0.0, 0.0

    T_base_mug0 = countertop_T_base(0)
    if not START_WELDED:
        for w in ((pick_world,) if rack_world is None else (rack_world, pick_world)):
            w.add_object("countertop_mug", mug_pieces, T_base_mug0)

    T_w3_tcp = rack_ops.t_wrist3_tcp()
    T_tcp_obj = make_T(config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT)
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)

    # ---- scene --------------------------------------------------------------------------------
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg())
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=True, with_robot_contacts=True))
    weld_path = dscene.author_weld(scene.stage)
    dscene.set_weld_enabled(scene.stage, weld_path, False)  # the mug starts FREE on the countertop
    rig = CameraRig(config.CAMERAS, hw=config.CAMERA_HW) if args_cli.enable_cameras else None
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()
    robot = scene["robot"]
    obj = scene["carried_object"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    dw = scene["dishwasher"]
    rack_jids = dw.find_joints(rack_joint)[0] if rack_joint is not None else None
    device = robot.data.joint_pos.torch.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]
    gripper_bodies = tuple(sensor_names[1])

    def mug_pose_t(instance: int = 0) -> torch.Tensor:
        (pos_w, yaw_deg) = config.OBJECT_COUNTERTOP_POSES_W[instance]
        rot = countertop_staging_rot(yaw_deg)
        return torch.tensor(
            np.concatenate([np.array(pos_w), rot.as_quat()])[None], dtype=torch.float32, device=device
        )

    # settle once with jaws open: the teleport template for every trial reset
    for _ in range(60):
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    obj.write_root_pose_to_sim_index(root_pose=mug_pose_t())
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
    for _ in range(90):
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    template_open = robot.data.joint_pos.torch.clone()

    def step_sim(n: int = 1) -> None:
        for _ in range(n):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    def reset_trial() -> None:
        """Weld OFF, racks at the scenario's initial state, object staged (countertop, or
        welded at the carry pose for START_WELDED classes)."""
        dscene.set_weld_enabled(scene.stage, weld_path, False)
        dscene.set_rack_target_override(None)
        full = template_open.clone()
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        dw.write_joint_position_to_sim_index(position=dw.data.default_joint_pos.torch.clone())
        dw.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(dw.data.default_joint_pos.torch))
        if START_WELDED:
            q_arm = template_open[0, arm_ids].cpu().numpy().astype(float)
            pos, quat = dscene.grasp_pose_w(q_arm)
            pose = torch.tensor(np.concatenate([pos, quat])[None], dtype=torch.float32, device=device)
            obj.write_root_pose_to_sim_index(root_pose=pose)
        else:
            obj.write_root_pose_to_sim_index(root_pose=mug_pose_t())
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))
        if START_WELDED:
            dscene.set_weld_enabled(scene.stage, weld_path, True)
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        for _ in range(30):
            step_sim(1)

    def object_pose_base() -> np.ndarray:
        p = obj.data.root_pos_w.torch[0].cpu().numpy()
        q = obj.data.root_quat_w.torch[0].cpu().numpy()
        return T_base_w @ make_T(p, q)

    def arm_q_now() -> np.ndarray:
        return robot.data.joint_pos.torch[0, arm_ids].cpu().numpy().astype(float)

    def ik_track(T_tcp_from: np.ndarray, T_tcp_to: np.ndarray, n: int, q_seed: np.ndarray) -> np.ndarray | None:
        """Branch-continuous IK along a straight TCP segment (rotation held; used for the
        scripted descend/lift/disengage moves)."""
        qs, q_prev = [], np.asarray(q_seed, dtype=float)
        for s in np.linspace(0.0, 1.0, n):
            T = T_tcp_from.copy()
            T[:3, 3] = (1 - s) * T_tcp_from[:3, 3] + s * T_tcp_to[:3, 3]
            sols = []
            for sol in ik_wrist3_all(T @ T_inv(T_w3_tcp), q_seed=q_prev):
                sol = sol + np.round((q_prev - sol) / (2.0 * np.pi)) * 2.0 * np.pi
                sols.append(sol)
            sols.sort(key=lambda q: float(np.abs(q - q_prev).max()))
            if not sols or float(np.abs(sols[0] - q_prev).max()) > 0.8:
                return None
            qs.append(sols[0])
            q_prev = sols[0]
        return np.array(qs)

    trial_id = 0
    summary = []
    for slot_id in parse_ids(args_cli.slots):
        for seed in parse_ids(args_cli.seeds):
            for rep in range(args_cli.repeats):
                tag = f"trial_{slot_id:02d}_{seed:02d}_{rep}"
                json_path = os.path.join(args_cli.out, f"{tag}.json")
                if args_cli.skip_existing and os.path.isfile(json_path):
                    print(f"[INFO] {tag}: exists, skipping")
                    continue
                trial_id += 1
                record = {"trial": tag, "slot": slot_id, "seed": seed, "repeat": rep,
                          "object": config.ACTIVE_OBJECT, "placement_mode": config.active_object_spec().placement.mode,
                          "scenario": args_cli.scenario, "rack_action": ACTION,
                          "success": False, "failure_stage": None, "plan_time_s": None,
                          "path_len_rad": None, "exec_steps": 0, "goal_config_index": None,
                          "grasp_force_n": None, "exec_pad_peak_n": None, "retract": None,
                          "rack_final_err_m": None, "pick_weld_err_mm": None,
                          "media": {}}
                slot = slots[slot_id]
                goals = goal_sets.get(slot_id, np.zeros((0, 6)))

                reset_trial()
                video = None
                frame_i = 0
                if rig is not None:
                    video = VideoWriter(os.path.join(args_cli.media, f"{tag}.mp4"), fps=config.CAMERA_FPS)
                    record["media"]["video"] = os.path.relpath(video.path, PROJECT_ROOT)

                def capture():
                    nonlocal frame_i
                    if video is None:
                        return
                    frame_i += 1
                    if frame_i % 2 == 0:
                        rig.update(dt)
                        video.add(rig.grab()["front"])

                def run_path(path_q, aperture, stage_name, exclude=(), rack_targets=None):
                    """Execute a joint path under time parameterization with contact gating.
                    Returns None on success, else the failure detail string."""
                    for wp in planning.time_parameterize(np.asarray(path_q)):
                        if rack_targets is not None:
                            dscene.set_rack_target_override(rack_targets)
                        dscene.hold_targets(scene, arm_q=wp, aperture=aperture)
                        step_sim(1)
                        capture()
                        peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names,
                                                                     exclude=exclude)
                        if peak > config.RETRACT_GRAZE_MAX_N:
                            return f"{stage_name}: {body} ({peak:.1f} N)"
                    return None

                try:
                    if len(goals) == 0:
                        for _ in range(120):
                            dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
                            step_sim(1)
                            capture()
                        record["failure_stage"] = "no-goal-config"
                        raise StopIteration

                    if ACTION is None:  # internal state: racks pre-positioned, no rack phase
                        q_free = np.asarray(config.HOME_Q, dtype=float)
                    else:
                        # ================= PHASE R: rack reconfiguration ==========================
                        approach_aperture = (config.GRIPPER_APERTURE_OPEN_RAD if ACTION["mode"] == "pull"
                                             else config.RACK_PUSH_APERTURE_RAD)
                        approach = rack_ops.approach_dir(T_base_rack)
                        # a human reaches the handle horizontally from the front; try both rod-axis
                        # signs (wrist-flip) and keep the first with a valid approach AND slide path
                        plan_r = None
                        for flip in (False, True):
                            T_engage = rack_ops.engage_pose(T_base_rack, rack_params, ACTION, e0, flip=flip)
                            T_pre = T_engage.copy()
                            T_pre[:3, 3] = T_engage[:3, 3] - approach * config.RACK_APPROACH_HOVER_M
                            pre_sols = [q for q in ik_wrist3_all(T_pre @ T_inv(T_w3_tcp), q_seed=np.array(config.HOME_Q))
                                        if not rack_world.in_collision(q)]
                            if not pre_sols:
                                continue
                            arm_path = rack_ops.slide_arm_path(rack_world, T_base_rack, rack_params, ACTION,
                                                               e0, pre_sols[0], flip=flip)
                            if arm_path is None:
                                continue
                            plan_r = (flip, T_engage, T_pre, pre_sols, arm_path)
                            break
                        if plan_r is None:
                            record["failure_stage"] = "rack-slide-plan"
                            record["failure_detail"] = "no flip variant yields a valid engage + slide path"
                            raise StopIteration
                        flip_r, T_engage, T_pre, pre_sols, arm_path = plan_r

                        res = planning.plan_to_goals(rack_world, np.array(config.HOME_Q), np.array(pre_sols),
                                                     seed=seed * 7 + rep + 1)
                        if res.status != "solved":
                            record["failure_stage"] = "rack-approach-plan"
                            raise StopIteration
                        fail = run_path(res.path_q, approach_aperture, "rack-approach")
                        if fail:
                            record["failure_stage"] = "rack-approach-plan"
                            record["failure_detail"] = fail
                            raise StopIteration
                        q_pre = np.asarray(res.path_q[-1], dtype=float)

                        # scripted horizontal reach-in onto the handle (gripper-rack contact is the
                        # POINT here: gripper bodies leave the unexpected-contact gate, arm stays)
                        seg = ik_track(T_pre, T_engage, 15, q_pre)
                        if seg is None:
                            record["failure_stage"] = "rack-approach-plan"
                            record["failure_detail"] = "IK discontinuity on the engage reach-in"
                            raise StopIteration
                        fail = run_path(seg, approach_aperture, "rack-engage", exclude=gripper_bodies)
                        if fail:
                            record["failure_stage"] = "rack-slide-fault"
                            record["failure_detail"] = fail
                            raise StopIteration
                        if ACTION["mode"] == "pull":  # wrap the rod
                            dscene.ramp_gripper(scene, sim, config.RACK_HANDLE_APERTURE_RAD, 45,
                                                arm_q=seg[-1], per_step=lambda i: capture())
                        # re-seed the slide path from the actually-reached engage config
                        arm_path2 = rack_ops.slide_arm_path(rack_world, T_base_rack, rack_params, ACTION,
                                                            e0, seg[-1], flip=flip_r)
                        arm_path = arm_path if arm_path2 is None else arm_path2
                        slide_aperture = (config.RACK_HANDLE_APERTURE_RAD if ACTION["mode"] == "pull"
                                          else config.RACK_PUSH_APERTURE_RAD)
                        n_steps = config.RACK_SLIDE_STEPS
                        for i in range(n_steps):
                            s = (i + 1) / n_steps
                            e_s = e0 + (e1 - e0) * s
                            idx = s * (len(arm_path) - 1)
                            lo = int(np.floor(idx))
                            hi = min(lo + 1, len(arm_path) - 1)
                            wp = arm_path[lo] + (idx - lo) * (arm_path[hi] - arm_path[lo])
                            dscene.set_rack_target_override({rack_joint: e_s})
                            dscene.hold_targets(scene, arm_q=wp, aperture=slide_aperture)
                            step_sim(1)
                            capture()
                            peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names,
                                                                         exclude=gripper_bodies)
                            if peak > config.RETRACT_GRAZE_MAX_N:
                                record["failure_stage"] = "rack-slide-fault"
                                record["failure_detail"] = f"arm contact during slide: {body} ({peak:.1f} N)"
                                raise StopIteration
                        for _ in range(60):
                            dscene.hold_targets(scene, arm_q=arm_path[-1], aperture=slide_aperture)
                            step_sim(1)
                            capture()
                        rack_err = abs(float(dw.data.joint_pos.torch[0, rack_jids[0]]) - e1)
                        record["rack_final_err_m"] = round(rack_err, 4)
                        if rack_err > config.RACK_SLIDE_TOL_M:
                            record["failure_stage"] = "rack-slide-fault"
                            record["failure_detail"] = f"rack settled {rack_err * 1e3:.1f} mm off target"
                            raise StopIteration

                        # disengage: open (pull mode) and back straight off the handle
                        if ACTION["mode"] == "pull":
                            dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_OPEN_RAD, 45,
                                                arm_q=arm_path[-1], per_step=lambda i: capture())
                        T_now = fk_wrist3(arm_path[-1]) @ T_w3_tcp
                        T_up = T_now.copy()
                        T_up[:3, 3] = T_now[:3, 3] - approach * config.RACK_APPROACH_HOVER_M
                        seg = ik_track(T_now, T_up, 12, arm_path[-1])
                        if seg is not None:
                            fail = run_path(seg, config.GRIPPER_APERTURE_OPEN_RAD, "rack-disengage",
                                            exclude=gripper_bodies)
                            if fail:
                                record["failure_stage"] = "rack-slide-fault"
                                record["failure_detail"] = fail
                                raise StopIteration
                        q_free = seg[-1] if seg is not None else arm_path[-1]

                    if START_WELDED:
                        # already welded at the carry pose: visible close onto the object,
                        # then carry (the weld bears the load — v0's grasp-acquisition-out-
                        # of-scope framing for flat discs that cannot be pinched off a table)
                        dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_GRASP_RAD,
                                            config.GRIPPER_CLOSE_RAMP_STEPS, arm_q=q_free,
                                            per_step=lambda i: capture())
                        q_carry = np.asarray(q_free, dtype=float)
                        record["q_carry"] = [round(float(v), 6) for v in q_carry]
                    else:
                        # ================= PHASE P: pick the mug off the countertop ==============
                        T_base_obj0 = object_pose_base()  # measured (absorbs any settle drift)
                        T_grasp_tcp = T_base_obj0 @ T_inv(T_tcp_obj)
                        T_pre_pick = T_grasp_tcp.copy()
                        T_pre_pick[2, 3] += config.PICK_HOVER_M
                        pre_sols = [q for q in ik_wrist3_all(T_pre_pick @ T_inv(T_w3_tcp), q_seed=q_free)
                                    if not pick_world.in_collision(q)]
                        if not pre_sols:
                            record["failure_stage"] = "pick-plan"
                            record["failure_detail"] = "no collision-free IK at the pre-grasp hover"
                            raise StopIteration
                        res = planning.plan_to_goals(pick_world, q_free, np.array(pre_sols),
                                                     seed=seed * 7 + rep + 2)
                        if res.status != "solved":
                            record["failure_stage"] = "pick-plan"
                            raise StopIteration
                        fail = run_path(res.path_q, config.GRIPPER_APERTURE_OPEN_RAD, "pick-approach")
                        if fail:
                            record["failure_stage"] = "pick-plan"
                            record["failure_detail"] = fail
                            raise StopIteration
                        q_hover = np.asarray(res.path_q[-1], dtype=float)

                        seg = ik_track(T_pre_pick, T_grasp_tcp, 20, q_hover)
                        if seg is None:
                            record["failure_stage"] = "pick-plan"
                            record["failure_detail"] = "IK discontinuity on the pick descent"
                            raise StopIteration
                        fail = run_path(seg, config.GRIPPER_APERTURE_OPEN_RAD, "pick-descend",
                                        exclude=gripper_bodies)
                        if fail:
                            record["failure_stage"] = "pick-grasp-fault"
                            record["failure_detail"] = fail
                            raise StopIteration
                        for _ in range(30):
                            dscene.hold_targets(scene, arm_q=seg[-1], aperture=config.GRIPPER_APERTURE_OPEN_RAD)
                            step_sim(1)
                            capture()
                        f0 = dscene.grip_forces(scene)
                        if max(f0["partners"].values()) > 0.05:
                            record["failure_stage"] = "pick-grasp-fault"
                            record["failure_detail"] = f"contact before the close ({max(f0['partners'].values()):.2f} N)"
                            raise StopIteration
                        dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_GRASP_RAD,
                                            config.GRIPPER_CLOSE_RAMP_STEPS, arm_q=seg[-1],
                                            per_step=lambda i: capture())
                        for _ in range(60):
                            dscene.hold_targets(scene, arm_q=seg[-1])
                            step_sim(1)
                            capture()
                        gf0 = dscene.grip_forces(scene)
                        record["grasp_force_n"] = round(float(np.mean(gf0["pads_n"])), 2)
                        # per-pad band + non-pad silence; the EXTERNAL residual is legitimately
                        # nonzero here (the countertop still supports the mug) — it is gated
                        # after the lift instead, when the mug must be airborne
                        grip_detail = None
                        for pad, mag in zip(config.GRIP_PAD_BODIES, gf0["pads_n"]):
                            if not config.GRIP_FORCE_MIN_N <= mag <= config.GRIP_FORCE_MAX_N:
                                grip_detail = f"{pad} force {mag:.2f} N outside the calibrated band"
                        for nm, mag in gf0["partners"].items():
                            if nm not in config.GRIP_PAD_BODIES and mag > config.CONTACT_FORCE_THRESH_N:
                                grip_detail = f"non-pad partner {nm} touches the mug ({mag:.2f} N)"
                        if grip_detail:
                            record["failure_stage"] = "pick-grasp-fault"
                            record["failure_detail"] = grip_detail
                            raise StopIteration

                        # hidden weld takes the load (anchors == the grasp transform -> zero error)
                        dscene.set_weld_enabled(scene.stage, weld_path, True)
                        for _ in range(30):
                            dscene.hold_targets(scene, arm_q=seg[-1])
                            step_sim(1)
                            capture()
                        obj_pos = obj.data.root_pos_w.torch[0].cpu().numpy()
                        weld_err = float(np.linalg.norm(obj_pos - dscene.grasp_pose_w(seg[-1])[0])) * 1e3
                        record["pick_weld_err_mm"] = round(weld_err, 2)
                        if weld_err > 5.0:
                            record["failure_stage"] = "pick-weld-fault"
                            record["failure_detail"] = f"weld error {weld_err:.1f} mm after enable"
                            raise StopIteration
                        pick_world.remove_object("countertop_mug")  # it is in-hand now

                        T_lift = T_grasp_tcp.copy()
                        T_lift[2, 3] += config.PICK_LIFT_M
                        seg = ik_track(T_grasp_tcp, T_lift, 15, seg[-1])
                        if seg is None:
                            record["failure_stage"] = "pick-plan"
                            record["failure_detail"] = "IK discontinuity on the lift"
                            raise StopIteration
                        fail = run_path(seg, None, "pick-lift", exclude=gripper_bodies)
                        if fail:
                            record["failure_stage"] = "pick-grasp-fault"
                            record["failure_detail"] = fail
                            raise StopIteration
                        gf_lift = dscene.grip_forces(scene)
                        if gf_lift["external_n"] > config.CONTACT_FORCE_THRESH_N:
                            record["failure_stage"] = "pick-grasp-fault"
                            record["failure_detail"] = (
                                f"mug still externally loaded after the lift ({gf_lift['external_n']:.2f} N)"
                            )
                            raise StopIteration
                        q_carry = seg[-1]
                        record["q_carry"] = [round(float(v), 6) for v in q_carry]

                    # ================= PHASE F: plan to the slot and place ====================
                    rng = np.random.default_rng(seed * 1000 + rep)
                    sub = goals[rng.choice(len(goals), min(config.GOALS_PER_PLAN, len(goals)), replace=False)]
                    if world.in_collision(q_carry):
                        record["failure_stage"] = "pick-plan"
                        record["failure_detail"] = "carry start config in collision"
                        raise StopIteration
                    res = planning.plan_to_goals(world, q_carry, sub, seed=seed * 7 + rep + 1)
                    record["plan_time_s"] = round(res.plan_time_s, 3)
                    if res.status != "solved":
                        record["failure_stage"] = "planner-timeout"
                        raise StopIteration
                    record["path_len_rad"] = round(res.path_len_rad, 3)
                    record["goal_config_index"] = int(res.goal_index)

                    dense = planning.time_parameterize(res.path_q)
                    exec_fail, exec_pad_peak = None, 0.0
                    for wp in dense:
                        dscene.hold_targets(scene, arm_q=wp)  # squeeze stays commanded
                        step_sim(1)
                        capture()
                        peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names)
                        gf = dscene.grip_forces(scene)
                        exec_pad_peak = max(exec_pad_peak, max(gf["pads_n"]))
                        if peak > config.CONTACT_FORCE_THRESH_N:
                            exec_fail = f"{body} ({peak:.1f} N)"
                        elif gf["external_n"] > config.CONTACT_FORCE_THRESH_N:
                            exec_fail = f"carried_object external ({gf['external_n']:.1f} N)"
                        elif max(gf["pads_n"]) > config.GRIP_FORCE_EXEC_MAX_N:
                            exec_fail = f"pad squeeze spike ({max(gf['pads_n']):.1f} N)"
                        if exec_fail is not None:
                            break
                        record["exec_steps"] += 1
                    record["exec_pad_peak_n"] = round(exec_pad_peak, 2)
                    if exec_fail is not None:
                        record["failure_stage"] = "execution-collision"
                        record["failure_detail"] = exec_fail
                        raise StopIteration

                    # hold, verify quiet, then the visible release: open the jaws BEFORE the
                    # weld lets go (pads must unload cleanly), then drop the weld
                    goal_q = np.asarray(dense[-1], dtype=float)
                    for _ in range(30):
                        step_sim(1)
                        capture()
                    dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_OPEN_RAD,
                                        config.GRIPPER_OPEN_RAMP_STEPS, arm_q=goal_q,
                                        per_step=lambda i: capture())
                    for _ in range(15):
                        step_sim(1)
                        capture()
                    gf_open = dscene.grip_forces(scene)
                    if max(gf_open["partners"].values()) > 0.05:
                        record["failure_stage"] = "release-fault"
                        record["failure_detail"] = (
                            f"pads still load the mug after open ({max(gf_open['partners'].values()):.2f} N)"
                        )
                        raise StopIteration
                    dscene.set_weld_enabled(scene.stage, weld_path, False)
                    for _ in range(150):
                        step_sim(1)
                        capture()

                    # -- retract the tool along -z, validated in the post-release world -------
                    T_b_w3 = fk_wrist3(goal_q)
                    tool_dir = T_b_w3[:3, :3] @ np.array([0.0, 1.0, 0.0])  # tool z == +y_wrist3
                    T_target = T_b_w3.copy()
                    T_target[:3, 3] -= config.RETRACT_DIST_M * tool_dir
                    retract_world.add_object("placed_mug", mug_pieces, object_pose_base())
                    q_retract, retract_mode = None, ""
                    sols = []
                    for s in ik_wrist3_all(T_target, q_seed=goal_q):
                        s = s + np.round((goal_q - s) / (2.0 * np.pi)) * 2.0 * np.pi
                        if float(np.linalg.norm(s - goal_q)) <= 1.0:
                            sols.append(s)
                    sols.sort(key=lambda s: float(np.linalg.norm(s - goal_q)))

                    def retract_seg(sol):
                        n = max(20, int(np.ceil(float(np.linalg.norm(sol - goal_q))
                                                / config.PLAN_VALIDITY_RESOLUTION_RAD)))
                        return np.linspace(goal_q, sol, n)

                    try:
                        for sol in sols:
                            seg_r = retract_seg(sol)
                            verdicts = []
                            for qi in seg_r:
                                hit, pairs = retract_world.in_collision(qi, return_pairs=True)
                                mug_only = bool(hit) and all(p[1] == "placed_mug" for p in pairs)
                                verdicts.append((bool(hit), mug_only))
                            first_free = next((i for i, (h, _) in enumerate(verdicts) if not h), None)
                            if (first_free is not None and first_free <= int(0.7 * len(seg_r))
                                    and all(m for h, m in verdicts[:first_free] if h)
                                    and not any(h for h, _ in verdicts[first_free:])):
                                q_retract, retract_mode = sol, "ok"
                                break
                    finally:
                        retract_world.remove_object("placed_mug")
                    if q_retract is None:
                        for sol in sols:
                            if not any(retract_world.in_collision(qi) for qi in retract_seg(sol)):
                                q_retract, retract_mode = sol, "ok-statics-only"
                                break
                    if q_retract is None:
                        record["retract"] = "skipped-no-valid-path"
                    else:
                        retract_fail, graze_peak, graze_body = None, 0.0, ""
                        for wp in planning.time_parameterize(np.array([goal_q, q_retract])):
                            dscene.hold_targets(scene, arm_q=wp,
                                                aperture=config.GRIPPER_APERTURE_OPEN_RAD)
                            step_sim(1)
                            capture()
                            peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names)
                            if peak >= config.RETRACT_GRAZE_MAX_N:
                                retract_fail = f"{body} ({peak:.1f} N)"
                                break
                            if peak > max(graze_peak, config.CONTACT_FORCE_THRESH_N):
                                graze_peak, graze_body = peak, body
                        if retract_fail is not None:
                            record["failure_stage"] = "retract-collision"
                            record["failure_detail"] = retract_fail
                            raise StopIteration
                        record["retract"] = retract_mode + (
                            "" if graze_peak == 0.0 else f", grazed {graze_body} ({graze_peak:.2f} N)"
                        )

                    for _ in range(config.SETTLE_STEPS):
                        step_sim(1)
                        capture()

                    ev = evaluate_placement(slot, object_pose_base())
                    peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names)
                    ok = ev.pop("ok") and peak <= config.CONTACT_FORCE_THRESH_N
                    record["final_pose_err"] = ev
                    record["success"] = bool(ok)
                    if not ok:
                        record["failure_stage"] = "unstable-after-release"
                except StopIteration:
                    pass
                finally:
                    # trial-scoped world mutations must not leak into the next trial
                    if not START_WELDED:
                        pick_world.remove_object("countertop_mug")
                        pick_world.add_object("countertop_mug", mug_pieces, T_base_mug0)

                if video is not None:
                    video.close()
                if rig is not None:
                    target = (T_w_base @ np.append(slot.T_base_slot[:3, 3], 1.0))[:3]
                    rig.set_view("iso", target + np.array([-0.35, 0.35, 0.3]), target, sim.device)
                    for _ in range(3):
                        step_sim(1)
                        rig.update(dt)
                    from PIL import Image  # noqa: PLC0415

                    still = os.path.join(args_cli.media, f"{tag}_final.png")
                    Image.fromarray(rig.grab()["iso"]).save(still)
                    record["media"]["final"] = os.path.relpath(still, PROJECT_ROOT)
                    rig.set_view("iso", *config.CAMERAS["iso"], sim.device)

                with open(json_path, "w") as f:
                    json.dump(record, f, indent=2)
                summary.append(record)
                print(f"[INFO] {tag}: success={record['success']} stage={record['failure_stage']} "
                      f"rack_err={record['rack_final_err_m']} weld={record['pick_weld_err_mm']}mm "
                      f"plan={record['plan_time_s']}s err={record.get('final_pose_err')}")

    n_ok = sum(r["success"] for r in summary)
    print(f"[INFO] {n_ok}/{len(summary)} trials succeeded")
    print(f"[RESULT] {'PASS' if n_ok >= 1 else 'FAIL'}")
    if n_ok < 1:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
