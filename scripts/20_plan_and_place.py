# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase F: close -> plan -> execute -> open+release -> retract -> evaluate, per-trial JSON+MP4.

Per trial: reset the welded scene with the jaws OPEN, visibly close the gripper onto the mug
to the calibrated pinch band (abort as ``grasp-fault`` if out of band), plan with RRT-Connect
(goals = Phase E goal set subset, 5 s budget), execute with position drives under constant
joint-speed time parameterization while monitoring contacts (unexpected robot contact, mug
external residual, and the dynamic pad-force cap), visibly open the jaws at the goal (pad
forces must vanish), release the weld, retract the tool along -z validated against the
post-release collision world (placed mug as obstacle), settle, and evaluate the
docs/success_criteria.md conditions against the slot frame. Every trial writes
``results/trial_<id>.json``, a full MP4, and a final close-up still — failures capture the
failing moment by construction (the writer runs from step 0).

Run with (shake-out):
    scripts/run_kit.sh scripts/20_plan_and_place.py --headless --enable_cameras \
        --slots 2 --seeds 0 --repeats 2
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="OMPL plan-and-place trials.")
parser.add_argument("--slots", type=str, default="1,2,3", help="Comma-separated slot ids.")
parser.add_argument("--seeds", type=str, default="0", help="Comma list or a-b range of RNG seeds.")
parser.add_argument("--repeats", type=int, default=1, help="Trials per (slot, seed) pair.")
parser.add_argument("--out", type=str, default=None,
                    help="Trial-JSON dir (default: results/ or results/<scenario>).")
parser.add_argument("--media", type=str, default=None,
                    help="Media dir (default: media/F or media/F/<scenario>).")
parser.add_argument("--skip_existing", action="store_true", help="Skip trials whose JSON exists (resume).")
parser.add_argument("--scenario", type=str, default="lower_out",
                    help="Rack-state scenario (see config.SCENARIOS).")
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
config.apply_scenario(args_cli.scenario)
if args_cli.out is None:
    args_cli.out = (os.path.join(PROJECT_ROOT, "results") if config.SCENARIO_NAME == "lower_out"
                    else os.path.join(PROJECT_ROOT, "results", config.SCENARIO_NAME))
if args_cli.media is None:
    args_cli.media = config.scenario_media_dir("F")

from dishsim import planning  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.placement import SlotFrame  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T, quat_to_mat  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3, ik_wrist3_all  # noqa: E402


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


def main() -> None:
    os.makedirs(args_cli.out, exist_ok=True)
    os.makedirs(args_cli.media, exist_ok=True)

    cache_dir = config.scenario_cache_dir()
    with open(os.path.join(cache_dir, "slots", "slots.json")) as f:
        slots = {s["slot_id"]: SlotFrame.from_json(s) for s in json.load(f)["slots"]}
    with open(os.path.join(cache_dir, "slots", "goal_sets.json")) as f:
        goal_sets = {g["slot_id"]: np.array(g["configs"]) for g in json.load(f)["goal_sets"]}

    world = CollisionWorld(cache_dir=cache_dir, self_check=True)
    T_w3_obj = np.array(world.manifest["object"]["T_wrist3_obj"])
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)
    # post-release world for retract validation: same cache, carried object detached (it
    # re-enters per trial as the placed obstacle)
    retract_world = CollisionWorld(cache_dir=cache_dir, self_check=True)
    mug_pieces = retract_world.carried_object_pieces()
    retract_world.detach_carried_object()

    # ---- scene ------------------------------------------------------------------------------
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg())
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=True, with_robot_contacts=True))
    weld_path = dscene.author_weld(scene.stage)
    rig = CameraRig(config.CAMERAS, hw=config.CAMERA_HW) if args_cli.enable_cameras else None
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()
    robot = scene["robot"]
    obj = scene["carried_object"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = robot.data.joint_pos.torch.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]

    # settle with the jaws OPEN: this deformation-free, mimic-consistent state is the teleport
    # template for every trial reset (the visible close happens per trial, by target ramp)
    for _ in range(150):
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

    def reset_trial() -> float:
        """Re-weld + teleport everything home (jaws open); returns the weld error [mm]."""
        dscene.set_weld_enabled(scene.stage, weld_path, True)
        full = template_open.clone()
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        pos, quat = dscene.grasp_pose_w()
        pose_t = torch.tensor(np.concatenate([pos, quat])[None], dtype=full.dtype, device=device)
        obj.write_root_pose_to_sim_index(root_pose=pose_t)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=full.dtype, device=device))
        dscene.hold_targets(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
        step_sim(30)
        obj_pos = obj.data.root_pos_w.torch[0].cpu().numpy()
        return float(np.linalg.norm(obj_pos - dscene.grasp_pose_w()[0])) * 1e3

    def object_pose_base() -> np.ndarray:
        p = obj.data.root_pos_w.torch[0].cpu().numpy()
        q = obj.data.root_quat_w.torch[0].cpu().numpy()
        return T_base_w @ make_T(p, q)

    def placement_errors(slot: SlotFrame) -> tuple[float, float, float]:
        """(lateral [m], tilt [deg], bottom height above floor [m]) of the object vs the slot."""
        T_base_obj = object_pose_base()
        axis_dir = T_base_obj[:3, :3] @ np.array([0.0, 1.0, 0.0])
        p_bottom = (T_base_obj @ np.array([config.OBJECT_BODY_CENTER_XZ[0], -config.OBJECT_BBOX_HALF[1],
                                           config.OBJECT_BODY_CENTER_XZ[1], 1.0]))[:3]
        d = T_inv(slot.T_base_slot) @ np.append(p_bottom, 1.0)
        lateral = float(np.hypot(d[0], d[1]))
        tilt = float(np.degrees(np.arccos(np.clip(axis_dir @ slot.T_base_slot[:3, 2], -1.0, 1.0))))
        return lateral, tilt, float(d[2])

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
                          "scenario": config.SCENARIO_NAME,
                          "success": False, "failure_stage": None, "plan_time_s": None,
                          "path_len_rad": None, "exec_steps": 0, "goal_config_index": None,
                          "grasp_force_n": None, "exec_pad_peak_n": None, "retract": None,
                          "media": {}}
                slot = slots[slot_id]
                goals = goal_sets.get(slot_id, np.zeros((0, 6)))

                weld_err = reset_trial()
                if weld_err > 5.0:
                    print(f"[WARN] {tag}: weld re-attach error {weld_err:.1f} mm — restart recommended")
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

                try:
                    if len(goals) == 0:
                        # still show the scene the trial cannot solve: without this the video
                        # closes with zero frames (broken MP4) and the evidence shows nothing
                        for _ in range(120):
                            dscene.hold_targets(scene)
                            step_sim(1)
                            capture()
                        record["failure_stage"] = "no-goal-config"
                        raise StopIteration

                    # -- visible close onto the mug, then verify the calibrated pinch band ----
                    dscene.ramp_gripper(scene, sim, config.GRIPPER_APERTURE_GRASP_RAD,
                                        config.GRIPPER_CLOSE_RAMP_STEPS,
                                        per_step=lambda i: capture())
                    for _ in range(60):
                        dscene.hold_targets(scene)
                        step_sim(1)
                        capture()
                    gf0 = dscene.grip_forces(scene)
                    record["grasp_force_n"] = round(float(np.mean(gf0["pads_n"])), 2)
                    grip_ok, grip_detail = dscene.grip_gate(scene)
                    if not grip_ok:
                        record["failure_stage"] = "grasp-fault"
                        record["failure_detail"] = grip_detail
                        raise StopIteration

                    rng = np.random.default_rng(seed * 1000 + rep)
                    sub = goals[rng.choice(len(goals), min(config.GOALS_PER_PLAN, len(goals)), replace=False)]
                    res = planning.plan_to_goals(world, np.array(config.HOME_Q), sub, seed=seed * 7 + rep + 1)
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
                    # a retract is a short axial slide — a distant IK branch means a wrist-flip
                    # sweep over the placed mug (observed ejecting it); refuse those outright,
                    # and sample validation at the planner's own validity resolution. IK returns
                    # canonical-range angles while goal configs live in wrapped ranges, so first
                    # shift each solution by 2pi-multiples toward the goal branch.
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
                            seg = retract_seg(sol)
                            # slide-out criterion: at the goal the open jaws still flank the
                            # placed mug, and the 5 mm hull inflation swallows the real
                            # ~2.6 mm/side clearance — so initial "collision" with the mug
                            # obstacle is expected. Valid = early hits involve ONLY the placed
                            # mug, the cluster comes free within the first half of the segment,
                            # and never re-enters (the 0.1 N contact gate during execution
                            # guards the fine clearance of the actual slide).
                            verdicts = []
                            for qi in seg:
                                hit, pairs = retract_world.in_collision(qi, return_pairs=True)
                                mug_only = bool(hit) and all(p[1] == "placed_mug" for p in pairs)
                                verdicts.append((bool(hit), mug_only))
                            first_free = next((i for i, (h, _) in enumerate(verdicts) if not h), None)
                            if (first_free is not None and first_free <= int(0.7 * len(seg))
                                    and all(m for h, m in verdicts[:first_free] if h)
                                    and not any(h for h, _ in verdicts[first_free:])):
                                q_retract, retract_mode = sol, "ok"
                                break
                    finally:
                        retract_world.remove_object("placed_mug")
                    if q_retract is None:
                        # no branch passed the mug-aware slide-out — but parking with the jaws
                        # around the settled mug guarantees persistent interference (observed:
                        # the mug gets levered out during the settle window). A statics-only
                        # validated axial slide, guarded by the graze gate below, is strictly
                        # safer for the placement than staying put.
                        for sol in sols:
                            if not any(retract_world.in_collision(qi) for qi in retract_seg(sol)):
                                q_retract, retract_mode = sol, "ok-statics-only"
                                break
                    if q_retract is None:
                        record["retract"] = "skipped-no-valid-path"
                    else:
                        # sub-RETRACT_GRAZE_MAX_N brushes are recorded but not fatal — the
                        # placed mug shifts a few mm on landing and the jaws have ~2.6 mm/side;
                        # the final evaluation window is the arbiter of placement integrity
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

                    lateral, tilt, height = placement_errors(slot)
                    peak, body = dscene.unexpected_robot_contact(scene, sensors, sensor_names)
                    ok = (lateral <= config.SLOT_TOL_LATERAL_M and tilt <= config.SLOT_TOL_TILT_DEG
                          and height <= 0.02 and peak <= config.CONTACT_FORCE_THRESH_N)
                    record["final_pose_err"] = {"lateral_m": round(lateral, 4), "tilt_deg": round(tilt, 2),
                                                "bottom_height_m": round(height, 4)}
                    record["success"] = bool(ok)
                    if not ok:
                        record["failure_stage"] = "unstable-after-release"
                except StopIteration:
                    pass

                if video is not None:
                    video.close()
                if rig is not None:
                    # final close-up still aimed at the slot
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
                      f"plan={record['plan_time_s']}s len={record['path_len_rad']} "
                      f"err={record.get('final_pose_err')}")

    n_ok = sum(r["success"] for r in summary)
    print(f"[INFO] {n_ok}/{len(summary)} trials succeeded")
    print(f"[RESULT] {'PASS' if n_ok >= 1 else 'FAIL'}")
    if n_ok < 1:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
