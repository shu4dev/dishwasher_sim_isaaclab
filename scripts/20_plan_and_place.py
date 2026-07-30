# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase F: plan -> execute -> release -> evaluate, one JSON + MP4 per trial.

Per trial: reset the welded scene, plan with RRT-Connect (goals = Phase E goal set subset,
5 s budget), execute with position drives under constant joint-speed time parameterization
while monitoring contacts (any non-excluded contact before release fails the trial), release
the weld at the goal, settle, and evaluate the docs/success_criteria.md conditions against the
slot frame. Every trial writes ``results/trial_<id>.json``, a full MP4, and a final close-up
still — failures capture the failing moment by construction (the writer runs from step 0).

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
parser.add_argument("--out", type=str, default=os.path.join(PROJECT_ROOT, "results"))
parser.add_argument("--media", type=str, default=os.path.join(PROJECT_ROOT, "media", "F"))
parser.add_argument("--skip_existing", action="store_true", help="Skip trials whose JSON exists (resume).")
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
from dishsim import planning  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.placement import SlotFrame  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T, quat_to_mat  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3  # noqa: E402


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

    with open(os.path.join(config.CACHE_DIR, "slots", "slots.json")) as f:
        slots = {s["slot_id"]: SlotFrame.from_json(s) for s in json.load(f)["slots"]}
    with open(os.path.join(config.CACHE_DIR, "slots", "goal_sets.json")) as f:
        goal_sets = {g["slot_id"]: np.array(g["configs"]) for g in json.load(f)["goal_sets"]}

    world = CollisionWorld(self_check=True)
    T_w3_obj = np.array(world.manifest["object"]["T_wrist3_obj"])
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)

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
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    robot = scene["robot"]
    obj = scene["carried_object"]
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = robot.data.joint_pos.torch.device
    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]

    for _ in range(150):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    joint_template = robot.data.joint_pos.torch.clone()

    def step_sim(n: int = 1) -> None:
        for _ in range(n):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    def robot_contact_peak() -> tuple[float, str]:
        peak, body = 0.0, ""
        for si, sensor in enumerate(sensors):
            mags = sensor.data.net_forces_w.torch[0].norm(dim=-1)
            for bi in range(mags.shape[0]):
                name = sensor_names[si][bi] if bi < len(sensor_names[si]) else f"s{si}b{bi}"
                if name in config.PARITY_BODY_EXCLUDE:
                    continue
                if float(mags[bi]) > peak:
                    peak, body = float(mags[bi]), name
        return peak, body

    def object_contact_peak() -> float:
        return float(scene["object_contact"].data.net_forces_w.torch[0].norm())

    def set_arm_targets(q: np.ndarray) -> None:
        full = joint_template.clone()
        full[:, arm_ids] = torch.tensor(q, dtype=full.dtype, device=device)
        robot.set_joint_position_target_index(target=full)

    def reset_trial() -> float:
        """Re-weld + teleport everything home; returns the post-reset weld error [mm]."""
        dscene.set_weld_enabled(scene.stage, weld_path, True)
        full = joint_template.clone()
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        robot.set_joint_position_target_index(target=full)
        pos, quat = dscene.grasp_pose_w()
        pose_t = torch.tensor(np.concatenate([pos, quat])[None], dtype=full.dtype, device=device)
        obj.write_root_pose_to_sim_index(root_pose=pose_t)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=full.dtype, device=device))
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
                          "success": False, "failure_stage": None, "plan_time_s": None,
                          "path_len_rad": None, "exec_steps": 0, "goal_config_index": None,
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
                        record["failure_stage"] = "no-goal-config"
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
                    exec_fail = None
                    for wp in dense:
                        set_arm_targets(wp)
                        step_sim(1)
                        capture()
                        peak, body = robot_contact_peak()
                        obj_peak = object_contact_peak()
                        if peak > config.CONTACT_FORCE_THRESH_N or obj_peak > config.CONTACT_FORCE_THRESH_N:
                            exec_fail = f"{body or 'carried_object'} ({max(peak, obj_peak):.1f} N)"
                            break
                        record["exec_steps"] += 1
                    if exec_fail is not None:
                        record["failure_stage"] = "execution-collision"
                        record["failure_detail"] = exec_fail
                        raise StopIteration

                    # hold, verify quiet, release
                    for _ in range(30):
                        step_sim(1)
                        capture()
                    dscene.set_weld_enabled(scene.stage, weld_path, False)
                    for _ in range(config.SETTLE_STEPS + 150):
                        step_sim(1)
                        capture()

                    lateral, tilt, height = placement_errors(slot)
                    peak, body = robot_contact_peak()
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
