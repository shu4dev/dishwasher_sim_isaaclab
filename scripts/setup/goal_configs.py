# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive rack slots, generate IK goal sets, render the evidence.

Computation (slots + goal sets) is Kit-free (``dishsim.placement`` + the FCL world) and runs
first; the Kit session then poses the robot at accepted and rejected configurations for
labeled contact sheets. Outputs:

- ``assets/cache/slots/slots.json``, ``goal_sets.json`` (with per-slot rejection funnels)
- ``media/goals/slot_detection.png`` (top-down rack wires + slot cells)
- ``media/goals/accepted_slot<k>_sheet.png`` (robot at accepted goal configs)
- ``media/goals/rejected_sheet.png`` (labeled rejected configs: limit / collision + pair)

Run with:
    scripts/run_kit.sh scripts/setup/goal_configs.py --headless --enable_cameras
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Slot frames + IK goal sets + contact sheets.")
parser.add_argument("--seed", type=int, default=11)
parser.add_argument("--out_dir", type=str, default=None,
                    help="Media dir (default: media/goals or media/goals/<scenario>).")
parser.add_argument("--sheets_per_slot", type=int, default=6)
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement (see config.BASE_PLACEMENTS); default: the machine's.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
parser.add_argument("--object", type=str, default="mug", help="Carried object class (see config.OBJECTS).")
parser.add_argument("--max_store", type=int, default=None,
                    help="Cap the goal configs STORED per slot (the funnel still measures the "
                         "full acceptance for the feasibility record). Reachable-rich states "
                         "accept thousands of wrap-expanded configs per slot while the runner "
                         "consumes at most GOALS_PER_PLAN=64; 256 keeps a 4x margin and cuts "
                         "the artifact ~10x. Default: store everything (legacy bakes).")
parser.add_argument("--scenario", type=str, default=None,
                    help="Rack-state scenario (default: the placement state scripts/experiment/run_trials.py reads).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

_enable_cameras = args_cli.enable_cameras  # 2.1's AppLauncher pops this off the namespace
app_launcher = AppLauncher(args_cli)
args_cli.enable_cameras = _enable_cameras
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402
from dishsim.quats import xyzw_to_wxyz  # noqa: E402

# scenario BEFORE scene/robots imports — they bind rack targets + the derived USD at import
if args_cli.machine:
    config.apply_machine(args_cli.machine)  # first: it resets scenario + base placement
config.set_active_object(args_cli.object)
config.apply_scenario(args_cli.scenario or config.PLACEMENT_STATE)
if args_cli.placement:
    config.apply_base_placement(args_cli.placement)  # after machine/scenario — they reset it
if args_cli.out_dir is None:
    args_cli.out_dir = config.scenario_media_dir("goals")

from dishsim import placement  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.media import CameraRig, contact_sheet  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402
from dishsim.ur5e_kin import ik_wrist3_all  # noqa: E402

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def render_slot_detection(slots, out_png: str) -> None:
    """Top-down plot: rack wire vertices (base frame) + slot cells + reach circle."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import trimesh

    from dishsim.geometry import load_manifest

    manifest = load_manifest(config.scenario_cache_dir())
    entry = manifest["statics"]["E_shelf_1_04"]
    mesh = trimesh.load(os.path.join(config.scenario_cache_dir(), entry["mesh"]), force="mesh")
    T = np.array(entry["T_base_body"])
    verts = (T @ np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))]).T).T[:, :3]

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(verts[:, 0], verts[:, 1], s=0.2, c="0.55", linewidths=0, label="rack wires")
    for s in slots:
        cx, cy = s.T_base_slot[0, 3], s.T_base_slot[1, 3]
        half = s.width_m / 2.0
        ax.add_patch(plt.Rectangle((cx - half, cy - half), 2 * half, 2 * half, fill=False, ec="tab:blue"))
        ax.annotate(str(s.slot_id), (cx, cy), ha="center", va="center", color="tab:red", fontsize=12)
    ax.scatter([0], [0], marker="*", s=140, c="k", label="robot base")
    theta = np.linspace(0, 2 * np.pi, 128)
    ax.plot(config.UR5E_REACH_M * np.cos(theta), config.UR5E_REACH_M * np.sin(theta), "k--", lw=0.8, label="0.85 m reach")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (robot-base frame)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"Lower-rack slot derivation (top-down) — scenario {config.SCENARIO_NAME}")
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] slot detection plot: {out_png}")


def main() -> None:
    rng = np.random.default_rng(args_cli.seed)

    # ---- Kit-free computation ------------------------------------------------------------
    # thin-insertion modes (cutlery into a bay, a disc into a 30 mm tine gap, a flat lay
    # inside the third-rack tray rims): the merged gripper+object hull is a giant wedge that
    # can never fit — use the per-piece cluster
    merged = config.effective_placement_mode() not in ("basket_drop", "plate_slot", "flat_lay_third")
    world = CollisionWorld(cache_dir=config.scenario_cache_dir(), self_check=True, merged_cluster=merged)
    slots = placement.derive_slots(config.scenario_cache_dir())  # mode dispatch per object
    print(f"[INFO] scenario {config.SCENARIO_NAME}: home config in_collision: "
          f"{bool(world.in_collision(np.array(config.HOME_Q)))}")
    print(f"[INFO] derived {len(slots)} slots")
    for s in slots:
        print(f"[INFO]   slot {s.slot_id}: center base-frame "
              f"({s.T_base_slot[0,3]:.3f}, {s.T_base_slot[1,3]:.3f}, {s.T_base_slot[2,3]:.3f}), "
              f"cell {s.width_m*100:.1f} cm")

    goal_sets = []
    for s in slots:
        gs = placement.goal_configs(s, world, rng)
        if args_cli.max_store is not None and len(gs.configs) > args_cli.max_store:
            # deterministic thinning across the whole accepted set (not a prefix — configs
            # arrive grouped by pose sample, and a prefix would keep one pose's wraps only)
            keep = np.linspace(0, len(gs.configs) - 1, args_cli.max_store).astype(int)
            gs.configs = np.asarray(gs.configs)[keep]
        goal_sets.append(gs)
        print(f"[INFO] slot {s.slot_id}: {len(gs.configs)} goal configs "
              f"(funnel: {gs.n_pose_samples} poses -> {gs.n_ik_solutions} IK "
              f"-> -{gs.n_limit_reject} limits -> -{gs.n_collision_reject} collision)")
    slots_path, goals_path = placement.save_slots(
        slots, goal_sets, os.path.join(config.scenario_cache_dir(), "slots")
    )
    print(f"[INFO] wrote {slots_path} and {goals_path}")

    nonempty = [g for g in goal_sets if len(g.configs) > 0]
    min_feasible = config.state_params()["min_feasible_slots"]
    print(f"[INFO] scenario {config.SCENARIO_NAME}: {len(nonempty)}/{len(slots)} slots feasible")
    check(f"at least {min_feasible} slots with non-empty goal sets", len(nonempty) >= min_feasible,
          f"{len(nonempty)}/{len(slots)} slots feasible")

    render_slot_detection(slots, os.path.join(args_cli.out_dir, "slot_detection.png"))

    # rejected examples for the media sheet (with reasons)
    rejected: list[tuple[np.ndarray, str]] = []
    T_obj_w3 = T_inv(np.array(world.manifest["object"]["T_wrist3_obj"]))
    for s in slots:
        if len(rejected) >= 6:
            break
        for T_base_obj in placement.sample_goal_poses(s, 8, rng):
            sols = ik_wrist3_all(T_base_obj @ T_obj_w3)
            for q in sols:
                hit, pairs = world.in_collision(q, return_pairs=True)
                if hit:
                    reason = f"collision: {pairs[0][0]} vs {pairs[0][1]}" if pairs else "collision"
                    rejected.append((q, f"slot {s.slot_id} | {reason}"))
                    break
            if len(rejected) >= 6:
                break

    if not args_cli.enable_cameras:
        print("[WARN] cameras disabled — no contact sheets (run with --enable_cameras)")
        print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
        if FAILURES:
            raise SystemExit(1)
        return

    # ---- Kit pass: pose the robot and shoot contact sheets --------------------------------
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device,
                                gravity=(0.0, 0.0, 0.0))
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=True))
    dscene.author_weld(scene.stage)
    rig = CameraRig(config.CAMERAS, hw=(480, 854))  # smaller tiles for sheets
    sim.reset()
    rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    robot = scene["robot"]
    obj = scene["carried_object"]
    for _ in range(120):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    joint_template = robot.data.joint_pos.clone()
    arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
    device = joint_template.device
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_w3_obj = np.array(world.manifest["object"]["T_wrist3_obj"])

    from dishsim.ur5e_kin import fk_wrist3  # noqa: PLC0415

    def pose_at(q: np.ndarray) -> None:
        full = joint_template.clone()
        full[:, arm_ids] = torch.tensor(q, dtype=full.dtype, device=device)
        robot.write_joint_position_to_sim(position=full)
        robot.write_joint_velocity_to_sim(velocity=torch.zeros_like(full))
        robot.set_joint_position_target(target=full)
        T_w_obj = T_w_base @ fk_wrist3(q) @ T_w3_obj
        pos, quat = T_to_pos_quat(T_w_obj)
        pose_t = torch.tensor(np.concatenate([pos, xyzw_to_wxyz(quat)])[None], dtype=full.dtype, device=device)
        obj.write_root_pose_to_sim(root_pose=pose_t)
        obj.write_root_velocity_to_sim(root_velocity=torch.zeros((1, 6), dtype=full.dtype, device=device))
        for _ in range(2):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        rig.update(dt)

    def shot(q: np.ndarray) -> np.ndarray:
        pose_at(q)
        T_w_obj = T_w_base @ fk_wrist3(q) @ T_w3_obj
        target = T_w_obj[:3, 3]
        rig.set_view("iso", target + np.array([-0.45, 0.45, 0.35]), target, sim.device)
        for _ in range(3):
            sim.step()
            scene.update(dt)
            rig.update(dt)
        return rig.grab()["iso"]

    sheet_count = 0
    for gs in goal_sets:
        if len(gs.configs) == 0 or sheet_count >= 4:
            continue
        take = min(args_cli.sheets_per_slot, len(gs.configs))
        idxs = np.linspace(0, len(gs.configs) - 1, take).astype(int)
        images = [shot(np.array(gs.configs[i])) for i in idxs]
        labels = [f"slot {gs.slot_id} | goal config {i}" for i in idxs]
        out = contact_sheet(images, labels, os.path.join(args_cli.out_dir, f"accepted_slot{gs.slot_id}_sheet.png"), cols=3)
        print(f"[INFO] accepted sheet: {out}")
        sheet_count += 1
    check("accepted contact sheets rendered", sheet_count >= min(3, len(nonempty)),
          f"{sheet_count} sheets ({len(nonempty)} feasible slots)")

    if rejected:
        images = [shot(q) for q, _ in rejected]
        labels = [label for _, label in rejected]
        out = contact_sheet(images, labels, os.path.join(args_cli.out_dir, "rejected_sheet.png"), cols=3)
        print(f"[INFO] rejected sheet: {out} ({len(rejected)} configs)")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
