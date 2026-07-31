# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase D gate: FCL collision world vs Isaac Sim contact ground truth (~200 configs).

Protocol (docs/environment.md has the rationale):

- Sample mix: uniform over the parity domain, near-boundary bisections, deep collisions, and
  task-relevant home->rack interpolations.
- FCL verdict: :class:`dishsim.collision_world.CollisionWorld` with ``self_check=False`` —
  PhysX runs the robot with self-collisions disabled, so the simulator cannot see them and
  they must not count against parity.
- Isaac ground truth: gravity off, joints teleported (kinematic write + holding targets +
  matching welded-object pose), 3 settle steps, then *unexpected* contact forces on all robot
  bodies + the carried object's external residual; contact iff any exceeds
  ``config.CONTACT_FORCE_THRESH_N``. Masked as expected: ``base_link`` (sits on the pedestal
  by construction; mirrored on the FCL side) and the calibrated finger-pad pinch on the mug
  (constant by rigidity — the pad bodies are checked on their residual after subtracting the
  mug reaction, and the mug on its non-gripper external residual, so real world contact on
  either still counts).

Acceptance: agreement >= 95 %, and non-conservative mismatches (FCL free / Isaac contact)
<= 1 %. Conservative mismatches (FCL collision / Isaac free) are acceptable — that is what the
hull-inflation margin is for.

Run with:
    scripts/run_kit.sh scripts/14_parity_check.py --headless --enable_cameras
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="FCL vs Isaac collision parity check.")
parser.add_argument("--n_uniform", type=int, default=80)
parser.add_argument("--n_boundary", type=int, default=60)
parser.add_argument("--n_deep", type=int, default=40)
parser.add_argument("--n_task", type=int, default=20)
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--out_dir", type=str, default=None,
                    help="Media dir (default: media/D or media/D/<scenario>).")
parser.add_argument("--scenario", type=str, default="lower_out",
                    help="Rack-state scenario (see config.SCENARIOS).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys
import time

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
if args_cli.out_dir is None:
    args_cli.out_dir = config.scenario_media_dir("D")

from dishsim import scene as dscene  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.media import CameraRig  # noqa: E402
from dishsim.transforms import T_to_pos_quat, make_T  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3, ik_wrist3_all  # noqa: E402

# parity sampling domain: +-pi per joint (full-2pi windups add nothing to parity and mostly
# produce ground collisions)
DOMAIN = np.array([[-np.pi, np.pi]] * 6)


def sample_configs(world: CollisionWorld, rng: np.random.Generator) -> list[dict]:
    samples: list[dict] = []

    def add(q, kind):
        samples.append({"q": np.asarray(q, dtype=float), "kind": kind})

    uniform = rng.uniform(DOMAIN[:, 0], DOMAIN[:, 1], size=(max(args_cli.n_uniform * 4, 400), 6))
    verdicts = [world.in_collision(q) for q in uniform]
    free_pool = [q for q, v in zip(uniform, verdicts) if not v]
    hit_pool = [q for q, v in zip(uniform, verdicts) if v]
    print(f"[INFO] uniform pool: {len(hit_pool)}/{len(uniform)} in collision")

    for q in uniform[: args_cli.n_uniform]:
        add(q, "uniform")

    # near-boundary: bisect a free/colliding pair toward the surface, keep the FREE endpoint —
    # a genuinely-near sample (mm..cm gap), not a knife-edge midpoint that stresses nothing but
    # sub-mm modeling noise
    n_pairs = min(args_cli.n_boundary, len(free_pool), len(hit_pool))
    for i in range(n_pairs):
        qa = free_pool[i % len(free_pool)].copy()  # free
        qb = hit_pool[i % len(hit_pool)].copy()  # colliding
        for _ in range(8):
            qm = 0.5 * (qa + qb)
            if world.in_collision(qm):
                qb = qm
            else:
                qa = qm
        add(qa, "boundary")

    # deep collisions
    for q in (hit_pool * 3)[: args_cli.n_deep]:
        add(q, "deep")

    # task-relevant: home -> IK poses above the extended rack
    home = np.array(config.HOME_Q)
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    goals = []
    for xy in ((0.5, 0.0), (0.6, 0.1), (0.55, -0.12), (0.65, 0.0)):
        # tool pointing down over the rack, wrist_3 pose derived from a TCP goal 30 cm up
        T_base_tcp = make_T((xy[0], xy[1], 0.42), (1.0, 0.0, 0.0, 0.0))  # tool z down (Rx(pi))
        from dishsim.transforms import T_inv  # noqa: PLC0415

        T_base_w3 = T_base_tcp @ T_inv(dscene.t_wrist3_tcp())
        sols = ik_wrist3_all(T_base_w3, q_seed=home)
        goals.extend(sols[:2])
    for i in range(args_cli.n_task):
        if not goals:
            break
        g = goals[i % len(goals)]
        frac = rng.uniform(0.2, 1.0)
        add(home + frac * (g - home), "task")

    return samples


def main() -> None:
    rng = np.random.default_rng(args_cli.seed)
    # per-link fidelity (merged_cluster=False): parity validates the exact geometry model;
    # planning then uses the strictly-more-conservative merged variant on top of it
    world = CollisionWorld(cache_dir=config.scenario_cache_dir(), self_check=False, merged_cluster=False)

    t0 = time.perf_counter()
    _ = [world.in_collision(q) for q in rng.uniform(-np.pi, np.pi, size=(200, 6))]
    per_query_ms = (time.perf_counter() - t0) / 200 * 1e3
    print(f"[INFO] FCL query latency: {per_query_ms:.2f} ms/query")

    samples = sample_configs(world, rng)
    for s in samples:
        s["fcl"] = bool(world.in_collision(s["q"]))
        s["min_dist"] = float(world.min_distance(s["q"])) if not s["fcl"] else -1.0
    print(f"[INFO] {len(samples)} samples prepared")

    # ---- Isaac ground truth ------------------------------------------------------------------
    sim = SimulationContext(
        sim_utils.SimulationCfg(
            dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg(), gravity=(0.0, 0.0, 0.0)
        )
    )
    scene_cfg = dscene.make_scene_cfg(with_object=True, with_robot_contacts=True)
    scene = InteractiveScene(scene_cfg)
    dscene.author_weld(scene.stage)
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

    # settle ONCE so the mimic finger cluster reaches its consistent closed-on-the-mug state;
    # the settled joint vector is the teleport template. (Teleporting mimics to inconsistent
    # values makes the cluster snap violently every sample — that artifact produced phantom
    # finger/object contact spikes in early parity runs. The contact-deformed settled state is
    # the correct template: it is exactly the state held during planned motion.)
    for _ in range(120):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    joint_template = robot.data.joint_pos.torch.clone()
    grip_ok, grip_detail = dscene.grip_gate(scene)
    print(f"[INFO] settled grip gate: {'OK' if grip_ok else 'FAIL — ' + grip_detail}")

    sensors = [scene["robot_contacts_arm"], scene["robot_contacts_gripper"]]
    sensor_names = [list(getattr(s, "body_names", [])) for s in sensors]

    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_w3_obj = np.array(world.manifest["object"]["T_wrist3_obj"])

    def isaac_contact(q: np.ndarray) -> tuple[bool, float, str]:
        full = joint_template.clone()
        full[:, arm_ids] = torch.tensor(q, dtype=full.dtype, device=device)
        robot.write_joint_position_to_sim_index(position=full)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(full))
        robot.set_joint_position_target_index(target=full)
        T_w_obj = T_w_base @ fk_wrist3(q) @ T_w3_obj
        pos, quat = T_to_pos_quat(T_w_obj)
        pose = torch.tensor(np.concatenate([pos, quat])[None], dtype=full.dtype, device=device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=full.dtype, device=device))
        peak, peak_body = 0.0, ""
        for step in range(4):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            if step == 0:
                continue  # skip the immediate post-teleport step (constraint-settling noise)
            p, b = dscene.unexpected_robot_contact(scene, sensors, sensor_names)
            if p > peak:
                peak, peak_body = p, b
            ext = dscene.grip_forces(scene)["external_n"]
            if ext > peak:
                peak, peak_body = ext, "carried_object"
        return peak > config.CONTACT_FORCE_THRESH_N, peak, peak_body

    results = []
    for i, s in enumerate(samples):
        isaac, peak, peak_body = isaac_contact(s["q"])
        results.append({**{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()},
                        "isaac": isaac, "peak_force": peak, "peak_body": peak_body})
        if (i + 1) % 50 == 0:
            print(f"[INFO] ground truth {i+1}/{len(samples)}")

    # ---- metrics ------------------------------------------------------------------------------
    n = len(results)
    tt = sum(1 for r in results if r["fcl"] and r["isaac"])
    ff = sum(1 for r in results if not r["fcl"] and not r["isaac"])
    fcl_only = [r for r in results if r["fcl"] and not r["isaac"]]  # conservative
    isaac_only = [r for r in results if not r["fcl"] and r["isaac"]]  # NOT acceptable
    agreement = (tt + ff) / n
    print(f"[INFO] confusion: both-collide {tt}, both-free {ff}, "
          f"fcl-only(conservative) {len(fcl_only)}, isaac-only(VIOLATION) {len(isaac_only)}")
    for r in isaac_only:
        print(f"[INFO]   violation: kind={r['kind']} body={r['peak_body']} peak={r['peak_force']:.2f} N "
              f"min_dist={r['min_dist']:.4f} q={np.round(r['q'], 3)}")
    from collections import Counter
    print("[INFO] violation peak-body histogram:", dict(Counter(r["peak_body"] for r in isaac_only)))

    os.makedirs(args_cli.out_dir, exist_ok=True)
    with open(os.path.join(args_cli.out_dir, "parity_results.json"), "w") as f:
        json.dump({"agreement": agreement, "per_query_ms": per_query_ms,
                   "confusion": {"both_collide": tt, "both_free": ff,
                                 "fcl_only": len(fcl_only), "isaac_only": len(isaac_only)},
                   "results": [{k: v for k, v in r.items() if k != "q"} | {"q": list(np.round(r["q"], 4))}
                                for r in results]}, f, indent=2)

    # ---- media: pose 2-3 disagreements in Isaac ------------------------------------------------
    if rig is not None:
        from PIL import Image  # noqa: PLC0415

        disagreements = (isaac_only + fcl_only)[:3]
        for k, r in enumerate(disagreements):
            isaac_contact(np.array(r["q"]))  # pose it
            tcp = scene["ee_frame"].data.target_pos_w.torch[0, 0].cpu().numpy()
            rig.set_view("iso", tcp + np.array([-0.35, 0.35, 0.25]), tcp, sim.device)
            for _ in range(3):
                sim.step()
                scene.update(dt)
                rig.update(dt)
            tag = "violation" if r in isaac_only else "conservative"
            path = os.path.join(args_cli.out_dir, f"disagree_{k}_{tag}.png")
            Image.fromarray(rig.grab()["iso"]).save(path)
            print(f"[INFO] disagreement still: {path} (fcl={r['fcl']}, isaac={r['isaac']}, kind={r['kind']})")

    ok = agreement >= 0.95 and len(isaac_only) <= max(1, int(0.01 * n))
    print(f"[INFO] agreement {agreement*100:.1f} % over {n} configs")
    print(f"[RESULT] {'PASS' if ok else 'FAIL'}")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
