# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Measure how a released plate SETTLES in its tine gap — the distribution behind the tolerance.

The place primitive holds the object at a release pose (hover above the slot), drops the weld,
and settles; the settled pose is judged by ``placement.evaluate_placement``. This probe
reproduces exactly that initial condition without the arm: teleport the plate to a sampled
release pose at rest, let it fall, settle, evaluate — one Kit boot covers every feasible gap
x N repeats, where full episodes would cost a boot per data point.

Why it exists (2026-08-14): the one measured Bosch plate trial failed ONLY lateral settle,
14.5 mm against ``tol_lateral_m`` = 0.012 — a v1 tolerance tuned for the 40 mm ArtVIP pitch.
On the Bosch's 50 mm pitch a disc's free lateral play inside a gap is
(pitch − wire − plate)/2 ≈ 16 mm: a plate that rolls to rest AGAINST its tine — how real
plates sit — measures 14-16 mm of "error". This probe measures the actual distribution so the
tolerance can be derived from the machine's geometry instead of eyeballed.

    scripts/run_kit.sh scripts/setup/probe_plate_settle.py --headless \
        --machine bosch800 --placement side_winner --repeats 8
"""

import argparse
import json
import os
import sys

import numpy as np
from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Plate settle distribution probe.")
parser.add_argument("--machine", type=str, default=None)
parser.add_argument("--placement", type=str, default=None)
parser.add_argument("--scenario", type=str, default="placement")
parser.add_argument("--object", type=str, default="plate")
parser.add_argument("--repeats", type=int, default=8, help="Release samples per feasible gap.")
parser.add_argument("--seed", type=int, default=7)
parser.add_argument("--out", type=str, default=None,
                    help="Report JSON (default results/plate_settle/<machine>/report.json).")
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

if args_cli.machine:
    config.apply_machine(args_cli.machine)
if args_cli.placement:
    config.apply_base_placement(args_cli.placement)
config.set_active_object(args_cli.object)
config.apply_scenario(config.resolve_rack_state(args_cli.scenario))

from dishsim import placement  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402


def main() -> int:
    cdir = config.scenario_cache_dir(object_name=args_cli.object)
    gs_path = os.path.join(cdir, "slots", "goal_sets.json")
    slots_path = os.path.join(cdir, "slots", "slots.json")
    if not (os.path.exists(gs_path) and os.path.exists(slots_path)):
        print(f"[FAIL] no baked slots/goal sets under {cdir} — bake the state first")
        return 1
    slots = {s["slot_id"]: placement.SlotFrame.from_json(s)
             for s in json.load(open(slots_path))["slots"]}
    feasible = [g["slot_id"] for g in json.load(open(gs_path))["goal_sets"] if g["configs"]]
    print(f"[INFO] {args_cli.object}/{config.SCENARIO_NAME}: feasible slots {feasible}")
    if not feasible:
        print("[FAIL] no feasible slots to probe")
        return 1

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg()))
    spec = config.OBJECTS[args_cli.object]
    scene = InteractiveScene(dscene.make_scene_cfg(
        with_object=False, with_robot_contacts=False,
        objects=[{"name": "probe", "usd_path": spec.usd_path,
                  "pos": (-2.0, -1.5, 0.10), "quat": (0.0, 0.0, 0.0, 1.0),
                  "contact_filters": []}]))
    sim.reset()
    dscene.write_default_states(scene, aperture=config.GRIPPER_APERTURE_OPEN_RAD)
    dt = sim.get_physics_dt()
    obj = scene["probe"]
    device = obj.data.root_pos_w.torch.device
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)

    def step(n):
        for _ in range(n):
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)

    def teleport(T_base_obj):
        pos_w, quat = T_to_pos_quat(T_w_base @ T_base_obj)
        obj.write_root_pose_to_sim_index(root_pose=torch.tensor(
            np.concatenate([pos_w, quat])[None], dtype=torch.float32, device=device))
        obj.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))

    rng = np.random.default_rng(args_cli.seed)
    results = []
    for sid in feasible:
        slot = slots[sid]
        for k in range(args_cli.repeats):
            T_rel = placement.sample_goal_poses(slot, 1, rng)[0]
            teleport(T_rel)
            step(int(config.SETTLE_STEPS))
            T_settled = T_base_w @ make_T(obj.data.root_pos_w.torch[0].cpu().numpy(),
                                          obj.data.root_quat_w.torch[0].cpu().numpy())
            ev = placement.evaluate_placement(slot, T_settled)
            results.append({"slot": sid, "repeat": k, **{m: round(float(v), 5)
                            for m, v in ev.items() if m != "ok"}, "ok": bool(ev["ok"])})
            print(f"[INFO] slot {sid} rep {k}: lateral {ev['lateral_m'] * 1e3:6.1f} mm  "
                  f"tilt {ev['tilt_deg']:5.1f} deg  bottom {ev['bottom_height_m'] * 1e3:6.1f} mm"
                  f"  ok={ev['ok']}")
            teleport(make_T((-2.0 - 0.3 * (k % 4), -1.5, 0.10), (0, 0, 0, 1)))  # park
            step(10)

    lat = np.array([r["lateral_m"] for r in results])
    tilt = np.array([r["tilt_deg"] for r in results])
    bot = np.array([r["bottom_height_m"] for r in results])
    mp = config.placement_mode_params(slots[feasible[0]].mode)  # the probed slots' own mode
    summary = {
        "machine": config.MACHINE, "placement": config.BASE_PLACEMENT,
        "scenario": config.SCENARIO_NAME, "object": args_cli.object,
        "n": len(results), "n_ok": int(sum(r["ok"] for r in results)),
        "lateral_mm": {"mean": round(float(lat.mean()) * 1e3, 2),
                       "p95": round(float(np.percentile(lat, 95)) * 1e3, 2),
                       "max": round(float(lat.max()) * 1e3, 2)},
        "tilt_deg": {"mean": round(float(tilt.mean()), 2),
                     "p95": round(float(np.percentile(tilt, 95)), 2),
                     "max": round(float(tilt.max()), 2)},
        "bottom_mm": {"mean": round(float(bot.mean()) * 1e3, 2),
                      "max": round(float(bot.max()) * 1e3, 2)},
        "tolerances": {"tol_lateral_m": mp["tol_lateral_m"], "tol_tilt_deg": mp["tol_tilt_deg"],
                       "tol_bottom_m": mp["tol_bottom_m"]},
        "results": results,
    }
    out = args_cli.out or os.path.join(PROJECT_ROOT, "results", "plate_settle",
                                       config.MACHINE, "report.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(summary, f, indent=1)
    print(f"\n[INFO] {summary['n_ok']}/{summary['n']} inside current tolerances "
          f"(tol_lateral {mp['tol_lateral_m'] * 1e3:.0f} mm)")
    print(f"[INFO] lateral mm: mean {summary['lateral_mm']['mean']}, "
          f"p95 {summary['lateral_mm']['p95']}, max {summary['lateral_mm']['max']}")
    print(f"[INFO] tilt deg:   mean {summary['tilt_deg']['mean']}, "
          f"p95 {summary['tilt_deg']['p95']}, max {summary['tilt_deg']['max']}")
    print(f"[INFO] report -> {os.path.relpath(out, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
