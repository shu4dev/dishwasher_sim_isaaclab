#!/usr/bin/env python3
"""Export the UR5e + 2F-85 collision model (hull vertices per link, link-frame offsets) for Kit-free checks.

    scripts/run_kit.sh frigidaire/scripts/setup/frigidaire_arm_model.py --headless

Spawns the arm alone (gripper de-instanced as in every robot run), reads each link's collision-mesh hull and the
open gripper's hull (robot/collide.py link_hulls) and calibrates the fixed offsets between the Isaac link frames
and the analytic FK frames at HOME_Q. Writes results/robot/arm_model.json (mount-independent).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    out = {"result": "FAIL"}
    try:
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
        import numpy as np
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation
        import omni.usd
        from dishsim_frigidaire.robot import collide as C, ur5e
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
        robot = Articulation(ur5e.robot_cfg())
        stage = omni.usd.get_context().get_stage()
        ur5e.deinstance_gripper(stage)
        sim.reset()
        for _ in range(5):
            sim.step()
            robot.update(sim.get_physics_dt())
        hulls = C.link_hulls(stage, robot=robot)
        q = robot.data.joint_pos[0, [list(robot.joint_names).index(j) for j in ur5e.ARM_JOINTS]].cpu().numpy()
        X = C.calibrate(robot, q, np.eye(4))
        out.update(result="PASS", q_calibration=q.tolist(),
                   hulls={k: [np.asarray(v).tolist() for _, v in lst] for k, lst in hulls.items()},
                   X_link={k: np.asarray(v).tolist() for k, v in X.items()})
    except Exception:
        out["error"] = traceback.format_exc()
        print(out["error"], flush=True)
    p = ROOT / "results/robot/arm_model.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out) + "\n")
    print(f"[RESULT] {out['result']} arm model: {({k: len(v) for k, v in out.get('hulls', {}).items()})}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
