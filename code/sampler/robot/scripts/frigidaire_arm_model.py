#!/usr/bin/env python3
"""Export the UR5e + EPick collision model (hull vertices per link, link-frame offsets) for Kit-free checks.

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_arm_model.py --headless

Spawns the bare 6.2 arm (robot/ur5e.py) with the EPick body authored on its flange (robot/epick.py author, at
/World/Robot/epick), reads each link's collision-mesh hull and the EPick body's collider from the live stage (robot/collide.py
link_hulls: the tool under the "gripper" key, in wrist_3_link's frame; the cup is visual only) and calibrates the fixed
offsets between the Isaac link frames and the analytic FK frames at the joints read after five steps (q_calibration).
Writes data/results/robot/arm_model.json (mount-independent; the same schema, the tool under "gripper").
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[4]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit below
    add_args(parser)
    args = parser.parse_args()
    app = launch(args)
    out = {"result": "FAIL"}
    try:
        sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
        import numpy as np
        import isaaclab.sim as sim_utils
        from isaaclab_physx.physics import PhysxCfg
        from isaaclab.assets import Articulation
        import omni.usd
        from pxr import Usd, UsdPhysics
        from sampler.robot import collide as C, epick, ur5e
        sim_utils.enable_extension("isaacsim.robot.surface_gripper")   # the robot schema epick.author applies
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu", physics=PhysxCfg(enable_external_forces_every_iteration=False)))
        robot = Articulation(ur5e.robot_cfg())
        stage = omni.usd.get_context().get_stage()
        w3_path = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"))
                       if p.GetName() == "wrist_3_link" and p.HasAPI(UsdPhysics.RigidBodyAPI))
        epick.author(stage, w3_path, "/World/Robot/epick")
        sim.reset()
        for _ in range(5):
            sim.step()
            robot.update(sim.get_physics_dt())
        hulls = C.link_hulls(stage, robot=robot)     # the arm's links and the EPick body ("gripper", on wrist_3_link)
        missing = [k for k in (*C.ARM_LINKS, "gripper") if not hulls.get(k)]
        q = robot.data.joint_pos.torch[0, [list(robot.joint_names).index(j) for j in ur5e.ARM_JOINTS]].cpu().numpy()
        X = C.calibrate(robot, q, np.eye(4))
        out.update(result="FAIL" if missing else "PASS", missing=missing, w3_path=w3_path, q_calibration=q.tolist(),
                   hulls={k: [np.asarray(v).tolist() for _, v in lst] for k, lst in hulls.items()},
                   X_link={k: np.asarray(v).tolist() for k, v in X.items()})
    except Exception:
        out["error"] = traceback.format_exc()
        print(out["error"], flush=True)
    p = ROOT / "data/results/robot/arm_model.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out) + "\n")
    print(f"[RESULT] {out['result']} arm model: {({k: len(v) for k, v in out.get('hulls', {}).items()})} missing {out.get('missing')}",
          flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
