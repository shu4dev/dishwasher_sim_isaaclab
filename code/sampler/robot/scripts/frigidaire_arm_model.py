#!/usr/bin/env python3
"""Export the UR5e + EPick collision model (hull vertices per link, link-frame offsets) for Kit-free checks.

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_arm_model.py --headless

Spawns the bare 6.2 arm (robot/ur5e.py), reads each link's collision-mesh hull (robot/collide.py link_hulls), puts the
EPick body's hull under collide's "gripper" key (robot/epick.py: the BODY_R_M cylinder from the flange to BODY_LEN_M
along +z, in wrist_3_link's frame, which is the body frame; the cup is visual only) and calibrates the fixed offsets
between the Isaac link frames and the analytic FK frames at the joints read after five steps (q_calibration). Writes
data/results/robot/arm_model.json (mount-independent; the same schema, the tool under "gripper").
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[4]
TOOL_SIDES = 32               # the body cylinder as two 32-gons (at the flange and at the body's far end)


def tool_points(n=TOOL_SIDES):
    """The EPick body's collider (epick.py) as points in wrist_3_link's frame: radius BODY_R_M, z from 0 to BODY_LEN_M."""
    import numpy as np
    from sampler.robot import epick
    a = np.linspace(0., 2 * np.pi, n, endpoint=False)
    ring = np.c_[epick.BODY_R_M * np.cos(a), epick.BODY_R_M * np.sin(a)]
    return np.r_[np.c_[ring, np.zeros(n)], np.c_[ring, np.full(n, epick.BODY_LEN_M)]]


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
        from sampler.robot import collide as C, ur5e
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu", physics=PhysxCfg(enable_external_forces_every_iteration=False)))
        robot = Articulation(ur5e.robot_cfg())
        stage = omni.usd.get_context().get_stage()
        sim.reset()
        for _ in range(5):
            sim.step()
            robot.update(sim.get_physics_dt())
        hulls = C.link_hulls(stage)                  # the arm's links (the bare arm carries no gripper colliders)
        hulls["gripper"] = [C._hull(tool_points())]  # the EPick body, riding on wrist_3_link (collide.ArmWorld.link_poses)
        q = robot.data.joint_pos.torch[0, [list(robot.joint_names).index(j) for j in ur5e.ARM_JOINTS]].cpu().numpy()
        X = C.calibrate(robot, q, np.eye(4))
        out.update(result="PASS", q_calibration=q.tolist(),
                   hulls={k: [np.asarray(v).tolist() for _, v in lst] for k, lst in hulls.items()},
                   X_link={k: np.asarray(v).tolist() for k, v in X.items()})
    except Exception:
        out["error"] = traceback.format_exc()
        print(out["error"], flush=True)
    p = ROOT / "data/results/robot/arm_model.json"
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
