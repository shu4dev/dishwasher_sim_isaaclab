#!/usr/bin/env python3
"""Kit smoke of the UR5e + Robotiq 2F-85: spawn, reach HOME_Q, open/close the gripper, measure finger travel.

    code/scripts/run_kit.sh code/frigidaire/scripts/setup/frigidaire_robot_smoke.py --headless [--enable_cameras]

Writes data/results/robot/smoke/smoke.json: joint and body names, HOME_Q tracking error, and per close step the
commanded vs measured finger_joint angle and the fingertip-pad separation (the 2026-08 suspect: fingers barely
moved between open and closed). With cameras, stills of open and closed jaws.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "data/results/robot/smoke"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    cameras = bool(getattr(args, "enable_cameras", False))
    app = AppLauncher(args).app
    result = {"result": "FAIL"}
    try:
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
        import numpy as np
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation
        from dishsim_frigidaire.robot import ur5e

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
        sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=2000.).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.))
        robot = Articulation(ur5e.robot_cfg(pos=(0., 0., 0.3)))
        sim.reset()
        names, bodies = list(robot.joint_names), list(robot.body_names)
        arm = [names.index(j) for j in ur5e.ARM_JOINTS]
        fj = names.index("finger_joint")
        pads = [i for i, b in enumerate(bodies) if b.endswith("inner_finger") or b.endswith("inner_finger_pad")]
        target = robot.data.default_joint_pos.clone()

        def step(n):
            for _ in range(n):
                robot.set_joint_position_target(target)
                robot.write_data_to_sim()
                sim.step()
                robot.update(sim.get_physics_dt())

        step(240)
        q = robot.data.joint_pos[0].cpu().numpy()
        result["home_err_rad"] = float(np.abs(q[arm] - np.asarray(ur5e.HOME_Q)).max())
        result["joint_names"], result["body_names"], result["pad_bodies"] = names, bodies, [bodies[i] for i in pads]

        def pad_sep():
            p = robot.data.body_link_pos_w[0, pads].cpu().numpy()
            return float(np.linalg.norm(p[0] - p[-1]) * 1e3) if len(p) >= 2 else None

        trace = []
        for cmd in list(np.linspace(0., ur5e.FINGER_CLOSED, 9)) + [0.]:
            target[0, fj] = float(cmd)
            step(90)
            trace.append({"cmd": round(float(cmd), 3), "meas": round(float(robot.data.joint_pos[0, fj]), 4),
                          "pad_sep_mm": None if pad_sep() is None else round(pad_sep(), 1)})
        result["close_trace"] = trace
        if cameras:
            from isaaclab.sensors import Camera, CameraCfg
            OUT.mkdir(parents=True, exist_ok=True)
            cam = Camera(CameraCfg(prim_path="/World/Cam", height=480, width=640, data_types=["rgb"],
                                   spawn=sim_utils.PinholeCameraCfg(focal_length=18.)))
            sim.reset()
            tcp = robot.data.body_link_pos_w[0, pads].mean(0).cpu().numpy() if pads else np.array([.4, 0., .6])
            cam.set_world_poses_from_view(torch.tensor([tcp + np.array([.35, .35, .15])], dtype=torch.float32),
                                          torch.tensor([tcp], dtype=torch.float32))
            from PIL import Image
            for label, cmd in (("open", 0.), ("closed", ur5e.FINGER_CLOSED)):
                target[0, fj] = cmd
                step(120)
                for _ in range(24):
                    sim.render()
                cam.update(sim.get_physics_dt())
                Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(OUT / f"gripper_{label}.png")
        ok = result["home_err_rad"] < .02 and trace[-2]["meas"] > .6 and len(pads) >= 2
        result["result"] = "PASS" if ok else "FAIL"
    except Exception:
        result["error"] = traceback.format_exc()
        print(result["error"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "smoke.json").write_text(json.dumps(result, indent=1) + "\n")
    print(f"[RESULT] {result['result']} robot smoke: home err {result.get('home_err_rad')}, "
          f"close {[(t['cmd'], t['meas'], t['pad_sep_mm']) for t in result.get('close_trace', [])]}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
