#!/usr/bin/env python3
"""Kit test of the generated 2F-85 grasps (robot/grasp.py) on one HOTEC bowl resting in a given pose class.

    scripts/run_kit.sh frigidaire/scripts/setup/frigidaire_grasp_test.py --headless --enable_cameras --pose lying [--tag t]

A bowl rests on a wide kinematic table (upright / mouth down / lying on its side, axis along x); after it settles
the arm (robot/rig.py, pads calibrated, gripper de-instanced) moves above the first reachable generated grasp,
descends in a straight line, closes, lifts 10 cm and holds 3 s. PASS = lifted >= 5 cm and < 5 mm drift over the
hold. Writes results/robot/grasp_test/<tag>.json (+ stills).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results/robot/grasp_test"
BOWL_USD = ROOT / "assets/models/hotec_wheatstraw/v2/bowl.usda"
TABLE_TOP = .40


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pose", choices=("upright", "inverted", "lying"), default="lying")
    parser.add_argument("--tag", default=None)
    parser.add_argument("--candidate", type=int, default=None, help="use this grasp candidate index")
    parser.add_argument("--insert-mm", type=float, default=None, help="override the foot/rim insertion depth")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    cameras = bool(getattr(args, "enable_cameras", False))
    app = AppLauncher(args).app
    tag = args.tag or args.pose
    res = {"result": "FAIL", "pose": args.pose}
    try:
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
        import numpy as np
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
        import omni.usd
        from dishsim_frigidaire.robot import grasp as G, ur5e
        from dishsim_frigidaire.robot.rig import Rig, T_of
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
        sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=2000.).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.))
        robot = Articulation(ur5e.robot_cfg(gripper_effort=8., gripper_stiffness=400.))
        bowl = RigidObject(RigidObjectCfg(prim_path="/World/Bowl", spawn=sim_utils.UsdFileCfg(
            usd_path=str(BOWL_USD), rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 1.5, .1))))
        table = RigidObject(RigidObjectCfg(prim_path="/World/Table", spawn=sim_utils.CuboidCfg(
            size=(.4, .4, .1), rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=.6, dynamic_friction=.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.5, .5, .55))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(.5, 0., TABLE_TOP - .05))))
        stage = omni.usd.get_context().get_stage()
        ur5e.deinstance_gripper(stage)
        ur5e.pad_material(stage)
        cam = None
        if cameras:
            from isaaclab.sensors import Camera, CameraCfg
            cam = Camera(CameraCfg(prim_path="/World/Cam", height=480, width=640, data_types=["rgb"],
                                   spawn=sim_utils.PinholeCameraCfg(focal_length=18.)))
        sim.reset()
        rig = Rig(sim, robot, np.eye(4), on_step=lambda: (bowl.update(sim.get_physics_dt()), table.update(sim.get_physics_dt())))
        # park the arm high, TCP down
        q_park = rig.ik(G.tcp_frame(np.array([.45, 0., .75]), [0., 1., 0.]), np.asarray(ur5e.HOME_Q))
        rig.teleport_arm(q_park)
        # the bowl in its pose class, dropped 5 mm onto the table
        if args.pose == "upright":
            pos, rot = (.5, 0., TABLE_TOP + .005), (1., 0., 0., 0.)
        elif args.pose == "inverted":
            pos, rot = (.5, 0., TABLE_TOP + G.HEIGHT + .005), (0., 1., 0., 0.)
        else:                                               # mouth toward +x, axis horizontal, rim resting on the table
            pos, rot = (.46, 0., TABLE_TOP + G.RIM_R + .005), (math.cos(math.pi / 4), 0., math.sin(math.pi / 4), 0.)
        bowl.write_root_pose_to_sim(torch.tensor([[*pos, *rot]], dtype=torch.float32))
        bowl.write_root_velocity_to_sim(torch.zeros(1, 6))
        rig.step(240)
        p, q = bowl.data.root_pos_w[0].cpu().numpy(), bowl.data.root_quat_w[0].cpu().numpy()
        T_obj = T_of(p, (q[1], q[2], q[3], q[0]))
        res["bowl_class"] = G.bowl_class(T_obj)
        res["bowl_axis"] = np.round(T_obj[:3, 2], 3).tolist()
        if args.insert_mm is not None:
            G.FOOT_INSERT_M = G.RIM_INSERT_M = args.insert_mm * 1e-3
        cands = G.bowl_grasps(T_obj)
        order = [args.candidate] if args.candidate is not None else range(len(cands))
        chosen = None
        for i in order:
            g = cands[i]
            if rig.ik(g["T_tcp"], q_park) is not None and rig.ik(G.hover_along(g["T_tcp"], .1), q_park) is not None:
                chosen = (i, g)
                break
        if chosen is None:
            raise RuntimeError(f"no reachable grasp among {len(cands)}")
        i, g = chosen
        res.update(candidate=i, n_candidates=len(cands), grasp_kind=g["kind"], tcp=np.round(g["T_tcp"][:3, 3], 4).tolist())
        rig.teleport_arm(rig.ik(G.hover_along(g["T_tcp"], .1), q_park))
        rig.step(30)
        res["approach_ok"] = bool(rig.move_line(g["T_tcp"], n=60))
        res["approach_fail"] = getattr(rig, "last_fail", None)
        res["approach_ik"] = rig.line_q(G.hover_along(g["T_tcp"], .1), g["T_tcp"], 60, rig.q()) is not None
        def shot(label):
            if cam is None:
                return
            from PIL import Image
            c = rig.tcp()[:3, 3]
            for eye, name in ((np.array([.25, .30, .08]), "a"), (np.array([0., -.35, .02]), "b")):
                cam.set_world_poses_from_view(torch.tensor([c + eye], dtype=torch.float32),
                                              torch.tensor([c + np.array([0., 0., -.02])], dtype=torch.float32))
                for _ in range(30):
                    sim.render()
                cam.update(sim.get_physics_dt())
                OUT.mkdir(parents=True, exist_ok=True)
                Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(OUT / f"{tag}_{label}_{name}.png")
        shot("preclose")
        res["theta"] = rig.gripper(True)
        shot("closed")
        z0 = float(bowl.data.root_pos_w[0, 2])
        H = G.hover_along(g["T_tcp"], .1)
        res["lift_ok"] = bool(rig.move_line(H, n=60))
        rig.step(60)
        b0 = bowl.data.root_pos_w[0].cpu().numpy().copy()
        rig.step(360)
        b1 = bowl.data.root_pos_w[0].cpu().numpy()
        res["lifted_m"] = round(float(b0[2]) - z0, 4)
        res["drift_3s_m"] = round(float(np.linalg.norm(b1 - b0)), 5)
        res["in_hand_mm"] = np.round((np.linalg.inv(rig.tcp()) @ np.r_[b1, 1.])[:3] * 1e3, 1).tolist()
        if cam is not None:
            from PIL import Image
            c = rig.tcp()[:3, 3]
            cam.set_world_poses_from_view(torch.tensor([c + np.array([.3, .35, .05])], dtype=torch.float32),
                                          torch.tensor([c + np.array([0., 0., -.05])], dtype=torch.float32))
            for _ in range(30):
                sim.render()
            cam.update(sim.get_physics_dt())
            OUT.mkdir(parents=True, exist_ok=True)
            Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(OUT / f"{tag}_held.png")
        res["result"] = "PASS" if res["lifted_m"] > .05 and res["drift_3s_m"] < .005 else "FAIL"
    except Exception:
        res["error"] = traceback.format_exc()
        print(res["error"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{tag}.json").write_text(json.dumps(res, indent=1) + "\n")
    print(f"[RESULT] {res['result']} grasp test {tag}: class {res.get('bowl_class')} axis {res.get('bowl_axis')} cand "
          f"{res.get('candidate')}/{res.get('n_candidates')} approach {res.get('approach_ok')} theta {res.get('theta')} "
          f"lifted {res.get('lifted_m')} drift {res.get('drift_3s_m')} in-hand {res.get('in_hand_mm')}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
