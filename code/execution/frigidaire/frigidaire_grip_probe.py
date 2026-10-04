#!/usr/bin/env python3
"""Friction gate G0: can the Robotiq 2F-85 hold a HOTEC bowl by a rim pinch with NO weld?

    code/util/run_kit.sh code/execution/frigidaire/frigidaire_grip_probe.py --headless [--enable_cameras] \\
        [--effort 10] [--insert-mm 12] [--tag base]

The arm holds the TCP straight down (analytic IK). After reset a bowl (HOTEC v2, 67 g) is placed upright on a
kinematic support so that its rim wall sits between the open pads; the jaws close; the support then drops 10 cm
and the bowl must hang from the pads for 3 s. Logged per phase: finger_joint command/measured, applied finger
torque, bowl position relative to the TCP. PASS = the bowl stays in the jaws with < 5 mm slip. Writes
data/results/robot/grip_probe/<tag>.json (+ stills with cameras). No weld or attachment is ever authored.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/results/robot/grip_probe"
BOWL_USD = ROOT / "data/assets/models/hotec_wheatstraw/v2/bowl.usda"
RIM_R, WALL, HEIGHT = .074, .0035, .075
FOOT_R, FOOT_W = .035, .005                   # HOTEC bowl foot ring: 70 mm diameter, 5 mm wide, 4 mm tall       # HOTEC bowl parameters (rim diameter 148 mm, wall 3.5 mm, height 75 mm)
TCP_TARGET = (.45, 0., .30)                   # robot base frame [m]
SLIP_MAX_M = .005


def T_of(pos, quat_xyzw):
    import numpy as np
    x, y, z, w = quat_xyzw
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = pos
    return T


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--effort", type=float, default=10., help="finger_joint effort limit [N m]")
    parser.add_argument("--stiffness", type=float, default=40., help="finger_joint drive stiffness")
    parser.add_argument("--insert-mm", type=float, default=12., help="rim top above the TCP [mm]")
    parser.add_argument("--close", type=float, default=.8, help="finger_joint close target [rad]")
    parser.add_argument("--tag", default="base")
    parser.add_argument("--inverted", action="store_true", help="bowl mouth down: pinch the foot ring")
    parser.add_argument("--lying", action="store_true", help="bowl on its side (axis along the TCP x): pinch the rim at its side point")
    parser.add_argument("--foot-clamp", action="store_true", help="inverted bowl: clamp the whole foot ring (TCP on the axis)")
    parser.add_argument("--object", choices=("bowl", "cube"), default="bowl", help="cube = 30 mm x 30 mm x 60 mm, 50 g")
    parser.add_argument("--fix", choices=("none", "no_nested_root", "no_collider_mass", "both", "deinstance", "pad_boxes", "pads"), default="none",
                        help="diagnostic stage edits before reset: drop the gripper's nested (disabled) articulation root, "
                             "and/or the MassAPI authored on its collider meshes")
    parser.add_argument("--base-z", type=float, default=0., help="robot base height above the ground [m]")
    parser.add_argument("--tcp-z", type=float, default=.30, help="TCP height above the robot base [m]")
    parser.add_argument("--bowl-iters", type=int, default=4, help="bowl solver position iterations")
    parser.add_argument("--geometry", action="store_true", help="diagnostic: fingertip collider boxes in the TCP frame, open and closed")
    parser.add_argument("--press", choices=("box", "bowl", "ground"), default=None,
                        help="diagnostic: lower the open gripper 6 cm onto a 2 kg box or onto the bowl rim")
    parser.add_argument("--no-close", action="store_true", help="control: the jaws stay open (the bowl must fall)")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    cameras = bool(getattr(args, "enable_cameras", False))
    app = AppLauncher(args).app
    result = {"result": "FAIL", "args": {k: v for k, v in vars(args).items() if k in ("effort", "stiffness", "insert_mm", "close", "tag", "no_close", "bowl_iters")}}
    try:
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
        import numpy as np
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
        from dishsim_frigidaire.robot import kin, ur5e

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
        sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=2000.).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.))
        robot = Articulation(ur5e.robot_cfg(pos=(0., 0., args.base_z), gripper_effort=args.effort, gripper_stiffness=args.stiffness))
        if args.object == "cube":
            spawn_obj = sim_utils.CuboidCfg(size=(.03, .03, .06), rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0),
                                            mass_props=sim_utils.MassPropertiesCfg(mass=.05), collision_props=sim_utils.CollisionPropertiesCfg(),
                                            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=.45, dynamic_friction=.35))
        else:
            spawn_obj = sim_utils.UsdFileCfg(usd_path=str(BOWL_USD), rigid_props=sim_utils.RigidBodyPropertiesCfg(
                sleep_threshold=0.0, solver_position_iteration_count=args.bowl_iters))   # never asleep
        bowl = RigidObject(RigidObjectCfg(prim_path="/World/Bowl", spawn=spawn_obj,
                                          init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 1.5, .0))))
        support = RigidObject(RigidObjectCfg(
            prim_path="/World/Support",
            spawn=sim_utils.CylinderCfg(radius=(.09 if "--inverted" in sys.argv else .03), height=(.2 if "--lying" in sys.argv else .1), rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                                        collision_props=sim_utils.CollisionPropertiesCfg(),
                                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.5, .5, .55))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, -1.5, .05))))
        box = RigidObject(RigidObjectCfg(prim_path="/World/Box", spawn=sim_utils.CuboidCfg(
            size=(.2, .2, .2), rigid_props=sim_utils.RigidBodyPropertiesCfg(sleep_threshold=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=2.), collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(-1.5, 1.5, .1))))
        from isaaclab.sensors import ContactSensor, ContactSensorCfg
        pads = ContactSensor(ContactSensorCfg(prim_path="/World/Robot/Gripper/Robotiq_2F_85/.*_inner_finger",
                                              filter_prim_paths_expr=["/World/Bowl"], update_period=0.0))
        if args.fix != "none":
            from pxr import Usd, UsdPhysics, PhysxSchema
            st = sim.stage
            if args.fix in ("no_nested_root", "both"):
                g = st.GetPrimAtPath("/World/Robot/Gripper/Robotiq_2F_85")
                g.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                g.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            if args.fix in ("no_collider_mass", "both"):
                for p in Usd.PrimRange(st.GetPrimAtPath("/World/Robot/Gripper"), Usd.TraverseInstanceProxies()):
                    if p.HasAPI(UsdPhysics.CollisionAPI) and p.HasAPI(UsdPhysics.MassAPI) and not p.IsInstanceProxy():
                        p.RemoveAPI(UsdPhysics.MassAPI)
            if args.fix == "pads":                          # the fix + calibrated pads (ur5e.py)
                result["deinstanced"] = ur5e.deinstance_gripper(st)
                result["pad_colliders"] = ur5e.pad_material(st)
            if args.fix == "deinstance":                   # instanced meshes -> real prims (colliders included)
                for p in Usd.PrimRange(st.GetPrimAtPath("/World/Robot/Gripper")):
                    if p.IsInstance():
                        p.SetInstanceable(False)
            if args.fix == "pad_boxes":                    # explicit box pads at the fingertip meshes' extent (body frame)
                from pxr import UsdGeom, Gf
                for side in ("left", "right"):
                    bp = st.GetPrimAtPath(f"/World/Robot/Gripper/Robotiq_2F_85/{side}_inner_finger")
                    mesh = next(p for p in Usd.PrimRange(bp, Usd.TraverseInstanceProxies())
                                if p.HasAPI(UsdPhysics.CollisionAPI) and "fingertips" in p.GetName())
                    pts = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
                    M = np.array(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
                    Bw = np.array(UsdGeom.Xformable(bp).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
                    P = (np.linalg.inv(Bw) @ M @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
                    lo, hi = P.min(0), P.max(0)
                    cube = UsdGeom.Cube.Define(st, bp.GetPath().AppendChild("pad_proxy"))
                    cube.CreateSizeAttr(1.0)
                    x = UsdGeom.Xformable(cube)
                    x.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
                    x.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
                    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                    result.setdefault("pad_boxes_body_m", {})[side] = [lo.round(4).tolist(), hi.round(4).tolist()]
            result["fix"] = args.fix
        cam = None
        if cameras:
            from isaaclab.sensors import Camera, CameraCfg
            cam = Camera(CameraCfg(prim_path="/World/Cam", height=480, width=640, data_types=["rgb"],
                                   spawn=sim_utils.PinholeCameraCfg(focal_length=18.)))
        # TCP straight down, closing axis unknown until measured: yaw 0
        T_w3_tcp = T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
        T_tcp = np.eye(4)
        T_tcp[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]     # x along +X, z down
        T_tcp[:3, 3] = (TCP_TARGET[0], TCP_TARGET[1], args.tcp_z)
        T_base = np.eye(4)
        T_base[2, 3] = args.base_z
        sols = kin.ik_wrist3_all(T_tcp @ np.linalg.inv(T_w3_tcp), q_seed=np.asarray(ur5e.HOME_Q))
        if len(sols) == 0:
            raise RuntimeError("no IK solution for the TCP-down pose")
        q = min(sols, key=lambda s: float(np.abs(s - np.asarray(ur5e.HOME_Q)).max()))
        result["q_down"] = [round(float(v), 4) for v in q]
        sim.reset()
        names, bodies = list(robot.joint_names), list(robot.body_names)
        arm = [names.index(j) for j in ur5e.ARM_JOINTS]
        fj = names.index("finger_joint")
        w3 = bodies.index("wrist_3_link")
        kn = [bodies.index("left_outer_knuckle"), bodies.index("right_outer_knuckle")]
        target = robot.data.default_joint_pos.clone()
        target[0, arm] = torch.tensor(q, dtype=target.dtype)
        robot.write_joint_state_to_sim(target, torch.zeros_like(target))
        support_z = [None]

        def step(n, log=None):
            for _ in range(n):
                robot.set_joint_position_target(target)
                robot.write_data_to_sim()
                sim.step()
                robot.update(sim.get_physics_dt())
                bowl.update(sim.get_physics_dt())
                pads.update(sim.get_physics_dt())
                if log is not None:
                    log.append(sample())

        def tcp_world():
            p = robot.data.body_link_pos_w[0, w3].cpu().numpy()
            qw = robot.data.body_link_quat_w[0, w3].cpu().numpy()             # WXYZ at the Isaac Lab surface
            return T_of(p, (qw[1], qw[2], qw[3], qw[0])) @ T_w3_tcp

        def sample():
            T = tcp_world()
            b = bowl.data.root_pos_w[0].cpu().numpy()
            return {"theta": round(float(robot.data.joint_pos[0, fj]), 4),
                    "inner": [round(float(robot.data.joint_pos[0, names.index(n)]), 4) for n in ur5e.INNER_FINGER_SIGNS],
                    "torque": round(float(robot.data.applied_torque[0, fj]), 3),
                    "pad_bowl_N": [round(float(v), 2) for v in pads.data.force_matrix_w[0, :, 0].norm(dim=-1).cpu()],
                    "pad_net_N": [round(float(v), 2) for v in pads.data.net_forces_w[0].norm(dim=-1).cpu()],
                    "bowl_rel_tcp_mm": [round(float(v) * 1e3, 1) for v in (b - T[:3, 3])]}

        step(120)
        T = tcp_world()
        result["tcp_world"] = [round(float(v), 4) for v in T[:3, 3]]
        result["tcp_z_axis"] = [round(float(v), 3) for v in T[:3, 2]]
        u = T[:3, 1].copy()                 # the closing axis is the TCP's y (measured pad geometry: pads at +-y)
        u[2] = 0.
        u /= np.linalg.norm(u)
        result["closing_axis_world"] = [round(float(v), 3) for v in u]
        if args.geometry:
            from pxr import Usd, UsdGeom, UsdPhysics
            stage = sim.stage
            tips = {}
            for side in ("left", "right"):
                body_prim = stage.GetPrimAtPath(f"/World/Robot/Gripper/Robotiq_2F_85/{side}_inner_finger")
                mesh = next(p for p in Usd.PrimRange(body_prim, Usd.TraverseInstanceProxies())
                            if p.HasAPI(UsdPhysics.CollisionAPI) and "fingertips" in p.GetName())
                pts = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=float)
                # mesh -> body: the relative transform in the (static) authored USD
                M = np.array(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
                Bw = np.array(UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
                tips[side] = (np.linalg.inv(Bw) @ M, pts)
            geo = {}
            for label, theta in (("open", 0.), ("half", .4), ("closed", .8)):
                ur5e.set_gripper(target, names, theta)
                step(120)
                row = {}
                for side, (B_M, pts) in tips.items():
                    i = bodies.index(f"{side}_inner_finger")
                    p = robot.data.body_link_pos_w[0, i].cpu().numpy()
                    qw = robot.data.body_link_quat_w[0, i].cpu().numpy()
                    Wb = T_of(p, (qw[1], qw[2], qw[3], qw[0]))
                    P = (np.linalg.inv(T) @ Wb @ B_M @ np.c_[pts, np.ones(len(pts))].T).T[:, :3] * 1e3
                    row[side] = {"min_mm": [round(float(v), 1) for v in P.min(0)], "max_mm": [round(float(v), 1) for v in P.max(0)]}
                geo[label] = row
            result.update(geometry=geo, result="PASS", tcp_axes_world=np.round(T[:3, :3], 3).tolist())
            raise StopIteration
        if args.press:
            if args.press == "ground":
                ur5e.set_gripper(target, names, .8)              # closed: the tips reach 32 mm below the TCP
                step(120)
                obj = box                                        # parked far away: only the TCP track matters
            elif args.press == "box":
                obj = box                                        # on the ground under the TCP (top at 0.2 m)
                box.write_root_pose_to_sim(torch.tensor([[T[0, 3], T[1, 3], .1, 1., 0., 0., 0.]], dtype=torch.float32))
            else:
                obj = bowl                                       # rim wall under one fingertip, rim top 2 cm below the TCP
                c = T[:3, 3] + (RIM_R - WALL / 2) * u
                fz = T[2, 3] - .02 - HEIGHT
                support.write_root_pose_to_sim(torch.tensor([[c[0], c[1], fz - .05, 1., 0., 0., 0.]], dtype=torch.float32))
                bowl.write_root_pose_to_sim(torch.tensor([[c[0], c[1], fz + .001, 1., 0., 0., 0.]], dtype=torch.float32))
            step(60)
            p0 = obj.data.root_pos_w[0].cpu().numpy().copy()
            log = []
            for i in range(120):                                 # TCP down 6 cm in 1 s
                Tg = T.copy()
                Tg[2, 3] -= .06 * (i + 1) / 120
                sol = kin.ik_wrist3_all(np.linalg.inv(T_base) @ Tg @ np.linalg.inv(T_w3_tcp), q_seed=np.asarray(q))
                qq = min(sol, key=lambda s_: float(np.abs(s_ - q).max()))
                target[0, arm] = torch.tensor(qq, dtype=target.dtype)
                step(1)
                box.update(sim.get_physics_dt())
                log.append({"cmd_dz_mm": round(-60 * (i + 1) / 120, 1), "tcp_dz_mm": round(float(tcp_world()[2, 3] - T[2, 3]) * 1e3, 1),
                            "obj_dz_mm": round(float(obj.data.root_pos_w[0, 2].cpu() - p0[2]) * 1e3, 1),
                            "obj_dxy_mm": round(float(np.linalg.norm(obj.data.root_pos_w[0, :2].cpu().numpy() - p0[:2])) * 1e3, 1)})
            step(60)
            box.update(sim.get_physics_dt())
            result.update(press=args.press, press_log=log[::10] + [log[-1]])
            blocked = log[-1]["tcp_dz_mm"] > -50 or (args.press != "ground" and (abs(log[-1]["obj_dz_mm"]) > 2 or log[-1]["obj_dxy_mm"] > 2))
            result["result"] = "PASS" if blocked else "FAIL"              # PASS = the gripper touched the object
            raise StopIteration
        # bowl upright: its rim wall (mid-thickness) on the TCP along the closing axis, rim top insert_mm above the TCP
        if args.object == "cube":
            centre, foot_z = T[:3, 3].copy(), T[2, 3] + args.insert_mm * 1e-3 - .06
        elif args.lying:                                     # axis along the TCP x; the rim side point (wall normal = closing axis) on the TCP
            ax = T[:3, 0] / np.linalg.norm(T[:3, 0])
            # base-centred origin; the mouth faces +ax; rim plane at origin + HEIGHT*ax; rim point = axis point + (RIM_R-WALL/2)*(-u)
            centre = T[:3, 3] + (RIM_R - WALL / 2) * u - (HEIGHT - args.insert_mm * 1e-3) * ax
            centre[2] = T[2, 3]                                # the axis at the TCP height: the side point is level with it
            foot_z = T[2, 3] - RIM_R                            # lowest body point ~ the rim radius below the axis
        elif args.inverted:                                  # foot ring (outer radius 35 mm, 5 mm wide) on the TCP
            centre = T[:3, 3].copy() if args.foot_clamp else T[:3, 3] + (FOOT_R - FOOT_W / 2) * u
            foot_z = T[2, 3] + args.insert_mm * 1e-3 - HEIGHT   # the rim (on the support) height
        else:
            centre = T[:3, 3] + (RIM_R - WALL / 2) * u
            foot_z = T[2, 3] + args.insert_mm * 1e-3 - HEIGHT
        support_z[0] = foot_z - .05
        support.write_root_pose_to_sim(torch.tensor([[centre[0], centre[1], support_z[0], 1., 0., 0., 0.]], dtype=torch.float32))
        oz = foot_z + (.03 if args.object == "cube" else HEIGHT if args.inverted else 0.) + .001   # cube centre / bowl base
        rot = (0., 1., 0., 0.) if args.inverted else (1., 0., 0., 0.)          # WXYZ: 180 deg about x = mouth down
        if args.lying:                                        # local +z (mouth) -> world +x: 90 deg about y
            oz, rot = centre[2], (0.7071068, 0., 0.7071068, 0.)
        bowl.write_root_pose_to_sim(torch.tensor([[centre[0], centre[1], oz, *rot]], dtype=torch.float32))
        bowl.write_root_velocity_to_sim(torch.zeros(1, 6))
        phases = {}
        phases["settle_open"] = []
        step(60, phases["settle_open"])
        phases["close"] = []
        for s in np.linspace(0., 0. if args.no_close else args.close, 90):
            ur5e.set_gripper(target, names, s)
            step(1, phases["close"])
        phases["hold_on_support"] = []
        step(60, phases["hold_on_support"])

        def shot(label, eye_off=(.25, .30, .02)):
            if cam is None:
                return
            from PIL import Image
            p = T[:3, 3]
            cam.set_world_poses_from_view(torch.tensor([p + np.array(eye_off)], dtype=torch.float32),
                                          torch.tensor([p + np.array([0., 0., -.03])], dtype=torch.float32))
            for _ in range(30):
                sim.render()
            cam.update(sim.get_physics_dt())
            OUT.mkdir(parents=True, exist_ok=True)
            Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(OUT / f"{args.tag}_{label}.png")
        shot("closed")
        shot("closed_side", (.35, 0., .0))
        z0 = phases["hold_on_support"][-1]["bowl_rel_tcp_mm"][2]
        phases["support_drop"] = []
        for i in range(60):                                   # 10 cm in 0.5 s
            z = support_z[0] - .1 * (i + 1) / 60
            support.write_root_pose_to_sim(torch.tensor([[centre[0], centre[1], z, 1., 0., 0., 0.]], dtype=torch.float32))
            step(1, phases["support_drop"])
        phases["hang"] = []
        step(360, phases["hang"])
        z1 = phases["hang"][-1]["bowl_rel_tcp_mm"][2]
        slip = (z0 - z1) * 1e-3                               # settle-in shift: the pivot on the pinch
        h0, h1 = np.array(phases["hang"][0]["bowl_rel_tcp_mm"]), np.array(phases["hang"][-1]["bowl_rel_tcp_mm"])
        result["drift_3s_m"] = round(float(np.linalg.norm(h1 - h0)) * 1e-3, 5)   # the gate (user 2026-09-29): stable hold
        result.update(phases={k: v[:: max(1, len(v) // 12)] + [v[-1]] for k, v in phases.items()},
                      slip_m=round(slip, 4), held=bool(result["drift_3s_m"] < SLIP_MAX_M and slip < .06),   # stable, and not fallen with the support (10 cm)
                      final_rel_tcp_mm=phases["hang"][-1]["bowl_rel_tcp_mm"], theta_closed=phases["hold_on_support"][-1]["theta"])
        if cam is not None:
            from PIL import Image
            p = T[:3, 3]
            cam.set_world_poses_from_view(torch.tensor([p + np.array([.25, .30, .02])], dtype=torch.float32),
                                          torch.tensor([p + np.array([0., 0., -.03])], dtype=torch.float32))
            for _ in range(30):
                sim.render()
            cam.update(sim.get_physics_dt())
            OUT.mkdir(parents=True, exist_ok=True)
            Image.fromarray(cam.data.output["rgb"][0, ..., :3].cpu().numpy()).save(OUT / f"{args.tag}_hang.png")
        result["result"] = "PASS" if result["held"] else "FAIL"
    except StopIteration:
        pass
    except Exception:
        result["error"] = traceback.format_exc()
        print(result["error"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{args.tag}.json").write_text(json.dumps(result, indent=1) + "\n")
    if result.get("press"):
        print(f"[INFO] press {result['press']}: {result['press_log'][-1]}", flush=True)
    print(f"[RESULT] {result['result']} grip probe {args.tag}: slip {result.get('slip_m')} m, theta closed "
          f"{result.get('theta_closed')}, final bowl-TCP {result.get('final_rel_tcp_mm')} mm, TCP z-axis {result.get('tcp_z_axis')}",
          flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
