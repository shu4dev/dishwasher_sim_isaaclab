#!/usr/bin/env python3
"""Kit test of the native 6.2 UR5e with an EPick-sized suction body (plans/2026-11-09-native-ur5e-epick.md, Plan A).

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_epick_test.py --headless --enable_cameras

Step 0 opens the streamed arm file and records its facts (the ETag from one HTTP HEAD, layers, variant sets,
articulation root, joints, bodies, flange / tool prims, every prim path with its type). Step 1 teleports the arm to 20
joint poses (uniform in kin.JOINT_LIMITS at seed 0, re-rolled Kit-free while a link origin is within 0.15 m of the ground
or the table) and compares wrist_3's Isaac pose with kin.fk_wrist3 at the measured joints, the fixed frame offset
calibrated at HOME_Q (collide.calibrate). Step 2 picks the plate face up, the bowl and the cup mouth down on their flat
patches (robot/epick.py): the cup tip lowered to the patch (then 1 and 4 mm above it while nothing attaches), close
(150 ms), lift 100 mm, hold 5 s (an RTX still), set down, open (180 ms). The attachment force is the dish's momentum balance while it is held clear of the table
(m (a - g) along and across the cup axis; the extension exposes no joint-force reader). Writes
data/results/robot/epick/epick.json and held_<dish>.png.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "data/results/robot/epick"
DISH_USD = ROOT / "data/assets/hotec_wheatstraw/usd"
DISHES = (("plate", False), ("bowl", True), ("cup", True))           # (kind, mouth down)
PARK = {"plate": (-1.5, -.5, .03), "bowl": (-1.5, 0., .1), "cup": (-1.5, .5, .12)}   # out of the arm's reach
BASE = (0., 0., .3)                                                   # the smoke's height
TABLE_TOP, SPOT, TABLE_HALF = .30, (.45, 0.), (.2, .2, .05)
DT = 1 / 120
N_POSES, SEED, CLEAR_M = 20, 0, .15
POSE_TOL_MM, POSE_TOL_DEG = 1., .1
LIFT_M, HOLD_S, SET_TOL_MM, AIR_M = .10, 5., 5., .002
STANDOFFS_M = (0., .001, .004)        # the tip at the patch (the plan), then 1 and 4 mm above it if no attachment


def twenty_poses(T_wb):
    """Uniform joint poses in kin.JOINT_LIMITS (seed 0), re-rolled while a link origin is within CLEAR_M of the ground or
    the table box (Kit-free)."""
    import numpy as np
    from sampler.robot import kin
    rng = np.random.default_rng(SEED)
    lo = np.array([SPOT[0] - TABLE_HALF[0], SPOT[1] - TABLE_HALF[1], TABLE_TOP - 2 * TABLE_HALF[2]])
    hi = np.array([SPOT[0] + TABLE_HALF[0], SPOT[1] + TABLE_HALF[1], TABLE_TOP])
    out, tries = [], 0
    while len(out) < N_POSES:
        q = rng.uniform(kin.JOINT_LIMITS[:, 0], kin.JOINT_LIMITS[:, 1])
        tries += 1
        p = np.array([(T_wb @ T)[:3, 3] for T in kin.fk_all_links(q).values()])
        if (p[:, 2] >= CLEAR_M).all() and (np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.), axis=1) >= CLEAR_M).all():
            out.append(q)
    return out, tries


def rot_deg(R):
    """Rotation angle of R, accurate for small angles (atan2 of the skew part over the trace)."""
    import numpy as np
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return float(np.degrees(np.arctan2(np.linalg.norm(v) / 2., (np.trace(R) - 1.) / 2.)))


def file_facts(url):
    """Step 0: the ETag (one HTTP HEAD, urllib's generic client name) and the opened file's structure."""
    import urllib.request
    from pxr import Usd, UsdPhysics
    facts = {"url": url}
    try:
        with urllib.request.urlopen(urllib.request.Request(url, method="HEAD"), timeout=30) as r:
            facts.update(http_status=r.status, etag=r.headers.get("ETag"), content_length=r.headers.get("Content-Length"),
                         last_modified=r.headers.get("Last-Modified"))
    except Exception as e:
        facts["head_error"] = repr(e)
    try:
        st = Usd.Stage.Open(url)
    except Exception as e:
        facts["open_error"] = repr(e)
        return facts
    dp = st.GetDefaultPrim()
    prims = list(Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)))
    vsets = {}
    for p in prims:
        vs = p.GetVariantSets()
        if vs.GetNames():
            vsets[str(p.GetPath())] = {n: {"selection": vs.GetVariantSet(n).GetVariantSelection(),
                                           "values": list(vs.GetVariantSet(n).GetVariantNames())} for n in vs.GetNames()}
    facts.update(
        default_prim=str(dp.GetPath()), layers=[layer.identifier for layer in st.GetUsedLayers()], variant_sets=vsets,
        articulation_roots=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)],
        joints=[{"path": str(p.GetPath()), "type": p.GetTypeName(),
                 "body0": [str(t) for t in UsdPhysics.Joint(p).GetBody0Rel().GetTargets()],
                 "body1": [str(t) for t in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()]} for p in prims if p.IsA(UsdPhysics.Joint)],
        bodies=[str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)],
        frames=[[str(p.GetPath()), p.GetTypeName()] for p in prims if any(k in p.GetName().lower() for k in ("flange", "tool"))],
        prims=[[str(p.GetPath()), p.GetTypeName()] for p in prims])
    return facts


def patch_z(usd_path, mouth_down, r_max):
    """The flat patch's height in the dish frame from its collision vertices within r_max of the axis (the highest for
    face up, the outer bottom for mouth down) and the dish's height."""
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics
    st = Usd.Stage.Open(str(usd_path))
    root = st.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    inv = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(root)).T)
    pts = []
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh):
            P = np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float)
            pts.append(((inv @ np.asarray(cache.GetLocalToWorldTransform(p)).T) @ np.c_[P, np.ones(len(P))].T).T[:, :3])
    P = np.vstack(pts)
    near = P[np.hypot(P[:, 0], P[:, 1]) <= r_max, 2]
    return float(near.min() if mouth_down else near.max()), float(P[:, 2].max())


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit below
    add_args(parser)
    args = parser.parse_args()
    cameras = bool(getattr(args, "enable_cameras", False))
    app = launch(args)
    res = {"result": "FAIL"}
    try:
        sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
        import numpy as np
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxRigidBodyCfg
        from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg
        from isaaclab_physx.renderers import IsaacRtxRendererCfg
        from isaaclab_physx.physics import PhysxCfg
        from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
        import carb
        import omni.physx
        import omni.usd
        from pxr import Usd, UsdPhysics
        from sampler.robot import collide, epick, kin, ur5e
        from sampler.robot.rig import Rig, T_of
        sim_utils.enable_extension("isaacsim.robot.surface_gripper")   # before authoring (robot schema) and before play
        from isaacsim.robot.surface_gripper import _surface_gripper
        from isaaclab_physx.assets import SurfaceGripper

        # ---- step 0: the file
        res["file"] = facts = file_facts(ur5e.UR5E_USD)
        roots, dp = facts.get("articulation_roots") or [], facts.get("default_prim", "")
        root_rel = roots[0][len(dp):] or None if roots and roots[0].startswith(dp) else None
        res["articulation_root_rel"] = root_rel
        print(f"[INFO] step 0: etag {facts.get('etag')} roots {roots} variants {facts.get('variant_sets')} frames "
              f"{facts.get('frames')} prims {len(facts.get('prims', []))}", flush=True)

        # ---- scene
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device="cpu", physics=PhysxCfg(enable_external_forces_every_iteration=False)))
        sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=2000.).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.))
        robot = Articulation(ur5e.robot_cfg(pos=BASE, articulation_root=root_rel))
        dishes, paths = {}, {}
        for kind, _ in DISHES:
            paths[kind] = f"/World/{kind.capitalize()}"
            dishes[kind] = RigidObject(RigidObjectCfg(prim_path=paths[kind], spawn=sim_utils.UsdFileCfg(
                usd_path=str(DISH_USD / f"{kind}.usda"), rigid_props=[sim_utils.UsdPhysicsRigidBodyCfg(), PhysxRigidBodyCfg(sleep_threshold=0.0)]),
                init_state=RigidObjectCfg.InitialStateCfg(pos=PARK[kind])))
        table = RigidObject(RigidObjectCfg(prim_path="/World/Table", spawn=sim_utils.CuboidCfg(
            size=tuple(2 * h for h in TABLE_HALF), rigid_props=[sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True), PhysxRigidBodyCfg()],
            collision_props=[sim_utils.UsdPhysicsCollisionCfg(), PhysxCollisionCfg()],
            physics_material=PhysxRigidBodyMaterialCfg(static_friction=.6, dynamic_friction=.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.5, .5, .55))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(SPOT[0], SPOT[1], TABLE_TOP - TABLE_HALF[2]))))
        stage = omni.usd.get_context().get_stage()
        w3_path = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"))
                       if p.GetName() == "wrist_3_link" and p.HasAPI(UsdPhysics.RigidBodyAPI))
        sg_path = epick.author(stage, w3_path, "/World/Robot/epick")
        gripper = SurfaceGripper(epick.gripper_cfg(sg_path))
        mass = {k: float(stage.GetPrimAtPath(p).GetAttribute("physics:mass").Get()) for k, p in paths.items()}
        patch = {k: patch_z(DISH_USD / f"{k}.usda", down, epick.CUP_R_M) for k, down in DISHES}
        res.update(w3_path=w3_path, gripper_prim=sg_path, dish_mass_kg=mass, table=table.cfg.prim_path,
                   patch_local_m={k: [round(v, 5) for v in zh] for k, zh in patch.items()})
        cam = None
        if cameras:
            from isaaclab.sensors import Camera, CameraCfg
            cam = Camera(CameraCfg(prim_path="/World/Cam", height=480, width=640, data_types=["rgb"], renderer_cfg=IsaacRtxRendererCfg(),
                                   spawn=sim_utils.PinholeCameraCfg(focal_length=18.)))
        sim.reset()
        view, gstat = gripper.gripper_view, _surface_gripper.GripperStatus
        names, bodies = list(robot.joint_names), list(robot.body_names)
        res.update(joint_names=names, body_names=bodies)

        def status():
            s = view.get_surface_gripper_status()[0]
            return s if isinstance(s, str) else gstat(s).name

        def dish_T(kind):
            d = dishes[kind]
            return T_of(d.data.root_pos_w.torch[0].cpu().numpy(), d.data.root_quat_w.torch[0].cpu().numpy())   # XYZW

        mon = {"kind": None}
        g = np.array([0., 0., -9.81])

        def on_step():
            for d in dishes.values():
                d.update(DT)
            k = mon["kind"]
            if k is None:
                return
            st, mon["n"] = status(), mon["n"] + 1
            v = dishes[k].data.root_lin_vel_w.torch[0].cpu().numpy().astype(float)
            z = float(dishes[k].data.root_pos_w.torch[0, 2])
            mon["trace"].append(st)
            if st == "Closed" and mon["closed_step"] is None:
                mon["closed_step"] = mon["n"]
            if mon["closed_step"] is not None and not mon["opened"] and st != "Closed":
                mon["break_steps"] += 1
            if st == "Closed" and z - mon["z0"] > AIR_M and mon["v_prev"] is not None:
                F = mass[k] * ((v - mon["v_prev"]) / DT - g)        # the attachment's force on the dish (nothing else touches it)
                axis = rig.tcp()[:3, 2]                              # cup axis, toward the dish
                c = float(F @ axis)
                mon["coax_n"] = max(mon["coax_n"], abs(c))
                mon["shear_n"] = max(mon["shear_n"], float(np.linalg.norm(F - c * axis)))
                mon["air_steps"] += 1
            mon["v_prev"] = v

        T_cfg = np.eye(4)
        T_cfg[:3, 3] = BASE
        rig = Rig(sim, robot, T_cfg, on_step=on_step, T_w3_tcp=epick.T_W3_TIP)
        rig.teleport_arm(np.asarray(ur5e.HOME_Q))     # the asset starts at its authored pose, not init_state
        rig.step(60)
        bi = bodies.index("base_link") if "base_link" in bodies else 0
        T_wb = T_of(robot.data.body_link_pos_w.torch[0, bi].cpu().numpy(), robot.data.body_link_quat_w.torch[0, bi].cpu().numpy())
        rig.T_wb, rig.T_bw = T_wb, np.linalg.inv(T_wb)
        q_home = rig.q()
        X = collide.calibrate(robot, q_home, T_wb)["wrist_3_link"]
        res.update(base_body=bodies[bi], T_wb=np.round(T_wb, 6).tolist(),
                   base_vs_cfg_mm=round(float(np.linalg.norm(T_wb[:3, 3] - T_cfg[:3, 3])) * 1e3, 3),
                   base_vs_cfg_deg=round(rot_deg(T_wb[:3, :3]), 4), home_err_rad=float(np.abs(q_home - np.asarray(ur5e.HOME_Q)).max()),
                   X_w3={"pos_mm": round(float(np.linalg.norm(X[:3, 3])) * 1e3, 4), "rot_deg": round(rot_deg(X[:3, :3]), 4),
                         "T": np.round(X, 6).tolist()})
        if "epick" in bodies:            # the body against kin's flange frame (coaxial mount)
            ei = bodies.index("epick")
            E = np.linalg.inv(T_wb @ kin.fk_wrist3(q_home)) @ T_of(robot.data.body_link_pos_w.torch[0, ei].cpu().numpy(),
                                                                   robot.data.body_link_quat_w.torch[0, ei].cpu().numpy())
            res["mount_vs_kin_flange"] = {"pos_mm": round(float(np.linalg.norm(E[:3, 3])) * 1e3, 4), "rot_deg": round(rot_deg(E[:3, :3]), 4)}

        # ---- step 1: 20 joint poses by teleport
        w3 = bodies.index("wrist_3_link")
        poses, tries = twenty_poses(T_cfg)
        rows = []
        for q in poses:
            rig.teleport_arm(q)
            rig.step(3)
            qm = rig.q()
            E = np.linalg.inv(T_wb @ kin.fk_wrist3(qm) @ X) @ T_of(robot.data.body_link_pos_w.torch[0, w3].cpu().numpy(),
                                                                   robot.data.body_link_quat_w.torch[0, w3].cpu().numpy())
            rows.append({"q_cmd": np.round(q, 4).tolist(), "q_meas": np.round(qm, 4).tolist(),
                         "pos_mm": round(float(np.linalg.norm(E[:3, 3])) * 1e3, 4), "rot_deg": round(rot_deg(E[:3, :3]), 4)})
        res.update(poses=rows, pose_tries=tries, pose_max_mm=max(r["pos_mm"] for r in rows), pose_max_deg=max(r["rot_deg"] for r in rows))
        print(f"[INFO] step 1: {len(rows)} poses ({tries} drawn), max {res['pose_max_mm']} mm {res['pose_max_deg']} deg; "
              f"X_w3 {res['X_w3']['pos_mm']} mm {res['X_w3']['rot_deg']} deg", flush=True)

        # ---- step 2: the three picks
        def down(p):
            T = np.diag([1., -1., -1., 1.])        # cup axis (tcp z) straight down
            T[:3, 3] = p
            return T

        q_park = rig.ik(down([SPOT[0], SPOT[1], TABLE_TOP + .25]), np.asarray(ur5e.HOME_Q))
        rig.teleport_arm(q_park)
        rig.step(30)

        def still(name):
            if cam is None:
                return None
            from PIL import Image
            c = rig.tcp()[:3, 3]
            cam.set_world_poses_from_view(torch.tensor([c + np.array([.30, .35, .05])], dtype=torch.float32),
                                          torch.tensor([c + np.array([0., 0., -.05])], dtype=torch.float32))
            for _ in range(30):
                sim.render()
            cam.update(DT)
            OUT.mkdir(parents=True, exist_ok=True)
            Image.fromarray(cam.data.output["rgb"].torch[0, ..., :3].cpu().numpy()).save(OUT / f"held_{name}.png")
            return f"held_{name}.png"

        sq = omni.physx.get_physx_scene_query_interface()
        attach_prim = stage.GetPrimAtPath(sg_path.rsplit("/", 1)[0] + "/attach")

        def ms(step):
            return None if step is None else round(step * DT * 1e3, 1)

        def rle(xs):
            out = []
            for x in xs:
                if out and out[-1][0] == x:
                    out[-1][1] += 1
                else:
                    out.append([x, 1])
            return out

        def held(kind):
            return any(o == paths[kind] or o.startswith(paths[kind] + "/") for o in view.get_gripped_objects()[0])

        def ray(axis):
            """Our own scene query from the attachment frame along the cup axis (diagnostic, the extension's reach)."""
            try:
                o = rig.tcp()[:3, 3] - epick.CLEARANCE_M * axis
                h = sq.raycast_closest(carb.Float3(*o), carb.Float3(*axis), epick.CLEARANCE_M + epick.MAX_GRIP_DISTANCE_M)
                return {k: (round(float(h[k]), 5) if k == "distance" else h[k]) for k in ("hit", "rigidBody", "collision", "distance") if k in h}
            except Exception as e:
                return {"error": repr(e)}

        def cycle(kind, mouth_down, row):
            d, (zp, H) = dishes[kind], patch[kind]
            pos = (SPOT[0], SPOT[1], TABLE_TOP + (H if mouth_down else 0.) + .02)
            d.write_root_pose_to_sim_index(root_pose=torch.tensor([[*pos, *((1., 0., 0., 0.) if mouth_down else (0., 0., 0., 1.))]], dtype=torch.float32))
            d.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6))
            rig.step(240)
            T0 = dish_T(kind)
            p0 = T0[:3, 3].copy()
            tip = (T0 @ np.array([0., 0., zp, 1.]))[:3]
            row.update(start=np.round(p0, 4).tolist(), patch_w=np.round(tip, 4).tolist(), tilt_deg=round(float(np.degrees(np.arccos(abs(T0[2, 2])))), 2))
            q_hover = rig.ik(down(tip + [0., 0., LIFT_M]), rig.q())
            row["approach_ok"] = bool(q_hover is not None and rig.move_joint(q_hover) and rig.move_line(down(tip)))
            if not row["approach_ok"]:
                row["fail"] = getattr(rig, "last_fail", "no IK")
                return
            axis = rig.tcp()[:3, 2]
            row["tip_dz_mm"] = round(float(rig.tcp()[2, 3] - tip[2]) * 1e3, 3)          # > 0: the tip above the patch
            row["tries"], i = [], 0
            for so in STANDOFFS_M:                     # the plan's tip at the patch first, then small stand-offs
                if so and not rig.move_line(down(tip + [0., 0., so])):
                    break
                t = {"standoff_mm": so * 1e3, "ray": ray(axis)}
                mon.update(kind=kind, n=0, closed_step=None, opened=False, break_steps=0, z0=float(p0[2]), v_prev=None,
                           coax_n=0., shear_n=0., air_steps=0, trace=[])
                view.apply_gripper_action([1.])
                attach_step = None
                for i in range(1, round(epick.RETRY_INTERVAL_S / DT) + 1):
                    rig.step(1)
                    if attach_step is None and held(kind):
                        attach_step = i
                    if attach_step is not None and mon["closed_step"] is not None:
                        break
                rig.step(max(0, round(epick.GRIP_S / DT) - i))                      # the cycle's 150 ms at least
                t.update(attach_ms=ms(attach_step), closed_ms=ms(mon["closed_step"]), status=rle(mon["trace"]),
                         gripped=list(view.get_gripped_objects()[0]), clearance_after=attach_prim.GetAttribute("isaac:clearanceOffset").Get())
                row["tries"].append(t)
                if attach_step is not None:
                    break
                mon["opened"] = True
                view.apply_gripper_action([-1.])
                rig.step(round(epick.RELEASE_S / DT))
            last = row["tries"][-1] if row["tries"] else {}
            so = last.get("standoff_mm", 0.) / 1e3
            row.update(attached=last.get("attach_ms") is not None, standoff_mm=last.get("standoff_mm"), closed_ms=last.get("closed_ms"),
                       close_status=status())
            row["lift_ok"] = bool(rig.move_line(down(tip + [0., 0., so + LIFT_M])))
            pa = dish_T(kind)[:3, 3].copy()
            row["lifted_mm"] = round(float(pa[2] - p0[2]) * 1e3, 2)
            rig.step(round(HOLD_S / DT))
            pb = dish_T(kind)[:3, 3].copy()
            row.update(hold_status=status(), hold_drift_mm=round(float(np.linalg.norm(pb - pa)) * 1e3, 3), still=still(kind))
            row["down_ok"] = bool(rig.move_line(down(tip + [0., 0., so])))
            mon["opened"] = True
            view.apply_gripper_action([-1.])
            rig.step(round(epick.RELEASE_S / DT))
            row.update(open_status=status(), gripped_after_open=list(view.get_gripped_objects()[0]), status=rle(mon["trace"]))
            mon["kind"] = None
            row["retreat_ok"] = bool(rig.move_line(down(tip + [0., 0., so + LIFT_M])))
            rig.step(60)
            p1 = dish_T(kind)[:3, 3]
            row.update(set_down_mm=round(float(np.linalg.norm(p1 - p0)) * 1e3, 3), break_steps=mon["break_steps"], air_steps=mon["air_steps"],
                       coax_max_n=round(mon["coax_n"], 3), shear_max_n=round(mon["shear_n"], 3))
            row["pass"] = bool(row["attached"] and row["closed_ms"] is not None and row["closed_ms"] <= epick.GRIP_S * 1e3 + .5
                               and row["lift_ok"] and row["lifted_mm"] >= (LIFT_M * 1e3 - SET_TOL_MM)
                               and row["hold_status"] == "Closed" and row["hold_drift_mm"] < SET_TOL_MM and row["down_ok"]
                               and row["open_status"] == "Open" and not row["gripped_after_open"] and row["set_down_mm"] <= SET_TOL_MM
                               and row["break_steps"] == 0 and row["coax_max_n"] < epick.COAXIAL_FORCE_LIMIT_N
                               and row["shear_max_n"] < epick.SHEAR_FORCE_LIMIT_N and (cam is None or row["still"] is not None))

        res["dishes"] = {}
        for kind, mouth_down in DISHES:
            row = res["dishes"][kind] = {"mouth_down": mouth_down, "weight_n": round(mass[kind] * 9.81, 3)}
            try:
                cycle(kind, mouth_down, row)
            except Exception:
                row["error"] = traceback.format_exc()
                print(row["error"], flush=True)
            mon["kind"] = None
            print(f"[INFO] {kind}: {row}", flush=True)
            dishes[kind].write_root_pose_to_sim_index(root_pose=torch.tensor([[*PARK[kind], 0., 0., 0., 1.]], dtype=torch.float32))
            dishes[kind].write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6))
            rig.move_joint(q_park)

        ok_file = bool(facts.get("etag")) and "prims" in facts
        ok_poses = res["pose_max_mm"] < POSE_TOL_MM and res["pose_max_deg"] < POSE_TOL_DEG
        ok_dishes = all(res["dishes"].get(k, {}).get("pass") for k, _ in DISHES)
        res["result"] = "PASS" if ok_file and ok_poses and ok_dishes else "FAIL"
    except Exception:
        res["error"] = traceback.format_exc()
        print(res["error"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "epick.json").write_text(json.dumps(res, indent=1) + "\n")
    dl = res.get("dishes", {})
    print(f"[RESULT] {res['result']} epick: etag {res.get('file', {}).get('etag')} poses max {res.get('pose_max_mm')} mm "
          f"{res.get('pose_max_deg')} deg; " + "; ".join(
              f"{k} pass {r.get('pass')} standoff {r.get('standoff_mm')} closed {r.get('closed_ms')} ms lift {r.get('lifted_mm')} "
              f"drift {r.get('hold_drift_mm')} set {r.get('set_down_mm')} mm "
              f"coax {r.get('coax_max_n')}/100 shear {r.get('shear_max_n')}/30 N" for k, r in dl.items()), flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
