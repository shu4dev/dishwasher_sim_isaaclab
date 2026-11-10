"""EPick-sized suction body on the native 6.2 UR5e (plans/2026-11-09-native-ur5e-epick.md, Plan A).

A generic body of the Robotiq EPick's size, coaxial on the flange (no CAD): a solid cylinder carrying one 40 mm
1.5-bellows cup, the cup modelled as one Isaac surface-gripper attachment point (a D6 joint with every DOF locked:
rigid this round). The flange is wrist_3_link's origin with +z out of the flange (kin.fk_wrist3's frame; the 6.0
asset calibrated to it with X = identity), so the body frame is the flange frame and the cup tip sits TIP_M along +z.
The cup is visual only: the attachment ray starts at the tip, and only the body's cylinder collides.
Kit-side helpers import pxr / Isaac Lab lazily (boot-first).
"""
from __future__ import annotations

import numpy as np

BODY_LEN_M = .1023            # Robotiq EPick manual (2021), with coupling
BODY_R_M = .044               # the EPick's collision radius
BODY_MASS_KG = .788           # 2021 manual, with coupling
CUP_R_M = .020                # one 40 mm 1.5-bellows cup
CUP_LEN_M = .025              # UNVERIFIED: no cup drawing read (PickNik's 15 mm placeholder rejected)
TIP_M = BODY_LEN_M + CUP_LEN_M
T_W3_TIP = np.eye(4)          # the cup tip in wrist_3_link's frame (rig.Rig's T_w3_tcp)
T_W3_TIP[2, 3] = TIP_M
MAX_GRIP_DISTANCE_M = .010
COAXIAL_FORCE_LIMIT_N = 100.  # one 40 mm cup at 80 % vacuum (schema units: N per attachment point)
SHEAR_FORCE_LIMIT_N = 30.     # the same cup at mu 0.3
RETRY_INTERVAL_S = .150       # tries for the manual's grip time
GRIP_S, RELEASE_S = .150, .180  # the manual's grip and release times, no blow-off


def inertia_diag(m=BODY_MASS_KG, r=BODY_R_M, h=BODY_LEN_M):
    """Solid cylinder about its centre, axis z."""
    return (m * (3 * r * r + h * h) / 12., m * (3 * r * r + h * h) / 12., m * r * r / 2.)


def author(stage, w3_path, root):
    """Author the body (rigid, part of the articulation through a fixed joint at the flange), the visual cup, the
    attachment D6 joint and the IsaacSurfaceGripper prim, BEFORE ``sim.reset()``. Returns the gripper prim path."""
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    from usd.schema.isaac import robot_schema

    cache = UsdGeom.XformCache()
    body = UsdGeom.Xform.Define(stage, root)                # the flange frame at the authored pose (USD: local * parent = world)
    body.AddTransformOp().Set(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(w3_path))
                              * cache.GetLocalToWorldTransform(body.GetPrim().GetParent()).GetInverse())
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim()).CreateDisableGravityAttr(True)   # as the arm's links (robot_cfg)
    mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
    mass.CreateMassAttr(BODY_MASS_KG)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0., 0., BODY_LEN_M / 2.))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia_diag()))
    shell = UsdGeom.Cylinder.Define(stage, f"{root}/body")
    shell.CreateAxisAttr("Z")
    shell.CreateHeightAttr(BODY_LEN_M)
    shell.CreateRadiusAttr(BODY_R_M)
    shell.CreateDisplayColorAttr([Gf.Vec3f(.25, .25, .28)])
    shell.AddTranslateOp().Set(Gf.Vec3d(0., 0., BODY_LEN_M / 2.))
    UsdPhysics.CollisionAPI.Apply(shell.GetPrim())
    cup = UsdGeom.Cylinder.Define(stage, f"{root}/cup")     # visual only
    cup.CreateAxisAttr("Z")
    cup.CreateHeightAttr(CUP_LEN_M)
    cup.CreateRadiusAttr(CUP_R_M)
    cup.CreateDisplayColorAttr([Gf.Vec3f(.15, .35, .75)])
    cup.AddTranslateOp().Set(Gf.Vec3d(0., 0., BODY_LEN_M + CUP_LEN_M / 2.))

    mount = UsdPhysics.FixedJoint.Define(stage, f"{root}/mount")
    mount.CreateBody0Rel().SetTargets([w3_path])
    mount.CreateBody1Rel().SetTargets([root])
    for set_pos, set_rot in ((mount.CreateLocalPos0Attr, mount.CreateLocalRot0Attr), (mount.CreateLocalPos1Attr, mount.CreateLocalRot1Attr)):
        set_pos().Set(Gf.Vec3f(0., 0., 0.))
        set_rot().Set(Gf.Quatf(1., 0., 0., 0.))

    # the attachment point: gripper_grasp.py's D6 (all six DOFs locked), body0 the EPick, forward axis +z from the tip;
    # body1 is wrist_3_link at the same point until a close re-points it at the dish
    attach = UsdPhysics.Joint.Define(stage, f"{root}/attach")
    attach.CreateBody0Rel().SetTargets([root])
    attach.CreateBody1Rel().SetTargets([w3_path])
    attach.CreateLocalPos0Attr().Set(Gf.Vec3f(0., 0., TIP_M))
    attach.CreateLocalRot0Attr().Set(Gf.Quatf(1., 0., 0., 0.))
    attach.CreateLocalPos1Attr().Set(Gf.Vec3f(0., 0., TIP_M))
    attach.CreateLocalRot1Attr().Set(Gf.Quatf(1., 0., 0., 0.))
    attach.CreateExcludeFromArticulationAttr().Set(True)
    prim = attach.GetPrim()
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(prim, axis)
        lim.CreateLowAttr().Set(1.)
        lim.CreateHighAttr().Set(-1.)
    robot_schema.ApplyAttachmentPointAPI(prim)
    for attr, val in ((robot_schema.Attributes.FORWARD_AXIS, "Z"), (robot_schema.Attributes.CLEARANCE_OFFSET, 0.)):
        prim.CreateAttribute(attr.name, attr.type).Set(val)
    sg = robot_schema.CreateSurfaceGripper(stage, f"{root}/SurfaceGripper")
    sg.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([prim.GetPath()])
    return str(sg.GetPath())


def gripper_cfg(prim_path):
    """Isaac Lab's SurfaceGripperCfg (CPU physics only) with this cup's limits."""
    from isaaclab_physx.assets import SurfaceGripperCfg
    return SurfaceGripperCfg(prim_path=prim_path, max_grip_distance=MAX_GRIP_DISTANCE_M, coaxial_force_limit=COAXIAL_FORCE_LIMIT_N,
                             shear_force_limit=SHEAR_FORCE_LIMIT_N, retry_interval=RETRY_INTERVAL_S)
