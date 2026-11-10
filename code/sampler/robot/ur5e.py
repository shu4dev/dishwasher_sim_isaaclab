"""UR5e articulation config, plus the Robotiq 2F-85 constants kept for Plan B (plans/2026-11-09-native-ur5e-epick.md).

Since 2026-11-09 the arm is the bare Isaac Sim 6.2 ``Robots/UniversalRobots/ur5e/ur5e.usd``, streamed from NVIDIA's asset
root at run time (no gripper variant, no project copy; every result records the file's ETag); the EPick-sized suction
body is authored on its flange by ``epick.py``. The 2F-85 helpers below (``finger_joint`` 0 = open, 0.8 = closed, the
inner fingers mimic-driven, the TCP 130 mm off wrist_3) served the 6.0 mirror with the ``Gripper = Robotiq_2f_85``
variant and go with Plan B. Arm actuator gains are the on-corrallab values.
"""
from __future__ import annotations

import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
UR5E_USD = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Robots/UniversalRobots/"
            "ur5e/ur5e.usd")
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
HOME_Q = (0.0, -1.712, 1.712, -1.571, -1.571, 0.0)              # code/util/dishsim/config.py (elbow up, facing +x)
T_WRIST3_TCP_POS = (0.0, 0.130, 0.0)                              # code/util/dishsim/config.py (frozen anchors)
T_WRIST3_TCP_QUAT_XYZW = (-0.5, -0.5, -0.5, 0.5)
FINGER_OPEN, FINGER_CLOSED = 0.0, 0.8                              # finger_joint [rad]; limits 0-0.82
MAX_OPENING_M = 0.085
# the two stiff (k = 10) inner-finger drives must be commanded mimic-consistently or they fight the mimic
# constraint (on-corrallab config.GRIPPER_INNER_FINGER_SIGNS; measured signs)
INNER_FINGER_SIGNS = {"left_inner_finger_joint": -1.0, "right_inner_finger_joint": 1.0}


def set_gripper(target, joint_names, theta):
    """Write a finger_joint aperture into a joint-target tensor row 0, inner fingers mimic-consistent."""
    target[0, joint_names.index("finger_joint")] = float(theta)
    for name, sign in INNER_FINGER_SIGNS.items():
        target[0, joint_names.index(name)] = sign * float(theta)


def robot_cfg(prim_path="/World/Robot", pos=(0., 0., 0.), rot_xyzw=(0., 0., 0., 1.), articulation_root=None):
    """ArticulationCfg of the bare 6.2 arm, arm actuators only. ``articulation_root``: the root prim relative to
    ``prim_path`` as step 0 of frigidaire_epick_test.py reads it from the file (None = Isaac Lab finds the single root)."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

    return ArticulationCfg(
        prim_path=prim_path,
        articulation_root_prim_path=articulation_root,
        spawn=sim_utils.UsdFileCfg(
            usd_path=UR5E_USD,
            rigid_props=[sim_utils.UsdPhysicsRigidBodyCfg(),
                         PhysxRigidBodyCfg(disable_gravity=True, max_depenetration_velocity=5.0)],
            articulation_props=[PhysxArticulationCfg(
                enabled_self_collisions=False, solver_position_iteration_count=16, solver_velocity_iteration_count=1)],
            activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=dict(zip(ARM_JOINTS, HOME_Q)), pos=tuple(pos), rot=tuple(rot_xyzw)),
        actuators={
            "shoulder": ImplicitActuatorCfg(joint_names_expr=["shoulder_.*"], joint_effort_limit=150.0, joint_velocity_limit=3.14,
                                            stiffness=1320.0, damping=72.6636085, friction=0.0, armature=0.0),
            "elbow": ImplicitActuatorCfg(joint_names_expr=["elbow_joint"], joint_effort_limit=150.0, joint_velocity_limit=3.14,
                                         stiffness=600.0, damping=34.64101615, friction=0.0, armature=0.0),
            "wrist": ImplicitActuatorCfg(joint_names_expr=["wrist_.*"], joint_effort_limit=28.0, joint_velocity_limit=3.14,
                                         stiffness=216.0, damping=29.39387691, friction=0.0, armature=0.0),
        },
    )


def deinstance_gripper(stage, root="/World/Robot/Gripper"):
    """Make the 2F-85's instanced mesh prims real prims BEFORE ``sim.reset()``.

    Measured 2026-09-29 (code/sampler/robot/scripts/frigidaire_grip_probe.py): on Isaac Sim 4.5 the 6.0 asset's
    gripper colliders live inside instanceable ``visuals`` Xforms; PhysX creates the shapes (property query lists
    them) but they generate NO contacts with anything (a 30 mm cube, the ground, a bowl: 0 N, fingers close
    through). De-instanced, the same pads stop on a 30 mm cube at 0.56 rad with 26-60 N and hold it. This is the
    root cause of the 2026-08 "pads give 0 N on this box" finding. Returns the number of prims de-instanced."""
    from pxr import Usd
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.IsInstance():
            prim.SetInstanceable(False)
            n += 1
    return n


PAD_FRICTION = 1.0           # silicone-like pads (real 2F-85 pads are rubber); PhysX averages with the dish's 0.45 static
PAD_TORSIONAL_RADIUS_M = .01  # contact patch radius for torsional friction: resists a dish pivoting about the pinch


def pad_material(stage, friction=PAD_FRICTION, torsional_radius=PAD_TORSIONAL_RADIUS_M, root="/World/Robot/Gripper"):
    """Bind a pad physics material and a torsional patch radius to the fingertip colliders (call after
    ``deinstance_gripper``, before ``sim.reset()``). Returns the number of pad colliders changed."""
    from pxr import Usd, UsdPhysics, UsdShade, PhysxSchema
    mat_path = "/World/PhysicsMaterials/GripperPad"
    mat = UsdShade.Material.Define(stage, mat_path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction) * .9)
    api.CreateRestitutionAttr(0.)
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and "fingertips" in prim.GetName():
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateTorsionalPatchRadiusAttr(float(torsional_radius))
            n += 1
    return n
