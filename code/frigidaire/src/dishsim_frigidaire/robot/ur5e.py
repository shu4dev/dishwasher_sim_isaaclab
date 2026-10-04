"""UR5e + Robotiq 2F-85 articulation config and gripper constants.

Ported from ``src/dishsim/robots.py`` on branch on-corrallab (cdc3f51). The asset is the Isaac Sim 6.0
``Robots/UniversalRobots/ur5e/ur5e.usd`` with the ``Gripper = Robotiq_2f_85`` variant, mirrored into
``data/assets/robots/`` by ``code/frigidaire/scripts/setup/mirror_robot_usd.sh`` (two crate-0.9 layers converted to usda for
Isaac Sim 4.5). ``finger_joint`` (0 = open, 0.8 = closed) is the only commanded finger joint; the others follow
through PhysX mimic constraints. Actuator gains are the on-corrallab values; the finger armature/damping are
load-bearing (the near-massless finger links resonate without them).
"""
from __future__ import annotations

import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
UR5E_USD = os.path.join(ROOT, "data", "assets", "robots", "Assets", "Isaac", "6.0", "Isaac", "Robots", "UniversalRobots", "ur5e",
                        "ur5e.usd")
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
HOME_Q = (0.0, -1.712, 1.712, -1.571, -1.571, 0.0)              # code/src/dishsim/config.py (elbow up, facing +x)
T_WRIST3_TCP_POS = (0.0, 0.130, 0.0)                              # code/src/dishsim/config.py (frozen anchors)
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


def robot_cfg(prim_path="/World/Robot", pos=(0., 0., 0.), rot_wxyz=(1., 0., 0., 0.), gripper_effort=10.,
              gripper_stiffness=40.):
    """ArticulationCfg of the arm; gripper effort/stiffness are parameters (the friction gate varies them)."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    if not os.path.isfile(UR5E_USD):
        raise FileNotFoundError(f"{UR5E_USD} missing: run code/frigidaire/scripts/setup/mirror_robot_usd.sh")
    return ArticulationCfg(
        prim_path=prim_path,
        articulation_root_prim_path="/root_joint",   # the asset carries a second, disabled root on the gripper
        spawn=sim_utils.UsdFileCfg(
            usd_path=UR5E_USD, variants={"Gripper": "Robotiq_2f_85"},
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=16, solver_velocity_iteration_count=1),
            activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={**dict(zip(ARM_JOINTS, HOME_Q)), "finger_joint": 0.0, ".*_inner_finger_joint": 0.0,
                       ".*_inner_finger_knuckle_joint": 0.0, ".*_outer_.*_joint": 0.0},
            pos=tuple(pos), rot=tuple(rot_wxyz)),
        actuators={
            "shoulder": ImplicitActuatorCfg(joint_names_expr=["shoulder_.*"], effort_limit_sim=150.0, velocity_limit_sim=3.14,
                                            stiffness=1320.0, damping=72.6636085, friction=0.0, armature=0.0),
            "elbow": ImplicitActuatorCfg(joint_names_expr=["elbow_joint"], effort_limit_sim=150.0, velocity_limit_sim=3.14,
                                         stiffness=600.0, damping=34.64101615, friction=0.0, armature=0.0),
            "wrist": ImplicitActuatorCfg(joint_names_expr=["wrist_.*"], effort_limit_sim=28.0, velocity_limit_sim=3.14,
                                         stiffness=216.0, damping=29.39387691, friction=0.0, armature=0.0),
            "gripper_drive": ImplicitActuatorCfg(joint_names_expr=["finger_joint"], effort_limit_sim=gripper_effort,
                                                 velocity_limit_sim=1.0, stiffness=gripper_stiffness, damping=1.0,
                                                 friction=0.0, armature=0.001),
            "gripper_finger": ImplicitActuatorCfg(joint_names_expr=[".*_inner_finger_joint"], effort_limit_sim=10.0,
                                                  velocity_limit_sim=1.0, stiffness=10.0, damping=0.05, friction=0.0,
                                                  armature=0.001),
            "gripper_passive": ImplicitActuatorCfg(joint_names_expr=[".*_inner_finger_knuckle_joint", "right_outer_knuckle_joint"],
                                                   effort_limit_sim=1.0, velocity_limit_sim=1.0, stiffness=0.0, damping=0.05,
                                                   friction=0.0, armature=0.001),
        },
    )


def deinstance_gripper(stage, root="/World/Robot/Gripper"):
    """Make the 2F-85's instanced mesh prims real prims BEFORE ``sim.reset()``.

    Measured 2026-09-29 (code/frigidaire/scripts/setup/frigidaire_grip_probe.py): on Isaac Sim 4.5 the 6.0 asset's
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
