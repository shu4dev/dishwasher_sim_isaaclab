"""UR5e articulation config (plans/2026-11-09-native-ur5e-epick.md, plans/2026-11-09-delete-pinch-layer.md).

Since 2026-11-09 the arm is the bare Isaac Sim 6.2 ``Robots/UniversalRobots/ur5e/ur5e.usd``, streamed from NVIDIA's asset
root at run time (no gripper variant, no project copy; every result records the file's ETag); the EPick-sized suction
body is authored on its flange by ``epick.py``. Arm actuator gains are the on-corrallab values.
"""
from __future__ import annotations

import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
UR5E_USD = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Robots/UniversalRobots/"
            "ur5e/ur5e.usd")
ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
HOME_Q = (0.0, -1.712, 1.712, -1.571, -1.571, 0.0)              # code/util/dishsim/config.py (elbow up, facing +x)


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
