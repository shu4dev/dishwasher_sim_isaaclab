# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a UR5e arm with a Robotiq 2F-85 gripper.

The Isaac Sim 6.0 asset ``Robots/UniversalRobots/ur5e/ur5e.usd`` ships the gripper as a USD
variant (``Gripper = Robotiq_2f_85``), pre-assembled on ``wrist_3_link`` via a fixed joint.
The gripper uses native PhysX mimic joints: commanding ``finger_joint`` drives the other five
finger joints through mimic constraints, so only ``finger_joint`` belongs in the action space.

There is no UR5e config in ``isaaclab_assets``; this one follows the in-tree
``UR10e_ROBOTIQ_2F_85_CFG`` pattern (same asset family and gripper variant) with UR5e effort
limits. Arm gains are seeded from the UR10e values and treated as a tuning knob.

Joint names (verified against the composed stage, see ``docs/joint_report.md``):
arm ``shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,
wrist_3_joint``; gripper drive ``finger_joint`` (limits 0–0.82 rad, 0 = open); mimic-driven
``right_outer_knuckle_joint``, ``left/right_inner_finger_joint``,
``left/right_inner_finger_knuckle_joint``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from .. import ASSETS_DIR

# Prefer the local mirror (created by `retrieve_file_path`, see docs/asset_survey.md); fall back
# to the live Nucleus S3 URL.
_UR5E_LOCAL = os.path.join(
    ASSETS_DIR, "robots", "Assets", "Isaac", "6.0", "Isaac", "Robots", "UniversalRobots", "ur5e", "ur5e.usd"
)
UR5E_USD_PATH = _UR5E_LOCAL if os.path.isfile(_UR5E_LOCAL) else f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur5e/ur5e.usd"

UR5E_ROBOTIQ_2F_85_CFG = ArticulationCfg(
    # the asset carries a second, disabled ArticulationRootAPI on the gripper subtree
    # (physxArticulation:articulationEnabled = False); point at the real root explicitly
    articulation_root_prim_path="/root_joint",
    spawn=sim_utils.UsdFileCfg(
        usd_path=UR5E_USD_PATH,
        variants={"Gripper": "Robotiq_2f_85"},
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # elbow-up home pose facing +x
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": -1.571,
            "wrist_2_joint": -1.571,
            "wrist_3_joint": 0.0,
            # gripper open
            "finger_joint": 0.0,
            ".*_inner_finger_joint": 0.0,
            ".*_inner_finger_knuckle_joint": 0.0,
            ".*_outer_.*_joint": 0.0,
        },
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
    actuators={
        # UR5e joint torque limits: 150 N*m (shoulder/elbow), 28 N*m (wrists); max speed ~pi rad/s.
        "shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*"],
            effort_limit_sim=150.0,
            velocity_limit_sim=3.14,
            stiffness=1320.0,
            damping=72.6636085,
            friction=0.0,
            armature=0.0,
        ),
        "elbow": ImplicitActuatorCfg(
            joint_names_expr=["elbow_joint"],
            effort_limit_sim=150.0,
            velocity_limit_sim=3.14,
            stiffness=600.0,
            damping=34.64101615,
            friction=0.0,
            armature=0.0,
        ),
        "wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_.*"],
            effort_limit_sim=28.0,
            velocity_limit_sim=3.14,
            stiffness=216.0,
            damping=29.39387691,
            friction=0.0,
            armature=0.0,
        ),
        # gripper: only finger_joint is actuated; the rest follow through PhysX mimic constraints.
        # Gains follow the in-tree gear-assembly retune (contact-stable), plus a small armature
        # and damping on every finger joint: the near-massless mimic-constrained finger links
        # otherwise resonate and blow up when the wrist moves fast (exploding-gripper failure
        # mode observed under random PPO actions).
        "gripper_drive": ImplicitActuatorCfg(
            joint_names_expr=["finger_joint"],
            effort_limit_sim=10.0,
            velocity_limit_sim=1.0,
            stiffness=40.0,
            damping=1.0,
            friction=0.0,
            armature=0.001,
        ),
        "gripper_finger": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_finger_joint"],
            effort_limit_sim=10.0,
            velocity_limit_sim=1.0,
            stiffness=10.0,
            damping=0.05,
            friction=0.0,
            armature=0.001,
        ),
        "gripper_passive": ImplicitActuatorCfg(
            joint_names_expr=[".*_inner_finger_knuckle_joint", "right_outer_knuckle_joint"],
            effort_limit_sim=1.0,
            velocity_limit_sim=1.0,
            stiffness=0.0,
            damping=0.05,
            friction=0.0,
            armature=0.001,
        ),
    },
)
"""UR5e + Robotiq 2F-85 (USD gripper variant), position-controlled arm, minimal gripper actuation."""
