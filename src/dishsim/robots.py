# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Articulation configs: UR5e + Robotiq 2F-85, and the ArtVIP dishwasher (variant ``dishwasher_2``).

**UR5e + Robotiq 2F-85.** The Isaac Sim 6.0 asset ``Robots/UniversalRobots/ur5e/ur5e.usd`` ships
the gripper as a USD variant (``Gripper = Robotiq_2f_85``), pre-assembled on ``wrist_3_link`` via
a fixed joint. The gripper uses native PhysX mimic joints: commanding ``finger_joint`` drives the
other five finger joints through mimic constraints, so only ``finger_joint`` may ever be
commanded. In the v0 planning project the gripper is **frozen** (collision geometry only) — the
actuator groups below still matter because the near-massless mimic-constrained finger links
resonate and blow up without the armature/damping values.

**Dishwasher.** ArtVIP ``dishwasher_2`` with the passive-door derived USD (world-weld removed,
authored door drive neutralized — see :mod:`dishsim.usd_prep`). ``DISHWASHER_CFG`` keeps the
RL-era passive-door setup that the inspection script's stability/door tests are written against.
Phase C adds a ``DISHWASHER_V0_CFG`` (door locked open at 90 deg, lower rack extended).

Joint names (verified against the composed stage, see ``docs/joint_report.md``):
arm ``shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint,
wrist_3_joint``; gripper drive ``finger_joint`` (limits 0–0.82 rad, 0 = open); mimic-driven
``right_outer_knuckle_joint``, ``left/right_inner_finger_joint``,
``left/right_inner_finger_knuckle_joint``; dishwasher door ``RevoluteJoint_dishwasher_2_middle``
(axis X, limits 0–90 deg, body ``E_door_4``), racks ``PrismaticJoint_dishwasher_2_up`` /
``_down`` (limits −0.2–0 m, bodies ``E_shelf_03`` / ``E_shelf_1_04``), articulation root
``/root/E_body_5``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import ASSETS_DIR, config
from .usd_prep import make_dishwasher_rl_usd, make_dishwasher_v0_usd

# ---------------------------------------------------------------------------------------------
# UR5e + Robotiq 2F-85
# ---------------------------------------------------------------------------------------------

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
        activate_contact_sensors=True,
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
        # mode observed under fast arm motion).
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

# ---------------------------------------------------------------------------------------------
# ArtVIP dishwasher (variant dishwasher_2)
# ---------------------------------------------------------------------------------------------

_DISHWASHER_SRC_USD = os.path.join(
    ASSETS_DIR,
    "artvip",
    "Articulated_objects",
    "major_appliances",
    "dishwasher",
    "dishwasher_2",
    "model_dishwasher_2.usda",
)

# Passive-door derived copy (world-weld joint removed, door drive neutralized); generated on
# demand, the downloaded original stays pristine. See usd_prep.py for the why.
DISHWASHER_USD_PATH = (
    make_dishwasher_rl_usd(_DISHWASHER_SRC_USD) if os.path.isfile(_DISHWASHER_SRC_USD) else _DISHWASHER_SRC_USD
)

DISHWASHER_DOOR_JOINT = "RevoluteJoint_dishwasher_2_middle"
DISHWASHER_RACK_JOINTS = ["PrismaticJoint_dishwasher_2_up", "PrismaticJoint_dishwasher_2_down"]
DISHWASHER_DOOR_BODY = "E_door_4"
DISHWASHER_BASE_BODY = "E_body_5"
DISHWASHER_LOWER_RACK_BODY = "E_shelf_1_04"
DISHWASHER_UPPER_RACK_BODY = "E_shelf_03"

DISHWASHER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DISHWASHER_USD_PATH,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True,
            enabled_self_collisions=False,
        ),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={
            DISHWASHER_DOOR_JOINT: 0.0,  # door closed
            DISHWASHER_RACK_JOINTS[0]: 0.0,  # racks stowed
            DISHWASHER_RACK_JOINTS[1]: 0.0,
        },
    ),
    actuators={
        # passive door: no spring, viscous damping + friction only (overrides the USD drive
        # that would otherwise pull the door toward 90 deg). The bottom-hinged door is
        # gravity-unstable at 0 deg (~0.2 N*m closed-state gravity torque on the 1.06 kg door),
        # so the joint friction acts as the latch surrogate that keeps it closed unaided.
        "door": ImplicitActuatorCfg(
            joint_names_expr=[DISHWASHER_DOOR_JOINT],
            effort_limit_sim=50.0,
            velocity_limit_sim=10.0,
            stiffness=0.0,
            damping=5.0,
            friction=0.6,
        ),
        # racks held stowed with a stiff position drive
        "racks": ImplicitActuatorCfg(
            joint_names_expr=DISHWASHER_RACK_JOINTS,
            effort_limit_sim=100.0,
            velocity_limit_sim=1.0,
            stiffness=200.0,
            damping=10.0,
        ),
    },
)
"""ArtVIP dishwasher_2 with a passive (freely swinging, damped) door and stowed racks."""

# v0 derived copy: static machine — door limits clamped open, rack drive targets at the
# configured extensions (see usd_prep.make_dishwasher_v0_usd).
DISHWASHER_V0_USD_PATH = (
    make_dishwasher_v0_usd(
        _DISHWASHER_SRC_USD,
        door_open_deg=config.DOOR_OPEN_DEG,
        door_band_deg=config.DOOR_BAND_DEG,
        rack_targets=config.RACK_JOINT_TARGETS,
    )
    if os.path.isfile(_DISHWASHER_SRC_USD)
    else _DISHWASHER_SRC_USD
)

DISHWASHER_V0_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DISHWASHER_V0_USD_PATH,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True,
            enabled_self_collisions=False,
        ),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=config.DISHWASHER_POS_W,
        rot=config.DISHWASHER_QUAT_W,
        joint_pos={
            DISHWASHER_DOOR_JOINT: config.DOOR_INIT_RAD,
            "PrismaticJoint_dishwasher_2_up": config.RACK_UPPER_EXT_M,
            "PrismaticJoint_dishwasher_2_down": config.RACK_LOWER_EXT_M,
        },
    ),
    actuators={
        # stiff position holds; the clamped USD limits back them up (belt and braces). The
        # bottom-hinged door's gravity torque (~2 N*m) additionally presses it into the 90 deg
        # limit stop.
        "door": ImplicitActuatorCfg(
            joint_names_expr=[DISHWASHER_DOOR_JOINT],
            effort_limit_sim=200.0,
            velocity_limit_sim=1.0,
            stiffness=300.0,
            damping=30.0,
        ),
        "racks": ImplicitActuatorCfg(
            joint_names_expr=DISHWASHER_RACK_JOINTS,
            effort_limit_sim=200.0,
            velocity_limit_sim=0.5,
            stiffness=1000.0,
            damping=50.0,
        ),
    },
)
"""ArtVIP dishwasher_2 as a static v0 obstacle: door locked open at 90 deg, lower rack fully out."""
