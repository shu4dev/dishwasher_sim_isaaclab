# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the ArtVIP dishwasher (variant ``dishwasher_2``) prepared for RL.

The source USD authors an *active* drive on the door joint (stiffness 20 toward 90 deg) and
sprung rack drives (stiffness 10 toward the stowed position). For a robot-opens-door task the
door must be passive, so this config overrides the door drive to zero stiffness with moderate
damping and joint friction, while the racks are held stowed with a stiff position drive. All
overrides live here in the :class:`~isaaclab.assets.ArticulationCfg` — the downloaded USD is
never modified.

Joint/body names are taken from ``docs/asset_survey.md`` / ``docs/joint_report.md``:
door ``RevoluteJoint_dishwasher_2_middle`` (axis X, limits 0–90 deg, body ``E_door_4`` with a
child ``handle`` mesh), racks ``PrismaticJoint_dishwasher_2_up`` / ``_down`` (limits −0.2–0 m,
bodies ``E_shelf_03`` / ``E_shelf_1_04``), articulation root ``/root/E_body_5``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from .. import ASSETS_DIR
from ..utils import make_dishwasher_rl_usd

_DISHWASHER_SRC_USD = os.path.join(
    ASSETS_DIR,
    "artvip",
    "Articulated_objects",
    "major_appliances",
    "dishwasher",
    "dishwasher_2",
    "model_dishwasher_2.usda",
)

# RL-ready derived copy (world-weld joint removed, door drive neutralized); generated on demand,
# the downloaded original stays pristine. See utils/usd_prep.py for the why.
DISHWASHER_USD_PATH = (
    make_dishwasher_rl_usd(_DISHWASHER_SRC_USD) if os.path.isfile(_DISHWASHER_SRC_USD) else _DISHWASHER_SRC_USD
)

DISHWASHER_DOOR_JOINT = "RevoluteJoint_dishwasher_2_middle"
DISHWASHER_RACK_JOINTS = ["PrismaticJoint_dishwasher_2_up", "PrismaticJoint_dishwasher_2_down"]
DISHWASHER_DOOR_BODY = "E_door_4"
DISHWASHER_BASE_BODY = "E_body_5"

DISHWASHER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=DISHWASHER_USD_PATH,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=True,
            enabled_self_collisions=False,
        ),
        activate_contact_sensors=False,
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
        # racks held stowed for the door-opening task
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
