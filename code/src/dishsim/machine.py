# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Machine articulation configs: the static dishwasher twins.

``DISHWASHER_V0_CFG`` is the static machine (door locked open at 90 deg, racks at the
scenario extensions); the derived USD is authored on demand at import (see
:mod:`dishsim.usd_prep`).

Joint names (verified against the composed stage, see ``docs/joint_report.md``): door
``RevoluteJoint_dishwasher_2_middle`` (axis X, limits 0–90 deg, body ``E_door_4``), racks
``PrismaticJoint_dishwasher_2_up`` / ``_down`` (limits −0.2–0 m, bodies ``E_shelf_03`` /
``E_shelf_1_04``), articulation root ``/root/E_body_5``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from . import ASSETS_DIR, config
from .quats import xyzw_to_wxyz
from .usd_prep import make_dishwasher_v0_usd

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

DISHWASHER_DOOR_JOINT = "RevoluteJoint_dishwasher_2_middle"
DISHWASHER_RACK_JOINTS = ["PrismaticJoint_dishwasher_2_up", "PrismaticJoint_dishwasher_2_down"]
if config.HAS_THIRD_RACK:
    DISHWASHER_RACK_JOINTS = DISHWASHER_RACK_JOINTS + [config.RACK_THIRD_JOINT]

# v0 derived copy: static machine — door limits clamped open, rack drive targets at the
# configured extensions. The baseline machine derives from the ArtVIP source
# (usd_prep.make_dishwasher_v0_usd); the Bosch 800 is fully self-authored
# (usd_prep.make_bosch800_usd). Both require config.apply_machine()/apply_scenario()
# BEFORE this import (per-scenario derived copies).
_V0_SUFFIX = "" if config.SCENARIO_NAME == "both_out" else f"_{config.SCENARIO_NAME}"
if config.MACHINE == "bosch800":
    from .usd_prep import make_bosch800_usd

    DISHWASHER_V0_USD_PATH = make_bosch800_usd(
        door_open_deg=config.DOOR_OPEN_DEG,
        door_band_deg=config.DOOR_BAND_DEG,
        rack_targets=config.RACK_JOINT_TARGETS,
        suffix=_V0_SUFFIX,
    )
else:
    DISHWASHER_V0_USD_PATH = (
        make_dishwasher_v0_usd(
            _DISHWASHER_SRC_USD,
            door_open_deg=config.DOOR_OPEN_DEG,
            door_band_deg=config.DOOR_BAND_DEG,
            rack_targets=config.RACK_JOINT_TARGETS,
            suffix=_V0_SUFFIX,
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
        # config quats are project-order XYZW; isaaclab 2.1 cfg tuples are WXYZ
        rot=xyzw_to_wxyz(config.DISHWASHER_QUAT_W),
        joint_pos={
            DISHWASHER_DOOR_JOINT: config.DOOR_INIT_RAD,
            "PrismaticJoint_dishwasher_2_up": config.RACK_UPPER_EXT_M,
            "PrismaticJoint_dishwasher_2_down": config.RACK_LOWER_EXT_M,
            **({config.RACK_THIRD_JOINT: config.RACK_THIRD_EXT_M} if config.HAS_THIRD_RACK else {}),
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
