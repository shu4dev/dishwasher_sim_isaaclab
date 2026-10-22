# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Machine articulation configs: the static dishwasher twins.

``DISHWASHER_V0_CFG`` is the static machine (door locked open at 90 deg, racks at the
scenario extensions), loaded from the per-state machine USD the asset dataset ships
(``data/assets/<machine>/usd/``, racks built in; authored by ``usd_prep`` in
``data/assets/bosch800/source/``, never at import since 2026-10-21).

Joint names (verified against the composed stage, see ``docs/joint_report.md``): door
``RevoluteJoint_dishwasher_2_middle`` (axis X, limits 0–90 deg, body ``E_door_4``), racks
``PrismaticJoint_dishwasher_2_up`` / ``_down`` (limits −0.2–0 m, bodies ``E_shelf_03`` /
``E_shelf_1_04``), articulation root ``/root/E_body_5``.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from . import config
from .quats import xyzw_to_wxyz

DISHWASHER_DOOR_JOINT = "RevoluteJoint_dishwasher_2_middle"
DISHWASHER_RACK_JOINTS = ["PrismaticJoint_dishwasher_2_up", "PrismaticJoint_dishwasher_2_down"]
if config.HAS_THIRD_RACK:
    DISHWASHER_RACK_JOINTS = DISHWASHER_RACK_JOINTS + [config.RACK_THIRD_JOINT]

# v0 derived copy: static machine — door limits clamped open, rack drive targets at the
# configured extensions, the procedural racks built in. The baseline machine derives from the
# ArtVIP source (usd_prep.make_dishwasher_v0_usd, written next to the ArtVIP layers it references);
# the Bosch 800 is fully self-authored (usd_prep.make_bosch800_usd). One file per rack state, so
# config.apply_machine()/apply_scenario() must run BEFORE this import.
_V0_SUFFIX = "" if config.SCENARIO_NAME == "both_out" else f"_{config.SCENARIO_NAME}"
_V0_NAME = "model_bosch800_v0" if config.MACHINE == "bosch800" else "model_dishwasher_2_v0"
DISHWASHER_V0_USD_PATH = os.path.join(config.machine_asset_dir(), "usd", f"{_V0_NAME}{_V0_SUFFIX}.usda")
if not os.path.isfile(DISHWASHER_V0_USD_PATH):
    raise FileNotFoundError(
        f"{DISHWASHER_V0_USD_PATH}: no machine USD for {config.MACHINE} / {config.SCENARIO_NAME} in the asset "
        "dataset (restore: hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets; "
        "new states: data/assets/bosch800/source/export_usd.py)")

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
