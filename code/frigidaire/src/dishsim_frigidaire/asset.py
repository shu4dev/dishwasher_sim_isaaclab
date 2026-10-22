# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""FDPC4221AS Isaac Lab loading (spawn, gains, passive forces) and the asset's component table.

No Kit or pxr imports occur at module import. The USD authoring half moved to
``data/assets/frigidaire_fdpc4221as/source/asset.py`` on 2026-10-21; ``BODY_POSITIONS`` and ``JOINT_LIMITS`` are
read from the asset folder's ``parameters.json`` (``body_positions_m``, ``joint_limits_usd``).
"""
from pathlib import Path
import math

import numpy as np

from .geometry import PARAMETERS_JSON
from .paths import ASSET_DIR

COMPONENT_FILES = {"Cabinet": "cabinet.usdc", "Door": "door.usdc",
                   "LowerRack": "lower_rack.usdc", "UpperRack": "upper_rack.usdc",
                   "SilverwareBasket": "silverware_basket.usdc"}
BODY_POSITIONS = {name: tuple(position) for name, position in PARAMETERS_JSON["body_positions_m"].items()}
JOINT_LIMITS = {name: tuple(limits) for name, limits in PARAMETERS_JSON["joint_limits_usd"].items()}


def spawn(prim_path, position=(0.,0.,0.), yaw=0., mode="passive", usd_path=None):
    """Spawn before reset. Returns (articulation, independent basket).

    Position and yaw specify a world pose, with angles in radians. Existing parent
    transforms may translate or rotate the asset hierarchy, but must not scale it.
    The world's weld frame follows the requested pose; changing only an ancestor
    Xform after spawning would otherwise snap the fixed articulation back.
    """
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
    if mode not in {"scripted","passive"}:
        raise ValueError(mode)
    q = (math.cos(yaw/2),0.,0.,math.sin(yaw/2))
    asset = str(usd_path or ASSET_DIR/"fdpc4221as.usdc")
    if "://" not in asset:
        asset = str(Path(asset).resolve())
    stage = omni.usd.get_context().get_stage()
    # Isaac Lab's USD spawner accepts a transform relative to the parent. The
    # public helper accepts a world pose, as do the fixed joint and initial states.
    # Missing intermediate parents are created as identity transforms by USD.
    parent_path = Sdf.Path(prim_path).GetParentPath()
    while parent_path != Sdf.Path.emptyPath and not stage.GetPrimAtPath(parent_path):
        parent_path = parent_path.GetParentPath()
    parent = stage.GetPrimAtPath(parent_path) if parent_path != Sdf.Path.emptyPath else stage.GetPseudoRoot()
    parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(parent)
    linear = np.asarray(parent_world)[:3, :3]
    if not np.allclose(linear @ linear.T, np.eye(3), atol=1e-6) or np.linalg.det(linear) < 0:
        raise ValueError("Frigidaire spawn requires a parent without scale, shear, or reflection")
    requested_world = Gf.Matrix4d(1)
    requested_world.SetRotate(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
    requested_world.SetTranslateOnly(Gf.Vec3d(*map(float, position)))
    local = requested_world * parent_world.GetInverse()
    local_q = local.ExtractRotationQuat()
    sim_utils.UsdFileCfg(usd_path=asset).func(prim_path,sim_utils.UsdFileCfg(usd_path=asset),
        translation=tuple(local.ExtractTranslation()),
        orientation=(local_q.GetReal(), *local_q.GetImaginary()))
    fixed = UsdPhysics.FixedJoint.Get(stage,prim_path+"/Joints/base_fixed")
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*map(float,position)))
    fixed.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
    cfg = ArticulationCfg(prim_path=prim_path,spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=position,rot=q,joint_pos={".*":0.}),
        actuators={"door":ImplicitActuatorCfg(joint_names_expr=["door_hinge"],
                    stiffness=300. if mode=="scripted" else 0., damping=45. if mode=="scripted" else .8,
                    effort_limit_sim=30.,velocity_limit_sim=.65),
                   "racks":ImplicitActuatorCfg(joint_names_expr=[".*_slide"],
                    stiffness=1600. if mode=="scripted" else 0., damping=180. if mode=="scripted" else 8.,
                    effort_limit_sim=60.,velocity_limit_sim=.15)})
    appliance = Articulation(cfg)
    door_mass=UsdPhysics.MassAPI(stage.GetPrimAtPath(prim_path+"/Door"))
    appliance._frigidaire_door_mass=float(door_mass.GetMassAttr().Get())
    appliance._frigidaire_door_com=tuple(door_mass.GetCenterOfMassAttr().Get())
    basket_world = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(prim_path+"/SilverwareBasket"))
    basket_q = basket_world.ExtractRotationQuat()
    basket = RigidObject(RigidObjectCfg(prim_path=prim_path+"/SilverwareBasket",spawn=None,
        init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(basket_world.ExtractTranslation()),
            rot=(basket_q.GetReal(), *basket_q.GetImaginary()))))
    appliance._frigidaire_mode = mode
    return appliance,basket


def apply_mode(appliance, mode):
    """Switch resistance/drive gains after reset; no position teleporting."""
    import torch
    if mode not in {"passive","scripted"}:
        raise ValueError(mode)
    stiffness,damping=[],[]
    for name in appliance.joint_names:
        door=name=="door_hinge"
        stiffness.append((300. if door else 1600.) if mode=="scripted" else 0.)
        damping.append((45. if door else 180.) if mode=="scripted" else (.8 if door else 8.))
    k=torch.tensor([stiffness],device=appliance.device)
    d=torch.tensor([damping],device=appliance.device)
    appliance.write_joint_stiffness_to_sim(k)
    appliance.write_joint_damping_to_sim(d)
    # ImplicitActuator mirrors the gains for effort estimation; update that cache too.
    for actuator in appliance.actuators.values():
        actuator.stiffness[:]=k[:,actuator.joint_indices]
        actuator.damping[:]=d[:,actuator.joint_indices]
    appliance.set_joint_effort_target(torch.zeros_like(appliance.data.joint_pos))
    appliance._frigidaire_mode=mode


def step_passive(appliance):
    """Approximate door spring counterbalance plus bounded smooth rack friction.

    Feed this before write_data_to_sim each step in passive mode. Effort offsets
    approximate gravity for the authored door COM; no position servo holds it.
    The mechanical squeeze latch is represented by a released state in this mode.
    """
    import torch
    if getattr(appliance,"_frigidaire_mode",None)!="passive":
        return
    effort=torch.zeros_like(appliance.data.joint_pos)
    for i,name in enumerate(appliance.joint_names):
        q=appliance.data.joint_pos[:,i]
        velocity=appliance.data.joint_vel[:,i]
        if name=="door_hinge":
            mass=appliance._frigidaire_door_mass
            _,cy,cz=appliance._frigidaire_door_com
            effort[:,i]=-mass*9.81*(cz*torch.sin(q)-cy*torch.cos(q)) - .30*torch.tanh(velocity/.015)
        else:
            effort[:,i]=-1.2*torch.tanh(velocity/.008)
    appliance.set_joint_effort_target(effort)
    return effort
