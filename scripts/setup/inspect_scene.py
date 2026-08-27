# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Scene inspection for the dishwasher arrangement project.

Spawns ground plane, dome light, the ArtVIP dishwasher (fixed base), a pedestal, and
(optionally) a YCB mug on a side table. Writes the dishwasher joint report to
``docs/joint_report.md``, steps physics and asserts stability.

Modes:
  default            dishwasher uses the passive-door RL config (``DISHWASHER_CFG``): the door
                     drive is overridden to zero stiffness so the door stays closed unaided
  --test_door        additionally runs the three-part door test: stays closed unaided, opens
                     under commanded effort, comes to rest within limits
  --preserve_drives  keeps the USD-authored drives instead (documents the as-shipped behavior —
                     the door drive is a stiff spring toward 90 deg and the sim may go unstable;
                     informational only, no assertions)

Usage:
    /workspace/isaaclab/isaaclab.sh -p scripts/setup/inspect_scene.py --headless
    /workspace/isaaclab/isaaclab.sh -p scripts/setup/inspect_scene.py --headless --test_door
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Inspect the dishwasher-manipulation scene.")
parser.add_argument("--dishwasher", type=str, default="dishwasher_2", help="ArtVIP dishwasher variant name.")
parser.add_argument("--steps", type=int, default=500, help="Number of physics steps for the stability check.")
parser.add_argument("--report", type=str, default=os.path.join(PROJECT_ROOT, "docs", "joint_report.md"))
parser.add_argument("--test_door", action="store_true", help="Run the three-part passive-door test.")
parser.add_argument("--preserve_drives", action="store_true", help="Keep USD drives (informational, no assertions).")
parser.add_argument("--with_mug", action="store_true", help="Also spawn the derived YCB mug on a side table.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils.configclass import configclass
from isaaclab_physx.physics import PhysxCfg

from pxr import Gf, Usd, UsdGeom, UsdPhysics

# make the (not necessarily pip-installed) project package importable
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim.machine import DISHWASHER_CFG  # noqa: E402
from dishsim.usd_prep import make_dishwasher_rl_usd  # noqa: E402

DISHWASHER_DIR = os.path.join(PROJECT_ROOT, "assets", "artvip", "Articulated_objects", "major_appliances", "dishwasher")

# scene geometry (meters). The dishwasher door is bottom-hinged at its front face and sweeps a
# ~0.36 m arc toward the base anchor when opening; the dishwasher stands far enough back that
# the fully open door clears the pedestal (see the swept-arc numbers in the generated report).
PEDESTAL_SIZE = (0.25, 0.25, 0.25)
# world-frame target for the closed-door handle (the fixed spawn convention)
HANDLE_TARGET_XY = (0.64, 0.0)


def resolve_dishwasher_usd(name_or_path: str) -> str:
    """Resolve a variant name like ``dishwasher_2`` (or an explicit path) to a model USD path."""
    if os.path.isfile(name_or_path):
        return os.path.abspath(name_or_path)
    for ext in ("usda", "usd"):
        candidate = os.path.join(DISHWASHER_DIR, name_or_path, f"model_{name_or_path}.{ext}")
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f"No dishwasher USD found for '{name_or_path}' under {DISHWASHER_DIR}")


def usd_joint_table(stage: Usd.Stage) -> list[dict]:
    """Collect joint info (type, axis, limits, drive params, bodies) from a USD stage."""
    rows = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            kind, drive_token, unit = "revolute", "angular", "deg"
            joint = UsdPhysics.RevoluteJoint(prim)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            kind, drive_token, unit = "prismatic", "linear", "m"
            joint = UsdPhysics.PrismaticJoint(prim)
        elif prim.IsA(UsdPhysics.FixedJoint):
            rows.append({"name": prim.GetName(), "type": "fixed", "path": str(prim.GetPath())})
            continue
        else:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, drive_token)
        body1 = prim.GetRelationship("physics:body1").GetTargets()
        mimic = any("mimicjoint" in a.GetName().lower() for a in prim.GetAttributes())
        rows.append({
            "name": prim.GetName(),
            "type": kind,
            "path": str(prim.GetPath()),
            "axis": str(joint.GetAxisAttr().Get()),
            "lower": joint.GetLowerLimitAttr().Get(),
            "upper": joint.GetUpperLimitAttr().Get(),
            "unit": unit,
            "stiffness": drive.GetStiffnessAttr().Get() if drive else None,
            "damping": drive.GetDampingAttr().Get() if drive else None,
            "target": drive.GetTargetPositionAttr().Get() if drive else None,
            "max_force": drive.GetMaxForceAttr().Get() if drive else None,
            "body1": body1[0].name if body1 else "?",
            "mimic": mimic,
        })
    return rows


def usd_collision_audit(stage: Usd.Stage) -> dict[str, int]:
    """Count mesh collision approximations used in the stage."""
    counts: dict[str, int] = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            approx = str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
            counts[approx] = counts.get(approx, 0) + 1
    return counts


def analyze_dishwasher_usd(usd_path: str) -> dict:
    """Extract door/rack joints, handle geometry, and placement info from the dishwasher USD."""
    stage = Usd.Stage.Open(usd_path)
    info: dict = {"joints": usd_joint_table(stage), "collision": usd_collision_audit(stage)}

    # articulation root. NOTE: at spawn, Isaac Lab places the ROOT LINK frame at the configured
    # pose (the root prim's own asset-frame offset is discarded), so all placement math below is
    # expressed relative to the root link frame, not the asset origin.
    info["articulation_roots"] = [
        str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    root_prim = stage.GetPrimAtPath(info["articulation_roots"][0])
    root_xf = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    info["root_link_translate"] = tuple(root_xf.ExtractTranslation())

    # door = revolute joint whose name matches door/hinge, else largest ~90 deg span
    revolutes = [j for j in info["joints"] if j["type"] == "revolute"]
    prismatics = [j for j in info["joints"] if j["type"] == "prismatic"]
    door = next((j for j in revolutes if "door" in j["name"].lower() or "hinge" in j["name"].lower()), None)
    if door is None and revolutes:
        door = max(revolutes, key=lambda j: abs(j["upper"] - j["lower"]))
    info["door_joint"] = door
    info["rack_joints"] = prismatics

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    root_range = bbox_cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    info["bbox_min"] = tuple(root_range.GetMin())
    info["bbox_max"] = tuple(root_range.GetMax())

    if door is not None:
        door_body_path = None
        for prim in stage.Traverse():
            if str(prim.GetPath()) == door["path"]:
                targets = prim.GetRelationship("physics:body1").GetTargets()
                door_body_path = str(targets[0]) if targets else None
                # hinge anchor: localPos1 is expressed in the door-body frame
                info["hinge_local_pos"] = tuple(prim.GetAttribute("physics:localPos1").Get() or (0, 0, 0))
                break
        info["door_body_path"] = door_body_path
        if door_body_path:
            door_prim = stage.GetPrimAtPath(door_body_path)
            door_xform = UsdGeom.Xformable(door_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            door_range = bbox_cache.ComputeWorldBound(door_prim).ComputeAlignedRange()
            info["door_bbox_min"] = tuple(door_range.GetMin())
            info["door_bbox_max"] = tuple(door_range.GetMax())
            # panel length = extent above the hinge (the door is bottom-hinged and closed-vertical)
            hinge_local = Gf.Vec3d(*info.get("hinge_local_pos", (0, 0, 0)))
            hinge_asset = door_xform.Transform(hinge_local)
            info["hinge_asset_pos"] = tuple(hinge_asset)
            info["door_panel_length"] = info["door_bbox_max"][2] - hinge_asset[2]
            # handle: an Xform whose name contains "handle" under the door body
            handle_prim = next(
                (p for p in Usd.PrimRange(door_prim) if "handle" in p.GetName().lower()),
                None,
            )
            if handle_prim is not None:
                handle_range = bbox_cache.ComputeWorldBound(handle_prim).ComputeAlignedRange()
                center = (handle_range.GetMin() + handle_range.GetMax()) / 2.0
                info["handle_asset_pos"] = tuple(center)
                info["handle_in_door_frame"] = tuple(door_xform.GetInverse().Transform(Gf.Vec3d(*center)))
                info["handle_path"] = str(handle_prim.GetPath())

    # front face: the handle sits on the door, which is the front of the machine
    if "handle_asset_pos" in info:
        body_center_y = (info["bbox_min"][1] + info["bbox_max"][1]) / 2.0
        info["front_axis"] = "-Y" if info["handle_asset_pos"][1] < body_center_y else "+Y"
    else:
        info["front_axis"] = "-Y"  # fallback
    # yaw that turns the front face toward the robot at -X
    info["yaw_deg"] = -90.0 if info["front_axis"] == "-Y" else 90.0
    # front-face offset from the asset origin along the front axis
    info["front_offset"] = abs(info["bbox_min"][1] if info["front_axis"] == "-Y" else info["bbox_max"][1])
    return info


def make_preserve_drives_cfg(usd_path: str, prim_path: str) -> ArticulationCfg:
    """Dishwasher cfg that keeps the USD-authored drives (the honest 'before' picture)."""
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True,
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={".*": 0.0}),
        actuators={
            # stiffness/damping None -> inherit the USD-authored drive gains
            "all": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=None, damping=None),
        },
    )


def quat_from_yaw(yaw_deg: float) -> tuple[float, float, float, float]:
    """XYZW quaternion for a rotation about +Z."""
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def fmt(value, digits: int = 3) -> str:
    if value is None:
        return "—"
    if isinstance(value, (tuple, list)) or (torch.is_tensor(value) and value.numel() > 1):
        seq = value.tolist() if torch.is_tensor(value) else value
        return "(" + ", ".join(f"{v:.{digits}f}" for v in seq) + ")"
    if torch.is_tensor(value):
        value = value.item()
    return f"{value:.{digits}f}" if isinstance(value, float) else str(value)


def live_joint_table(articulation) -> list[str]:
    """Markdown rows for the live (post-init) joint properties of an articulation."""
    data = articulation.data
    names = articulation.joint_names
    limits = data.joint_pos_limits.torch[0]
    stiffness = data.joint_stiffness.torch[0]
    damping = data.joint_damping.torch[0]
    efforts = data.joint_effort_limits.torch[0]
    lines = ["| # | joint | limits [rad or m] | stiffness | damping | max effort |", "|---|---|---|---|---|---|"]
    for i, name in enumerate(names):
        lines.append(
            f"| {i} | `{name}` | [{limits[i, 0]:.3f}, {limits[i, 1]:.3f}] "
            f"| {stiffness[i]:.3f} | {damping[i]:.4f} | {efforts[i]:.1f} |"
        )
    return lines


def usd_joint_md(rows: list[dict]) -> list[str]:
    lines = [
        "| joint | type | axis | limits | drive k | drive d | drive target | mimic |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for j in rows:
        if j["type"] == "fixed":
            lines.append(f"| `{j['name']}` | fixed | — | — | — | — | — | — |")
            continue
        lines.append(
            f"| `{j['name']}` | {j['type']} | {j['axis']} | [{fmt(j['lower'], 2)}, {fmt(j['upper'], 2)}] {j['unit']}"
            f" | {fmt(j['stiffness'], 2)} | {fmt(j['damping'], 2)} | {fmt(j['target'], 2)} |"
            f" {'yes' if j['mimic'] else ''} |"
        )
    return lines


def main():
    dishwasher_usd = resolve_dishwasher_usd(args_cli.dishwasher)
    print(f"[INFO] Dishwasher USD: {dishwasher_usd}")
    dw_info = analyze_dishwasher_usd(dishwasher_usd)
    if dw_info["door_joint"] is None:
        raise RuntimeError("No revolute door joint found in the dishwasher USD.")
    print(f"[INFO] Door joint: {dw_info['door_joint']['name']}, racks: {[j['name'] for j in dw_info['rack_joints']]}")
    print(f"[INFO] Front axis {dw_info['front_axis']} -> yaw {dw_info['yaw_deg']} deg")

    # placement: everything is relative to the ROOT LINK frame (spawn pose = root link pose).
    # Choose the spawn position so the closed-door handle lands at HANDLE_TARGET_XY.
    yaw = math.radians(dw_info["yaw_deg"])
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    def rot2(v):
        return (cos_y * v[0] - sin_y * v[1], sin_y * v[0] + cos_y * v[1])

    root_t = dw_info["root_link_translate"]
    handle_rel = tuple(a - r for a, r in zip(dw_info["handle_asset_pos"], root_t))
    handle_rel_rot = rot2(handle_rel)
    dw_z = -(dw_info["bbox_min"][2] - root_t[2])  # ground the asset (root link z above bbox bottom)
    dw_pos = (HANDLE_TARGET_XY[0] - handle_rel_rot[0], HANDLE_TARGET_XY[1] - handle_rel_rot[1], dw_z)
    dw_rot = quat_from_yaw(dw_info["yaw_deg"])
    print(f"[INFO] Spawn pos (root link frame): {tuple(round(v, 4) for v in dw_pos)}")

    # swept-arc clearance: the fully open door tip must not reach the pedestal
    if dw_info["front_axis"] == "-Y":
        front_rel_y = dw_info["bbox_min"][1] - root_t[1]
    else:
        front_rel_y = dw_info["bbox_max"][1] - root_t[1]
    front_x = dw_pos[0] + rot2((0.0, front_rel_y))[0]
    door_len = dw_info.get("door_panel_length", 0.45)
    door_tip_open_x = front_x - door_len
    pedestal_edge_x = PEDESTAL_SIZE[0] / 2
    door_clearance = door_tip_open_x - pedestal_edge_x
    print(f"[INFO] Front face x = {front_x:.3f} m, open-door tip x = {door_tip_open_x:.3f} m, "
          f"pedestal edge x = {pedestal_edge_x:.3f} m -> clearance {door_clearance:.3f} m")

    if args_cli.preserve_drives:
        dishwasher_cfg = make_preserve_drives_cfg(dishwasher_usd, "{ENV_REGEX_NS}/Dishwasher")
    else:
        dishwasher_cfg = DISHWASHER_CFG.replace(prim_path="{ENV_REGEX_NS}/Dishwasher")
        # RL-ready derived copy: world-weld removed, door drive neutralized (utils/usd_prep.py)
        dishwasher_cfg.spawn.usd_path = make_dishwasher_rl_usd(dishwasher_usd)
    dishwasher_cfg.init_state.pos = dw_pos
    dishwasher_cfg.init_state.rot = dw_rot

    @configclass
    class InspectSceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/GroundPlane", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(
            prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        )
        pedestal = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Pedestal",
            spawn=sim_utils.CuboidCfg(
                size=PEDESTAL_SIZE,
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, PEDESTAL_SIZE[2] / 2)),
        )
        dishwasher = dishwasher_cfg

    scene_cfg = InspectSceneCfg(num_envs=1, env_spacing=3.0)

    # handle frame on the door body (offset measured in the door-prim/hinge frame)
    handle_offset = dw_info.get("handle_in_door_frame")
    if handle_offset is not None:
        scene_cfg.handle_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Dishwasher/" + dw_info["articulation_roots"][0].split("/")[-1],
            debug_vis=False,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Dishwasher/" + dw_info["door_body_path"].split("/")[-1],
                    name="door_handle",
                    # handle x out of the door (door-frame -y), y along the bar: yaw -90 deg
                    offset=OffsetCfg(pos=tuple(handle_offset), rot=(0.0, 0.0, -0.70710678, 0.70710678)),
                ),
            ],
        )

    if args_cli.with_mug:
        mug_usd = os.path.join(PROJECT_ROOT, "assets", "props", "025_mug_physics.usd")
        if not os.path.isfile(mug_usd):
            raise FileNotFoundError(f"{mug_usd} missing — run scripts/setup/make_prop_physics_usd.py --object 025_mug first.")
        scene_cfg.side_table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/SideTable",
            spawn=sim_utils.CuboidCfg(
                size=(0.35, 0.35, 0.5),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.4, 0.3, 0.2)),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, -0.6, 0.25)),
        )
        scene_cfg.mug = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Mug",
            spawn=sim_utils.UsdFileCfg(usd_path=mug_usd),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, -0.6, 0.55)),
        )

    # pin the PhysX backend explicitly (the RL task uses PhysX; the unset default is not PhysX here)
    sim = SimulationContext(sim_utils.SimulationCfg(device=args_cli.device, physics=PhysxCfg()))
    sim.set_camera_view([1.8, -1.8, 1.2], [0.4, 0.0, 0.4])
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    print("[INFO] Scene initialized.")

    dishwasher = scene["dishwasher"]
    sim_dt = sim.get_physics_dt()

    # standalone scripts must write the default states themselves (in the RL workflow the event
    # manager's reset_scene_to_default does this); also hold the default pose with the PD drives
    for articulation in (dishwasher,):
        root_pose = articulation.data.default_root_pose.torch.clone()
        root_pose[:, :3] += scene.env_origins
        articulation.write_root_pose_to_sim_index(root_pose=root_pose)
        articulation.write_root_velocity_to_sim_index(root_velocity=articulation.data.default_root_vel.torch.clone())
        articulation.write_joint_position_to_sim_index(position=articulation.data.default_joint_pos.torch.clone())
        articulation.write_joint_velocity_to_sim_index(velocity=articulation.data.default_joint_vel.torch.clone())
        articulation.set_joint_position_target_index(target=articulation.data.default_joint_pos.torch.clone())
    scene.reset()

    door_ids, _ = dishwasher.find_joints(dw_info["door_joint"]["name"])
    door_id = door_ids[0]

    # -- stability run ---------------------------------------------------------------------
    steps = args_cli.steps
    scene.update(sim_dt)
    door_angles, max_vels = [], []
    nan_found = False
    for _ in range(steps):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        door_angles.append(float(dishwasher.data.joint_pos.torch[0, door_id]))
        vel_d = dishwasher.data.joint_vel.torch[0]
        max_vels.append(float(vel_d.abs().max()))
        if not torch.isfinite(vel_d).all():
            nan_found = True
            break

    door_final_deg = math.degrees(door_angles[-1]) if door_angles else float("nan")
    tail_vel = max(max_vels[-50:]) if len(max_vels) >= 50 else max(max_vels, default=float("nan"))

    stability = {
        "steps": len(door_angles),
        "nan": nan_found,
        "door_final_deg": door_final_deg,
        "door_max_deg": math.degrees(max(door_angles, default=float("nan"))),
        "tail_max_abs_joint_vel": tail_vel,
    }
    stability_ok = (not nan_found) and tail_vel < 0.5 and abs(door_final_deg) < 5.0
    print(f"[INFO] Stability: {stability} -> {'PASS' if stability_ok else 'FAIL'}")

    # -- geometry measurements (env 0, after settling so all buffers are live) --------------
    # handle: FrameTransformer output vs. ground truth from asset geometry + spawn transform
    handle_world = None
    handle_true = None
    if "handle_frame" in scene.keys():
        handle_world = scene["handle_frame"].data.target_pos_w.torch[0, 0]
    if "handle_asset_pos" in dw_info:
        hx, hy = rot2(handle_rel[:2])
        ref = dishwasher.data.joint_pos.torch
        handle_true = torch.tensor(
            [dw_pos[0] + hx, dw_pos[1] + hy, dw_pos[2] + handle_rel[2]],
            dtype=ref.dtype,
            device=ref.device,
        )
    handle_frame_err = (
        float(torch.linalg.norm(handle_world - handle_true))
        if handle_world is not None and handle_true is not None
        else None
    )

    # -- door test (passive config only) ---------------------------------------------------
    door_test = None
    if args_cli.test_door and not args_cli.preserve_drives:
        door_test = {}
        # 1) covered by the stability run: door should have stayed closed unaided
        door_test["closed_unaided_deg"] = door_final_deg
        door_test["closed_unaided_pass"] = abs(door_final_deg) < 5.0 and not nan_found
        # 2) opens under commanded effort
        effort = torch.full((1, 1), 8.0, device=dishwasher.data.joint_pos.torch.device)
        opened_deg = 0.0
        for _ in range(120):
            dishwasher.set_joint_effort_target_index(target=effort, joint_ids=[door_id])
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim_dt)
            opened_deg = math.degrees(float(dishwasher.data.joint_pos.torch[0, door_id]))
        door_test["opened_deg_after_effort"] = opened_deg
        door_test["opens_pass"] = opened_deg > 30.0
        # 3) rests without oscillation once effort is removed
        zero = torch.zeros((1, 1), device=effort.device)
        for _ in range(300):
            dishwasher.set_joint_effort_target_index(target=zero, joint_ids=[door_id])
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim_dt)
        rest_deg = math.degrees(float(dishwasher.data.joint_pos.torch[0, door_id]))
        rest_vel = float(dishwasher.data.joint_vel.torch[0, door_id].abs())
        door_test["rest_deg"] = rest_deg
        door_test["rest_vel"] = rest_vel
        door_test["rests_pass"] = -1.0 <= rest_deg <= 91.0 and rest_vel < 0.05
        print(f"[INFO] Door test: {door_test}")

    # -- write report ----------------------------------------------------------------------
    mode = "USD drives preserved" if args_cli.preserve_drives else "passive-door RL config"
    lines = ["# Joint report", ""]
    lines.append(f"Generated by `scripts/setup/inspect_scene.py` (variant `{args_cli.dishwasher}`, mode: {mode}).")
    lines += ["", f"- Dishwasher USD: `{os.path.relpath(dishwasher_usd, PROJECT_ROOT)}`"]

    lines += ["", "## Dishwasher — USD-authored joints (as shipped)", ""]
    lines += usd_joint_md(dw_info["joints"])
    lines += [
        "",
        "- Note: USD angular drive gains are authored per-degree; the live table below shows the",
        "  per-radian values PhysX actually uses (×57.3). The as-shipped door drive is a stiff",
        "  spring toward 90 deg — the passive config zeroes it (see `src/dishsim/machine.py`).",
        f"- Articulation root(s): `{', '.join(dw_info['articulation_roots'])}`",
        f"- **Door joint: `{dw_info['door_joint']['name']}`** (body `{dw_info.get('door_body_path')}`), "
        f"limits [{dw_info['door_joint']['lower']:.0f}, {dw_info['door_joint']['upper']:.0f}] deg",
        f"- **Rack joints: {', '.join('`' + j['name'] + '`' for j in dw_info['rack_joints'])}**",
        f"- Handle prim: `{dw_info.get('handle_path', 'n/a')}`, center in door-body frame: "
        f"{fmt(dw_info.get('handle_in_door_frame'))} m",
        f"- Hinge anchor in door-body frame: {fmt(dw_info.get('hinge_local_pos'))} m; "
        f"asset-frame position: {fmt(dw_info.get('hinge_asset_pos'))} m (front-bottom hinge)",
        f"- Door panel length above hinge: {fmt(dw_info.get('door_panel_length'))} m",
        f"- Asset bbox: {fmt(dw_info['bbox_min'])} — {fmt(dw_info['bbox_max'])} m; "
        f"front axis {dw_info['front_axis']}; spawn yaw {dw_info['yaw_deg']:.0f} deg; spawn pos {fmt(dw_pos)}",
        f"- Swept-arc check: open-door tip at x = {door_tip_open_x:.3f} m vs pedestal edge at "
        f"x = {pedestal_edge_x:.3f} m -> clearance {door_clearance:.3f} m "
        f"({'OK' if door_clearance > 0.03 else 'TOO TIGHT'})",
        f"- Collision approximations: {dw_info['collision']}",
    ]

    lines += ["", f"## Dishwasher — live articulation (post-init, {mode})", ""]
    lines += live_joint_table(dishwasher)
    lines += [
        "",
        f"- Body names: {dishwasher.body_names}",
        "",
        "### Handle frame (env-style FrameTransformer)",
        "",
        f"- Handle frame (world, door closed): {fmt(handle_world)} m; ground truth from asset geometry: "
        f"{fmt(handle_true)} m; error: {fmt(handle_frame_err, 4)} m",
    ]

    lines += ["", "## Stability check", ""]
    for key, value in stability.items():
        lines.append(f"- {key}: {fmt(value, 4)}")
    if args_cli.preserve_drives:
        lines.append("- result: informational only (`--preserve_drives` keeps the as-shipped door "
                     "drive, which is expected to move the door and may destabilize the sim)")
    else:
        lines.append(f"- **result: {'PASS' if stability_ok else 'FAIL'}**")

    if door_test is not None:
        lines += ["", "## Door test (passive RL config)", ""]
        lines += [
            f"- stays closed unaided: {door_test['closed_unaided_deg']:.2f} deg after {steps} steps -> "
            f"{'PASS' if door_test['closed_unaided_pass'] else 'FAIL'}",
            f"- opens under 8 N·m for 2 s: {door_test['opened_deg_after_effort']:.1f} deg -> "
            f"{'PASS' if door_test['opens_pass'] else 'FAIL'}",
            f"- rests after release: {door_test['rest_deg']:.1f} deg at {door_test['rest_vel']:.4f} rad/s -> "
            f"{'PASS' if door_test['rests_pass'] else 'FAIL'}",
        ]

    os.makedirs(os.path.dirname(args_cli.report), exist_ok=True)
    with open(args_cli.report, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[INFO] Report written to {args_cli.report}")

    # -- final verdict ---------------------------------------------------------------------
    if args_cli.preserve_drives:
        print("[RESULT] INFORMATIONAL (no assertions in --preserve_drives mode)")
        return
    ok = stability_ok if door_test is None else (
        stability_ok
        and door_test["closed_unaided_pass"]
        and door_test["opens_pass"]
        and door_test["rests_pass"]
    )
    print(f"[RESULT] {'PASS' if ok else 'FAIL'}")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
    simulation_app.close()
