# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Placement slots in the lower rack + IK goal-configuration generation (Kit-free).

Reality check baked in: the ArtVIP ``dishwasher_2`` lower rack (``E_shelf_1_04``) is a shallow
**wire basket** (~5 cm deep grid of wires), not a plate rack with tall tines — measured in
Phase D's decomposition overlays. The carried mug therefore *stands on the wire floor*; a
"slot" is a standing position on a grid derived from the basket geometry (footprint-sized
cells inset from the rim). The derivation is geometric and recorded per slot (``source``
field); tolerances come from :mod:`dishsim.config`.

Everything here runs in the plain venv (numpy + trimesh + the FCL world); the only Isaac use
is the Kit pass in ``scripts/15_goal_configs.py`` that renders contact sheets.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np
import trimesh

from . import config
from .geometry import load_manifest
from .transforms import T_inv, make_T
from .ur5e_kin import JOINT_LIMITS, expand_2pi_wraps, ik_wrist3_all

# object-frame constants (measured, see config): the mug axis runs along +y_obj through
# (x, z) = OBJECT_BODY_CENTER_XZ; origin is the bbox center
_AXIS_XZ = np.array(config.OBJECT_BODY_CENTER_XZ)
_HALF_H = config.OBJECT_HEIGHT_M / 2.0


@dataclass
class SlotFrame:
    """A standing slot on the lower-rack wire floor, in the robot-base frame."""

    slot_id: int
    T_base_slot: np.ndarray  # [4, 4]; origin ON the wire floor at the slot center, z up
    width_m: float  # usable footprint width (square cell edge)
    source: str  # "derived" | "manual"

    def to_json(self) -> dict:
        return {
            "slot_id": self.slot_id,
            "T_base_slot": self.T_base_slot.tolist(),
            "width_m": self.width_m,
            "source": self.source,
        }

    @staticmethod
    def from_json(d: dict) -> "SlotFrame":
        return SlotFrame(d["slot_id"], np.array(d["T_base_slot"]), d["width_m"], d["source"])


@dataclass
class GoalSet:
    """IK goal configurations for one slot, with the rejection funnel (scarcity is signal)."""

    slot_id: int
    configs: np.ndarray  # [K, 6] (possibly K == 0)
    n_pose_samples: int = 0
    n_ik_solutions: int = 0
    n_limit_reject: int = 0
    n_collision_reject: int = 0

    def to_json(self) -> dict:
        return {
            "slot_id": self.slot_id,
            "configs": np.asarray(self.configs).tolist(),
            "funnel": {
                "pose_samples": self.n_pose_samples,
                "ik_solutions": self.n_ik_solutions,
                "limit_reject": self.n_limit_reject,
                "collision_reject": self.n_collision_reject,
                "accepted": int(len(self.configs)),
            },
        }


def derive_slots_from_rack(cache_dir: str = config.CACHE_DIR) -> list[SlotFrame]:
    """Grid of standing slots on the lower-rack wire floor, from the cached rack geometry.

    Method: load the rack body mesh, take its bbox in the (axis-aligned) body frame, inset by
    ``SLOT_RIM_INSET_M`` for the rim walls, find the wire-floor top plane (low-percentile
    z-vertices + wire diameter), and tile the interior with footprint-sized cells. Slot frames
    inherit the rack body's orientation (the rack is axis-aligned with the machine; z stays up
    in the base frame — asserted).

    Returns:
        Ordered slots (row-major over the grid), all ``source="derived"``.
    """
    manifest = load_manifest(cache_dir)
    entry = manifest["statics"]["E_shelf_1_04"]
    mesh = trimesh.load(os.path.join(cache_dir, entry["mesh"]), force="mesh")
    T_base_rack = np.array(entry["T_base_body"])

    mn, mx = mesh.bounds  # rack BODY frame
    # wire-floor top: the base wires live in the bottom slab of the basket
    verts = mesh.vertices
    bottom = verts[verts[:, 2] < mn[2] + 0.015]
    floor_top_z = float(np.percentile(bottom[:, 2], 95))

    footprint = 2.0 * max(config.OBJECT_BBOX_HALF[0], config.OBJECT_BBOX_HALF[2])
    pitch = config.SLOT_GRID_PITCH_M
    lo = mn[:2] + config.SLOT_RIM_INSET_M
    hi = mx[:2] - config.SLOT_RIM_INSET_M
    nx = max(1, int((hi[0] - lo[0]) // pitch) + 1)
    ny = max(1, int((hi[1] - lo[1]) // pitch) + 1)
    xs = lo[0] + (hi[0] - lo[0] - (nx - 1) * pitch) / 2.0 + np.arange(nx) * pitch
    ys = lo[1] + (hi[1] - lo[1] - (ny - 1) * pitch) / 2.0 + np.arange(ny) * pitch

    slots = []
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            T_rack_slot = np.eye(4)
            T_rack_slot[:3, 3] = (x, y, floor_top_z)
            T_base_slot = T_base_rack @ T_rack_slot
            # frame sanity: slot z must stay "up" in the base frame (the rack is level)
            assert T_base_slot[2, 2] > 0.99, f"slot frame tilted: z-axis {T_base_slot[:3, 2]}"
            slots.append(
                SlotFrame(slot_id=len(slots), T_base_slot=T_base_slot, width_m=footprint, source="derived")
            )
    return slots


def object_pose_for_slot(slot: SlotFrame, yaw: float, lateral: np.ndarray, tilt: np.ndarray, hover: float) -> np.ndarray:
    """Object (mug) pose standing at a slot: axis up (+tilt), bottom ``hover`` above the floor.

    Args:
        slot: Target slot.
        yaw: Rotation of the object about the slot z-axis [rad].
        lateral: Lateral offset of the axis footprint from the slot center, shape [2] [m].
        tilt: Small tilt angles about slot x/y, shape [2] [rad].
        hover: Bottom clearance above the wire floor [m].

    Returns:
        T_base_obj, shape [4, 4].
    """
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    # object frame -> "standing" frame: y_obj (mug axis) -> +z, then yaw about z.
    # NOTE Rx(+90) maps +y to +z; the -90 variant stands the mug on its head 8 cm underground
    # and sends the wrist below the floor (cost one full debugging cycle — leave the sign be).
    R_stand = Rotation.from_euler("x", np.pi / 2).as_matrix()  # maps +y_obj to +z
    R = (
        Rotation.from_euler("xy", tilt).as_matrix()
        @ Rotation.from_euler("z", yaw).as_matrix()
        @ R_stand
    )
    # axis point at the mug BOTTOM (obj frame): (axis_x, -half_h_along_axis, axis_z)
    p_bottom_obj = np.array([_AXIS_XZ[0], -config.OBJECT_BBOX_HALF[1], _AXIS_XZ[1]])
    T = np.eye(4)
    T[:3, :3] = R
    # place the (rotated) bottom axis point at slot center + lateral, hover above the floor
    bottom_target = slot.T_base_slot[:3, 3] + slot.T_base_slot[:3, :3] @ np.array(
        [lateral[0], lateral[1], hover]
    )
    T[:3, 3] = bottom_target - R @ p_bottom_obj
    return T


def sample_goal_poses(slot: SlotFrame, n: int, rng: np.random.Generator) -> list[np.ndarray]:
    """Sample object poses in the slot's tolerance region (free yaw, small lateral/tilt)."""
    poses = []
    for _ in range(n):
        yaw = rng.uniform(0.0, 2.0 * np.pi)
        lateral = rng.uniform(-config.SLOT_TOL_LATERAL_M, config.SLOT_TOL_LATERAL_M, 2) * 0.66
        tilt = rng.uniform(-1.0, 1.0, 2) * np.radians(config.SLOT_TOL_TILT_DEG) * 0.3
        poses.append(object_pose_for_slot(slot, yaw, lateral, tilt, config.RELEASE_HOVER_M))
    return poses


def goal_configs(
    slot: SlotFrame,
    world,
    rng: np.random.Generator,
    n_samples: int = config.GOAL_POSE_SAMPLES_PER_SLOT,
    limit_margin: float = config.PLAN_JOINT_BOUNDS_MARGIN_RAD,
) -> GoalSet:
    """All valid IK goal configurations for a slot (pose samples x IK branches x wraps).

    Pipeline per pose sample: T_base_wrist3 = T_base_obj . inv(T_wrist3_obj) -> all analytic
    IK branches -> +-2pi wrap expansion -> joint-limit filter -> collision filter (attached
    object included, self-check on). Zero surviving configs is signal, not error — the funnel
    counts say which stage killed a slot.
    """
    manifest = world.manifest
    T_w3_obj = np.array(manifest["object"]["T_wrist3_obj"])
    T_obj_w3 = T_inv(T_w3_obj)
    limits = JOINT_LIMITS.copy()
    limits[:, 0] += limit_margin
    limits[:, 1] -= limit_margin

    gs = GoalSet(slot_id=slot.slot_id, configs=np.zeros((0, 6)))
    accepted: list[np.ndarray] = []
    for T_base_obj in sample_goal_poses(slot, n_samples, rng):
        gs.n_pose_samples += 1
        sols = ik_wrist3_all(T_base_obj @ T_obj_w3)
        gs.n_ik_solutions += len(sols)
        if len(sols) == 0:
            continue
        expanded = expand_2pi_wraps(sols, limits=JOINT_LIMITS, margin=limit_margin)
        for q in expanded:
            if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
                gs.n_limit_reject += 1
                continue
            if world.in_collision(q):
                gs.n_collision_reject += 1
                continue
            accepted.append(q)
    if accepted:
        # deduplicate near-identical configs
        keep: list[np.ndarray] = []
        for q in accepted:
            if not any(np.max(np.abs(q - k)) < 1e-3 for k in keep):
                keep.append(q)
        gs.configs = np.array(keep)
    return gs


def save_slots(slots: list[SlotFrame], goal_sets: list[GoalSet], out_dir: str) -> tuple[str, str]:
    """Write slots.json + goal_sets.json under ``out_dir`` (usually assets/cache/slots)."""
    os.makedirs(out_dir, exist_ok=True)
    slots_path = os.path.join(out_dir, "slots.json")
    with open(slots_path, "w") as f:
        json.dump({"frame": "robot_base", "slots": [s.to_json() for s in slots]}, f, indent=2)
    goals_path = os.path.join(out_dir, "goal_sets.json")
    with open(goals_path, "w") as f:
        json.dump({"goal_sets": [g.to_json() for g in goal_sets]}, f, indent=2)
    return slots_path, goals_path
