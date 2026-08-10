# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Placement slots in the lower rack + IK goal-configuration generation (Kit-free).

Reality check baked in: the ArtVIP ``dishwasher_2`` lower rack (``E_shelf_1_04``) is a shallow
**wire basket** (~5 cm deep grid of wires), not a plate rack with tall tines — measured in
the decomposition overlays. The carried mug therefore *stands on the wire floor*; a
"slot" is a standing position on a grid derived from the basket geometry (footprint-sized
cells inset from the rim). The derivation is geometric and recorded per slot (``source``
field); tolerances come from :mod:`dishsim.config`.

Everything here runs in the plain venv (numpy + trimesh + the FCL world); the only Isaac use
is the Kit pass in ``scripts/setup/goal_configs.py`` that renders contact sheets.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np
import trimesh

from . import config
from .geometry import config_hash, load_manifest
from .transforms import T_inv, make_T
from .ur5e_kin import JOINT_LIMITS, expand_2pi_wraps, ik_wrist3_all

# Object-frame values (measured, see config) are read from `config` at CALL time, never bound
# at import — set_active_object() rewrites the active-object view in place, and an import-time
# copy here would silently keep the previous object's geometry.


@dataclass
class SlotFrame:
    """A placement slot in the lower rack, in the robot-base frame.

    ``mode`` selects the goal-pose generator and success criteria: ``floor_stand`` (origin ON
    the wire floor, z up — the v0 standing cell), ``plate_slot`` (origin at the disc bottom
    edge between two tines), ``basket_drop`` (origin at the basket-bay floor center; goals
    hover above it).
    """

    slot_id: int
    T_base_slot: np.ndarray  # [4, 4]; origin per mode (see class docstring), z up
    width_m: float  # usable footprint width (square cell edge / gap width / bay width)
    source: str  # "derived" | "manual"
    mode: str = "floor_stand"
    rack: str = "lower"
    params: dict = field(default_factory=dict)
    #: Geometry-derived label (see :func:`slot_names`). Carried so a record can report WHICH
    #: slot an object went to in the terms config uses, while ``slot_id`` stays the int that
    #: metrics aggregation sorts on.
    name: str | None = None

    def to_json(self) -> dict:
        return {
            "slot_id": self.slot_id,
            "T_base_slot": self.T_base_slot.tolist(),
            "width_m": self.width_m,
            "source": self.source,
            "mode": self.mode,
            "rack": self.rack,
            "params": self.params,
            "name": self.name,
        }

    @staticmethod
    def from_json(d: dict) -> "SlotFrame":
        return SlotFrame(
            d["slot_id"],
            np.array(d["T_base_slot"]),
            d["width_m"],
            d["source"],
            mode=d.get("mode", "floor_stand"),
            rack=d.get("rack", "lower"),
            params=d.get("params", {}),
            name=d.get("name"),
        )


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

    # standing footprint = the two axis-perpendicular half extents (Y-up mug: x/z; Z-up: x/y)
    if tuple(config.OBJECT_AXIS_OBJ) == (0.0, 1.0, 0.0):
        footprint = 2.0 * max(config.OBJECT_BBOX_HALF[0], config.OBJECT_BBOX_HALF[2])
    else:
        footprint = 2.0 * max(config.OBJECT_BBOX_HALF[0], config.OBJECT_BBOX_HALF[1])
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

    # object frame -> "standing" frame (axis-aware): the Y-up mug stands via Rx(+90) — the
    # -90 variant stands it on its head 8 cm underground (cost one full debugging cycle);
    # Z-up objects stand as authored. Then yaw about z.
    R_stand = _spec_R_stand()
    R = (
        Rotation.from_euler("xy", tilt).as_matrix()
        @ Rotation.from_euler("z", yaw).as_matrix()
        @ R_stand
    )
    # axis point at the object BOTTOM (obj frame), per the axis convention
    axis_uv = config.OBJECT_BODY_CENTER_XZ
    if tuple(config.OBJECT_AXIS_OBJ) == (0.0, 1.0, 0.0):
        p_bottom_obj = np.array([axis_uv[0], -config.OBJECT_BBOX_HALF[1], axis_uv[1]])
    else:
        p_bottom_obj = np.array([axis_uv[0], axis_uv[1], -config.OBJECT_BBOX_HALF[2]])
    T = np.eye(4)
    T[:3, :3] = R
    # place the (rotated) bottom axis point at slot center + lateral, hover above the floor
    bottom_target = slot.T_base_slot[:3, 3] + slot.T_base_slot[:3, :3] @ np.array(
        [lateral[0], lateral[1], hover]
    )
    T[:3, 3] = bottom_target - R @ p_bottom_obj
    return T


# ---------------------------------------------------------------------------------------------
# mode-aware slot derivation + goal poses (multi-object v1)
# ---------------------------------------------------------------------------------------------


def derive_slots(cache_dir: str = config.CACHE_DIR) -> list[SlotFrame]:
    """Slots for the ACTIVE object's placement mode (dispatch; ids are mode-local)."""
    mode = config.active_object_spec().placement.mode
    if mode == "floor_stand":
        return derive_slots_from_rack(cache_dir)
    if mode == "plate_slot":
        return derive_plate_slots(cache_dir)
    if mode == "basket_drop":
        return derive_basket_slots(cache_dir)
    raise ValueError(f"no robot slot derivation for placement mode {mode!r}")


def _rack_T(cache_dir: str) -> np.ndarray:
    manifest = load_manifest(cache_dir)
    return np.array(manifest["statics"]["E_shelf_1_04"]["T_base_body"])


def derive_plate_slots(cache_dir: str = config.CACHE_DIR) -> list[SlotFrame]:
    """One slot per tine-bank gap (11 gaps; goal-config collision filtering prunes the ones
    over the cutlery basket). Slot origin = the disc's bottom-edge target on the bank floor;
    slot z up, slot x along the rack pitch direction."""
    from . import rack_gen  # noqa: PLC0415

    p = config.RACK_GEN["E_shelf_1_04"]
    T_base_rack = _rack_T(cache_dir)
    xs = np.asarray(rack_gen._plate_tine_xs(p))
    gaps = (xs[:-1] + xs[1:]) / 2.0
    z_floor = rack_gen.floor_top_z(p)
    lean = float(p["plate_tine_lean_deg"])
    mp = config.placement_mode_params("plate_slot")
    y_bottom = float(config.active_object_spec().placement.params.get("seat_y_m", mp["seat_y_m"]))
    slots = []
    for gi, gx in enumerate(gaps):
        T_rack_slot = np.eye(4)
        T_rack_slot[:3, 3] = (float(gx), y_bottom, z_floor)
        slots.append(
            SlotFrame(
                slot_id=gi,
                T_base_slot=T_base_rack @ T_rack_slot,
                width_m=float(p["plate_tine_pitch"]),
                source="derived",
                mode="plate_slot",
                rack="lower",
                params={"lean_deg": lean},
            )
        )
    return slots


def derive_basket_slots(cache_dir: str = config.CACHE_DIR) -> list[SlotFrame]:
    """One slot per cutlery-basket bay (origin at the bay floor center)."""
    from . import rack_gen  # noqa: PLC0415

    p = config.RACK_GEN["E_shelf_1_04"]
    T_base_rack = _rack_T(cache_dir)
    b = p["basket"]
    z_bfloor = rack_gen.floor_top_z(p) + b["floor_t"]
    mp = config.placement_mode_params("basket_drop")
    dx = float(mp["drop_x_offset_m"])
    slots = []
    for bi, (x0, x1, y0, y1) in enumerate(rack_gen.basket_bays(p)):
        T_rack_slot = np.eye(4)
        T_rack_slot[:3, 3] = ((x0 + x1) / 2.0 + dx, (y0 + y1) / 2.0, z_bfloor)
        slots.append(
            SlotFrame(
                slot_id=bi,
                T_base_slot=T_base_rack @ T_rack_slot,
                width_m=float(x1 - x0),
                source="derived",
                mode="basket_drop",
                rack="basket",
                params={"bay": [x0, x1, y0, y1], "top_z": float(b["h"] - b["floor_t"])},
            )
        )
    return slots


def _spec_R_stand() -> np.ndarray:
    """Rotation standing the ACTIVE object's axis up (design/base z)."""
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    axis = tuple(config.OBJECT_AXIS_OBJ)
    if axis == (0.0, 1.0, 0.0):
        return Rotation.from_euler("x", np.pi / 2).as_matrix()  # the mug convention
    if axis == (0.0, 0.0, 1.0):
        return np.eye(3)
    raise ValueError(f"floor_stand unsupported for axis {axis}")


def object_pose_for_mode(slot: SlotFrame, spin: float, lateral: np.ndarray, tilt: np.ndarray, hover: float) -> np.ndarray:
    """Object goal pose at a slot, per the slot's mode (generalizes the v0 standing pose).

    Args:
        slot: Target slot.
        spin: Free rotation about the object's own axis [rad].
        lateral: Lateral offset from the slot origin, shape [2] [m] (mode-scaled).
        tilt: Small perturbation angles, shape [2] [rad].
        hover: Release clearance [m].

    Returns:
        T_base_obj, shape [4, 4].
    """
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    if slot.mode == "floor_stand":
        return object_pose_for_slot(slot, spin, lateral, tilt, hover)

    R_slot = slot.T_base_slot[:3, :3]
    spec = config.active_object_spec()
    mp = config.placement_mode_params(slot.mode)
    if slot.mode == "plate_slot":
        lean = np.radians(slot.params.get("lean_deg", mp["default_lean_deg"]))
        r = spec.rim_radius_m
        # disc face normal along slot x, leaned toward slot +y like the tines; spin about the
        # disc normal is free; tiny lateral x inside the gap
        R_local = (
            Rotation.from_euler("x", -(lean + tilt[0])).as_matrix()
            @ Rotation.from_euler("y", np.pi / 2).as_matrix()
            @ Rotation.from_euler("z", spin).as_matrix()
        )
        t_local = np.array(
            [lateral[0] * 0.25, np.sin(lean) * r + lateral[1] * 0.25, hover + np.cos(lean) * r]
        )
    elif slot.mode == "basket_drop":
        # cutlery hangs head-DOWN below the gripper (grasped at the handle = top), released
        # high above the bay so gravity inserts it
        R_local = (
            Rotation.from_euler("z", tilt[0] * 0.5).as_matrix()
            @ Rotation.from_euler("y", np.pi / 2).as_matrix()  # +x_obj (head) -> -z
            @ Rotation.from_euler("x", spin * 0.05).as_matrix()
        )
        t_local = np.array(
            [lateral[0] * 0.3, lateral[1] * 0.3, hover + spec.height_m / 2.0]
        )
    else:
        raise ValueError(f"no goal-pose generator for mode {slot.mode!r}")

    T = np.eye(4)
    T[:3, :3] = R_slot @ R_local
    T[:3, 3] = slot.T_base_slot[:3, 3] + R_slot @ t_local
    return T


def _top_grasp_spin(slot: SlotFrame) -> float | None:
    """Spin putting the grasped feature at the TOP of the placed object (or None if free).

    For plate_slot the grasp point is a fixed rim location in the object frame and
    the spin rotates it around the rim: only spins with the gripper ABOVE the object are
    reachable (any other spin buries the wrist in the rack — measured 0/11 feasible slots
    with uniform spin). Solved numerically: argmax of the grasp point's world z over spin.
    """
    if slot.mode != "plate_slot":
        return None
    pos, _ = config.grasp_transform(config.active_object_spec())
    # the grasped feature in the OBJECT frame: invert the tcp<-obj transform's grasp point.
    # For edge_pinch it is the rim point on +x_obj; for rim_edge the rim point on +y_obj.
    spec = config.active_object_spec()
    u, v = spec.body_center_uv
    if spec.grasp.family == "edge_pinch":
        p_g = np.array([u + spec.rim_radius_m, v, 0.0])
    else:
        p_g = np.array([u, v + spec.rim_radius_m, spec.bbox_half[2]])
    spins = np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False)
    zs = []
    for s in spins:
        T = object_pose_for_mode(slot, s, np.zeros(2), np.zeros(2), config.RELEASE_HOVER_M)
        zs.append((T @ np.append(p_g, 1.0))[2])
    return float(spins[int(np.argmax(zs))])


def sample_goal_poses(slot: SlotFrame, n: int, rng: np.random.Generator) -> list[np.ndarray]:
    """Sample object poses in the slot's tolerance region (spin, small lateral/tilt).

    The spin is uniform for standing/basket modes; for plate_slot it concentrates
    in a +-0.5 rad window around the top-grasp spin (see :func:`_top_grasp_spin`).
    """
    poses = []
    hover = float(config.placement_mode_params(slot.mode)["release_hover_m"])
    spin0 = _top_grasp_spin(slot)
    for _ in range(n):
        if spin0 is None:
            spin = rng.uniform(0.0, 2.0 * np.pi)
        else:
            spin = spin0 + rng.uniform(-0.5, 0.5)
        lateral = rng.uniform(-config.SLOT_TOL_LATERAL_M, config.SLOT_TOL_LATERAL_M, 2) * 0.66
        tilt = rng.uniform(-1.0, 1.0, 2) * np.radians(config.SLOT_TOL_TILT_DEG) * 0.3
        poses.append(object_pose_for_mode(slot, spin, lateral, tilt, hover))
    return poses


def goal_configs(
    slot: SlotFrame,
    world,
    rng: np.random.Generator,
    n_samples: int = config.GOAL_POSE_SAMPLES_PER_SLOT,
    limit_margin: float = config.PLAN_JOINT_BOUNDS_MARGIN_RAD,
    early_exit_accepted: int | None = None,
) -> GoalSet:
    """All valid IK goal configurations for a slot (pose samples x IK branches x wraps).

    Pipeline per pose sample: T_base_wrist3 = T_base_obj . inv(T_wrist3_obj) -> all analytic
    IK branches -> +-2pi wrap expansion -> joint-limit filter -> collision filter (attached
    object included, self-check on). Zero surviving configs is signal, not error — the funnel
    counts say which stage killed a slot.

    Args:
        early_exit_accepted: Stop sampling once this many configurations are accepted (the
            base-pose sweep asks "is this slot feasible", not "enumerate its goals"). None —
            the default, used by the goal bake — keeps the full sample budget, so baked
            funnels are unchanged.
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
        if early_exit_accepted is not None and len(accepted) >= early_exit_accepted:
            break
    if accepted:
        # deduplicate near-identical configs
        keep: list[np.ndarray] = []
        for q in accepted:
            if not any(np.max(np.abs(q - k)) < 1e-3 for k in keep):
                keep.append(q)
        gs.configs = np.array(keep)
    return gs


def save_slots(slots: list[SlotFrame], goal_sets: list[GoalSet], out_dir: str) -> tuple[str, str]:
    """Write slots.json + goal_sets.json under ``out_dir`` (usually the scenario's cache slots dir).

    Both files are stamped with the active scenario and the collision-world ``config_hash`` so
    consumers can detect a manifest/slots mismatch (legacy files without stamps still load).
    """
    os.makedirs(out_dir, exist_ok=True)
    stamp = {"scenario": config.SCENARIO_NAME, "config_hash": config_hash()}
    slots_path = os.path.join(out_dir, "slots.json")
    with open(slots_path, "w") as f:
        json.dump({"frame": "robot_base", **stamp, "slots": [s.to_json() for s in slots]}, f, indent=2)
    goals_path = os.path.join(out_dir, "goal_sets.json")
    with open(goals_path, "w") as f:
        json.dump({**stamp, "goal_sets": [g.to_json() for g in goal_sets]}, f, indent=2)
    return slots_path, goals_path


def evaluate_placement(slot: SlotFrame, T_base_obj: np.ndarray) -> dict:
    """Per-mode success evaluation of a settled object pose (physics-backed).

    Returns ``{"lateral_m", "tilt_deg", "bottom_height_m", "ok"}``; the tilt reference and
    tolerances depend on the slot mode (documented in docs/success_criteria.md).
    """
    spec = config.active_object_spec()
    axis_obj = np.array(spec.axis_obj)
    axis_base = T_base_obj[:3, :3] @ axis_obj
    R_slot = slot.T_base_slot[:3, :3]
    p_slot = slot.T_base_slot[:3, 3]
    mp = config.placement_mode_params(slot.mode)

    if slot.mode == "floor_stand":
        # v0 criteria: bottom axis point near the slot center, axis near slot z
        if tuple(spec.axis_obj) == (0.0, 1.0, 0.0):
            u, v = config.OBJECT_BODY_CENTER_XZ
            p_bottom_obj = np.array([u, -spec.bbox_half[1], v])
        else:
            u, v = config.OBJECT_BODY_CENTER_XZ
            p_bottom_obj = np.array([u, v, -spec.bbox_half[2]])
        p_bottom = (T_base_obj @ np.append(p_bottom_obj, 1.0))[:3]
        d = R_slot.T @ (p_bottom - p_slot)
        lateral = float(np.hypot(d[0], d[1]))
        tilt = float(np.degrees(np.arccos(np.clip(axis_base @ R_slot[:, 2], -1.0, 1.0))))
        ok = (lateral <= mp["tol_lateral_m"] and tilt <= mp["tol_tilt_deg"]
               and abs(d[2]) <= mp["tol_bottom_m"])
        return {"lateral_m": round(lateral, 4), "tilt_deg": round(tilt, 2), "bottom_height_m": round(float(d[2]), 4), "ok": bool(ok)}

    d = R_slot.T @ (T_base_obj[:3, 3] - p_slot)
    if slot.mode == "plate_slot":
        lean = np.radians(slot.params.get("lean_deg", mp["default_lean_deg"]))
        # target disc normal (slot frame): x leaned by the tine angle; flip-insensitive
        n_target = R_slot @ np.array([np.cos(0.0), 0.0, 0.0])
        cosang = abs(float(axis_base @ n_target))
        tilt = float(np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))))
        lateral = abs(float(d[0]))  # off-gap drift along the pitch direction
        bottom = float(d[2] - spec.rim_radius_m * np.cos(lean))  # center height above nominal
        ok = (lateral <= mp["tol_lateral_m"] and tilt <= mp["tol_tilt_deg"]
               and abs(bottom) <= mp["tol_bottom_m"])
        return {"lateral_m": round(lateral, 4), "tilt_deg": round(tilt, 2), "bottom_height_m": round(bottom, 4), "ok": bool(ok)}

    if slot.mode == "basket_drop":
        bay = slot.params.get("bay")
        top_z = slot.params.get("top_z", mp["default_top_z_m"])
        # success = the settled bbox CENTER is inside the bay volume, below the basket top
        inside_xy = abs(d[0]) <= slot.width_m / 2.0 and abs(d[1]) <= 0.035
        if bay is not None:
            half_x = (bay[1] - bay[0]) / 2.0
            half_y = (bay[3] - bay[2]) / 2.0
            inside_xy = abs(d[0]) <= half_x and abs(d[1]) <= half_y + mp["tol_lateral_pad_m"]
        inside_z = mp["tol_bottom_min_m"] <= d[2] <= top_z + spec.height_m / 2.0
        lateral = float(np.hypot(d[0], d[1]))
        tilt = float(np.degrees(np.arccos(np.clip(abs(axis_base[2]), -1.0, 1.0))))
        ok = bool(inside_xy and inside_z)
        return {"lateral_m": round(lateral, 4), "tilt_deg": round(tilt, 2), "bottom_height_m": round(float(d[2]), 4), "ok": ok}

    raise ValueError(f"no evaluation for mode {slot.mode!r}")


# ---------------------------------------------------------------------------------------------
# slot names (stable labels for config, derived from geometry)
# ---------------------------------------------------------------------------------------------

#: Quantisation applied before grouping slots into rows/columns [m]. Slot coordinates carry
#: float noise — measured, adjacent cells in one row differ in the last ulp — so an exact sort
#: interleaves rows and mislabels them. 0.1 mm is far below the 60 mm grid pitch and far above
#: the noise.
_SLOT_QUANT_M = 1e-4

_DEPTH_TOKENS = {1: ("near",), 2: ("near", "far"), 3: ("near", "mid", "far"),
                 4: ("near", "mid1", "mid2", "far")}


def _rack_xy(slots) -> dict:
    """Rack-frame (lateral, depth) per slot id, quantised.

    Sorting raw base-frame coordinates would bake in the rack's orientation; every slot inherits
    the same rack rotation, so mapping back through it recovers the grid axes without needing the
    manifest. Only the ORDERING matters, so the constant offset is irrelevant.
    """
    R0 = np.asarray(slots[0].T_base_slot)[:3, :3]
    out = {}
    for s in slots:
        v = R0.T @ np.asarray(s.T_base_slot)[:3, 3]
        out[s.slot_id] = (round(float(v[0]) / _SLOT_QUANT_M) * _SLOT_QUANT_M,
                          round(float(v[1]) / _SLOT_QUANT_M) * _SLOT_QUANT_M)
    return out


def _lateral_tokens(n: int) -> list:
    """Left-to-right labels for ``n`` columns: centre only when ``n`` is odd."""
    if n == 1:
        return ["centre"]
    half = n // 2
    left = [f"left{k}" for k in range(half, 0, -1)]
    right = [f"right{k}" for k in range(1, half + 1)]
    return left + (["centre"] if n % 2 else []) + right


def _depth_tokens(n: int) -> list:
    return list(_DEPTH_TOKENS.get(n, tuple(f"row{k}" for k in range(n))))


def slot_names(slots) -> dict:
    """Stable ``{name: slot_id}`` derived from the slots' own geometry.

    Config refers to goal slots by name so that a mapping survives the grid changing. Slot ids
    are positional (``slot_id = len(slots)`` in derivation order) and would silently re-point to
    different physical cells if ``SLOT_GRID_PITCH_M`` or ``SLOT_RIM_INSET_M`` were retuned — a
    config that named ids would then be wrong without anything failing.

    The vocabulary follows what each mode's grid actually *is*, rather than forcing one shape on
    all three:

    ==============  ==========================  ==================================
    mode            grid                        names
    ==============  ==========================  ==================================
    ``floor_stand`` depth x lateral             ``near_centre``, ``mid_left1``
    ``plate_slot``  lateral only (gaps)         ``gap_left1``, ``gap_centre`` …
    ``basket_drop`` lateral only (x-split bays) ``gap_left1``, ``gap_centre`` …
    ==============  ==========================  ==================================

    Names are ordinal within the RACK, so they are invariant to how far the rack is pulled out —
    ``near_centre`` keeps meaning the same cell. Which slots are *feasible* is not invariant:
    that depends on the extension and is measured per state.

    Args:
        slots: Slots from one :func:`derive_slots` call (one mode, ids mode-local).

    Returns:
        ``{name: slot_id}``; empty when ``slots`` is empty.
    """
    if not slots:
        return {}
    xy = _rack_xy(slots)
    laterals = sorted({v[0] for v in xy.values()})
    depths = sorted({v[1] for v in xy.values()})
    lat_tok, dep_tok = _lateral_tokens(len(laterals)), _depth_tokens(len(depths))
    mode = slots[0].mode

    out = {}
    for s in slots:
        lat, dep = xy[s.slot_id]
        li, di = laterals.index(lat), depths.index(dep)
        if len(laterals) > 1 and len(depths) > 1:
            name = f"{dep_tok[di]}_{lat_tok[li]}"
        elif len(laterals) > 1:  # 1-D along the rack's lateral axis (plate tine gaps)
            name = f"gap_{lat_tok[li]}"
        else:  # 1-D along depth (bowl leans, cutlery bays)
            name = f"bay_{dep_tok[di]}" if mode == "basket_drop" else dep_tok[di]
        out[name] = s.slot_id
    assert len(out) == len(slots), f"slot names collided for mode {mode!r}: {sorted(out)}"
    by_id = {i: n for n, i in out.items()}
    for s in slots:  # stamp the label on, so a record can report it without a lookup table
        s.name = by_id[s.slot_id]
    return out
