# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free rack-design evaluation: score a candidate lower-rack layout against the robot.

Why this exists: the base-pose sweep proved every dead slot class is blocked by the gripper
cluster + payload at the machine-fixed goal pose — the rack's own fixtures don't admit the
payload with the collision margin. The racks are procedural (:mod:`dishsim.rack_gen`), so the
fix is a measured redesign. This module evaluates a candidate ``RACK_GEN`` parameter dict
against the *cached* scene in seconds: generate the candidate's parts, swap them into a loaded
:class:`~dishsim.collision_world.CollisionWorld` via :meth:`replace_static`, rebuild the slots
analytically from the candidate parameters, and run the shipped goal funnel
(:func:`dishsim.placement.goal_configs`) per slot. One Kit bake happens only after a design
wins.

Measured design rules this encodes (2026-08-09 diagnostics):

- A goal pose that *leans* the payload onto a fixture is unplaceable by construction — it sits
  inside the ``COLLISION_MARGIN_M`` inflation. Robot-facing goals must hover in free corridor
  space and let gravity settle the lean (the ``basket_drop`` pattern).
- Tie wires cross the tine gaps *through the disc plane* — a robot-facing plate bank must not
  have them (``tine_tie_frac: None``).
- The fork-drop stack (gripper above a hanging fork) clears the machine only through the open
  mouth front: drop columns must stay at rack-frame y where the world-x of the column is in
  front of the shell top (bays at y ≲ 0.10 measured safe, ≥ 0.126 measured capped).
- Slots at rack y ≳ 0.14 risk the stowed upper rack's roof (y ≥ 0.200, z ≥ 0.154) through the
  cluster's lateral extent; feasibility there is yaw-dependent — measure, don't assume.

Frames: rack design frame per :mod:`dishsim.rack_gen` (x lateral, y depth with 0 at the front
edge, z up from the lowest wire); slots are emitted in the robot-base frame via the cached
``T_base_body``. No ``pxr``/``omni`` imports — venv only.
"""

import copy
from dataclasses import dataclass, field

import numpy as np

from . import config, rack_gen
from .collision_world import CollisionWorld
from .placement import SlotFrame, goal_configs
from .transforms import T_inv

#: Zones stripped from robot-facing candidate builds when ``drop_ties`` is set (the rear
#: fill-only bank keeps its ties for realism; robot-placed discs cannot coexist with them).
_TIE_ZONE = "tie"


@dataclass
class Candidate:
    """A candidate lower-rack design: parameter overlay + evaluation knobs.

    ``overlay`` entries replace keys of ``config.RACK_GEN["E_shelf_1_04"]``; a value of
    ``None`` deletes the key (e.g. ``{"bowl_tine_xs": None}`` removes the bowl tines).
    """

    name: str
    overlay: dict = field(default_factory=dict)
    drop_ties: bool = True
    #: plate goal corridor depth [m] (rack y of the release plane); None = mid of plate_rows_y
    plate_corridor_y: float | None = None
    #: release lean for robot plate goals [deg] (settle lean comes from the tines)
    plate_release_lean_deg: float = 0.0

    def params(self) -> dict:
        p = copy.deepcopy(config.RACK_GEN["E_shelf_1_04"])
        for k, v in self.overlay.items():
            if v is None:
                p.pop(k, None)
            else:
                p[k] = v
        return p

    def parts(self) -> list:
        parts = rack_gen.build_rack(self.params())
        if self.drop_ties:
            parts = [pt for pt in parts if getattr(pt, "zone", "") != _TIE_ZONE]
        return parts


def load_world(state: str, cls: str) -> CollisionWorld:
    """A (state, class) world from the baked caches, per the goal bake's cluster convention."""
    config.set_active_object(cls)
    config.apply_scenario(state)
    merged = config.OBJECTS[cls].placement.mode not in ("basket_drop", "plate_slot")
    return CollisionWorld(cache_dir=config.scenario_cache_dir(state, object_name=cls),
                          self_check=True, object_attached=True, merged_cluster=merged)


def swap_rack(world: CollisionWorld, cand: Candidate) -> np.ndarray:
    """Install the candidate's parts as the world's lower rack; returns ``T_base_rack``."""
    T = np.array(world.manifest["statics"]["E_shelf_1_04"]["T_base_body"])
    world.replace_static("E_shelf_1_04", [pt.mesh for pt in cand.parts()])
    return T


# ---------------------------------------------------------------------------------------------
# analytic slot builders (explicit params — no manifest/config_hash coupling)
# ---------------------------------------------------------------------------------------------


def plate_slots(cand: Candidate, T_base_rack: np.ndarray) -> list[SlotFrame]:
    p = cand.params()
    xs = np.asarray(rack_gen._plate_tine_xs(p))
    gaps = (xs[:-1] + xs[1:]) / 2.0
    z_floor = rack_gen.floor_top_z(p)
    y = cand.plate_corridor_y
    if y is None:
        y = float(np.mean(p["plate_rows_y"]))
    out = []
    for gi, gx in enumerate(gaps):
        T = np.eye(4)
        T[:3, 3] = (float(gx), y, z_floor)
        out.append(SlotFrame(gi, T_base_rack @ T, float(p["plate_tine_pitch"]), "derived",
                             mode="plate_slot", rack="lower",
                             params={"lean_deg": cand.plate_release_lean_deg}))
    return out


def basket_slots(cand: Candidate, T_base_rack: np.ndarray) -> list[SlotFrame]:
    p = cand.params()
    b = p["basket"]
    z = rack_gen.floor_top_z(p) + b["floor_t"]
    mp = config.placement_mode_params("basket_drop")
    rotated = "dividers_x" in b
    out = []
    for bi, (x0, x1, y0, y1) in enumerate(rack_gen.basket_bays(p)):
        # Slot frames stay IDENTITY-oriented for both bay orientations. Measured: rotating the
        # slot 90 deg rotates the wrist yaw of every drop, and those arm configurations sweep
        # the shell/counter — the identity-yaw approach is the one the arm can execute. The
        # fork's slim cross-section (16 x 9 mm inflated to 26 x 19) fits either bay axis.
        T = np.eye(4)
        # the drop offset exists only to dodge the arch handle bar — a handleless basket
        # drops dead-center in the bay
        has_handle = b.get("handle") is not None
        dx = float(mp["drop_x_offset_m"]) if (has_handle and not rotated) else 0.0
        dy = float(mp["drop_x_offset_m"]) if (has_handle and rotated) else 0.0
        T[:3, 3] = ((x0 + x1) / 2.0 + dx, (y0 + y1) / 2.0 + dy, z)
        out.append(SlotFrame(bi, T_base_rack @ T, float(min(x1 - x0, y1 - y0)),
                             "derived", mode="basket_drop", rack="basket",
                             params={"bay": [x0, x1, y0, y1],
                                     "top_z": float(b["h"] - b["floor_t"])}))
    return out


def floor_slots(cand: Candidate, T_base_rack: np.ndarray, spec) -> list[SlotFrame]:
    """Standing grid over the candidate's open floor, sized for ``spec`` (cup/bowl/...)."""
    p = cand.params()
    z_floor = rack_gen.floor_top_z(p)
    if tuple(spec.axis_obj) == (0.0, 1.0, 0.0):
        footprint = 2.0 * max(spec.bbox_half[0], spec.bbox_half[2])
    else:
        footprint = 2.0 * max(spec.bbox_half[0], spec.bbox_half[1])
    pitch = config.SLOT_GRID_PITCH_M
    W, D = p["footprint"]
    lo = np.array([config.SLOT_RIM_INSET_M] * 2)
    hi = np.array([W - config.SLOT_RIM_INSET_M, D - config.SLOT_RIM_INSET_M])
    nx = max(1, int((hi[0] - lo[0]) // pitch) + 1)
    ny = max(1, int((hi[1] - lo[1]) // pitch) + 1)
    xs = lo[0] + (hi[0] - lo[0] - (nx - 1) * pitch) / 2.0 + np.arange(nx) * pitch
    ys = lo[1] + (hi[1] - lo[1] - (ny - 1) * pitch) / 2.0 + np.arange(ny) * pitch
    out = []
    for y in ys:
        for x in xs:
            T = np.eye(4)
            T[:3, 3] = (float(x), float(y), z_floor)
            out.append(SlotFrame(len(out), T_base_rack @ T, footprint, "derived",
                                 mode="floor_stand", rack="lower"))
    return out


# ---------------------------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------------------------


def _funnel(world: CollisionWorld, slot: SlotFrame, seed: int, *, n: int, early: int | None) -> int:
    gs = goal_configs(slot, world, np.random.default_rng(seed), n_samples=n,
                      early_exit_accepted=early)
    return int(len(gs.configs))


def score_design(cand: Candidate, *, n_samples: int = 32, early: int | None = 3,
                 states: tuple = ("placement",), log=print) -> dict:
    """Success-bar scorecard for a candidate design, per state.

    Floor cells are evaluated with the object standing upright (``floor_stand`` goal
    generation), which for the bowl means the stand-upright placement policy.
    """
    out = {"name": cand.name, "states": {}}
    for state in states:
        row: dict = {}
        # plates (per-piece cluster world)
        world = load_world(state, "plate")
        T = swap_rack(world, cand)
        accepted = [_funnel(world, s, 1000 + s.slot_id, n=n_samples, early=early)
                    for s in plate_slots(cand, T)]
        row["plate"] = accepted
        # fork bays
        world = load_world(state, "fork")
        T = swap_rack(world, cand)
        accepted = [_funnel(world, s, 2000 + s.slot_id, n=n_samples, early=early)
                    for s in basket_slots(cand, T)]
        row["fork"] = accepted
        # standing classes on the floor grid (bowl included: stand-upright policy)
        for cls in ("cup", "tumbler", "mug", "bowl"):
            world = load_world(state, cls)
            T = swap_rack(world, cand)
            spec = config.OBJECTS[cls]
            mode0 = spec.placement.mode
            object.__setattr__(spec.placement, "mode", "floor_stand")
            try:
                slots = floor_slots(cand, T, spec)
                accepted = {s.slot_id: _funnel(world, s, 3000 + s.slot_id, n=n_samples, early=early)
                            for s in slots}
            finally:
                object.__setattr__(spec.placement, "mode", mode0)
            row[cls] = [sid for sid, n_acc in accepted.items() if n_acc > 0]
        row["floor_common"] = sorted(set(row["cup"]) & set(row["tumbler"]))
        bar = {
            "plates": sum(1 for a in row["plate"] if a > 0),
            "bowls": len(row["bowl"]),
            "bays": sum(1 for a in row["fork"] if a > 0),
            "floor_common": len(row["floor_common"]),
        }
        row["bar"] = bar
        out["states"][state] = row
        log(f"[{cand.name} @ {state}] plates {bar['plates']}/{len(row['plate'])} "
            f"bowls {bar['bowls']} bays {bar['bays']}/{len(row['fork'])} "
            f"floor* {bar['floor_common']} (cup {row['cup']} ∩ tumbler {row['tumbler']})")
    return out
