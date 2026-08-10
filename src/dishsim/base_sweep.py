# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free robot base-pose sweep: score candidate base placements against the cached scene.

Why this exists: the baked goal-set funnels show ``limit_reject: 0`` everywhere — every
unreachable slot is *collision*-bound (the arm+payload cannot enter the machine mouth), not
distance-bound. That makes the base pose (standoff AND approach angle) the lever, and it has
never been measured: the current placement comes from one heuristic (closed-door handle at
world (0.64, 0)). This module scores candidate base poses offline so the adopted placement is
a measured optimum, not a guess.

Method — re-expression, not re-extraction. Every cached artifact is either body-local (meshes,
CoACD pieces), wrist-frame (gripper/payload), or a base-frame static pose ``T_base_body``.
Moving the robot base does not move the machine; it re-expresses the statics::

    T_delta      = inv(T_w_newbase) @ T_w_oldbase        (pure z-yaw + translation)
    T_newbase_b  = T_delta @ T_oldbase_b                 (machine bodies)
    slots        = T_delta @ T_oldbase_slot              (slots ride the rack)

The robot stays at the origin of its own frame (``fk_all_links`` base == identity), the
pedestal stays under the base and the ground under everything, so those two statics keep their
cached base-frame poses (candidate base height is fixed at the front placement's in v1 — a
height axis would change the pedestal *geometry*, not just poses). One
:class:`~dishsim.collision_world.CollisionWorld` per (state, class) is loaded once per worker
process and re-posed per candidate via :meth:`CollisionWorld.set_static_transform` —
milliseconds instead of a Kit re-extraction.

What is scored (mirroring the shipped pipeline, not a private notion of reachability):

- **Slot funnels** per (state, class) via :func:`dishsim.placement.goal_configs` on the
  re-based world — the exact pipeline that bakes ``goal_sets.json`` (including the
  per-class ``merged_cluster`` choice: thin-insertion modes use the per-piece cluster).
- **Countertop pick coverage** on the existing slab with reach_map's criterion (collision-free
  IK at the hover AND the grasp pose, every class) so the result is comparable with the
  measured ``TASK["spawn_rect_w"]`` baseline.
- **Rack feasibility gates** mirroring ``task/rack.py``: both wrist flips, pre-engage IK plus
  the full :func:`dishsim.rack_ops.slide_arm_path`, for the ``both_in`` pull and the
  ``both_out`` push.
- **Static clearance gate** in the WORLD frame (the machine never moves there): the pedestal
  box and the base_link hull at the candidate pose vs conservative convex hulls of the machine
  bodies in every loaded machine state.

Ranking is lexicographic: (number of success-bar criteria met, weighted score). The criteria
are the user-approved bar — plates > 0, bowls > 0, common cup∩tumbler floor slots > 4, all 3
cutlery bays, pick band deeper than the front placement's 0.120 m.

Frames: robot-base frame of the CANDIDATE unless suffixed ``_w`` (world), meters, Z-up, XYZW.
Determinism: every funnel rng is seeded from (candidate key, state, class, slot) via crc32.
"""

import dataclasses
import json
import os
import zlib
from dataclasses import dataclass

import fcl
import numpy as np
import trimesh

from . import config, rack_ops
from .collision_world import CollisionWorld, _fcl_convex, _tf
from .geometry import dishwasher_bodies
from .placement import SlotFrame, derive_slots, goal_configs
from .transforms import T_inv, make_T
from .ur5e_kin import ik_wrist3_all

#: Placement modes whose goal bake uses the per-piece cluster (see scripts/setup/goal_configs.py:
#: the merged gripper+object hull is a wedge that can never fit a thin insertion).
_THIN_MODES = ("basket_drop", "plate_slot")

#: Success-bar thresholds (user-approved). ``pick_depth_m`` is the front placement's measured
#: spawn-rectangle x-depth (config.TASK["spawn_rect_w"], measured by reach_map.py).
CRITERIA = {
    "plates": 1,  # feasible plate tine gaps (of 11), max over placement/placement_open
    "bowls": 1,  # feasible bowl lean slots (of 3), max over states
    "floor_common": 5,  # cup AND tumbler feasible floor slots (front baseline: 4)
    "bays": 3,  # fork cutlery bays (of 3)
    "pick_depth_m": 0.120,  # x-depth of the largest pickable rectangle on the slab
}


def _yaw_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    """XYZW quaternion for a pure z-yaw [deg]."""
    half = np.radians(yaw_deg) / 2.0
    return (0.0, 0.0, float(np.sin(half)), float(np.cos(half)))


@dataclass(frozen=True)
class Candidate:
    """A candidate base placement: world x/y of the base origin, a pure z-yaw [deg], and an
    optional base height z [m].

    ``z=None`` pins the height to the active placement's (``ROBOT_BASE_POS_W[2]``) — the v1
    behavior, byte-stable. A non-None z changes the pedestal COLUMN geometry, which pure
    re-expression cannot model; :meth:`SweepContext.rebase_all` therefore additionally
    re-expresses the ground static and swaps the baked pedestal for a candidate-height
    column (v2, for the elevated/side mounts of the Bosch study).
    """

    x: float
    y: float
    yaw_deg: float
    z: float | None = None

    @property
    def z_eff(self) -> float:
        """Effective base height [m] (the active placement's when ``z`` is None)."""
        return float(config.ROBOT_BASE_POS_W[2] if self.z is None else self.z)

    @property
    def key(self) -> str:
        suffix = "" if self.z is None else f"_z{self.z:+.3f}"
        return f"x{self.x:+.4f}_y{self.y:+.4f}_yaw{self.yaw_deg:+.2f}{suffix}"

    def T_w_base(self) -> np.ndarray:
        """World pose of the candidate base frame, shape [4, 4]."""
        return make_T((self.x, self.y, self.z_eff), _yaw_quat(self.yaw_deg))

    def T_delta(self) -> np.ndarray:
        """Re-expression transform new-base <- old-base, shape [4, 4]."""
        T_w_b0 = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
        return T_inv(self.T_w_base()) @ T_w_b0

    def to_json(self) -> dict:
        d = {"x": self.x, "y": self.y, "yaw_deg": self.yaw_deg}
        if self.z is not None:
            d["z"] = self.z
        return d

    @staticmethod
    def from_json(d: dict) -> "Candidate":
        z = d.get("z")
        return Candidate(float(d["x"]), float(d["y"]), float(d["yaw_deg"]),
                         None if z is None else float(z))


#: The front placement expressed as a candidate (identity re-expression).
FRONT = Candidate(0.0, 0.0, 0.0)


def _reference_class() -> str:
    """Class carried by the pick/rack-gate worlds (hand empty — only the cache identity).

    v1: the frozen mug. Bosch: the cup — the legacy Y-up mug's pinch gate fails extraction
    at any base reference except the frozen v1 spot (measured 2026-08-10: pad force 0.81 N
    at two different shifted references, in-band at (0, 0, 0.25); cup/tumbler seat
    correctly everywhere). Known limitation to root-cause before the mug joins v2 studies.
    """
    return "cup" if config.MACHINE == "bosch800" else "mug"


def funnel_combos() -> list[tuple[str, str]]:
    """The (state, class) pairs the sweep scores — exactly the baked combos that carry the
    active machine's success-bar counts."""
    demo = [k for k, s in config.OBJECTS.items() if s.robot_demo]
    if config.MACHINE == "bosch800":
        # Bosch bar: lower-extended (placement), middle-extended (middle_out drinkware),
        # third-rack flat-lay (third_out fork). The mug sits out (see _reference_class).
        combos = [("placement", cls) for cls in demo if cls != "mug"]
        combos += [("middle_out", c) for c in ("cup", "tumbler")]
        combos += [("third_out", "fork")]
        return combos
    combos = [("placement", cls) for cls in demo]
    combos += [("placement_open", "plate"), ("placement_open", "bowl")]
    return combos


def _seeded_rng(*parts) -> np.random.Generator:
    """Deterministic rng from string-able parts (process-independent, unlike hash())."""
    key = "|".join(str(p) for p in parts).encode()
    return np.random.default_rng(zlib.crc32(key))


def rebase_statics(world: CollisionWorld, statics0: dict[str, np.ndarray],
                   T_delta: np.ndarray) -> None:
    """Re-pose one world's MACHINE statics by ``T_delta`` (new-base <- old-base).

    The pedestal and the ground are base-anchored — they keep their cached base-frame poses.
    ``statics0`` holds the cached (front-placement) ``T_base_body`` per static; passing an
    identity ``T_delta`` restores the world exactly.
    """
    for name in dishwasher_bodies():
        if name in statics0:
            world.set_static_transform(name, T_delta @ statics0[name])


# ---------------------------------------------------------------------------------------------
# per-process context: worlds loaded once, re-posed per candidate
# ---------------------------------------------------------------------------------------------


class SweepContext:
    """Everything one worker process needs: worlds, original static poses, slots, gate geometry.

    Loading flips ``config.apply_scenario`` / ``config.set_active_object`` per cache so every
    ``load_manifest`` staleness check runs against the config the cache was baked under (the
    FRONT placement — the sweep never mutates the placement constants). Scoring re-activates
    the class it evaluates, because the goal-pose generators read the active spec at call time.
    """

    def __init__(self):
        self.worlds: dict[tuple[str, str], CollisionWorld] = {}
        self.statics0: dict[tuple[str, str], dict[str, np.ndarray]] = {}
        self.slots0: dict[tuple[str, str], list[SlotFrame]] = {}
        self.machine_objs_w: dict[str, list[fcl.CollisionObject]] = {}

        for state, cls in funnel_combos():
            world = self._load(state, cls, attached=True,
                               merged=config.effective_placement_mode(cls) not in _THIN_MODES)
            key = (state, cls)
            self.worlds[key] = world
            self.statics0[key] = self._statics(world)
            self.slots0[key] = derive_slots(config.scenario_cache_dir(state, object_name=cls))

        # pick world: the machine and nothing carried — the state a pick happens in
        ref = _reference_class()
        self.pick_world = self._load("placement", ref, attached=False, merged=True)
        self.pick_statics0 = self._statics(self.pick_world)

        # rack-gate worlds: the two robot-facing initial states, nothing carried
        self.rack_worlds: dict[str, CollisionWorld] = {}
        self.rack_statics0: dict[str, dict[str, np.ndarray]] = {}
        for scenario in config.SCENARIOS:
            world = self._load(scenario, ref, attached=False, merged=True)
            self.rack_worlds[scenario] = world
            self.rack_statics0[scenario] = self._statics(world)

        # world-frame machine hulls for the static-clearance gate, one set per distinct
        # machine state (the machine never moves in the world; only the candidate base does)
        T_w_b0 = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
        for state, world in (("placement", self.pick_world),
                             *self.rack_worlds.items()):
            objs = []
            for name in dishwasher_bodies():
                entry = world.manifest["statics"].get(name)
                if entry is None:
                    continue
                mesh = trimesh.load(os.path.join(world.cache_dir, entry["mesh"]), force="mesh")
                T_w_body = T_w_b0 @ np.array(entry["T_base_body"])
                objs.append(fcl.CollisionObject(_fcl_convex(mesh), _tf(T_w_body)))
            self.machine_objs_w[state] = objs

        # candidate-side gate geometry: the pedestal box and the base_link hull
        self.pedestal_mesh = trimesh.creation.box(extents=config.PEDESTAL_SIZE)
        base_entry = self.pick_world.manifest["arm_links"]["base_link"]
        self.base_link_mesh = trimesh.load(
            os.path.join(self.pick_world.cache_dir, base_entry["mesh"]), force="mesh"
        )

        config.set_active_object(ref)

    @staticmethod
    def _load(state: str, cls: str, *, attached: bool, merged: bool) -> CollisionWorld:
        config.set_active_object(cls)
        config.apply_scenario(state)
        cache = config.scenario_cache_dir(state, object_name=cls)
        return CollisionWorld(cache_dir=cache, self_check=True,
                              object_attached=attached, merged_cluster=merged)

    @staticmethod
    def _statics(world: CollisionWorld) -> dict[str, np.ndarray]:
        return {name: np.array(entry["T_base_body"])
                for name, entry in world.manifest["statics"].items()}

    # -- per-candidate re-posing ---------------------------------------------------------------

    def rebase_all(self, cand: Candidate) -> None:
        """Re-pose every world's machine statics for ``cand``.

        At the baseline height (``cand.z is None``) the pedestal and ground keep their cached
        base-frame poses — the v1 behavior, byte-stable. At a non-baseline height the ground
        is world-anchored like the machine (re-expressed by ``T_delta``) and the baked
        25 cm pedestal is disabled in favor of a full candidate-height support column from
        the base plane down to the floor (an ``add_object`` extra, checked against every
        movable).
        """
        T_delta = cand.T_delta()
        z0 = float(config.ROBOT_BASE_POS_W[2])
        height_axis = cand.z is not None and abs(cand.z_eff - z0) > 1e-9
        pairs = [*((self.worlds[k], self.statics0[k]) for k in self.worlds),
                 (self.pick_world, self.pick_statics0),
                 *((self.rack_worlds[s], self.rack_statics0[s]) for s in self.rack_worlds)]
        for world, statics0 in pairs:
            rebase_statics(world, statics0, T_delta)
            if "ground" in statics0:
                world.set_static_transform(
                    "ground", (T_delta @ statics0["ground"]) if height_axis else statics0["ground"])
            if "pedestal" in statics0:
                world.set_static_enabled("pedestal", not height_axis)
            if world.has_object("pedestal_column"):
                world.remove_object("pedestal_column")
            if height_axis:
                col_h = cand.z_eff
                col = trimesh.creation.box(
                    extents=(config.PEDESTAL_SIZE[0], config.PEDESTAL_SIZE[1], col_h))
                T_base_col = make_T((0.0, 0.0, -col_h / 2.0), (0.0, 0.0, 0.0, 1.0))
                world.add_object("pedestal_column", [col], T_base_col)

    def slots_for(self, cand: Candidate, state: str, cls: str) -> list[SlotFrame]:
        """The (state, class) slots re-expressed in the candidate base frame."""
        T_delta = cand.T_delta()
        return [dataclasses.replace(s, T_base_slot=T_delta @ s.T_base_slot)
                for s in self.slots0[(state, cls)]]


# ---------------------------------------------------------------------------------------------
# gates
# ---------------------------------------------------------------------------------------------


def gate_static_clearance(ctx: SweepContext, cand: Candidate) -> bool:
    """Candidate base column (pedestal box + base_link hull) clear of the machine in every
    loaded machine state.

    Runs in the WORLD frame, so it needs no re-basing. Conservative on purpose: machine bodies
    are single convex hulls (a hull fills the rack basket and the machine mouth), which rejects
    a pedestal *inside* those volumes — where no sane base stands anyway.
    """
    T_w_base = cand.T_w_base()
    # full support column from the base plane to the floor (candidate height, not the
    # baked pedestal's — at the baseline height the two coincide)
    col_h = cand.z_eff
    col_mesh = trimesh.creation.box(
        extents=(config.PEDESTAL_SIZE[0], config.PEDESTAL_SIZE[1], col_h))
    T_w_ped = T_w_base @ make_T((0.0, 0.0, -col_h / 2.0), (0, 0, 0, 1))
    probes = [
        fcl.CollisionObject(_fcl_convex(col_mesh), _tf(T_w_ped)),
        fcl.CollisionObject(_fcl_convex(ctx.base_link_mesh), _tf(T_w_base)),
    ]
    for objs in ctx.machine_objs_w.values():
        for machine_obj in objs:
            for probe in probes:
                res = fcl.CollisionResult()
                if fcl.collide(probe, machine_obj, fcl.CollisionRequest(), res):
                    return False
    return True


def gate_home_free(ctx: SweepContext) -> bool:
    """HOME_Q collision-free with the mug carried and with the hand empty (after re-basing)."""
    home = np.array(config.HOME_Q)
    ref = _reference_class()
    return not ctx.worlds[("placement", ref)].in_collision(home) and not ctx.pick_world.in_collision(home)


def gate_rack(ctx: SweepContext, cand: Candidate, scenario: str) -> bool:
    """The scenario's rack action stays executable: pre-engage IK + full slide path, both
    wrist flips (mirrors task/rack.py's plan step, on the re-based world)."""
    world = ctx.rack_worlds[scenario]
    action = config.SCENARIOS[scenario]["rack_action"]
    body = action["body"]
    params = config.RACK_GEN[body]
    st = world.manifest["statics_state"]
    cached_ext = float(st["rack_lower_m"] if body == "E_shelf_1_04" else st["rack_upper_m"])
    T_base_body = cand.T_delta() @ ctx.rack_statics0[scenario][body]
    T_w3_tcp_inv = T_inv(np.array(world.manifest["t_wrist3_tcp"]))
    approach = rack_ops.approach_dir(T_base_body)
    home = np.array(config.HOME_Q)
    for flip in (False, True):
        T_engage = rack_ops.engage_pose(T_base_body, params, action, cached_ext, flip=flip)
        T_pre = T_engage.copy()
        T_pre[:3, 3] = T_engage[:3, 3] - approach * config.RACK_APPROACH_HOVER_M
        pre_sols = [q for q in ik_wrist3_all(T_pre @ T_w3_tcp_inv, q_seed=home)
                    if not world.in_collision(q)]
        if not pre_sols:
            continue
        path = rack_ops.slide_arm_path(world, T_base_body, params, action, cached_ext,
                                       pre_sols[0], flip=flip)
        if path is not None:
            return True
    return False


# ---------------------------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------------------------


def largest_rectangle(mask: np.ndarray) -> tuple[int, int, int, int]:
    """Largest all-True axis-aligned rectangle in a boolean grid.

    Returns:
        ``(i0, i1, j0, j1)`` inclusive index bounds, or ``(0, -1, 0, -1)`` when empty.
    """
    n_rows, n_cols = mask.shape
    heights = np.zeros(n_cols, dtype=int)
    best = (0, (0, -1, 0, -1))
    for i in range(n_rows):
        heights = np.where(mask[i], heights + 1, 0)
        stack: list[tuple[int, int]] = []
        for j in range(n_cols + 1):
            h = heights[j] if j < n_cols else 0
            start = j
            while stack and stack[-1][1] >= h:
                sj, sh = stack.pop()
                area = sh * (j - sj)
                if area > best[0] and sh > 0:
                    best = (area, (i - sh + 1, i, sj, j - 1))
                start = sj
            stack.append((start, h))
    return best[1]


def _stand_R(spec, yaw_deg: float) -> np.ndarray:
    """Rotation standing the object axis up, then yawing about world z (reach_map convention)."""
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    axis = tuple(spec.axis_obj)
    if axis == (0.0, 1.0, 0.0):
        R0 = Rotation.from_euler("x", 90.0, degrees=True).as_matrix()
    elif axis == (0.0, 0.0, 1.0):
        R0 = np.eye(3)
    else:
        R0 = Rotation.from_euler("y", -90.0, degrees=True).as_matrix()
    return Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix() @ R0


def _bottom_offset(spec, R: np.ndarray) -> float:
    """Distance from the object origin down to its lowest bbox point under ``R``."""
    h = np.array(spec.bbox_half)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return -float((R @ corners.T).T[:, 2].min())


def _pick_feasible(world, T_hover, T_grasp, T_tcp_w3, q_seed) -> bool:
    """reach_map's criterion: one arm configuration clears BOTH the hover and the grasp pose."""
    hover_sols = [q for q in ik_wrist3_all(T_hover @ T_tcp_w3, q_seed=q_seed)
                  if not world.in_collision(q)]
    if not hover_sols:
        return False
    return any(not world.in_collision(q)
               for q in ik_wrist3_all(T_grasp @ T_tcp_w3, q_seed=hover_sols[0]))


def pick_coverage(ctx: SweepContext, cand: Candidate, *, step: float, n_yaws: int,
                  classes: list[str] | None = None) -> dict:
    """Pickable-cell coverage of the existing countertop slab from the candidate base pose.

    Same criterion as reach_map.py (both-pose IK, every class), full-transform world->base
    conversion (the candidate may be yawed). Cells below world x 0.65 are mapped but excluded
    from the rectangle metric, mirroring reach_map's ``--rect_x_min`` door-clearance rule.

    Returns:
        ``{"per_class": {name: cells}, "union_cells", "depth_x_m", "rect_w"}``.
    """
    world = ctx.pick_world
    T_b1_w = T_inv(cand.T_w_base())
    T_tcp_w3 = T_inv(np.array(world.manifest["t_wrist3_tcp"]))
    q_seed = np.array(config.HOME_Q)

    cx, cy, cz = config.COUNTERTOP_CENTER_W
    sx, sy, _ = config.COUNTERTOP_SIZE
    top_z = cz + config.COUNTERTOP_SIZE[2] / 2.0
    xs = np.arange(cx - sx / 2.0, cx + sx / 2.0 + 1e-9, step)
    ys = np.arange(cy - sy / 2.0, cy + sy / 2.0 + 1e-9, step)
    yaws = np.linspace(0.0, 360.0, n_yaws, endpoint=False)

    names = classes or [k for k, s in config.OBJECTS.items() if s.robot_demo]
    masks = {}
    for name in names:
        spec = config.OBJECTS[name]
        T_obj_tcp = T_inv(make_T(*config.grasp_transform(spec)))
        mask = np.zeros((len(xs), len(ys)), dtype=bool)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for yaw in yaws:
                    R = _stand_R(spec, float(yaw))
                    T_w_obj = np.eye(4)
                    T_w_obj[:3, :3] = R
                    T_w_obj[:3, 3] = (x, y, top_z + _bottom_offset(spec, R))
                    T_grasp = (T_b1_w @ T_w_obj) @ T_obj_tcp
                    T_hover = T_grasp.copy()
                    T_hover[2, 3] += config.PICK_HOVER_M
                    if _pick_feasible(world, T_hover, T_grasp, T_tcp_w3, q_seed):
                        mask[i, j] = True
                        break
        masks[name] = mask

    union = np.logical_or.reduce(list(masks.values()))
    allowed = union & (xs[:, None] >= 0.65)
    i0, i1, j0, j1 = largest_rectangle(allowed)
    depth = float(xs[i1] - xs[i0] + step) if i1 >= i0 else 0.0
    rect = ({"x_min": float(xs[i0]), "x_max": float(xs[i1]),
             "y_min": float(ys[j0]), "y_max": float(ys[j1])} if i1 >= i0 else None)
    return {
        "per_class": {k: int(v.sum()) for k, v in masks.items()},
        "union_cells": int(allowed.sum()),
        "depth_x_m": depth,
        "rect_w": rect,
        "step_m": step,
        "n_yaws": n_yaws,
    }


def funnel_feasible_slots(ctx: SweepContext, cand: Candidate, state: str, cls: str, *,
                          n_samples: int, early_exit: int | None) -> dict:
    """Feasible-slot ids for one (state, class) at the candidate pose, via the shipped funnel.

    Returns:
        ``{"feasible": [slot_id, ...], "accepted": {slot_id: n}}``.
    """
    config.set_active_object(cls)
    world = ctx.worlds[(state, cls)]
    feasible, accepted = [], {}
    for slot in ctx.slots_for(cand, state, cls):
        rng = _seeded_rng(cand.key, state, cls, slot.slot_id, n_samples)
        gs = goal_configs(slot, world, rng, n_samples=n_samples, early_exit_accepted=early_exit)
        accepted[slot.slot_id] = int(len(gs.configs))
        if len(gs.configs):
            feasible.append(slot.slot_id)
    return {"feasible": feasible, "accepted": accepted}


# ---------------------------------------------------------------------------------------------
# scoring
# ---------------------------------------------------------------------------------------------

#: Named effort presets: (funnel combos subset or None=all, n_samples, early_exit,
#: pick step, pick yaws, pick classes subset or None=all).
LEVELS = {
    "proxy": {"combos": [("placement", "cup"), ("placement", "fork"),
                         ("placement_open", "plate"), ("placement_open", "bowl")],
              "n_samples": 6, "early_exit": 1,
              "pick_step": 0.10, "pick_yaws": 2, "pick_classes": ["mug", "plate"]},
    "full": {"combos": None, "n_samples": 16, "early_exit": 3,
             "pick_step": 0.05, "pick_yaws": 4, "pick_classes": None},
    "final": {"combos": None, "n_samples": config.GOAL_POSE_SAMPLES_PER_SLOT, "early_exit": None,
              "pick_step": 0.05, "pick_yaws": 4, "pick_classes": None},
}


def run_gates(ctx: SweepContext, cand: Candidate) -> dict:
    """Gate one candidate (cheapest first). Re-bases the worlds as a side effect.

    ``rack_push`` (the both_out stow) is recorded but NOT a hard gate: at the current config
    the push's final waypoint rejects with ``gripper~E_body_5`` for every IK branch — a
    cluster-vs-machine collision at the machine-fixed handle TCP, which no base pose can
    change (measured 2026-08-09, after the countertop widening; the front placement fails it
    too). A candidate that restores it scores a bonus instead.
    """
    gates = {"static_clearance": gate_static_clearance(ctx, cand)}
    if gates["static_clearance"]:
        ctx.rebase_all(cand)
        gates["home_free"] = gate_home_free(ctx)
        if gates["home_free"]:
            gates["rack_pull"] = gate_rack(ctx, cand, "both_in")
            if gates["rack_pull"]:
                gates["rack_push"] = gate_rack(ctx, cand, "both_out")
    gates["ok"] = all(gates.get(k, False)
                      for k in ("static_clearance", "home_free", "rack_pull"))
    return gates


def score_candidate(ctx: SweepContext, cand: Candidate, level: str) -> dict:
    """Gate + score one candidate at a named effort level; returns the scorecard dict."""
    knobs = LEVELS[level]
    card = {"candidate": cand.to_json(), "key": cand.key, "level": level}
    card["gates"] = run_gates(ctx, cand)
    if not card["gates"]["ok"]:
        card["feasible"] = False
        return card
    card["feasible"] = True

    combos = [c for c in (knobs["combos"] or funnel_combos()) if c in ctx.worlds]
    funnels = {}
    for state, cls in combos:
        funnels[f"{state}/{cls}"] = funnel_feasible_slots(
            ctx, cand, state, cls, n_samples=knobs["n_samples"], early_exit=knobs["early_exit"])
    card["funnels"] = funnels

    def n_feasible(state: str, cls: str) -> int:
        entry = funnels.get(f"{state}/{cls}")
        return len(entry["feasible"]) if entry else 0

    def common_floor() -> int:
        a = funnels.get("placement/cup")
        b = funnels.get("placement/tumbler")
        if a is None or b is None:
            return 0
        return len(set(a["feasible"]) & set(b["feasible"]))

    card["pick"] = pick_coverage(ctx, cand, step=knobs["pick_step"], n_yaws=knobs["pick_yaws"],
                                 classes=knobs["pick_classes"])

    if config.MACHINE == "bosch800":
        # Bosch A2 bar: destinations per rack tier + the pick band (docs plan §A2)
        demo = [k for k, s in config.OBJECTS.items() if s.robot_demo]
        counts = {
            "lower_union": sum(n_feasible("placement", c) for c in demo),
            "middle_union": sum(n_feasible("middle_out", c) for c in ("cup", "tumbler")),
            "third_flat": n_feasible("third_out", "fork"),
            "floor_common": common_floor(),
        }
        card["counts"] = counts
        criteria = {
            "lower": counts["lower_union"] >= 1,
            "middle": counts["middle_union"] >= 1,
            "third_flat": counts["third_flat"] >= 1,
            "pick_band": card["pick"]["depth_x_m"] > CRITERIA["pick_depth_m"],
        }
        card["criteria"] = criteria
        card["n_criteria"] = int(sum(criteria.values()))
        card["weighted"] = float(
            2.0 * counts["lower_union"] + 3.0 * counts["middle_union"]
            + 2.0 * counts["third_flat"] + 1.0 * counts["floor_common"]
            + 10.0 * card["pick"]["depth_x_m"] + 0.02 * card["pick"]["union_cells"]
            + (3.0 if card["gates"].get("rack_push") else 0.0)
        )
        return card

    counts = {
        "plate": max(n_feasible("placement", "plate"), n_feasible("placement_open", "plate")),
        "bowl": max(n_feasible("placement", "bowl"), n_feasible("placement_open", "bowl")),
        "fork": n_feasible("placement", "fork"),
        "floor_cup": n_feasible("placement", "cup"),
        "floor_tumbler": n_feasible("placement", "tumbler"),
        "floor_common": common_floor(),
        "floor_mug": n_feasible("placement", "mug"),
    }
    card["counts"] = counts

    criteria = {
        "plates": counts["plate"] >= CRITERIA["plates"],
        "bowls": counts["bowl"] >= CRITERIA["bowls"],
        "floor": counts["floor_common"] >= CRITERIA["floor_common"],
        "bays": counts["fork"] >= CRITERIA["bays"],
        "pick_band": card["pick"]["depth_x_m"] > CRITERIA["pick_depth_m"],
    }
    # the proxy level scores a funnel subset — its criteria are comparable within the level
    # only, which is all ranking needs; stage promotion re-scores at a fuller level
    card["criteria"] = criteria
    card["n_criteria"] = int(sum(criteria.values()))
    card["weighted"] = float(
        4.0 * counts["plate"] + 3.0 * counts["bowl"] + 2.0 * counts["fork"]
        + 1.0 * counts["floor_common"] + 10.0 * card["pick"]["depth_x_m"]
        + 0.02 * card["pick"]["union_cells"]
        + (3.0 if card["gates"].get("rack_push") else 0.0)
    )
    return card


def rank_key(card: dict) -> tuple:
    """Sort key: infeasible last, then criteria met, then weighted score."""
    if not card.get("feasible"):
        return (-1, -1.0)
    return (card["n_criteria"], card["weighted"])
