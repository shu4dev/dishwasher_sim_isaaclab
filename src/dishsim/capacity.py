# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reachable-capacity planning — what "a full load" means, counted honestly.

The machine's geometric capacity (every slot the racks provide) and its ROBOT capacity (what
the arm can actually place from the measured base mount) are different numbers, and only the
second one defines a full load for the loading task. ``fill_plan.py`` computes the first by
teleport; this module computes the second, Kit-free, from artifacts that already exist:

- a slot is REACHABLE iff its baked goal set (``slots/goal_sets.json``) is non-empty — the
  same predicate the episode runner's slot assignment uses;
- a load is JOINTLY placeable only if each item still has a collision-free goal config with
  every earlier item sitting at its own goal — the exact re-filter
  ``task.primitives._place`` runs live, executed here against the baked configs with
  neighbours added to the state's :class:`~dishsim.collision_world.CollisionWorld`. Without
  this, "N reachable slots" overcounts: adjacent goals share approach corridors;
- an item may only ride a rack through a scripted transition if its worst-case tolerance
  pose clears the geometry that will pass overhead (:func:`z_budget_clearance_m`). Measured
  on the Bosch (2026-08-14, scratchpad z-budget table): a tumbler on the middle rack misses
  the third rack's underside by 15.8 mm at worst-case tilt, a cup clears by 20.6 mm — which
  is why a full load puts only cups on the middle rack.

Packing (candidate order, occupancy, most-constrained-first) is shared with the runner via
:mod:`dishsim.task.slotting`, so the planned N is exactly what ``run_task._assign_slots``
would hand out.

Context discipline: the CALLER applies machine and base placement (the order contract:
machine -> object -> scenario -> placement). :func:`plan_full_load` flips
``config.apply_scenario`` per state itself and restores the entry scenario on exit; every
per-class read runs under ``config.active_object``.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np

from . import config, placement, rack_gen
from .collision_world import CollisionWorld, load_object_pieces
from .geometry import config_hash
from .task import slotting

CAPACITY_SCHEMA_VERSION = 1

#: Loading phases in execution order, per machine: each is an INTERNAL state (no rack
#: action) in which exactly one rack is extended and loadable. The LOWER rack loads LAST:
#: its stow drives the loaded rack back up over the door sill, which stalls under load
#: (measured 2026-08-14: 176.9 mm short of stowed with one bowl aboard), while the third and
#: middle racks stow in tub rails and the lower rack always EXTENDS outward (downhill, the
#: same direction the robot pull uses). Top-down order means no transition ever stows the
#: lower rack, and the episode ends with the loaded machine open — the v0 semantics.
LOADING_STATES: dict[str, tuple[str, ...]] = {
    "bosch800": ("third_out", "middle_out", "placement"),
    "artvip_compact": ("placement",),
}

#: Class request order per state. A tuple entry round-robins its classes until each retires.
#: ``plates_first``: the rear plate bank fills before the floor grid so disc placements are
#: certified against an empty rack mouth; the floor grid then round-robins bowl/cup/tumbler.
#: Measured (2026-08-14, placement phase at side_winner): the round-robin packs 13 items
#: (2 plates, 4 bowls, 4 cups, 3 tumblers) where a bowls-first order manages only 9 — six
#: 71 mm-radius bowls at 153 mm separation blanket the reachable floor before drinkware gets
#: a turn. The middle-rack phase requests both drinkware classes and lets the z-budget gate
#: retire what cannot ride (measured: the tumbler).
POLICIES: dict[str, dict[str, tuple]] = {
    "plates_first": {
        "placement": ("plate", ("bowl", "cup", "tumbler")),
        "middle_out": (("cup", "tumbler"),),
        "third_out": ("fork",),
    },
}

#: Required worst-case clearance [m] between a rider's top and the geometry passing
#: overhead during a scripted rack transition (settle noise + rack ride wobble allowance).
Z_BUDGET_MARGIN_M = 0.015

#: The mid spray arm hangs this far under the middle rack's body frame [m] — mirrors the
#: authored geometry in ``usd_prep.make_bosch800_usd`` (MidSprayArm disc at z -0.045).
_MID_SPRAY_DROP_M = 0.045

#: Joint-certification work bounds: stop counting survivors at the cap (4x the runtime
#: ``GOALS_PER_PLAN`` budget of 64 — anything past "plenty" is unread), and stop scanning a
#: slot's configs once at least one survivor exists and this many configs were checked
#: (reachable-rich bakes carry thousands of wrap-expanded configs per slot).
_CERTIFY_CONFIG_CAP = 256
_CERTIFY_SCAN_CAP = 512

#: Measured upright-settle reliability per (state, class) — release-at-goal probes,
#: ``scripts/setup/probe_plate_settle.py``, 2026-08-14 (fractions of releases inside the
#: placement tolerances after SETTLE_STEPS). The Bosch racks' OEM wire lattice has 41-46 mm
#: cells; scaled drinkware bases (~50-60 mm) wedge into them roughly half the time whatever
#: the release alignment (wire-crossing anchoring measured 228/300 — better, still short).
#: Real machines load glasses rim-down, which needs a flip regrasp this stack does not have
#: (docs/known_limitations.md). Bowls span any cell and plates seat in their bank.
MEASURED_SETTLE_RELIABILITY: dict[tuple[str, str], float] = {
    ("placement", "bowl"): 59 / 60,
    ("placement", "cup"): 49 / 82,
    ("placement", "tumbler"): 64 / 88,
    ("middle_out", "cup"): 63 / 120,
}

#: A class joins the certified load in a state only at or above this measured reliability
#: (unmeasured pairs pass — the table carries every measured deficiency).
SETTLE_RELIABILITY_BAR = 0.90


@dataclass(frozen=True)
class PlannedPlacement:
    """One certified item of the full load.

    Attributes:
        item_id: Globally unique ``<class>_<nn>`` (scene prim name in the episode).
        object_class: Registry key in :data:`dishsim.config.OBJECTS`.
        instance: Per-class instance index (global across phases).
        state: Machine state this item is placed in.
        slot_id: Destination slot id within its class's slot table.
        slot_name: Geometry-derived slot name (``placement.slot_names`` vocabulary).
        mode: Effective placement mode.
        rack: Destination rack (``"lower" | "upper" | "third" | "basket"``).
        T_base_slot: Slot frame in the robot base frame, shape [4, 4].
        T_base_obj: Nominal resting pose used for joint certification, shape [4, 4].
        acquire: ``"pick"`` or ``"weld"`` (:data:`config.WELD_ACQUIRE_FAMILIES` — the two
            acquisition families' success rates are never summed).
        n_goal_configs: Baked goal configs surviving the neighbour re-filter at plan time.
    """

    item_id: str
    object_class: str
    instance: int
    state: str
    slot_id: int
    slot_name: str
    mode: str
    rack: str
    T_base_slot: np.ndarray
    T_base_obj: np.ndarray
    acquire: str
    n_goal_configs: int

    def to_json(self) -> dict:
        return {
            "item_id": self.item_id, "object_class": self.object_class,
            "instance": self.instance, "state": self.state, "slot_id": self.slot_id,
            "slot_name": self.slot_name, "mode": self.mode, "rack": self.rack,
            "T_base_slot": np.asarray(self.T_base_slot).tolist(),
            "T_base_obj": np.asarray(self.T_base_obj).tolist(),
            "acquire": self.acquire, "n_goal_configs": self.n_goal_configs,
        }


@dataclass
class PhaseCapacity:
    """One state's share of the load, with the funnel that explains the count."""

    state: str
    items: list[PlannedPlacement] = field(default_factory=list)
    #: Per class: slots_total, reachable (non-empty baked goal set), assigned, stopped_by
    #: (``"z-budget" | "overlap-or-blocked" | "cap" | None`` — None = never requested).
    funnel: dict[str, dict] = field(default_factory=dict)

    def counts(self) -> dict:
        out: dict[str, int] = {}
        for it in self.items:
            out[it.object_class] = out.get(it.object_class, 0) + 1
        return out


@dataclass
class CapacityPlan:
    machine: str
    base_placement: str
    policy: str
    config_hash_by_state: dict[str, str]
    classes: list[str]
    phases: list[PhaseCapacity]

    @property
    def total_items(self) -> int:
        return sum(len(p.items) for p in self.phases)

    def to_json(self) -> dict:
        return {
            "schema_version": CAPACITY_SCHEMA_VERSION,
            "machine": self.machine, "base_placement": self.base_placement,
            "policy": self.policy, "config_hash_by_state": self.config_hash_by_state,
            "classes": self.classes, "total_items": self.total_items,
            "phases": [
                {"state": p.state, "counts": p.counts(), "funnel": p.funnel,
                 "items": [it.to_json() for it in p.items]}
                for p in self.phases
            ],
        }


def capacity_classes() -> list[str]:
    """Robot-capable classes on the ACTIVE machine (calibrated demo classes; the legacy mug
    sits out of Bosch studies for the same reason as :func:`config.reference_class`)."""
    out = [name for name, spec in config.OBJECTS.items() if spec.robot_demo]
    if config.MACHINE == "bosch800":
        out = [c for c in out if c != "mug"]
    return out


def _rider_floor_z_w(state: str) -> float | None:
    """World z of the floor plane whose items RIDE a rack through this state's exit
    transition (None when the state has no loaded rack of its own)."""
    racks = config.MACHINE_GEN.get(config.MACHINE, {}).get("racks")
    if not racks:
        return None
    if state == "placement":
        zl = rack_gen._z_levels(config.RACK_GEN["E_shelf_1_04"])
        return racks["lower"]["rail_z"] + zl["floor_top"]
    if state == "middle_out":
        zl = rack_gen._z_levels(config.RACK_GEN["E_shelf_03"])
        return racks["middle"]["rail_z"] + zl["floor_top"]
    if state == "third_out":
        zl = rack_gen._tray_z_levels(config.RACK_GEN["E_shelf_third"])
        return racks["third"]["rail_z"] + zl["floor_top"]
    return None


def _overhead_datum_z_w(state: str) -> float | None:
    """World z of the lowest geometry passing over this state's riders on transition out."""
    racks = config.MACHINE_GEN.get(config.MACHINE, {}).get("racks")
    if not racks:
        return None
    if state == "placement":
        # stowing lower rack passes under the middle rack and its slung spray arm
        return racks["middle"]["rail_z"] - _MID_SPRAY_DROP_M
    if state == "middle_out":
        # stowing middle rack passes under the third rack's tray wires
        return racks["third"]["rail_z"]
    if state == "third_out":
        # Stowing third rack passes under the tub ceiling. The top spray head hangs 15 mm
        # below it but only over the tray's CENTER — the channel zone, whose floor drops
        # 28 mm — so the plain ceiling is the binding bound for riders in every zone.
        return config.MACHINE_GEN[config.MACHINE]["tub"]["ceiling_z"]
    return None


def _worst_top_m(object_class: str, mode: str) -> float:
    """Height [m] of the item's highest point above its floor plane at the worst pose the
    placement tolerances still accept."""
    spec = config.OBJECTS[object_class]
    mp = config.placement_mode_params(mode)
    if mode == "plate_slot":
        # a disc may settle anywhere inside the tilt tolerance; fully upright is the ceiling
        return 2.0 * spec.rim_radius_m + mp["tol_bottom_m"]
    if mode == "flat_lay_third":
        # a LYING piece: thickness, plus the long half-axis pitched to the tilt tolerance
        tilt = np.radians(mp["tol_tilt_deg"])
        return float(2.0 * min(spec.bbox_half)
                     + max(spec.bbox_half) * np.sin(tilt) + mp["tol_bottom_m"])
    tilt = np.radians(mp["tol_tilt_deg"])
    return float(spec.height_m * np.cos(tilt) + spec.rim_radius_m * np.sin(tilt)
                 + mp["tol_bottom_m"])


def z_budget_clearance_m(object_class: str, state: str, *, last_state: bool) -> float | None:
    """Worst-case clearance [m] between this class's top and the transition overhead.

    Returns None when no gate applies (final phase, or no machine rack model). Negative
    means the overhead geometry would strike a legally-placed item.
    """
    if last_state:
        return None
    floor, datum = _rider_floor_z_w(state), _overhead_datum_z_w(state)
    if floor is None or datum is None:
        return None
    mode = config.effective_placement_mode(object_class)
    return float(datum - (floor + _worst_top_m(object_class, mode)))


@dataclass
class _StateTables:
    """Per-class slot/goal tables for one state, loaded under that state's scenario."""

    config_hash: str
    slots: dict[str, dict[int, placement.SlotFrame]]
    goal_sets: dict[str, dict[int, np.ndarray]]
    slot_names: dict[str, dict[str, int]]


def load_state_tables(state: str, classes: list[str]) -> _StateTables:
    """Load ``slots.json`` + ``goal_sets.json`` per class, verifying each stamp against the
    live :func:`~dishsim.geometry.config_hash`. Must be called under
    ``config.apply_scenario(state)``."""
    slots, goal_sets, names = {}, {}, {}
    live_hash = None
    for cls in classes:
        cdir = config.scenario_cache_dir(state, object_name=cls)
        with config.active_object(cls):
            live_hash = config_hash()
        for fname in ("slots.json", "goal_sets.json"):
            path = os.path.join(cdir, "slots", fname)
            if not os.path.exists(path):
                raise FileNotFoundError(
                    f"missing {path} — bake it with: scripts/setup/build_state.py "
                    f"--machine {config.MACHINE} --placement {config.BASE_PLACEMENT} "
                    f"--state {state} --classes {cls}")
            stamped = json.load(open(path))
            if stamped.get("config_hash") != live_hash:
                raise RuntimeError(
                    f"{path}: config_hash {stamped.get('config_hash')} != live {live_hash} "
                    f"— the cache was baked under a different config/placement; rebake it")
            if fname == "slots.json":
                slots[cls] = {s["slot_id"]: placement.SlotFrame.from_json(s)
                              for s in stamped["slots"]}
            else:
                goal_sets[cls] = {g["slot_id"]: np.array(g["configs"])
                                  for g in stamped["goal_sets"]}
        names[cls] = placement.slot_names(list(slots[cls].values()))
    return _StateTables(live_hash or "", slots, goal_sets, names)


def _nominal_rest_pose(slot: placement.SlotFrame) -> np.ndarray:
    """The item's nominal settled pose at a slot (zero perturbation, zero hover), used as
    the neighbour obstacle during joint certification. Runs under the item's active
    object."""
    spin = placement._top_grasp_spin(slot) or 0.0
    return placement.object_pose_for_mode(slot, spin, np.zeros(2), np.zeros(2), 0.0)


def _class_streams(policy_states: tuple, classes: list[str]) -> list[list[str]]:
    """Policy entries -> round-robin streams, filtered to the available classes."""
    streams = []
    for entry in policy_states:
        group = [entry] if isinstance(entry, str) else list(entry)
        group = [c for c in group if c in classes]
        if group:
            streams.append(group)
    return streams


def plan_full_load(*, states: tuple[str, ...] | None = None, classes: list[str] | None = None,
                   policy: str = "plates_first",
                   per_class_cap: dict[str, int] | None = None,
                   log=print) -> CapacityPlan:
    """Grow the largest jointly-certified load, one item at a time.

    Args:
        states: Loading states in phase order; defaults per machine
            (:data:`LOADING_STATES`).
        classes: Class pool; defaults to :func:`capacity_classes`.
        policy: Key of :data:`POLICIES` — the class request order.
        per_class_cap: Optional hard cap per class (e.g. ``{"fork": 8}``).
        log: Progress sink (the script passes ``print``; tests pass a no-op).

    Returns:
        The plan; ``plan.total_items`` is the headline "full load = N".
    """
    states = states or LOADING_STATES.get(config.MACHINE, ("placement",))
    classes = list(classes or capacity_classes())
    caps = dict(per_class_cap or {})
    policy_table = POLICIES[policy]

    entry_scenario = config.SCENARIO_NAME
    counters: dict[str, int] = {}
    placed_by_class_total: dict[str, int] = {}
    phases: list[PhaseCapacity] = []
    hash_by_state: dict[str, str] = {}
    try:
        for si, state in enumerate(states):
            config.apply_scenario(state)
            phase = PhaseCapacity(state)
            streams = _class_streams(policy_table.get(state, ()), classes)
            phase_classes = sorted({c for g in streams for c in g})
            if not phase_classes:
                phases.append(phase)
                continue
            tables = load_state_tables(state, phase_classes)
            hash_by_state[state] = tables.config_hash

            for cls in phase_classes:
                phase.funnel[cls] = {
                    "slots_total": len(tables.slots[cls]),
                    "reachable": sum(1 for v in tables.goal_sets[cls].values() if len(v)),
                    "assigned": 0, "stopped_by": None,
                }

            # measured gates retire classes before any request (rules, not policy):
            # z-budget (transition overhead strike) and settle reliability (rack floor
            # cannot stand the class up dependably — see MEASURED_SETTLE_RELIABILITY)
            last = si == len(states) - 1
            for cls in list(phase_classes):
                clr = z_budget_clearance_m(cls, state, last_state=last)
                if clr is not None and clr < Z_BUDGET_MARGIN_M:
                    log(f"[INFO] {state}: {cls} excluded by z-budget "
                        f"(worst-case clearance {clr * 1000:.1f} mm < "
                        f"{Z_BUDGET_MARGIN_M * 1000:.0f} mm)")
                    phase.funnel[cls]["stopped_by"] = "z-budget"
                    streams = [[c for c in g if c != cls] for g in streams]
                    continue
                rel = MEASURED_SETTLE_RELIABILITY.get((state, cls))
                if rel is not None and rel < SETTLE_RELIABILITY_BAR:
                    log(f"[INFO] {state}: {cls} excluded by measured settle reliability "
                        f"({rel:.0%} < {SETTLE_RELIABILITY_BAR:.0%})")
                    phase.funnel[cls]["stopped_by"] = "settle-reliability"
                    streams = [[c for c in g if c != cls] for g in streams]
            streams = [g for g in streams if g]

            worlds: dict[str, CollisionWorld] = {}
            occupied: list[tuple[str, np.ndarray, float]] = []
            neighbours: list[tuple[str, list, np.ndarray]] = []

            def world_for(cls: str) -> CollisionWorld:
                if cls not in worlds:
                    merged = config.effective_placement_mode(cls) not in (
                        "basket_drop", "plate_slot", "flat_lay_third")
                    with config.active_object(cls):
                        w = CollisionWorld(
                            cache_dir=config.scenario_cache_dir(state, object_name=cls),
                            self_check=True, object_attached=True, merged_cluster=merged)
                    for name, pieces, T in neighbours:
                        w.add_object(name, pieces, T)
                    worlds[cls] = w
                return worlds[cls]

            for stream in streams:
                active = list(stream)
                while active:
                    for cls in list(active):
                        if caps.get(cls) is not None and \
                                placed_by_class_total.get(cls, 0) >= caps[cls]:
                            phase.funnel[cls]["stopped_by"] = "cap"
                            active.remove(cls)
                            continue
                        item = _try_place_one(cls, state, tables, world_for(cls),
                                              occupied, counters)
                        if item is None:
                            phase.funnel[cls]["stopped_by"] = "overlap-or-blocked"
                            active.remove(cls)
                            continue
                        with config.active_object(cls):
                            pieces = load_object_pieces(
                                config.scenario_cache_dir(state, object_name=cls))
                        neighbours.append((item.item_id, pieces, item.T_base_obj))
                        for w in worlds.values():
                            w.add_object(item.item_id, pieces, item.T_base_obj)
                        occupied.append((item.mode, item.T_base_slot[:3, 3],
                                         float(config.OBJECTS[cls].rim_radius_m)))
                        phase.items.append(item)
                        phase.funnel[cls]["assigned"] += 1
                        placed_by_class_total[cls] = placed_by_class_total.get(cls, 0) + 1
                        log(f"[INFO] {state}: {item.item_id} -> {item.slot_name} "
                            f"({item.n_goal_configs} joint-certified configs)")
            phases.append(phase)
    finally:
        config.apply_scenario(entry_scenario)

    return CapacityPlan(config.MACHINE, config.BASE_PLACEMENT, policy, hash_by_state,
                        classes, phases)


def _try_place_one(cls: str, state: str, tables: _StateTables, world: CollisionWorld,
                   occupied: list, counters: dict) -> PlannedPlacement | None:
    """First candidate slot whose baked goal set survives the neighbour re-filter."""
    mode = config.effective_placement_mode(cls)
    ids = slotting.candidate_slot_ids(
        cls, mode, tables.slots[cls], tables.slot_names[cls], tables.goal_sets[cls],
        type_slots=config.TASK.get("type_slots") or {}, slot_pools=config.TASK["slot_pools"])
    r = float(config.OBJECTS[cls].rim_radius_m)
    for sid in ids:
        slot = tables.slots[cls][sid]
        centre = slot.T_base_slot[:3, 3]
        if any(slotting.occupancy_conflict(m, c, rr, mode, centre, r,
                                           float(config.TASK["slot_separation_margin_m"]))
               for m, c, rr in occupied):
            continue
        # The exact live re-filter primitives._place runs: baked configs vs the world with
        # every earlier item resting at its goal. Bounded on purpose: acceptance needs ONE
        # survivor, and reachable-rich bakes carry thousands of configs per slot (the runtime
        # planner consumes at most GOALS_PER_PLAN=64) — counting them all made the plan take
        # minutes per item for a number nothing reads past "plenty".
        n_surviving, scanned = 0, 0
        for q in tables.goal_sets[cls][sid]:
            scanned += 1
            if not world.in_collision(q):
                n_surviving += 1
                if n_surviving >= _CERTIFY_CONFIG_CAP:
                    break
            if scanned >= _CERTIFY_SCAN_CAP and n_surviving > 0:
                break
        if not n_surviving:
            continue
        with config.active_object(cls):
            T_obj = _nominal_rest_pose(slot)
        n = counters.get(cls, 0)
        counters[cls] = n + 1
        names_inv = {v: k for k, v in tables.slot_names[cls].items()}
        spec = config.OBJECTS[cls]
        acquire = "weld" if spec.grasp.family in config.WELD_ACQUIRE_FAMILIES else "pick"
        return PlannedPlacement(
            item_id=f"{cls}_{n:02d}", object_class=cls, instance=n, state=state,
            slot_id=sid, slot_name=str(names_inv.get(sid, sid)), mode=mode, rack=slot.rack,
            T_base_slot=np.asarray(slot.T_base_slot), T_base_obj=T_obj, acquire=acquire,
            n_goal_configs=n_surviving)
    return None


def write_plan(plan: CapacityPlan, path: str, extra: dict | None = None) -> str:
    """Serialize the plan (atomic tmp+rename); ``extra`` merges top-level keys (figure
    path, generation stamp)."""
    payload = plan.to_json()
    payload.update(extra or {})
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=1)
    os.replace(tmp, path)
    return path


def render_reachability_figure(plan: CapacityPlan, tables_by_state: dict[str, _StateTables],
                               out_png: str) -> str:
    """Top-down reach map, one panel per loading state, in the WORLD frame.

    Filled dot = slot with a baked goal config (reachable), hollow circle = unreachable,
    dark ring = assigned by the plan — the states differ by fill and ring, never by color
    alone. The counter panel band (``TASK["spawn_rect_w"]``) and robot base anchor the
    geometry.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation

    ink, muted, series = "#1f1f1f", "#9a9a9a", "#2a78d6"
    T_w_base = np.eye(4)
    T_w_base[:3, :3] = Rotation.from_quat(config.ROBOT_BASE_QUAT_W).as_matrix()  # XYZW
    T_w_base[:3, 3] = config.ROBOT_BASE_POS_W

    fig, axes = plt.subplots(1, len(plan.phases), figsize=(5.2 * len(plan.phases), 5.6),
                             sharex=True, sharey=True)
    axes = np.atleast_1d(axes)
    rect = config.TASK["spawn_rect_w"]
    for ax, phase in zip(axes, plan.phases):
        tables = tables_by_state[phase.state]
        assigned = {(it.object_class, it.slot_id) for it in phase.items}
        n_reach = n_total = 0
        for cls in sorted(tables.slots):
            for sid, slot in tables.slots[cls].items():
                p = (T_w_base @ np.append(slot.T_base_slot[:3, 3], 1.0))[:2]
                reachable = len(tables.goal_sets[cls].get(sid, ())) > 0
                n_total += 1
                n_reach += bool(reachable)
                if (cls, sid) in assigned:
                    ax.scatter(*p, s=64, facecolor=series, edgecolor=ink, linewidth=1.6,
                               zorder=4)
                elif reachable:
                    ax.scatter(*p, s=26, facecolor=series, edgecolor="none", alpha=0.75,
                               zorder=3)
                else:
                    ax.scatter(*p, s=22, facecolor="none", edgecolor=muted, linewidth=0.9,
                               zorder=2)
        ax.add_patch(plt.Rectangle((rect["x_min"], rect["y_min"]),
                                   rect["x_max"] - rect["x_min"],
                                   rect["y_max"] - rect["y_min"],
                                   facecolor="none", edgecolor=muted, linestyle="--",
                                   linewidth=1.0))
        ax.plot(*config.ROBOT_BASE_POS_W[:2], marker="s", markersize=7, color=ink)
        counts = ", ".join(f"{k} {v}" for k, v in sorted(phase.counts().items())) or "none"
        ax.set_title(f"{phase.state} — {len(phase.items)} placed ({counts})",
                     fontsize=10, color=ink)
        ax.text(0.02, 0.02, f"{n_reach}/{n_total} slot-class pairs reachable",
                transform=ax.transAxes, fontsize=8, color=muted)
        ax.set_aspect("equal")
        ax.set_xlabel("world x [m]", fontsize=8, color=ink)
        ax.tick_params(labelsize=7, colors=muted)
        for s in ax.spines.values():
            s.set_color(muted)
    axes[0].set_ylabel("world y [m]", fontsize=8, color=ink)
    handles = [
        plt.Line2D([], [], marker="o", ls="", mfc=series, mec="none", ms=6,
                   label="reachable (goal configs baked)"),
        plt.Line2D([], [], marker="o", ls="", mfc="none", mec=muted, ms=6,
                   label="unreachable"),
        plt.Line2D([], [], marker="o", ls="", mfc=series, mec=ink, mew=1.6, ms=9,
                   label="assigned in the full load"),
        plt.Line2D([], [], ls="--", color=muted, label="counter pick band"),
        plt.Line2D([], [], marker="s", ls="", color=ink, ms=6, label="robot base"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=5, fontsize=8, frameon=False)
    fig.suptitle(f"UR5e reachable capacity — {plan.machine} @ {plan.base_placement}: "
                 f"full load = {plan.total_items} items", fontsize=12, color=ink)
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=160, facecolor="white")
    plt.close(fig)
    return out_png
