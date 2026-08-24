# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Destination-slot assignment — feasible, unused, and not overlapping another item.

This is the one packing rule in the repository, shared by the episode runner (which assigns
slots to spawned items) and the capacity planner (which asks how many items the machine can
hold at all). Sharing it is the point: a "full load = N" number computed by the planner is only
defensible if N is exactly what the runner's assignment would hand out.

Two independent scarcities bite here, and both are measured rather than assumed.

**Reachability.** Only some slots have any goal configurations at all — empty goal sets are
expected signal (docs/success_criteria.md), not a bug: they mark rack regions the carried
object cannot be brought into without hitting the machine.

**Overlap.** The slot grids are *overlapping candidate* grids, laid out at
``SLOT_GRID_PITCH_M`` = 60 mm for a task that places ONE object per trial and wants dense
coverage. Two objects assigned to adjacent candidates would be told to occupy the same space.
Compatibility is mode-aware (:func:`occupancy_conflict`):

- Standing/lying modes check each class's body radius (``rim_radius_m`` — the standing
  circle, not the bbox diagonal, so a mug's handle does not veto a neighbour it would simply
  be turned away from) against every already-assigned slot.
- Two ``plate_slot`` gaps never conflict with each other: a tine bank exists to hold parallel
  discs one gap apart, so the rack pitch — not the disc radius — is the separation. The old
  circle rule read a plate as a 70 mm sphere and forbade adjacent gaps outright. Whether a
  particular pair of gaps is *jointly reachable* stays the goal-set/collision layer's call
  (the capacity planner re-filters goal configs with neighbours attached).

**Preference.** ``type_slots`` gives an ORDERED list of slot NAMES per object type. Order is a
preference, not a permission: it decides which slot a type reaches for first, never whether a
slot is legal. Legality is still a non-empty goal set plus no overlap with an already-assigned
slot, so a named slot that is infeasible for this class, or already taken, is skipped rather
than failing the object.

Items are served most-constrained-first, the standard heuristic, and that stays load-bearing
even with explicit lists — a list makes a type MORE constrained, not less. Measured: the mug
can use 2 slots and the cup 4, so serving the mug first gets both placed; reverse it and the
cup takes the mug's slot and the mug gets nothing. Items left without a slot are returned
absent, and the caller reports them rather than silently dropping them.

Deliberately pure: every config-derived value arrives as a parameter (the runner passes
``config.effective_placement_mode`` and registry radii; the capacity planner passes the same),
so this module imports nothing from ``config`` and stays testable without a scene.
"""

from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class SlotRequest:
    """One item asking for a destination slot.

    Attributes:
        item_id: Stable identifier, e.g. ``"cup_00"``; the assignment is keyed by it.
        object_class: Registry key in :data:`dishsim.config.OBJECTS`.
    """

    item_id: str
    object_class: str


def candidate_slot_ids(object_class: str, mode: str, slots: Mapping[int, object],
                       slot_names: Mapping[str, int], goal_sets: Mapping[int, Sequence],
                       *, type_slots: Mapping[str, Sequence[str]] | None,
                       slot_pools: Mapping[str, Sequence[int] | None]) -> list[int]:
    """Ordered candidate slot ids for one class: preference list, then reachability filter.

    Args:
        object_class: The item's registry class.
        mode: The class's EFFECTIVE placement mode (machine overrides applied).
        slots: ``{slot_id: SlotFrame}`` for this class.
        slot_names: Geometry-derived ``{name: slot_id}`` for this class.
        goal_sets: ``{slot_id: configs}``; an empty entry means unreachable.
        type_slots: Optional ``{class: ordered slot names}`` preference table.
        slot_pools: Optional ``{mode: slot ids}`` pool table.

    Returns:
        Feasible candidate ids in preference order (``type_slots`` preference table, else
        the mode's ``slot_pools`` entry, else every slot).

    Raises:
        SystemExit: A ``type_slots`` entry names a slot that does not exist for this mode.
    """
    names = (type_slots or {}).get(object_class)
    pool = (slot_pools or {}).get(mode)
    # `is not None`, not truthiness: an EMPTY list is a deliberate "this type has nowhere to
    # go", and must not silently fall through to every slot in the rack.
    if names is not None:
        unknown = [n for n in names if n not in slot_names]
        if unknown:
            raise SystemExit(
                f"[FAIL] TASK['type_slots'][{object_class!r}] names {unknown}, which do not "
                f"exist for placement mode {mode!r}. Valid names: {sorted(slot_names)}")
        ids = [slot_names[n] for n in names]
    elif pool is not None:
        ids = list(pool)
    else:
        ids = sorted(slots)
    return [s for s in ids if len(goal_sets.get(s, ())) > 0]


def occupancy_conflict(mode_a: str, centre_a: np.ndarray, radius_a: float,
                       mode_b: str, centre_b: np.ndarray, radius_b: float,
                       margin_m: float) -> bool:
    """Would two assigned slots tell their objects to occupy the same space?

    Standing circles by default; two ``plate_slot`` gaps are separated by the tine bank's own
    pitch and never conflict (see the module docstring — this is the documented v2 change that
    makes adjacent-gap plate loads assignable). A plate against a *standing* neighbour keeps
    the conservative circle rule.

    Args:
        mode_a: Effective placement mode of the already-assigned slot.
        centre_a: Slot centre [m] in the robot-base frame, shape [3].
        radius_a: Occupying class's body radius [m].
        mode_b: Effective placement mode of the candidate slot.
        centre_b: Candidate slot centre [m], shape [3].
        radius_b: Candidate class's body radius [m].
        margin_m: Extra separation margin [m].
    """
    dist = float(np.linalg.norm(np.asarray(centre_a) - np.asarray(centre_b)))
    if mode_a == "plate_slot" and mode_b == "plate_slot":
        # distinct gaps never conflict; the SAME gap (byte-equal centre from the same
        # derivation, so 0.1 mm is generous) is still one receptacle
        return dist < 1e-4
    return dist < (radius_a + radius_b + margin_m)


def assign_slots(requests: Sequence[SlotRequest], slots_by_class: Mapping[str, Mapping[int, object]],
                 goal_sets_by_class: Mapping[str, Mapping[int, Sequence]],
                 slot_names_by_class: Mapping[str, Mapping[str, int]], *,
                 type_slots: Mapping[str, Sequence[str]] | None,
                 slot_pools: Mapping[str, Sequence[int] | None],
                 margin_m: float, mode_fn: Callable[[str], str],
                 rim_radius_fn: Callable[[str], float],
                 taken: Sequence[tuple[str, np.ndarray, float]] = ()) -> dict[str, object]:
    """Assign each request a destination slot; requests left without one are simply absent.

    Args:
        requests: Items wanting slots.
        slots_by_class: Per class, ``{slot_id: SlotFrame}``.
        goal_sets_by_class: Per class, ``{slot_id: configs}``.
        slot_names_by_class: Per class, ``{name: slot_id}``.
        type_slots: Optional per-class ordered slot-name preference.
        slot_pools: Optional per-mode slot-id pools.
        margin_m: Slot separation margin [m].
        mode_fn: Class -> EFFECTIVE placement mode (pass
            :func:`dishsim.config.effective_placement_mode`).
        rim_radius_fn: Class -> body radius [m].
        taken: Already-occupied ``(mode, centre, radius)`` triples (the capacity planner grows
            a load one item at a time and threads its occupancy through here).

    Returns:
        ``{item_id: SlotFrame}`` for every request that got a slot.
    """
    feasible = {}
    for req in requests:
        cls = req.object_class
        feasible[req.item_id] = candidate_slot_ids(
            cls, mode_fn(cls), slots_by_class[cls], slot_names_by_class[cls],
            goal_sets_by_class[cls], type_slots=type_slots, slot_pools=slot_pools)

    out, occupied = {}, list(taken)
    for req in sorted(requests, key=lambda r: (len(feasible[r.item_id]), r.item_id)):
        mode, r = mode_fn(req.object_class), float(rim_radius_fn(req.object_class))
        for sid in feasible[req.item_id]:
            slot = slots_by_class[req.object_class][sid]
            centre = slot.T_base_slot[:3, 3]
            if any(occupancy_conflict(m, c, rr, mode, centre, r, margin_m)
                   for m, c, rr in occupied):
                continue
            occupied.append((mode, centre, r))
            out[req.item_id] = slot
            break
    return out
