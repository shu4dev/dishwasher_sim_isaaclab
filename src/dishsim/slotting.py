# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Destination-slot candidacy — feasible, unused, and not overlapping another item.

This is the one packing rule in the repository, used by the capacity planner
(:mod:`dishsim.capacity`) to grow a jointly-placeable load one item at a time.

Two independent scarcities bite here, and both are measured rather than assumed.

**Placeability.** Only some slots accept a class at all — an empty ``goal_sets`` entry marks
a slot whose release pose collides in the empty machine (the capacity planner feeds its
placeable bool-map through this parameter; docs/success_criteria.md).

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
  particular pair of gaps is *jointly placeable* stays the collision layer's call (the
  capacity planner re-queries the release pose with neighbours attached).

**Preference.** ``type_slots`` gives an ORDERED list of slot NAMES per object type. Order is a
preference, not a permission: it decides which slot a type reaches for first, never whether a
slot is legal. Legality is still a non-empty feasibility entry plus no overlap with an
already-assigned slot, so a named slot that is infeasible for this class, or already taken,
is skipped rather than failing the object.

Deliberately pure: every config-derived value arrives as a parameter, so this module imports
nothing from ``config`` and stays testable without a scene.
"""

from typing import Mapping, Sequence

import numpy as np


def candidate_slot_ids(object_class: str, mode: str, slots: Mapping[int, object],
                       slot_names: Mapping[str, int], goal_sets: Mapping[int, Sequence],
                       *, type_slots: Mapping[str, Sequence[str]] | None,
                       slot_pools: Mapping[str, Sequence[int] | None]) -> list[int]:
    """Ordered candidate slot ids for one class: preference list, then feasibility filter.

    Args:
        object_class: The item's registry class.
        mode: The class's EFFECTIVE placement mode (machine overrides applied).
        slots: ``{slot_id: SlotFrame}`` for this class.
        slot_names: Geometry-derived ``{name: slot_id}`` for this class.
        goal_sets: ``{slot_id: entries}``; an empty entry means not placeable.
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
