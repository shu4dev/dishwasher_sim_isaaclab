# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pick-order cost functions — how the sequencer ranks the currently-pickable objects.

The sequencer never contains an ordering rule; it asks a cost function to score each candidate
and takes the cheapest. Swapping the heuristic is a config change (``config.TASK["cost_fn"]``)
or a ``--cost_fn`` flag, exactly like swapping a planner.

Adding one is the same two-step change the planner registry uses, and touches no sequencer code:

1. Write a function with the :class:`CostFn` signature below.
2. Add it to :data:`COST_FUNCTIONS`.

A cost function is deliberately weak: it sees a candidate and a snapshot of the world, returns
a float, and lower wins. It may not mutate anything, and it is never asked about *feasibility* —
whether an object can be picked at all is decided by the support graph (Stage B) and the grasp
checker (Stage C) before ranking ever happens. Keeping those concerns apart is what lets the
cost function be swapped without changing which layouts are solvable.
"""

from typing import Callable, Protocol

import numpy as np


class CostCandidate(Protocol):
    """What a cost function is allowed to see about one pickable object.

    Attributes:
        item_id: Stable per-episode identifier, e.g. ``"mug_00"``.
        object_class: Registry key in :data:`dishsim.config.OBJECTS`.
        T_base_obj: Measured object pose in the robot-base frame, shape [4, 4].
        T_base_grasp: Candidate grasp TCP pose in the robot-base frame, shape [4, 4].
        grasp_q: Arm configuration achieving the grasp [rad], shape [6], or ``None``.
    """

    item_id: str
    object_class: str
    T_base_obj: np.ndarray
    T_base_grasp: np.ndarray
    grasp_q: np.ndarray | None


#: A cost function: ``(candidate, current_q, current_T_base_tcp) -> float`` (lower is picked
#: first). ``current_q`` is the arm's present configuration and ``current_T_base_tcp`` its
#: present tool pose, so a heuristic may reason in joint space, Cartesian space, or neither.
CostFn = Callable[[CostCandidate, np.ndarray, np.ndarray], float]


def nearest_first(candidate: CostCandidate, current_q: np.ndarray, current_T_base_tcp: np.ndarray) -> float:
    """Cartesian distance from the current tool position to the candidate's grasp point [m].

    The default. Cheap, stable, and independent of IK branch choice — two candidates at the
    same place score the same however their arm configurations differ.
    """
    return float(np.linalg.norm(current_T_base_tcp[:3, 3] - candidate.T_base_grasp[:3, 3]))


def shortest_ik(candidate: CostCandidate, current_q: np.ndarray, current_T_base_tcp: np.ndarray) -> float:
    """Joint-space distance from the current configuration to the candidate's grasp [rad].

    Closer to what the arm actually pays than :func:`nearest_first`, because a short Cartesian
    hop across a wrist flip can be a long joint-space move. Candidates with no known grasp
    configuration fall back to the Cartesian cost so they stay rankable.
    """
    if candidate.grasp_q is None:
        return nearest_first(candidate, current_q, current_T_base_tcp)
    return float(np.linalg.norm(np.asarray(candidate.grasp_q) - np.asarray(current_q)))


def farthest_first(candidate: CostCandidate, current_q: np.ndarray, current_T_base_tcp: np.ndarray) -> float:
    """Inverse of :func:`nearest_first` — clears the far field first.

    Present mainly so the swappability of the ordering layer is demonstrable rather than
    asserted: it produces a strictly different pick order on the same scene.
    """
    return -nearest_first(candidate, current_q, current_T_base_tcp)


#: Registry key -> cost function. The key is what ``config.TASK["cost_fn"]`` and ``--cost_fn``
#: accept, and what is recorded in every episode's provenance block.
COST_FUNCTIONS: dict[str, CostFn] = {
    "nearest_first": nearest_first,
    "shortest_ik": shortest_ik,
    "farthest_first": farthest_first,
}


def available_costs() -> list[str]:
    """Registered cost-function names, sorted."""
    return sorted(COST_FUNCTIONS)


def make_cost(name: str) -> CostFn:
    """Look up a registered cost function.

    Args:
        name: Registry key (see :func:`available_costs`).

    Returns:
        The cost function.
    """
    if name not in COST_FUNCTIONS:
        raise ValueError(f"unknown cost function {name!r} (choices: {available_costs()})")
    return COST_FUNCTIONS[name]
