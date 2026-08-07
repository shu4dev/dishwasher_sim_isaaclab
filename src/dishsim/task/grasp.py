# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""State-dependent grasp availability (Stage C).

Whether an object can be picked is a property of the world *right now*, not of the object. The
same mug is graspable alone on the counter and ungraspable once something is placed beside it,
and it becomes graspable again the moment that neighbour is cleared. So nothing here is
precomputed: every candidate is re-derived against the current collision world, on every
iteration of the sequencer's loop.

Two things are checked, and the second is the one that matters with neighbours:

* the **pre-grasp hover** must have a collision-free arm configuration — what the single-object
  runner checks;
* the **descent** beneath it must stay collision-free too. A hover can be perfectly reachable
  while the forearm sweeps through the object next to it on the way down. Measured before this
  existed: 56–60 N contacts mid-descent at poses the hover check had approved.

The object being grasped is removed from the world for the duration of the check. It has to be —
it is registered as an obstacle exactly like every other item, and an object must not be allowed
to veto its own grasp. The final stretch of the descent is also excluded, because at the grasp
pose the pads are *inside* the object's inflated hull by design; checking there would reject
every valid grasp.

When the nominal grasp is blocked, a yaw sweep looks for another way in. Rotating the tool about
the object's own axis leaves the pinch geometry identical for the rotationally symmetric classes
this task uses, so an alternative yaw is a genuinely equivalent grasp rather than a compromise —
and it is very often the difference between blocked and pickable when a neighbour is on one side.

Every rejection is counted. "Unpickable" that cannot say *why* is indistinguishable from a bug,
so :class:`GraspFunnel` mirrors what :class:`dishsim.placement.GoalSet` records for goal
generation: how many candidates were tried, how many died at IK, and how many at collision.
"""

from dataclasses import dataclass, field

import numpy as np

from .. import config
from ..transforms import T_inv
from .sequencer import GraspCandidate


@dataclass
class GraspFunnel:
    """Why a grasp search ended where it did."""

    n_candidates: int = 0
    n_no_ik: int = 0
    n_hover_blocked: int = 0
    n_descent_blocked: int = 0
    accepted_yaw_deg: float | None = None
    tried_yaw_deg: list = field(default_factory=list)

    @property
    def found(self) -> bool:
        return self.accepted_yaw_deg is not None

    def reason(self) -> str:
        """One line explaining a failure, in the terms the sequencer reports."""
        if self.found:
            return f"grasp found at yaw {self.accepted_yaw_deg:.0f} deg"
        if self.n_candidates == 0:
            return "no grasp candidates were generated"
        if self.n_no_ik == self.n_candidates:
            return f"no IK solution at any of {self.n_candidates} grasp yaws (out of reach)"
        if self.n_descent_blocked:
            return (f"all {self.n_candidates} grasp yaws blocked "
                    f"({self.n_hover_blocked} at the hover, {self.n_descent_blocked} on the descent)")
        return f"all {self.n_candidates} grasp yaws blocked at the pre-grasp hover"

    def to_json(self) -> dict:
        return {
            "candidates": self.n_candidates,
            "no_ik": self.n_no_ik,
            "hover_blocked": self.n_hover_blocked,
            "descent_blocked": self.n_descent_blocked,
            "accepted_yaw_deg": self.accepted_yaw_deg,
        }


def yaw_sweep(n: int) -> list:
    """``n`` yaw offsets [deg], nominal first then outward in alternating directions.

    Ordered so the nominal grasp is always tried first and the search widens symmetrically —
    the first acceptable candidate is then the one closest to the calibrated pose, not an
    arbitrary one that happens to come first in a linear scan.
    """
    if n <= 1:
        return [0.0]
    step = 360.0 / n
    out = [0.0]
    for k in range(1, n // 2 + 1):
        out.append(k * step)
        if len(out) < n:
            out.append(-k * step)
    return out[:n]


def find_grasp(item, profile, motion, *, n_yaws: int | None = None,
               descent_samples=(0.0, 0.25, 0.5, 0.75)) -> tuple:
    """Find a collision-free grasp for ``item`` in the CURRENT world.

    Args:
        item: The object, carrying a measured ``T_base_obj``.
        profile: Its :class:`~dishsim.task.primitives.GraspProfile` (grasp transform, geometry).
        motion: The motion layer, for IK, validity queries and world mutation.
        n_yaws: Grasp yaws to try; defaults to ``config.TASK["grasp_yaw_samples"]``.
        descent_samples: Fractions of the hover-to-grasp travel to validate.

    Returns:
        ``(candidate, funnel)``; ``candidate`` is ``None`` when the object is not pickable now.
    """
    n_yaws = int(config.TASK["grasp_yaw_samples"] if n_yaws is None else n_yaws)
    T_obj = np.asarray(item.T_base_obj, dtype=float)
    funnel = GraspFunnel()
    # A weld-acquired class is snapped to the carry transform at the hover and never descends, so
    # validating a descent for it would veto grasps on the strength of a motion that never runs.
    if getattr(profile, "acquire", "pick") == "weld":
        descent_samples = ()

    # An object cannot be allowed to veto its own grasp: it is an obstacle like every other item,
    # and the gripper is meant to close on it.
    had_obstacle = motion.world.has_object(item.item_id)
    if had_obstacle:
        motion.clear_obstacle(item.item_id)
    try:
        for yaw_deg in yaw_sweep(n_yaws):
            funnel.n_candidates += 1
            funnel.tried_yaw_deg.append(float(yaw_deg))
            T_grasp = T_obj @ _yaw_about_object_axis(yaw_deg) @ T_inv(profile.T_tcp_obj)
            T_hover = T_grasp.copy()
            T_hover[2, 3] += config.PICK_HOVER_M

            sols = motion.reachable(T_hover, q_seed=motion.current_q(), check_collision=False)
            if len(sols) == 0:
                funnel.n_no_ik += 1
                continue
            free = [q for q in sols if not motion.world.in_collision(q)]
            if not free:
                funnel.n_hover_blocked += 1
                continue
            if descent_samples:
                q = _descent_clear(motion, T_hover, T_grasp, free, descent_samples)
                if q is None:
                    funnel.n_descent_blocked += 1
                    continue
            else:
                q = free[0]     # weld-acquire: the hover IS the working pose, there is no descent
            funnel.accepted_yaw_deg = float(yaw_deg)
            return GraspCandidate(item_id=item.item_id, object_class=item.object_class,
                                  T_base_obj=T_obj, T_base_grasp=T_grasp, grasp_q=q), funnel
    finally:
        if had_obstacle:
            motion.set_obstacle(item.item_id, profile.pieces, T_obj)
    return None, funnel


def _yaw_about_object_axis(yaw_deg: float) -> np.ndarray:
    """Rotation about the object's own +z, as a homogeneous transform.

    Applied in the OBJECT frame, so the tool orbits the object rather than the base — which is
    what makes the alternative an equivalent grasp for a rotationally symmetric class.
    """
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", float(yaw_deg), degrees=True).as_matrix()
    return T


def _descent_clear(motion, T_hover, T_grasp, hover_sols, samples):
    """First hover configuration whose descent stays collision-free, or ``None``."""
    for q_hover in hover_sols:
        seg = motion.line_configs(T_hover, T_grasp, len(samples) + 2, q_hover)
        if seg is None:
            continue
        n_check = int(round(samples[-1] * (len(seg) - 1))) + 1
        if not any(motion.world.in_collision(q) for q in seg[:n_check]):
            return q_hover
    return None


class GraspFinder:
    """Stateful grasp oracle: what the sequencer calls, and what recovery widens.

    Wrapping the search in an object rather than a bare function exists for one reason — the
    recovery ladder's cheapest rung is "look harder", and it needs somewhere to put that decision
    that survives to the next iteration of the loop. It also collects the per-item funnels, so a
    deadlock can explain itself object by object instead of just asserting that nothing worked.
    """

    def __init__(self, profiles, motion, *, n_yaws: int | None = None) -> None:
        self.profiles = profiles
        self.motion = motion
        self.n_yaws = int(config.TASK["grasp_yaw_samples"] if n_yaws is None else n_yaws)
        self.funnels: dict = {}

    def set_yaw_samples(self, n: int) -> None:
        """Widen (or narrow) the sweep for subsequent queries — the recovery ladder's rung 1."""
        self.n_yaws = int(n)

    def reasons(self) -> dict:
        """``{item_id: why it was not pickable}`` from the most recent probe of each item."""
        return {k: f.reason() for k, f in self.funnels.items() if not f.found}

    def __call__(self, item):
        candidate, funnel = find_grasp(item, self.profiles[item.object_class], self.motion,
                                       n_yaws=self.n_yaws)
        self.funnels[item.item_id] = funnel
        return candidate
