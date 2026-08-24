# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The task sequencer: which object to pick next, and in what order.

This is the only module that decides anything about the *task*. It never plans a motion, never
touches a collision world, and never imports :mod:`dishsim.planners` — it asks the motion layer
to move, and asks injected callables about feasibility. ``tests/test_layer_boundary.py`` holds
that line.

Every stage-specific rule is a constructor argument rather than a branch in the loop, so the
Stage B and Stage C work slots in without editing the control flow that Stage A validates:

===========  =========================  ==========================================
Stage        argument                   effect on ``pickable()``
===========  =========================  ==========================================
A            (none)                     everything still on the counter is pickable
B            ``support_fn``             an object under another is not pickable
C            ``grasp_fn``               an object with no collision-free grasp is not
C            ``recovery``               what to try when nothing is pickable
===========  =========================  ==========================================

The loop re-reads measured world state before *every* pick and rebuilds the support graph from
it. Planning more than one pick ahead on a snapshot is the tempting optimisation and it is
wrong: removing an object lets its neighbours shift, so any grasp computed against the old
snapshot is aimed at where the object used to be.
"""

from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from .cost import CostFn, make_cost
from .episode import EpisodeResult, PickRecord


@dataclass
class TaskItem:
    """One object the episode has to clear.

    Attributes:
        item_id: Stable per-episode identifier, ``"<class>_<nn>"``.
        object_class: Registry key in :data:`dishsim.config.OBJECTS`.
        instance: Scene prim instance index.
        T_base_obj: Last measured pose in the robot-base frame, shape [4, 4].
        goal_slot: Allocated destination slot, or ``None`` until allocation.
        state: ``"pending"`` | ``"placed"`` | ``"failed"``.
    """

    item_id: str
    object_class: str
    instance: int
    T_base_obj: np.ndarray
    goal_slot: object | None = None
    state: str = "pending"


@dataclass
class GraspCandidate:
    """A concrete way to pick one item — what the cost function ranks.

    Matches the :class:`~dishsim.task.cost.CostCandidate` protocol.
    """

    item_id: str
    object_class: str
    T_base_obj: np.ndarray
    T_base_grasp: np.ndarray
    grasp_q: np.ndarray | None = None


class PickPlaceFn(Protocol):
    """Executes one pick-and-place. Supplied by the runner; see :mod:`dishsim.task.primitives`."""

    def __call__(self, item: TaskItem, candidate: GraspCandidate) -> "PickOutcome":
        ...


@dataclass
class PickOutcome:
    """What a pick-and-place attempt produced."""

    success: bool
    failure_stage: str | None = None
    failure_detail: str | None = None
    plan_time_s: float = 0.0
    n_replans: int = 0


#: ``(items) -> {item_id: {ids resting ON it}}``. Stage B.
SupportFn = Callable[[list], dict]
#: ``(item) -> GraspCandidate | None``. Returning ``None`` means "not pickable right now".
#: Stage A supplies a purely geometric version; Stage C makes it state-dependent.
GraspFn = Callable[[TaskItem], GraspCandidate | None]


class TaskSequencer:
    """Clears a set of objects into the machine, deciding order as it goes.

    Args:
        items: Objects to clear.
        motion: The motion layer (used only via the injected primitives and for ``current_q``).
        pick_place: Executes one pick-and-place.
        grasp_fn: Produces a grasp candidate for an item, or ``None`` if not pickable now.
        slot_fn: ``(item) -> slot`` allocating a destination; may return ``None``.
        cost_fn: Ordering heuristic; a name or a callable. Defaults to ``config.TASK["cost_fn"]``.
        refresh_fn: ``() -> {item_id: T_base_obj}`` re-reading measured world state.
        support_fn: Stage B support graph.
        recovery: Stage C ``(sequencer, remaining) -> bool``; True means "try again".
        max_recoveries: Cap on recovery invocations.
        on_event: Optional progress callback ``(event_name, payload)``.
    """

    def __init__(self, items, motion, *, pick_place: PickPlaceFn, grasp_fn: GraspFn,
                 slot_fn=None, cost_fn=None, refresh_fn=None, support_fn: SupportFn | None = None,
                 recovery=None, max_recoveries: int = 0, on_event=None) -> None:
        from .. import config  # noqa: PLC0415

        self.items = list(items)
        self.motion = motion
        self.pick_place = pick_place
        self.grasp_fn = grasp_fn
        self.slot_fn = slot_fn
        self.refresh_fn = refresh_fn
        self.support_fn = support_fn
        self.recovery = recovery
        self.max_recoveries = int(max_recoveries)
        self.on_event = on_event
        name = cost_fn if cost_fn is not None else config.TASK["cost_fn"]
        self.cost_name = name if isinstance(name, str) else getattr(name, "__name__", "custom")
        self.cost_fn: CostFn = make_cost(name) if isinstance(name, str) else name
        self.support_graph: dict = {}
        self.blocked_reasons: dict = {}
        self._n_recoveries = 0

    # ---- state -------------------------------------------------------------------------------

    def remaining(self) -> list:
        """Items still on the countertop."""
        return [it for it in self.items if it.state == "pending"]

    def refresh(self) -> None:
        """Re-read measured poses and rebuild the support graph.

        Called before every pick. Objects shift when their neighbours are removed, so both the
        poses and the graph are derived from what the simulator reports now, never carried over.
        """
        if self.refresh_fn is not None:
            poses = self.refresh_fn()
            by_id = {it.item_id: it for it in self.items}
            for item_id, T in poses.items():
                if item_id in by_id:
                    by_id[item_id].T_base_obj = np.asarray(T, dtype=float)
        self.support_graph = self.support_fn(self.remaining()) if self.support_fn else {}

    def pickable(self) -> list:
        """Currently pickable items, with a grasp candidate for each.

        Two independent gates, both re-evaluated every iteration:

        * **support** (Stage B) — an object with something resting on it is not pickable, no
          matter how good its grasp looks;
        * **grasp** (Stage C) — an object with no collision-free grasp in the CURRENT world is
          not pickable, even with nothing on top of it.

        Returns:
            ``[(item, candidate)]``. Reasons for exclusions land in :attr:`blocked_reasons`.
        """
        self.blocked_reasons = {}
        out = []
        for item in self.remaining():
            resting_on_it = self.support_graph.get(item.item_id, ())
            if resting_on_it:
                self.blocked_reasons[item.item_id] = (
                    f"supports {sorted(resting_on_it)} — must be cleared first"
                )
                continue
            candidate = self.grasp_fn(item)
            if candidate is None:
                self.blocked_reasons[item.item_id] = "no collision-free grasp in the current world"
                continue
            out.append((item, candidate))
        return out

    def choose(self, pickable):
        """Cheapest candidate under the ordering heuristic.

        Ties break on ``item_id`` so a given world state always yields the same choice —
        without that, an episode would not be reproducible from its seed.
        """
        q = self.motion.current_q()
        T_tcp = self._current_tcp(q)
        scored = [(float(self.cost_fn(cand, q, T_tcp)), item.item_id, item, cand)
                  for item, cand in pickable]
        scored.sort(key=lambda t: (t[0], t[1]))
        cost, _, item, cand = scored[0]
        return item, cand, cost

    def _current_tcp(self, q) -> np.ndarray:
        """Current TCP pose, for cost functions that reason in Cartesian space."""
        tcp_pose = getattr(self.motion, "tcp_pose", None)
        if tcp_pose is not None:
            return tcp_pose(q)
        from ..ur5e_kin import fk_wrist3  # noqa: PLC0415

        return fk_wrist3(np.asarray(q, dtype=float))  # stub motion layers: wrist pose is enough

    # ---- the loop ----------------------------------------------------------------------------

    def run(self, *, episode_id: str = "ep", seed: int = 0, classes=None) -> EpisodeResult:
        """Clear every item, choosing the order as the world changes.

        Returns:
            An :class:`~dishsim.task.episode.EpisodeResult`. A pick that fails does not abort
            the episode — the object is marked failed and the loop continues with the rest, so
            one bad grasp costs one object rather than the whole run.
        """
        result = EpisodeResult(
            episode_id=episode_id, seed=int(seed),
            classes=list(classes if classes is not None else [it.object_class for it in self.items]),
            cost_fn=self.cost_name,
        )
        order = 0
        while self.remaining():
            self.refresh()
            candidates = self.pickable()

            if not candidates:
                if self.recovery is not None and self._n_recoveries < self.max_recoveries:
                    self._n_recoveries += 1
                    self._emit("recovery", {"attempt": self._n_recoveries})
                    if self.recovery(self, self.remaining()):
                        continue
                result.status = "deadlock"
                result.detail = (
                    f"{len(self.remaining())} object(s) remain but none is pickable after "
                    f"{self._n_recoveries} recovery attempt(s)"
                )
                break

            item, candidate, cost = self.choose(candidates)
            item.goal_slot = self.slot_fn(item) if self.slot_fn is not None else None
            rec = PickRecord(
                item_id=item.item_id, object_class=item.object_class, order=order,
                goal_slot=getattr(item.goal_slot, "slot_id", None),
                goal_slot_name=getattr(item.goal_slot, "name", None),
                cost=cost, n_candidates=len(candidates),
            )
            self._emit("pick_start", {"item": item.item_id, "cost": cost,
                                      "candidates": [i.item_id for i, _ in candidates]})

            if item.goal_slot is None:
                item.state = "failed"
                rec.failure_stage = "no-goal-config"
                rec.failure_detail = "no free slot available for this object's placement mode"
            else:
                outcome = self.pick_place(item, candidate)
                item.state = "placed" if outcome.success else "failed"
                rec.success = outcome.success
                rec.failure_stage = outcome.failure_stage
                rec.failure_detail = outcome.failure_detail
                rec.plan_time_s = outcome.plan_time_s
                rec.n_replans = outcome.n_replans

            result.picks.append(rec)
            self._emit("pick_end", {"item": item.item_id, "success": rec.success,
                                    "stage": rec.failure_stage})
            order += 1

        if result.status != "deadlock":
            failed = [p for p in result.picks if not p.success]
            result.status = "cleared" if not failed else "partial"
            if failed:
                result.detail = f"{len(failed)} of {len(result.picks)} picks failed"
        result.unplaced = [it.item_id for it in self.items if it.state != "placed"]
        result.blocked_reasons = dict(self.blocked_reasons)
        result.support_graph = {k: list(v) for k, v in self.support_graph.items()}
        result.n_recoveries = self._n_recoveries
        return result

    def _emit(self, name: str, payload: dict) -> None:
        if self.on_event is not None:
            self.on_event(name, payload)
