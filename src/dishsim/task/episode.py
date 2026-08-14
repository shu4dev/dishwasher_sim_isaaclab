# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Episode records — the Phase 2 artifact for a multi-object task.

One episode clears N objects, so it produces N per-pick records plus one episode-level record
that ties them together. The per-pick records deliberately keep the EXISTING trial schema
(``TRIAL_SCHEMA_VERSION``), which is what lets Phase 3 read multi-object runs with no changes at
all: ``compute_metrics.py`` aggregates them as trials, ``render_videos.py`` renders their
trajectories, ``verify_replay.py`` checks them one by one.

The episode record adds what a per-pick record cannot express — the order picks were made in and
why, what the support graph looked like, and whether the episode ended clean, deadlocked or
aborted. It is written on every exit path, failures included, for the same reason the trial
runner writes a recording even for a failed trial: the failed episode is the interesting one.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np

#: Bumped whenever the on-disk episode schema changes shape. Kept independent of the per-pick
#: ``TRIAL_SCHEMA_VERSION`` so the two can evolve separately.
#: 2 — added the home-anchoring fields (``start_home_err_rad``, ``end_home_err_rad``,
#: ``home_return_status``, ``post_home_displacement_mm``).
#: 3 — multi-phase full-load episodes: per-pick ``phase``/``phase_state``/``wave``/
#: ``acquired`` (all None/absent in single-phase episodes, so v2 consumers read v3 records
#: unchanged) and episode-level ``phases``/``transitions``.
EPISODE_SCHEMA_VERSION = 3


@dataclass
class PickRecord:
    """One object's journey from the countertop to a slot.

    Attributes:
        item_id: Stable per-episode identifier.
        object_class: Registry key.
        order: 0-based position in the executed pick order.
        goal_slot: Slot id the object was placed into, or ``None``.
        goal_slot_name: The slot's derived name (see :func:`dishsim.placement.slot_names`), or ``None``.
        success: Whether the object ended up placed within tolerance.
        failure_stage: Failure taxonomy value (docs/success_criteria.md), or ``None``.
        failure_detail: Human-readable cause, or ``None``.
        cost: Value the ordering heuristic assigned when this item was chosen.
        n_candidates: How many objects were pickable at that moment.
        plan_time_s: Planning time attributed to this pick.
        n_replans: Stage C — times the grasp was recomputed after the object shifted.
        trial_path: Per-pick trial JSON, relative to the project root.
        trajectory_path: Per-pick trajectory ``.npz``, relative to the project root.
        phase: 0-based loading-phase index (None in single-phase episodes).
        phase_state: Machine state the pick placed in (None in single-phase episodes).
        wave: 0-based countertop restock wave within the phase (None in single-phase).
        acquired: ``"pick"`` or ``"weld"`` — the two acquisition families report separately
            and their success rates are never summed (None in pre-v3 records; consumers
            fall back to the trial record's field).
    """

    item_id: str
    object_class: str
    order: int
    goal_slot: int | None = None
    goal_slot_name: str | None = None
    success: bool = False
    failure_stage: str | None = None
    failure_detail: str | None = None
    cost: float | None = None
    n_candidates: int = 0
    plan_time_s: float = 0.0
    n_replans: int = 0
    trial_path: str | None = None
    trajectory_path: str | None = None
    phase: int | None = None
    phase_state: str | None = None
    wave: int | None = None
    acquired: str | None = None

    def to_json(self) -> dict:
        d = dict(self.__dict__)
        d["cost"] = None if self.cost is None else round(float(self.cost), 6)
        d["plan_time_s"] = round(float(self.plan_time_s), 4)
        return d


@dataclass
class EpisodeResult:
    """Outcome of one multi-object episode.

    Attributes:
        episode_id: Identifier, unique within a run.
        seed: Layout seed — with the class list, this reproduces the scene exactly.
        classes: Object classes requested, in draw order.
        cost_fn: Name of the ordering heuristic used.
        picks: Per-pick records, in executed order.
        status: ``"cleared"`` (everything placed), ``"partial"`` (some pick failed),
            ``"deadlock"`` (objects remained but none was pickable), or ``"error"``.
        detail: Human-readable explanation for a non-cleared status.
        unplaced: Item ids still on the countertop when the episode ended.
        blocked_reasons: For a deadlock, why each remaining item was not pickable.
        support_graph: Final support graph, ``{item_id: [ids resting on it]}``.
        layout_rejection: Rejection funnel from layout generation.
        n_layout_attempts: Stage B — whole-layout resamples spent before one settled valid.
        n_recoveries: Stage C — recovery ladder invocations.
        start_home_err_rad: Largest per-joint deviation from ``HOME_Q`` [rad] measured at the
            first pick, after any corrective retreat.
        end_home_err_rad: The same, measured after the closing retreat — the evidence that the
            recorded trajectory ends where it started.
        home_return_status: How the closing retreat got there: ``"skipped"``, ``"planned"`` or
            ``"lerp"``. A ``"lerp"`` was not collision-checked, so pair it with
            :attr:`post_home_displacement_mm` before trusting the final scene.
        post_home_displacement_mm: ``{item_id: mm}`` each PLACED object moved during the closing
            retreat. Success verdicts are decided before the retreat runs and are never revised
            by it; this reports whether parking the arm disturbed the load.
        rack_phase: Per-action records from the rack prologue (empty when the racks started
            pre-positioned). An episode whose rack action failed is an ``error`` with no picks —
            every pick plans against a world posed at the post-action extension, so a rack that
            never arrived invalidates all of them.
        phases: Per-loading-phase summaries of a full-load episode (empty in single-phase
            episodes): ``{"phase", "state", "n_items", "n_placed", "n_waves",
            "transition_ok"}``. Unlike a failed rack PROLOGUE, a failed mid-episode
            transition does not void the earlier phases — their picks planned against their
            own worlds — so the episode degrades to ``partial`` and keeps what was placed.
        transitions: Scripted rack transitions in execution order
            (:meth:`dishsim.task.rack.TransitionOutcome.to_json` documents the shape).
    """

    episode_id: str
    seed: int
    classes: list = field(default_factory=list)
    cost_fn: str = ""
    picks: list = field(default_factory=list)
    status: str = "error"
    detail: str | None = None
    unplaced: list = field(default_factory=list)
    blocked_reasons: dict = field(default_factory=dict)
    support_graph: dict = field(default_factory=dict)
    layout_rejection: dict = field(default_factory=dict)
    n_layout_attempts: int = 1
    n_recoveries: int = 0
    start_home_err_rad: float | None = None
    end_home_err_rad: float | None = None
    home_return_status: str | None = None
    post_home_displacement_mm: dict = field(default_factory=dict)
    rack_phase: list = field(default_factory=list)
    phases: list = field(default_factory=list)
    transitions: list = field(default_factory=list)

    @property
    def n_placed(self) -> int:
        return sum(1 for p in self.picks if p.success)

    def to_json(self) -> dict:
        return {
            "schema_version": EPISODE_SCHEMA_VERSION,
            "episode_id": self.episode_id,
            "seed": int(self.seed),
            "classes": list(self.classes),
            "cost_fn": self.cost_fn,
            "status": self.status,
            "detail": self.detail,
            "n_objects": len(self.classes),
            "n_placed": self.n_placed,
            "pick_order": [p.item_id for p in self.picks],
            "picks": [p.to_json() for p in self.picks],
            "unplaced": list(self.unplaced),
            "blocked_reasons": dict(self.blocked_reasons),
            "support_graph": {k: sorted(v) for k, v in self.support_graph.items()},
            "layout_rejection": dict(self.layout_rejection),
            "n_layout_attempts": int(self.n_layout_attempts),
            "n_recoveries": int(self.n_recoveries),
            "start_home_err_rad": _round(self.start_home_err_rad, 6),
            "end_home_err_rad": _round(self.end_home_err_rad, 6),
            "home_return_status": self.home_return_status,
            "post_home_displacement_mm": {k: round(float(v), 3)
                                          for k, v in self.post_home_displacement_mm.items()},
            "rack_phase": list(self.rack_phase),
            "phases": list(self.phases),
            "transitions": list(self.transitions),
        }


def _round(x, n: int):
    return None if x is None else round(float(x), n)


def merge_wave_results(master: EpisodeResult, wave_result: EpisodeResult, *,
                       phase_index: int, state: str, wave_index: int,
                       acquired_by_item: dict | None = None) -> None:
    """Fold one wave's sequencer result into the episode-level master record.

    The sequencer numbers picks from 0 within each of its runs; the master re-offsets
    ``order`` so it stays episode-global (the property every consumer of ``pick_order``
    assumes). Blocked/unplaced/recovery bookkeeping accumulates; the final ``status`` is NOT
    computed here — the runner decides it once, at the end, from what was planned vs placed
    (see :func:`full_load_status`).

    Args:
        master: The episode-level accumulator.
        wave_result: One ``TaskSequencer.run`` return value.
        phase_index: 0-based loading-phase index stamped onto the wave's picks.
        state: Machine state stamped onto the wave's picks.
        wave_index: 0-based restock-wave index within the phase.
        acquired_by_item: Optional ``{item_id: "pick" | "weld"}`` stamped per pick.
    """
    base = len(master.picks)
    for p in wave_result.picks:
        p.order += base
        p.phase, p.phase_state, p.wave = int(phase_index), state, int(wave_index)
        if acquired_by_item:
            p.acquired = acquired_by_item.get(p.item_id, p.acquired)
        master.picks.append(p)
    master.unplaced.extend(i for i in wave_result.unplaced if i not in master.unplaced)
    master.blocked_reasons.update(wave_result.blocked_reasons)
    master.support_graph = wave_result.support_graph or master.support_graph
    master.n_recoveries += wave_result.n_recoveries
    if wave_result.status == "deadlock" and master.detail is None:
        master.detail = f"phase {phase_index} wave {wave_index}: {wave_result.detail}"


def full_load_status(master: EpisodeResult, *, n_planned: int,
                     transitions_ok: bool) -> str:
    """Episode status for a full-load run: ``cleared`` only when every planned item placed.

    A failed transition degrades rather than errors — earlier phases' picks planned against
    their own worlds and stay valid evidence — unless nothing at all was placed.
    """
    if master.n_placed >= n_planned and transitions_ok:
        return "cleared"
    if master.n_placed == 0 and not transitions_ok:
        return "error"
    return "partial"


def write_episode(path: str, result: EpisodeResult, *, extra: dict | None = None) -> str:
    """Write an episode record as JSON.

    Args:
        path: Destination ``.json`` path.
        result: The episode outcome.
        extra: Additional provenance merged into the top level (planner block, config hash,
            object/scenario, USD stamps).

    Returns:
        The path written.
    """
    doc = result.to_json()
    if extra:
        doc.update(extra)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(doc, f, indent=2, default=_json_default)
    return path


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    raise TypeError(f"not JSON serialisable: {type(o)!r}")


def summarize_episodes(episodes) -> dict:
    """Aggregate episode records — the multi-object analogue of :func:`dishsim.metrics.summarize`.

    Args:
        episodes: Decoded episode JSON documents.

    Returns:
        Counts, clear rate, per-class placement rate and the failure-stage histogram.
    """
    n = len(episodes)
    if n == 0:
        return {"n_episodes": 0, "clear_rate": 0.0, "n_picks": 0, "pick_success_rate": 0.0}
    cleared = sum(1 for e in episodes if e.get("status") == "cleared")
    picks = [p for e in episodes for p in e.get("picks", [])]
    ok = sum(1 for p in picks if p.get("success"))
    stages: dict[str, int] = {}
    per_class: dict[str, dict] = {}
    for p in picks:
        if p.get("failure_stage"):
            stages[p["failure_stage"]] = stages.get(p["failure_stage"], 0) + 1
        c = per_class.setdefault(p.get("object_class", "?"), {"picks": 0, "success": 0})
        c["picks"] += 1
        c["success"] += int(bool(p.get("success")))
    for c in per_class.values():
        c["rate"] = round(c["success"] / c["picks"], 4) if c["picks"] else 0.0
    return {
        "n_episodes": n,
        "n_cleared": cleared,
        "clear_rate": round(cleared / n, 4),
        "n_picks": len(picks),
        "n_pick_success": ok,
        "pick_success_rate": round(ok / len(picks), 4) if picks else 0.0,
        "status_counts": {s: sum(1 for e in episodes if e.get("status") == s)
                          for s in sorted({e.get("status", "error") for e in episodes})},
        "failure_stages": stages,
        "by_class": per_class,
        "n_deadlocks": sum(1 for e in episodes if e.get("status") == "deadlock"),
        "n_recoveries": sum(int(e.get("n_recoveries", 0)) for e in episodes),
        # home anchoring: the worst parking error over the run, and how many episodes had to
        # fall back to the un-collision-checked retreat
        "max_end_home_err_rad": max(
            (float(e["end_home_err_rad"]) for e in episodes
             if e.get("end_home_err_rad") is not None), default=None),
        "n_home_lerp": sum(1 for e in episodes if e.get("home_return_status") == "lerp"),
        **_phase_extras(episodes, picks),
    }


def _phase_extras(episodes, picks) -> dict:
    """Multi-phase aggregates; empty for runs of pre-v3 / single-phase records only."""
    transitions = [t for e in episodes for t in e.get("transitions", [])]
    by_state: dict[str, dict] = {}
    for p in picks:
        if p.get("phase_state") is None:
            continue
        c = by_state.setdefault(p["phase_state"], {"picks": 0, "success": 0})
        c["picks"] += 1
        c["success"] += int(bool(p.get("success")))
    for c in by_state.values():
        c["rate"] = round(c["success"] / c["picks"], 4) if c["picks"] else 0.0
    # weld-acquire vs pick stay separate lines — the two rates are never summed
    by_acquired: dict[str, dict] = {}
    for p in picks:
        if p.get("acquired") is None:
            continue
        c = by_acquired.setdefault(p["acquired"], {"picks": 0, "success": 0})
        c["picks"] += 1
        c["success"] += int(bool(p.get("success")))
    for c in by_acquired.values():
        c["rate"] = round(c["success"] / c["picks"], 4) if c["picks"] else 0.0
    if not transitions and not by_state and not by_acquired:
        return {}
    return {
        "n_transitions": len(transitions),
        "transition_ok_rate": (round(sum(1 for t in transitions if t.get("ok"))
                                     / len(transitions), 4) if transitions else None),
        "by_phase_state": by_state,
        "by_acquired": by_acquired,
    }
