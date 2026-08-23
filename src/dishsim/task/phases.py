# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Full-load episode structure: loading phases, countertop waves, rack-state transitions.

A full load spans machine states — on the Bosch the lower rack loads in ``placement``, the
middle rack in ``middle_out``, the third rack in ``third_out`` — so a full-load episode is an
ordered list of PHASES, each "spawn what fits on the counter, clear it, restock" (WAVES),
with a scripted rack transition between phases. This module is the Kit-free planning half:
what the phases are, which items go in which wave, and which complete joint-target dicts a
transition must command. Execution lives in the runner and
:class:`dishsim.task.rack.ScriptedTransition`.

Phase states must be INTERNAL states (no ``rack_action``): a scenario with a rack action is
an episode PROLOGUE (the robot pulls the rack), not a phase — the sequencer's phases begin
after any prologue has already run.
"""

import json
from dataclasses import dataclass, field

import numpy as np

from .. import config


@dataclass(frozen=True)
class PhaseItem:
    """One planned item of a phase, pinned to its slot by the capacity plan.

    ``T_base_slot`` rides along so the runner can seed slot assignment without re-deriving;
    ``instance`` numbers the item within its class (globally across phases — item ids are
    scene prim names and must be unique).
    """

    item_id: str
    object_class: str
    instance: int
    slot_id: int
    slot_name: str
    mode: str
    acquire: str
    T_base_slot: np.ndarray | None = None


@dataclass
class Phase:
    """One loading phase: a machine state plus the items destined for its rack."""

    state: str
    items: list = field(default_factory=list)


def load_full_load_plan(path: str, *, machine: str, base_placement: str) -> list[Phase]:
    """Read a capacity-plan artifact (see :mod:`dishsim.capacity`) into phases.

    Hard-fails on a machine/placement mismatch — a plan certified for one mount says nothing
    about another. A config-hash drift is a WARNING only: the runner's own cache pre-flight
    re-validates every hash it loads, so a stale stamp here means "replan for an honest N",
    not "cannot run".
    """
    with open(path) as f:
        doc = json.load(f)
    if doc.get("machine") != machine or doc.get("base_placement") != base_placement:
        raise SystemExit(
            f"[FAIL] full-load plan {path} was computed for "
            f"{doc.get('machine')}@{doc.get('base_placement')}, not "
            f"{machine}@{base_placement} — re-run scripts/setup/plan_full_load.py")
    phases = []
    for ph in doc.get("phases", []):
        state = ph["state"]
        if config.state_params(state).get("rack_action"):
            raise SystemExit(f"[FAIL] plan phase state {state!r} carries a rack action — "
                             f"phase states must be internal (no-action) states")
        items = [PhaseItem(item_id=it["item_id"], object_class=it["object_class"],
                           instance=int(it["instance"]), slot_id=int(it["slot_id"]),
                           slot_name=str(it["slot_name"]), mode=it["mode"],
                           acquire=it["acquire"],
                           T_base_slot=np.array(it["T_base_slot"]))
                 for it in ph.get("items", [])]
        if items:
            phases.append(Phase(state, items))
    if not phases:
        raise SystemExit(f"[FAIL] full-load plan {path} contains no items")
    return phases


def parse_phases_arg(text: str) -> list[tuple[str, dict]]:
    """Parse an explicit debug composition: ``"placement:plate=2,cup=1;third_out:fork=2"``.

    Returns ``[(state, {class: count})]`` in phase order. States resolve through
    :func:`config.resolve_rack_state` and must be internal (no rack action).
    """
    phases = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        state_part, _, spec_part = chunk.partition(":")
        state = config.resolve_rack_state(state_part.strip())
        if config.state_params(state).get("rack_action"):
            raise ValueError(f"phase state {state!r} carries a rack action — use it as "
                             f"--scenario (the episode prologue), not as a phase")
        counts: dict = {}
        for tok in spec_part.split(","):
            tok = tok.strip()
            if not tok:
                continue
            cls, _, n = tok.partition("=")
            cls = cls.strip()
            if cls not in config.OBJECTS:
                raise ValueError(f"unknown object class {cls!r} in phase spec")
            counts[cls] = counts.get(cls, 0) + int(n or 1)
        if not counts:
            raise ValueError(f"phase {state!r} names no objects")
        phases.append((state, counts))
    if not phases:
        raise ValueError("empty --phases spec")
    return phases


def plan_waves(items, *, rect: dict, min_separation_m: float, radius_fn,
               fill_factor: float, max_wave: int | None = None) -> list[list]:
    """Split a phase's items into countertop restock waves, deterministically.

    Greedy in plan order: an item joins the current wave while the wave's summed footprint
    area (each item a disc of ``radius + separation/2``) stays under ``fill_factor`` of the
    spawn-rect area — random rejection-sampled layouts stop converging well before geometric
    packing density, which is what the factor models. ``max_wave`` caps count regardless.

    Args:
        items: :class:`PhaseItem` values in plan order.
        rect: ``TASK["spawn_rect_w"]``-shaped dict.
        min_separation_m: Layout separation [m] (``TASK["min_separation_m"]``).
        radius_fn: ``object_class -> footprint radius [m]`` (pass
            :func:`dishsim.task.layout.footprint_radius` over the registry spec).
        fill_factor: Fraction of the rect area a wave may claim (``TASK["wave_fill_factor"]``).
        max_wave: Optional hard cap on items per wave (``TASK["max_wave_objects"]``).
    """
    area = max((rect["x_max"] - rect["x_min"]) * (rect["y_max"] - rect["y_min"]), 1e-9)
    budget = float(fill_factor) * area
    waves: list[list] = []
    current: list = []
    used = 0.0
    for item in items:
        r = float(radius_fn(item.object_class)) + min_separation_m / 2.0
        a = float(np.pi) * r * r
        over_area = current and used + a > budget
        over_count = max_wave is not None and len(current) >= max_wave
        if over_area or over_count:
            waves.append(current)
            current, used = [], 0.0
        current.append(item)
        used += a
    if current:
        waves.append(current)
    return waves


def state_joint_targets(state: str) -> dict:
    """The state's COMPLETE rack-joint target dict (same mapping as ``apply_scenario``,
    without touching module globals)."""
    sc = config.state_params(state)
    out = {"PrismaticJoint_dishwasher_2_down": float(sc["rack_lower_m"]),
           "PrismaticJoint_dishwasher_2_up": float(sc["rack_upper_m"])}
    if config.HAS_THIRD_RACK:
        out[config.RACK_THIRD_JOINT] = float(sc.get("rack_third_m", 0.0))
    return out


def transition_targets(from_state: str, to_state: str) -> list[dict]:
    """Ordered complete target dicts taking the machine ``from_state -> to_state``.

    Stow-first: retracting joints move in the first dict (under whatever is already
    overhead), extending joints in the second — a loaded lower rack is never in flight at
    the same time as the rack extending above it. Every dict contains EVERY rack joint of
    the machine; the replace-not-merge override latch makes that a structural requirement,
    not a convention (see :class:`~dishsim.task.rack.ScriptedTransition`).
    """
    src, dst = state_joint_targets(from_state), state_joint_targets(to_state)
    stow = {j: dst[j] for j in dst if abs(dst[j]) < abs(src[j]) - 1e-9}
    step1 = {**src, **stow}
    out = []
    if step1 != src:
        out.append(step1)
    if dst != step1:
        out.append(dst)
    return out or [dst]
