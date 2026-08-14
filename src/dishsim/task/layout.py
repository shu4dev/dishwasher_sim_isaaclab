# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Randomized countertop layouts (Kit-free): where the episode's objects start.

Stage A generates non-overlapping layouts — random position inside the measured spawn
rectangle, random yaw, a minimum clear gap between footprint circles, and every item checked
against the robot's reach before it is accepted. Stage B relaxes the separation rule to allow
stacking; the reach guarantee moves to a spawn-settle-check-resample loop in the runner, because
once objects can rest on each other their final poses are decided by physics, not by us.

Determinism is a hard requirement: the same seed must produce the same layout, since a seed is
the only thing recorded in the episode manifest that can reproduce a scene. Sampling therefore
draws from a single seeded ``numpy`` generator and nothing else — no time, no set iteration
order, no dict ordering that depends on insertion history.

Reachability is injected as a predicate rather than computed here. This module has no collision
world and no IK; the runner passes a callable that closes over both. That keeps the layout
generator testable without a cache and keeps one definition of "reachable" — the runner's.
"""

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .. import config
from ..transforms import T_inv, make_T

#: ``(object_class, T_base_obj) -> bool``: can the robot actually pick this object here?
#: Supplied by the caller (the runner closes over a collision world and the IK). Layout
#: generation is pure geometry without it — passing ``None`` skips the check entirely.
ReachFn = Callable[[str, np.ndarray], bool]


@dataclass
class LayoutItem:
    """One object placed on the countertop.

    Attributes:
        item_id: Stable per-episode identifier, ``"<class>_<nn>"``.
        object_class: Registry key in :data:`dishsim.config.OBJECTS`.
        instance: Per-class instance index, 0-based.
        T_base_obj: Spawn pose in the robot-base frame, shape [4, 4].
        xy_w: Sampled surface position in world coordinates [m], shape [2].
        yaw_deg: Sampled yaw about world z [deg].
        radius_m: Footprint circle radius used for separation [m].
        support: ``item_id`` this object was spawned resting on, or ``None`` for the counter.
    """

    item_id: str
    object_class: str
    instance: int
    T_base_obj: np.ndarray
    xy_w: np.ndarray
    yaw_deg: float
    radius_m: float
    support: str | None = None

    def to_json(self) -> dict:
        return {
            "item_id": self.item_id,
            "object_class": self.object_class,
            "instance": self.instance,
            "T_base_obj": np.asarray(self.T_base_obj).tolist(),
            "xy_w": np.asarray(self.xy_w).tolist(),
            "yaw_deg": float(self.yaw_deg),
            "radius_m": float(self.radius_m),
            "support": self.support,
        }


@dataclass
class LayoutRejection:
    """Why rejection sampling spent the draws it did — scarcity is signal, not noise.

    A layout that only just fit, or a class that was nearly impossible to place, shows up here
    rather than as a silent success. Mirrors the funnel that
    :class:`dishsim.placement.GoalSet` records for goal generation.
    """

    n_draws: int = 0
    out_of_bounds: int = 0
    too_close: int = 0
    unreachable: int = 0
    per_class_draws: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        return {
            "draws": self.n_draws,
            "out_of_bounds": self.out_of_bounds,
            "too_close": self.too_close,
            "unreachable": self.unreachable,
            "per_class_draws": dict(self.per_class_draws),
        }


#: Half-extents within this relative tolerance of each other mean a rotationally symmetric
#: footprint (a disc or cylinder), whose yaw-invariant radius is the larger half-extent rather
#: than the bbox diagonal.
_ROUND_TOL = 0.10


def footprint_radius(spec) -> float:
    """Yaw-invariant radius of an object's staged footprint [m].

    A single scalar valid for every yaw is what lets separation be checked without re-deriving
    geometry per draw. For a rectangular footprint that scalar is the bbox diagonal; for a round
    one it is just the radius, and using the diagonal there would over-estimate a plate by 41%
    and make it unplaceable in a rectangle it comfortably fits.

    The extents taken are the ones perpendicular to the staging "up" — which follows
    :func:`stand_rotation`, not the rack convention.
    """
    h = spec.bbox_half
    a, b = (h[0], h[2]) if tuple(spec.axis_obj) == (0.0, 1.0, 0.0) else (h[0], h[1])
    if abs(a - b) <= _ROUND_TOL * max(a, b):  # disc / cylinder: yawing does not change it
        return float(max(a, b))
    return float(math.hypot(a, b))


def effective_rect(spec, rect: dict) -> dict:
    """Where an object's CENTER may be sampled: reachable *and* fully supported.

    Two different constraints get conflated easily and must not be. ``rect``
    (``TASK["spawn_rect_w"]``) bounds where a *grasp* is reachable, and it was measured by
    sampling object centers — so it is a rectangle of centers. The countertop, separately,
    has to physically support the whole footprint. Intersecting the two is the only bound that
    satisfies both, and it is why a large object gets a smaller sampling region than a small one
    rather than being rejected over and over at the edges.

    Args:
        spec: Object registry entry.
        rect: Reachable-center bounds in world coordinates.

    Returns:
        Intersected bounds, same keys as ``rect``.

    Raises:
        RuntimeError: If the intersection is empty — the object cannot stand anywhere both
            reachable and supported.
    """
    r = footprint_radius(spec)
    cx, cy = config.COUNTERTOP_CENTER_W[0], config.COUNTERTOP_CENTER_W[1]
    sx, sy = config.COUNTERTOP_SIZE[0], config.COUNTERTOP_SIZE[1]
    out = {
        "x_min": max(rect["x_min"], cx - sx / 2.0 + r),
        "x_max": min(rect["x_max"], cx + sx / 2.0 - r),
        "y_min": max(rect["y_min"], cy - sy / 2.0 + r),
        "y_max": min(rect["y_max"], cy + sy / 2.0 - r),
    }
    if out["x_min"] > out["x_max"] or out["y_min"] > out["y_max"]:
        raise RuntimeError(
            f"{spec.name!r} (footprint radius {r * 1000:.0f} mm) has no pose that is both "
            f"reachable and fully on the countertop: reach rect {rect} intersected with the "
            f"countertop inset by the footprint gives {out}. Widen TASK['spawn_rect_w'] or the "
            f"countertop."
        )
    return out


def stand_rotation(spec, yaw_deg: float) -> np.ndarray:
    """Rotation resting the object on the countertop, then yawing about world z.

    This follows the COUNTERTOP staging convention of
    :func:`dishsim.scene.countertop_pose_w` — the legacy Y-up mug stands via ``Rx(+90)`` and
    everything else rests as authored — and deliberately NOT
    :func:`dishsim.fill_plan._stand_R`, which is the *rack* convention. The two differ for
    length-along-x cutlery: the rack convention stands a fork head-up so it can drop into a
    basket bay, whereas on a counter a fork lies flat. Standing one on end here would spawn a
    9 mm footprint that topples during the settle and silently invalidates the layout.
    """
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    R0 = (Rotation.from_euler("x", 90.0, degrees=True).as_matrix()
          if tuple(spec.axis_obj) == (0.0, 1.0, 0.0) else np.eye(3))
    return Rotation.from_euler("z", float(yaw_deg), degrees=True).as_matrix() @ R0


def bottom_offset(spec, R: np.ndarray) -> float:
    """Distance from the object origin down to its lowest bbox point under ``R`` [m]."""
    h = np.array(spec.bbox_half)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return -float((R @ corners.T).T[:, 2].min())


def surface_pose(spec, xy_w, yaw_deg: float, surface_z_w: float, hover: float = 0.0) -> np.ndarray:
    """Base-frame pose standing an object at ``xy_w`` on a surface at ``surface_z_w``.

    Args:
        spec: Object registry entry.
        xy_w: World surface position [m], shape [2].
        yaw_deg: Yaw about world z [deg].
        surface_z_w: World z of the supporting surface's top face [m].
        hover: Clearance left below the object so it settles onto the surface [m].

    Returns:
        T_base_obj, shape [4, 4].
    """
    R = stand_rotation(spec, yaw_deg)  # world-frame rotation (yaw is about world z)
    T_w_obj = np.eye(4)
    T_w_obj[:3, :3] = R
    T_w_obj[:3, 3] = (float(xy_w[0]), float(xy_w[1]), surface_z_w + hover + bottom_offset(spec, R))
    # full-transform re-expression: a yawed base must rotate the world pose into the base
    # frame, not just shift it (exact subtraction at the identity base quaternion)
    return T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)) @ T_w_obj


def countertop_top_z_w() -> float:
    """World z of the countertop's top face [m]."""
    return float(config.COUNTERTOP_CENTER_W[2] + config.COUNTERTOP_SIZE[2] / 2.0)


def sample_classes(rng: np.random.Generator, n: int, classes=None) -> list[str]:
    """Draw ``n`` object classes with replacement from the task pool."""
    pool = list(classes if classes is not None else config.TASK["classes"])
    if not pool:
        raise ValueError("empty class pool")
    return [pool[int(i)] for i in rng.integers(0, len(pool), size=n)]


def plan_countertop_layout(
    classes,
    seed: int,
    *,
    reach_fn: ReachFn | None = None,
    allow_stacking: bool = False,
    rect=None,
    min_separation_m: float | None = None,
    max_resamples: int | None = None,
    stack_fraction: float | None = None,
    occupied=(),
) -> tuple[list[LayoutItem], LayoutRejection]:
    """Sample a reproducible countertop layout.

    Objects are placed one at a time by rejection sampling: draw a position inside the spawn
    rectangle and a yaw, reject if the footprint circle comes within ``min_separation_m`` of an
    already-placed one (Stage A) or if ``reach_fn`` says the robot could not pick it there, and
    keep drawing until the per-item budget is spent.

    Args:
        classes: Object-class names to place, in order. Placement order is the draw order, so
            the same list plus the same seed is the same layout.
        seed: Seed for the layout's random generator.
        reach_fn: Optional ``(object_class, T_base_obj) -> bool`` reachability predicate. When
            ``None``, poses are accepted on geometry alone.
        allow_stacking: Stage B. When true, overlapping footprints are permitted and an
            overlapping candidate is spawned resting on the object it overlaps.
        rect: Spawn bounds ``{"x_min", "x_max", "y_min", "y_max"}`` in world coordinates;
            defaults to ``config.TASK["spawn_rect_w"]``.
        min_separation_m: Clear gap between footprint circles; defaults to
            ``config.TASK["min_separation_m"]``. Ignored when ``allow_stacking``.
        max_resamples: Per-item draw budget; defaults to ``config.TASK["max_spawn_resamples"]``.
        stack_fraction: With ``allow_stacking``, the probability that an object is deliberately
            aimed at an already-placed one rather than drawn uniformly. Defaults to
            ``config.TASK["stack_fraction"]``.
        occupied: Footprint circles already ON the counter that this layout must clear —
            ``(xy_w, radius_m)`` pairs. A restock wave passes the measured leftovers of the
            previous wave here; a fresh layout leaves it empty.

    Returns:
        ``(items, rejection)``. ``items`` has one entry per requested class, in request order.

    Raises:
        RuntimeError: If an object cannot be placed within its draw budget. The message names
            the class, the budget, and the rejection breakdown, so the cause (rectangle too
            small, separation too large, class unreachable) is readable without a debugger.
    """
    rect = dict(rect if rect is not None else config.TASK["spawn_rect_w"])
    sep = float(config.TASK["min_separation_m"] if min_separation_m is None else min_separation_m)
    budget = int(config.TASK["max_spawn_resamples"] if max_resamples is None else max_resamples)
    hover = float(config.TASK["spawn_hover_m"])
    stack_fraction = float(config.TASK["stack_fraction"] if stack_fraction is None else stack_fraction)
    stack_jitter = float(config.TASK["stack_jitter_frac"])
    rng = np.random.default_rng(seed)
    surface_z = countertop_top_z_w()

    items: list[LayoutItem] = []
    rej = LayoutRejection()
    counters: dict[str, int] = {}

    for object_class in classes:
        spec = config.OBJECTS[object_class]
        r = footprint_radius(spec)
        eff = effective_rect(spec, rect)  # reachable centers AND fully-supported footprint
        placed = False
        drawn = 0
        for _ in range(budget):
            drawn += 1
            rej.n_draws += 1
            x = float(rng.uniform(eff["x_min"], eff["x_max"]))
            y = float(rng.uniform(eff["y_min"], eff["y_max"]))
            yaw = float(rng.uniform(0.0, 360.0))

            support = None
            if allow_stacking:
                # Deliberately stack a fraction of the objects. Relying on a uniform draw to
                # land one footprint on top of another produces stacks so rarely that the
                # support constraint would go untested on most seeds — "stacking enabled" has
                # to mean stacks actually appear.
                if items and rng.random() < stack_fraction:
                    base = items[int(rng.integers(0, len(items)))]
                    jitter = rng.uniform(-1.0, 1.0, 2) * stack_jitter * base.radius_m
                    x, y = float(base.xy_w[0] + jitter[0]), float(base.xy_w[1] + jitter[1])
                support = _stack_support(items, np.array([x, y]), r)
            else:
                if _too_close(items, np.array([x, y]), r, sep):
                    rej.too_close += 1
                    continue
                if any(float(np.linalg.norm(np.array([x, y]) - np.asarray(oxy))) < (r + orad + sep)
                       for oxy, orad in occupied):
                    rej.too_close += 1
                    continue

            z_surface = surface_z
            if support is not None:
                base = next(it for it in items if it.item_id == support)
                z_surface = _top_z_w(base)

            T = surface_pose(spec, (x, y), yaw, z_surface, hover=hover)
            if reach_fn is not None and not reach_fn(object_class, T):
                rej.unreachable += 1
                continue

            inst = counters.get(object_class, 0)
            counters[object_class] = inst + 1
            items.append(
                LayoutItem(
                    item_id=f"{object_class}_{inst:02d}",
                    object_class=object_class,
                    instance=inst,
                    T_base_obj=T,
                    xy_w=np.array([x, y]),
                    yaw_deg=yaw,
                    radius_m=r,
                    support=support,
                )
            )
            placed = True
            break

        rej.per_class_draws[object_class] = rej.per_class_draws.get(object_class, 0) + drawn
        if not placed:
            raise RuntimeError(
                f"could not place {object_class!r} in {budget} draws "
                f"(sampling rect {eff}, separation {sep * 1000:.0f} mm, footprint radius "
                f"{r * 1000:.0f} mm; rejections so far: {rej.too_close} too-close, "
                f"{rej.unreachable} unreachable). Widen TASK['spawn_rect_w'], lower "
                f"TASK['min_separation_m'], or drop TASK['n_objects']."
            )

    return items, rej


def _too_close(items, xy: np.ndarray, r: float, sep: float) -> bool:
    """Would a footprint circle at ``xy`` come within ``sep`` of an already-placed one?"""
    return any(float(np.linalg.norm(xy - it.xy_w)) < (r + it.radius_m + sep) for it in items)


def _stack_support(items, xy: np.ndarray, r: float) -> str | None:
    """Stage B: the already-placed item a candidate at ``xy`` would rest on, if any.

    Picks the highest overlapping item, so a candidate dropped onto a two-object pile lands on
    the top of it rather than tunnelling to the one underneath.
    """
    overlapping = [it for it in items if float(np.linalg.norm(xy - it.xy_w)) < (r + it.radius_m)]
    if not overlapping:
        return None
    return max(overlapping, key=_top_z_w).item_id


def _top_z_w(item: LayoutItem) -> float:
    """World z of an item's highest bbox point [m]."""
    spec = config.OBJECTS[item.object_class]
    # lift the base-frame pose to the world through the full base transform (identical z-only
    # math at the identity base quaternion)
    T_w_obj = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W) @ item.T_base_obj
    h = np.array(spec.bbox_half)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    top_local = float((T_w_obj[:3, :3] @ corners.T).T[:, 2].max())
    return float(T_w_obj[2, 3] + top_local)


def resolve_spawn_counts(spec: dict, rng: np.random.Generator) -> list:
    """Expand ``{object_class: count}`` into the class list an episode will spawn.

    A count is either an exact ``int`` or an inclusive ``(lo, hi)`` range drawn per episode, so
    the number of objects can vary while staying reproducible from the seed.

    Two ordering decisions matter and are deliberate:

    **Keys are consumed in sorted order, never dict or CLI order.** A spec is a set of
    constraints, not a sequence, and with a range the count is itself a draw — so iterating in
    insertion order would mean ``--spawn "mug=1,cup=2-4"`` and ``--spawn "cup=2-4,mug=1"``
    produce different compositions from the same seed, and adding a key to the config would
    silently change every existing seed.

    **The expanded list is then shuffled.** Placement order is draw order, so an unshuffled list
    would always let the alphabetically-first type claim the best countertop space. Composition
    should be fixed by config; position should not be.

    Args:
        spec: ``{object_class: int | (lo, hi)}``.
        rng: Seeded generator; both the range draws and the shuffle consume it.

    Returns:
        Object-class names, one entry per object to spawn, in placement order.

    Raises:
        ValueError: On an unknown class, a malformed or negative count, or an inverted range.
    """
    if not isinstance(spec, dict) or not spec:
        raise ValueError(f"spawn_counts must be a non-empty mapping, got {spec!r}")
    out: list = []
    for name in sorted(spec):  # sorted, not insertion order — see the docstring
        if name not in config.OBJECTS:
            raise ValueError(
                f"spawn_counts names unknown object class {name!r} "
                f"(choices: {sorted(config.OBJECTS)})"
            )
        value = spec[name]
        if isinstance(value, (list, tuple)):
            if len(value) != 2:
                raise ValueError(f"spawn_counts[{name!r}] range must be (lo, hi), got {value!r}")
            lo, hi = int(value[0]), int(value[1])
            if lo > hi:
                raise ValueError(f"spawn_counts[{name!r}] range {value!r} is inverted")
        else:
            lo = hi = int(value)
        if lo < 0:
            raise ValueError(f"spawn_counts[{name!r}] is negative: {value!r}")
        out.extend([name] * int(rng.integers(lo, hi + 1)))

    # Fisher-Yates over rng.integers — the same primitive sample_classes uses, so the whole
    # composition stream stays on one documented generator call.
    for i in range(len(out) - 1, 0, -1):
        j = int(rng.integers(0, i + 1))
        out[i], out[j] = out[j], out[i]
    return out


def parse_spawn_arg(text: str) -> dict:
    """Parse ``"cup=2-4,mug=1"`` into a :func:`resolve_spawn_counts` spec.

    Args:
        text: Comma-separated ``class=count`` or ``class=lo-hi`` terms.

    Returns:
        ``{object_class: int | (lo, hi)}``.
    """
    spec: dict = {}
    for term in (t.strip() for t in text.split(",") if t.strip()):
        name, sep, count = term.partition("=")
        if not sep:
            raise ValueError(f"--spawn term {term!r} is not 'class=count' or 'class=lo-hi'")
        name, count = name.strip(), count.strip()
        try:
            if "-" in count:
                lo, hi = count.split("-", 1)
                spec[name] = (int(lo), int(hi))
            else:
                spec[name] = int(count)
        except ValueError as exc:
            raise ValueError(f"--spawn term {term!r} has a non-integer count") from exc
    if not spec:
        raise ValueError("--spawn parsed to an empty specification")
    return spec
