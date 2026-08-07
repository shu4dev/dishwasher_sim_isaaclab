# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Support graph: which object rests on which (Stage B).

The sequencer refuses to pick an object with something resting on it, so this decides which
objects are even candidates. It is rebuilt from MEASURED poses before every pick, never carried
over — removing one object lets its neighbours settle somewhere new, and a stale graph either
blocks a pickable object forever or authorises a pick that pulls a pile over.

Two backends, chosen by ``config.TASK["support_backend"]``:

``geometric``
    Pure geometry over measured poses: footprint-circle overlap in xy, plus vertical adjacency
    between one object's bottom and another's top. No simulator needed, so it is testable
    Kit-free and is the only backend available when replaying a recording.

``contact``
    PhysX contact forces between object pairs, read from each object's ``ContactSensor``
    filtered against its peers. Ground truth about *touching*, but touching is not resting:
    two objects leaning together touch without either supporting the other, so contact pairs
    are still oriented by height before becoming support edges.

``both``
    Runs both and returns the geometric result, logging any disagreement. That is how the two
    stay honest about each other — a persistent disagreement means one of them is wrong, and
    silently trusting either would hide it.

Direction, not just adjacency, is what matters: the graph maps ``supporter -> {objects resting
ON it}``, and an object is pickable when its entry is empty. Edges are *direct* only. If C rests
on B and B rests on A, then A's entry is ``{B}`` and B's is ``{C}`` — so C is picked first, and
after the rebuild B becomes free. Recording transitive dependants would work too, but it would
make "who do I clear next" a search rather than a lookup, for no gain.
"""

from dataclasses import dataclass, field

import numpy as np

from .. import config


@dataclass
class SupportGraph:
    """Directed "rests on" relation over a set of items.

    Attributes:
        supports: ``{item_id: {ids resting directly ON it}}`` — the sequencer's pickability test.
        rests_on: ``{item_id: {ids it rests directly on}}`` — the inverse, for diagnostics.
        grounded: ids resting on the countertop rather than on another object.
        disagreements: pairs the backends disagreed about, when running ``both``.
    """

    supports: dict = field(default_factory=dict)
    rests_on: dict = field(default_factory=dict)
    grounded: set = field(default_factory=set)
    disagreements: list = field(default_factory=list)

    def pickable(self, item_id: str) -> bool:
        """Is nothing resting on this object?"""
        return not self.supports.get(item_id)

    def to_json(self) -> dict:
        return {
            "supports": {k: sorted(v) for k, v in self.supports.items() if v},
            "rests_on": {k: sorted(v) for k, v in self.rests_on.items() if v},
            "grounded": sorted(self.grounded),
            "disagreements": list(self.disagreements),
        }


def object_extent(object_class: str, T_base_obj: np.ndarray) -> tuple:
    """Footprint centre, radius, bottom z and top z of an object at a pose.

    Args:
        object_class: Registry key.
        T_base_obj: Pose in the robot-base frame, shape [4, 4].

    Returns:
        ``(centre_xy [2], radius [m], bottom_z [m], top_z [m])``, all in the robot-base frame.
    """
    from .layout import footprint_radius  # noqa: PLC0415  (avoids a circular import at module load)

    spec = config.OBJECTS[object_class]
    T = np.asarray(T_base_obj, dtype=float)
    h = np.array(spec.bbox_half, dtype=float)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    z = (T[:3, :3] @ corners.T).T[:, 2]
    return T[:2, 3], footprint_radius(spec), float(T[2, 3] + z.min()), float(T[2, 3] + z.max())


def build_support_graph(items, *, backend: str | None = None, forces=None,
                        surface_z_base: float | None = None) -> SupportGraph:
    """Determine which objects rest on which.

    Args:
        items: Objects with ``item_id``, ``object_class`` and measured ``T_base_obj``.
        backend: ``"geometric"`` | ``"contact"`` | ``"both"``; defaults to
            ``config.TASK["support_backend"]``.
        forces: For the contact backends, ``{item_id: {peer_id: force_N}}`` measured this step.
        surface_z_base: Countertop top in the robot-base frame [m]; defaults to the configured
            countertop. Used to tell "resting on the counter" from "resting on an object".

    Returns:
        A :class:`SupportGraph`.
    """
    backend = backend or config.TASK["support_backend"]
    if surface_z_base is None:
        from .layout import countertop_top_z_w  # noqa: PLC0415

        surface_z_base = countertop_top_z_w() - float(config.ROBOT_BASE_POS_W[2])

    geo = _geometric(items, surface_z_base)
    if backend == "geometric":
        return geo
    con = _from_contacts(items, forces or {}, surface_z_base)
    if backend == "contact":
        return con

    # "both": geometry is authoritative, contacts are the cross-check. Reporting the difference
    # is the point — a backend that is quietly wrong is worse than one that is loudly wrong.
    disagreements = []
    for a in sorted(set(geo.supports) | set(con.supports)):
        g, c = geo.supports.get(a, set()), con.supports.get(a, set())
        for b in sorted(g ^ c):
            disagreements.append({"supporter": a, "resting": b,
                                  "geometric": b in g, "contact": b in c})
    geo.disagreements = disagreements
    return geo


def _geometric(items, surface_z_base: float) -> SupportGraph:
    """Footprint overlap plus vertical adjacency, oriented by height."""
    gap = float(config.TASK["support_gap_m"])
    frac = float(config.TASK["support_overlap_frac"])
    ext = {it.item_id: object_extent(it.object_class, it.T_base_obj) for it in items}

    graph = SupportGraph(supports={it.item_id: set() for it in items},
                         rests_on={it.item_id: set() for it in items})
    for upper in items:
        c_u, r_u, bot_u, _ = ext[upper.item_id]
        # An object whose bottom is still at counter height rests on the COUNTER, whatever is
        # beside it. Without this an adjacent short neighbour would look like a supporter.
        if bot_u <= surface_z_base + gap:
            graph.grounded.add(upper.item_id)
            continue
        found = False
        for lower in items:
            if lower.item_id == upper.item_id:
                continue
            c_l, r_l, bot_l, top_l = ext[lower.item_id]
            if bot_u <= bot_l:
                continue  # strictly-higher-rests-on-lower keeps the relation acyclic
            # "Supported by", not "sitting exactly on top of". Requiring the upper bottom to be
            # level with the lower top misses NESTING: cups stack by sitting down inside one
            # another, so the upper object's bottom ends up well below the lower object's rim.
            # Measured on a real settled stack, the geometric backend called a nested pair
            # unsupported while PhysX reported them in contact. The band below spans from just
            # above the lower object's own bottom up to a gap past its top, which covers nesting
            # and resting-on-top alike while still excluding anything floating clear.
            if bot_u > top_l + gap:
                continue  # floating clear above it
            d = float(np.linalg.norm(np.asarray(c_u) - np.asarray(c_l)))
            overlap = (r_u + r_l) - d  # linear overlap of the two footprint circles
            if overlap < frac * min(r_u, r_l):
                continue  # merely adjacent, not on top of
            graph.supports[lower.item_id].add(upper.item_id)
            graph.rests_on[upper.item_id].add(lower.item_id)
            found = True
        if not found:
            graph.grounded.add(upper.item_id)
    return graph


def _from_contacts(items, forces, surface_z_base: float) -> SupportGraph:
    """Measured object-object contact, oriented by height.

    Contact alone is symmetric and does not say who holds up whom — two objects leaning together
    register a pair without either supporting the other. The pair is therefore oriented the same
    way the geometric backend orients it: the object whose bottom is higher rests on the other.
    """
    thresh = float(config.TASK.get("support_force_thresh_n", 0.05))
    ext = {it.item_id: object_extent(it.object_class, it.T_base_obj) for it in items}
    ids = {it.item_id for it in items}

    graph = SupportGraph(supports={it.item_id: set() for it in items},
                         rests_on={it.item_id: set() for it in items})
    for a in sorted(ids):
        bot_a = ext[a][2]
        if bot_a <= surface_z_base + float(config.TASK["support_gap_m"]):
            graph.grounded.add(a)
        touching = {b: f for b, f in (forces.get(a) or {}).items()
                    if b in ids and b != a and float(f) > thresh}
        for b in sorted(touching):
            if bot_a > ext[b][2]:
                graph.supports[b].add(a)
                graph.rests_on[a].add(b)
        if not graph.rests_on[a]:
            graph.grounded.add(a)
    return graph


def clearing_order(graph: SupportGraph) -> list:
    """Ids in an order that never picks an object before what rests on it.

    Not what the sequencer uses — it re-decides after every pick from fresh state — but exactly
    what a test needs to assert the graph implies a workable top-down order, and it fails loudly
    on a cycle instead of looping.
    """
    remaining = dict((k, set(v)) for k, v in graph.supports.items())
    out = []
    while remaining:
        free = sorted(k for k, v in remaining.items() if not (v & set(remaining)))
        if not free:
            raise ValueError(f"support graph has a cycle among {sorted(remaining)}")
        out.extend(free)
        for k in free:
            remaining.pop(k)
    return out
