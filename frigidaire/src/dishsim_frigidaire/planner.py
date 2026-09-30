# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Bench-side planning helpers for the Frigidaire FDPC4221AS: world, support gate and sequencer.

Kit-free. Objects start in a messy pile on a countertop above the machine and unorganized in the
racks; the HOTEC benchmark (``frigidaire/scripts/experiment/frigidaire_bench.py``) plans teleport
moves, one object at a time, on the driver ``dishsim.rearrange.run_episode``. This module supplies
the world (an FCL mirror of the appliance, the counter slab and every object), the support graph
and the sequencer (the benchmark's greedy rule with counter objects first, the support gate and
the counter cap mirrored so no refused command is ever emitted).

Frames: an instance stores racks-OUT world poses (the Isaac scene); FCL checks run there with the
instance's measured component frames. Counter objects are neither scored nor occluders. Goal poses
are certified upstream, so only their pairwise clearance is re-checked per move; buffer cells are
checked against everything.

The pool-search planner (ExposurePlanner, FirstFitBaseline, the geometric pool and its result
files) was retired with the v3-era experiments on 2026-09-29; it remains in git history.
"""
from __future__ import annotations

import numpy as np

from dishsim.rearrange import Instance, Move
from dishsim.transforms import make_T, T_to_pos_quat
from .paths import ASSET_DIR
from .random_poses import compose_pose, relative_pose

COUNTER = {"size_m": (1.2, .6, .04), "center_m": (0., 0., .894), "top_z_m": .914}   # worktop over the cabinet (top .8509)
BUFFER_PITCH_M = .13          # bowl diameter .14: neighbouring cells cannot both hold a bowl, FCL decides
BUFFER_HOVER_M = .002
BAND_TOLERANCE_M = .01        # an object resting flat on the slab has its origin AT the top; the band starts 1 cm lower
SUPPORT_MIN_DZ_M = .005       # a dish-dish contact whose upper centre sits this much higher is a support edge


# ----------------------------------------------------------------------------- poses
def pose_dict(T):
    p, q = T_to_pos_quat(T)
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def pose_T(pose):
    return make_T(pose["position_m"], pose["quaternion_xyzw"])


def world_from_local(frame, local):
    p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], local["position_m"], local["quaternion_xyzw"])
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def local_from_world(frame, world):
    p, q = relative_pose(world["position_m"], world["quaternion_xyzw"], frame["position_m"], frame["quaternion_xyzw"])
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def in_counter_band(T, counter=None):
    """Over the slab and above its top: the benchmark's counter predicate for this machine."""
    counter = counter or COUNTER
    x, y, z = np.asarray(T, dtype=float)[:3, 3]
    cx, cy, _ = counter["center_m"]
    hx, hy = counter["size_m"][0] / 2, counter["size_m"][1] / 2
    return bool(abs(x - cx) <= hx and abs(y - cy) <= hy and z > counter["top_z_m"] - BAND_TOLERANCE_M)


def _key(T):
    return tuple(np.round(np.asarray(T, dtype=float)[:3].ravel(), 7))


# ----------------------------------------------------------------------------- support graph
def support_edges(contact_pairs, poses, ids):
    """Directed edges (supporter, supported) from settled dish-dish contacts: the higher centre is supported."""
    ids, edges = set(ids), set()
    for a, b in contact_pairs:
        if a not in ids or b not in ids or a == b:
            continue
        dz = float(poses[b]["position_m"][2]) - float(poses[a]["position_m"][2])
        if dz > SUPPORT_MIN_DZ_M:
            edges.add((a, b))
        elif -dz > SUPPORT_MIN_DZ_M:
            edges.add((b, a))
    return sorted(list(e) for e in edges)


# ----------------------------------------------------------------------------- world
def slab_body(checker, counter=None):
    """The counter slab as a body the checker's ``pair`` accepts."""
    fcl = checker.fcl
    counter = counter or COUNTER
    size, center = np.asarray(counter["size_m"], dtype=float), np.asarray(counter["center_m"], dtype=float)
    obj = fcl.CollisionObject(fcl.Box(*size), fcl.Transform(np.eye(3), center))
    manager = fcl.DynamicAABBTreeCollisionManager()
    manager.registerObjects([obj])
    manager.setup()
    return {"id": "Counter", "kind": "Counter", "objects": [obj], "manager": manager,
            "bounds": (center - size / 2, center + size / 2)}


class PlannerWorld:
    """FCL mirror with the benchmark's world duck-type (sync/move_collides/blockers/buffer_poses/in_counter)."""

    def __init__(self, instance, asset_dir=ASSET_DIR, checker=None):
        from .initial_state_candidates import COMPONENT_NAMES, InitialCollisionChecker
        from .organization import OrganizationGeometry
        self.checker = checker or InitialCollisionChecker(asset_dir)
        snap = instance["initial_snapshot"]["poses"]
        self.frames = {name: snap[name] for name in COMPONENT_NAMES}
        self.checker.update_components(self.frames)
        self.geometry = OrganizationGeometry(checker=self.checker)
        self.slab = self._slab_body()
        self.classes, self._poses, self._bodies, self._buffer = {}, {}, {}, {}
        self.initial = {o["object_id"]: pose_T(o["pose_world"]) for o in instance["objects"]}
        self.support = [tuple(e) for e in instance.get("support", [])]
        self.certified = set()
        self.n_queries = 0

    def _slab_body(self):
        return slab_body(self.checker)

    def body(self, kind, T, key):
        return self.checker._body(kind, pose_dict(T), key)

    def sync(self, poses, classes):
        self.classes.update(classes)
        for k, T in poses.items():
            T = np.asarray(T, dtype=float)
            if k not in self._poses or not np.allclose(self._poses[k], T, atol=1e-9):
                self._poses[k] = T.copy()
                self._bodies[k] = self.body(self.classes[k], T, k)

    def snapshot(self):
        return {k: v.copy() for k, v in self._poses.items()}

    def clear(self):
        self._poses.clear()
        self._bodies.clear()

    def certify(self, T):
        """Mark a pose as settled-certified: the appliance check is skipped for it (pairs are not)."""
        self.certified.add(_key(T))

    def _collisions(self, item_id, T):
        body = self.body(self.classes[item_id], T, item_id)
        self.n_queries += 1
        hits = []
        if _key(T) not in self.certified and not self.checker.against_components(body)["valid"]:
            hits.append("appliance")
        if not self.checker.pair(body, self.slab)["valid"]:
            hits.append("Counter")
        for k, other in self._bodies.items():
            if k != item_id and not self.checker.pair(body, other)["valid"]:
                hits.append(k)
        return hits

    def move_collides(self, item_id, T, object_class=None):
        return bool(self._collisions(item_id, T))

    def blockers(self, item_id, T, object_class=None):
        return sorted(self._collisions(item_id, T))

    def buffer_poses(self, kind):
        if kind not in self._buffer:
            lower = self.checker.bounds[kind][0]
            z = COUNTER["top_z_m"] + BUFFER_HOVER_M - float(lower[2])
            cx, cy, _ = COUNTER["center_m"]
            hx, hy = COUNTER["size_m"][0] / 2 - .05, COUNTER["size_m"][1] / 2 - .05
            xs, ys = np.arange(-hx, hx + 1e-9, BUFFER_PITCH_M), np.arange(-hy, hy + 1e-9, BUFFER_PITCH_M)
            self._buffer[kind] = [make_T((cx + x, cy + y, z), (0., 0., 0., 1.)) for y in ys for x in xs]
        return self._buffer[kind]

    def in_counter(self, T):
        return in_counter_band(T, getattr(self, "counter", None))

    def resting_on(self, item_id):
        """Objects still resting on ``item_id`` (support edge and the supported object never moved)."""
        return [b for a, b in self.support
                if a == item_id and b in self._poses and np.allclose(self._poses[b], self.initial[b], atol=1e-6)]


# ----------------------------------------------------------------------------- instance <-> driver
def item_order(instance):
    """Counter objects first, top of the pile first, then inside objects in file order."""
    counter = [o for o in instance["objects"] if o["start"] == "Counter"]
    inside = [o for o in instance["objects"] if o["start"] != "Counter"]
    counter.sort(key=lambda o: -float(o["pose_world"]["position_m"][2]))
    return [o["object_id"] for o in counter + inside]


def to_rearrange_instance(instance):
    items = [{"item_id": o["object_id"], "object_class": o["kind"], "T_base_init": pose_T(o["pose_world"]),
              "target": {"T_base_obj": None}} for o in instance["objects"]]
    r = Instance(name=instance["instance_id"], machine="frigidaire", base_placement="none", state="racks_out",
                 items=items, meta={"n_objects": len(items), "counter_cap": instance["counter"]["cap"],
                                    "inventory": instance["inventory"], "seed": instance.get("seed")})
    r.planner_instance = instance
    return r


def goal_T(world, rack, local):
    return pose_T(world_from_local(world.frames[rack], local))


# ----------------------------------------------------------------------------- sequencer
def sequence(world, goals, order, cap, max_moves=None):
    """Greedy with buffering, dry-run on the mirror; a Move list, or None when it cannot finish.

    Pass 1 sends home any misplaced, unsupported object whose goal is free. Pass 2 parks one
    blocker on the counter, only while the band holds fewer than ``cap`` objects (the driver
    would refuse it otherwise). Objects that still support another never move.
    """
    saved = world.snapshot()
    poses = dict(saved)
    kinds = dict(world.classes)
    relocated, plan = set(), []
    limit = max_moves or 10 * max(1, len(order))

    def at(i):
        return bool(np.allclose(poses[i], goals[i], atol=1e-6))

    try:
        while not all(at(i) for i in order) and len(plan) < limit:
            step = None
            for i in order:
                if at(i) or world.resting_on(i):
                    continue
                if not world.move_collides(i, goals[i]):
                    step = (i, goals[i])
                    break
            if step is None and sum(world.in_counter(T) for T in poses.values()) < cap:
                for i in order:
                    if at(i):
                        continue
                    for b in world.blockers(i, goals[i]):
                        if b not in goals or at(b) or b in relocated or world.resting_on(b):
                            continue
                        T_buf = next((T for T in world.buffer_poses(kinds[b]) if not world.move_collides(b, T)), None)
                        if T_buf is not None:
                            relocated.add(b)
                            step = (b, T_buf)
                            break
                    if step is not None:
                        break
            if step is None:
                break
            plan.append(Move(step[0], np.asarray(step[1], dtype=float)))
            poses[step[0]] = np.asarray(step[1], dtype=float)
            world.sync({step[0]: poses[step[0]]}, kinds)
        done = all(at(i) for i in order)
    finally:
        world.sync(saved, kinds)
    return plan if done else None
