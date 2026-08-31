# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free FCL collision world for arrangement planning.

Loads exclusively from the ``assets/cache/`` dump (meshes + ``scene_state.json`` manifest —
never from USD or the simulator), so queries run at millisecond scale in any plain Python
process. The manifest's frame convention and config hash are asserted at load: if the scene
config changed since extraction, loading fails loudly instead of answering from a stale world.

Frames: everything in the **base frame** (meters, Z-up, XYZW), matching the manifest.

Geometry model:
- statics (dishwasher bodies incl. the racks, pedestal, ground): CoACD convex pieces (or a
  single hull for the boxes) in one broadphase manager;
- placed objects (:meth:`CollisionWorld.add_object`) and candidate objects
  (:meth:`CollisionWorld.object_in_collision`): one inflated convex hull per CoACD piece.
  Hull inflation by ``config.COLLISION_MARGIN_M`` biases verdicts conservative.

The hot path is :meth:`CollisionWorld.object_in_collision` — "would this object, teleported to
this pose, interpenetrate the machine or any already-placed object?"
"""

import os

import fcl
import numpy as np
import trimesh

from . import config
from .geometry import coacd_dir_for, load_manifest


def _fcl_convex(mesh: trimesh.Trimesh) -> fcl.Convex:
    hull = mesh.convex_hull
    faces = np.hstack([np.full((len(hull.faces), 1), 3, dtype=np.int64), hull.faces.astype(np.int64)])
    return fcl.Convex(hull.vertices.astype(np.float64), len(hull.faces), faces.flatten())


def _inflated_hull(mesh: trimesh.Trimesh, margin: float) -> trimesh.Trimesh:
    hull = mesh.convex_hull
    if margin > 0:
        pushed = hull.vertices + hull.vertex_normals * margin
        hull = trimesh.Trimesh(vertices=pushed).convex_hull
    return hull


def _tf(T: np.ndarray) -> fcl.Transform:
    return fcl.Transform(T[:3, :3], T[:3, 3])


def load_object_pieces(cache_dir: str) -> list:
    """Convex pieces of a cache's carried object, without building a whole world.

    A multi-object episode carries a different class on each pick but plans against ONE world,
    so it needs each class's geometry cheaply. Constructing a :class:`CollisionWorld` per class
    would re-load every static's decomposition to get at one object.

    Args:
        cache_dir: Collision-cache root (see :func:`dishsim.config.scenario_cache_dir`).

    Returns:
        Convex pieces in the object's body frame.
    """
    manifest = load_manifest(cache_dir)
    entry = manifest["object"]
    if not entry.get("coacd"):
        return [trimesh.load(os.path.join(cache_dir, entry["mesh"]), force="mesh")]
    out_dir = coacd_dir_for("object", entry["mesh"], cache_dir)
    if not os.path.isdir(out_dir) or not os.listdir(out_dir):
        raise RuntimeError(
            f"missing CoACD pieces for the object in {cache_dir} — "
            "run scripts/setup/decompose_meshes.py first"
        )
    return [trimesh.load(os.path.join(out_dir, f), force="mesh") for f in sorted(os.listdir(out_dir))]


class CollisionWorld:
    """FCL world over the cached scene statics; hot path is :meth:`object_in_collision`.

    Shipped robot-era manifests also carry ``arm_links``/``gripper_links``/``object`` sections;
    only ``statics`` is read — the extra keys are ignored, not errors.
    """

    def __init__(
        self,
        cache_dir: str = config.CACHE_DIR,
        margin: float = config.COLLISION_MARGIN_M,
    ):
        self.cache_dir = cache_dir
        self.margin = margin
        self.manifest = load_manifest(cache_dir)

        # ---- statics ------------------------------------------------------------------------
        self._static_objs: dict[str, list[fcl.CollisionObject]] = {}
        for name, entry in self.manifest["statics"].items():
            T = np.array(entry["T_base_body"])
            pieces = self._load_pieces(name, entry)
            self._static_objs[name] = [fcl.CollisionObject(_fcl_convex(piece), _tf(T)) for piece in pieces]
        self._static_mgr = fcl.DynamicAABBTreeCollisionManager()
        self._static_mgr.registerObjects([o for objs in self._static_objs.values() for o in objs])
        self._static_mgr.setup()

        # ---- placed objects (arrangement mutation API) ---------------------------------------
        self._extra: dict[str, list[fcl.CollisionObject]] = {}
        self._extra_keys: dict[str, tuple] = {}
        # Placed obstacles get their own broadphase: a linear scan over every placed item's every
        # piece is what dominates once the hulls are cached (12.9 ms vs 1.8 ms for a 32-piece bowl
        # against 8 placed neighbours). Registration is kept incremental, mirroring the static
        # manager — the tree only answers "does anything hit?", so the name-resolving path below
        # still walks the items themselves.
        self._extra_mgr = fcl.DynamicAABBTreeCollisionManager()
        self._extra_mgr.setup()

        # ---- candidate-geometry caches (the hot path) ----------------------------------------
        # Inflating one piece's hull costs ~10 ms (two ``convex_hull`` passes plus the
        # ``fcl.Convex`` build) and is a function of (piece, margin) ONLY — never of the queried
        # pose. Rebuilding it inside the per-query loop made a free `object_in_collision` cost
        # ~219 ms for a 32-piece bowl; caching the geometry and re-posing one persistent
        # `fcl.CollisionObject` per piece via ``setTransform`` takes the same query to ~1.8 ms
        # (~120x) and `add_object` from ~216 ms to ~0.01 ms — measured 2026-08 on
        # bosch800 / placement / side_winner, bowl + plate, verdicts and blocker sets identical
        # over every derived slot and buffer cell at 0/4/8 placed neighbours.
        #
        # Keying: ``id(piece)`` is only safe because the value keeps a STRONG reference to the
        # mesh, so CPython can never recycle the id of a collected mesh into a live entry. The
        # margin is part of the key as well, so a cache can never hand back geometry inflated by
        # a different margin even if one is introduced per call later.
        self._hull_cache: dict[tuple[int, float], tuple[trimesh.Trimesh, fcl.Convex]] = {}
        self._probe_objs: dict[tuple[int, float], fcl.CollisionObject] = {}

    # ------------------------------------------------------------------------------------------

    def _inflated_convex(self, piece: trimesh.Trimesh, margin: float) -> fcl.Convex:
        """Margin-inflated convex hull of ``piece`` as FCL geometry, built at most once.

        Args:
            piece: Convex piece in its object's body frame.
            margin: Hull inflation [m].

        Returns:
            The cached :class:`fcl.Convex`. FCL geometry is read-only during collision, so one
            instance is safe to share across every collision object that needs it.
        """
        key = (id(piece), float(margin))
        entry = self._hull_cache.get(key)
        if entry is None:
            entry = (piece, _fcl_convex(_inflated_hull(piece, margin)))
            self._hull_cache[key] = entry
        return entry[1]

    def _probe_object(self, piece: trimesh.Trimesh, T: np.ndarray) -> fcl.CollisionObject:
        """The reusable query proxy for ``piece``, re-posed to ``T``.

        One collision object per piece is enough because a query only ever holds one pose per
        piece at a time (single-threaded, no re-entrancy): posing it is a ``setTransform``,
        which python-fcl follows with ``computeAABB()`` — required, since the static broadphase
        queries the candidate's AABB.
        """
        key = (id(piece), float(self.margin))
        obj = self._probe_objs.get(key)
        if obj is None:
            obj = fcl.CollisionObject(self._inflated_convex(piece, self.margin), _tf(T))
            self._probe_objs[key] = obj
        else:
            obj.setTransform(_tf(T))
        return obj

    # ------------------------------------------------------------------------------------------

    def _load_pieces(self, name: str, entry: dict) -> list[trimesh.Trimesh]:
        mesh_rel = entry["mesh"]
        if entry.get("coacd"):
            out_dir = coacd_dir_for(name, mesh_rel, self.cache_dir)
            if not os.path.isdir(out_dir) or not os.listdir(out_dir):
                raise RuntimeError(
                    f"missing CoACD pieces for '{name}' — run scripts/setup/decompose_meshes.py first"
                )
            files = sorted(os.listdir(out_dir))
            if name in config.RACK_GEN:
                # rack piece counts are deterministic (rack_gen parts) — reject a piece dir
                # truncated by an interrupted scripts/setup/decompose_meshes.py export, where a missing piece would be
                # a physically present wire that FCL never sees
                from . import rack_gen  # local import: only needed on the rack path

                # route through the builder dispatcher (the Bosch third rack is a tray)
                expected = len(rack_gen.build(config.RACK_GEN[name]))
                if len(files) != expected:
                    raise RuntimeError(
                        f"rack piece dir for '{name}' has {len(files)} pieces, expected {expected} "
                        "(interrupted export?) — re-run scripts/setup/decompose_meshes.py --force"
                    )
            return [trimesh.load(os.path.join(out_dir, f), force="mesh") for f in files]
        return [trimesh.load(os.path.join(self.cache_dir, mesh_rel), force="mesh")]

    # ------------------------------------------------------------------------------------------

    def object_in_collision(
        self, pieces: list[trimesh.Trimesh], T_base_obj: np.ndarray, return_pairs: bool = False
    ):
        """Would an object at ``T_base_obj`` interpenetrate the statics or any placed object?

        The teleport-feasibility primitive: pose the candidate's body-frame convex pieces
        (inflated by the world margin, matching :meth:`add_object`) and collide them against
        the static broadphase plus every ``add_object`` obstacle.

        Args:
            pieces: Convex pieces in the object's body frame (see :func:`load_object_pieces`).
            T_base_obj: Candidate object pose in the base frame, shape [4, 4].
            return_pairs: Also return the offending (piece, partner) name pairs.

        Returns:
            bool, or (bool, pairs) when ``return_pairs``.
        """
        T = np.asarray(T_base_obj, dtype=float)
        pairs: list[tuple[str, str]] = []
        for i, piece in enumerate(pieces):
            obj = self._probe_object(piece, T)
            cdata = fcl.CollisionData()
            self._static_mgr.collide(obj, cdata, fcl.defaultCollisionCallback)
            if cdata.result.is_collision:
                if not return_pairs:
                    return True
                pairs.append((f"piece_{i}", self._resolve_static_partner(obj)))
            if self._extra and self._hits_extra(obj):
                if not return_pairs:
                    return True
                # the broadphase only says THAT something hit; naming every offending item still
                # needs the per-item scan, and it only runs on the (rare) reported collision
                for extra_name, objs in self._extra.items():
                    for eo in objs:
                        res = fcl.CollisionResult()
                        if fcl.collide(obj, eo, fcl.CollisionRequest(), res):
                            pairs.append((f"piece_{i}", extra_name))
                            break
        hit = len(pairs) > 0
        return (hit, pairs) if return_pairs else hit

    def _hits_extra(self, obj: fcl.CollisionObject) -> bool:
        """Does ``obj`` hit any placed obstacle? (broadphase; same narrowphase test as the scan)."""
        cdata = fcl.CollisionData()
        self._extra_mgr.collide(obj, cdata, fcl.defaultCollisionCallback)
        return bool(cdata.result.is_collision)

    def _resolve_static_partner(self, obj: fcl.CollisionObject) -> str:
        """Identify which static body a candidate hits (slow path, only on reported collisions)."""
        for name, objs in self._static_objs.items():
            for so in objs:
                res = fcl.CollisionResult()
                if fcl.collide(obj, so, fcl.CollisionRequest(), res):
                    return name
        return "static"

    # ------------------------------------------------------------------------------------------
    # world-state mutation (placed objects, rack states)
    # ------------------------------------------------------------------------------------------

    def add_object(self, name: str, meshes: list[trimesh.Trimesh], T_base_obj: np.ndarray) -> None:
        """Add a free-standing obstacle (e.g. an already-placed item) at ``T_base_obj``.

        Shares the inflated hulls with :meth:`object_in_collision` (same cache, same margin), so
        adding an obstacle whose class has already been queried costs no geometry work. Re-adding
        the same name with the same meshes only re-poses the existing collision objects — a
        placed item that moves must not pay for a rebuild.
        """
        tf = _tf(np.asarray(T_base_obj))
        keys = tuple((id(mesh), float(self.margin)) for mesh in meshes)
        if self._extra_keys.get(name) == keys:
            for obj in self._extra[name]:
                obj.setTransform(tf)
            self._extra_mgr.update()
            return
        for obj in self._extra.pop(name, []):
            self._extra_mgr.unregisterObject(obj)
        objs = [fcl.CollisionObject(self._inflated_convex(mesh, self.margin), tf) for mesh in meshes]
        self._extra[name] = objs
        self._extra_keys[name] = keys
        self._extra_mgr.registerObjects(objs)
        self._extra_mgr.update()

    def has_object(self, name: str) -> bool:
        """Is a free-standing obstacle registered under ``name``?"""
        return name in self._extra

    def set_object_pose(self, name: str, T_base_obj: np.ndarray) -> None:
        tf = _tf(np.asarray(T_base_obj))
        for obj in self._extra[name]:
            obj.setTransform(tf)
        self._extra_mgr.update()

    def remove_object(self, name: str) -> None:
        for obj in self._extra.pop(name, []):
            self._extra_mgr.unregisterObject(obj)
        self._extra_keys.pop(name, None)
        self._extra_mgr.update()

    def set_static_enabled(self, name: str, enabled: bool) -> None:
        """Temporarily drop a static body from (or restore it to) the broadphase."""
        disabled = getattr(self, "_static_disabled", set())
        self._static_disabled = disabled
        if enabled and name in disabled:
            for obj in self._static_objs[name]:
                self._static_mgr.registerObject(obj)
            disabled.discard(name)
        elif not enabled and name not in disabled:
            for obj in self._static_objs[name]:
                self._static_mgr.unregisterObject(obj)
            disabled.add(name)
        self._static_mgr.update()
