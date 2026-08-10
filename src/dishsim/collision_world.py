# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free FCL collision world for the v0 scene (and the future MCTS rearrangement planner).

Loads exclusively from the ``assets/cache/`` dump (meshes + ``scene_state.json`` manifest —
never from USD or the simulator), so queries run at millisecond scale in any plain Python
process. The manifest's frame convention and config hash are asserted at load: if the scene
config changed since extraction, loading fails loudly instead of answering from a stale world.

Frames: everything in the **robot-base frame** (meters, Z-up, XYZW), matching the manifest.

Geometry model:
- statics (dishwasher bodies incl. both racks, pedestal, ground): CoACD convex pieces (or a
  single hull for the boxes) in one broadphase manager;
- arm links: one inflated convex hull each, posed by :func:`dishsim.ur5e_kin.fk_all_links`;
- gripper links + carried object: rigid cluster on ``wrist_3_link`` (fingers frozen at the
  calibrated grasp aperture — the extraction-time contact-pinch state, which is also the
  aperture held during all planned motion), posed by the cached ``T_wrist3_*`` transforms;
- hull inflation by ``config.COLLISION_MARGIN_M`` biases verdicts conservative (the
  FCL-vs-Isaac parity knob). The 5 mm margin also covers the release-time jaw opening: the
  pads sweep only ~2.6 mm/side outward from the grasp aperture to fully open on the ~80 mm
  mug, so the open-jaw envelope stays inside the inflated grasp-aperture hulls.

Self-collision: PhysX runs the robot with self-collisions DISABLED, so the simulator neither
prevents nor reports them; the planner still refuses self-colliding configurations
(``self_check=True``) using a coarse cluster-vs-proximal-links test. Parity checks against
Isaac must therefore run with ``self_check=False``.
"""

import os

import fcl
import numpy as np
import trimesh

from . import config
from .geometry import coacd_dir_for, load_manifest
from .ur5e_kin import fk_all_links

#: arm links allowed to approach the gripper cluster without being flagged (kinematically
#: adjacent / rigidly attached)
_CLUSTER_SKIP_LINKS = {"wrist_1_link", "wrist_2_link", "wrist_3_link"}
#: arm-arm pairs to check (non-adjacent only; adjacent links overlap by construction)
_ARM_ORDER = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]


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
    """FCL world over the cached v0 scene; hot path is :meth:`in_collision`."""

    def __init__(
        self,
        cache_dir: str = config.CACHE_DIR,
        margin: float = config.COLLISION_MARGIN_M,
        self_check: bool = True,
        object_attached: bool = True,
        merged_cluster: bool = True,
    ):
        """Args:
            merged_cluster: Merge the (rigid) gripper links into one convex hull and the
                carried object into one hull — ~3x faster queries, strictly more conservative.
                The parity check runs with ``False`` (per-link fidelity, the validated model).
        """
        self.cache_dir = cache_dir
        self.margin = margin
        self.self_check = self_check
        self.manifest = load_manifest(cache_dir)

        # ---- statics ------------------------------------------------------------------------
        self._static_objs: dict[str, list[fcl.CollisionObject]] = {}
        self._static_T: dict[str, np.ndarray] = {}
        self._static_lookup: dict[int, str] = {}
        for name, entry in self.manifest["statics"].items():
            T = np.array(entry["T_base_body"])
            self._static_T[name] = T
            pieces = self._load_pieces(name, entry)
            objs = []
            for piece in pieces:
                obj = fcl.CollisionObject(_fcl_convex(piece), _tf(T))
                objs.append(obj)
                self._static_lookup[id(obj)] = name
            self._static_objs[name] = objs
        self._static_mgr = fcl.DynamicAABBTreeCollisionManager()
        self._static_mgr.registerObjects([o for objs in self._static_objs.values() for o in objs])
        self._static_mgr.setup()

        # ---- movables: arm links ------------------------------------------------------------
        self._arm: dict[str, fcl.CollisionObject] = {}
        self._arm_geom_meshes: dict[str, trimesh.Trimesh] = {}
        for name, entry in self.manifest["arm_links"].items():
            mesh = trimesh.load(os.path.join(cache_dir, entry["mesh"]), force="mesh")
            hull = _inflated_hull(mesh, margin)
            self._arm_geom_meshes[name] = hull
            self._arm[name] = fcl.CollisionObject(_fcl_convex(hull))

        # ---- movables: gripper cluster (+ optionally the carried object) ---------------------
        self._merged_cluster = bool(merged_cluster)
        self._cluster: list[tuple[str, np.ndarray, fcl.CollisionObject]] = []  # (name, T_wrist3_geom, obj)
        self._cluster_geom_meshes: dict[str, trimesh.Trimesh] = {}
        cluster_meshes_w3: list[trimesh.Trimesh] = []
        gripper_meshes_w3: list[trimesh.Trimesh] = []
        for name, entry in self.manifest["gripper_links"].items():
            mesh = trimesh.load(os.path.join(cache_dir, entry["mesh"]), force="mesh")
            hull = _inflated_hull(mesh, margin)
            T = np.array(entry["T_wrist3_link"])
            if merged_cluster:
                gripper_meshes_w3.append(hull.copy().apply_transform(T))
            else:
                self._cluster.append((name, T, fcl.CollisionObject(_fcl_convex(hull))))
                self._cluster_geom_meshes[name] = hull
                cluster_meshes_w3.append(hull.copy().apply_transform(T))
        if merged_cluster and gripper_meshes_w3:
            gripper_hull = trimesh.util.concatenate(gripper_meshes_w3).convex_hull
            self._cluster.append(("gripper", np.eye(4), fcl.CollisionObject(_fcl_convex(gripper_hull))))
            self._cluster_geom_meshes["gripper"] = gripper_hull
            cluster_meshes_w3.append(gripper_hull)
        self.object_attached = object_attached
        self._object_objs: list[fcl.CollisionObject] = []
        if object_attached:
            self._object_pieces = self._load_pieces("object", self.manifest["object"])
            T_obj = np.array(self.manifest["object"]["T_wrist3_obj"])
            if merged_cluster:
                whole = trimesh.util.concatenate(self._object_pieces)
                hull = _inflated_hull(whole, margin)
                obj = fcl.CollisionObject(_fcl_convex(hull))
                self._cluster.append(("object_0", T_obj, obj))
                self._cluster_geom_meshes["object_0"] = hull
                self._object_objs.append(obj)
                cluster_meshes_w3.append(hull.copy().apply_transform(T_obj))
            else:
                for i, piece in enumerate(self._object_pieces):
                    hull = _inflated_hull(piece, margin)
                    obj = fcl.CollisionObject(_fcl_convex(hull))
                    self._cluster.append((f"object_{i}", T_obj, obj))
                    self._cluster_geom_meshes[f"object_{i}"] = hull
                    self._object_objs.append(obj)
                    cluster_meshes_w3.append(hull.copy().apply_transform(T_obj))
        # coarse single-hull envelope of the whole cluster for the self-collision test
        cluster_all = trimesh.util.concatenate(cluster_meshes_w3)
        self._cluster_hull_w3 = cluster_all.convex_hull
        self._cluster_hull_obj = fcl.CollisionObject(_fcl_convex(self._cluster_hull_w3))
        self._cluster_hull_dirty = False

        # ---- extra world objects (MCTS mutation API) -----------------------------------------
        self._extra: dict[str, list[fcl.CollisionObject]] = {}

        # arm-arm pairs to check (non-adjacent)
        self._arm_pairs = [
            (a, b)
            for i, a in enumerate(_ARM_ORDER)
            for j, b in enumerate(_ARM_ORDER)
            if j > i + 1 and a in self._arm and b in self._arm
        ]

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

                expected = len(rack_gen.build_rack(config.RACK_GEN[name]))
                if len(files) != expected:
                    raise RuntimeError(
                        f"rack piece dir for '{name}' has {len(files)} pieces, expected {expected} "
                        "(interrupted export?) — re-run scripts/setup/decompose_meshes.py --force"
                    )
            return [trimesh.load(os.path.join(out_dir, f), force="mesh") for f in files]
        return [trimesh.load(os.path.join(self.cache_dir, mesh_rel), force="mesh")]

    def _pose_movables(self, q: np.ndarray) -> dict[str, np.ndarray]:
        fk = fk_all_links(np.asarray(q, dtype=float))
        for name, obj in self._arm.items():
            obj.setTransform(_tf(fk[name]))
        T_w3 = fk["wrist_3_link"]
        for name, T_rel, obj in self._cluster:
            obj.setTransform(_tf(T_w3 @ T_rel))
        self._cluster_hull_obj.setTransform(_tf(T_w3))
        return fk

    # ------------------------------------------------------------------------------------------

    def in_collision(self, q: np.ndarray, return_pairs: bool = False):
        """Robot(+carried object) vs world and (optionally) self collision at joint vector ``q``.

        Args:
            q: Arm joint positions, shape [6].
            return_pairs: Also return the list of offending (body, partner) name pairs.

        Returns:
            bool, or (bool, pairs) when ``return_pairs``.
        """
        fk = self._pose_movables(q)
        pairs: list[tuple[str, str]] = []

        movables = [
            (n, o) for n, o in self._arm.items() if n not in config.WORLD_CHECK_EXCLUDE
        ] + [(n, o) for n, _, o in self._cluster]
        for name, obj in movables:
            cdata = fcl.CollisionData()
            self._static_mgr.collide(obj, cdata, fcl.defaultCollisionCallback)
            if cdata.result.is_collision:
                if not return_pairs:
                    return True
                pairs.append((name, self._resolve_static_partner(obj)))
            for extra_name, objs in self._extra.items():
                for eo in objs:
                    res = fcl.CollisionResult()
                    if fcl.collide(obj, eo, fcl.CollisionRequest(), res):
                        pairs.append((name, extra_name))
                        if not return_pairs:
                            return True

        # carried object vs proximal arm links: the object is a SEPARATE rigid body in PhysX
        # (not an articulation link), so the simulator fully simulates and reports these
        # contacts — this check is always on, regardless of self_check. (Object vs wrist_2/3
        # and vs gripper links is constant by rigidity and deliberately NOT checked: the pads
        # PRESS the object by design — the pinch is verified in-band at build by the
        # scripts/setup/calibrate_grasp.py calibration gates and the scripts/setup/extract_geometry.py grip_gate assert before dump_cache.)
        for link in ("base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link"):
            if link not in self._arm:
                continue
            for obj in self._object_objs:
                res = fcl.CollisionResult()
                if fcl.collide(self._arm[link], obj, fcl.CollisionRequest(), res):
                    pairs.append((link, "carried_object"))
                    if not return_pairs:
                        return True
                    break

        if self.self_check:
            for a, b in self._arm_pairs:
                res = fcl.CollisionResult()
                if fcl.collide(self._arm[a], self._arm[b], fcl.CollisionRequest(), res):
                    pairs.append((a, b))
                    if not return_pairs:
                        return True
            for name in self._arm:
                if name in _CLUSTER_SKIP_LINKS:
                    continue
                res = fcl.CollisionResult()
                if fcl.collide(self._arm[name], self._cluster_hull_obj, fcl.CollisionRequest(), res):
                    pairs.append((name, "gripper_cluster"))
                    if not return_pairs:
                        return True

        hit = len(pairs) > 0
        return (hit, pairs) if return_pairs else hit

    def _resolve_static_partner(self, obj: fcl.CollisionObject) -> str:
        """Identify which static body a movable hits (slow path, only on reported collisions)."""
        for name, objs in self._static_objs.items():
            for so in objs:
                res = fcl.CollisionResult()
                if fcl.collide(obj, so, fcl.CollisionRequest(), res):
                    return name
        return "static"

    def in_collision_batch(self, Q: np.ndarray) -> np.ndarray:
        """Vector of verdicts for configurations ``Q`` of shape [N, 6]."""
        return np.array([self.in_collision(q) for q in np.atleast_2d(Q)], dtype=bool)

    def min_distance(self, q: np.ndarray) -> float:
        """Smallest robot/cluster-to-world distance [m] (negative-ish == in collision).

        Diagnostics / near-contact sampling; slower than :meth:`in_collision`.
        """
        if self.in_collision(q):
            return -1.0
        self._pose_movables(q)
        best = np.inf
        movables = [
            o for n, o in self._arm.items() if n not in config.WORLD_CHECK_EXCLUDE
        ] + [o for _, _, o in self._cluster]
        for obj in movables:
            ddata = fcl.DistanceData()
            self._static_mgr.distance(obj, ddata, fcl.defaultDistanceCallback)
            best = min(best, float(ddata.result.min_distance))
        return best

    # ------------------------------------------------------------------------------------------
    # world-state mutation (MCTS reuse)
    # ------------------------------------------------------------------------------------------

    def add_object(self, name: str, meshes: list[trimesh.Trimesh], T_base_obj: np.ndarray) -> None:
        """Add a free-standing obstacle (e.g. an already-placed item) at ``T_base_obj``."""
        objs = []
        for mesh in meshes:
            obj = fcl.CollisionObject(_fcl_convex(_inflated_hull(mesh, self.margin)), _tf(np.asarray(T_base_obj)))
            objs.append(obj)
        self._extra[name] = objs

    def has_object(self, name: str) -> bool:
        """Is a free-standing obstacle registered under ``name``?"""
        return name in self._extra

    def object_names(self) -> list:
        """Names of the currently registered free-standing obstacles."""
        return sorted(self._extra)

    def set_object_pose(self, name: str, T_base_obj: np.ndarray) -> None:
        for obj in self._extra[name]:
            obj.setTransform(_tf(np.asarray(T_base_obj)))

    def remove_object(self, name: str) -> None:
        self._extra.pop(name, None)

    def set_static_enabled(self, name: str, enabled: bool) -> None:
        """Temporarily drop a static body from (or restore it to) the broadphase.

        Used for the rack being manipulated: the gripper INTENTIONALLY contacts its handle
        through the slide, so FCL validity there must ignore that body — arm-link safety
        against it is enforced by the live unexpected-contact gate during execution instead.
        """
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

    def set_static_offset(self, name: str, offset_y_body: float) -> None:
        """Re-pose a static body's pieces by a translation along its BODY-frame y axis.

        Body y is the rack prismatic axis (the joints' localRot is identity), so this slides a
        rack to an intermediate extension relative to its CACHED pose — used to validate arm
        configurations against the moving rack during the drive-synchronized slide. The cached
        extensions are recorded in ``manifest["statics_state"]``. Pass 0.0 to restore.
        """
        T = self._static_T[name] @ np.array(
            [[1.0, 0, 0, 0], [0, 1.0, 0, float(offset_y_body)], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
        )
        for obj in self._static_objs[name]:
            obj.setTransform(_tf(T))
        self._static_mgr.update()

    def set_static_transform(self, name: str, T_base_body: np.ndarray) -> None:
        """Re-pose a static body's pieces to an ABSOLUTE base-frame transform.

        The base-pose sweep re-expresses the machine statics under a candidate robot base
        (``T_newbase_body = T_delta @ T_oldbase_body``, see :mod:`dishsim.base_sweep`); the
        geometry never rebuilds — only the transforms move, so one loaded world serves
        thousands of candidate poses. Later :meth:`set_static_offset` calls compose against
        the pose set here, so a rack slide validated after a re-base slides the re-based rack.

        Args:
            T_base_body: New base-frame pose of the body, shape [4, 4].
        """
        T = np.asarray(T_base_body, dtype=float).copy()
        self._static_T[name] = T
        for obj in self._static_objs[name]:
            obj.setTransform(_tf(T))
        self._static_mgr.update()

    def carried_object_pieces(self) -> list[trimesh.Trimesh]:
        """Body-frame CoACD pieces of the carried object.

        Grab these *before* :meth:`detach_carried_object` to re-add the object as a placed
        obstacle via :meth:`add_object` (retract-path validation, MCTS placements).
        """
        if not self.object_attached:
            raise RuntimeError("carried object already detached — copy the pieces first")
        return list(self._object_pieces)

    def detach_carried_object(self) -> None:
        """Drop the carried object from the moving cluster (post-release planning queries)."""
        self._cluster = [(n, T, o) for n, T, o in self._cluster if not n.startswith("object_")]
        self._object_objs = []
        self.object_attached = False
        # The coarse cluster envelope must be refreshed here too, not just on attach. Leaving it
        # stale means every empty-handed query after a release runs its self-collision test
        # against the silhouette of the object that is no longer in the hand — measured at 26 of
        # 4000 sampled configurations flipping from free to blocked.
        self._rebuild_cluster_hull()

    def attach_object(self, pieces: list, T_wrist3_obj: np.ndarray) -> None:
        """Attach a payload to the moving cluster — the inverse of :meth:`detach_carried_object`.

        A multi-object task picks, places and picks again, so the cluster has to gain and lose
        payloads repeatedly. Without this the only way to carry a second object would be to
        rebuild the whole world (re-loading every static's convex pieces) between picks.

        The payload's geometry comes from the caller rather than the manifest, so successive
        picks may carry *different* object classes against one world. Merging follows the
        world's ``merged_cluster`` setting, matching how the manifest object would have been
        attached at construction.

        Args:
            pieces: Convex pieces in the object's body frame.
            T_wrist3_obj: Payload pose in the ``wrist_3_link`` frame, shape [4, 4].
        """
        if self.object_attached:
            self.detach_carried_object()
        if not pieces:
            raise ValueError("attach_object needs at least one convex piece")
        self._object_pieces = [p.copy() for p in pieces]
        T_obj = np.asarray(T_wrist3_obj, dtype=float)
        self._object_objs = []
        if self._merged_cluster:
            whole = trimesh.util.concatenate(self._object_pieces)
            hull = _inflated_hull(whole, self.margin)
            obj = fcl.CollisionObject(_fcl_convex(hull))
            self._cluster.append(("object_0", T_obj, obj))
            self._cluster_geom_meshes["object_0"] = hull
            self._object_objs.append(obj)
        else:
            for i, piece in enumerate(self._object_pieces):
                hull = _inflated_hull(piece, self.margin)
                obj = fcl.CollisionObject(_fcl_convex(hull))
                self._cluster.append((f"object_{i}", T_obj, obj))
                self._cluster_geom_meshes[f"object_{i}"] = hull
                self._object_objs.append(obj)
        self.object_attached = True
        self._rebuild_cluster_hull()

    def _rebuild_cluster_hull(self) -> None:
        """Refresh the coarse cluster envelope used by the self-collision test.

        The envelope is a single convex hull over every cluster member. Leaving it stale after a
        payload change would test self-collision against the *previous* payload's silhouette —
        a false verdict in both directions.
        """
        meshes = []
        for name, T, _ in self._cluster:
            mesh = self._cluster_geom_meshes.get(name)
            if mesh is not None:
                meshes.append(mesh.copy().apply_transform(T))
        if not meshes:
            return
        self._cluster_hull_w3 = trimesh.util.concatenate(meshes).convex_hull
        self._cluster_hull_obj = fcl.CollisionObject(_fcl_convex(self._cluster_hull_w3))
        self._cluster_hull_dirty = False
