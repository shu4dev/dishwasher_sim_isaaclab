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
- gripper links + carried object: rigid cluster on ``wrist_3_link`` (frozen fingers), posed by
  the cached ``T_wrist3_*`` transforms;
- hull inflation by ``config.COLLISION_MARGIN_M`` biases verdicts conservative (the
  FCL-vs-Isaac parity knob).

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
        self._static_lookup: dict[int, str] = {}
        for name, entry in self.manifest["statics"].items():
            T = np.array(entry["T_base_body"])
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
        self._cluster: list[tuple[str, np.ndarray, fcl.CollisionObject]] = []  # (name, T_wrist3_geom, obj)
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
                cluster_meshes_w3.append(hull.copy().apply_transform(T))
        if merged_cluster and gripper_meshes_w3:
            gripper_hull = trimesh.util.concatenate(gripper_meshes_w3).convex_hull
            self._cluster.append(("gripper", np.eye(4), fcl.CollisionObject(_fcl_convex(gripper_hull))))
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
                self._object_objs.append(obj)
                cluster_meshes_w3.append(hull.copy().apply_transform(T_obj))
            else:
                for i, piece in enumerate(self._object_pieces):
                    hull = _inflated_hull(piece, margin)
                    obj = fcl.CollisionObject(_fcl_convex(hull))
                    self._cluster.append((f"object_{i}", T_obj, obj))
                    self._object_objs.append(obj)
                    cluster_meshes_w3.append(hull.copy().apply_transform(T_obj))
        # coarse single-hull envelope of the whole cluster for the self-collision test
        cluster_all = trimesh.util.concatenate(cluster_meshes_w3)
        self._cluster_hull_w3 = cluster_all.convex_hull
        self._cluster_hull_obj = fcl.CollisionObject(_fcl_convex(self._cluster_hull_w3))

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
                    f"missing CoACD pieces for '{name}' — run scripts/13_decompose_meshes.py first"
                )
            return [
                trimesh.load(os.path.join(out_dir, f), force="mesh") for f in sorted(os.listdir(out_dir))
            ]
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
        # and vs gripper links is constant by rigidity — verified free at build.)
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

    def set_object_pose(self, name: str, T_base_obj: np.ndarray) -> None:
        for obj in self._extra[name]:
            obj.setTransform(_tf(np.asarray(T_base_obj)))

    def remove_object(self, name: str) -> None:
        self._extra.pop(name, None)

    def detach_carried_object(self) -> None:
        """Drop the carried object from the moving cluster (post-release planning queries)."""
        self._cluster = [(n, T, o) for n, T, o in self._cluster if not n.startswith("object_")]
        self._object_objs = []
        self.object_attached = False
