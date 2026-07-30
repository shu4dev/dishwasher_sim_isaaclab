# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""python-fcl + trimesh smoke test: mesh-vs-mesh collision, convex objects, broadphase manager."""

import fcl
import numpy as np
import trimesh


def _bvh(mesh: trimesh.Trimesh) -> fcl.BVHModel:
    model = fcl.BVHModel()
    model.beginModel(len(mesh.vertices), len(mesh.faces))
    model.addSubModel(mesh.vertices.astype(np.float64), mesh.faces.astype(np.int64))
    model.endModel()
    return model


def _convex(mesh: trimesh.Trimesh) -> fcl.Convex:
    hull = mesh.convex_hull
    faces_flat = np.hstack([np.full((len(hull.faces), 1), 3, dtype=np.int64), hull.faces.astype(np.int64)])
    return fcl.Convex(hull.vertices.astype(np.float64), len(hull.faces), faces_flat.flatten())


def test_bvh_collision_and_separation():
    box = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    g1, g2 = _bvh(box), _bvh(box)
    o1 = fcl.CollisionObject(g1, fcl.Transform())

    o2 = fcl.CollisionObject(g2, fcl.Transform(np.array([0.5, 0.0, 0.0])))
    res = fcl.CollisionResult()
    fcl.collide(o1, o2, fcl.CollisionRequest(), res)
    assert res.is_collision, "overlapping boxes must collide"

    o3 = fcl.CollisionObject(g2, fcl.Transform(np.array([3.0, 0.0, 0.0])))
    res = fcl.CollisionResult()
    fcl.collide(o1, o3, fcl.CollisionRequest(), res)
    assert not res.is_collision, "separated boxes must not collide"


def test_convex_collision_and_distance():
    box = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    c1 = fcl.CollisionObject(_convex(box), fcl.Transform())
    c2 = fcl.CollisionObject(_convex(box), fcl.Transform(np.array([0.8, 0.0, 0.0])))
    res = fcl.CollisionResult()
    fcl.collide(c1, c2, fcl.CollisionRequest(), res)
    assert res.is_collision

    c3 = fcl.CollisionObject(_convex(box), fcl.Transform(np.array([2.0, 0.0, 0.0])))
    dres = fcl.DistanceResult()
    d = fcl.distance(c1, c3, fcl.DistanceRequest(), dres)
    assert abs(d - 1.0) < 1e-3, f"expected 1.0 m gap between unit boxes 2 m apart, got {d}"


def test_broadphase_manager_round_trip():
    box = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    statics = [
        fcl.CollisionObject(_bvh(box), fcl.Transform(np.array([float(i * 3), 0.0, 0.0]))) for i in range(4)
    ]
    mgr = fcl.DynamicAABBTreeCollisionManager()
    mgr.registerObjects(statics)
    mgr.setup()

    probe_hit = fcl.CollisionObject(_bvh(box), fcl.Transform(np.array([3.2, 0.0, 0.0])))
    cdata = fcl.CollisionData()
    mgr.collide(probe_hit, cdata, fcl.defaultCollisionCallback)
    assert cdata.result.is_collision

    probe_free = fcl.CollisionObject(_bvh(box), fcl.Transform(np.array([1.5, 3.0, 0.0])))
    cdata = fcl.CollisionData()
    mgr.collide(probe_free, cdata, fcl.defaultCollisionCallback)
    assert not cdata.result.is_collision
