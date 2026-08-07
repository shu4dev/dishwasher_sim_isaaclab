# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""``CollisionWorld.attach_object`` — the inverse of ``detach_carried_object``.

A multi-object task picks, places, and picks again, so the moving cluster must gain and lose
payloads repeatedly against ONE world. Before this existed, ``detach_carried_object`` was
one-way and the only way to carry a second object was to rebuild the world from the cache.

The subtle part is the coarse cluster envelope used by the self-collision test: it is a single
convex hull over every cluster member, so a payload swap that forgot to rebuild it would keep
testing against the previous payload's silhouette — wrong in both directions, and invisible
unless asserted.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_collision_world_payload.py -v
"""

import os

import numpy as np
import pytest
import trimesh

from dishsim import config
from dishsim.collision_world import CollisionWorld

needs_cache = pytest.mark.skipif(
    not os.path.exists(os.path.join(config.CACHE_DIR, "scene_state.json")),
    reason="baseline geometry cache not built yet (run scripts/setup/extract_geometry.py)",
)


@pytest.fixture()
def world():
    return CollisionWorld(cache_dir=config.CACHE_DIR, self_check=True)


@needs_cache
def test_detach_then_attach_restores_a_payload(world):
    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    world.detach_carried_object()
    assert not world.object_attached

    world.attach_object(pieces, T)
    assert world.object_attached
    assert any(n.startswith("object_") for n, _, _ in world._cluster)


@needs_cache
def test_reattaching_the_same_payload_reproduces_the_original_verdicts(world):
    """A detach/attach round trip must be a no-op, or carrying is not repeatable."""
    rng = np.random.default_rng(0)
    qs = [rng.uniform(-1.5, 1.5, 6) for _ in range(200)]
    before = [bool(world.in_collision(q)) for q in qs]

    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    world.detach_carried_object()
    world.attach_object(pieces, T)

    after = [bool(world.in_collision(q)) for q in qs]
    assert after == before, f"{sum(a != b for a, b in zip(before, after))}/{len(qs)} verdicts changed"


@needs_cache
def test_attaching_never_leaves_two_payloads(world):
    """Pick-place-pick must not accumulate payloads in the cluster."""
    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    for _ in range(3):
        world.attach_object(pieces, T)
    n_object = sum(1 for n, _, _ in world._cluster if n.startswith("object_"))
    assert n_object == len(world._object_objs)
    # merged_cluster=True (the default) collapses a payload to exactly one hull
    assert n_object == 1


@needs_cache
def test_a_bigger_payload_makes_strictly_more_configurations_collide(world):
    """Attaching different geometry must actually change what the world reports.

    Guards the failure this class of bug produces: an attach that updates bookkeeping but not
    the FCL objects would leave verdicts identical and look like it worked.
    """
    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    rng = np.random.default_rng(1)
    qs = [rng.uniform(-1.5, 1.5, 6) for _ in range(200)]
    base = [bool(world.in_collision(q)) for q in qs]

    big = [trimesh.creation.box(extents=(0.30, 0.30, 0.30))]
    world.attach_object(big, T)
    grown = [bool(world.in_collision(q)) for q in qs]

    assert sum(grown) > sum(base), (sum(base), sum(grown))
    assert all(g for b, g in zip(base, grown) if b), "a larger payload cannot un-collide a config"


@needs_cache
def test_self_collision_envelope_tracks_the_attached_payload(world):
    """The coarse cluster hull must be rebuilt on attach, not left stale."""
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    small_extent = world._cluster_hull_w3.extents.copy()

    world.attach_object([trimesh.creation.box(extents=(0.40, 0.40, 0.40))], T)
    grown_extent = world._cluster_hull_w3.extents.copy()
    assert np.any(grown_extent > small_extent + 1e-6), (small_extent, grown_extent)

    world.detach_carried_object()
    world.attach_object([trimesh.creation.box(extents=(0.01, 0.01, 0.01))], T)
    shrunk_extent = world._cluster_hull_w3.extents.copy()
    assert np.all(shrunk_extent < grown_extent + 1e-6)


@needs_cache
def test_attach_rejects_empty_geometry(world):
    with pytest.raises(ValueError):
        world.attach_object([], np.eye(4))


@needs_cache
def test_carried_pieces_must_be_copied_before_detaching(world):
    """The documented order-of-operations trap stays enforced."""
    world.detach_carried_object()
    with pytest.raises(RuntimeError):
        world.carried_object_pieces()


@needs_cache
def test_detaching_restores_the_empty_handed_envelope(world):
    """Detach must refresh the self-collision hull, not just drop the payload objects.

    Leaving it stale means every empty-handed query after a release tests self-collision against
    the silhouette of an object that is no longer in the hand. The error is conservative — it can
    only reject free configurations, never approve colliding ones — but it silently shrinks the
    reachable set after the first place in a multi-object episode.
    """
    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    world.detach_carried_object()
    empty_extent = world._cluster_hull_w3.extents.copy()

    world.attach_object([trimesh.creation.box(extents=(0.30, 0.30, 0.30))], T)
    assert np.any(world._cluster_hull_w3.extents > empty_extent + 1e-6)

    world.detach_carried_object()
    assert np.allclose(world._cluster_hull_w3.extents, empty_extent, atol=1e-9), (
        "the envelope still carries the detached payload"
    )
    world.attach_object(pieces, T)  # leave the fixture as found


@needs_cache
def test_detach_restores_the_verdicts_of_a_freshly_built_empty_world(world):
    """The strongest form: after a payload round trip, answers match a world built without one."""
    reference = CollisionWorld(cache_dir=config.CACHE_DIR, self_check=True, object_attached=False)
    pieces = world.carried_object_pieces()
    T = np.array(world.manifest["object"]["T_wrist3_obj"])
    world.detach_carried_object()
    world.attach_object(pieces, T)
    world.detach_carried_object()

    rng = np.random.default_rng(3)
    qs = [rng.uniform(-1.5, 1.5, 6) for _ in range(400)]
    mismatched = [q for q in qs
                  if bool(world.in_collision(q)) != bool(reference.in_collision(q))]
    assert not mismatched, f"{len(mismatched)}/{len(qs)} verdicts differ from a fresh empty world"
