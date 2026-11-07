# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The collision-world cache format: manifest reading and piece directories.

A cache is a folder of body-frame meshes plus a manifest, extracted once from a machine USD; the Kit-free
:mod:`dishsim.collision_world` loads only from it. Layout::

    <cache root>/<state>/<object>/
      meshes/<name>.obj          # body-frame meshes (statics + object)
      scene_state.json           # manifest: frames, poses, transforms, frame convention
    <cache root>/coacd/<body>_<digest>/piece_*.obj   # each body's convex pieces (the carried object's too)

The Bosch 800 / ArtVIP caches, their extraction scripts and their config hash left the repo on 2026-11-01
(plans/2026-11-01-merged-benchmark.md; git history, tag isaac45-final). No shipped machine carries a cache today.
"""

import hashlib
import json
import os

from . import config

MESH_DIR = "meshes"
MANIFEST = "scene_state.json"


def load_manifest(cache_dir: str) -> dict:
    """Read the cache manifest and check its frame convention."""
    with open(os.path.join(cache_dir, MANIFEST)) as f:
        manifest = json.load(f)
    assert manifest["frame_convention"] == config.FRAME_CONVENTION, (
        f"cache frame convention {manifest['frame_convention']} != {config.FRAME_CONVENTION}"
    )
    return manifest


def coacd_dir_for(name: str, mesh_path: str, cache_dir: str) -> str:
    """Deterministic decomposition directory for a body: keyed by mesh bytes + its ``config.COACD`` params.

    ``cache_dir`` (``<cache root>/<state>/<object>``) is the mesh-read root; the pieces live in
    ``<cache root>/coacd/``. The digest is content-addressed, so caches whose meshes are byte-identical share one
    decomposition.
    """
    params = config.COACD.get(name, config.COACD["default"])
    with open(os.path.join(cache_dir, mesh_path), "rb") as f:
        digest = hashlib.sha256(f.read() + json.dumps(params, sort_keys=True).encode()).hexdigest()[:16]
    root = os.path.dirname(os.path.dirname(os.path.normpath(cache_dir)))
    return os.path.join(root, "coacd", f"{name}_{digest}")
