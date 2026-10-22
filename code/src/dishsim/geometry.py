# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The collision-world cache format: its config hash, manifest reading and piece directories.

The cache is extracted once per (machine, rack state, object) and ships in the asset dataset; the extraction
(``dump_cache``) moved with the rest of the Bosch machine authoring to ``data/assets/bosch800/source/`` on
2026-10-21. The Kit-free :mod:`dishsim.collision_world` loads only from the cache.

Cache layout (``<machine>`` = ``bosch800`` or ``artvip_dishwasher_2``, see ``config.machine_asset_dir``)::

    data/assets/<machine>/cache/<state>/<object>/
      meshes/<name>.obj          # body-frame meshes (statics + object)
      scene_state.json           # manifest: frames, poses, transforms, config hash
    data/assets/<machine>/cache/coacd/<body>_<hash>/piece_*.obj   # the machine bodies' convex pieces
    data/assets/bosch_props/pieces/object_<hash>/piece_*.obj      # the carried object's convex pieces
"""

import hashlib
import json
import os

import numpy as np

from . import config

MESH_DIR = "meshes"
MANIFEST = "scene_state.json"


def config_hash() -> str:
    """Hash of every config value the collision world depends on (cache invalidation key)."""
    payload = {
        "robot_base": config.ROBOT_BASE_POS_W,
        "dishwasher": (config.DISHWASHER_POS_W, config.DISHWASHER_QUAT_W),
        "door": (config.DOOR_OPEN_DEG, config.DOOR_BAND_DEG, round(config.DOOR_INIT_RAD, 6)),
        "racks": config.RACK_JOINT_TARGETS,
        "home_q": config.HOME_Q,
        "aperture": config.GRIPPER_APERTURE_GRASP_RAD,
        "tcp": (config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT),
        "grasp": (config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT),
        "object": config.OBJECT_NAME,
        "pedestal": (config.PEDESTAL_SIZE, config.PEDESTAL_POS_W),
        # the racks are procedural (rack_gen) — any shape-parameter change must invalidate the
        # cache, since the manifest hash never covers mesh bytes; the countertop is authored
        # machine geometry, so it invalidates too
        "rack_gen": config.RACK_GEN,
        "rack_gen_version": config.RACK_GEN_VERSION,
        "countertop": (config.COUNTERTOP_SIZE, config.COUNTERTOP_CENTER_W),
    }
    # Non-mug objects additionally hash their registry geometry (scale/mass/bbox/coacd), so a
    # registry edit invalidates that object's caches. Conditional on purpose: the mug payload
    # predates the registry and the frozen v0 baseline hash must stay byte-stable (the mug's
    # dims are frozen measured values, covered by `object`/`grasp`/`aperture` above).
    if config.ACTIVE_OBJECT != "mug":
        spec = config.active_object_spec()
        payload["object_spec"] = (spec.scale, spec.mass_kg, spec.bbox_half, spec.coacd)
    # Non-default machines additionally hash their name + parametric geometry + per-joint
    # travels (their RACK_GEN, rack targets, spawn pose and countertop already flow through
    # the unconditional keys above). Conditional for the same reason as object_spec: the
    # baseline payload — and every validated v1 cache hash — must stay byte-stable.
    if config.MACHINE != config.MACHINE_BASELINE_NAME:
        payload["machine"] = (
            config.MACHINE,
            config.MACHINE_GEN.get(config.MACHINE),
            config.RACK_TRAVEL_LIMITS_BY_JOINT_M,
        )
    # Non-default base placements hash the placement name + base QUATERNION — the v1
    # payload only covers the base position (the known coverage gap this closes).
    if config.BASE_PLACEMENT != "front":
        payload["base_placement"] = (config.BASE_PLACEMENT, config.ROBOT_BASE_QUAT_W)
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:16]


# ---------------------------------------------------------------------------------------------
# Kit-free cache reading
# ---------------------------------------------------------------------------------------------


def load_manifest(cache_dir: str = config.CACHE_DIR) -> dict:
    """Read and validate the cache manifest (frame convention + config hash)."""
    with open(os.path.join(cache_dir, MANIFEST)) as f:
        manifest = json.load(f)
    assert manifest["frame_convention"] == config.FRAME_CONVENTION, (
        f"cache frame convention {manifest['frame_convention']} != {config.FRAME_CONVENTION}"
    )
    if manifest["config_hash"] != config_hash():
        raise RuntimeError(
            "collision-world cache is stale (config changed since extraction) — re-run "
            "data/assets/bosch800/source/extract_geometry.py and decompose_meshes.py"
        )
    return manifest


def coacd_dir_for(name: str, mesh_path: str, cache_dir: str = config.CACHE_DIR) -> str:
    """Deterministic decomposition directory for a body: keyed by mesh bytes + params.

    ``cache_dir`` is the mesh-read root only; the pieces live in the active machine's
    ``data/assets/<machine>/cache/coacd/``, except the carried object's, which belong to the props folder
    (``data/assets/bosch_props/pieces/``). The digest is content-addressed, so scenario caches whose meshes
    are byte-identical share one decomposition instead of re-running per scenario. Rack bodies
    key on their generator config (their pieces are rack_gen's exact convex parts, not CoACD).
    """
    if name in config.RACK_GEN:
        params = {"rack_gen": config.RACK_GEN[name], "version": config.RACK_GEN_VERSION}
    else:
        params = config.COACD.get(name, config.COACD["default"])
    with open(os.path.join(cache_dir, mesh_path), "rb") as f:
        digest = hashlib.sha256(f.read() + json.dumps(params, sort_keys=True).encode()).hexdigest()[:16]
    if name == "object":
        return os.path.join(config.ASSETS_DIR, "bosch_props", "pieces", f"{name}_{digest}")
    return os.path.join(config.machine_asset_dir(), "cache", "coacd", f"{name}_{digest}")
