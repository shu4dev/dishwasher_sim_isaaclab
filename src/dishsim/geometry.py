# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""USD -> mesh extraction and the collision-world cache format.

Kit-side code (called from ``scripts/setup/extract_geometry.py``) dumps per-rigid-body trimeshes
plus a ``scene_state.json`` manifest into ``assets/cache/``; the Kit-free
:mod:`dishsim.collision_world` loads only from that cache. ``pxr``/``isaaclab`` imports stay
inside functions so the module itself remains importable everywhere (the cache *reading*
helpers are Kit-free).

Cache layout::

    assets/cache/
      meshes/<name>.obj          # body-frame meshes (statics + robot links + object)
      scene_state.json           # manifest: frames, poses, transforms, config hash
      coacd/<name>_<hash>/piece_*.obj   # written by scripts/setup/decompose_meshes.py
"""

import hashlib
import json
import os

import numpy as np

from . import config

MESH_DIR = "meshes"
MANIFEST = "scene_state.json"

#: dishwasher rigid bodies to extract (body-frame meshes + world poses); machines with a
#: third rack extend this via :func:`dishwasher_bodies`
DISHWASHER_BODIES = ["E_body_5", "E_door_4", "E_shelf_03", "E_shelf_1_04"]


def dishwasher_bodies() -> list[str]:
    """The active machine's extractable rigid bodies (adds the third rack when present)."""
    return DISHWASHER_BODIES + (["E_shelf_third"] if config.HAS_THIRD_RACK else [])
#: arm links whose meshes move with fk_all_links
ARM_LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]


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
# Kit-side extraction
# ---------------------------------------------------------------------------------------------


def _mesh_prims_under(prim):
    from isaaclab.utils.mesh import PRIMITIVE_MESH_TYPES  # noqa: PLC0415
    import isaaclab.sim as sim_utils  # noqa: PLC0415

    return sim_utils.get_all_matching_child_prims(
        prim.GetPath(), lambda p: p.GetTypeName() in PRIMITIVE_MESH_TYPES + ["Mesh"]
    )


def extract_prim_mesh(body_prim, stage) -> "object":
    """Concatenate all meshes under a rigid-body prim into one trimesh in the BODY frame.

    Returns:
        A ``trimesh.Trimesh`` (body-frame vertices [m]) or None if no meshes found.
    """
    import trimesh  # noqa: PLC0415
    import isaaclab.sim as sim_utils  # noqa: PLC0415
    from isaaclab.utils.mesh import create_trimesh_from_geom_mesh, create_trimesh_from_geom_shape  # noqa: PLC0415

    from .transforms import make_T  # noqa: PLC0415

    parts = []
    for mesh_prim in _mesh_prims_under(body_prim):
        mesh = (
            create_trimesh_from_geom_mesh(mesh_prim)
            if mesh_prim.GetTypeName() == "Mesh"
            else create_trimesh_from_geom_shape(mesh_prim)
        )
        if mesh is None or len(mesh.vertices) == 0:
            continue
        mesh.apply_scale(sim_utils.resolve_prim_scale(mesh_prim))
        rel_pos, rel_quat = sim_utils.resolve_prim_pose(mesh_prim, body_prim)
        mesh.apply_transform(make_T(rel_pos, rel_quat))
        parts.append(mesh)
    if not parts:
        return None
    return trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]


def dump_cache(scene, sim, cache_dir: str = config.CACHE_DIR) -> str:
    """Dump body-frame meshes + the scene_state.json manifest from the LIVE settled scene.

    Must run with ``use_fabric=False`` (stale-transform trap) after the scene has settled at
    the v0 static state. Asserts live link poses against the analytic FK before writing.

    Returns:
        Path to the manifest.
    """
    import trimesh  # noqa: PLC0415

    from . import scene as dscene  # noqa: PLC0415
    from .transforms import T_inv, make_T  # noqa: PLC0415
    from .ur5e_kin import fk_all_links  # noqa: PLC0415

    stage = scene.stage
    mesh_dir = os.path.join(cache_dir, MESH_DIR)
    os.makedirs(mesh_dir, exist_ok=True)

    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)
    manifest: dict = {
        "frame_convention": config.FRAME_CONVENTION,
        "config_hash": config_hash(),
        "object_name": config.OBJECT_NAME,
        "home_q": list(config.HOME_Q),
        "statics": {},
        "arm_links": {},
        "gripper_links": {},
    }

    def body_pose_w(articulation, body_name):
        ids, _ = articulation.find_bodies(body_name)
        pos = articulation.data.body_link_pos_w.torch[0, ids[0]].cpu().numpy()
        quat = articulation.data.body_link_quat_w.torch[0, ids[0]].cpu().numpy()
        return make_T(pos, quat)

    def save_mesh(name, mesh):
        path = os.path.join(mesh_dir, f"{name}.obj")
        mesh.export(path)
        return f"{MESH_DIR}/{name}.obj"

    # -- dishwasher bodies (static in v0) ------------------------------------------------------
    dw = scene["dishwasher"]
    env_path = "/World/envs/env_0"
    for body in dishwasher_bodies():
        prim = None
        for cand in stage.Traverse():
            if cand.GetName() == body and str(cand.GetPath()).startswith(f"{env_path}/Dishwasher"):
                prim = cand
                break
        assert prim is not None, f"dishwasher body prim {body} not found"
        mesh = extract_prim_mesh(prim, stage)
        assert mesh is not None, f"no meshes under {body}"
        if body in config.RACK_GEN:
            # earliest Kit-stage guard against a scale/pre-compensation bug: the extracted
            # (USD-round-tripped) rack must match the generator's design-space geometry
            from . import rack_gen  # noqa: PLC0415

            ref = rack_gen.merged_mesh(rack_gen.build(config.RACK_GEN[body]))
            dev = np.abs(np.asarray(mesh.bounds) - np.asarray(ref.bounds)).max()
            assert dev < 2e-3, f"{body}: extracted bounds deviate {dev * 1e3:.2f} mm from rack_gen"
        T_base_body = T_base_w @ body_pose_w(dw, body)
        manifest["statics"][body] = {
            "mesh": save_mesh(body, mesh),
            "T_base_body": T_base_body.tolist(),
            "coacd": True,
        }

    # -- pedestal + ground (procedural boxes; base frame poses from config) -------------------
    ped = trimesh.creation.box(extents=config.PEDESTAL_SIZE)
    T_base_ped = T_base_w @ make_T(config.PEDESTAL_POS_W, (0, 0, 0, 1))
    manifest["statics"]["pedestal"] = {
        "mesh": save_mesh("pedestal", ped),
        "T_base_body": T_base_ped.tolist(),
        "coacd": False,
    }
    ground = trimesh.creation.box(extents=(4.0, 4.0, 0.02))
    T_base_ground = T_base_w @ make_T((0.4, 0.0, -0.01), (0, 0, 0, 1))
    manifest["statics"]["ground"] = {
        "mesh": save_mesh("ground", ground),
        "T_base_body": T_base_ground.tolist(),
        "coacd": False,
    }

    # -- robot links ---------------------------------------------------------------------------
    robot = scene["robot"]
    fk = fk_all_links(np.array(config.HOME_Q))
    link_prim_paths = {
        name: f"{env_path}/Robot/{name}" for name in ARM_LINKS
    }
    fk_errs = {}
    for name in ARM_LINKS:
        prim = stage.GetPrimAtPath(link_prim_paths[name])
        assert prim.IsValid(), f"arm link prim missing: {link_prim_paths[name]}"
        mesh = extract_prim_mesh(prim, stage)
        if mesh is None:
            continue  # some links may carry no geometry
        T_base_link_live = T_base_w @ body_pose_w(robot, name)
        T_base_link_fk = fk[name]
        err = float(np.linalg.norm(T_base_link_live[:3, 3] - T_base_link_fk[:3, 3]))
        fk_errs[name] = err
        manifest["arm_links"][name] = {"mesh": save_mesh(f"link_{name}", mesh)}
    assert max(fk_errs.values()) < 2e-3, f"live-vs-FK link pose mismatch: {fk_errs}"
    manifest["fk_link_errs_m"] = fk_errs

    # -- gripper links: rigid relative to wrist_3 at the frozen aperture ----------------------
    T_w_wrist3 = body_pose_w(robot, "wrist_3_link")
    gripper_names = [n for n in robot.body_names if n not in ARM_LINKS]
    gripper_prim_base = f"{env_path}/Robot/Gripper/Robotiq_2F_85"
    for name in gripper_names:
        prim_name = "base_link" if name == "base_link_0" else name
        prim = stage.GetPrimAtPath(f"{gripper_prim_base}/{prim_name}")
        if not prim.IsValid():
            continue
        mesh = extract_prim_mesh(prim, stage)
        if mesh is None:
            continue
        T_wrist3_link = T_inv(T_w_wrist3) @ body_pose_w(robot, name)
        manifest["gripper_links"][name] = {
            "mesh": save_mesh(f"gripper_{name}", mesh),
            "T_wrist3_link": T_wrist3_link.tolist(),
        }

    # -- carried object ------------------------------------------------------------------------
    obj = scene["carried_object"]
    obj_prim = stage.GetPrimAtPath(f"{env_path}/CarriedObject")
    mesh = extract_prim_mesh(obj_prim, stage)
    assert mesh is not None, "no meshes under CarriedObject"
    T_wrist3_obj_live = T_inv(T_w_wrist3) @ make_T(
        obj.data.root_pos_w.torch[0].cpu().numpy(), obj.data.root_quat_w.torch[0].cpu().numpy()
    )
    T_wrist3_obj_analytic = dscene.t_wrist3_obj()
    weld_dev = float(np.linalg.norm(T_wrist3_obj_live[:3, 3] - T_wrist3_obj_analytic[:3, 3]))
    assert weld_dev < 2e-3, f"live weld transform deviates {weld_dev*1e3:.2f} mm from analytic"
    manifest["object"] = {
        "mesh": save_mesh("object", mesh),
        "T_wrist3_obj": T_wrist3_obj_analytic.tolist(),
        "coacd": True,
    }
    manifest["t_wrist3_tcp"] = dscene.t_wrist3_tcp().tolist()

    # -- static machine state + scenario (recorded for review; not part of config_hash) -------
    from . import scene as _s  # noqa: PLC0415

    manifest["statics_state"] = _s.statics_report(scene)
    manifest["scenario"] = config.SCENARIO_NAME

    path = os.path.join(cache_dir, MANIFEST)
    with open(path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[INFO] cache manifest written to {path} (hash {manifest['config_hash']})")
    return path


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
            "scripts/setup/extract_geometry.py and scripts/setup/decompose_meshes.py"
        )
    return manifest


def coacd_dir_for(name: str, mesh_path: str, cache_dir: str = config.CACHE_DIR) -> str:
    """Deterministic decomposition directory for a body: keyed by mesh bytes + params.

    ``cache_dir`` is the mesh-read root only; the output always lives under the baseline
    ``assets/cache/coacd/``. The digest is content-addressed, so scenario caches whose meshes
    are byte-identical share one decomposition instead of re-running per scenario. Rack bodies
    key on their generator config (their pieces are rack_gen's exact convex parts, not CoACD).
    """
    if name in config.RACK_GEN:
        params = {"rack_gen": config.RACK_GEN[name], "version": config.RACK_GEN_VERSION}
    else:
        params = config.COACD.get(name, config.COACD["default"])
    with open(os.path.join(cache_dir, mesh_path), "rb") as f:
        digest = hashlib.sha256(f.read() + json.dumps(params, sort_keys=True).encode()).hexdigest()[:16]
    return os.path.join(config.CACHE_DIR, "coacd", f"{name}_{digest}")
