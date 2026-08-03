# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive prepared copies of the ArtVIP dishwasher USD.

The ArtVIP assets ship two properties that break relocated spawns:

1. A ``PhysicsFixedJoint`` with ``body1`` but **no** ``body0`` — a world-space weld that pins the
   machine at its originally authored pose. Spawning the articulation anywhere else makes this
   weld fight the articulation's fixed base and the sim blows up within a few seconds. The
   derived copy removes it; ``fix_root_link=True`` in the :class:`~isaaclab.assets.ArticulationCfg`
   keeps the base fixed at the spawn pose.
2. An active door drive (stiffness 20 per-degree ≈ 1146 N·m/rad toward 90 deg, acceleration
   type). Actuator configs override the gains after initialization, but the authored drive still
   acts during the physics-reset step and kicks the door. The passive-door copy zeroes the
   revolute drive gains/targets and switches the drives to force type so configured gains mean
   N·m/rad.

Derived files are written next to the source (so relative asset references keep resolving); the
original is never modified. Two derivations exist:

- :func:`make_dishwasher_rl_usd` (``_rl`` suffix): passive door — used by the archived RL task
  and by the inspection script's stability/door tests. Keeps the original ArtVIP basket meshes.
- :func:`make_dishwasher_v0_usd` (``_v0`` suffix): the v0 planning scene — door joint limits
  clamped to a narrow band at the open position and rack drive targets set to the configured
  extensions, so the machine is a *static* obstacle. The lock itself is belt-and-braces: the
  clamped USD limits make the open pose inescapable even if drive gains misbehave, and the
  actuator config (``DISHWASHER_V0_CFG``) holds the joints with stiff position drives on top.
  Additionally REPLACES both flat ArtVIP basket meshes with the procedural realistic racks from
  :mod:`dishsim.rack_gen` (see ``config.RACK_GEN``), stamping a hash of the generator config,
  the door clamp, and the rack targets into the layer's ``customLayerData`` so changing any of
  them regenerates the derived file automatically.
"""

import os
import re

#: customLayerData key carrying the derivation-config hash in derived v0 files
_RACK_STAMP_KEY = "dishsim_v0_stamp"


def _derived_stamp(door_open_deg: float, door_band_deg: float, rack_targets: dict | None) -> str:
    """Hash of everything a derived v0 file bakes in: the full rack-generator config PLUS the
    door clamp, the rack drive targets, and the countertop — so editing any of them
    regenerates the file instead of silently reusing stale geometry/joints."""
    from . import config, rack_gen  # noqa: PLC0415

    payload = {
        "rack_gen": config.RACK_GEN,
        "door": (round(float(door_open_deg), 4), round(float(door_band_deg), 4)),
        "rack_targets": {k: round(float(v), 6) for k, v in sorted((rack_targets or {}).items())},
        "countertop": (config.COUNTERTOP_SIZE, config.COUNTERTOP_CENTER_W),
    }
    return rack_gen.params_hash(payload, config.RACK_GEN_VERSION)


def _rl_stamp() -> str:
    """Stamp for the RL/inspection derived copy (rack meshes + countertop, no v0 joints)."""
    from . import config, rack_gen  # noqa: PLC0415

    payload = {
        "rack_gen": config.RACK_GEN,
        "countertop": (config.COUNTERTOP_SIZE, config.COUNTERTOP_CENTER_W),
    }
    return rack_gen.params_hash(payload, config.RACK_GEN_VERSION)


def _stamped_hash(usda_path: str) -> str | None:
    """Read the rack-gen stamp from a derived ``.usda`` header by TEXT, deliberately not via
    ``Sdf.Layer`` — an Sdf handle opened here would stay registered under the file's identity,
    go stale when the file is re-exported below, and could then serve stale content to the
    subsequent scene-spawn open of the same path."""
    try:
        with open(usda_path, encoding="utf-8", errors="ignore") as f:
            head = f.read(4096)
    except OSError:
        return None
    m = re.search(rf'{_RACK_STAMP_KEY}\s*=\s*"([0-9a-f]{{16}})"', head)
    return m.group(1) if m else None


def _ensure_insert_material(stage):
    """Self-contained dark material for the fold-down insert row (Bosch/Whirlpool signature:
    the clip-in tine row is a darker gray than the rack). A dedicated UsdPreviewSurface is
    used because a bound material overrides displayColor under RTX and the rack's own
    ``mat_1029771`` is a referenced PBR override we should not mutate."""
    from pxr import Sdf, UsdShade  # noqa: PLC0415

    path = "/root/materials/mat_rackgen_insert"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        return UsdShade.Material(prim)
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.28, 0.28, 0.30))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.35)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _author_countertop(stage) -> None:
    """Author the freestanding-machine worktop slab as a child Mesh of the shell body.

    The ArtVIP tub is open-topped in collision; a real freestanding dishwasher has a worktop —
    and the manipulation trials rest the mug on it. Attached to /root/E_body_5 so it extracts
    into the shell's cached mesh (CoACD absorbs a box trivially) with no manifest changes.
    The slab is authored in the E_body_5 frame, converted from the configured WORLD box via
    the machine spawn pose (yaw -90 about z at DISHWASHER_POS_W; E_body_5 frame == asset root).
    """
    import numpy as np  # noqa: PLC0415
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt  # noqa: PLC0415

    from . import config  # noqa: PLC0415

    body = stage.GetPrimAtPath("/root/E_body_5")
    assert body.IsValid(), "shell prim /root/E_body_5 not found"
    old = stage.GetPrimAtPath("/root/E_body_5/Countertop")
    if old.IsValid():
        stage.RemovePrim(old.GetPath())

    # world -> machine frame: p_m = R(+90z) @ (p_w - t)  (spawn is R(-90z) at t)
    t = np.array(config.DISHWASHER_POS_W)
    c_w = np.array(config.COUNTERTOP_CENTER_W) - t
    center_m = np.array([-c_w[1], c_w[0], c_w[2]])
    sx, sy, sz = config.COUNTERTOP_SIZE
    ext_m = np.array([sy, sx, sz]) / 2.0  # world x/y swap under the 90-degree yaw

    signs = np.array(
        [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
        dtype=float,
    )
    pts = (center_m[None, :] + signs * ext_m[None, :]).astype(np.float32)
    faces = np.array(
        [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7)],
        dtype=np.int32,
    )
    mesh = UsdGeom.Mesh.Define(stage, "/root/E_body_5/Countertop")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
    mesh.CreateSubdivisionSchemeAttr("none")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([(0.85, 0.85, 0.86)])
    print("[INFO] countertop slab authored on /root/E_body_5")


def _author_rack_meshes(stage) -> None:
    """Replace the baked ArtVIP basket mesh under each rack body with the rack_gen geometry.

    The body Xforms (with their non-uniform x-scale that the prismatic-joint anchors depend on)
    and the joints themselves are left untouched; only the child ``Mesh`` prims are swapped.
    Authored x-coordinates are pre-divided by the Xform scale (see rack_gen.mesh_arrays_usd) so
    the world-space geometry equals the design geometry exactly. The fold-down insert row is
    authored as a SECOND mesh (``RackGenInsert``) with its own dark material; extraction
    concatenates all child meshes, so the cache/FCL side sees one body either way.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: PLC0415

    from . import config, rack_gen  # noqa: PLC0415

    group_prims = {"frame": "RackGen", "insert": "RackGenInsert"}
    for body, params in config.RACK_GEN.items():
        xf_prim = stage.GetPrimAtPath(f"/root/{body}")
        assert xf_prim.IsValid(), f"rack body prim /root/{body} not found"
        m = Gf.Matrix4d(xf_prim.GetAttribute("xformOp:transform").Get())
        sx = m.GetRow3(0).GetLength()
        assert abs(sx - params["usd_scale_x"]) < 1e-3, (
            f"{body}: authored Xform x-scale {sx:.4f} != config usd_scale_x {params['usd_scale_x']}"
        )
        for child in list(xf_prim.GetChildren()):  # drop the flat ArtVIP basket mesh(es)
            if child.IsA(UsdGeom.Mesh):
                stage.RemovePrim(child.GetPath())

        parts = rack_gen.build_rack(params)
        groups = rack_gen.parts_by_group(parts)
        for group, prim_name in group_prims.items():
            gparts = groups.get(group)
            if not gparts:
                continue
            points, counts, indices = rack_gen.mesh_arrays_usd(gparts, params["usd_scale_x"])
            mesh = UsdGeom.Mesh.Define(stage, f"/root/{body}/{prim_name}")
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(counts))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(indices))
            lo, hi = points.min(axis=0), points.max(axis=0)
            mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)

            prim = mesh.GetPrim()
            # same SDF mesh collision the ArtVIP racks used, finer voxels for the 2.2 mm wires.
            # The Physx* schemas are authored by NAME (AddAppliedSchema + raw attributes, the
            # same pattern the ArtVIP asset uses) because the PhysxSchema codegen module only
            # exists inside Kit, while this also runs bare (scripts/02).
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("sdf")
            prim.AddAppliedSchema("PhysxCollisionAPI")
            prim.AddAppliedSchema("PhysxSDFMeshCollisionAPI")
            prim.CreateAttribute("physxSDFMeshCollision:resolution", Sdf.ValueTypeNames.Int).Set(
                int(params["sdf_resolution"])
            )
            if group == "insert":
                mesh.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.28, 0.30)])  # fallback if unbound
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(_ensure_insert_material(stage))
            else:
                mat_prim = stage.GetPrimAtPath("/root/materials/mat_1029771")  # original rack material
                if mat_prim.IsValid():
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(mat_prim))
            print(f"[INFO] rack_gen: {body}/{prim_name} <- {len(gparts)} parts, {len(points)} verts")
        # explicit mass: deterministic vs PhysX auto-mass on a thin wire shell; the prismatic
        # rack axis is horizontal, so weight loads the joint constraint, not the drive
        UsdPhysics.MassAPI.Apply(xf_prim).CreateMassAttr(float(params["mass_kg"]))


def make_dishwasher_rl_usd(src_path: str, force: bool = False) -> str:
    """Create (or reuse) the passive-door derived copy of a dishwasher USD.

    Args:
        src_path: Path to the original ``model_<variant>.usd(a)`` file.
        force: Regenerate even if the derived file already exists.

    Returns:
        Path to the derived ``model_<variant>_rl.usda`` file.
    """
    root, _ = os.path.splitext(src_path)
    dst_path = f"{root}_rl.usda"
    if os.path.isfile(dst_path) and not force and _stamped_hash(dst_path) == _rl_stamp():
        return dst_path

    # deferred: importing pxr at module scope breaks Kit if this package is imported before
    # SimulationApp starts (which happens — Kit walks importable modules at boot)
    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    stage = Usd.Stage.Open(src_path)

    # 1) remove world-weld fixed joints (body1 set, body0 empty)
    removed = []
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                removed.append(str(prim.GetPath()))
    for path in removed:
        stage.RemovePrim(path)

    # 2) neutralize revolute (door) drives and make all joint drives force-type
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.CreateStiffnessAttr(0.0)
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTypeAttr("force")
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
            if drive:
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTypeAttr("force")

    # same procedural racks + countertop as the v0 copy (visual/report consistency); the
    # joints stay as-shipped — this copy is the mechanical reference for docs/joint_report.md
    _author_rack_meshes(stage)
    _author_countertop(stage)

    layer = stage.GetRootLayer()
    layer.customLayerData = {**(layer.customLayerData or {}), _RACK_STAMP_KEY: _rl_stamp()}
    layer.Export(dst_path)
    print(f"[INFO] Passive-door copy written to {dst_path} (removed world welds: {removed or 'none'})")
    return dst_path


def make_dishwasher_v0_usd(
    src_path: str,
    door_open_deg: float = 90.0,
    door_band_deg: float = 5.0,
    rack_targets: dict[str, float] | None = None,
    force: bool = False,
    suffix: str = "",
) -> str:
    """Create (or reuse) the v0 derived copy: static machine, door locked open, racks pinned.

    Transformations relative to the original:

    1. World-weld fixed joints removed (same reason as :func:`make_dishwasher_rl_usd`).
    2. Every revolute (door) joint: limits clamped to
       ``[door_open_deg - door_band_deg, door_open_deg]`` (USD revolute limits are in degrees),
       drive neutralized (stiffness 0, force type) — the actuator config supplies the hold; the
       clamped limits plus gravity (the bottom-hinged door falls into the upper limit) make the
       pose static regardless.
    3. Every prismatic (rack) joint: drive switched to force type with the target set from
       ``rack_targets`` (joint name -> extension [m]) so the authored spring no longer pulls the
       rack to the stowed position during the physics-reset step.
    4. Both flat ArtVIP basket meshes are replaced with the procedural realistic racks built by
       :mod:`dishsim.rack_gen` from ``config.RACK_GEN`` (SDF mesh collision, explicit mass).

    Args:
        src_path: Path to the original ``model_<variant>.usd(a)`` file.
        door_open_deg: Door-open angle the limits clamp around [deg].
        door_band_deg: Width of the allowed band below the open angle [deg].
        rack_targets: Prismatic drive targets by joint name [m]; joints not listed keep their
            authored target.
        force: Regenerate even if the derived file already exists.
        suffix: Extra filename token (e.g. ``_both_out``) so per-scenario derived copies don't
            collide. The filename does NOT encode the derivation values — instead a hash of the
            rack-generator config, the door clamp, and the rack targets is stamped into the
            layer's customLayerData, and an existing file is reused only when it matches.

    Returns:
        Path to the derived ``model_<variant>_v0<suffix>.usda`` file.
    """
    root, _ = os.path.splitext(src_path)
    dst_path = f"{root}_v0{suffix}.usda"
    stamp = _derived_stamp(door_open_deg, door_band_deg, rack_targets)
    if os.path.isfile(dst_path) and not force and _stamped_hash(dst_path) == stamp:
        return dst_path

    # deferred pxr import — same Kit-boot reason as above
    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    stage = Usd.Stage.Open(src_path)

    removed = []
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                removed.append(str(prim.GetPath()))
    for path in removed:
        stage.RemovePrim(path)

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.GetLowerLimitAttr().Set(door_open_deg - door_band_deg)
            joint.GetUpperLimitAttr().Set(door_open_deg)
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.CreateStiffnessAttr(0.0)
                drive.CreateTargetPositionAttr(door_open_deg)
                drive.CreateTypeAttr("force")
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
            if drive:
                target = (rack_targets or {}).get(prim.GetName())
                if target is not None:
                    drive.CreateTargetPositionAttr(float(target))
                drive.CreateTypeAttr("force")

    _author_rack_meshes(stage)
    _author_countertop(stage)

    layer = stage.GetRootLayer()
    layer.customLayerData = {**(layer.customLayerData or {}), _RACK_STAMP_KEY: stamp}
    layer.Export(dst_path)
    print(
        f"[INFO] v0 copy written to {dst_path} (door clamped to "
        f"[{door_open_deg - door_band_deg}, {door_open_deg}] deg; removed world welds: {removed or 'none'}; "
        f"derivation stamp {stamp})"
    )
    return dst_path
