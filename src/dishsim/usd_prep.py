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
original is never modified. The derivation (the ``_rl`` passive-door variant lives in git
history with the retired inspection script):

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


def _ensure_material(stage, path: str, color, metallic: float, roughness: float):
    """Self-contained UsdPreviewSurface for rack_gen groups (dark fold-down insert row,
    light-gray polypropylene cutlery basket). A dedicated material is used because a bound
    material overrides displayColor under RTX and the rack's own ``mat_1029771`` is a
    referenced PBR override we should not mutate."""
    from pxr import Sdf, UsdShade  # noqa: PLC0415

    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        return UsdShade.Material(prim)
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tuple(color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
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

    _author_box(stage, "/root/E_body_5/Countertop", center_m - ext_m, center_m + ext_m,
                color=(0.85, 0.85, 0.86))
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

    group_prims = {"frame": "RackGen", "insert": "RackGenInsert", "basket": "RackGenBasket"}
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

        # the builder dispatcher routes on the params dict (the Bosch third rack declares
        # {"builder": "tray"}); v1 dicts carry no builder key and take build_rack
        parts = rack_gen.build(params)
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
            # exists inside Kit, while this also runs bare (scripts/setup/prepare_dishwasher_usd.py).
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("sdf")
            prim.AddAppliedSchema("PhysxCollisionAPI")
            prim.AddAppliedSchema("PhysxSDFMeshCollisionAPI")
            prim.CreateAttribute("physxSDFMeshCollision:resolution", Sdf.ValueTypeNames.Int).Set(
                int(params["sdf_resolution"])
            )
            if group == "insert":
                # Bosch/Whirlpool signature: the clip-in tine row is a darker gray than the rack
                mesh.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.28, 0.30)])  # fallback if unbound
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(_ensure_material(
                    stage, "/root/materials/mat_rackgen_insert", (0.28, 0.28, 0.30), 0.35, 0.35))
            elif group == "basket":
                mesh.CreateDisplayColorAttr([Gf.Vec3f(0.62, 0.64, 0.66)])  # fallback if unbound
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(_ensure_material(
                    stage, "/root/materials/mat_rackgen_basket", (0.62, 0.64, 0.66), 0.0, 0.55))
            else:
                mat_prim = stage.GetPrimAtPath("/root/materials/mat_1029771")  # original rack material
                if mat_prim.IsValid():
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(mat_prim))
                else:
                    # self-authored stages (Bosch) have no ArtVIP material — without a color
                    # the wires render unset-white and vanish against the white shell
                    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.48, 0.50, 0.53)])
            print(f"[INFO] rack_gen: {body}/{prim_name} <- {len(gparts)} parts, {len(points)} verts")
        # explicit mass: deterministic vs PhysX auto-mass on a thin wire shell; the prismatic
        # rack axis is horizontal, so weight loads the joint constraint, not the drive
        UsdPhysics.MassAPI.Apply(xf_prim).CreateMassAttr(float(params["mass_kg"]))


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

    1. World-weld fixed joints removed (the authored world-space weld pins the asset origin).
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


# ---------------------------------------------------------------------------------------------
# Bosch 800 (self-authored machine — no ArtVIP sublayers)
# ---------------------------------------------------------------------------------------------


def _bosch_stamp(door_open_deg: float, door_band_deg: float, rack_targets: dict | None) -> str:
    """Derivation stamp for the self-authored Bosch USD: the parametric machine geometry
    PLUS everything the v0 stamp covers, so any change regenerates the file."""
    from . import config, rack_gen  # noqa: PLC0415

    payload = {
        "machine_gen": config.MACHINE_GEN["bosch800"],
        "rack_gen": config.RACK_GEN,
        "travel": config.RACK_TRAVEL_LIMITS_BY_JOINT_M,
        "door": (round(float(door_open_deg), 4), round(float(door_band_deg), 4)),
        "rack_targets": {k: round(float(v), 6) for k, v in sorted((rack_targets or {}).items())},
        "countertop": (config.COUNTERTOP_SIZE, config.COUNTERTOP_CENTER_W),
    }
    return rack_gen.params_hash(payload, config.RACK_GEN_VERSION)


def _author_box(stage, path: str, lo, hi, color=(0.75, 0.76, 0.78)):
    """Author an axis-aligned convex box mesh with collision between corners lo/hi [m]."""
    import numpy as np  # noqa: PLC0415
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt  # noqa: PLC0415

    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    signs = np.array(
        [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
        dtype=float,
    )
    pts = (lo[None, :] + signs * (hi - lo)[None, :]).astype(np.float32)
    faces = np.array(
        [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7)],
        dtype=np.int32,
    )
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
    mesh.CreateSubdivisionSchemeAttr("none")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([tuple(color)])
    return mesh


def _author_disc(stage, path: str, center_xy, z0: float, z1: float, radius: float,
                 color=(0.55, 0.56, 0.58), sides: int = 12):
    """Author a convex 12-gon prism (spray-arm swept volume) with collision."""
    import numpy as np  # noqa: PLC0415
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt  # noqa: PLC0415

    ang = np.linspace(0.0, 2.0 * np.pi, sides, endpoint=False)
    ring = np.stack([center_xy[0] + radius * np.cos(ang), center_xy[1] + radius * np.sin(ang)], axis=1)
    pts = np.concatenate(
        [np.column_stack([ring, np.full(sides, z0)]), np.column_stack([ring, np.full(sides, z1)])]
    ).astype(np.float32)
    faces = []
    for i in range(sides):  # side quads as triangles
        j = (i + 1) % sides
        faces += [(i, j, sides + j), (i, sides + j, sides + i)]
    for i in range(1, sides - 1):  # caps
        faces += [(0, i + 1, i), (sides, sides + i, sides + i + 1)]
    faces = np.array(faces, dtype=np.int32)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
    mesh.CreateSubdivisionSchemeAttr("none")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([tuple(color)])
    return mesh


def make_bosch800_usd(
    door_open_deg: float = 90.0,
    door_band_deg: float = 5.0,
    rack_targets: dict[str, float] | None = None,
    force: bool = False,
    suffix: str = "",
) -> str:
    """Author the Bosch 800 v0 machine USD from ``config.MACHINE_GEN["bosch800"]``.

    Fully self-authored (no ArtVIP sublayers): tub shell as explicit convex slabs, door
    panel with dispenser bump on a revolute hinge, three procedural racks on prismatic
    joints, spray-arm swept volumes, and the niche/counter geometry. Reuses the six
    canonical body/joint names so downstream references work unchanged, and adds the
    third-rack pair (``E_shelf_third`` / ``PrismaticJoint_dishwasher_2_third``).

    v0 semantics match :func:`make_dishwasher_v0_usd`: door limits clamped to
    ``[door_open_deg - door_band_deg, door_open_deg]`` with a neutralized force-type
    drive (the actuator config supplies the hold), rack drives force-type with targets
    from ``rack_targets``, per-joint travel limits from
    ``config.RACK_TRAVEL_LIMITS_BY_JOINT_M``. A hash of the full derivation config is
    stamped into ``customLayerData`` and an existing file is reused only when it matches.

    Machine frame: X width (centered), Y depth (front at -d/2, door opens toward -y),
    Z up from the floor [m]. Must run under ``config.apply_machine("bosch800")``.

    Args:
        door_open_deg: Door-open angle the limits clamp around [deg].
        door_band_deg: Width of the allowed band below the open angle [deg].
        rack_targets: Prismatic drive targets by joint name [m].
        force: Regenerate even if the derived file exists and the stamp matches.
        suffix: Extra filename token (e.g. ``_both_in``) for per-scenario copies.

    Returns:
        Path to ``assets/machines/bosch800/model_bosch800_v0<suffix>.usda``.
    """
    from . import config  # noqa: PLC0415

    assert config.MACHINE == "bosch800", "apply_machine('bosch800') must run before authoring"
    out_dir = os.path.join(config.MACHINE_USD_DIR, "bosch800")
    os.makedirs(out_dir, exist_ok=True)
    dst_path = os.path.join(out_dir, f"model_bosch800_v0{suffix}.usda")
    stamp = _bosch_stamp(door_open_deg, door_band_deg, rack_targets)
    if os.path.isfile(dst_path) and not force and _stamped_hash(dst_path) == stamp:
        return dst_path

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    if os.path.isfile(dst_path):
        os.remove(dst_path)  # stale stamp — CreateNew requires a fresh path

    g = config.MACHINE_GEN["bosch800"]
    body, door, tub, spray, racks = g["body"], g["door"], g["tub"], g["spray"], g["racks"]
    bw2, bd2 = body["w"] / 2.0, body["d"] / 2.0  # body half extents
    tw2 = tub["w"] / 2.0
    y_front = -bd2 + door["t"]  # tub front plane = door inner face when closed
    y_rear = y_front + tub["d"]
    floor_z, ceil_z = tub["floor_z"], tub["ceiling_z"]
    y_mid = (y_front + y_rear) / 2.0

    stage = Usd.Stage.CreateNew(dst_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())

    def _rigid_body(path: str, translate=(0.0, 0.0, 0.0), mass: float | None = None):
        xf = UsdGeom.Xform.Define(stage, path)
        # proper Xformable op (well-formed uniform xformOpOrder) — hand-rolled attributes
        # rendered fine but broke the physics->USD transform write-back, so joint-driven
        # links drew at their authored pose while PhysX moved them (measured on the first
        # bring-up: door open in physics, closed on camera)
        UsdGeom.Xformable(xf).AddTransformOp().Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(*translate)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        # match the ArtVIP applied-schema set — omni.physx tracks these for write-back
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        if mass is not None:
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        return xf

    # ---- E_body_5: shell as explicit convex slabs -------------------------------------------
    # articulation root on the BASE RIGID BODY, matching the ArtVIP layout — Isaac Lab's
    # fix_root_link welds to the articulation prim and requires it to be a rigid body
    shell = _rigid_body("/root/E_body_5", mass=40.0)
    UsdPhysics.ArticulationRootAPI.Apply(shell.GetPrim())
    shell.GetPrim().AddAppliedSchema("PhysxArticulationAPI")
    slabs = {
        # base compartment (toe-kick recess at the front): sump/pump volume
        "BaseBox": ((-bw2, -bd2 + g["toe_kick"]["setback"], 0.0), (bw2, bd2, floor_z - 0.015)),
        "TubFloor": ((-tw2, y_front, floor_z - 0.015), (tw2, y_rear, floor_z)),
        "WallLeft": ((-bw2, y_front, floor_z - 0.015), (-tw2, bd2, ceil_z + 0.015)),
        "WallRight": ((tw2, y_front, floor_z - 0.015), (bw2, bd2, ceil_z + 0.015)),
        "WallRear": ((-tw2, y_rear, floor_z - 0.015), (tw2, bd2, ceil_z + 0.015)),
        "TubRoof": ((-tw2, y_front, ceil_z), (tw2, y_rear, ceil_z + 0.015)),
        # controls/CrystalDry structure between tub roof and body top
        "TopBand": ((-bw2, y_front, ceil_z + 0.015), (bw2, bd2, body["h"])),
        # niche counter over the machine (13 mm install gap above the body top)
        "NicheTop": ((-bw2, -bd2, body["h"] + 0.013), (bw2, bd2, body["h"] + 0.051)),
    }
    # interior surfaces darker than the shell: all-white-on-white renders read as a solid
    # wall (measured on the first bring-up stills — the mouth was illegible)
    interior = {"TubFloor", "WallLeft", "WallRight", "WallRear", "TubRoof"}
    for name, (lo, hi) in slabs.items():
        color = (0.42, 0.44, 0.47) if name in interior else (0.75, 0.76, 0.78)
        _author_box(stage, f"/root/E_body_5/{name}", lo, hi, color=color)
    # spray-arm swept volumes (estimated dims — docs/bosch800_source_data.md §5)
    _author_disc(stage, "/root/E_body_5/LowerSprayArm", (0.0, y_mid), floor_z,
                 floor_z + spray["arm_t"], spray["lower_span"] / 2.0)
    _author_disc(stage, "/root/E_body_5/TopSprayHead", (0.0, y_mid), ceil_z - 0.015, ceil_z,
                 spray["top_dia"] / 2.0)

    # ---- E_door_4: panel + dispenser bump, authored CLOSED ----------------------------------
    door_xf = _rigid_body("/root/E_door_4", mass=5.0)
    dw2 = door["w"] / 2.0
    _author_box(stage, "/root/E_door_4/Panel",
                (-dw2, -bd2, door["bottom_z"]), (dw2, -bd2 + door["t"], body["h"]),
                color=(0.52, 0.54, 0.57))
    disp = door["dispenser"]
    dz0 = door["bottom_z"] + disp["center_from_bottom"] - disp["h"] / 2.0
    _author_box(stage, "/root/E_door_4/Dispenser",
                (-disp["w"] / 2.0, -bd2 + door["t"], dz0),
                (disp["w"] / 2.0, -bd2 + door["t"] + disp["bump"], dz0 + disp["h"]),
                color=(0.30, 0.30, 0.32))

    # ---- racks: Xforms at the STOWED pose; meshes authored by _author_rack_meshes -----------
    rack_xforms = {
        "E_shelf_1_04": ((-config.RACK_GEN["E_shelf_1_04"]["footprint"][0] / 2.0,
                          y_front - 0.005, racks["lower"]["rail_z"]),
                         config.RACK_GEN["E_shelf_1_04"]["mass_kg"]),
        "E_shelf_03": ((-config.RACK_GEN["E_shelf_03"]["footprint"][0] / 2.0,
                        y_front - 0.012, racks["middle"]["rail_z"]),
                       config.RACK_GEN["E_shelf_03"]["mass_kg"]),
        "E_shelf_third": ((-config.RACK_GEN["E_shelf_third"]["footprint"][0] / 2.0,
                           y_front - 0.002, racks["third"]["rail_z"]),
                          config.RACK_GEN["E_shelf_third"]["mass_kg"]),
    }
    for name, (translate, _mass) in rack_xforms.items():
        _rigid_body(f"/root/{name}", translate=translate)  # MassAPI authored by _author_rack_meshes
    # mid spray arm rides UNDER the middle rack (rack frame: footprint center, below z=0)
    mid_fp = config.RACK_GEN["E_shelf_03"]["footprint"]
    _author_disc(stage, "/root/E_shelf_03/MidSprayArm",
                 (mid_fp[0] / 2.0, mid_fp[1] / 2.0), -0.045, -0.045 + spray["arm_t"],
                 spray["mid_span"] / 2.0)

    # ---- joints ------------------------------------------------------------------------------
    hinge_y = -bd2 + door["hinge_setback"]
    hinge_z = door["hinge_z"]
    rev = UsdPhysics.RevoluteJoint.Define(stage, "/root/RevoluteJoint_dishwasher_2_middle")
    rev.GetBody0Rel().SetTargets(["/root/E_body_5"])
    rev.GetBody1Rel().SetTargets(["/root/E_door_4"])
    rev.CreateAxisAttr("X")
    rev.CreateLocalPos0Attr(Gf.Vec3f(0.0, hinge_y, hinge_z))
    rev.CreateLocalPos1Attr(Gf.Vec3f(0.0, hinge_y, hinge_z))
    # right-hand +angle about +X takes +z to -y: the door top tips outward over the front,
    # so the v1 sign convention (0 = closed, +90 = open) holds with an identity joint frame
    rev.CreateLowerLimitAttr(door_open_deg - door_band_deg)
    rev.CreateUpperLimitAttr(door_open_deg)
    drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(door_open_deg)

    rack_joint_specs = {
        "PrismaticJoint_dishwasher_2_down": "E_shelf_1_04",
        "PrismaticJoint_dishwasher_2_up": "E_shelf_03",
        "PrismaticJoint_dishwasher_2_third": "E_shelf_third",
    }
    for joint_name, rack_body in rack_joint_specs.items():
        pj = UsdPhysics.PrismaticJoint.Define(stage, f"/root/{joint_name}")
        pj.GetBody0Rel().SetTargets(["/root/E_body_5"])
        pj.GetBody1Rel().SetTargets([f"/root/{rack_body}"])
        pj.CreateAxisAttr("Y")  # negative extension slides toward -y, out over the door
        t = rack_xforms[rack_body][0]
        pj.CreateLocalPos0Attr(Gf.Vec3f(*t))
        pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        lo, hi = config.RACK_TRAVEL_LIMITS_BY_JOINT_M[joint_name]
        pj.CreateLowerLimitAttr(float(lo))
        pj.CreateUpperLimitAttr(float(hi))
        d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)
        target = (rack_targets or {}).get(joint_name, 0.0)
        d.CreateTargetPositionAttr(float(target))

    # rack meshes (rack_gen; the tray builder routes via rack_gen.build) + the side counter
    _author_rack_meshes(stage)
    _author_countertop(stage)

    layer = stage.GetRootLayer()
    layer.customLayerData = {**(layer.customLayerData or {}), _RACK_STAMP_KEY: stamp}
    layer.Save()
    print(
        f"[INFO] Bosch 800 v0 machine written to {dst_path} (door clamped to "
        f"[{door_open_deg - door_band_deg}, {door_open_deg}] deg; stamp {stamp})"
    )
    return dst_path
