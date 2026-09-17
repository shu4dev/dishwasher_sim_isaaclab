# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Standalone FDPC4221AS USD authoring and Isaac Lab loading.

No Kit or pxr imports occur at module import. Geometry is independent of benchmark
configuration and caches. Measurements absent from the references are estimates.
"""
from pathlib import Path
import hashlib
import json
import math
import re
import tempfile

import numpy as np

from .geometry import PARAMETERS

from .paths import ASSET_DIR
ROOT = "/FrigidaireFDPC4221AS"
COMPONENT_FILES = {"Cabinet": "cabinet.usdc", "Door": "door.usdc",
                   "LowerRack": "lower_rack.usdc", "UpperRack": "upper_rack.usdc",
                   "SilverwareBasket": "silverware_basket.usdc"}
BODY_POSITIONS = {name: tuple(position) for name, position in PARAMETERS["origins"].items()}
JOINT_LIMITS = {"door_hinge": (0., 90.), "lower_slide": (-.49, 0.), "upper_slide": (-.44, 0.)}
MATERIALS = {
    "CoatedWire": ((.61, .65, .68), 0.03, .31),
    "BasketPlastic": ((.045, .050, .055), 0., .48),
    "WheelPlastic": ((.047, .055, .064), 0., .38),
    "TubPlastic": ((.55, .58, .59), 0., .40),
    "Stainless": ((.57, .60, .63), .87, .30),
    "BlackPlastic": ((.025, .031, .037), 0., .38),
    "Rubber": ((.012, .015, .018), 0., .68),
    "PlateCeramic": ((.56, .78, .88), 0., .22),
    "BowlCeramic": ((.87, .44, .19), 0., .25),
    "CupCeramic": ((.20, .65, .47), 0., .22),
    "UtensilSteel": ((.73, .71, .56), .72, .25),
}


def _attr(prim, name, kind, value):
    return prim.CreateAttribute(name, kind, custom=False).Set(value)


def _identifier(name):
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


def _pose(prim, position, quaternion=None, scale=None):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    if quaternion is not None:
        xf.AddOrientOp().Set(Gf.Quatf(float(quaternion[0]), Gf.Vec3f(*map(float, quaternion[1:]))))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))


def _materials(stage, root):
    from pxr import Gf, Sdf, UsdShade, UsdPhysics
    for name, (color, metal, rough) in MATERIALS.items():
        material = UsdShade.Material.Define(stage, root + "/Looks/" + name)
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    material = UsdShade.Material.Define(stage, root + "/Looks/Contact")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.45)
    api.CreateDynamicFrictionAttr(.32)
    api.CreateRestitutionAttr(0.)
    rolling=UsdShade.Material.Define(stage,root+"/Looks/RollerContact")
    api=UsdPhysics.MaterialAPI.Apply(rolling.GetPrim())
    api.CreateStaticFrictionAttr(.06)
    api.CreateDynamicFrictionAttr(.04)
    api.CreateRestitutionAttr(0.)
    rolling.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    _attr(rolling.GetPrim(),"physxMaterial:frictionCombineMode",Sdf.ValueTypeNames.Token,"min")


def _bind(stage, prim, root, material, physics=False):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material.Get(stage, root + "/Looks/" + material),
        materialPurpose="physics" if physics else UsdShade.Tokens.allPurpose)


def _collision(stage, prim, root, material="Contact"):
    from pxr import Sdf, UsdPhysics
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    _attr(prim, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, .0005)
    _attr(prim, "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.)
    _bind(stage, prim, root, material, physics=True)


def _simplify(path, tolerance=.00015):
    """Douglas-Peucker with closed-polyline handling; bounds centreline deviation."""
    p = np.asarray(path, dtype=float)
    if len(p) <= 2:
        return p
    delta = p[-1] - p[0]
    if np.dot(delta, delta) < 1e-18:
        mid = len(p) // 2
        return np.concatenate((_simplify(p[:mid+1], tolerance)[:-1], _simplify(p[mid:], tolerance)))
    t = np.clip((p-p[0]) @ delta / np.dot(delta, delta), 0, 1)
    errors = np.linalg.norm(p - (p[0] + t[:, None]*delta), axis=1)
    index = int(errors.argmax())
    if errors[index] <= tolerance:
        return p[[0, -1]]
    return np.concatenate((_simplify(p[:index+1], tolerance)[:-1], _simplify(p[index:], tolerance)))


def _mesh(stage, path, points, normals, faces):
    from pxr import UsdGeom, Vt
    p = np.asarray(points, dtype=np.float32)
    faces = np.asarray(faces, dtype=np.int32)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(p))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    if normals is not None:
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, dtype=np.float32)))
        mesh.SetNormalsInterpolation("vertex")
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.array([p.min(0), p.max(0)], dtype=np.float32)))
    return mesh.GetPrim()


def _solid(stage, path, spec):
    from pxr import UsdGeom
    if spec["kind"] == "box":
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1.)
        _pose(shape.GetPrim(), spec["center"], scale=spec["size"])
    elif spec["kind"] == "cylinder":
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateAxisAttr(spec.get("axis", "Z"))
        shape.CreateRadiusAttr(float(spec["radius"]))
        shape.CreateHeightAttr(float(spec["height"]))
        _pose(shape.GetPrim(), spec["center"])
    else:
        raise ValueError(spec["kind"])
    return shape.GetPrim()


def _component_bounds(component):
    points = [np.asarray(m.points) for m in component["meshes"].values() if len(m.points)]
    for s in component["solids"]:
        if s["kind"] == "box":
            extent = np.asarray(s["size"]) / 2
        else:
            extent = np.full(3, s["radius"])
            extent["XYZ".index(s.get("axis", "Z"))] = s["height"] / 2
        points.append(np.array([np.asarray(s["center"])-extent, np.asarray(s["center"])+extent]))
    all_points = np.concatenate(points)
    return np.array([all_points.min(0), all_points.max(0)])


def write_component(component, name, filename):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    path = Path(filename)
    stage = Usd.Stage.CreateNew(str(path))
    root = "/" + name
    body = UsdGeom.Xform.Define(stage, root).GetPrim()
    stage.SetDefaultPrim(body)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.GetRootLayer().customLayerData = {"kilogramsPerUnit": 1., "generator": "dishsim_frigidaire.asset"}
    UsdPhysics.RigidBodyAPI.Apply(body)
    body.AddAppliedSchema("PhysxRigidBodyAPI")
    _attr(body, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 16)
    _attr(body, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 4)
    _attr(body, "physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool, True)
    bounds = _component_bounds(component)
    mass = UsdPhysics.MassAPI.Apply(body)
    mass.CreateMassAttr(float(component["mass"]))
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, bounds.mean(0))))
    size = bounds[1]-bounds[0]
    inertia = float(component["mass"])/12 * np.array([size[1]**2+size[2]**2,
                                                       size[0]**2+size[2]**2,
                                                       size[0]**2+size[1]**2])
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    geometry_key = {"UpperRack": "upper_rack", "LowerRack": "lower_rack"}.get(name)
    revision = PARAMETERS[geometry_key]["geometry_revision"] if geometry_key else "fdpc4221as_photo_v1"
    body.CreateAttribute("geometryRevision", Sdf.ValueTypeNames.String).Set(revision)
    body.CreateAttribute("dimensionStatus", Sdf.ValueTypeNames.String).Set("photo estimates; see parameters.json")
    _materials(stage, root)
    collision_scope = UsdGeom.Scope.Define(stage, root + "/Collisions").GetPrim()
    UsdGeom.Imageable(collision_scope).CreateVisibilityAttr("invisible")
    triangles = 0
    for key, mesh in component["meshes"].items():
        prim = _mesh(stage, root + "/Visuals/" + _identifier(key), mesh.points, mesh.normals, mesh.faces)
        _bind(stage, prim, root, mesh.material)
        triangles += len(mesh.faces)
    count = 0
    for key, path_points, radius in component["wires"]:
        simplified = _simplify(path_points)
        for a, b in zip(simplified[:-1], simplified[1:]):
            length = float(np.linalg.norm(b-a))
            if length < 1e-9:
                continue
            direction = (b-a)/length
            q = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*map(float, direction))).GetQuat()
            shape = UsdGeom.Capsule.Define(stage, root + f"/Collisions/Wire_{count:05d}")
            shape.CreateAxisAttr("Z")
            shape.CreateRadiusAttr(float(radius))
            shape.CreateHeightAttr(length)
            _pose(shape.GetPrim(), (a+b)/2, [q.GetReal(), *q.GetImaginary()])
            _collision(stage, shape.GetPrim(), root)
            shape.GetPrim().CreateAttribute("wireFamily", Sdf.ValueTypeNames.String).Set(key)
            count += 1
    for i, spec in enumerate(component["solids"]):
        key = _identifier(spec.get("name", f"Solid_{i:04d}")) + f"_{i:04d}"
        if spec.get("visual", True):
            prim = _solid(stage, root + "/Visuals/" + key, spec)
            _bind(stage, prim, root, spec["material"])
        if spec.get("collision", True):
            prim = _solid(stage, root + "/Collisions/" + key, spec)
            _collision(stage, prim, root, spec.get("physics_material","Contact"))
            count += 1
    for key, pos in component.get("sites", {}).items():
        prim = UsdGeom.Xform.Define(stage, root + "/Manipulation/" + key).GetPrim()
        q = component.get("site_quats", {}).get(key,
            {"plate": (.70710678, .70710678, 0., 0.), "cup": (0., 1., 0., 0.)}.get(key, (1., 0., 0., 0.)))
        _pose(prim, pos)
        prim.CreateAttribute("quatWXYZ", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*q))
        prim.CreateAttribute("status", Sdf.ValueTypeNames.String).Set("candidate; validate with actual object and gripper")
    stage.GetRootLayer().Save()
    return {"bounds_m": bounds.tolist(), "mass_kg": float(component["mass"]),
            "triangles": triangles, "colliders": count, "wire_paths": len(component["wires"]),
            "geometry_revision": revision}


def write_assembly(output_dir):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateNew(str(Path(output_dir)/"fdpc4221as.usdc"))
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    for name, filename in COMPONENT_FILES.items():
        prim = stage.DefinePrim(ROOT + "/" + name, "Xform")
        prim.GetReferences().AddReference(filename)
        _pose(prim, BODY_POSITIONS[name])
    cabinet = stage.GetPrimAtPath(ROOT + "/Cabinet")
    UsdPhysics.ArticulationRootAPI.Apply(cabinet)
    cabinet.AddAppliedSchema("PhysxArticulationAPI")
    _attr(cabinet, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, True)
    _attr(cabinet, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 32)
    _attr(cabinet, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 8)
    fixed = UsdPhysics.FixedJoint.Define(stage, ROOT + "/Joints/base_fixed")
    fixed.CreateBody1Rel().SetTargets([ROOT + "/Cabinet"])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(0))
    fixed.CreateLocalRot0Attr(Gf.Quatf(1))
    for key, body_name in [("door_hinge", "Door"), ("lower_slide", "LowerRack"), ("upper_slide", "UpperRack")]:
        angular = key == "door_hinge"
        joint_type = UsdPhysics.RevoluteJoint if angular else UsdPhysics.PrismaticJoint
        joint = joint_type.Define(stage, ROOT + "/Joints/" + key)
        joint.CreateAxisAttr("X" if angular else "Y")
        joint.CreateLowerLimitAttr(JOINT_LIMITS[key][0])
        joint.CreateUpperLimitAttr(JOINT_LIMITS[key][1])
        joint.CreateBody0Rel().SetTargets([ROOT + "/Cabinet"])
        joint.CreateBody1Rel().SetTargets([ROOT + "/" + body_name])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*BODY_POSITIONS[body_name]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if angular else "linear")
        drive.CreateTypeAttr("force")
        # Asset starts passive; loading helper selects driven setup explicitly.
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(.08 if angular else 8.)
        drive.CreateMaxForceAttr(30. if angular else 60.)
        drive.CreateTargetPositionAttr(0.)
        joint.GetPrim().AddAppliedSchema("PhysxJointAPI")
        _attr(joint.GetPrim(), "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, 45. if angular else .15)
    root.CreateAttribute("model", Sdf.ValueTypeNames.String).Set("Frigidaire FDPC4221AS")
    root.CreateAttribute("mode", Sdf.ValueTypeNames.String).Set("passive; use spawn() for counterbalance and relocated base anchoring")
    stage.GetRootLayer().Save()


def _fixture_mesh(profile, sectors=64):
    """Closed surfaces of revolution; profile is (radius, z)."""
    import trimesh
    mesh = trimesh.creation.revolve(np.asarray(profile), sections=sectors)
    mesh.fix_normals()
    return mesh


def write_fixtures(output_dir):
    """Full-size contact fixtures, local geometry and no external dependencies."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    specs = {
        "plate": ("PlateCeramic", .65, [(0,-.004),(.118,-.004),(.130,0),(.130,.005),(.118,.005),(0,.002)]),
        "cup": ("CupCeramic", .25, [(0,-.05),(.040,-.05),(.040,.05),(.035,.05),(.035,-.045),(0,-.045)]),
        "bowl": ("BowlCeramic", .35, [(0,0),(.035,0),(.070,.065),(.066,.065),(.032,.005),(0,.005)]),
    }
    directory = Path(output_dir)/"fixtures"
    directory.mkdir(exist_ok=True)
    for name, (material, mass, profile) in specs.items():
        stage = Usd.Stage.CreateNew(str(directory/f"{name}.usdc"))
        root = "/" + name.title()
        body = UsdGeom.Xform.Define(stage, root).GetPrim()
        stage.SetDefaultPrim(body)
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(mass)
        _materials(stage, root)
        mesh = _fixture_mesh(profile)
        prim = _mesh(stage, root + "/Visual", mesh.vertices, mesh.vertex_normals, mesh.faces)
        _bind(stage, prim, root, material)
        coll = UsdGeom.Scope.Define(stage, root + "/Collisions")
        UsdGeom.Imageable(coll).CreateVisibilityAttr("invisible")
        if name == "plate":
            prim = _solid(stage, root + "/Collisions/Disc", {"kind":"cylinder", "center":(0,0,.0005),
                          "radius":.130, "height":.009, "axis":"Z"})
            _collision(stage, prim, root)
        else:
            # A convex piece per wall sector preserves the cup/bowl cavity.
            wall = ([(.040,-.05),(.040,.05),(.035,.05),(.035,-.05)] if name == "cup" else
                    [(.035,0),(.070,.065),(.066,.065),(.031,0)])
            import trimesh
            for j in range(32):
                vertices = [(r*math.cos(a), r*math.sin(a), z)
                            for a in [j*math.tau/32, (j+1)*math.tau/32] for r,z in wall]
                hull = trimesh.convex.convex_hull(vertices)
                prim = _mesh(stage, root + f"/Collisions/Wall_{j}", hull.vertices, None, hull.faces)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
                _collision(stage, prim, root)
            prim = _solid(stage, root + "/Collisions/Base", {"kind":"cylinder", "center":(0,0,-.0475 if name=="cup" else .0025),
                          "radius":.038 if name=="cup" else .033, "height":.005,"axis":"Z"})
            _collision(stage, prim, root)
        stage.GetRootLayer().Save()
    # A utensil with a broad head that cannot disappear through a lattice aperture.
    component = {"mass":.035, "meshes":{}, "wires":[], "sites":{}, "solids":[
        {"kind":"box", "name":"Handle", "center":(0,0,-.0125),"size":(.008,.004,.155),"material":"UtensilSteel"},
        {"kind":"box", "name":"Head", "center":(0,0,.069),"size":(.027,.004,.042),"material":"UtensilSteel"}]}
    write_component(component, "Utensil", directory/"utensil.usdc")
    (directory/"dimensions.json").write_text(json.dumps({"plate_diameter_m":.26,
        "cup_diameter_m":.08,"cup_height_m":.10,"bowl_diameter_m":.14,"bowl_height_m":.065,
        "utensil_length_m":.18,"status":"procedural contact fixtures, not benchmark-scaled props"},indent=2)+"\n")


def write_example_scene(output_dir):
    """A portable stage with floor, lighting, camera and the complete appliance."""
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
    filename=Path(output_dir)/"example_scene.usda"
    if filename.exists():
        filename.unlink()
    stage=Usd.Stage.CreateNew(str(filename))
    world=UsdGeom.Xform.Define(stage,"/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage,"Z")
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    scene=UsdPhysics.Scene.Define(stage,"/World/Physics")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
    scene.CreateGravityMagnitudeAttr(9.81)
    root=stage.DefinePrim("/World/FrigidaireFDPC4221AS","Xform")
    root.GetReferences().AddReference("fdpc4221as.usdc")
    ground=_solid(stage,"/World/Floor",{"kind":"box","center":(0,0,-.025),"size":(6,6,.05)})
    UsdPhysics.CollisionAPI.Apply(ground)
    UsdGeom.Gprim(ground).CreateDisplayColorAttr([Gf.Vec3f(.19,.22,.25)])
    dome=UsdLux.DomeLight.Define(stage,"/World/Fill")
    dome.CreateIntensityAttr(1100.)
    key=UsdLux.DistantLight.Define(stage,"/World/Key")
    key.CreateIntensityAttr(2200.)
    key.CreateAngleAttr(15.)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(30,-25,-25))
    camera=UsdGeom.Camera.Define(stage,"/World/Camera")
    camera.CreateFocalLengthAttr(45.)
    camera.CreateHorizontalApertureAttr(36.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
    matrix=Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(1.65,-2.2,1.6),Gf.Vec3d(0,-.30,.43),Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(camera).AddTransformOp().Set(matrix)
    stage.GetRootLayer().Save()


def _validate_component_geometry(component, name):
    """Reject invalid generated meshes before any existing asset is replaced."""
    for key, mesh in component["meshes"].items():
        points, faces = np.asarray(mesh.points), np.asarray(mesh.faces)
        if (not len(points) or not len(faces) or not np.isfinite(points).all()
                or faces.min() < 0 or faces.max() >= len(points)):
            raise ValueError(f"{name}/{key}: invalid mesh points or indices")
        area2 = np.linalg.norm(np.cross(points[faces[:, 1]]-points[faces[:, 0]],
                                        points[faces[:, 2]]-points[faces[:, 0]]), axis=1)
        if area2.min() <= 1e-14:
            raise ValueError(f"{name}/{key}: degenerate mesh face")


def _validate_component_usd(filename, name, expected):
    """Reopen the staged component and check its authoring before installation."""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim() if stage else None
    if not root or str(root.GetPath()) != "/" + name:
        raise ValueError(f"{name}: missing or incorrect default prim")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1. or UsdGeom.GetStageUpAxis(stage) != "Z":
        raise ValueError(f"{name}: incorrect units or up axis")
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(filename))
    if unresolved:
        raise ValueError(f"{name}: unresolved USD dependencies: {unresolved}")
    mass = UsdPhysics.MassAPI(root)
    inertia = np.asarray(mass.GetDiagonalInertiaAttr().Get(), dtype=float)
    if (not np.isclose(mass.GetMassAttr().Get(), expected["mass_kg"], rtol=1e-6)
            or inertia.shape != (3,)
            or not np.isfinite(inertia).all() or (inertia <= 0).any()):
        raise ValueError(f"{name}: invalid mass or inertia")
    colliders = triangles = 0
    for prim in stage.Traverse():
        colliders += prim.HasAPI(UsdPhysics.CollisionAPI)
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        points = np.asarray(mesh.GetPointsAttr().Get())
        if (not len(points) or not np.isfinite(points).all() or not len(faces)
                or np.any(counts != 3) or len(faces) != 3*len(counts)
                or faces.min() < 0 or faces.max() >= len(points)):
            raise ValueError(f"{name}: invalid authored mesh {prim.GetPath()}")
        triangles += len(counts)
        if not UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
            raise ValueError(f"{name}: unbound mesh {prim.GetPath()}")
    if colliders != expected["colliders"] or triangles != expected["triangles"]:
        raise ValueError(f"{name}: geometry counts changed during USD authoring")


_STALE_LOAD_NOTICE = (
    "> Upper-rack geometry has changed. Existing full-load scenes, poses, counts, "
    "reports and their hashes are historical evidence for the previous geometry. "
    "They have been preserved unchanged and do not validate the current rack. "
    "Rerun loading and Isaac Sim validation before claiming a current capacity.\n\n"
)
_LOWER_STALE_LOAD_NOTICE = _STALE_LOAD_NOTICE.replace(
    "Upper-rack geometry has changed.", "Lower-rack geometry and the basket mounting position have changed.")


def _validate_basket_only_layer_change(before, edited, position):
    """Compare every assembly opinion except the basket's translation default."""
    from pxr import Sdf

    path = ROOT + "/SilverwareBasket.xformOp:translate"
    a, b = Sdf.Layer.CreateAnonymous(), Sdf.Layer.CreateAnonymous()
    a.TransferContent(before)
    b.TransferContent(edited)
    original, changed = a.GetAttributeAtPath(path), b.GetAttributeAtPath(path)
    if not original or not changed or original.default is None:
        raise ValueError("Assembly must author the basket translation in its root layer")
    if not np.allclose(changed.default, position, rtol=0., atol=1e-9):
        raise ValueError("Staged assembly has an incorrect basket translation")
    changed.default = original.default
    if a.ExportToString() != b.ExportToString():
        raise ValueError("Assembly change outside the basket translation")


def _patch_basket_assembly(source, destination, position):
    """Copy all authored assembly opinions, then change one translation default."""
    from pxr import Gf, Sdf

    before = Sdf.Layer.FindOrOpen(str(source))
    if not before:
        raise ValueError("Could not open the existing assembly")
    edited = Sdf.Layer.CreateAnonymous("basket-position.usda")
    edited.TransferContent(before)
    path = ROOT + "/SilverwareBasket.xformOp:translate"
    translation = edited.GetAttributeAtPath(path)
    if not translation or translation.default is None or edited.ListTimeSamplesForPath(path):
        raise ValueError("Assembly must have a static authored basket translation")
    previous = list(translation.default)
    translation.default = Gf.Vec3d(*map(float, position))
    _validate_basket_only_layer_change(before, edited, position)
    if not edited.Export(str(destination)):
        raise ValueError("Could not export the staged assembly")
    reopened = Sdf.Layer.FindOrOpen(str(destination))
    _validate_basket_only_layer_change(before, reopened, position)
    return {"body": "SilverwareBasket", "previous_position_m": previous,
            "current_position_m": list(position),
            "preserved_opinions": "all assembly opinions except this translation default"}


def _validate_rack_assembly(filename, rack_name, basket_position):
    """Verify the staged rack and retained component references compose together."""
    from pxr import Usd, UsdGeom, UsdUtils

    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim() if stage else None
    if not root or str(root.GetPath()) != ROOT:
        raise ValueError("Staged assembly has an incorrect default prim")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1. or UsdGeom.GetStageUpAxis(stage) != "Z":
        raise ValueError("Staged assembly has incorrect units or up axis")
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(filename))
    if unresolved:
        raise ValueError(f"Staged assembly has unresolved dependencies: {unresolved}")
    for name in COMPONENT_FILES:
        if not stage.GetPrimAtPath(ROOT + "/" + name + "/Visuals"):
            raise ValueError(f"Staged assembly could not compose {name}")
    basket = stage.GetPrimAtPath(ROOT + "/SilverwareBasket")
    if not np.allclose(basket.GetAttribute("xformOp:translate").Get(), basket_position, rtol=0., atol=1e-9):
        raise ValueError("Composed basket position disagrees with geometry parameters")
    if not stage.GetPrimAtPath(ROOT + "/" + rack_name + "/Collisions"):
        raise ValueError(f"Staged assembly has no {rack_name} collision geometry")


def _update_rack(output_dir, name):
    """Stage one rack revision and invalidate preserved historical load evidence."""
    from .geometry import _lower_rack, _upper_rack

    geometry_key = {"UpperRack": "upper_rack", "LowerRack": "lower_rack"}[name]
    label = geometry_key.replace("_", "-")
    notice = _STALE_LOAD_NOTICE if name == "UpperRack" else _LOWER_STALE_LOAD_NOTICE

    required = [*COMPONENT_FILES.values(), "fdpc4221as.usdc", "example_scene.usda",
                "parameters.json", "geometry_validation.json", "fixtures/dimensions.json",
                *(f"fixtures/{name}.usdc" for name in ("plate", "cup", "bowl", "utensil")),
                "tableware/catalog.json"]
    missing = [name for name in required if not (output_dir/name).is_file()]
    if missing:
        raise ValueError(f"--component {name} requires an existing complete bundle; missing: "
                         + ", ".join(missing))
    parameters = json.loads((output_dir/"parameters.json").read_text())
    report = json.loads((output_dir/"geometry_validation.json").read_text())
    if (not all(name in report.get("components", {}) for name in COMPONENT_FILES)
            or geometry_key not in parameters.get("geometry", {})
            or name not in parameters.get("candidate_sites", {})
            or not report.get("tableware", {}).get("items")):
        raise ValueError(f"--component {name} requires complete geometry and tableware metadata")
    if name == "LowerRack" and (
            "SilverwareBasket" not in parameters["geometry"].get("origins", {})
            or "SilverwareBasket" not in parameters.get("body_positions_m", {})):
        raise ValueError("--component LowerRack requires existing basket origin and body-position metadata")
    tableware_files = [item["usd"] for item in report["tableware"]["items"].values()]
    missing = [name for name in tableware_files if not (output_dir/name).is_file()]
    if missing:
        raise ValueError("Existing bundle has missing tableware: " + ", ".join(missing))

    # Record actual input bytes, rather than trusting potentially old report hashes.
    before = {str(path.relative_to(output_dir)): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(output_dir.rglob("*")) if path.is_file()}
    filename = COMPONENT_FILES[name]
    component = {"UpperRack": _upper_rack, "LowerRack": _lower_rack}[name]()
    _validate_component_geometry(component, name)
    with tempfile.TemporaryDirectory(prefix="."+label+"-", dir=output_dir) as temporary:
        staging = Path(temporary)
        report["components"][name] = write_component(component, name, staging/filename)
        _validate_component_usd(staging/filename, name, report["components"][name])
        changes = [filename]
        assembly_update = None
        if name == "LowerRack":
            assembly = "fdpc4221as.usdc"
            reserved = {filename, assembly, "parameters.json", "geometry_validation.json", "README.md", "CAPACITY.md"}
            # Retain an equivalent relative-reference environment without rewriting
            # any existing dependencies. Reserved names are newly staged files.
            for path in output_dir.iterdir():
                if path != staging and path.name not in reserved:
                    (staging/path.name).symlink_to(path.resolve(), target_is_directory=path.is_dir())
            position = PARAMETERS["origins"]["SilverwareBasket"]
            assembly_update = _patch_basket_assembly(output_dir/assembly, staging/assembly, position)
            _validate_rack_assembly(staging/assembly, name, position)
            changes.append(assembly)
            parameters["geometry"]["origins"]["SilverwareBasket"] = list(position)
            parameters["body_positions_m"]["SilverwareBasket"] = list(position)
        parameters["geometry"][geometry_key] = PARAMETERS[geometry_key]
        parameters["candidate_sites"][name] = {
            "positions_m": component.get("sites", {}),
            "quat_wxyz_overrides": component.get("site_quats", {})}
        report["result"] = f"PASS ({name} USD authoring only; physics and loading unvalidated)"
        report["isaac_sim_validated"] = False
        report["sha256"] = {name: digest for name, digest in before.items() if name.endswith(".usdc")}
        for changed_file in changes:
            report["sha256"][changed_file] = hashlib.sha256((staging/changed_file).read_bytes()).hexdigest()
        report["component_update"] = {
            "component": name, "previous_sha256": before[filename],
            "current_sha256": report["sha256"][filename],
            "geometry_revision": PARAMETERS[geometry_key]["geometry_revision"],
            "unchanged_component_reports": [body for body in COMPONENT_FILES if body != name],
            "physics_validation_status": f"required after {label} geometry update",
            "loading_validation_status": "required; candidate sites and preserved full-load poses are not accepted placements for this revision",
            "prior_full_load_evidence": {
                "status": "stale; previous geometry only; original files and hashes retained",
                "sha256": {name: digest for name, digest in before.items()
                           if Path(name).name.startswith("full_load")}}}
        if assembly_update is not None:
            report["component_update"]["assembly_update"] = {
                **assembly_update, "previous_sha256": before["fdpc4221as.usdc"],
                "current_sha256": report["sha256"]["fdpc4221as.usdc"]}
        changes.extend(["parameters.json", "geometry_validation.json"])
        for metadata_name, value in [("parameters.json", parameters), ("geometry_validation.json", report)]:
            (staging/metadata_name).write_text(json.dumps(value, indent=2)+"\n")
        for document in ("README.md", "CAPACITY.md"):
            if not (output_dir/document).is_file():
                continue
            original = (output_dir/document).read_text()
            if notice in original:
                continue
            (staging/document).write_text(notice+original)
            changes.append(document)

        # Check the bundle did not change while authoring, then replace only the
        # explicit targets. Retain original bytes to roll back install errors.
        for name, digest in before.items():
            if hashlib.sha256((output_dir/name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"Bundle changed during {label} staging: {name}")
        originals = {name: (output_dir/name).read_bytes() for name in changes}
        installed = []
        try:
            for name in changes:
                (staging/name).replace(output_dir/name)
                installed.append(name)
        except OSError:
            for name in reversed(installed):
                (staging/name).write_bytes(originals[name])
                (staging/name).replace(output_dir/name)
            raise
    return report


def _update_upper_rack(output_dir):
    """Compatibility helper for an upper-only component update."""
    return _update_rack(output_dir, "UpperRack")


def build(output_dir=ASSET_DIR, component=None):
    """Build a complete bundle, or update one rack in an existing bundle."""
    if component is not None:
        if component not in {"UpperRack", "LowerRack"}:
            raise ValueError(f"Unsupported component update: {component!r}; expected 'UpperRack' or 'LowerRack'")
        return _update_rack(Path(output_dir), component)
    from .geometry import PARAMETERS, build_components
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    components = build_components()
    report = {"result":"PASS (USD authoring only)", "isaac_sim_validated":False, "components":{}}
    for name, component in components.items():
        for key, mesh in component["meshes"].items():
            points, faces = np.asarray(mesh.points), np.asarray(mesh.faces)
            assert np.isfinite(points).all() and faces.min() >= 0 and faces.max() < len(points), (name,key)
            area2 = np.linalg.norm(np.cross(points[faces[:,1]]-points[faces[:,0]],points[faces[:,2]]-points[faces[:,0]]),axis=1)
            assert area2.min() > 1e-14, (name,key,"degenerate face")
        filename = output_dir/COMPONENT_FILES[name]
        if filename.exists():
            filename.unlink()
        report["components"][name] = write_component(component,name,filename)
    target = output_dir/"fdpc4221as.usdc"
    if target.exists():
        target.unlink()
    write_assembly(output_dir)
    for filename in (output_dir/"fixtures").glob("*.usdc"):
        filename.unlink()
    write_fixtures(output_dir)
    from .tableware import write_tableware
    report["tableware"] = write_tableware(output_dir)
    write_example_scene(output_dir)
    (output_dir/"parameters.json").write_text(json.dumps({"geometry":PARAMETERS,"body_positions_m":BODY_POSITIONS,
        "candidate_sites":{name:{"positions_m":c.get("sites",{}),"quat_wxyz_overrides":c.get("site_quats",{})}
                           for name,c in components.items()},
        "joint_limits_usd":JOINT_LIMITS,"joint_units":"USD angle degrees; Isaac Lab angle radians; slides metres",
        "dimension_sources":["assets/models/frigidaire_fdpc4221as/references/spec.pdf page 3", "https://www.frigidaire.ca/Kitchen/Dishwashers/Dishwasher/FDPC4221AS/"],
        "collision_centerline_error_max_m":.00015,"contact_offset_m":.0005,
        "roller_friction":{"static":.06,"dynamic":.04,"combine":"min",
                           "reason":"rolling approximation for constrained nonrotating roller geometry"},
        "mass_and_inertia_status":"engineering estimates; inertia approximated from component envelope",
        "passive_forces_status":"approximate counterbalance/resistance, not measured appliance forces"},indent=2)+"\n")
    report["sha256"] = {str(p.relative_to(output_dir)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output_dir.rglob("*.usdc"))}
    (output_dir/"geometry_validation.json").write_text(json.dumps(report,indent=2)+"\n")
    (output_dir/"README.md").write_text("# Frigidaire FDPC4221AS\n\n"
        "Open fdpc4221as.usdc for the appliance, or example_scene.usda for a lit scene. "
        "Copy this entire directory to preserve relative component references.\n\n"
        "Meters, Z up, front -Y, width X. Default prim /FrigidaireFDPC4221AS. "
        "Door 0..90 degrees; lower_slide -0.49..0 m; upper_slide -0.44..0 m. "
        "The basket is an independent rigid body.\n\n"
        "For scripted motion or passive spring counterbalance, use dishsim_frigidaire.asset.spawn "
        "and frigidaire/scripts/experiment/frigidaire_demo.py in the source repository. "
        "The loader also relocates the world-side fixed joint when spawning off origin.\n\n"
        "Dimensions absent from the reference specification are estimates. See parameters.json. "
        "geometry_validation.json covers authoring only; physics validation is recorded separately "
        "in ../validation and rendered views and rack dimension layouts in ../images. "
        "The empty example scene contains the complete appliance with the polished 52-tine upper "
        "rack, 72-tine lower rack, and corrected basket position.\n\n"
        "The fixtures and tableware directories contain optional independent rigid-body prototypes. "
        "Dish placements and capacity are unvalidated for this rack revision. Historical loaded "
        "scenes and their evidence live separately in ../history.\n")
    return report


def spawn(prim_path, position=(0.,0.,0.), yaw=0., mode="passive", usd_path=None):
    """Spawn before reset. Returns (articulation, independent basket).

    Position and yaw specify a world pose, with angles in radians. Existing parent
    transforms may translate or rotate the asset hierarchy, but must not scale it.
    The world's weld frame follows the requested pose; changing only an ancestor
    Xform after spawning would otherwise snap the fixed articulation back.
    """
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
    if mode not in {"scripted","passive"}:
        raise ValueError(mode)
    q = (math.cos(yaw/2),0.,0.,math.sin(yaw/2))
    asset = str(usd_path or ASSET_DIR/"fdpc4221as.usdc")
    if "://" not in asset:
        asset = str(Path(asset).resolve())
    stage = omni.usd.get_context().get_stage()
    # Isaac Lab's USD spawner accepts a transform relative to the parent. The
    # public helper accepts a world pose, as do the fixed joint and initial states.
    # Missing intermediate parents are created as identity transforms by USD.
    parent_path = Sdf.Path(prim_path).GetParentPath()
    while parent_path != Sdf.Path.emptyPath and not stage.GetPrimAtPath(parent_path):
        parent_path = parent_path.GetParentPath()
    parent = stage.GetPrimAtPath(parent_path) if parent_path != Sdf.Path.emptyPath else stage.GetPseudoRoot()
    parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(parent)
    linear = np.asarray(parent_world)[:3, :3]
    if not np.allclose(linear @ linear.T, np.eye(3), atol=1e-6) or np.linalg.det(linear) < 0:
        raise ValueError("Frigidaire spawn requires a parent without scale, shear, or reflection")
    requested_world = Gf.Matrix4d(1)
    requested_world.SetRotate(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
    requested_world.SetTranslateOnly(Gf.Vec3d(*map(float, position)))
    local = requested_world * parent_world.GetInverse()
    local_q = local.ExtractRotationQuat()
    sim_utils.UsdFileCfg(usd_path=asset).func(prim_path,sim_utils.UsdFileCfg(usd_path=asset),
        translation=tuple(local.ExtractTranslation()),
        orientation=(local_q.GetReal(), *local_q.GetImaginary()))
    fixed = UsdPhysics.FixedJoint.Get(stage,prim_path+"/Joints/base_fixed")
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*map(float,position)))
    fixed.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
    cfg = ArticulationCfg(prim_path=prim_path,spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=position,rot=q,joint_pos={".*":0.}),
        actuators={"door":ImplicitActuatorCfg(joint_names_expr=["door_hinge"],
                    stiffness=300. if mode=="scripted" else 0., damping=45. if mode=="scripted" else .8,
                    effort_limit_sim=30.,velocity_limit_sim=.65),
                   "racks":ImplicitActuatorCfg(joint_names_expr=[".*_slide"],
                    stiffness=1600. if mode=="scripted" else 0., damping=180. if mode=="scripted" else 8.,
                    effort_limit_sim=60.,velocity_limit_sim=.15)})
    appliance = Articulation(cfg)
    door_mass=UsdPhysics.MassAPI(stage.GetPrimAtPath(prim_path+"/Door"))
    appliance._frigidaire_door_mass=float(door_mass.GetMassAttr().Get())
    appliance._frigidaire_door_com=tuple(door_mass.GetCenterOfMassAttr().Get())
    basket_world = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(prim_path+"/SilverwareBasket"))
    basket_q = basket_world.ExtractRotationQuat()
    basket = RigidObject(RigidObjectCfg(prim_path=prim_path+"/SilverwareBasket",spawn=None,
        init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(basket_world.ExtractTranslation()),
            rot=(basket_q.GetReal(), *basket_q.GetImaginary()))))
    appliance._frigidaire_mode = mode
    return appliance,basket


def apply_mode(appliance, mode):
    """Switch resistance/drive gains after reset; no position teleporting."""
    import torch
    if mode not in {"passive","scripted"}:
        raise ValueError(mode)
    stiffness,damping=[],[]
    for name in appliance.joint_names:
        door=name=="door_hinge"
        stiffness.append((300. if door else 1600.) if mode=="scripted" else 0.)
        damping.append((45. if door else 180.) if mode=="scripted" else (.8 if door else 8.))
    k=torch.tensor([stiffness],device=appliance.device)
    d=torch.tensor([damping],device=appliance.device)
    appliance.write_joint_stiffness_to_sim(k)
    appliance.write_joint_damping_to_sim(d)
    # ImplicitActuator mirrors the gains for effort estimation; update that cache too.
    for actuator in appliance.actuators.values():
        actuator.stiffness[:]=k[:,actuator.joint_indices]
        actuator.damping[:]=d[:,actuator.joint_indices]
    appliance.set_joint_effort_target(torch.zeros_like(appliance.data.joint_pos))
    appliance._frigidaire_mode=mode


def step_passive(appliance):
    """Approximate door spring counterbalance plus bounded smooth rack friction.

    Feed this before write_data_to_sim each step in passive mode. Effort offsets
    approximate gravity for the authored door COM; no position servo holds it.
    The mechanical squeeze latch is represented by a released state in this mode.
    """
    import torch
    if getattr(appliance,"_frigidaire_mode",None)!="passive":
        return
    effort=torch.zeros_like(appliance.data.joint_pos)
    for i,name in enumerate(appliance.joint_names):
        q=appliance.data.joint_pos[:,i]
        velocity=appliance.data.joint_vel[:,i]
        if name=="door_hinge":
            mass=appliance._frigidaire_door_mass
            _,cy,cz=appliance._frigidaire_door_com
            effort[:,i]=-mass*9.81*(cz*torch.sin(q)-cy*torch.cos(q)) - .30*torch.tanh(velocity/.015)
        else:
            effort[:,i]=-1.2*torch.tanh(velocity/.008)
    appliance.set_joint_effort_target(effort)
    return effort
