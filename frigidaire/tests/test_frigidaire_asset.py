"""Independent USD acceptance checks for the standalone FDPC4221AS asset.

The inspection runs once in a subprocess using Isaac's bundled USD, without
starting Kit. This keeps ordinary pytest collection independent of pxr's loader
and makes ``scripts/run_py.sh -m pytest frigidaire/tests/test_frigidaire_asset.py`` sufficient.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import pytest


_ROOT = "/FrigidaireFDPC4221AS"
_FILES = ("fdpc4221as", "cabinet", "door", "upper_rack", "lower_rack", "silverware_basket", "example_scene")
_BODIES = ("Cabinet", "Door", "UpperRack", "LowerRack", "SilverwareBasket")


@pytest.fixture(scope="module")
def inspected_asset():
    from dishsim_frigidaire.usd_bootstrap import bundled_usd_environment

    # Keep test scratch local and writable even when the asset collection is mounted read-only.
    scratch = Path(__file__).resolve().parents[2] / "build/frigidaire_tests"
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="frigidaire_test_", dir=scratch) as temporary:
        directory = Path(temporary)
        run = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), str(directory)],
            env=bundled_usd_environment(),
            text=True,
            capture_output=True,
            timeout=180,
            check=False,
        )
        assert run.returncode == 0, f"USD asset inspection failed:\n{run.stdout}\n{run.stderr}"
        report_path = directory / "inspection.json"
        assert report_path.is_file(), f"Inspection did not write its report:\n{run.stdout}\n{run.stderr}"
        return json.loads(report_path.read_text())


def test_all_components_compose_with_local_dependencies(inspected_asset):
    """Moving the complete folder must retain all geometry and material references."""
    report = inspected_asset
    assert set(report["files"]) == set(_FILES)
    for name, info in report["files"].items():
        assert info["default_prim"], name
        assert info["meters_per_unit"] == 1.0, name
        assert info["up_axis"] == "Z", name
        assert info["unresolved_assets"] == [], name
        assert info["geometry_count"] > 0, name
        assert info["absolute_references"] == [], name
    assert report["root"] == _ROOT
    assert report["relocated_body_count"] == len(_BODIES)
    from dishsim_frigidaire.geometry import PARAMETERS
    for rack in ("upper_rack", "lower_rack"):
        assert report["files"][rack]["geometry_revision"] == PARAMETERS[rack]["geometry_revision"]


def test_meshes_and_mass_properties_are_physically_valid(inspected_asset):
    report = inspected_asset
    assert report["invalid_meshes"] == []
    assert report["unbound_meshes"] == []
    assert report["documented_origin_error_m"] < 1e-7
    assert set(report["mass_properties"]) == set(_BODIES)
    for body, props in report["mass_properties"].items():
        assert props["mass"] > 0, body
        inertia = np.asarray(props["inertia"])
        assert np.isfinite(inertia).all() and (inertia > 0).all(), body
        assert 2 * inertia.max() <= inertia.sum() + 1e-8, body
    assert np.allclose(report["closed_assembly_dimensions"], [0.6096, 0.635, 0.8509], atol=0.001)
    assert abs(report["door_open_depth"] - 1.25095) < 0.001


def test_rack_and_basket_colliders_preserve_openings(inspected_asset):
    """A narrow vertical probe can pass through real floor apertures in every basket."""
    for name, info in inspected_asset["apertures"].items():
        assert info["unsupported_colliders"] == [], name
        assert info["collider_count"] > 50, name
        assert info["clear_ray_fraction"] > 0.12, (
            f"{name}: too few floor apertures remain open to a 1 mm diameter probe: {info}"
        )


def test_wire_collision_centerlines_match_visible_geometry(inspected_asset):
    """Exported capsule transforms cannot silently rotate or shorten a wire."""
    assert inspected_asset["maximum_wire_centerline_error_m"] <= 0.00025
    assert inspected_asset["maximum_wire_radius_error_m"] <= 1e-7
    assert inspected_asset["unmatched_wire_families"] == []


def test_joints_remain_consistent_after_translated_rotated_spawn(inspected_asset):
    """All constraint frames must coincide in world coordinates at either spawn pose."""
    report = inspected_asset
    assert set(report["joint_types"]) == {"door_hinge", "upper_slide", "lower_slide"}
    assert report["joint_types"]["door_hinge"] == "PhysicsRevoluteJoint"
    assert report["joint_types"]["upper_slide"] == "PhysicsPrismaticJoint"
    assert report["joint_types"]["lower_slide"] == "PhysicsPrismaticJoint"
    assert report["maximum_anchor_error_m"] < 1e-5
    assert report["maximum_frame_angle_error_deg"] < 1e-3
    assert np.allclose(report["spawned_origin"], [1.23, -0.87, 0.11], atol=1e-6)
    assert report["spawned_forward_error"] < 1e-6


def test_basket_is_removable_and_contact_filters_are_narrow(inspected_asset):
    report = inspected_asset
    assert report["basket_joint_count"] == 0
    assert report["articulation_self_collisions"] is True
    assert report["basket_filtered_pairs"] == []
    assert report["door_rack_filtered_pairs"] == []


def test_loader_uses_world_pose_beneath_transformed_parents(inspected_asset):
    """Exercise actual spawn math; stub only Isaac's object construction boundary."""
    for name, errors in inspected_asset["loader_world_poses"].items():
        assert max(errors.values()) < 1e-6, (name, errors)
    assert inspected_asset["scaled_parent_rejected"] is True


@pytest.fixture(scope="module")
def reconstructed_components():
    from dishsim_frigidaire.geometry import build_components

    return build_components()


def _wire_family_bounds(component, prefix):
    wires = [(path, radius) for name, path, radius in component["wires"]
             if name.startswith(prefix)]
    assert wires, prefix
    return (np.min([path.min(axis=0)-radius for path, radius in wires], axis=0),
            np.max([path.max(axis=0)+radius for path, radius in wires], axis=0))


def test_revised_rim_envelopes_match_user_dimensions(reconstructed_components):
    """The dimensions apply to the actual wire rim, excluding wheel/grip projections."""
    for body, rim, width, length in [
        ("LowerRack", "UpperRim", .54864, .58166),
        ("UpperRack", "TopRim", .50800, .54864),
    ]:
        lo, hi = _wire_family_bounds(reconstructed_components[body], rim)
        assert np.allclose((hi-lo)[:2], [width, length], atol=0.0001), body


def test_lower_tines_match_measured_grid_and_basket_clearance(reconstructed_components):
    """All 72 tines fit the agreed margins beside the translated, full-size basket."""
    from dishsim_frigidaire.geometry import PARAMETERS

    rack = reconstructed_components["LowerRack"]
    rows = {}
    for name, path, radius in rack["wires"]:
        if not name.startswith("TineBank") or "_Tooth" not in name:
            continue
        rows.setdefault(round(float(path[0, 1]), 6), []).append((path, radius))
    assert len(rows) == 6
    ys = np.array(sorted(rows))
    assert np.allclose(ys, [-.18583, -.112498, -.039166, .034166, .107498, .18083], atol=1e-9)
    assert np.allclose(np.diff(ys), .073332, atol=1e-9)
    lo, hi = _wire_family_bounds(rack, "UpperRim")
    assert np.allclose([ys[0]-lo[1], hi[1]-ys[-1]], [.105, .110], atol=1e-7)
    for y in ys:
        teeth = sorted(rows[y], key=lambda t: t[0][0, 0])
        assert len(teeth) == 12
        xs = np.asarray([path[0, 0] for path, _ in teeth])
        assert np.allclose(np.diff(xs), .03169454545454545, atol=1e-9)
        assert np.allclose([xs[0]-lo[0], hi[0]-xs[-1]], [.080, .120], atol=1e-7)
        assert all(np.allclose(path[-1]-path[0], [.008, 0, .105], atol=1e-9)
                   and abs(radius-.00195) < 1e-9 for path, radius in teeth)
    basket = reconstructed_components["SilverwareBasket"]
    basket_lo, basket_hi = _wire_family_bounds(basket, "")
    seat = np.asarray(PARAMETERS["origins"]["SilverwareBasket"])-PARAMETERS["origins"]["LowerRack"]
    assert np.allclose(seat, [.2135, .120, .011], atol=1e-9)
    assert np.allclose(rack["sites"]["basket_seat"], seat, atol=1e-9)
    assert np.allclose((basket_hi-basket_lo)[:2], [.088, .312], atol=1e-7)
    tine_right = max(path[:, 0].max()+radius for row in rows.values() for path, radius in row)
    assert basket_lo[0]+seat[0]-tine_right >= .00523-1e-9
    reserved = PARAMETERS["lower_rack"]["basket_reserved_x"]
    assert reserved[0] <= basket_lo[0]+seat[0]
    assert reserved[1] >= basket_hi[0]+seat[0]


def test_lower_tine_rails_remain_attached_to_floor(reconstructed_components):
    rack = reconstructed_components["LowerRack"]
    floor = [w for w in rack["wires"] if w[0].startswith("FloorLongU")]
    rails = [w for w in rack["wires"] if w[0].startswith("TineBank") and w[0].endswith("_Base")]
    assert len(rails) == 6
    for name, rail, radius in rails:
        assert np.allclose(rail[:, 2], .006, atol=1e-9)
        crossings = 0
        for _, path, floor_radius in floor:
            if not rail[0, 0] <= path[0, 0] <= rail[-1, 0]:
                continue
            floor_z = float(np.interp(rail[0, 1], path[:, 1], path[:, 2]))
            assert 0 < rail[0, 2]-floor_z < radius+floor_radius
            crossings += 1
        assert crossings >= 10
        for tooth_name, tooth, _ in rack["wires"]:
            if tooth_name.startswith(name[:-len("_Base")]+"_Tooth"):
                assert np.allclose(tooth[0, [1, 2]], rail[0, [1, 2]], atol=1e-9)


def test_upper_floor_preserves_five_loading_channels(reconstructed_components):
    """Both outer slopes, both mug valleys and the central valley must remain distinct."""
    rack = reconstructed_components["UpperRack"]
    _, section, _ = next(w for w in rack["wires"] if w[0].startswith("ContouredCrossU"))
    height = lambda x: float(np.interp(x, section[:, 0], section[:, 2]))
    # Ridge-to-valley differences survive rounded bends and are large enough
    # to change how a glass/mug rests, rather than a decorative corrugation.
    for sign in [-1, 1]:
        assert height(sign*.153)-height(sign*.234) > .020
        assert height(sign*.153)-height(sign*.130) > .020
        assert height(sign*.065)-height(sign*.109) > .010
        assert height(sign*.065)-height(0) > .010
    assert .126 < .112-section[:, 2].min() < .132
    front_ends = [path[0] for name, path, _ in rack["wires"]
                  if name.startswith("LongitudinalCradle")]
    assert len(front_ends) == 9
    assert all(abs(point[1]+.27212) < 1e-7 and abs(point[2]-.112) < 1e-7
               for point in front_ends)


def test_upper_tines_match_measured_grid_and_edge_datums(reconstructed_components):
    """52 base centers meet the agreed spacings without changing the wire rim."""
    rack = reconstructed_components["UpperRack"]
    teeth = [(name, path, radius) for name, path, radius in rack["wires"]
             if name.startswith("BowlComb") and "_Tooth" in name]
    assert len(teeth) == 52
    xs = np.unique([path[0, 0] for _, path, _ in teeth])
    assert np.allclose(xs, [-.1325, -.0375, .0375, .1325], atol=1e-9)
    assert np.allclose(np.diff(xs), [.095, .075, .095], atol=1e-9)
    lo, hi = _wire_family_bounds(rack, "TopRim")
    assert np.allclose([xs[0]-lo[0], hi[0]-xs[-1]], .1215, atol=1e-7)
    for x in xs:
        column = [path for _, path, _ in teeth if abs(path[0, 0]-x) < 1e-9]
        ys = np.sort([path[0, 1] for path in column])
        assert len(column) == 13
        assert np.allclose(np.diff(ys), .033, atol=1e-9)
        assert abs(hi[1]-ys[-1]-.0515) < 1e-7
        assert abs(ys[0]-lo[1]-.10114) < 1e-7
        assert all(np.allclose(path[-1]-path[0], [0., .008, .091], atol=1e-9)
                   for path in column)
    assert all(abs(radius-.0018) < 1e-9 for _, _, radius in teeth)


def test_upper_tine_rails_connect_to_the_contoured_floor(reconstructed_components):
    """Outer tine banks must meet their lower floor channels, with no hanging bases."""
    rack = reconstructed_components["UpperRack"]
    _, section, floor_radius = next(w for w in rack["wires"] if w[0].startswith("ContouredCrossU"))
    rails = [(name, path, radius) for name, path, radius in rack["wires"]
             if name.startswith("BowlComb") and name.endswith("_Base")]
    assert len(rails) == 4
    for name, rail, radius in rails:
        x, _, z = rail[0]
        floor_z = float(np.interp(x, section[:, 0], section[:, 2]))
        assert abs(z-floor_z-.003) < 1e-9
        assert 0 < z-floor_z < radius+floor_radius
        bank = name[:-len("_Base")]
        for tooth_name, tooth, _ in rack["wires"]:
            if tooth_name.startswith(bank+"_Tooth"):
                assert np.allclose(tooth[0, [0, 2]], rail[0, [0, 2]], atol=1e-9)
                assert rail[0, 1] <= tooth[0, 1] <= rail[-1, 1]


def test_deeper_lower_rack_clears_closed_door_and_has_flush_open_support(reconstructed_components):
    """A depth target is not useful if its rim jams the door or its rollers hit a step."""
    from dishsim_frigidaire.geometry import PARAMETERS

    cabinet, door, lower = (reconstructed_components[n] for n in ["Cabinet", "Door", "LowerRack"])
    door_origin = np.asarray(PARAMETERS["origins"]["Door"])
    lower_origin = np.asarray(PARAMETERS["origins"]["LowerRack"])
    solids = {s["name"]: s for s in door["solids"]}
    liner = solids["InnerLiner"]
    front_surface = door_origin[1]+liner["center"][1]+liner["size"][1]/2
    tub_back = next(s for s in cabinet["solids"] if s["name"] == "TubBack")
    back_surface = tub_back["center"][1]-tub_back["size"][1]/2
    rim_lo, rim_hi = _wire_family_bounds(lower, "UpperRim")
    assert lower_origin[1]+rim_lo[1]-front_surface > .002
    assert back_surface-lower_origin[1]-rim_hi[1] > .002
    tracks = [s for s in cabinet["solids"] if s["name"].startswith("LowerWheelTrack")]
    track_tops = [s["center"][2]+s["size"][2]/2 for s in tracks]
    for name in ["InnerLiner", "HingeBridge", "OpenDoorWheelRunway-1", "OpenDoorWheelRunway1"]:
        solid = solids[name]
        # +90 degrees about X maps the local +Y support face to world +Z.
        open_top = door_origin[2]+solid["center"][1]+solid["size"][1]/2
        assert np.allclose(track_tops, open_top, atol=.0001), name
    wheel_edge = max(abs(s["center"][0])+s["height"]/2 for s in lower["solids"]
                     if s["name"].startswith("Wheel_") and s["kind"] == "cylinder")
    side_rim = solids["InnerSideRim1"]
    assert side_rim["center"][0]-side_rim["size"][0]/2-wheel_edge > .002


def _point_segment_distance_2d(points, a, b):
    delta = b - a
    denominator = float(np.dot(delta, delta))
    if denominator < 1e-15:
        return np.linalg.norm(points - a, axis=1)
    t = np.clip((points - a) @ delta / denominator, 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * delta), axis=1)


def _projected_apertures(stage, body):
    """Project analytic colliders onto XY and test full-height vertical rays.

    Capsule projection is exact. Box/cylinder projections use their projected
    bounding rectangles, which are conservative for the small wheel cylinders.
    Thus an open ray reported here is genuinely open through the whole component.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    bounds = bbox.ComputeWorldBound(body).ComputeAlignedRange()
    lo, hi = np.asarray(bounds.GetMin()), np.asarray(bounds.GetMax())
    # Stay inside the rim. Offset sample counts avoid alignment with a regular
    # lattice; using unlike prime counts also detects repeated narrow floor gaps.
    xs = np.linspace(lo[0] + 0.18 * (hi[0] - lo[0]), hi[0] - 0.18 * (hi[0] - lo[0]), 43)
    ys = np.linspace(lo[1] + 0.18 * (hi[1] - lo[1]), hi[1] - 0.18 * (hi[1] - lo[1]), 47)
    points = np.array(np.meshgrid(xs, ys)).reshape(2, -1).T
    blocked = np.zeros(len(points), dtype=bool)
    unsupported = []
    count = 0
    probe_radius = 0.0005
    for prim in Usd.PrimRange(body):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        count += 1
        transform = cache.GetLocalToWorldTransform(prim)
        if prim.IsA(UsdGeom.Capsule):
            shape = UsdGeom.Capsule(prim)
            axis = {"X": 0, "Y": 1, "Z": 2}[str(shape.GetAxisAttr().Get())]
            a, b = np.zeros(3), np.zeros(3)
            a[axis], b[axis] = -shape.GetHeightAttr().Get() / 2, shape.GetHeightAttr().Get() / 2
            a = np.asarray(transform.Transform(Gf.Vec3d(*a)))
            b = np.asarray(transform.Transform(Gf.Vec3d(*b)))
            scale = max(np.linalg.norm(np.asarray(transform.ExtractRotationMatrix())[i]) for i in range(3))
            radius = shape.GetRadiusAttr().Get() * scale + probe_radius
            blocked |= _point_segment_distance_2d(points, a[:2], b[:2]) <= radius
        elif prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Sphere):
            box = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            bmin, bmax = np.asarray(box.GetMin())[:2], np.asarray(box.GetMax())[:2]
            blocked |= ((points >= bmin - probe_radius) & (points <= bmax + probe_radius)).all(axis=1)
        else:
            unsupported.append(str(prim.GetPath()))
    return {
        "collider_count": count,
        "unsupported_colliders": unsupported,
        "clear_ray_fraction": float((~blocked).mean()),
    }


def _distances_to_segments(points, segments):
    """Shortest point-to-polyline distances, independent of the author's simplifier."""
    points, segments = np.asarray(points), np.asarray(segments)
    a, delta = segments[:, 0], segments[:, 1] - segments[:, 0]
    denominator = np.einsum("ij,ij->i", delta, delta)
    denominator = np.maximum(denominator, 1e-20)
    distances = []
    for chunk in np.array_split(points, max(1, (len(points) + 127) // 128)):
        relative = chunk[:, None] - a
        t = np.clip(np.einsum("ijk,jk->ij", relative, delta) / denominator, 0., 1.)
        errors = np.linalg.norm(relative - t[:, :, None] * delta, axis=2)
        distances.extend(errors.min(axis=1))
    return np.asarray(distances)


def _inspect_loader(asset_dir):
    """Call the real loader against USD with lightweight Isaac API stand-ins.

    Kit runtime tests own physics integration. These stand-ins reproduce only
    documented parent-relative USD spawning and capture the authored initial
    states, allowing hierarchy/frame errors to be checked without booting Kit.
    """
    import math
    import types
    from unittest.mock import patch

    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from dishsim_frigidaire.asset import spawn

    active_stage = [None]

    class Config(types.SimpleNamespace):
        InitialStateCfg = types.SimpleNamespace

    class Asset:
        def __init__(self, cfg):
            self.cfg = cfg

    class FileConfig(Config):
        @staticmethod
        def func(prim_path, cfg, translation, orientation):
            prim = UsdGeom.Xform.Define(active_stage[0], prim_path)
            prim.GetPrim().GetReferences().AddReference(cfg.usd_path)
            prim.AddTranslateOp().Set(Gf.Vec3d(*translation))
            prim.AddOrientOp().Set(Gf.Quatf(orientation[0], Gf.Vec3f(*orientation[1:])))

    def module(name, **attrs):
        result = types.ModuleType(name)
        result.__dict__.update(attrs)
        return result

    usd_context = module("omni.usd", get_context=lambda: types.SimpleNamespace(get_stage=lambda: active_stage[0]))
    sim = module("isaaclab.sim", UsdFileCfg=FileConfig)
    actuators = module("isaaclab.actuators", ImplicitActuatorCfg=Config)
    assets = module("isaaclab.assets", Articulation=Asset, ArticulationCfg=Config,
                    RigidObject=Asset, RigidObjectCfg=Config)
    stubs = {"omni": module("omni", usd=usd_context), "omni.usd": usd_context,
             "isaaclab": module("isaaclab", sim=sim, actuators=actuators, assets=assets),
             "isaaclab.sim": sim, "isaaclab.actuators": actuators, "isaaclab.assets": assets}
    measured = {}
    with patch.dict(sys.modules, stubs):
        for name, path in [("world_parent", "/World/Appliance"),
                           ("rotated_parent", "/World/Room/Appliance"),
                           ("missing_intermediate_parent", "/World/Room/NewParent/Appliance")]:
            stage = active_stage[0] = Usd.Stage.CreateInMemory()
            UsdGeom.Xform.Define(stage, "/World")
            if name != "world_parent":
                room = UsdGeom.Xform.Define(stage, "/World/Room")
                room.AddTranslateOp().Set(Gf.Vec3d(2., -3., .4))
                room.AddRotateXYZOp().Set(Gf.Vec3f(15., -8., 37.))
            position, yaw = (1.23, -.87, .11), .61
            q = (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))
            expected = Gf.Matrix4d(1).SetRotate(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
            expected.SetTranslateOnly(Gf.Vec3d(*position))
            appliance, basket = spawn(path, position=position, yaw=yaw, usd_path=str(asset_dir / "fdpc4221as.usdc"))
            cache = UsdGeom.XformCache()
            cabinet_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path + "/Cabinet"))
            basket_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path + "/SilverwareBasket"))
            fixed = UsdPhysics.FixedJoint.Get(stage, path + "/Joints/base_fixed")
            weld = Gf.Matrix4d(1).SetRotate(Gf.Quatd(fixed.GetLocalRot0Attr().Get()))
            weld.SetTranslateOnly(Gf.Vec3d(fixed.GetLocalPos0Attr().Get()))
            init_q = basket.cfg.init_state.rot
            basket_initial = Gf.Matrix4d(1).SetRotate(Gf.Quatd(init_q[0], Gf.Vec3d(*init_q[1:])))
            basket_initial.SetTranslateOnly(Gf.Vec3d(*basket.cfg.init_state.pos))
            measured[name] = {
                "cabinet_world_matrix": float(np.abs(np.asarray(cabinet_world) - np.asarray(expected)).max()),
                "world_weld_matrix": float(np.abs(np.asarray(cabinet_world) - np.asarray(weld)).max()),
                "articulation_initial_position": float(np.linalg.norm(np.asarray(appliance.cfg.init_state.pos) - position)),
                "articulation_initial_rotation": float(np.linalg.norm(np.asarray(appliance.cfg.init_state.rot) - q)),
                "basket_initial_world_matrix": float(np.abs(np.asarray(basket_world) - np.asarray(basket_initial)).max()),
            }
        stage = active_stage[0] = Usd.Stage.CreateInMemory()
        parent = UsdGeom.Xform.Define(stage, "/World")
        parent.AddScaleOp().Set(Gf.Vec3f(2., 1., 1.))
        try:
            spawn("/World/Appliance", usd_path=str(asset_dir / "fdpc4221as.usdc"))
        except ValueError as error:
            rejected = "scale" in str(error)
        else:
            rejected = False
    return measured, rejected


def _inspect(directory):
    import math
    import shutil

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

    from dishsim_frigidaire.asset import build
    from dishsim_frigidaire.geometry import build_components

    asset_dir = directory / "asset"
    build(asset_dir)
    report = {"files": {}, "invalid_meshes": [], "unbound_meshes": [], "mass_properties": {}}
    for name in _FILES:
        filename = asset_dir / (name + (".usda" if name == "example_scene" else ".usdc"))
        stage = Usd.Stage.Open(str(filename))
        layers, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(filename)))
        report["files"][name] = {
            "default_prim": bool(stage.GetDefaultPrim()),
            "geometry_revision": stage.GetDefaultPrim().GetAttribute("geometryRevision").Get(),
            "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "unresolved_assets": list(unresolved),
            "geometry_count": sum(p.IsA(UsdGeom.Gprim) for p in stage.Traverse()),
            "absolute_references": [str(ref) for layer in layers for ref in layer.GetExternalReferences()
                                    if Path(ref).is_absolute() or "://" in ref],
        }
    stage = Usd.Stage.Open(str(asset_dir / "fdpc4221as.usdc"))
    report["root"] = str(stage.GetDefaultPrim().GetPath())
    cache = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get())
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        bad = not np.isfinite(points).all() or len(points) == 0 or (counts != 3).any()
        bad = bad or len(indices) != int(counts.sum()) or indices.min() < 0 or indices.max() >= len(points)
        if not bad:
            faces = indices.reshape(-1, 3)
            area2 = np.linalg.norm(np.cross(points[faces[:, 1]] - points[faces[:, 0]],
                                           points[faces[:, 2]] - points[faces[:, 0]]), axis=1)
            bad = bool((area2 <= 1e-14).any())
        if bad:
            report["invalid_meshes"].append(str(prim.GetPath()))
        if not UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
            report["unbound_meshes"].append(str(prim.GetPath()))
    for name in _BODIES:
        body = stage.GetPrimAtPath(_ROOT + "/" + name)
        mass = UsdPhysics.MassAPI(body)
        report["mass_properties"][name] = {
            "mass": mass.GetMassAttr().Get(), "inertia": list(mass.GetDiagonalInertiaAttr().Get()),
        }
    documented = json.loads((asset_dir / "parameters.json").read_text())
    report["documented_origin_error_m"] = max(
        float(np.linalg.norm(np.asarray(cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(_ROOT + "/" + name)).ExtractTranslation()) - np.asarray(position)))
        for name, position in documented["geometry"]["origins"].items()
    )
    cabinet = stage.GetPrimAtPath(_ROOT + "/Cabinet")
    cabinet_range = bbox.ComputeWorldBound(cabinet).ComputeAlignedRange()
    report["cabinet_dimensions"] = list(cabinet_range.GetSize())
    report["closed_assembly_dimensions"] = list(
        bbox.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange().GetSize())
    # Evaluate the specified fully-open depth without running physics or editing
    # the generated asset: only this fresh stage's anonymous session layer changes.
    open_stage = Usd.Stage.Open(stage.GetRootLayer())
    with Usd.EditContext(open_stage, open_stage.GetSessionLayer()):
        UsdGeom.Xformable(open_stage.GetPrimAtPath(_ROOT + "/Door")).AddRotateXOp().Set(90.)
    open_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], False, True)
    report["door_open_depth"] = float(
        open_bbox.ComputeWorldBound(open_stage.GetDefaultPrim()).ComputeAlignedRange().GetSize()[1])
    report["apertures"] = {
        name: _projected_apertures(stage, stage.GetPrimAtPath(_ROOT + "/" + name))
        for name in ["LowerRack", "UpperRack", "SilverwareBasket"]
    }

    # Compare each exported analytic wire with the source visual path. This
    # catches axis/quaternion errors as well as an overaggressive simplifier.
    centerline_error, radius_error, unmatched = 0., 0., []
    for name, component in build_components().items():
        source = {}
        for family, path, radius in component["wires"]:
            item = source.setdefault(family, {"segments": [], "radii": []})
            item["segments"].extend(zip(path[:-1], path[1:]))
            item["radii"].append(radius)
        authored = {}
        body = stage.GetPrimAtPath(_ROOT + "/" + name)
        body_inverse = cache.GetLocalToWorldTransform(body).GetInverse()
        for prim in Usd.PrimRange(body):
            family = prim.GetAttribute("wireFamily").Get()
            if not family:
                continue
            shape = UsdGeom.Capsule(prim)
            assert shape, str(prim.GetPath())
            transform = cache.GetLocalToWorldTransform(prim) * body_inverse
            axis = "XYZ".index(str(shape.GetAxisAttr().Get()))
            a, b = np.zeros(3), np.zeros(3)
            a[axis], b[axis] = -shape.GetHeightAttr().Get() / 2, shape.GetHeightAttr().Get() / 2
            a, b = [np.asarray(transform.Transform(Gf.Vec3d(*p))) for p in (a, b)]
            item = authored.setdefault(family, {"segments": [], "radii": []})
            item["segments"].append([a, b])
            item["radii"].append(shape.GetRadiusAttr().Get())
        unmatched.extend(name + "/" + key for key in set(source) ^ set(authored))
        for key in set(source) & set(authored):
            a, b = np.asarray(source[key]["segments"]), np.asarray(authored[key]["segments"])
            samples_a = np.concatenate([a[:, 0], a[:, 1], a.mean(axis=1)])
            samples_b = np.concatenate([b[:, 0], b[:, 1], b.mean(axis=1)])
            centerline_error = max(centerline_error, float(_distances_to_segments(samples_a, b).max()),
                                  float(_distances_to_segments(samples_b, a).max()))
            radius_error = max(radius_error, max(min(abs(r - q) for q in source[key]["radii"])
                                                 for r in authored[key]["radii"]))
    report["maximum_wire_centerline_error_m"] = centerline_error
    report["maximum_wire_radius_error_m"] = radius_error
    report["unmatched_wire_families"] = unmatched

    report["articulation_self_collisions"] = cabinet.GetAttribute("physxArticulation:enabledSelfCollisions").Get()
    report["basket_joint_count"] = 0
    report["basket_filtered_pairs"], report["door_rack_filtered_pairs"] = [], []
    report["joint_types"] = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            targets = joint.GetBody0Rel().GetTargets() + joint.GetBody1Rel().GetTargets()
            report["basket_joint_count"] += any(str(t).endswith("/SilverwareBasket") for t in targets)
            if prim.GetName() != "base_fixed":
                report["joint_types"][prim.GetName()] = prim.GetTypeName()
        for relationship in prim.GetRelationships():
            if "filteredPairs" not in relationship.GetName():
                continue
            for target in relationship.GetTargets():
                pair = [str(prim.GetPath()), str(target)]
                if any("/SilverwareBasket" in p for p in pair):
                    report["basket_filtered_pairs"].append(pair)
                if any("/Door" in p for p in pair) and any("Rack" in p for p in pair):
                    report["door_rack_filtered_pairs"].append(pair)

    # Test actual USD composition after copying the folder, independent of the
    # generator's original absolute location. Reference under a new prim name.
    relocated = directory / "relocated"
    shutil.copytree(asset_dir, relocated)
    placed = Usd.Stage.CreateInMemory()
    placed_root = UsdGeom.Xform.Define(placed, "/World/Appliance")
    placed_root.GetPrim().GetReferences().AddReference(str(relocated / "fdpc4221as.usdc"))
    position, yaw = (1.23, -.87, .11), .61
    placed_root.AddTranslateOp().Set(Gf.Vec3d(*position))
    quaternion = Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
    placed_root.AddOrientOp().Set(quaternion)
    weld = UsdPhysics.FixedJoint.Get(placed, "/World/Appliance/Joints/base_fixed")
    weld.CreateLocalPos0Attr(Gf.Vec3f(*position))
    weld.CreateLocalRot0Attr(quaternion)
    report["relocated_body_count"] = sum(bool(placed.GetPrimAtPath("/World/Appliance/" + n)) for n in _BODIES)
    placed_cache = UsdGeom.XformCache()
    transform = placed_cache.GetLocalToWorldTransform(placed_root.GetPrim())
    report["spawned_origin"] = list(transform.Transform(Gf.Vec3d(0)))
    expected_forward = np.array([math.sin(yaw), -math.cos(yaw), 0.])
    report["spawned_forward_error"] = float(np.linalg.norm(
        np.asarray(transform.TransformDir(Gf.Vec3d(0, -1, 0))) - expected_forward))
    anchor_error, frame_angle_error = 0., 0.
    for checked_stage in (stage, placed):
        checked_cache = UsdGeom.XformCache()
        for prim in checked_stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            frames = []
            for side in (0, 1):
                target = getattr(joint, f"GetBody{side}Rel")().GetTargets()
                body_world = (checked_cache.GetLocalToWorldTransform(checked_stage.GetPrimAtPath(target[0]))
                              if target else Gf.Matrix4d(1))
                local_pos = getattr(joint, f"GetLocalPos{side}Attr")().Get()
                local_rot = getattr(joint, f"GetLocalRot{side}Attr")().Get()
                local = Gf.Matrix4d(1)
                local.SetRotate(Gf.Quatd(local_rot))
                local.SetTranslateOnly(Gf.Vec3d(local_pos))
                frames.append(local * body_world)
            anchor_error = max(anchor_error, (frames[0].ExtractTranslation() - frames[1].ExtractTranslation()).GetLength())
            delta = frames[0].ExtractRotation().GetQuat().GetInverse() * frames[1].ExtractRotation().GetQuat()
            frame_angle_error = max(frame_angle_error, Gf.Rotation(delta).GetAngle())
    report["maximum_anchor_error_m"] = anchor_error
    report["maximum_frame_angle_error_deg"] = frame_angle_error
    report["loader_world_poses"], report["scaled_parent_rejected"] = _inspect_loader(asset_dir)
    (directory / "inspection.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ["closed_assembly_dimensions", "door_open_depth",
                     "apertures", "maximum_wire_centerline_error_m", "maximum_anchor_error_m"]}))


if __name__ == "__main__":
    _inspect(Path(sys.argv[1]))
