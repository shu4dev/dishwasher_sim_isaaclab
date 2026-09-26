# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""HOTEC wheat-straw generator: property checks, Kit-free (no Kit, renderer, or GPU).

The nominal sizes are restated here from the brief, independently of the generator table, so an
accidental change of the targets fails loudly. No byte digests are pinned: the estimated
parameters are expected to change once the user measures the real pieces.
"""

import json
from pathlib import Path
import tempfile

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from dishsim import hotec_gen as h

ROOT = Path(__file__).resolve().parents[1]
NOMINAL = {"plate": (.2286, .0400), "bowl": (.1480, .0750), "cup": (.0711, .1049)}   # the brief's literals
ADVERTISED_ML = {"bowl": 769., "cup": 355.}


@pytest.fixture(scope="module")
def built():
    table = h.resolve()
    return {kind: h.evaluate(kind, table) for kind in h.KINDS}, table


def _contains(pieces, point):
    point = np.asarray(point, dtype=float)
    for vertices, _ in pieces:
        equations = ConvexHull(vertices).equations
        if np.all(equations[:, :3] @ point + equations[:, 3] <= 1e-10):
            return True
    return False


@pytest.mark.parametrize("kind", h.KINDS)
def test_outer_dimensions_and_base_origin(kind, built):
    _, geom, _, checks = built[0][kind]
    p = geom["visual"][0]
    diameter, height = NOMINAL[kind]
    assert np.allclose(np.ptp(p, axis=0), (diameter, diameter, height), atol=5e-4)
    assert abs(p[:, 2].min()) < 1e-9 and abs(p[:, 2].max() - height) < 1e-9
    n_rows = len(geom["dense"])
    rim = p[1 + (n_rows - 2) * h.RING: 1 + (n_rows - 1) * h.RING]
    assert np.allclose(rim[:, :2].mean(axis=0), 0., atol=1e-9)
    assert checks["dimensions"]["ok"]


@pytest.mark.parametrize("kind", h.KINDS)
def test_visual_watertight_and_oriented(kind, built):
    _, geom, _, checks = built[0][kind]
    p, n, f = geom["visual"]
    assert checks["mesh"]["closed_and_oriented"]
    assert np.allclose(np.linalg.norm(n, axis=1), 1.)
    assert checks["mesh"]["min_face_area_m2"] > 1e-14
    assert h.signed_volume(p, f) > 0
    assert len(p) == 2 + 2 * (len(geom["dense"]) - 1) * h.RING


def test_decoration_variants_stay_watertight():
    table = h.resolve({"cup.flutes.count": 0, "cup.rings.heights_m": [.030, .050, .070], "bowl.grooves.heights_m": []})
    for kind in ("cup", "bowl"):
        geom = h.geometry(kind, h.values(table, kind))
        assert h.closed_and_oriented(geom["visual"][2])
        assert not geom["fluted"] if kind == "cup" else True


@pytest.mark.parametrize("kind", h.KINDS)
def test_profile_simple_and_walls(kind, built):
    v, _, _, checks = built[0][kind]
    profile = checks["profile"]
    assert profile["simple_polygon"] and profile["inner_z_monotone"]
    assert profile["min_wall_m"] >= .6 * v["wall_m"]
    assert profile["rim_flat_m"] >= .0006


def test_cup_flutes_are_outer_only(built):
    _, geom, _, _ = built[0]["cup"]
    p, n_rows = geom["visual"][0], len(geom["dense"])
    inner_rings, _ = h.inner_ring_indices(n_rows)
    for ring in inner_rings:
        assert np.hypot(p[list(ring), 0], p[list(ring), 1]).std() < 1e-9
    outer_std = [np.hypot(p[1 + (r - 1) * h.RING: 1 + r * h.RING, 0], p[1 + (r - 1) * h.RING: 1 + r * h.RING, 1]).std()
                 for r in range(1, n_rows)]
    assert max(outer_std) > 1e-4                  # some ring is faceted
    assert outer_std[-1] < 1e-9 and outer_std[0] < 1e-9   # rim and base rings are round


@pytest.mark.parametrize("kind", h.KINDS)
def test_collision_open_cavity_and_solid_foot(kind, built):
    v, geom, _, checks = built[0][kind]
    c = checks["collision"]
    assert c["cavity_probes_open"] and c["foot_solid"] and c["floor_solid"] and c["no_protrusion"]
    assert c["max_cavity_intrusion_m"] <= .001
    assert c["pieces"] == geom["bands"] * geom["sectors"] and 150 <= c["pieces"] <= 220
    assert c["min_piece_volume_m3"] > 1e-12
    empty = {"plate": (0., 0., .020), "bowl": (0., 0., .040), "cup": (0., 0., .050)}[kind]
    assert not _contains(geom["pieces"], empty)
    assert _contains(geom["pieces"], (0., 0., v["foot_height_m"] + v["floor_m"] / 2))


@pytest.mark.parametrize("kind", ("bowl", "cup"))
def test_delivered_mesh_is_not_capped(kind, built):
    v, geom, cap, _ = built[0][kind]
    p, _, f = geom["visual"]
    top = cap["rim_z_m"]
    inner_rim = v["rim_diameter_m"] / 2 - v["wall_m"]
    for tri in f:
        if np.all(np.abs(p[tri, 2] - top) < 1e-6):
            assert np.hypot(p[tri, 0], p[tri, 1]).min() >= inner_rim - 1e-6   # only the rim flat lies in the rim plane


def test_capacity_bands_and_cross_checks(built):
    for kind, (lo, hi) in {"bowl": (600., 850.), "cup": (200., 330.)}.items():
        _, geom, cap, _ = built[0][kind]
        assert lo <= cap["brimful_ml"] <= hi
        assert cap["headspace_ml"] < cap["brimful_ml"]
        assert .998 <= cap["mesh_over_polyline"] <= 1.0005
        assert abs(cap["advertised_ml"] - ADVERTISED_ML[kind]) < 1.
        dense = geom["dense"][geom["dense"][:, 3] <= cap["rim_z_m"] + 1e-12]
        trapz = np.trapz(np.pi * dense[:, 2] ** 2, dense[:, 3]) * 1e6
        assert abs(trapz - cap["brimful_ml"]) / cap["brimful_ml"] < .002
    assert built[0]["plate"][2] is None


def test_parameter_tags_and_hash():
    table = h.resolve()
    for kind, entries in table.items():
        for name, entry in entries.items():
            assert entry["status"] in h.STATUSES, (kind, name)
        diameter, height = NOMINAL[kind]
        assert entries["rim_diameter_m"]["value"] == diameter and entries["rim_diameter_m"]["status"] == "nominal"
        assert entries["height_m"]["value"] == height and entries["height_m"]["status"] == "nominal"
    assert h.parameters_sha256(table) == h.parameters_sha256(h.resolve())
    changed = h.resolve({"bowl.wall_m": .0032})
    assert changed["bowl"]["wall_m"]["status"] == "measured"
    assert h.parameters_sha256(changed) != h.parameters_sha256(table)
    assert set(json.loads(json.dumps(h.sensitivity("cup", table)))[0]) >= {"wall_m", "base_diameter_m", "brimful_ml"}


def test_usd_round_trip(built):
    if not Path("/isaac-sim").exists():
        pytest.skip("USD round trip runs inside the Isaac container (pxr via scripts/run_py.sh)")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade   # must import there; a failure is a real failure
    _, table = built
    scratch = ROOT / "build/hotec_tests"
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pytest_", dir=scratch) as temporary:
        catalog, parameters = h.build(temporary, figures=False)
        assert catalog["parameters_sha256"] == parameters["parameters_sha256"] == h.parameters_sha256(table)
        for kind in h.KINDS:
            path = Path(temporary) / f"{kind}.usda"
            assert catalog["items"][kind]["sha256"] == h._sha256(path)
            stage = Usd.Stage.Open(str(path))
            root = stage.GetDefaultPrim()
            assert str(root.GetPath()) == f"/{h.ROOT_PRIMS[kind]}"
            assert UsdGeom.GetStageMetersPerUnit(stage) == 1. and UsdGeom.GetStageUpAxis(stage) == "Z"
            assert list(stage.GetRootLayer().GetExternalReferences()) == []
            data = stage.GetRootLayer().customLayerData
            assert data["hotec_parameters_sha256"] == catalog["parameters_sha256"]
            assert json.loads(data["hotec_parameters_json"]) == table[kind]
            assert root.HasAPI(UsdPhysics.RigidBodyAPI)
            listop = stage.GetRootLayer().GetPrimAtPath(root.GetPath()).GetInfo("apiSchemas")
            assert "PhysxRigidBodyAPI" in list(listop.prependedItems)
            assert root.GetAttribute("physxRigidBody:enableCCD").Get() is True
            assert (root.GetAttribute("physxRigidBody:solverPositionIterationCount").Get(),
                    root.GetAttribute("physxRigidBody:solverVelocityIterationCount").Get()) == (16, 4)
            colliders, visuals = 0, 0
            for prim in Usd.PrimRange(root):                     # v1-style build: mass_kg unset -> nothing mass-related
                assert not prim.HasAPI(UsdPhysics.MassAPI), prim.GetPath()
                for attr in prim.GetAuthoredAttributes():        # builtin (unauthored) schema attrs are fine
                    assert attr.GetName() not in ("physics:mass", "physics:density", "physics:centerOfMass",
                                                  "physics:diagonalInertia"), prim.GetPath()
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    colliders += 1
                    assert prim.IsA(UsdGeom.Mesh)
                    assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull"
                    assert "PhysxCollisionAPI" in list(stage.GetRootLayer().GetPrimAtPath(prim.GetPath()).GetInfo("apiSchemas").prependedItems)
                    assert prim.GetAttribute("physxCollision:contactOffset").Get() == pytest.approx(.0005)
                    assert UsdShade.MaterialBindingAPI(prim).GetDirectBinding("physics").GetMaterial()
                elif prim.IsA(UsdGeom.Mesh):
                    visuals += 1
                    assert UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            assert colliders == catalog["items"][kind]["convex_colliders"] and visuals == 1
            assert UsdGeom.Imageable(stage.GetPrimAtPath(root.GetPath().AppendChild("Collisions"))).GetVisibilityAttr().Get() == "invisible"
            vset = root.GetVariantSets().GetVariantSet("color")
            assert set(vset.GetVariantNames()) == set(h.COLOURS) and vset.GetVariantSelection() == h.DEFAULT_COLOUR
            shader = UsdShade.Shader(stage.GetPrimAtPath(root.GetPath().AppendPath("Looks/Surface/Shader")))
            seen = set()
            for name in h.COLOURS:
                vset.SetVariantSelection(name)
                seen.add(tuple(round(c, 3) for c in shader.GetInput("diffuseColor").Get()))
            assert len(seen) == len(h.COLOURS)
            bounds = h.usd_bounds(path)
            diameter, height = NOMINAL[kind]
            assert np.allclose(bounds["size_m"], (diameter, diameter, height), atol=5e-4)


def test_measured_mass_is_authored_on_the_root_only(tmp_path):
    """--set kind.mass_kg authors physics:mass on the default prim (tagged measured) and nothing else."""
    pytest.importorskip("pxr")
    from pxr import Usd, UsdPhysics
    masses = {"plate": .094625, "bowl": .067375, "cup": .0595}          # 757 / 539 / 476 g over 8 pieces
    catalog, parameters = h.build(tmp_path, overrides={f"{k}.mass_kg": v for k, v in masses.items()}, figures=False)
    for kind, mass in masses.items():
        entry = parameters["kinds"][kind]["mass_kg"]
        assert entry["value"] == mass and entry["status"] == "measured"
        assert catalog["items"][kind]["mass_kg"] == mass
        assert catalog["items"][kind]["mass_status"].startswith("measured: physics:mass = ")
        stage = Usd.Stage.Open(str(tmp_path / f"{kind}.usda"))
        root = stage.GetDefaultPrim()
        assert root.HasAPI(UsdPhysics.MassAPI)
        assert root.GetAttribute("physics:mass").Get() == pytest.approx(mass)
        assert stage.GetRootLayer().customLayerData["mass_status"].startswith("measured: physics:mass = ")
        for prim in Usd.PrimRange(root):
            if prim != root:
                assert not prim.HasAPI(UsdPhysics.MassAPI), prim.GetPath()
            for attr in prim.GetAuthoredAttributes():
                assert attr.GetName() not in ("physics:density", "physics:centerOfMass", "physics:diagonalInertia"), prim.GetPath()
    assert "Masses are authored as physics:mass" in (tmp_path / "README.md").read_text()

