"""Independent shape and USD checks for the full-size Frigidaire tableware.

The specified dimensions and open-space probes intentionally do not derive from
generator parameters: these catch accidental shrinking and filled collision hulls.
No Kit application, renderer, or GPU is required.
"""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from dishsim_frigidaire.tableware import CATALOG, tableware_geometry


_EXPECTED = {
    "dinner_plate": ([.260, .260, .020], .65),
    "salad_plate": ([.205, .205, .018], .40),
    "saucer": ([.150, .150, .018], .20),
    "bowl": ([.140, .140, .065], .35),
    "mug": ([.120, .085, .100], .30),
    "tumbler": ([.080, .080, .160], .25),
    "fork": ([.025, .012, .195], .045),
    "knife": ([.018, .006, .215], .055),
    "tablespoon": ([.040, .018, .190], .050),
    "teaspoon": ([.030, .012, .155], .025),
}


@pytest.fixture(scope="module")
def geometry():
    assert set(CATALOG) == set(_EXPECTED)
    return {kind: tableware_geometry(kind) for kind in _EXPECTED}


def _closed_and_oriented(faces):
    undirected, directed = Counter(), Counter()
    for face in faces:
        for a, b in zip(face, np.roll(face, -1)):
            undirected[tuple(sorted((int(a), int(b))))] += 1
            directed[(int(a), int(b))] += 1
    return (all(count == 2 for count in undirected.values()) and
            all(count == directed[(b, a)] for (a, b), count in directed.items()))


def _contains(geometry, point):
    point = np.asarray(point, dtype=float)
    for vertices, _ in geometry["convexes"]:
        equations = ConvexHull(vertices).equations
        if np.all(equations[:, :3] @ point + equations[:, 3] <= 1e-10):
            return True
    for a, b, radius in geometry["capsules"]:
        delta = b-a
        closest = a+np.clip(np.dot(point-a, delta)/np.dot(delta, delta), 0, 1)*delta
        if np.linalg.norm(point-closest) <= radius:
            return True
    return False


@pytest.mark.parametrize("kind", _EXPECTED)
def test_full_size_dimensions_and_closed_geometry(kind, geometry):
    g = geometry[kind]
    expected_size, expected_mass = _EXPECTED[kind]
    points = np.concatenate([v[0] for v in g["visuals"]])
    assert np.allclose(np.ptp(points, axis=0), expected_size, atol=1e-7), kind
    assert CATALOG[kind]["mass_kg"] == expected_mass
    for vertices, normals, faces in g["visuals"]:
        assert np.isfinite(vertices).all() and np.isfinite(normals).all()
        assert np.allclose(np.linalg.norm(normals, axis=1), 1., atol=1e-6)
        assert _closed_and_oriented(faces), kind
        area = np.cross(vertices[faces[:, 1]]-vertices[faces[:, 0]],
                        vertices[faces[:, 2]]-vertices[faces[:, 0]])
        assert np.all(np.linalg.norm(area, axis=1) > 1e-14), kind
    for vertices, faces in g["convexes"]:
        assert _closed_and_oriented(faces), kind
        assert ConvexHull(vertices).volume > 1e-12, kind


@pytest.mark.parametrize("kind, empty, support", [
    ("bowl", [0, 0, .030], [0, 0, .003]),
    ("mug", [0, 0, 0], [0, 0, -.047]),
    ("tumbler", [0, 0, 0], [0, 0, -.078]),
    ("dinner_plate", [0, 0, .004], [0, 0, -.005]),
    ("salad_plate", [0, 0, .004], [0, 0, -.0045]),
    ("saucer", [0, 0, .004], [0, 0, -.0045]),
    ("tablespoon", [0, .006, .065], [0, -.008, .065]),
    ("teaspoon", [0, .004, .0555], [0, -.005, .0555]),
])
def test_shells_have_open_cavities_and_real_support(kind, empty, support, geometry):
    assert not _contains(geometry[kind], empty), f"{kind} has a filled cavity"
    assert _contains(geometry[kind], support), f"{kind} has no supporting shell"


def test_mug_handle_and_fork_tine_gaps_remain_open(geometry):
    # Handle aperture is large enough to remain a meaningful manipulation feature.
    for z in (-.012, 0., .012):
        assert not _contains(geometry["mug"], [.057, 0, z])
    assert _contains(geometry["mug"], [.073, 0, 0])
    # The central fork slot must not become one solid convex head.
    for z in (.082, .087, .092):
        assert not _contains(geometry["fork"], [0, .002, z])
    assert _contains(geometry["fork"], [.0034666667, .0022, .085])


@pytest.fixture(scope="module")
def serialized():
    from dishsim_frigidaire.usd_bootstrap import bundled_usd_environment
    scratch = Path(__file__).resolve().parents[2]/"build/frigidaire_tests/tableware_check"
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pytest_", dir=scratch) as temporary:
        run = subprocess.run([sys.executable, str(Path(__file__).resolve()), temporary],
                             env=bundled_usd_environment(), capture_output=True,
                             text=True, timeout=60, check=False)
        assert run.returncode == 0, run.stdout+run.stderr
        return json.loads((Path(temporary)/"inspection.json").read_text())


def test_usds_preserve_dimensions_materials_physics_and_portability(serialized):
    assert set(serialized) == set(_EXPECTED)
    for kind, record in serialized.items():
        dimensions, mass = _EXPECTED[kind]
        assert record["default_prim"] == "/Tableware"
        assert record["units"] == 1. and record["up_axis"] == "Z"
        assert record["mass"] == pytest.approx(mass)
        assert np.allclose(record["actual_size_m"], dimensions, atol=1e-6)
        assert record["solver_iterations"] == [16, 4] and record["ccd"]
        assert record["invalid_colliders"] == []
        assert record["unbound_prims"] == []
        assert record["external_dependencies"] == []
        inertia = np.asarray(record["inertia"])
        assert (inertia > 0).all() and 2*inertia.max() <= inertia.sum()+1e-8


def _inspect(directory):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    from dishsim_frigidaire.tableware import write_tableware
    report = write_tableware(directory)
    inspected = {}
    for kind, info in report["items"].items():
        stage = Usd.Stage.Open(str(directory/info["usd"]))
        root = stage.GetDefaultPrim()
        mass = UsdPhysics.MassAPI(root)
        invalid, unbound = [], []
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if prim.IsA(UsdGeom.Mesh):
                    if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() != "convexHull":
                        invalid.append(str(prim.GetPath()))
                elif not prim.IsA(UsdGeom.Capsule):
                    invalid.append(str(prim.GetPath()))
                material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")[0]
                if not material:
                    unbound.append(str(prim.GetPath()))
            elif prim.IsA(UsdGeom.Mesh):
                if not UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]:
                    unbound.append(str(prim.GetPath()))
        inspected[kind] = {"default_prim": str(root.GetPath()),
                           "units": UsdGeom.GetStageMetersPerUnit(stage),
                           "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
                           "mass": mass.GetMassAttr().Get(),
                           "inertia": list(mass.GetDiagonalInertiaAttr().Get()),
                           "actual_size_m": info["actual_size_m"],
                           "solver_iterations": [root.GetAttribute("physxRigidBody:solverPositionIterationCount").Get(),
                                                 root.GetAttribute("physxRigidBody:solverVelocityIterationCount").Get()],
                           "ccd": root.GetAttribute("physxRigidBody:enableCCD").Get(),
                           "invalid_colliders": invalid, "unbound_prims": unbound,
                           "external_dependencies": list(stage.GetRootLayer().GetExternalReferences())}
    (directory/"inspection.json").write_text(json.dumps(inspected, indent=2)+"\n")


if __name__ == "__main__":
    _inspect(Path(sys.argv[1]))
