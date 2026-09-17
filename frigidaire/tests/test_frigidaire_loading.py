"""Fail-closed input integrity and support accounting, without launching Isaac."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile

import pytest

from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.load_validation import (
    externally_supported_indices, pose_motion_metrics, usdc_hashes, validate_manifest,
)
from dishsim_frigidaire.tableware import CATALOG


@pytest.fixture
def inputs():
    output = Path(__file__).resolve().parents[2] / "build/frigidaire_tests"
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="frigidaire_input_test_", dir=output) as directory:
        directory = Path(directory)
        usd = directory / "fdpc4221as.usdc"
        usd.write_bytes(b"input hash fixture; USD shape validation is tested separately")
        tableware = directory / "tableware"
        tableware.mkdir()
        catalog = deepcopy(CATALOG)
        written = deepcopy(catalog)
        for kind in catalog:
            data = kind.encode()
            (tableware / f"{kind}.usdc").write_bytes(data)
            written[kind]["sha256"] = hashlib.sha256(data).hexdigest()
        (tableware / "catalog.json").write_text(json.dumps({"items": written}))
        manifest = {"schema_version": 1, "asset_dir": "/an/original/path/that/need/not/exist",
                    "geometry_hashes": usdc_hashes(directory), "catalog": catalog,
                    "body_positions_m": {k: list(v) for k, v in BODY_POSITIONS.items()},
                    "objects": [{"id": "bowl_001", "kind": "bowl", "rack": "LowerRack",
                                 "position": [.15, -.14, .02], "quaternion_xyzw": [0., 0., 0., 1.]}],
                    "counts": {"bowl": 1}}
        yield manifest, usd


def test_portable_manifest_validates_without_following_recorded_source_path(inputs):
    manifest, usd = inputs
    manifest["objects"][0]["release_hover_m"] = 0.
    result = validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)
    assert result["object_count"] == 1
    assert result["counts"] == {"bowl": 1}


@pytest.mark.parametrize("change", ["modified", "missing", "added", "untrusted_key"])
def test_geometry_hash_map_is_complete_and_fail_closed(inputs, change):
    manifest, usd = inputs
    if change == "modified":
        usd.write_bytes(b"changed geometry")
    elif change == "missing":
        (usd.parent / "tableware/mug.usdc").unlink()
    elif change == "added":
        (usd.parent / "new_component.usdc").write_bytes(b"new collider")
    else:
        manifest["geometry_hashes"]["../outside_project.usdc"] = "0"*64
    with pytest.raises(ValueError, match="Stale manifest geometry_hashes"):
        validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)


@pytest.mark.parametrize("change, expected", [
    ("count", "counts disagree"), ("duplicate", "unique"), ("kind", "Unknown tableware"),
    ("rack", "Incorrect supporting rack"), ("nan", "nonfinite"),
    ("quaternion", "normalized"), ("id", "USD identifier"),
    ("catalog", "catalog differs"), ("frame", "Stale manifest supporting frame"),
    ("hover_negative", "release_hover_m"), ("hover_infinite", "release_hover_m"),
])
def test_invalid_simulation_inputs_fail_before_spawn(inputs, change, expected):
    manifest, usd = inputs
    entry = manifest["objects"][0]
    if change == "count":
        manifest["counts"]["bowl"] = 6
    elif change == "duplicate":
        manifest["objects"].append(deepcopy(entry))
    elif change == "kind":
        entry["kind"] = "unmodeled_serving_platter"
    elif change == "rack":
        entry["rack"] = "SilverwareBasket"
    elif change == "nan":
        entry["position"][2] = float("nan")
    elif change == "quaternion":
        entry["quaternion_xyzw"] = [0, 0, 0, 0]
    elif change == "id":
        entry["id"] = "../another_prim"
    elif change == "catalog":
        manifest["catalog"]["bowl"]["diameter_m"] = .10
    elif change == "frame":
        manifest["body_positions_m"]["UpperRack"][2] += .01
    elif change == "hover_negative":
        entry["release_hover_m"] = -.001
    elif change == "hover_infinite":
        entry["release_hover_m"] = float("inf")
    with pytest.raises(ValueError, match=expected):
        validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)


def test_bowl_is_accepted_on_either_rack_but_nowhere_else(inputs):
    manifest, usd = inputs
    manifest["objects"][0]["rack"] = "UpperRack"
    assert validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)["counts"] == {"bowl": 1}
    manifest["objects"][0]["rack"] = "SilverwareBasket"
    with pytest.raises(ValueError, match="Incorrect supporting rack"):
        validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)
    # single-rack kinds stay single-rack
    manifest["objects"][0].update(kind="tumbler", rack="LowerRack")
    manifest["counts"] = {"tumbler": 1}
    with pytest.raises(ValueError, match="Incorrect supporting rack"):
        validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)


def test_tableware_catalog_cannot_claim_another_size_or_asset(inputs):
    manifest, usd = inputs
    path = usd.parent / "tableware/catalog.json"
    authored = json.loads(path.read_text())
    authored["items"]["bowl"]["mass_kg"] = .001
    path.write_text(json.dumps(authored))
    with pytest.raises(ValueError, match="Authored tableware catalog disagrees"):
        validate_manifest(manifest, usd, CATALOG, BODY_POSITIONS)


def test_self_contact_does_not_establish_support():
    assert externally_supported_indices([(5, 5), (6, 6)]) == set()
    assert externally_supported_indices([(5, 5), (5, 2), (7, 6), (6, 7)]) == {2, 5, 6, 7}


def test_unsmoothed_speed_catches_brief_motion_inside_span_limit():
    result = pose_motion_metrics([[0, 0, 0], [.001, 0, 0], [0, 0, 0]], [[1, 0, 0, 0]]*3,
                                 [[-.1, 0, 0], [.1, 0, 0]], 1/120)
    assert result["root_position_span_m"] == pytest.approx(.001)
    assert result["peak_mesh_point_speed_m_s"] == pytest.approx(.12)


def test_mesh_speed_catches_rotation_about_a_stationary_actor_origin():
    import math
    result = pose_motion_metrics([[0, 0, 0], [0, 0, 0]],
                                 [[1, 0, 0, 0], [math.cos(.025), 0, 0, math.sin(.025)]],
                                 [[.2, 0, 0], [-.2, 0, 0]], .1)
    assert result["root_position_span_m"] == 0
    assert result["peak_mesh_point_speed_m_s"] == pytest.approx(4*math.sin(.025))
    assert result["quaternion_span_deg"] == pytest.approx(math.degrees(.05))


def test_quaternion_sign_change_is_not_motion():
    result = pose_motion_metrics([[0, 0, 0], [0, 0, 0]], [[1, 0, 0, 0], [-1, 0, 0, 0]],
                                 [[.2, 0, 0], [-.2, 0, 0]], 1/120)
    assert result["peak_mesh_point_speed_m_s"] == 0
    assert result["quaternion_span_deg"] == 0
