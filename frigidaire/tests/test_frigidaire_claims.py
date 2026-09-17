"""Claim-layout contracts without USD, FCL, or Isaac: counts, ordering, manifest schema."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from dishsim_frigidaire import claims, loading
from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.load_validation import validate_manifest
from dishsim_frigidaire.tableware import CATALOG


def _corners(size):
    hx, hy, hz = (s / 2 for s in size)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _lathe(size, base_radius):
    """Rim ring at the top and base ring at z=0, like the authored bowl (bottom z=0)."""
    angles = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    rim = np.stack([size[0] / 2 * np.cos(angles), size[1] / 2 * np.sin(angles), np.full(24, size[2])], 1)
    base = np.stack([base_radius * np.cos(angles), base_radius * np.sin(angles), np.zeros(24)], 1)
    return np.vstack([rim, base])


@pytest.fixture
def world():
    """Everything is collision-free; poses come out of the real builders."""
    points = {kind: _corners(spec["size_m"]) for kind, spec in CATALOG.items()}
    points["bowl"] = _lathe(CATALOG["bowl"]["size_m"], .028)
    return SimpleNamespace(points=points, collides=lambda objects: False,
                           candidate_objects=lambda c: [], add=lambda c: None,
                           reset_load=lambda entries: None)


@pytest.fixture
def asset_dir():
    output = Path(__file__).resolve().parents[2] / "build/frigidaire_tests"
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="frigidaire_claims_test_", dir=output) as directory:
        directory = Path(directory)
        (directory / "fdpc4221as.usdc").write_bytes(b"appliance fixture")
        tableware = directory / "tableware"
        tableware.mkdir()
        written = deepcopy(CATALOG)
        for kind in CATALOG:
            data = kind.encode()
            (tableware / f"{kind}.usdc").write_bytes(data)
            written[kind]["sha256"] = hashlib.sha256(data).hexdigest()
        (tableware / "catalog.json").write_text(json.dumps({"items": written}))
        yield directory


def _generate(asset_dir, world, variant):
    with patch.object(loading, "quaternion_xyzw", return_value=[0., 0., 0., 1.]):
        return claims.generate(asset_dir, variant, world=world)


def _row(manifest, bank):
    entries = [o for o in manifest["objects"] if o["slot"].startswith(f"lower_{bank}_")]
    return [o["kind"] for o in sorted(entries, key=lambda o: o["position"][0])]


@pytest.mark.parametrize("variant", claims.VARIANTS)
def test_lower_rows_read_left_to_right_as_claimed(asset_dir, world, variant):
    manifest = _generate(asset_dir, world, variant)
    assert manifest["packing"]["geometry_result"] == "PASS"
    assert _row(manifest, "front") == list(claims.LOWER_FRONT[variant])
    assert _row(manifest, "rear") == list(claims.LOWER_REAR)
    lower_bowls = [o for o in manifest["objects"] if o["kind"] == "bowl" and o["rack"] == "LowerRack"]
    assert len(lower_bowls) == claims.LOWER_BOWLS[variant]
    for bowl in lower_bowls:                       # ahead of the basket, right of the plates
        assert bowl["position"][1] < claims.BASKET_FRONT_Y
        assert bowl["position"][0] > max(o["position"][0] for o in manifest["objects"]
                                         if o["slot"].startswith("lower_front_"))
    if lower_bowls:
        assert lower_bowls[1]["position"][0] > lower_bowls[0]["position"][0]
        assert lower_bowls[0]["slot"] == "bowl_front_0" and lower_bowls[1]["slot"] == "bowl_front_1"


@pytest.mark.parametrize("variant", claims.VARIANTS)
def test_upper_and_basket_claims(asset_dir, world, variant):
    manifest = _generate(asset_dir, world, variant)
    upper = [o for o in manifest["objects"] if o["rack"] == "UpperRack"]
    tumblers = [o for o in upper if o["kind"] == "tumbler"]
    assert len(tumblers) == 12
    assert sum(o["position"][0] < 0 for o in tumblers) == 6
    assert all(abs(o["position"][0]) > .150 for o in tumblers)       # outer channels
    saucers = sorted((o for o in upper if o["kind"] == "saucer"), key=lambda o: o["position"][1])
    assert [o["slot"] for o in saucers] == ["saucer_00", "saucer_01"]
    bowls = [o for o in upper if o["kind"] == "bowl"]
    assert all(o["position"][0] == 0. for o in saucers + bowls)      # centre gap
    assert min(o["position"][1] for o in bowls) > max(o["position"][1] for o in saucers)
    assert manifest["packing"]["upper_bowl_fill"] == len(bowls) >= 1
    assert not any(o["kind"] == "mug" for o in manifest["objects"])
    basket = [o for o in manifest["objects"] if o["rack"] == "SilverwareBasket"]
    assert [o["candidate_key"] for o in basket] == list(claims.BASKET_KEYS)
    assert manifest["packing"]["by_rack"]["SilverwareBasket"] == {k: 4 for k in loading.CUTLERY}


def test_manifest_matches_the_simulation_input_contract(asset_dir, world):
    manifest = _generate(asset_dir, world, "A")
    manifest = json.loads(json.dumps(manifest))                       # what the script writes
    result = validate_manifest(manifest, asset_dir / "fdpc4221as.usdc", CATALOG, BODY_POSITIONS)
    assert result["counts"] == manifest["counts"]
    assert result["counts"]["dinner_plate"] == 12 and result["counts"]["salad_plate"] == 7
    assert result["counts"]["bowl"] == 2 + manifest["packing"]["upper_bowl_fill"]
    ids = [o["id"] for o in manifest["objects"]]
    assert len(ids) == len(set(ids))


def test_collisions_become_unplaced_items_not_silent_replans(asset_dir, world):
    blocked = {"lower_rear_10"}
    world.collides = lambda objects: bool(objects)
    world.candidate_objects = lambda c: [c] if c["slot"] in blocked else []
    manifest = _generate(asset_dir, world, "B")
    assert manifest["packing"]["geometry_result"] == "FAIL"
    assert [u["item"] for u in manifest["packing"]["unplaced"]] == ["lower_rear_gap10_salad_plate"]
    assert manifest["counts"]["salad_plate"] == 9
    assert _row(manifest, "rear") == list(claims.LOWER_REAR)[:-1]


def test_bowl_slabs_must_be_disjoint_along_the_mouth_axis():
    points = _lathe(CATALOG["bowl"]["size_m"], .028)
    orient = loading.rotation("Y", np.radians(65))
    first = ({"position": [.10, -.14, .05]}, orient)
    nesting = ({"position": [.10 + .060, -.14, .05]}, orient)     # 54 mm along the axis < 65 mm depth
    apart = ({"position": [.10 + .080, -.14, .05]}, orient)       # 72 mm along the axis (the v2 pair)
    assert not claims.slabs_disjoint(points, first, nesting)
    assert claims.slabs_disjoint(points, first, apart)
    assert claims.slabs_disjoint(points, apart, first)


def test_unknown_variant_is_rejected(asset_dir, world):
    with pytest.raises(ValueError, match="Unknown claim variant"):
        claims.generate(asset_dir, "C", world=world)
