"""Collision-checker contracts without launching Kit."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES, InitialCollisionChecker
from dishsim_frigidaire.loading import Part


def pose(p=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return {"position_m": list(p), "quaternion_xyzw": list(q)}


@pytest.fixture
def checker():
    fcl = pytest.importorskip("fcl")
    result = InitialCollisionChecker.__new__(InitialCollisionChecker)
    result.fcl = fcl
    result.penetration_limit_m = .001
    result.max_contacts_per_pair = 256
    result.request = fcl.CollisionRequest(enable_contact=True, num_max_contacts=256)
    result.parts = {name: [Part(fcl.Box(.1, .1, .1), np.eye(3), np.zeros(3), "test_box")]
                    for name in (*COMPONENT_NAMES, "bowl", "mug")}
    result.bounds = {name: (np.full(3, -.05), np.full(3, .05)) for name in result.parts}
    result.components = None
    return result


@pytest.mark.parametrize("distance, valid", [(.1001, True), (.1000, True), (.0995, True), (.0990, True), (.0985, False), (.0, False)])
def test_fcl_penetration_allowance_uses_contact_depth(checker, distance, valid):
    first = checker._body("bowl", pose(), "bowl_a")
    second = checker._body("bowl", pose([distance, 0, 0]), "bowl_b")
    result = checker.pair(first, second)
    assert result["valid"] == valid
    assert result["max_penetration_m"] == pytest.approx(max(0., .1-distance), abs=1e-10)
    json.dumps(result)


def test_multi_object_collision_and_component_frame_updates(checker):
    frames = {name: pose([10. + i, 0, 0]) for i, name in enumerate(COMPONENT_NAMES)}
    objects = [{"object_id": "a", "candidate_id": "same_template", "kind": "bowl", "pose_world": pose()},
               {"object_id": "b", "candidate_id": "same_template", "kind": "mug", "pose_world": pose([.09, 0, 0])}]
    result = checker.check_arrangement(objects, frames)
    assert not result["valid"] and result["pairs"][-1]["bodies"] == ["b", "a"]
    objects[1]["pose_world"] = pose([.15, 0, 0])
    assert checker.check_arrangement(objects, frames)["valid"]
    frames["LowerRack"] = pose()
    assert not checker.check_arrangement(objects, frames)["valid"]
    objects[1]["object_id"] = "a"
    with pytest.raises(ValueError, match="unique"):
        checker.check_arrangement(objects, frames)


def test_tableware_map_replaces_stock_dishes_and_admits_new_kinds():
    pytest.importorskip("fcl")
    from dishsim_frigidaire.paths import ASSET_DIR, REPO_ROOT
    hotec = REPO_ROOT / "data/assets/models/hotec_wheatstraw/v2"
    if not (Path(ASSET_DIR).is_dir() and hotec.is_dir()):
        pytest.skip("collection build or HOTEC assets not present")
    tableware = {kind: hotec / f"{kind}.usda" for kind in ("plate", "bowl", "cup")}
    checker = InitialCollisionChecker(ASSET_DIR, tableware=tableware)
    stock = InitialCollisionChecker(ASSET_DIR)
    assert "cup" in checker.parts and "cup" not in stock.parts                  # a new kind is admitted
    assert any("hotec_wheatstraw" in key for key in checker.asset_sha256)       # and hashed by its own file
    assert len(checker.parts["bowl"]) != len(stock.parts["bowl"]) or not np.allclose(
        checker.bounds["bowl"][1], stock.bounds["bowl"][1])                      # the stock bowl was replaced
    cup = checker.candidate_body({"object_id": "cup_01", "kind": "cup", "pose_world": pose([5., 5., 5.])})
    other = checker.candidate_body({"object_id": "cup_02", "kind": "cup", "pose_world": pose([5.2, 5., 5.])})
    assert checker.pair(cup, other)["valid"]
    with pytest.raises(ValueError, match="Unsupported"):
        stock.candidate_body({"kind": "cup", "pose_world": pose()})
