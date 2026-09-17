"""Host tests for experiment provenance and measured component transforms."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire import random_pose_assets as experiment
from dishsim_frigidaire import tableware
from dishsim_frigidaire.asset import BODY_POSITIONS, COMPONENT_FILES, JOINT_LIMITS
from dishsim_frigidaire.geometry import PARAMETERS


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class InputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "tableware").mkdir()
        names = ["fdpc4221as.usdc", *COMPONENT_FILES.values(),
                 *(f"tableware/{kind}.usdc" for kind in experiment.KINDS)]
        for name in names:
            (self.root / name).write_bytes(name.encode())
        self.usd = self.root / "fdpc4221as.usdc"
        self.parameters = {"geometry": PARAMETERS, "body_positions_m": BODY_POSITIONS,
                           "joint_limits_usd": JOINT_LIMITS}
        self.report = {"result": "PASS (USD authoring only)",
                       "components": {name: {} for name in COMPONENT_FILES},
                       "sha256": {name: digest(self.root / name) for name in names}}
        for rack, key in (("LowerRack", "lower_rack"), ("UpperRack", "upper_rack")):
            self.report["components"][rack]["geometry_revision"] = PARAMETERS[key]["geometry_revision"]
        self.catalog = {"source_sha256": digest(Path(tableware.__file__)),
                        "items": {kind: {**tableware.CATALOG[kind],
                                         "sha256": digest(self.root / f"tableware/{kind}.usdc")}
                                  for kind in experiment.KINDS}}
        self.write()

    def write(self):
        for name, data in (("parameters.json", self.parameters), ("geometry_validation.json", self.report),
                           ("tableware/catalog.json", self.catalog)):
            (self.root / name).write_text(json.dumps(data))

    def test_current_three_dishes_validate_without_a_rack_assignment(self):
        metadata = experiment.validate_inputs(self.usd)
        self.assertEqual(set(metadata["catalog"]), set(experiment.KINDS))
        self.assertEqual(metadata["asset_hashes"]["fdpc4221as.usdc"], digest(self.usd))
        self.assertEqual(metadata["catalog"]["mug"]["rack"], "upper")

    def test_modified_component_and_missing_dish_are_rejected(self):
        (self.root / "door.usdc").write_bytes(b"modified")
        with self.assertRaisesRegex(ValueError, "hash mismatch: door"):
            experiment.validate_inputs(self.usd)
        (self.root / "door.usdc").write_bytes(b"door.usdc")
        (self.root / "tableware/bowl.usdc").unlink()
        with self.assertRaisesRegex(ValueError, "asset missing"):
            experiment.validate_inputs(self.usd)

    def test_stale_geometry_and_catalog_do_not_validate(self):
        self.parameters = json.loads(json.dumps(self.parameters))
        self.parameters["geometry"]["lower_rack"]["tines_per_bank"] += 1
        self.write()
        with self.assertRaisesRegex(ValueError, "Stale parameters.geometry"):
            experiment.validate_inputs(self.usd)
        self.parameters["geometry"] = PARAMETERS
        self.catalog["items"]["mug"]["mass_kg"] = 10.
        self.write()
        with self.assertRaisesRegex(ValueError, "current CATALOG: mug"):
            experiment.validate_inputs(self.usd)

    def test_stale_catalog_source_and_component_report_are_rejected(self):
        self.catalog["source_sha256"] = "old"
        self.write()
        with self.assertRaisesRegex(ValueError, "different tableware source"):
            experiment.validate_inputs(self.usd)
        self.catalog["source_sha256"] = digest(Path(tableware.__file__))
        del self.report["components"]["Door"]
        self.write()
        with self.assertRaisesRegex(ValueError, "all appliance components"):
            experiment.validate_inputs(self.usd)


class Transform:
    def __init__(self, rotation, translation):
        self.rotation = np.asarray(rotation)
        self.translation = np.asarray(translation)


class CollisionObject:
    def __init__(self, geometry, transform):
        self.geometry = geometry
        self.transform = transform

    def setTransform(self, transform):
        self.transform = transform


class Manager:
    def __init__(self):
        self.objects = []
        self.updates = 0

    def registerObjects(self, objects):
        self.objects.extend(objects)

    def setup(self):
        pass

    def update(self):
        self.updates += 1

    def collide(self, candidate, data, callback):
        # Point-collider fake makes transform errors observable at the boundary.
        data.result.is_collision = any(np.allclose(candidate.transform.translation, obj.transform.translation)
                                       for obj in self.objects)


class TransformTests(unittest.TestCase):
    def setUp(self):
        from dishsim_frigidaire import loading
        self.part = SimpleNamespace(geometry=object(), rotation=np.eye(3), translation=np.array([1., 0., 0.]))
        fcl = SimpleNamespace(Transform=Transform, CollisionObject=CollisionObject,
                             DynamicAABBTreeCollisionManager=Manager,
                             CollisionData=lambda **kw: SimpleNamespace(result=SimpleNamespace(is_collision=False)),
                             CollisionRequest=lambda **kw: None, defaultCollisionCallback=None)
        with patch.dict(sys.modules, {"fcl": fcl}), \
                patch.object(loading, "collision_parts", return_value=[self.part]), \
                patch.object(loading, "visual_points", return_value=np.array([[0., 0., 0.]])):
            self.world = experiment.LiveCollisionWorld("unused")
        self.poses = {name: {"position_m": [10. * (i+1), 0., 0.],
                             "quaternion_xyzw": [0., 0., 0., 1.]}
                      for i, name in enumerate(COMPONENT_FILES)}

    def test_queries_require_every_measured_component(self):
        with self.assertRaisesRegex(RuntimeError, "measured appliance"):
            self.world.collides("mug", np.eye(4))
        with self.assertRaisesRegex(ValueError, "all five"):
            self.world.update_components({"LowerRack": self.poses["LowerRack"]})

    def test_rotation_moves_local_collider_offsets_and_collision_queries(self):
        q = [0., 0., np.sqrt(.5), np.sqrt(.5)]
        self.poses["Door"] = {"position_m": [2., 3., 4.], "quaternion_xyzw": q}
        self.world.update_components(self.poses)
        door = self.world.component_objects["Door"][0].transform
        np.testing.assert_allclose(door.translation, [2., 4., 4.], atol=1e-12)
        np.testing.assert_allclose(door.rotation @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)
        self.assertTrue(self.world.collides("mug", self.poses["Door"]))
        self.poses["Door"]["position_m"] = [8., 9., 10.]
        self.world.update_components(self.poses)
        self.assertFalse(self.world.collides("mug", {"position_m": [2., 3., 4.], "quaternion_xyzw": q}))
        self.assertEqual(self.world.manager.updates, 2)

    def test_invalid_pose_cannot_partially_update_world(self):
        self.world.update_components(self.poses)
        old = self.world.component_objects["Cabinet"][0].transform.translation.copy()
        self.poses["Cabinet"]["position_m"] = [99., 0., 0.]
        self.poses["Door"]["quaternion_xyzw"] = [0., 0., 0., 0.]
        with self.assertRaisesRegex(ValueError, "normalized"):
            self.world.update_components(self.poses)
        np.testing.assert_equal(self.world.component_objects["Cabinet"][0].transform.translation, old)


if __name__ == "__main__":
    unittest.main()
