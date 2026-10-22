"""Host tests for measured component transforms (the provenance half moved to the asset folder's
source/ with ``random_pose_assets.validate_inputs``)."""
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
from dishsim_frigidaire import random_pose_runtime as experiment
from dishsim_frigidaire import tableware
from dishsim_frigidaire.asset import BODY_POSITIONS, COMPONENT_FILES, JOINT_LIMITS
from dishsim_frigidaire.geometry import PARAMETERS


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
