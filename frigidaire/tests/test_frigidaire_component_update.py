"""Rack component installation semantics; runs with unittest and numpy, without USD.

    PYTHONPATH=src:frigidaire/src python3 -m unittest discover -s frigidaire/tests -p test_frigidaire_component_update.py

The optional USD test also exercises the authored component and composed bundle
when run in the Isaac runtime. All scratch files live in a temporary directory.
"""

from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dishsim_frigidaire import asset
from dishsim_frigidaire import geometry
from dishsim_frigidaire.paths import REPO_ROOT


def _temporary_directory(prefix):
    scratch = REPO_ROOT / "build/frigidaire_tests"
    scratch.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(prefix=prefix, dir=scratch)


def _build_usd_fixture(directory):
    # Exercise the migrated source without relying on a preinstalled bundle or
    # assuming current appliance assets include historical full-load scenes.
    asset.build(directory)
    (directory/"full_load.usda").write_text("#usda 1.0\n")
    (directory/"full_load_settled.json").write_text("{}\n")


def _snapshot(directory):
    return {str(path.relative_to(directory)): path.read_bytes()
            for path in directory.rglob("*") if path.is_file()}


def _fake_author(component, name, filename):
    Path(filename).write_bytes(f"new {name} USD bytes".encode())
    return {"bounds_m": asset._component_bounds(component).tolist(),
            "mass_kg": component["mass"], "wire_paths": len(component["wires"]),
            "triangles": sum(len(mesh.faces) for mesh in component["meshes"].values()),
            "colliders": 100}


class UpperRackComponentUpdateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = _temporary_directory("upper-rack-install-test-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)/"bundle"
        self.directory.mkdir()
        self.parameters = {
            "geometry": {"upper_rack": {"old": True}, "lower_rack": {"keep": 1},
                         "origins": {"UpperRack": [0, .008, .590], "SilverwareBasket": [.186, .128, .226]}},
            "body_positions_m": {"UpperRack": [0, .008, .590], "SilverwareBasket": [.186, .128, .226]},
            "candidate_sites": {name: {"old_site": name} for name in asset.COMPONENT_FILES},
            "joint_limits_usd": {"do_not_rewrite": [1, 2]},
            "additional_metadata": "retain"}
        self.report = {
            "result": "historical report", "isaac_sim_validated": True,
            "components": {name: {"old": name} for name in asset.COMPONENT_FILES},
            "tableware": {"items": {"cup": {"usd": "tableware/cup.usdc", "sha256": "old"}}},
            "sha256": {name: "old hash" for name in asset.COMPONENT_FILES.values()}}
        files = [*asset.COMPONENT_FILES.values(), "fdpc4221as.usdc", "example_scene.usda",
                 "fixtures/dimensions.json", "tableware/catalog.json", "tableware/cup.usdc",
                 *(f"fixtures/{name}.usdc" for name in ("plate", "cup", "bowl", "utensil")),
                 "full_load.usda", "full_load_settled.json", "full_load_manifest.json",
                 "full_load_banned_candidates.json", "USAGE.md", "custom/other.bin"]
        for name in files:
            path = self.directory/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"original bytes: {name}\n".encode())
        for name, value in [("parameters.json", self.parameters),
                            ("geometry_validation.json", self.report)]:
            (self.directory/name).write_text(json.dumps(value))
        for name in ("README.md", "CAPACITY.md"):
            (self.directory/name).write_text("# Historical capacity\n\n67 objects were supported.\n")
        (self.directory/"fdpc4221as.usdc").write_text(json.dumps({
            "basket_translation": [.186, .128, .226],
            "unchanged_opinions": {"references": asset.COMPONENT_FILES,
                                   "joint_stiffness": 300., "custom_user_opinion": "preserve"}}))
        self.before = _snapshot(self.directory)

    def _build(self):
        with patch.object(asset, "write_component", side_effect=_fake_author) as author, \
                patch.object(asset, "_validate_component_usd") as validate, \
                patch.object(geometry, "build_components", side_effect=AssertionError("full build called")):
            report = asset.build(self.directory, component="UpperRack")
        self.assertEqual(author.call_count, 1)
        self.assertEqual(author.call_args.args[1], "UpperRack")
        self.assertEqual(validate.call_count, 1)
        return report

    def test_partial_update_preserves_all_non_target_files_and_metadata(self):
        report = self._build()
        after = _snapshot(self.directory)
        permitted = {"upper_rack.usdc", "parameters.json", "geometry_validation.json",
                     "README.md", "CAPACITY.md"}
        self.assertEqual(set(after), set(self.before))
        self.assertEqual({name for name in after if after[name] != self.before[name]}, permitted)
        for name in self.before.keys()-permitted:
            self.assertEqual(after[name], self.before[name], name)
        parameters = json.loads(after["parameters.json"])
        expected = deepcopy(self.parameters)
        expected["geometry"]["upper_rack"] = geometry.PARAMETERS["upper_rack"]
        expected["candidate_sites"]["UpperRack"] = parameters["candidate_sites"]["UpperRack"]
        self.assertEqual(parameters, json.loads(json.dumps(expected)))
        upper = geometry._upper_rack()
        self.assertEqual(parameters["candidate_sites"]["UpperRack"]["positions_m"], upper["sites"])
        self.assertEqual(report["tableware"], self.report["tableware"])
        for name in asset.COMPONENT_FILES.keys()-{"UpperRack"}:
            self.assertEqual(report["components"][name], self.report["components"][name])
        for name, digest in report["sha256"].items():
            self.assertEqual(digest, hashlib.sha256(after[name]).hexdigest(), name)

    def test_partial_update_invalidates_physics_and_keeps_original_load_hashes(self):
        report = self._build()
        self.assertIs(report["isaac_sim_validated"], False)
        update = report["component_update"]
        self.assertEqual(update["geometry_revision"], geometry.PARAMETERS["upper_rack"]["geometry_revision"])
        self.assertIn("not accepted placements", update["loading_validation_status"])
        self.assertEqual(update["previous_sha256"], hashlib.sha256(self.before["upper_rack.usdc"]).hexdigest())
        evidence = update["prior_full_load_evidence"]
        self.assertIn("stale", evidence["status"])
        for name in ("full_load.usda", "full_load_manifest.json", "full_load_settled.json"):
            self.assertEqual(evidence["sha256"][name], hashlib.sha256(self.before[name]).hexdigest())
            self.assertEqual((self.directory/name).read_bytes(), self.before[name])
        for name in ("README.md", "CAPACITY.md"):
            text = (self.directory/name).read_text()
            self.assertTrue(text.startswith(asset._STALE_LOAD_NOTICE))
            self.assertTrue(text.endswith(self.before[name].decode()))
        self._build()
        for name in ("README.md", "CAPACITY.md"):
            self.assertEqual((self.directory/name).read_text().count(asset._STALE_LOAD_NOTICE), 1)

    def test_requires_existing_complete_bundle_without_creating_output(self):
        absent = self.directory.parent/"absent"
        with self.assertRaisesRegex(ValueError, "existing complete bundle"):
            asset.build(absent, component="UpperRack")
        self.assertFalse(absent.exists())
        for name in ("lower_rack.usdc", "fixtures/cup.usdc", "tableware/cup.usdc"):
            path = self.directory/name
            content = path.read_bytes()
            path.unlink()
            before = _snapshot(self.directory)
            with self.assertRaisesRegex(ValueError, "missing"):
                asset.build(self.directory, component="UpperRack")
            self.assertEqual(_snapshot(self.directory), before)
            path.write_bytes(content)

    def test_authoring_or_validation_failure_leaves_original_bundle(self):
        for failure_stage in ("author", "validation"):
            with self.subTest(failure_stage=failure_stage):
                def author(component, name, filename):
                    result = _fake_author(component, name, filename)
                    if failure_stage == "author":
                        raise RuntimeError("author failed")
                    return result
                with patch.object(asset, "write_component", side_effect=author), \
                        patch.object(asset, "_validate_component_usd", side_effect=ValueError("invalid USD")):
                    with self.assertRaises((RuntimeError, ValueError)):
                        asset.build(self.directory, component="UpperRack")
                self.assertEqual(_snapshot(self.directory), self.before)
                self.assertFalse(list(self.directory.glob(".upper-rack-*")))

    def test_install_failure_rolls_back_all_already_replaced_files(self):
        original_replace = Path.replace
        def fail_metadata_once(path, target):
            if path.name == "geometry_validation.json":
                raise OSError("simulated metadata installation failure")
            return original_replace(path, target)
        with patch.object(asset, "write_component", side_effect=_fake_author), \
                patch.object(asset, "_validate_component_usd"), \
                patch.object(Path, "replace", fail_metadata_once):
            with self.assertRaisesRegex(OSError, "installation failure"):
                asset.build(self.directory, component="UpperRack")
        self.assertEqual(_snapshot(self.directory), self.before)

    def test_invalid_geometry_fails_before_authoring(self):
        upper = geometry._upper_rack()
        mesh = next(iter(upper["meshes"].values()))
        mesh.faces[0] = [0, 0, 0]
        with patch.object(geometry, "_upper_rack", return_value=upper), \
                patch.object(asset, "write_component") as author:
            with self.assertRaisesRegex(ValueError, "degenerate"):
                asset.build(self.directory, component="UpperRack")
        author.assert_not_called()
        self.assertEqual(_snapshot(self.directory), self.before)

    def test_unsupported_component_fails_without_changes(self):
        with self.assertRaisesRegex(ValueError, "Unsupported component"):
            asset.build(self.directory, component="Cabinet")
        self.assertEqual(_snapshot(self.directory), self.before)

    def test_omitting_component_keeps_full_build_default(self):
        components = {name: {"meshes": {}, "sites": {}} for name in asset.COMPONENT_FILES}
        def author(component, name, filename):
            Path(filename).write_bytes(name.encode())
            return {"authored": name}
        with patch.object(geometry, "build_components", return_value=components), \
                patch.object(asset, "write_component", side_effect=author) as component_writer, \
                patch.object(asset, "write_assembly") as assembly_writer, \
                patch.object(asset, "write_fixtures") as fixture_writer, \
                patch.object(asset, "write_example_scene") as scene_writer, \
                patch("dishsim_frigidaire.tableware.write_tableware", return_value={}) as tableware_writer, \
                patch.object(asset, "_update_rack", side_effect=AssertionError("partial build called")):
            report = asset.build(self.directory.parent/"complete-build")
        self.assertEqual({call.args[1] for call in component_writer.call_args_list}, set(components))
        for writer in (assembly_writer, fixture_writer, scene_writer, tableware_writer):
            writer.assert_called_once()
        self.assertEqual(report["result"], "PASS (USD authoring only)")
        self.assertEqual(set(report["components"]), set(components))


def _fake_patch_basket_assembly(source, destination, position):
    contents = json.loads(Path(source).read_text())
    previous = contents["basket_translation"]
    contents["basket_translation"] = list(position)
    Path(destination).write_text(json.dumps(contents))
    return {"body": "SilverwareBasket", "previous_position_m": previous,
            "current_position_m": list(position), "preserved_opinions": "stubbed USD boundary"}


def _fake_validate_rack_assembly(filename, name, position):
    contents = json.loads(Path(filename).read_text())
    if contents["basket_translation"] != list(position):
        raise ValueError("incorrect basket translation")
    for reference in contents["unchanged_opinions"]["references"].values():
        if not (Path(filename).parent/reference).is_file():
            raise ValueError("missing staged reference: "+reference)


class LowerRackComponentUpdateTests(unittest.TestCase):
    setUp = UpperRackComponentUpdateTests.setUp

    def _build(self):
        with patch.object(asset, "write_component", side_effect=_fake_author) as author, \
                patch.object(asset, "_validate_component_usd") as validate, \
                patch.object(asset, "_patch_basket_assembly", side_effect=_fake_patch_basket_assembly) as assembly, \
                patch.object(asset, "_validate_rack_assembly", side_effect=_fake_validate_rack_assembly) as compose, \
                patch.object(geometry, "build_components", side_effect=AssertionError("full build called")):
            report = asset.build(self.directory, component="LowerRack")
        self.assertEqual(author.call_count, 1)
        self.assertEqual(author.call_args.args[1], "LowerRack")
        self.assertEqual(validate.call_count, 1)
        self.assertEqual(assembly.call_count, 1)
        self.assertEqual(compose.call_count, 1)
        return report

    def test_lower_update_preserves_upper_basket_and_all_other_component_bytes(self):
        report = self._build()
        after = _snapshot(self.directory)
        permitted = {"lower_rack.usdc", "fdpc4221as.usdc", "parameters.json", "geometry_validation.json",
                     "README.md", "CAPACITY.md"}
        self.assertEqual(set(after), set(self.before))
        self.assertEqual({name for name in after if after[name] != self.before[name]}, permitted)
        for name in self.before.keys()-permitted:
            self.assertEqual(after[name], self.before[name], name)
        for name in asset.COMPONENT_FILES.keys()-{"LowerRack"}:
            self.assertEqual(report["components"][name], self.report["components"][name])
        self.assertEqual(report["tableware"], self.report["tableware"])
        for name, digest in report["sha256"].items():
            self.assertEqual(digest, hashlib.sha256(after[name]).hexdigest(), name)
        old = json.loads(self.before["fdpc4221as.usdc"])
        new = json.loads(after["fdpc4221as.usdc"])
        self.assertEqual(new["unchanged_opinions"], old["unchanged_opinions"])
        self.assertEqual(new["basket_translation"], geometry.PARAMETERS["origins"]["SilverwareBasket"])

    def test_lower_metadata_and_authored_basket_origin_share_canonical_parameters(self):
        report = self._build()
        parameters = json.loads((self.directory/"parameters.json").read_text())
        expected = deepcopy(self.parameters)
        expected["geometry"]["lower_rack"] = geometry.PARAMETERS["lower_rack"]
        lower = geometry._lower_rack()
        expected["candidate_sites"]["LowerRack"] = {
            "positions_m": lower["sites"], "quat_wxyz_overrides": lower.get("site_quats", {})}
        position = geometry.PARAMETERS["origins"]["SilverwareBasket"]
        expected["geometry"]["origins"]["SilverwareBasket"] = position
        expected["body_positions_m"]["SilverwareBasket"] = position
        self.assertEqual(parameters, json.loads(json.dumps(expected)))
        self.assertEqual(asset.BODY_POSITIONS,
                         {name: tuple(pos) for name, pos in geometry.PARAMETERS["origins"].items()})
        self.assertEqual(list(asset.BODY_POSITIONS["SilverwareBasket"]), position)
        update = report["component_update"]
        self.assertEqual(update["component"], "LowerRack")
        self.assertEqual(update["geometry_revision"], geometry.PARAMETERS["lower_rack"]["geometry_revision"])
        self.assertEqual(update["assembly_update"]["previous_position_m"], [.186, .128, .226])
        self.assertEqual(update["assembly_update"]["current_position_m"], position)
        self.assertEqual(update["assembly_update"]["previous_sha256"],
                         hashlib.sha256(self.before["fdpc4221as.usdc"]).hexdigest())

    def test_prior_load_bytes_and_hashes_remain_stale_and_are_rejected(self):
        from dishsim_frigidaire.load_validation import validate_manifest

        recorded_hashes = {name: hashlib.sha256(value).hexdigest()
                           for name, value in self.before.items() if name.endswith(".usdc")}
        report = self._build()
        self.assertIs(report["isaac_sim_validated"], False)
        self.assertIn("not accepted placements", report["component_update"]["loading_validation_status"])
        evidence = report["component_update"]["prior_full_load_evidence"]
        for name, digest in evidence["sha256"].items():
            self.assertEqual(digest, hashlib.sha256(self.before[name]).hexdigest())
            self.assertEqual((self.directory/name).read_bytes(), self.before[name])
        with self.assertRaisesRegex(ValueError, "Stale manifest geometry_hashes"):
            validate_manifest({"schema_version": 1, "geometry_hashes": recorded_hashes},
                              self.directory/"fdpc4221as.usdc", {}, asset.BODY_POSITIONS)
        for name in ("README.md", "CAPACITY.md"):
            self.assertTrue((self.directory/name).read_text().startswith(asset._LOWER_STALE_LOAD_NOTICE))
        self._build()
        for name in ("README.md", "CAPACITY.md"):
            self.assertEqual((self.directory/name).read_text().count(asset._LOWER_STALE_LOAD_NOTICE), 1)

    def test_lower_update_after_upper_update_retains_upper_revision_and_bytes(self):
        UpperRackComponentUpdateTests._build(self)
        after_upper = _snapshot(self.directory)
        upper_parameters = json.loads(after_upper["parameters.json"])
        upper_report = json.loads(after_upper["geometry_validation.json"])
        self._build()
        after_lower = _snapshot(self.directory)
        self.assertEqual(after_lower["upper_rack.usdc"], after_upper["upper_rack.usdc"])
        parameters = json.loads(after_lower["parameters.json"])
        self.assertEqual(parameters["geometry"]["upper_rack"], upper_parameters["geometry"]["upper_rack"])
        self.assertEqual(parameters["candidate_sites"]["UpperRack"], upper_parameters["candidate_sites"]["UpperRack"])
        report = json.loads(after_lower["geometry_validation.json"])
        self.assertEqual(report["components"]["UpperRack"], upper_report["components"]["UpperRack"])

    def test_lower_assembly_validation_failure_leaves_every_original_file(self):
        with patch.object(asset, "write_component", side_effect=_fake_author), \
                patch.object(asset, "_validate_component_usd"), \
                patch.object(asset, "_patch_basket_assembly", side_effect=_fake_patch_basket_assembly), \
                patch.object(asset, "_validate_rack_assembly", side_effect=ValueError("assembly validation failed")):
            with self.assertRaisesRegex(ValueError, "assembly validation failed"):
                asset.build(self.directory, component="LowerRack")
        self.assertEqual(_snapshot(self.directory), self.before)
        self.assertFalse(list(self.directory.glob(".lower-rack-*")))

    def test_lower_install_failure_rolls_back_rack_assembly_and_metadata(self):
        original_replace = Path.replace
        for failure_name in ("fdpc4221as.usdc", "geometry_validation.json"):
            with self.subTest(failure_name=failure_name):
                failed = False
                def fail_once(path, target):
                    nonlocal failed
                    if path.name == failure_name and not failed:
                        failed = True
                        raise OSError("simulated lower installation failure")
                    return original_replace(path, target)
                with patch.object(asset, "write_component", side_effect=_fake_author), \
                        patch.object(asset, "_validate_component_usd"), \
                        patch.object(asset, "_patch_basket_assembly", side_effect=_fake_patch_basket_assembly), \
                        patch.object(asset, "_validate_rack_assembly", side_effect=_fake_validate_rack_assembly), \
                        patch.object(Path, "replace", fail_once):
                    with self.assertRaisesRegex(OSError, "lower installation failure"):
                        asset.build(self.directory, component="LowerRack")
                self.assertEqual(_snapshot(self.directory), self.before)

    def test_lower_partial_build_requires_basket_origin_metadata(self):
        parameters = deepcopy(self.parameters)
        del parameters["body_positions_m"]["SilverwareBasket"]
        (self.directory/"parameters.json").write_text(json.dumps(parameters))
        before = _snapshot(self.directory)
        with patch.object(asset, "write_component") as author:
            with self.assertRaisesRegex(ValueError, "basket origin"):
                asset.build(self.directory, component="LowerRack")
        author.assert_not_called()
        self.assertEqual(_snapshot(self.directory), before)


@unittest.skipUnless(importlib.util.find_spec("pxr"), "USD runtime is unavailable")
class UpperRackUSDUpdateTests(unittest.TestCase):
    def test_real_component_update_composes_and_preserves_other_assets(self):
        from pxr import Usd, UsdUtils

        with _temporary_directory("upper-rack-usd-test-") as temporary:
            directory = Path(temporary)/"bundle"
            _build_usd_fixture(directory)
            before = _snapshot(directory)
            report = asset.build(directory, component="UpperRack")
            stage = Usd.Stage.Open(str(directory/"fdpc4221as.usdc"))
            for name in asset.COMPONENT_FILES:
                self.assertTrue(stage.GetPrimAtPath(asset.ROOT+"/"+name+"/Visuals"), name)
                if name != "UpperRack":
                    self.assertEqual((directory/asset.COMPONENT_FILES[name]).read_bytes(),
                                     before[asset.COMPONENT_FILES[name]])
            self.assertEqual(list(UsdUtils.ComputeAllDependencies(str(directory/"fdpc4221as.usdc"))[2]), [])
            self.assertGreater(report["components"]["UpperRack"]["colliders"], 0)
            for name in ("fdpc4221as.usdc", "full_load.usda", "full_load_settled.json"):
                self.assertEqual((directory/name).read_bytes(), before[name])

    def test_lower_revision_only_changes_basket_translation_in_assembly(self):
        from pxr import Gf, Sdf, Usd, UsdUtils

        with _temporary_directory("lower-rack-usd-test-") as temporary:
            directory = Path(temporary)/"bundle"
            _build_usd_fixture(directory)
            before_bytes = _snapshot(directory)
            original = Sdf.Layer.CreateAnonymous()
            original.TransferContent(Sdf.Layer.FindOrOpen(str(directory/"fdpc4221as.usdc")))
            # Existing custom authored opinions must survive the partial update.
            original.customLayerData = {**original.customLayerData, "lower_update_test": "retain"}
            self.assertTrue(original.Export(str(directory/"fdpc4221as.usdc")))
            report = asset.build(directory, component="LowerRack")
            revised = Sdf.Layer.FindOrOpen(str(directory/"fdpc4221as.usdc"))
            revised.Reload()
            position = geometry.PARAMETERS["origins"]["SilverwareBasket"]
            asset._validate_basket_only_layer_change(original, revised, position)
            stage = Usd.Stage.Open(revised)
            self.assertEqual(list(stage.GetPrimAtPath(asset.ROOT+"/SilverwareBasket")
                                  .GetAttribute("xformOp:translate").Get()), position)
            self.assertTrue(stage.GetPrimAtPath(asset.ROOT+"/LowerRack/Collisions"))
            self.assertEqual(list(UsdUtils.ComputeAllDependencies(str(directory/"fdpc4221as.usdc"))[2]), [])
            for name in ("upper_rack.usdc", "silverware_basket.usdc", "cabinet.usdc", "door.usdc",
                         "full_load.usda", "full_load_settled.json"):
                self.assertEqual((directory/name).read_bytes(), before_bytes[name], name)
            self.assertIs(report["isaac_sim_validated"], False)
            # The opinion guard must catch changes beyond the single allowed default.
            bad = Sdf.Layer.CreateAnonymous()
            bad.TransferContent(revised)
            bad.GetAttributeAtPath(asset.ROOT+"/UpperRack.xformOp:translate").default = Gf.Vec3d(1, 2, 3)
            with self.assertRaisesRegex(ValueError, "outside the basket translation"):
                asset._validate_basket_only_layer_change(original, bad, position)


if __name__ == "__main__":
    unittest.main()
