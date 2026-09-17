"""Collection release gates and portable archives; no USD or Isaac required."""
from contextlib import redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile


_SCRIPT = Path(__file__).resolve().parents[1] / "scripts/setup/package_frigidaire.py"
_SPEC = importlib.util.spec_from_file_location("frigidaire_package", _SCRIPT)
package = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(package)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def snapshot(directory):
    return {path.relative_to(directory).as_posix(): path.read_bytes()
            for path in directory.rglob("*") if path.is_file()}


class CollectionPackageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="frigidaire-package-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.collection = self.root / "collection"
        self.source = self.root / "source"
        self.usd = self.collection / "usd"
        self.assembly = self.collection / "images/assembly"
        self.files = {
            "README.md": "Collection",
            "usd/example_scene.usda": "Empty scene",
            "usd/parameters.json": "{}",
            "references/spec.pdf": "Original specification",
            "references/upper_rack/front.jpg": "Original photo",
            "images/dimensions.png": "Drawing",
            "images/assembly/closed.png": "Closed current appliance",
            "images/assembly/contact_sheet.png": "Current contact sheet",
            "images/assembly/index.html": "Gallery",
            "history/v1/bundle/fdpc4221as.usdc": "Old v1 geometry",
            "history/v2/bundle/full_load.usda": "Old measured loaded scene",
            "history/v2/gallery/contact_sheet.png": "Old gallery",
            "history/v2/original.zip": "Original archive bytes",
        }
        for component in ("fdpc4221as", "cabinet", "door", "upper_rack", "lower_rack", "silverware_basket"):
            self.files[f"usd/{component}.usdc"] = f"Current {component}"
        for name, content in self.files.items():
            path = self.collection / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        source_files = {
            package.INSPECT_SOURCE, package.EVIDENCE_SOURCE,
            str(package.PACKAGE_SOURCE / "asset.py"),
            str(package.PACKAGE_SOURCE / "geometry.py"),
            str(package.PACKAGE_SOURCE / "__init__.py"),
            str(package.PACKAGE_SOURCE / "cutlery_candidates.json"),
        }
        for name in source_files:
            path = self.source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"Source fixture {name}")
        hashes = {name: package.digest(self.source / name) for name in source_files}
        self.base_report = {"result": "PASS", "asset_hashes": package.usd_hashes(self.usd),
                            "source_hashes": hashes}
        write_json(self.usd / "geometry_validation.json", {
            "result": "PASS (USD authoring only)", "components": {name: {} for name in package.COMPONENTS},
            "sha256": package.usd_hashes(self.usd, {".usdc"}),
        })
        write_json(self.collection / "validation/composition.json", {
            **self.base_report, "checks": {name: True for name in package.COMPOSITION_CHECKS},
            "parameters_sha256": package.digest(self.usd / "parameters.json"),
        })
        self.reports = {kind: {**deepcopy(self.base_report), "scope": "assembly"}
                        for kind in ("physics", "renders", "contacts")}
        self.reports["renders"].update({
            "images": [{"file": "closed.png", "sha256": package.digest(self.assembly / "closed.png")}],
            "contact_sheet": {"file": "contact_sheet.png", "sha256": package.digest(self.assembly / "contact_sheet.png")},
        })
        self.write_evidence()
        for rack in ("upper_rack", "lower_rack"):
            artifacts = [f"{rack}_overhead.png", f"{rack}_overhead.svg", f"{rack}_oblique.png"]
            for name in artifacts:
                path = self.collection / "images" / rack / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f"Diagram {name}")
            write_json(path.parent / "measurements.json", {
                "source_sha256": package.digest(self.source / package.PACKAGE_SOURCE / "geometry.py"),
                "artifacts": artifacts,
                "artifact_sha256": {name: package.digest(path.parent / name) for name in artifacts},
            })
        write_json(self.collection / "validation/dimensions.json", {
            "page": 3, "source_pdf_sha256": package.digest(self.collection / "references/spec.pdf"),
            "image_sha256": package.digest(self.collection / "images/dimensions.png"),
        })

    def write_evidence(self):
        for kind, report in self.reports.items():
            write_json(self.assembly / f"{kind}.json", report)
        write_json(self.assembly / "evidence.json", {
            **self.base_report, "scope": "assembly", "reports": self.reports, "image_index": "index.html",
        })

    def check(self):
        return package.check_collection(self.collection, self.source)

    def test_complete_collection_checks_without_writing_and_archives_every_product(self):
        before = snapshot(self.root)
        self.assertEqual(self.check()["result"], "PASS")
        self.assertEqual(snapshot(self.root), before)
        archive_path = self.root / "product.zip"
        report = package.package_collection(self.collection, archive_path, self.source)
        self.assertEqual(report["result"], "PASS")
        with zipfile.ZipFile(archive_path) as archive:
            for name, content in snapshot(self.collection).items():
                self.assertEqual(archive.read(f"collection/{name}"), content)
        self.assertEqual(snapshot(self.collection), {name[len("collection/"):]: value
                                                   for name, value in before.items() if name.startswith("collection/")})

    def test_missing_evidence_is_not_replaced_by_historical_pass(self):
        (self.assembly / "physics.json").unlink()
        write_json(self.collection / "history/v2/physics.json", self.reports["physics"])
        report = self.check()
        self.assertEqual(report["result"], "FAIL")
        self.assertFalse(report["checks"]["assembly_physics"])
        self.assertFalse(report["checks"]["assembly_evidence"])
        with self.assertRaisesRegex(ValueError, "release gates failed"):
            package.package_collection(self.collection, self.root / "absent/product.zip", self.source)
        self.assertFalse((self.root / "absent").exists())

    def test_new_or_changed_current_usd_invalidates_authoring_and_runtime(self):
        for filename in ("lower_rack.usdc", "unexpected.usdc"):
            with self.subTest(filename=filename):
                target = self.usd / filename
                original = target.read_bytes() if target.exists() else None
                target.write_bytes(b"Different current geometry")
                report = self.check()
                self.assertFalse(report["checks"]["usd_authoring"])
                self.assertFalse(report["checks"]["usd_composition"])
                self.assertFalse(report["checks"]["assembly_physics"])
                target.unlink() if original is None else target.write_bytes(original)

    def test_historical_geometry_does_not_contaminate_current_hashes(self):
        (self.collection / "history/v1/bundle/fdpc4221as.usdc").write_text("A different archived file")
        self.assertEqual(self.check()["result"], "PASS")

    def test_fixture_scope_and_nonpassing_current_reports_are_rejected(self):
        for changes in ({"scope": "fixtures"}, {"result": "NOT_RUN"}, {"source_hashes": {}}):
            with self.subTest(changes=changes):
                original = deepcopy(self.reports["physics"])
                self.reports["physics"].update(changes)
                self.write_evidence()
                self.assertFalse(self.check()["checks"]["assembly_physics"])
                self.reports["physics"] = original

    def test_source_change_invalidates_reports_and_layout_measurements(self):
        (self.source / package.PACKAGE_SOURCE / "geometry.py").write_text("Changed generator")
        report = self.check()
        for name in ("usd_composition", "assembly_physics", "upper_rack_images", "lower_rack_images"):
            self.assertFalse(report["checks"][name], name)

    def test_composition_requires_every_gate_and_combined_evidence_must_match(self):
        path = self.collection / "validation/composition.json"
        report = package.read(path)
        report["checks"]["polished_racks"] = False
        write_json(path, report)
        self.assertFalse(self.check()["checks"]["usd_composition"])
        self.reports["contacts"]["extra_measurement"] = "changed"
        write_json(self.assembly / "contacts.json", self.reports["contacts"])
        self.assertFalse(self.check()["checks"]["assembly_evidence"])

    def test_changed_render_and_dimension_images_are_rejected(self):
        (self.assembly / "closed.png").write_text("Old rack picture")
        (self.collection / "images/dimensions.png").write_text("Wrong diagram")
        report = self.check()
        self.assertFalse(report["checks"]["assembly_renders"])
        self.assertFalse(report["checks"]["dimension_image"])

    def test_changed_parameters_invalidate_composition(self):
        write_json(self.usd / "parameters.json", {"geometry": "changed metadata"})
        report = self.check()
        self.assertFalse(report["checks"]["usd_composition"])
        self.assertTrue(report["checks"]["usd_authoring"])

    def test_layout_hashes_cover_exactly_the_declared_unchanged_artifacts(self):
        directory = self.collection / "images/upper_rack"
        path = directory / "measurements.json"
        original = package.read(path)
        for change in ("modified", "missing_hash", "extra_hash", "empty_hashes"):
            with self.subTest(change=change):
                report = deepcopy(original)
                image = directory / "upper_rack_overhead.png"
                original_bytes = image.read_bytes()
                if change == "modified":
                    image.write_bytes(b"Old layout image")
                elif change == "missing_hash":
                    del report["artifact_sha256"]["upper_rack_overhead.png"]
                elif change == "extra_hash":
                    report["artifact_sha256"]["undeclared.png"] = "0" * 64
                else:
                    report["artifact_sha256"] = {}
                write_json(path, report)
                self.assertFalse(self.check()["checks"]["upper_rack_images"])
                image.write_bytes(original_bytes)
        write_json(path, original)
        self.assertEqual(self.check()["result"], "PASS")

    def test_report_paths_cannot_escape_and_archive_cannot_include_itself(self):
        self.reports["renders"]["images"][0]["file"] = "../../references/spec.pdf"
        self.write_evidence()
        self.assertFalse(self.check()["checks"]["assembly_renders"])
        self.reports["renders"]["images"][0]["file"] = "closed.png"
        self.write_evidence()
        with self.assertRaisesRegex(ValueError, "outside the collection"):
            package.package_collection(self.collection, self.collection / "product.zip", self.source)
        self.assertFalse((self.collection / "product.zip").exists())

    def test_check_cli_reports_missing_collection_without_creating_it(self):
        target = self.root / "does-not-exist"
        with redirect_stdout(io.StringIO()) as output:
            code = package.main(["--collection-dir", str(target), "--check"])
        self.assertEqual(code, 1)
        self.assertIn("Missing file", output.getvalue())
        self.assertFalse(target.exists())
        self.assertFalse((self.root / "does-not-exist.zip").exists())


if __name__ == "__main__":
    unittest.main()
