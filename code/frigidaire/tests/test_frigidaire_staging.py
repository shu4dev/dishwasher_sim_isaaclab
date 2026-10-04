"""Historical copy preservation, current-collection archiving and staging guards; no USD or rendering."""
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dishsim_frigidaire.paths import REPO_ROOT, SOURCE_ROOT

_SCRIPT = REPO_ROOT / "code/initialization/frigidaire/stage_frigidaire_collection.py"
_SPEC = importlib.util.spec_from_file_location("frigidaire_stage_for_test", _SCRIPT)
staging = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(staging)


def _snapshot(directory):
    return {str(path.relative_to(directory)): path.read_bytes()
            for path in directory.rglob("*") if path.is_file()}


class FrigidaireStagingTests(unittest.TestCase):
    def setUp(self):
        scratch = REPO_ROOT / "data/build/frigidaire_tests"
        scratch.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix="staging-", dir=scratch)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def test_copy_preserves_source_bytes_and_nested_history_and_is_repeatable(self):
        source, target = self.root/"source", self.root/"archived"
        (source/"nested").mkdir(parents=True)
        (source/"release.json").write_bytes(b'{"status": "historical"}\n')
        (source/"nested/asset.usdc").write_bytes(bytes(range(256))*100)
        before = _snapshot(source)
        staging.copy_verified(source, target)
        self.assertEqual(_snapshot(target), before)
        self.assertEqual(_snapshot(source), before)
        for name, contents in before.items():
            self.assertEqual(staging.digest(target/name), hashlib.sha256(contents).hexdigest())
        (target/"retained.txt").write_bytes(b"another preserved history file")
        staged = _snapshot(target)
        staging.copy_verified(source, target)
        self.assertEqual(_snapshot(target), staged)
        self.assertEqual(_snapshot(source), before)

    def test_differing_archived_file_is_rejected_without_overwriting_either_copy(self):
        source, target = self.root/"original.usdc", self.root/"archived.usdc"
        source.write_bytes(b"new original bytes")
        target.write_bytes(b"earlier archived bytes")
        before = _snapshot(self.root)
        with self.assertRaisesRegex(ValueError, "differing archived file"):
            staging.copy_verified(source, target)
        self.assertEqual(_snapshot(self.root), before)

    def test_corrupt_copy_is_detected_by_destination_checksum(self):
        source, target = self.root/"original.usdc", self.root/"archive/asset.usdc"
        source.write_bytes(b"original geometry bytes")
        def corrupt_copy(original, destination):
            Path(destination).write_bytes(b"corrupt copied bytes")
        with patch.object(staging.shutil, "copy2", side_effect=corrupt_copy):
            with self.assertRaisesRegex(ValueError, "Copy checksum mismatch"):
                staging.copy_verified(source, target)
        self.assertEqual(source.read_bytes(), b"original geometry bytes")
        self.assertFalse(target.exists())
        self.assertEqual(list(target.parent.iterdir()), [])
        staging.copy_verified(source, target)
        self.assertEqual(target.read_bytes(), source.read_bytes())

    def test_canonical_collection_is_rejected_before_any_directories_are_created(self):
        canonical = self.root/"installed_collection"
        with patch.object(staging, "COLLECTION_DIR", canonical):
            with self.assertRaisesRegex(ValueError, "separate directory"):
                staging.stage(canonical)
        self.assertFalse(canonical.exists())
        self.assertEqual(list(self.root.iterdir()), [])

    def test_symlink_alias_of_canonical_collection_is_rejected_before_changes(self):
        canonical, alias = self.root/"installed_collection", self.root/"alias"
        canonical.mkdir()
        (canonical/"existing.usdc").write_bytes(b"installed appliance")
        alias.symlink_to(canonical, target_is_directory=True)
        before = _snapshot(canonical)
        with patch.object(staging, "COLLECTION_DIR", canonical):
            with self.assertRaisesRegex(ValueError, "separate directory"):
                staging.stage(alias)
        self.assertEqual(_snapshot(canonical), before)
        self.assertEqual({path.name for path in canonical.iterdir()}, {"existing.usdc"})


class ArchiveCurrentTests(unittest.TestCase):
    """archive_current copies usd/images/validation/README into history/<version> and records what went stale."""

    def setUp(self):
        scratch = REPO_ROOT / "data/build/frigidaire_tests"
        scratch.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix="archive-", dir=scratch)
        self.addCleanup(temporary.cleanup)
        self.collection = Path(temporary.name)/"collection"
        self.validation = {"result": "PASS (USD authoring only)",
                           "components": {"LowerRack": {"geometry_revision": "lower_test_rev"},
                                          "SilverwareBasket": {"geometry_revision": "basket_test_rev"}},
                           "sha256": {"fdpc4221as.usdc": "0"*64}}
        self.parameters = {"body_positions_m": {"LowerRack": [0, .008, .215], "SilverwareBasket": [.17, .104, .222]}}
        self.manifest = {"status": "STAGED",
                         "archive_copy_map": [{"original": "assets/models/x", "archived": "history/v1/assets"}]}
        files = {
            "usd/fdpc4221as.usdc": b"current usd bytes",
            "usd/geometry_validation.json": json.dumps(self.validation).encode(),
            "usd/parameters.json": json.dumps(self.parameters).encode(),
            "images/x.png": b"\x89PNG current picture",
            "validation/collection_manifest.json": json.dumps(self.manifest).encode(),
            "README.md": b"# Collection\n",
        }
        for name, content in files.items():
            path = self.collection/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        self.before = _snapshot(self.collection)

    def archive(self, version="v9"):
        with redirect_stdout(io.StringIO()) as output:
            manifest = staging.archive_current(self.collection, version)
        return manifest, output.getvalue()

    def test_archives_usd_images_validation_and_readme_into_history(self):
        manifest, output = self.archive()
        archived = _snapshot(self.collection/"history/v9")
        copies = {"assets/fdpc4221as.usdc": "usd/fdpc4221as.usdc",
                  "assets/geometry_validation.json": "usd/geometry_validation.json",
                  "assets/parameters.json": "usd/parameters.json",
                  "gallery/x.png": "images/x.png",
                  "validation/collection_manifest.json": "validation/collection_manifest.json",
                  "README.md": "README.md"}
        self.assertEqual(set(archived), set(copies) | {"archive_manifest.json"})
        for archived_name, original in copies.items():
            self.assertEqual(archived[archived_name], self.before[original], archived_name)
        written = json.loads(archived["archive_manifest.json"])
        self.assertEqual(written["version"], "v9")
        self.assertEqual(written["geometry_revisions"],
                         {"LowerRack": "lower_test_rev", "SilverwareBasket": "basket_test_rev"})
        self.assertEqual(written["body_positions_m"], self.parameters["body_positions_m"])
        self.assertEqual(written["usdc_sha256"], self.validation["sha256"])
        self.assertEqual(written["stale_results"], staging.STALE_RESULTS)
        self.assertEqual([entry["archived"] for entry in written["archive_copy_map"]],
                         ["history/v9/assets", "history/v9/gallery", "history/v9/validation", "history/v9/README.md"])
        self.assertEqual(set(written["files"]), set(copies))
        for name, digest in written["files"].items():
            self.assertEqual(digest, hashlib.sha256(archived[name]).hexdigest())
        self.assertEqual(manifest, written)
        self.assertIn("[RESULT] PASS", output)
        after = _snapshot(self.collection)
        for name, content in self.before.items():
            if name != "validation/collection_manifest.json":
                self.assertEqual(after[name], content, name)
        updated = json.loads(after["validation/collection_manifest.json"])
        self.assertEqual(updated["status"], "STAGED")
        self.assertEqual(updated["archive_copy_map"],
                         self.manifest["archive_copy_map"] + written["archive_copy_map"])
        self.assertEqual(updated["stale_evidence"], staging.STALE_RESULTS)
        self.assertIn("v9", {p.name for p in (self.collection/"history").iterdir()})
        self.assertTrue((self.collection/"history/README.md").is_file())

    def test_existing_version_is_refused_without_changes(self):
        self.archive("v9")
        snapshot = _snapshot(self.collection)
        with self.assertRaises(FileExistsError):
            self.archive("v9")
        self.assertEqual(_snapshot(self.collection), snapshot)

    def test_missing_usd_bundle_is_refused_before_writing(self):
        (self.collection/"usd/fdpc4221as.usdc").unlink()
        with self.assertRaises(FileNotFoundError):
            self.archive("v9")
        self.assertFalse((self.collection/"history").exists())


if __name__ == "__main__":
    unittest.main()
