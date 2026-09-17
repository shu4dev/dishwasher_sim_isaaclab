"""Historical copy preservation and staging guards, without USD or rendering."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dishsim_frigidaire.paths import REPO_ROOT, SOURCE_ROOT

_SCRIPT = SOURCE_ROOT / "scripts/setup/stage_frigidaire_collection.py"
_SPEC = importlib.util.spec_from_file_location("frigidaire_stage_for_test", _SCRIPT)
staging = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(staging)


def _snapshot(directory):
    return {str(path.relative_to(directory)): path.read_bytes()
            for path in directory.rglob("*") if path.is_file()}


class FrigidaireStagingTests(unittest.TestCase):
    def setUp(self):
        scratch = REPO_ROOT / "build/frigidaire_tests"
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


if __name__ == "__main__":
    unittest.main()
