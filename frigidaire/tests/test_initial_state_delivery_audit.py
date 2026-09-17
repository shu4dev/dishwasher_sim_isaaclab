"""Integrity failures the final cross-artifact audit must detect."""
import hashlib
from pathlib import Path
import struct
import sys
from tempfile import TemporaryDirectory
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"frigidaire/scripts/evaluation"))
from frigidaire_initial_state_delivery_audit import png_dimensions, selection_errors, source_coverage, valid_full_graph_bound


class DeliveryAuditTests(unittest.TestCase):
    def test_duplicate_and_conflicting_selections_are_rejected(self):
        self.assertTrue(selection_errors([0, 0], [], [], {0}, []))
        errors = selection_errors([0, 1], [], [], {0, 1}, [(0, 1)])
        self.assertIn("Selected state contains a conflicting candidate pair", errors)

    def test_fixed_target_solver_bound_cannot_certify_full_catalog(self):
        record = {"bound_scope": "full finite compatibility graph", "target_count": None,
                  "status": 0, "geometric_cardinality_upper_bound": 35, "mip_dual_bound": -35.00007, "count": 35}
        self.assertTrue(valid_full_graph_bound(record))
        self.assertFalse(valid_full_graph_bound({**record, "target_count": 35}))
        self.assertFalse(valid_full_graph_bound({**record, "mip_dual_bound": -36.00007}))

    def test_archived_content_hash_covers_renamed_revision_file(self):
        with TemporaryDirectory(dir=ROOT/"outputs") as directory:
            root = Path(directory)
            archive = root/"source_revisions"/"revision"
            archive.mkdir(parents=True)
            content = b"# executable revision\n"
            (archive/"renamed_after.py").write_bytes(content)
            expected = hashlib.sha256(content).hexdigest()
            result = source_coverage(root, [("attempt/result.json", {"execution_source_hashes":
                {"missing/current/source.py": expected}})], {})
            self.assertFalse(result["missing"])
            self.assertEqual(result["versions"][0]["coverage"], "archived")
            self.assertFalse(result["needs_archive"])

    def test_missing_execution_hash_is_detected_without_inventing_baseline_hash(self):
        with TemporaryDirectory(dir=ROOT/"outputs") as directory:
            result = source_coverage(Path(directory), [("attempt/result.json", {"execution_source_hashes":
                {"missing/source.py": "0"*64}})], {})
            self.assertEqual(len(result["missing"]), 1)
            self.assertEqual(result["reference_count"], 1)

    def test_image_dimensions_come_from_png_bytes(self):
        with TemporaryDirectory(dir=ROOT/"outputs") as directory:
            path = Path(directory)/"example.png"
            path.write_bytes(b"\x89PNG\r\n\x1a\n"+struct.pack(">I", 13)+b"IHDR"+struct.pack(">II", 1920, 1440))
            self.assertEqual(png_dimensions(path), [1920, 1440])
            path.write_bytes(b"invalid")
            with self.assertRaises(ValueError):
                png_dimensions(path)


if __name__ == "__main__":
    unittest.main()
