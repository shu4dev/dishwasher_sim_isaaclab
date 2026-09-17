"""Evidence report and gallery contracts, without starting Isaac Sim.

Only runtime imports, the shutdown helper, and the application launcher are
stubbed. Report merging, CLI parsing, hashing, and gallery generation run normally.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy  # noqa: F401 — keep the extension loaded across temporary sys.modules stubs


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/evaluation/frigidaire_asset_evidence.py"


def load_evidence(*arguments):
    class AppLauncher:
        @staticmethod
        def add_app_launcher_args(parser):
            parser.add_argument("--enable_cameras", action="store_true")
            parser.add_argument("--device", default="cpu")

        def __init__(self, args):
            self.app = SimpleNamespace()

    isaaclab = ModuleType("isaaclab")
    app = ModuleType("isaaclab.app")
    app.AppLauncher = AppLauncher
    isaaclab.app = app
    pxr = ModuleType("pxr")
    for name in ("Gf", "Usd", "UsdGeom", "UsdPhysics"):
        setattr(pxr, name, SimpleNamespace())
    media = ModuleType("dishsim.media")
    media.release_sim_for_close = lambda: None
    stubs = {"isaaclab": isaaclab, "isaaclab.app": app,
             "torch": ModuleType("torch"), "pxr": pxr, "dishsim.media": media}
    spec = importlib.util.spec_from_file_location("frigidaire_evidence_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs), patch.object(sys, "path", list(sys.path)), \
            patch.object(sys, "argv", [str(SCRIPT), "--physics-only", *arguments]):
        spec.loader.exec_module(module)
    return module


class EvidenceContractTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="frigidaire-evidence-test-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.evidence = load_evidence("--assembly-only", "--out-dir", str(self.directory))
        self.asset_hashes = {"fdpc4221as.usdc": "current-asset"}
        self.source_hashes = {"frigidaire/src/dishsim_frigidaire/asset.py": "current-source"}

    def reports(self, **overrides):
        report = {"result": "PASS", "scope": "assembly", "asset_hashes": self.asset_hashes,
                  "source_hashes": self.source_hashes, **overrides}
        for name in ("physics", "renders", "contacts"):
            (self.directory / f"{name}.json").write_text(json.dumps(report))

    def merge(self):
        with patch.object(self.evidence, "source_hashes", return_value=self.source_hashes):
            self.evidence.merge_reports(self.asset_hashes)
        return json.loads((self.directory / "evidence.json").read_text())

    def test_matching_assembly_reports_pass(self):
        self.reports()
        result = self.merge()
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["scope"], "assembly")
        self.assertEqual(result["source_hashes"], self.source_hashes)

    def test_scope_and_either_hash_mismatch_are_stale(self):
        for changed in ({"scope": "fixtures"}, {"scope": None},
                        {"asset_hashes": {"fdpc4221as.usdc": "old"}},
                        {"source_hashes": {"asset.py": "old"}}):
            with self.subTest(changed=changed):
                self.reports(**changed)
                result = self.merge()
                self.assertEqual(result["result"], "INCOMPLETE")
                self.assertTrue(all(r["result"] == "STALE" for r in result["reports"].values()))

    def test_missing_physics_cannot_be_certified_by_renders(self):
        self.reports()
        (self.directory / "physics.json").unlink()
        result = self.merge()
        self.assertEqual(result["result"], "INCOMPLETE")
        self.assertEqual(result["reports"]["physics"]["result"], "NOT_RUN")

    def test_current_failure_survives_merge(self):
        self.reports(result="FAIL")
        self.assertEqual(self.merge()["result"], "FAIL")

    def test_fixture_scope_remains_available(self):
        self.evidence.args.assembly_only = False
        self.reports(scope="fixtures")
        self.assertEqual(self.merge()["result"], "PASS")
        self.assertEqual(self.merge()["scope"], "fixtures")

    def test_report_records_scope(self):
        with patch.object(self.evidence, "version_info", return_value={}), \
                patch.object(self.evidence, "source_hashes", return_value=self.source_hashes):
            self.assertEqual(self.evidence.Evidence(self.asset_hashes).data["scope"], "assembly")

    def test_output_aliases_accept_staging_outside_media(self):
        for option in ("--out-dir", "--out_dir"):
            self.assertEqual(load_evidence(option, str(self.directory)).args.out_dir, self.directory)
        defaults = load_evidence()
        self.assertEqual(defaults.args.out_dir, defaults.IMAGE_DIR / "assembly")
        self.assertEqual(defaults.args.usd, defaults.ASSET_DIR / "fdpc4221as.usdc")
        self.assertFalse(defaults.args.assembly_only)

    def test_source_hashes_follow_relocated_package(self):
        sources = self.evidence.source_hashes()
        for name in ("asset", "geometry", "paths"):
            path = f"frigidaire/src/dishsim_frigidaire/{name}.py"
            self.assertEqual(sources[path], hashlib.sha256((self.evidence.ROOT / path).read_bytes()).hexdigest())
        self.assertIn("frigidaire/scripts/evaluation/frigidaire_asset_evidence.py", sources)
        self.assertIn("frigidaire/src/dishsim_frigidaire/cutlery_candidates.json", sources)

    def test_usd_hashes_exclude_history_and_full_load(self):
        usd = self.directory / "usd"
        usd.mkdir()
        (usd / "fdpc4221as.usdc").write_bytes(b"assembly")
        (usd / "full_load.usda").write_bytes(b"optional scene")
        (usd / "parameters.json").write_text("{}")
        history = self.directory / "history/v2"
        history.mkdir(parents=True)
        (history / "lower_rack.usdc").write_bytes(b"old rack")
        self.assertEqual(self.evidence.hashes(usd),
                         {"fdpc4221as.usdc": hashlib.sha256(b"assembly").hexdigest()})

    def test_assembly_gallery_requires_empty_extended_image(self):
        from PIL import Image, ImageFont
        filenames = ("assembled_closed.png", "assembled_both_extended.png", "lower_rack_front_left.png",
                     "upper_rack_front_left.png", "silverware_basket_front_left.png", "assembled_exploded.png")
        for filename in filenames:
            Image.new("RGB", (8, 8), (80, 90, 100)).save(self.directory / filename)
        # The host has Pillow 7; bridge only API spelling and font sizing so the
        # same image composition can be verified without installing Kit's Pillow.
        font = ImageFont.load_default()
        resampling = getattr(Image, "Resampling", SimpleNamespace(LANCZOS=Image.LANCZOS))
        with patch.object(ImageFont, "load_default", return_value=font), \
                patch.object(Image, "Resampling", resampling, create=True):
            sheet = self.evidence.write_gallery([{"file": f, "label": f} for f in filenames])
        self.assertEqual(sheet["source_images"], list(filenames))
        self.assertNotIn("assembled_loaded.png", (self.directory / "index.html").read_text())
        self.assertEqual(sheet["sha256"], hashlib.sha256((self.directory / sheet["file"]).read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
