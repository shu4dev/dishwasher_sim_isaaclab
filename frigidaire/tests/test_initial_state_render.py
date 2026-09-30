"""Render helpers shared by the HOTEC bench, top-5 load, planner video and robot episode scripts.

Moved here on 2026-09-29 from test_frigidaire_initial_state_reporting.py when the v3-era
initial-state delivery pipeline (the reporting tests' subject) was retired; the helpers stayed live.
"""
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


def load_render():
    path = ROOT / "frigidaire/scripts/evaluation/frigidaire_initial_state_render.py"
    spec = importlib.util.spec_from_file_location("initial_state_render", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


render = load_render()


class InitialStateRenderTests(unittest.TestCase):
    def test_per_item_tints_and_exact_highest_label(self):
        objects = [{"object_id": f"dish_{i:03d}"} for i in range(50)]
        colors = render.item_tints(objects)
        self.assertEqual(len(set(tuple(value) for value in colors.values())), 50)
        self.assertEqual(render.label_for({"purpose": "highest"}), "Highest validated load found")

    def test_caption_fonts_are_large_and_hash_recorded(self):
        fonts, evidence = render.caption_fonts()
        self.assertEqual(fonts["title"].size, 42)
        self.assertEqual(fonts["detail"].size, 27)
        self.assertEqual(evidence["title"]["sha256"], render.digest(evidence["title"]["path"]))


if __name__ == "__main__":
    unittest.main()
