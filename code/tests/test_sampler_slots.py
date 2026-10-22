"""The slot finder reproduces the HOTEC families' sites on the Frigidaire (needs numpy and pxr: run_py.sh on the box).

Reference: frigidaire_hotec_exposure_search.candidate_families, the families the top-5 run (frigidaire_hotec_top5.py
level_families) filters by level. Same slot labels per kind, and the sites' centres (rack-local metres):
- plates: x = the gap's tine midpoint and y = the bank centreline (geometry, 1e-9 m); every family plate sits on its
  site's y, the front bank FRONT_PLATE_INSET_Y (10 mm) rearward (1e-9 m);
- lower-rack bowls: every family bowl's y inside its zone's two tine rows +- 10 mm (the families' own row offsets);
- upper-rack bowls: y = the gap's tine midpoint (1e-9 m); every family bowl within 20 mm (the families' offsets);
- cups: |x| = the glass channel's (1e-9 m); every family cup on its site's y (1e-9 m).
"""
import importlib.util
from pathlib import Path
import sys

import pytest

pytest.importorskip("numpy")
pytest.importorskip("pxr")
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
if not (ROOT / "data/assets/frigidaire_fdpc4221as/parameters.json").is_file():
    pytest.skip("data/assets (the HF dataset) not present", allow_module_level=True)
from sampler import loader, slots  # noqa: E402

EPS = 1e-9


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_finder_reproduces_hotec_family_sites():
    from dishsim_frigidaire import geometry
    B = module("frigidaire_bench", "code/planner/frigidaire/frigidaire_bench.py")
    HX = B.HX()
    fam, _, _ = HX.candidate_families(B.ASSETS)
    machine = loader.validate(module("wm", "code/src/sampler/writers/frigidaire_fdpc4221as.py").build(
        ROOT / "data/assets/frigidaire_fdpc4221as"), "machine")
    dishes = module("wd", "code/src/sampler/writers/hotec_wheatstraw.py").build(ROOT / "data/assets/hotec_wheatstraw")
    found = slots.find(machine, dishes)
    for kind in ("plate", "bowl", "cup"):
        assert {s["slot"] for s in found[kind]} == {c["slot"] for c, _ in fam[kind]}, kind
    site = {s["slot"]: s for kind in found for s in found[kind]}
    teeth, rows = geometry.lower_tine_positions()
    _, upper_ys = geometry.upper_tine_positions()
    for s in found["plate"]:
        gap, front = int(s["slot"][-2:]), "_front_" in s["slot"]
        assert abs(s["x_m"] - (teeth[gap] + teeth[gap + 1]) / 2) <= EPS
        assert abs(s["y_m"] - ((rows[0] + rows[1]) / 2 if front else (rows[-2] + rows[-1]) / 2)) <= EPS
    for c, _ in fam["plate"]:
        inset = HX.HL.FRONT_PLATE_INSET_Y if "_front_" in c["slot"] else 0.
        assert abs(c["position"][1] - site[c["slot"]]["y_m"] - inset) <= EPS
    for c, _ in fam["bowl"]:
        s = site[c["slot"]]
        if c["rack"] == "LowerRack":
            assert s["band_y_m"][0] - .010 - EPS <= c["position"][1] <= s["band_y_m"][1] + .010 + EPS, c["slot"]
        else:
            a = int(c["slot"].rsplit("_", 1)[1])
            b = next(j for j in range(a + 1, len(upper_ys)) if geometry.upper_tine_present(1, j))
            assert abs(s["y_m"] - (upper_ys[a] + upper_ys[b]) / 2) <= EPS
            assert abs(c["position"][1] - s["y_m"]) <= .020 + EPS
    for c, _ in fam["cup"]:
        s = site[c["slot"]]
        assert abs(abs(s["x_m"]) - HX.HL._GLASS) <= EPS and abs(c["position"][1] - s["y_m"]) <= EPS
