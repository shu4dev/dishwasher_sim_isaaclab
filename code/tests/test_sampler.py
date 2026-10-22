"""Sampler, standard library only: the manifest writers regenerate schema-valid manifests, and the pair file's
stage ledger (sample.py --to load, then a --variant rerun that must not overwrite the baseline)."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "code/src"))
from sampler import loader, sample  # noqa: E402

FRIG = ROOT / "data/assets/frigidaire_fdpc4221as"
HOTEC = ROOT / "data/assets/hotec_wheatstraw"
needs_assets = pytest.mark.skipif(not ((FRIG / "parameters.json").is_file() and (HOTEC / "catalog.json").is_file()),
                                  reason="data/assets (the HF dataset) not present")


def writer(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"code/src/sampler/writers/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def manifests():
    return writer("frigidaire_fdpc4221as").build(FRIG), writer("hotec_wheatstraw").build(HOTEC)


@needs_assets
def test_manifests_regenerate_and_validate():
    machine, dishes = manifests()
    loader.validate(machine, "machine")
    assert {c["kind"] for c in machine["containers"]} == {"rack", "basket"}      # basket admitted, unimplemented
    assert sorted(dishes) == ["bowl", "cup", "plate"]
    for dish in dishes.values():
        loader.validate(dish, "dish")
        assert dish["count"] == 8
    with pytest.raises(ValueError, match="missing 'containers'"):
        loader.validate({k: v for k, v in machine.items() if k != "containers"}, "machine")
    with pytest.raises(ValueError, match="not in"):
        loader.validate({**dishes["cup"], "kind": "saucer"}, "dish")


@needs_assets
def test_pair_ledger_and_variant_rerun(tmp_path, monkeypatch):
    machine, dishes = manifests()
    (tmp_path / "assets" / machine["name"]).mkdir(parents=True)
    (tmp_path / "assets" / machine["name"] / "machine.json").write_text(json.dumps(machine))
    (tmp_path / "assets" / "hotec_wheatstraw").mkdir()
    for kind, dish in dishes.items():
        (tmp_path / "assets" / "hotec_wheatstraw" / f"{kind}.json").write_text(json.dumps(dish))
    monkeypatch.setattr(loader, "ASSETS", tmp_path / "assets")
    sample.main(["--to", "load", "--out", str(tmp_path / "out"), "--seed", "3"])
    path = tmp_path / "out" / "frigidaire_fdpc4221as__hotec_wheatstraw__3.json"
    pair = loader.validate(json.loads(path.read_text()), "pair")
    assert [(s["stage"], s["result"]) for s in pair["stages"]] == [("load", "PASS")]
    assert pair["params"]["count"] == {"plate": 8, "bowl": 8, "cup": 8}
    assert pair["load"]["sites"]["cup"][0] == "glass_left_0"
    sample.main(["--from", "load", "--to", "load", "--input", str(path), "--variant", "graph=none"])
    assert len(json.loads(path.read_text())["stages"]) == 1                    # the baseline is never overwritten
    tagged = loader.validate(json.loads(path.with_name(f"{path.stem}__graph=none.json").read_text()), "pair")
    assert [s["stage"] for s in tagged["stages"]] == ["load", "load"] and tagged["variant"] == "graph=none"


def test_difficulty_preset_and_overrides():
    from sampler import difficulty
    hard = difficulty.resolve("hard")
    assert hard["count"] == {"plate": 8, "bowl": 7, "cup": 8} and hard["counter_allowance"] == 1
    mixed = difficulty.resolve("medium", kinds=["plate"], counter_allowance=5)
    assert mixed["count"] == {"plate": 8} and mixed["counter_allowance"] == 5
