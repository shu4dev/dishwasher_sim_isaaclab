"""The benchmark package, standard library only: the manifest writers regenerate schema-valid manifests, the sample
file's four blocks, stage ledger and index (code/planner/bench.py --to load, then a --variant rerun that must not
overwrite the baseline) and the level labels of explicit rosters."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "code/src"))
from bench import loader  # noqa: E402


def cli():
    spec = importlib.util.spec_from_file_location("bench_cli", ROOT / "code/planner/bench.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

FRIG = ROOT / "data/assets/frigidaire_fdpc4221as"
HOTEC = ROOT / "data/assets/hotec_wheatstraw"
needs_assets = pytest.mark.skipif(not ((FRIG / "parameters.json").is_file() and (HOTEC / "catalog.json").is_file()),
                                  reason="data/assets (the HF dataset) not present")


def writer(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"code/src/bench/writers/{name}.py")
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
def test_sample_ledger_and_variant_rerun(tmp_path, monkeypatch):
    machine, dishes = manifests()
    (tmp_path / "assets" / machine["name"]).mkdir(parents=True)
    (tmp_path / "assets" / machine["name"] / "machine.json").write_text(json.dumps(machine))
    (tmp_path / "assets" / "hotec_wheatstraw").mkdir()
    for kind, dish in dishes.items():
        (tmp_path / "assets" / "hotec_wheatstraw" / f"{kind}.json").write_text(json.dumps(dish))
    monkeypatch.setattr(loader, "ASSETS", tmp_path / "assets")
    bench = cli()
    bench.main(["--to", "load", "--out", str(tmp_path / "out"), "--seed", "3"])
    path = tmp_path / "out" / "frigidaire_fdpc4221as__hotec_wheatstraw__plate8-bowl8-cup8_a1__s3.json"
    sample = loader.validate(json.loads(path.read_text()), "sample")
    meta = sample["metadata"]
    assert sorted(sample) == ["dag", "goal", "initial", "metadata"] and meta["sample_id"] == path.stem
    assert [(s["stage"], s["result"]) for s in meta["stages"]] == [("load", "PASS")]
    assert meta["params"]["count"] == {"plate": 8, "bowl": 8, "cup": 8}
    assert meta["params"]["levels"] == {"kinds": "hard", "count": "hard", "allowance": "hard"}
    assert meta["provenance"]["load"]["sites"]["cup"][0] == "glass_left_0"
    bench.main(["--from", "load", "--to", "load", "--input", str(path), "--variant", "graph=none"])
    assert len(json.loads(path.read_text())["metadata"]["stages"]) == 1        # the baseline is never overwritten
    tagged = loader.validate(json.loads(path.with_name(f"{path.stem}__graph=none.json").read_text()), "sample")["metadata"]
    assert [s["stage"] for s in tagged["stages"]] == ["load", "load"] and tagged["variant"] == "graph=none"
    assert tagged["sample_id"] == f"{path.stem}__graph=none"
    index = [json.loads(line) for line in (path.parent / "index.jsonl").read_text().splitlines()]
    assert [i["file"] for i in index] == [path.name, f"{path.stem}__graph=none.json"]
    assert index[0]["last_stage"] == {"stage": "load", "result": "PASS"} and index[0]["dag_edges"] is None


def test_level_labels_come_from_the_explicit_roster():
    from bench import difficulty
    easy = difficulty.resolve(["bowl"], {"bowl": 7}, counter_allowance=3)
    assert easy["levels"] == {"kinds": "easy", "count": "easy", "allowance": "easy"}
    medium = difficulty.resolve(["plate", "bowl"], {"plate": 8, "bowl": 7}, counter_allowance=2)
    assert medium["levels"] == {"kinds": "medium", "count": "medium", "allowance": "medium"}
    hard = difficulty.resolve(["plate", "bowl", "cup"], {"plate": 8, "bowl": 7, "cup": 8}, counter_allowance=1)
    assert hard["levels"] == {"kinds": "hard", "count": "hard", "allowance": "hard"} and hard["on_counter"] is None
    mixed = difficulty.resolve(["plate"], {"plate": 8, "bowl": 7}, counter_allowance=5)
    assert mixed["count"] == {"plate": 8} and mixed["levels"] == {"kinds": "easy", "count": "easy", "allowance": None}
    assert difficulty.tag(hard) == "plate8-bowl7-cup8_a1"
    for count, (lo, hi) in (({"bowl": 7}, (2, 5)), ({"plate": 8, "bowl": 7}, (4, 11)), ({"plate": 8, "bowl": 7, "cup": 8}, (6, 17))):
        ns = {difficulty.draw_n(count, s) for s in range(200)}
        assert min(ns) >= lo and max(ns) <= hi and len(ns) > 3
    assert difficulty.draw_n({"bowl": 7}, 3) == difficulty.draw_n({"bowl": 7}, 3)   # from (roster, seed) only
