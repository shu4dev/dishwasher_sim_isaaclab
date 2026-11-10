"""graph.start_dag, standard library only (plans/2026-11-10-start-dependency-graph.md): the probe table of a three-dish
counter stack (bowl_01 on bowl_02 on bowl_05) reduces to a two-edge support chain from the top dish down, in the schema's
initial.dag shape."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
from sampler import loader  # noqa: E402
from sampler.graph import start_dag  # noqa: E402


def test_three_dish_stack_is_a_two_edge_chain():
    ids = ["bowl_01", "bowl_02", "bowl_05"]
    moved = {("support", "bowl_05"): {"bowl_01", "bowl_02"}, ("support", "bowl_02"): {"bowl_01"},
             ("lift", "bowl_05"): {"bowl_01", "bowl_02"}, ("lift", "bowl_02"): {"bowl_01"}}
    rows = [{"probe": p, "b": b, "a": a, "displaced": a in moved.get((p, b), ())}
            for p in ("support", "lift") for b in ids for a in ids if a != b]
    dag = start_dag({"ids": ids, "rows": rows})
    assert dag["edges"] == [{"a": "bowl_01", "b": "bowl_02", "kind": "support"},
                            {"a": "bowl_02", "b": "bowl_05", "kind": "support"}]
    assert dag["layers"] == {"bowl_01": 0, "bowl_02": 1, "bowl_05": 2} and dag["order"] == ids and dag["conflicts"] == []
    assert loader.errors(dag, loader.schema("sample")["properties"]["initial"]["properties"]["dag"]) == []
