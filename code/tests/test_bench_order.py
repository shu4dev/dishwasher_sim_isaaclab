"""bench/order.py on r05_00027, the one valid physical-cause sample (needs numpy, fcl and pxr: code/util/run_py.sh on the
box, with the record and the bench baseline under data/results): access_edges at the packing poses reproduces the
record's 11 access hits, and dag_order returns an order whose DAG the record's witness respects
(plans/2026-10-22-dag-build-order.md). The packing is the one the witness built: the restricted load
data/results/dot_rules/restricted/loads/r05_00027.json with the record's 9 re-poses (its entries_chosen)."""
import json
from pathlib import Path
import sys

import pytest

for _module in ("numpy", "fcl", "pxr"):
    pytest.importorskip(_module)
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
RECORD = next((p for p in (ROOT / f"data/results/dot_rules/dependency/{d}/r05_00027.json"
                           for d in ("samples_v5", "smoke_20261022")) if p.is_file()), None)
if RECORD is None or not (ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json").is_file():
    pytest.skip("r05_00027's record or the bench baseline (data/results) not present", allow_module_level=True)
from bench import order  # noqa: E402


def test_r05_00027_access_edges_and_witness():
    rec = json.loads(RECORD.read_text())
    entries = rec["entries_chosen"]
    side = order.move_side_of(rec)                                   # the record's move rule (2026-10-15b: +x in both banks)
    assert order.access_edges(entries, side=side) == sorted({(h["a"], h["b"]) for h in rec["access"]["hits"]})
    dag = order.dag_order(entries, 0, side=side)
    pos = {i: n for n, i in enumerate(rec["witness"]["order"])}
    assert dag["cycle"] is None and sorted(dag["order"]) == sorted(pos)
    assert all(pos[a] < pos[b] for a, b in dag["access"] + dag["support"])


def test_r05_00027_rest_model_plate_edges():
    """plans/2026-10-22-plate-rest-pose-dag.md: with the plates at the lone-plate rest table (order.REST_TABLE), the
    rest model's plate-pair access edges are the record's 9 plate hits and its DAG is acyclic. The record was built under
    the 2026-10-15b release (plates 3 mm toward +x in both banks), so the model uses that side (order.move_side_of)."""
    if not order.REST_TABLE.is_file():
        pytest.skip("the plate rest table (frigidaire_plate_rest.py --out data/results/sampler/plate_rest_62) not present")
    rec = json.loads(RECORD.read_text())
    entries = rec["entries_chosen"]
    plate = {e["id"] for e in entries if e["kind"] == "plate"}
    dag = order.dag_order(entries, 0, pose_model="rest", side=order.move_side_of(rec))
    assert sorted(e for e in dag["access"] if set(e) <= plate) == sorted(
        (h["a"], h["b"]) for h in rec["access"]["hits"] if {h["a"], h["b"]} <= plate)
    assert dag["cycle"] is None and dag["support"] == []
    pos = {i: n for n, i in enumerate(rec["witness"]["order"])}      # plans/2026-10-22-bowl-plate-edges.md: the bowl-plate
    assert all(pos[a] < pos[b] for a, b in dag["access"])            # edges too, none against the witness order


def test_r05_00027_rest_model_bowl_edges():
    """plans/2026-10-23-side-zone-chain-edges.md: the rest model's upper-rack bowl and cup pairs are the path check at the
    bowl table (order.BOWL_TABLE); on r05_00027 they equal the record's upper hits and the DAG stays acyclic."""
    if not (order.REST_TABLE.is_file() and order.BOWL_TABLE.is_file()):
        pytest.skip("the plate or bowl rest table (frigidaire_plate_rest.py) not present")
    rec = json.loads(RECORD.read_text())
    entries = rec["entries_chosen"]
    low = {e["id"] for e in entries if e["kind"] != "plate" and e["slot"].startswith("lower_")}
    up = {e["id"] for e in entries if e["kind"] != "plate"} - low
    hits = {(h["a"], h["b"]) for h in rec["access"]["hits"]}
    dag = order.dag_order(entries, 0, pose_model="rest", side=order.move_side_of(rec))
    assert sorted(e for e in dag["access"] if set(e) <= up) == sorted(e for e in hits if set(e) <= up)
    assert dag["cycle"] is None
