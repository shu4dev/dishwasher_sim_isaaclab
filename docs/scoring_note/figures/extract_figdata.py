"""Data for the scoring-note step figures, printed as a base64 npz between markers.

Runs Kit-free inside the container (Warp scorer + USD):
    scripts/run_py.sh docs/scoring_note/figures/extract_figdata.py > /tmp/out.txt
make_figures.py (host) decodes it. Scores the hard_s0 goal load with the revision-5 scorer at full
resolution and keeps, per dish kind, the food-contact triangles, the visual mesh and the 500 dots
(own frame), and per dish its id, kind, area, exposure E and the 500 dot scores e.
"""
import base64
import io
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "frigidaire/scripts/experiment"))
import frigidaire_bench as B  # noqa: E402

sc = B.Scorer()
E, hx = sc.E, sc.hx
inst = json.loads((ROOT / "results/benchmark/frigidaire_hotec/instances/hard/hard_s0.json").read_text())
kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
entries = [{"id": oid, "kind": kinds[oid], "rack": g["rack"],
            "position": g["settled_rack_local_pose"]["position_m"],
            "quaternion_xyzw": g["settled_rack_local_pose"]["quaternion_xyzw"]}
           for oid, g in inst["goal"]["objects"].items()]
basket = (np.asarray(hx.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
N, K = E.DEFAULTS["samples_per_object"], E.DEFAULTS["directions"]
res = E.score_arrangement(E.Arrangement("fig", "", "", hx.world_objects(entries), basket),
                          device=sc.device, samples=N, directions=K, baselines=False)

ref = {o["id"]: o["exposure"] for o in inst["goal"]["per_object_ref"]}
drift = max(abs(o["exposure"] - ref[o["id"]]) for o in res["objects"])
print(f"[INFO] S {res['score']:.6f} (S_ref {inst['goal']['S_ref']:.6f}), max |E - E_ref| {drift:.2e}", file=sys.stderr)

out = {"S": res["score"], "W": res["worst"], "N": N, "K": K,
       "ids": np.array([o["id"] for o in res["objects"]]),
       "kinds": np.array([kinds[o["id"]] for o in res["objects"]]),
       "racks": np.array([o["rack"] for o in res["objects"]]),
       "areas": np.array([o["area_m2"] for o in res["objects"]]),
       "E": np.array([o["exposure"] for o in res["objects"]]),
       "e": res["samples"]["exposure"].reshape(len(res["objects"]), N).astype(np.float32)}
for kind in ("plate", "bowl", "cup"):
    name = hx.SCORER_KIND[kind]
    fc = E.food_contact(name)
    s = E.surface_samples(fc, N)
    out[f"{kind}_food"] = fc.triangles.astype(np.float32)
    out[f"{kind}_visual"] = np.concatenate(E.dish_visuals(name)).astype(np.float32)
    out[f"{kind}_pts"] = s.points.astype(np.float32)
    out[f"{kind}_nrm"] = s.normals.astype(np.float32)
buf = io.BytesIO()
np.savez_compressed(buf, **out)
print("NPZ_BEGIN")
print(base64.b64encode(buf.getvalue()).decode())
print("NPZ_END")
