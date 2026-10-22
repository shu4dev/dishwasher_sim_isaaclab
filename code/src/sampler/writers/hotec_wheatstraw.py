#!/usr/bin/env python3
"""Write {plate,bowl,cup}.json (the sampler's dish manifests, schema code/src/sampler/schema/dish.json) for the HOTEC set.

Belongs in data/assets/hotec_wheatstraw/source/write_manifest.py (the HF dataset); staged in the repo because the
refactor run could not write the dataset copy. Standard library only: usd, prim, size, collider count from catalog.json,
mass from parameters.json, the count per kind from the catalog's set line. The allowed orientations name the slot-finder
mode per rack and copy the HOTEC families' zone rules (frigidaire_hotec_exposure_search.candidate_families,
frigidaire_hotec_load.py, dishsim_frigidaire/loading.py) with their source; food-contact samples are not written yet.

    python3 code/src/sampler/writers/hotec_wheatstraw.py [--asset-dir data/assets/hotec_wheatstraw] [--out-dir <asset-dir>]
"""
import argparse
import json
from pathlib import Path
import re

REPO = Path(__file__).absolute().parents[4]
FAMILIES = "code/planner/frigidaire/frigidaire_hotec_exposure_search.py candidate_families"
ORIENTATIONS = {
    "plate": [{"container_kind": "rack", "rack": "LowerRack", "mode": "between_tines",
               # front gaps whose mid x stays 20 mm ahead of the basket bay (frigidaire_hotec_load._FRONT_USABLE)
               "params": {"banks": ["front", "rear"], "front_keep_out_margin_m": .020},
               "source": f"{FAMILIES}; frigidaire_hotec_load._FRONT_USABLE"}],
    "bowl": [{"container_kind": "rack", "rack": "LowerRack", "mode": "over_tines",
              "params": {"zones": {"frontright": "front", "rear": "rear", "mid": "mid", "rearright": "rear"}},
              "source": f"{FAMILIES}; dishsim_frigidaire.loading.lower_bowl_variants"},
             {"container_kind": "rack", "rack": "UpperRack", "mode": "centre_gaps",
              "params": {"column": 1, "regular_only": False},
              "source": f"{FAMILIES}; frigidaire_hotec_load.UPPER_BOWL_GAPS"}],
    "cup": [{"container_kind": "rack", "rack": "UpperRack", "mode": "glass_channel",
             "params": {"sides": [-1, 1]}, "source": f"{FAMILIES}; frigidaire_hotec_load.CUP_SLOTS (+ 0 and 5 in search)"}],
}


def build(asset):
    asset = Path(asset)
    catalog = json.loads((asset / "catalog.json").read_text())
    params = json.loads((asset / "parameters.json").read_text())["kinds"]
    out = {}
    for kind, item in catalog["items"].items():
        count = re.search(rf"(\d+) {kind}s?\b", catalog["set"])
        out[kind] = {"schema_version": 1, "kind": kind, "dish_set": asset.name, "usd": f"usd/{item['usd']}",
                     "default_prim": item["default_prim"], "visual_mesh": None,
                     "collision_pieces": item["convex_colliders"], "mass_kg": params[kind]["mass_kg"]["value"],
                     "origin": "base_centre", "footprint_m": {"diameter": item["actual_size_m"][0], "height": item["actual_size_m"][2]},
                     "count": int(count.group(1)), "colours": catalog["variant_set"]["variants"],
                     "food_contact": None, "pooling_ring": None, "allowed_orientations": ORIENTATIONS[kind],
                     "provenance": {"catalog.json": f"items.{kind} (usd, prim, size, colliders), set (count), variant_set",
                                    "parameters.json": f"kinds.{kind}.mass_kg ({params[kind]['mass_kg']['status']})",
                                    "origin": catalog["origin"]}}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--asset-dir", type=Path, default=REPO / "data/assets/hotec_wheatstraw")
    ap.add_argument("--out-dir", type=Path, default=None, help="default <asset-dir>")
    args = ap.parse_args()
    out_dir = args.out_dir or args.asset_dir
    for kind, manifest in build(args.asset_dir).items():
        (out_dir / f"{kind}.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"[RESULT] PASS wrote {sorted(build(args.asset_dir))} to {out_dir}")


if __name__ == "__main__":
    main()
