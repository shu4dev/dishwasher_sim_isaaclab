#!/usr/bin/env python3
"""Write machine.json (the sampler's machine manifest, schema code/src/sampler/schema/machine.json) for the Frigidaire twin.

Belongs in data/assets/frigidaire_fdpc4221as/source/write_manifest.py (the HF dataset); staged in the repo because the
refactor run could not write the dataset copy. Standard library only. Every value is read from the asset folder's
parameters.json, components.json and usd/example_scene.usda, or parsed (ast, never imported) from the repo constant
named in "provenance"; nothing is typed in here except the cup ladder, copied from the HOTEC families' code.

    python3 code/src/sampler/writers/frigidaire_fdpc4221as.py [--asset-dir data/assets/frigidaire_fdpc4221as] [--out <asset-dir>/machine.json]
"""
import argparse
import ast
import json
from pathlib import Path
import re

REPO = Path(__file__).absolute().parents[4]


def literal(path, name):
    """A module-level literal assignment ``name = ...`` of a repo file, parsed, not imported."""
    for node in ast.parse(Path(path).read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(f"{name} not a literal in {path}")


def lower_grid(g):
    """geometry.lower_tine_positions / lower_tine_mask in plain python (rows = Y banks front to rear, columns = X)."""
    p, b = g["lower_rack"], g["silverware_basket"]
    xs = [-p["wire_width"] / 2 + p["tine_margins"]["left"] + i * p["tine_pitch"]["x"] for i in range(p["tines_per_bank"])]
    ys = [-p["wire_depth"] / 2 + p["tine_margins"]["front"]]
    for pitch in p["tine_pitch"]["y"]:
        ys.append(ys[-1] + pitch)
    seat, c = b["seat"]["origin_in_rack_m"], p["basket_bay"]["clearance"]
    fx = (seat[0] - b["width_x"] / 2, seat[0] + b["width_x"] / 2)
    fy = (seat[1] - b["length_y"] / 2, seat[1] + b["length_y"] / 2)
    present = [[not (fy[0] - c <= y <= fy[1] + c and x + p["tine_tip_offset_x"] >= fx[0] - c and x <= fx[1] + c)
                for x in xs] for y in ys]
    keep_out = [{"name": "basket_bay", "x_m": list(fx), "y_m": list(fy), "clearance_m": c}]
    return {"axis": p["tine_repetition_axis"], "columns_x_m": xs, "rows_y_m": ys, "present": present, "keep_out": keep_out}


def upper_grid(g):
    """geometry.upper_tine_positions / upper_tine_mask (rows = Y positions front to rear, columns = the X banks)."""
    p = g["upper_rack"]
    ys = [-p["wire_depth"] / 2 + p["tine_front_margin"] + i * p["tine_spacing"] for i in range(p["tines_per_bank"])]
    absent = p["tine_absent"]
    present = [[not (col in absent["columns"] and i in absent["indices"]) for col in range(p["tine_banks"])]
               for i in range(len(ys))]
    return {"axis": p["tine_repetition_axis"], "columns_x_m": [float(x) for x in p["tine_bank_x"]], "rows_y_m": ys,
            "present": present, "keep_out": []}


def build(asset, repo=REPO):
    asset, repo = Path(asset), Path(repo)
    params = json.loads((asset / "parameters.json").read_text())
    g = params["geometry"]
    sites = {name: c["sites"] for name, c in json.loads((asset / "components.json").read_text()).items()}
    scene = (asset / "usd/example_scene.usda").read_text()
    root = "/" + "/".join(re.findall(r'def Xform "(\w+)"', scene)[:2])          # /World/<asset prim referencing the usdc>
    hotec_load = repo / "code/planner/frigidaire/frigidaire_hotec_load.py"
    bench = repo / "code/planner/frigidaire/frigidaire_bench.py"
    exposure = repo / "code/frigidaire/src/dishsim_frigidaire/exposure.py"
    runtime = repo / "code/frigidaire/src/dishsim_frigidaire/random_pose_runtime.py"
    boxes, arms, counter = literal(hotec_load, "RACK_BOX"), literal(exposure, "ARM_SOURCES"), literal(bench, "COUNTER")
    extension = literal(runtime, "EXTENSION")
    limits = params["joint_limits_usd"]
    arm_of = {rack: {"name": arm, "center_m": list(spec["center"]), "radius_m": spec["radius"], "jet_angle_deg": None}
              for arm, spec in arms.items() for rack in spec["racks"]}
    b = g["silverware_basket"]
    seat = b["seat"]["origin_in_rack_m"]
    containers = [
        {"name": "LowerRack", "kind": "rack", "link": "LowerRack", "joint": "lower_slide", "extended_frame": "LowerRack",
         "box_m": [list(v) for v in boxes["LowerRack"]], "rim_height_m": g["lower_rack"]["rim_height"],
         "tine_grid": lower_grid(g), "spray_arm": arm_of["LowerRack"], "collision_cache": None},
        {"name": "UpperRack", "kind": "rack", "link": "UpperRack", "joint": "upper_slide", "extended_frame": "UpperRack",
         "box_m": [list(v) for v in boxes["UpperRack"]], "rim_height_m": g["upper_rack"]["rim_height"],
         "tine_grid": upper_grid(g), "spray_arm": arm_of["UpperRack"], "collision_cache": None,
         # cup ladder: the HOTEC families' glass slots, y = -.210 + .085 * index (frigidaire_hotec_exposure_search.py)
         "glass_channels": {"x_abs_m": abs(sites["UpperRack"]["right_glass_channel"][0]),
                            "ladder_y0_m": -.210, "ladder_pitch_m": .085, "ladder_count": 6}},
        {"name": "SilverwareBasket", "kind": "basket", "link": "SilverwareBasket", "joint": None,
         "extended_frame": "LowerRack",
         "box_m": [[seat[0] - b["width_x"] / 2, seat[0] + b["width_x"] / 2],
                   [seat[1] - b["length_y"] / 2, seat[1] + b["length_y"] / 2], [seat[2], seat[2] + b["body_height"]]],
         "rim_height_m": b["body_height"], "spray_arm": arm_of["SilverwareBasket"], "collision_cache": None}]
    joints = [{"name": "door_hinge", "kind": "revolute", "in": limits["door_hinge"][0], "out": limits["door_hinge"][1], "units": "deg"},
              {"name": "lower_slide", "kind": "prismatic", "in": 0.0, "out": extension["LowerRack"], "units": "m"},
              {"name": "upper_slide", "kind": "prismatic", "in": 0.0, "out": extension["UpperRack"], "units": "m"}]
    cs = g["coordinate_system"]
    return {"schema_version": 1, "name": asset.name, "usd": "usd/fdpc4221as.usdc", "root_prim": root,
            "units": cs["units"], "up_axis": cs["up"], "front": cs["front"], "frame": "machine",
            "body_positions_m": params["body_positions_m"], "joints": joints, "containers": containers,
            "counter": {k: list(v) if isinstance(v, tuple) else v for k, v in counter.items()},
            "provenance": {"parameters.json": "geometry.lower_rack / upper_rack / silverware_basket, joint_limits_usd, body_positions_m",
                           "components.json": "UpperRack.sites.right_glass_channel (glass channel |x|)",
                           "usd/example_scene.usda": "root prim",
                           str(hotec_load.relative_to(repo)): "RACK_BOX (rack-local containment boxes)",
                           "code/planner/frigidaire/frigidaire_hotec_exposure_search.py": "cup ladder y0 / pitch / count (candidate_families)",
                           str(bench.relative_to(repo)): "COUNTER (worktop slab)",
                           str(exposure.relative_to(repo)): "ARM_SOURCES (spray-arm discs); jet angle not recorded there",
                           str(runtime.relative_to(repo)): "EXTENSION (rack 'out' joint values)"}}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--asset-dir", type=Path, default=REPO / "data/assets/frigidaire_fdpc4221as")
    ap.add_argument("--out", type=Path, default=None, help="default <asset-dir>/machine.json")
    args = ap.parse_args()
    out = args.out or args.asset_dir / "machine.json"
    out.write_text(json.dumps(build(args.asset_dir), indent=1) + "\n")
    print(f"[RESULT] PASS wrote {out}")


if __name__ == "__main__":
    main()
