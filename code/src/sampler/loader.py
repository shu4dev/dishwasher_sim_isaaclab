"""The general loader: machine and dish manifests (data/assets/<asset>/machine.json, <kind>.json), checked against
code/src/sampler/schema/. Standard library only.

The schema check covers the draft-07 subset the three schemas use (type, required, properties, items, enum), so no
dependency is added (jsonschema is not in the container image's requirements).
"""
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
ASSETS = REPO / "data/assets"
SCHEMA_DIR = Path(__file__).resolve().parent / "schema"
TYPES = {"object": dict, "array": list, "string": str, "integer": int, "number": (int, float), "boolean": bool,
         "null": type(None)}


def schema(name):
    return json.loads((SCHEMA_DIR / f"{name}.json").read_text())


def errors(value, spec, where="$"):
    """Every violation of ``spec`` (the subset above) in ``value``, as 'path: message' strings."""
    types = spec.get("type")
    if types is not None:
        types = [types] if isinstance(types, str) else types
        ok = any(isinstance(value, TYPES[t]) and not (t in ("integer", "number") and isinstance(value, bool))
                 for t in types)
        if not ok:
            return [f"{where}: {type(value).__name__} is not {'|'.join(types)}"]
    out = []
    if "enum" in spec and value not in spec["enum"]:
        out.append(f"{where}: {value!r} not in {spec['enum']}")
    if isinstance(value, dict):
        out += [f"{where}: missing {key!r}" for key in spec.get("required", ()) if key not in value]
        for key, sub in spec.get("properties", {}).items():
            if key in value:
                out += errors(value[key], sub, f"{where}.{key}")
    if isinstance(value, list) and "items" in spec:
        for i, item in enumerate(value):
            out += errors(item, spec["items"], f"{where}[{i}]")
    return out


def validate(value, name):
    """Raise ValueError listing every violation of schema/<name>.json; returns ``value``."""
    found = errors(value, schema(name))
    if found:
        raise ValueError(f"{name} manifest invalid: " + "; ".join(found[:20]))
    return value


def read_machine(machine, assets=ASSETS):
    """machine.json of data/assets/<machine>, validated; ``dir`` added (the asset folder)."""
    path = Path(assets) / machine / "machine.json"
    return {**validate(json.loads(path.read_text()), "machine"), "dir": str(path.parent)}


def read_dishes(dishes, kinds=None, assets=ASSETS):
    """{kind: <kind>.json} of data/assets/<dishes>, validated; ``kinds`` limits which are read."""
    folder = Path(assets) / dishes
    found = sorted(p.stem for p in folder.glob("*.json") if p.stem not in ("catalog", "parameters", "machine"))
    out = {}
    for kind in kinds or found:
        path = folder / f"{kind}.json"
        if not path.is_file():
            raise FileNotFoundError(f"no dish manifest {path} (run the set's source/ manifest writer)")
        out[kind] = {**validate(json.loads(path.read_text()), "dish"), "dir": str(folder)}
    return out


def stage_load(state, params):
    """load stage (Kit-free): read and validate the manifests, cap the counts by the set's catalog, find the sites."""
    from . import slots
    assets = params.get("assets", ASSETS)
    machine = read_machine(state["machine"], assets)
    dishes = read_dishes(state["dishes"], state["params"]["kinds"], assets)
    over = {k: n for k, n in state["params"]["count"].items() if n > dishes[k]["count"]}
    if over:
        return state, "FAIL", f"count above the dish set's catalog: {over}"
    sites = slots.find(machine, dishes)
    state["load"] = {"machine_dir": machine["dir"], "frame": machine["frame"],
                     "containers": [c["name"] for c in machine["containers"]],
                     "dishes": {k: {"usd": d["usd"], "mass_kg": d["mass_kg"], "catalog_count": d["count"]}
                                for k, d in dishes.items()},
                     "sites": {k: [s["slot"] for s in v] for k, v in sites.items()}}
    return state, "PASS", f"{len(dishes)} kinds, {sum(map(len, sites.values()))} sites"
