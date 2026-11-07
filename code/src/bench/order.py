"""Kit-free build order of a restricted packing (plans/2026-10-22-dag-build-order.md): the packing's access and support
constraints at its own poses, decided before any drop, as a DAG; a topological order of it, or the cycle that makes the
packing unbuildable (the packer then draws again).

Access (a before b): b's packed pose covers a's insertion path (frigidaire_dependency_kit.access_hits, the move's 1 mm
path check, here on the packing instead of the built load). Support (a before b): b rests on a, i.e. b lowered by
SUPPORT_GAP_M plus the checker's 1 mm allowance meets a past that allowance while b at its pose does not: b's underside
lies within 2 mm above a's surface, inside b's footprint, on the same FCL pieces (the bench checker). Rack-local entries
are placed in the bench baseline's extended racks, as the dependency kit's goal_setup does. Needs numpy, fcl and pxr
(code/util/run_py.sh), imported on first use.
pose_model "rest" (plans/2026-10-22-plate-rest-pose-dag.md, plans/2026-10-23-side-zone-chain-edges.md): each plate at the
measured rest pose of its slot and move side (REST_TABLE, frigidaire_bench.move_side; the 6.2 table since 2026-11-03,
plans/2026-11-03-rear-plate-drop.md), each upper-rack bowl and cup at the measured rest pose of its
bowl_key (BOWL_TABLE, both from frigidaire_plate_rest.py; a key the table lacks keeps its packed pose), lower-rack bowls at
their packed poses, no support edges. Access pairs both ways (a dish whose pose covers another's descent to its move pose
puts that other first) for every pair with a plate and every pair of upper-rack bowls and cups; no edge between two
dishes when either is a lower-rack bowl, in the side zones and the rear row (lower_rear) alike.
"""
import json
from pathlib import Path
import random
import sys

from . import ROOT

SUPPORT_GAP_M = .002
BASELINE = ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json"     # the dependency kit's --baseline
REST_TABLE = ROOT / "data/results/sampler/plate_rest_62/plate_rest.json"             # frigidaire_plate_rest.py --out (6.2)
BOWL_TABLE = ROOT / "data/results/sampler/bowl_rest/bowl_rest.json"                  # ... --kinds bowl,cup --out


def _mods():
    """(frigidaire_bench, frigidaire_bench_kit, frigidaire_dependency_kit, dishsim_frigidaire.planner); the kits are
    Kit-free at import."""
    path = str(ROOT / "code/planner/frigidaire")
    if path not in sys.path:
        sys.path.append(path)
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    import frigidaire_dependency_kit as DK
    from dishsim_frigidaire import planner as P
    return B, K, DK, P


def _world(entries, frames=None, at=None, side=None):
    """(poses, moves, kinds, racks), world: each entry at its pose in ``at`` (entries by id; default its packed pose) and
    at its insertion move pose in the racks of ``frames`` (the five appliance component poses, included in ``poses``;
    default the bench baseline's); ``side`` = the plate move side rule (move_side_of)."""
    B, _, _, P = _mods()
    frames = frames or json.loads(BASELINE.read_text())["snapshot"]["poses"]
    at = {e["id"]: e for e in (at or entries)}
    poses, moves, kinds, racks = dict(frames), {}, {}, {}
    for e in entries:
        f, a = frames[e["rack"]], at[e["id"]]
        poses[e["id"]] = P.world_from_local(f, {"position_m": list(a["position"]),
                                                "quaternion_xyzw": list(a["quaternion_xyzw"])})
        moves[e["id"]] = P.world_from_local(f, B.move_local(e, side))
        kinds[e["id"]], racks[e["id"]] = e["kind"], e["rack"]
    return poses, moves, kinds, racks


def plate_rest_poses(table=REST_TABLE):
    """{(slot, side): rack-local {position, quaternion_xyzw}} of the lone-plate rest table (side: the move's, +1 = +x)."""
    rows = json.loads(Path(table).read_text())["entries"]
    return {(r["slot"], r["side"]): {"position": r["position"], "quaternion_xyzw": r["quaternion_xyzw"]} for r in rows}


def bowl_key(entry):
    """'kind|slot|variant' of a bowl or cup entry (or family candidate), the variant without its lift and, in the lower
    rack, without its x: the bowl table's key."""
    v = entry["variant"].split("_lift", 1)[0]
    return f"{entry['kind']}|{entry['slot']}|{v.rsplit('_x', 1)[0] if entry['slot'].startswith('lower_') else v}"


def bowl_rest_poses(table=BOWL_TABLE):
    """{bowl_key: {position, quaternion_xyzw, x_m}} of the lone bowl and cup rest table ({} when it is absent)."""
    if not Path(table).is_file():
        return {}
    return {r["key"]: {"position": r["position"], "quaternion_xyzw": r["quaternion_xyzw"], "x_m": r["x_m"]}
            for r in json.loads(Path(table).read_text())["entries"] if "position" in r}


def move_side_of(record):
    """The plate move side rule a build record was made under: frigidaire_bench.lean_side (3 mm toward the packed lean's
    side in both banks) when its release version is before 2026-11-03 (or missing), else frigidaire_bench.move_side (the
    rear bank toward the other side, plans/2026-11-03-rear-plate-drop.md)."""
    B = _mods()[0]
    if str((record.get("release") or {}).get("version") or "") < "2026-11-03":
        return lambda entry: B.lean_side(entry["variant"])
    return B.move_side


def rest_entries(entries, table=REST_TABLE, bowls=BOWL_TABLE, side=None):
    """``entries`` with every plate at the plate table's rest pose of its slot and move side (``side``, default
    frigidaire_bench.move_side; move_side_of for a record) and every upper-rack bowl and
    cup at the bowl table's rest pose of its key, moved along the rack x by the entry's x minus the row's; lower-rack bowls
    and keys the bowl table lacks keep their packed pose."""
    B, rest, bowl = _mods()[0], plate_rest_poses(table), bowl_rest_poses(bowls)
    out = []
    for e in entries:
        r = (rest[(e["slot"], (side or B.move_side)(e))] if e["kind"] == "plate" else
             None if e["slot"].startswith("lower_") else bowl.get(bowl_key(e)))
        if r is not None and e["kind"] != "plate":
            r = {"position": [r["position"][0] + e["position"][0] - r["x_m"], *r["position"][1:]],
                 "quaternion_xyzw": r["quaternion_xyzw"]}
        out.append(e if r is None else {**e, **r})
    return out


def access_edges(entries, frames=None, pose_model="packed", side=None):
    """[(a, b)]: a goes in before b, because b's pose (packed, or the rest model's) covers a's insertion path (to its move
    pose, from its packed entry either way); in the rest model the lower-rack bowl pairs get no edge. ``side`` = the plate
    move side rule (default move_side; move_side_of(record) for an older record)."""
    B, _, DK, _ = _mods()
    poses, moves, kinds, racks = _world(entries, frames, rest_entries(entries, side=side) if pose_model == "rest" else None,
                                        side)
    hits, _ = DK.access_hits(B, B.release_checker(), kinds, racks, moves, poses, [e["id"] for e in entries])
    edges = {(h["a"], h["b"]) for h in hits}
    if pose_model == "rest":
        low = {e["id"] for e in entries if e["kind"] != "plate" and e["slot"].startswith("lower_")}
        edges = {(a, b) for a, b in edges if "plate" in (kinds[a], kinds[b]) or not {a, b} & low}
    return sorted(edges)


def support_edges(entries, frames=None):
    """[(a, b)]: a goes in before b, because b rests on a at the packing poses (module docstring)."""
    B, _, _, _ = _mods()
    poses, _, kinds, _ = _world(entries, frames)
    checker, ids = B.release_checker(), [e["id"] for e in entries]
    drop = SUPPORT_GAP_M + checker.penetration_limit_m
    at = {i: checker._body(kinds[i], poses[i], i) for i in ids}
    low = {i: checker._body(kinds[i], B.lifted(poses[i], -drop, (0., 0., 1.)), i) for i in ids}
    return [(a, b) for b in ids for a in ids
            if a != b and checker.pair(at[b], at[a])["valid"] and not checker.pair(low[b], at[a])["valid"]]


def dag_order(entries, seed, frames=None, pose_model="packed", side=None):
    """{order, access, support, layers, cycle}: a topological order of the access and support edges, ties broken by the
    lean-side chain rule within a plate bank (frigidaire_bench_kit.chain_order over a seeded permutation), then by that
    permutation; layers = the longest prerequisite chain + 1. On a cycle: order and layers None, cycle = its dishes in
    edge order (each before the next, the last before the first)."""
    B, K, _, _ = _mods()
    access = access_edges(entries, frames, pose_model, side)
    support = [] if pose_model == "rest" else support_edges(entries, frames)
    by_id = {e["id"]: e for e in entries}
    perm = list(by_id)
    random.Random(seed).shuffle(perm)
    rank = {i: n for n, i in enumerate(K.chain_order(perm, by_id, B.lean_side)[0])}
    need = {i: {a for a, b in {*access, *support} if b == i} for i in by_id}
    order, layer = [], {}
    while True:
        ready = [i for i in by_id if i not in layer and need[i] <= layer.keys()]
        if not ready:
            break
        i = min(ready, key=rank.get)
        layer[i] = 1 + max((layer[a] for a in need[i]), default=-1)
        order.append(i)
    out = {"access": access, "support": support, "order": None, "layers": None, "cycle": None}
    if len(order) < len(by_id):                  # every dish left waits on another left: walk back until one repeats
        i, walk = min(set(by_id) - layer.keys(), key=rank.get), []
        while i not in walk:
            walk.append(i)
            i = min(a for a in need[i] if a not in layer)
        return {**out, "cycle": walk[walk.index(i):][::-1]}
    return {**out, "order": order, "layers": 1 + max(layer.values(), default=-1)}
