"""Kit-free build order of a restricted packing (plans/2026-10-22-dag-build-order.md): the packing's access and support
constraints at its own poses, decided before any drop, as a DAG; a topological order of it, or the cycle that makes the
packing unbuildable (the packer then draws again).

Access (a before b): b's packed pose covers a's insertion path (frigidaire_dependency_kit.access_hits, the move's 1 mm
path check, here on the packing instead of the built load). Support (a before b): b rests on a, i.e. b lowered by
SUPPORT_GAP_M plus the checker's 1 mm allowance meets a past that allowance while b at its pose does not: b's underside
lies within 2 mm above a's surface, inside b's footprint, on the same FCL pieces (the bench checker). Rack-local entries
are placed in the bench baseline's extended racks, as the dependency kit's goal_setup does. Needs numpy, fcl and pxr
(code/util/run_py.sh), imported on first use.
pose_model "rest" (plans/2026-10-22-plate-rest-pose-dag.md): each plate at the measured rest pose of its slot and lean side
(REST_TABLE, frigidaire_plate_rest.py), bowls and cups at their packed poses, no support edges; access edges only between
plates and between a plate and a bowl or cup, both ways (plans/2026-10-22-bowl-plate-edges.md: a bowl or cup whose packed
pose covers a plate's descent puts the plate first, a plate whose rest pose covers a bowl's or cup's descent puts the bowl
or cup first); no bowl-bowl, bowl-cup or cup-cup edges.
"""
import json
from pathlib import Path
import random
import sys

from . import ROOT

SUPPORT_GAP_M = .002
BASELINE = ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json"     # the dependency kit's --baseline
REST_TABLE = ROOT / "data/results/sampler/plate_rest/plate_rest.json"                # frigidaire_plate_rest.py --out


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


def _world(entries, frames=None):
    """(poses, moves, kinds, racks), world: each entry at its packed pose and at its insertion move pose in the racks of
    ``frames`` (the five appliance component poses, included in ``poses``; default the bench baseline's)."""
    B, _, _, P = _mods()
    frames = frames or json.loads(BASELINE.read_text())["snapshot"]["poses"]
    poses, moves, kinds, racks = dict(frames), {}, {}, {}
    for e in entries:
        f = frames[e["rack"]]
        poses[e["id"]] = P.world_from_local(f, {"position_m": list(e["position"]),
                                                "quaternion_xyzw": list(e["quaternion_xyzw"])})
        moves[e["id"]] = P.world_from_local(f, B.move_local(e))
        kinds[e["id"]], racks[e["id"]] = e["kind"], e["rack"]
    return poses, moves, kinds, racks


def plate_rest_poses(table=REST_TABLE):
    """{(slot, lean side): rack-local {position, quaternion_xyzw}} of the lone-plate rest table."""
    rows = json.loads(Path(table).read_text())["entries"]
    return {(r["slot"], r["side"]): {"position": r["position"], "quaternion_xyzw": r["quaternion_xyzw"]} for r in rows}


def rest_entries(entries, table=REST_TABLE):
    """``entries`` with every plate at the table's rest pose of its slot and lean side; bowls and cups as given."""
    B, rest = _mods()[0], plate_rest_poses(table)
    return [{**e, **rest[(e["slot"], B.lean_side(e["variant"]))]} if e["kind"] == "plate" else e for e in entries]


def access_edges(entries, frames=None, pose_model="packed"):
    """[(a, b)]: a goes in before b, because b's pose (packed, or the rest model's) covers a's insertion path."""
    B, _, DK, _ = _mods()
    if pose_model == "rest":
        entries = rest_entries(entries)
    poses, moves, kinds, racks = _world(entries, frames)
    hits, _ = DK.access_hits(B, B.release_checker(), kinds, racks, moves, poses, [e["id"] for e in entries])
    return sorted({(h["a"], h["b"]) for h in hits if pose_model != "rest" or "plate" in (kinds[h["a"]], kinds[h["b"]])})


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


def dag_order(entries, seed, frames=None, pose_model="packed"):
    """{order, access, support, layers, cycle}: a topological order of the access and support edges, ties broken by the
    lean-side chain rule within a plate bank (frigidaire_bench_kit.chain_order over a seeded permutation), then by that
    permutation; layers = the longest prerequisite chain + 1. On a cycle: order and layers None, cycle = its dishes in
    edge order (each before the next, the last before the first)."""
    B, K, _, _ = _mods()
    access = access_edges(entries, frames, pose_model)
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
