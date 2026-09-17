"""Finite reusable pose templates and geometric multi-dish packing proposals.

No Isaac/Kit import is performed. FCL and USD load only when the collision
checker is constructed. World and rack poses use metres, Z up and XYZW.
A compatible packing is a proposal, never a claim of physical stability.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from .random_poses import (KINDS, RACKS, compose_pose, relative_pose,
                           quaternion_matrix_xyzw)


COMPONENT_NAMES = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")


def _pose(position, quaternion):
    return {"position_m": np.asarray(position, dtype=float).tolist(),
            "quaternion_xyzw": np.asarray(quaternion, dtype=float).tolist()}


def _read_pose(pose):
    from .random_pose_assets import _pose as validate_pose
    rotation, position = validate_pose(pose)
    return position, np.asarray(pose["quaternion_xyzw"], dtype=float), rotation


def _frames(component_frames):
    if set(component_frames) != set(COMPONENT_NAMES):
        raise ValueError("Supply exactly all five measured component frames")
    for pose in component_frames.values():
        _read_pose(pose)
    return deepcopy(component_frames)


def _direction(rng):
    while True:
        direction = rng.normal(size=3)
        length = float(np.linalg.norm(direction))
        if length > 1e-12:
            return direction / length


def generate_catalog(accepted_json_path, baseline_components, seed=0,
                     variants_per_template=8, translation_radius_m=.010,
                     rotation_max_deg=10.):
    """Original final poses plus independent bounded rack-local perturbations.

    Translation is uniform in a ball. Rotation uses an independent uniform
    axis on S2 and an angle uniform in [0, max]; it is *not* Haar-uniform in a
    geodesic SO(3) ball. The rotation acts about the dish actor origin in rack
    coordinates. Each source final rack transform is inverted separately.
    """
    from .tableware import CATALOG
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if (isinstance(variants_per_template, bool)
            or not isinstance(variants_per_template, (int, np.integer))
            or variants_per_template < 0):
        raise ValueError("variants_per_template must be a nonnegative integer")
    if not (np.isfinite(translation_radius_m) and 0 <= translation_radius_m <= .010):
        raise ValueError("translation radius must be in [0, 0.010] metres")
    if not (np.isfinite(rotation_max_deg) and 0 <= rotation_max_deg <= 10.):
        raise ValueError("rotation maximum must be in [0, 10] degrees")
    baseline = _frames(baseline_components)
    source_path = Path(accepted_json_path)
    raw = source_path.read_bytes()
    document = json.loads(raw)
    trials = document["trials"]
    ids = [trial["trial_id"] for trial in trials]
    if len(set(ids)) != len(ids):
        raise ValueError("Accepted source trial IDs must be unique")
    streams = np.random.SeedSequence(int(seed)).spawn(len(trials))
    candidates = []
    for trial, stream in zip(trials, streams):
        if trial["outcome"] != "accepted":
            raise ValueError("Only accepted trials may seed the candidate catalog")
        kind, rack = trial["kind"], trial["rack"]
        if kind not in KINDS or rack not in RACKS:
            raise ValueError("Unsupported source object type or rack")
        source_p, source_q, _ = _read_pose(trial["final_pose"])
        rack_p, rack_q, _ = _read_pose(trial["final_rack_pose"])
        local_p, local_q = relative_pose(source_p, source_q, rack_p, rack_q)
        baseline_p, baseline_q, _ = _read_pose(baseline[rack])
        rng = np.random.default_rng(stream)
        for variant in range(int(variants_per_template) + 1):
            if variant:
                delta = _direction(rng) * translation_radius_m * rng.random() ** (1 / 3)
                axis = _direction(rng)
                angle = float(rng.uniform(0., rotation_max_deg))
                half = math.radians(angle) / 2
                delta_q = np.r_[axis * math.sin(half), math.cos(half)]
            else:
                delta, axis, angle, delta_q = np.zeros(3), np.array([1., 0., 0.]), 0., np.array([0., 0., 0., 1.])
            # Precompose delta rotation without rotating the local position.
            _, perturbed_q = compose_pose([0., 0., 0.], delta_q, [0., 0., 0.], local_q)
            perturbed_p = local_p + delta
            world_p, world_q = compose_pose(baseline_p, baseline_q, perturbed_p, perturbed_q)
            candidate_id = f"{trial['trial_id']}_v{variant:02d}"
            candidates.append({"candidate_index": len(candidates), "candidate_id": candidate_id,
                "object_id": candidate_id, "source_trial_id": trial["trial_id"],
                "kind": kind, "rack": rack, "variant_index": variant,
                "mass_kg": CATALOG[kind]["mass_kg"], "size_m": list(CATALOG[kind]["size_m"]),
                "source_final_pose": deepcopy(trial["final_pose"]),
                "source_final_rack_pose": deepcopy(trial["final_rack_pose"]),
                "source_rack_local_pose": _pose(local_p, local_q),
                "rack_local_pose": _pose(perturbed_p, perturbed_q),
                "pose_world": _pose(world_p, world_q),
                "perturbation": {"translation_m": delta.tolist(), "translation_norm_m": float(np.linalg.norm(delta)),
                                 "rotation_axis_rack": axis.tolist(), "rotation_angle_deg": angle,
                                 "rotation_quaternion_xyzw": delta_q.tolist()},
                "seed": int(seed), "seed_spawn_key": list(stream.spawn_key)})
    return {"schema_version": 1, "source_accepted_json": str(source_path),
            "source_accepted_sha256": hashlib.sha256(raw).hexdigest(),
            "source_trial_count": len(trials), "candidate_count": len(candidates),
            "seed": int(seed), "variants_per_template": int(variants_per_template),
            "translation_radius_m": float(translation_radius_m), "rotation_max_deg": float(rotation_max_deg),
            "sampling": "uniform translation ball; uniform S2 rotation axis and uniform angle; rack-local actor origin",
            "inventory": "reusable templates; each candidate at most once per state; source trial reuse permitted",
            "baseline_components": baseline, "candidates": candidates}


def _corners(lower, upper):
    return np.array([[x, y, z] for x in (lower[0], upper[0])
                     for y in (lower[1], upper[1]) for z in (lower[2], upper[2])])


def authored_collision_bounds(filename):
    """Conservative local AABB from every enabled authored collision shape."""
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    points = []
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim) * root_inverse).T
        if prim.IsA(UsdGeom.Mesh):
            local = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        elif prim.IsA(UsdGeom.Cube):
            extent = np.full(3, float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2)
            local = _corners(-extent, extent)
        elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
            is_capsule = prim.IsA(UsdGeom.Capsule)
            shape = UsdGeom.Capsule(prim) if is_capsule else UsdGeom.Cylinder(prim)
            radius = float(shape.GetRadiusAttr().Get())
            extent = np.full(3, radius)
            extent["XYZ".index(shape.GetAxisAttr().Get())] = float(shape.GetHeightAttr().Get()) / 2 + (radius if is_capsule else 0.)
            local = _corners(-extent, extent)
        else:
            raise ValueError(f"Unsupported authored collider: {prim.GetPath()}")
        points.append(local @ matrix[:3, :3].T + matrix[:3, 3])
    if not points:
        raise ValueError(f"No enabled colliders: {filename}")
    points = np.concatenate(points)
    return points.min(axis=0), points.max(axis=0)


def _overlap(first, second):
    return bool(np.all(first[0] <= second[1]) and np.all(second[0] <= first[1]))


class InitialCollisionChecker:
    """Reusable actual-geometry FCL checker with a 1 mm penetration allowance.

    FCL broadphase finds primitive pairs; all returned contact depths are
    examined. An empty/overflowing contact result or nonfinite depth fails
    closed. The allowance is contact penetration, never object inflation.
    """
    def __init__(self, asset_dir, penetration_limit_m=.001):
        import fcl
        from .asset import COMPONENT_FILES
        from .loading import collision_parts
        if not np.isfinite(penetration_limit_m) or not 0 <= penetration_limit_m <= .001:
            raise ValueError("initial penetration limit must be in [0, 0.001] metres")
        self.fcl = fcl
        self.penetration_limit_m = float(penetration_limit_m)
        self.max_contacts_per_pair = 256
        self.request = fcl.CollisionRequest(enable_contact=True, num_max_contacts=self.max_contacts_per_pair)
        directory = Path(asset_dir)
        filenames = {**{name: directory / filename for name, filename in COMPONENT_FILES.items()},
                     **{kind: directory / "tableware" / f"{kind}.usdc" for kind in KINDS}}
        self.parts = {name: collision_parts(path) for name, path in filenames.items()}
        self.bounds = {name: authored_collision_bounds(path) for name, path in filenames.items()}
        self.asset_sha256 = {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in filenames.values()}
        self.components = None

    def _body(self, kind, pose, body_id):
        p, _, orient = _read_pose(pose)
        objects = [self.fcl.CollisionObject(part.geometry,
                    self.fcl.Transform(orient @ part.rotation, orient @ part.translation + p))
                   for part in self.parts[kind]]
        manager = self.fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(objects)
        manager.setup()
        corners = _corners(*self.bounds[kind]) @ orient.T + p
        return {"id": body_id, "kind": kind, "objects": objects, "manager": manager,
                "bounds": (corners.min(axis=0), corners.max(axis=0))}

    def update_components(self, component_frames):
        component_frames = _frames(component_frames)
        self.components = {name: self._body(name, component_frames[name], name)
                           for name in COMPONENT_NAMES}

    def candidate_body(self, candidate):
        if candidate["kind"] not in KINDS:
            raise ValueError("Unsupported candidate dish kind")
        return self._body(candidate["kind"], candidate["pose_world"],
                          candidate.get("object_id", candidate.get("candidate_id", "dish")))

    def pair(self, first, second):
        result = {"valid": True, "max_penetration_m": 0., "primitive_pairs": 0,
                  "reason": None, "bodies": [first["id"], second["id"]]}
        if not _overlap(first["bounds"], second["bounds"]):
            return result
        def callback(a, b, data):
            contacts = self.fcl.CollisionResult()
            self.fcl.collide(a, b, self.request, contacts)
            data["primitive_pairs"] += 1
            if not contacts.is_collision:
                return False
            depths = np.asarray([contact.penetration_depth for contact in contacts.contacts], dtype=float)
            if not len(depths) or len(depths) >= self.max_contacts_per_pair or not np.isfinite(depths).all():
                data.update(valid=False, reason="unresolved_contact_query")
                return True
            peak = max(0., float(depths.max()))
            data["max_penetration_m"] = max(data["max_penetration_m"], peak)
            if peak > self.penetration_limit_m + 1e-12:
                data.update(valid=False, reason="initial_penetration")
                return True
            return False
        first["manager"].collide(second["manager"], result, callback)
        return result

    def against_components(self, body):
        if self.components is None:
            raise RuntimeError("Update measured appliance component frames before checking")
        details = []
        peak = 0.
        for component in self.components.values():
            result = self.pair(body, component)
            peak = max(peak, result["max_penetration_m"])
            if result["primitive_pairs"] or not result["valid"]:
                details.append(result)
            if not result["valid"]:
                return {"valid": False, "reason": result["reason"], "max_penetration_m": peak, "pairs": details}
        return {"valid": True, "reason": None, "max_penetration_m": peak, "pairs": details}

    def check_arrangement(self, objects, component_frames):
        self.update_components(component_frames)
        ids = [obj.get("object_id", obj.get("candidate_id")) for obj in objects]
        if None in ids or len(set(ids)) != len(ids):
            raise ValueError("Arrangement object IDs must be present and unique")
        bodies = []
        details = []
        peak = 0.
        for entry in objects:
            body = self.candidate_body(entry)
            result = self.against_components(body)
            peak = max(peak, result["max_penetration_m"])
            details.extend(result["pairs"])
            if not result["valid"]:
                return {"valid": False, "reason": result["reason"], "max_penetration_m": peak, "pairs": details}
            for other in bodies:
                result = self.pair(body, other)
                peak = max(peak, result["max_penetration_m"])
                if result["primitive_pairs"] or not result["valid"]:
                    details.append(result)
                if not result["valid"]:
                    return {"valid": False, "reason": result["reason"], "max_penetration_m": peak, "pairs": details}
            bodies.append(body)
        return {"valid": True, "reason": None, "max_penetration_m": peak, "pairs": details}


def build_compatibility(catalog, asset_dir, penetration_limit_m=.001, deadline=None, progress=None):
    """Build a conflict graph; deadlines are absolute time.monotonic seconds.

    A partial graph is returned with complete=False and is unusable for packing.
    This preserves work/evidence without treating untested pairs as compatible.
    """
    started = time.monotonic()
    checker = InitialCollisionChecker(asset_dir, penetration_limit_m)
    checker.update_components(catalog["baseline_components"])
    candidates = catalog["candidates"]
    graph = {"schema_version": 1, "complete": False, "candidate_count": len(candidates),
             "allowed_indices": [], "conflict_pairs": [], "initial_filter": [],
             "penetration_limit_m": float(penetration_limit_m), "asset_sha256": checker.asset_sha256,
             "source_accepted_sha256": catalog["source_accepted_sha256"],
             "candidate_catalog_sha256": hashlib.sha256(json.dumps(catalog, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
             "broadphase_pairs": 0, "tested_pairs": 0, "unresolved_pairs": 0,
             "bound_scope": "finite candidate geometry in common measured extended frames; not a physical or global maximum"}
    def expired():
        if deadline is not None and time.monotonic() >= deadline:
            graph["status"] = "time_budget_exhausted"
            graph["elapsed_s"] = time.monotonic() - started
            return True
        return False
    bodies = []
    for index, candidate in enumerate(candidates):
        if expired():
            return graph
        body = checker.candidate_body(candidate)
        result = checker.against_components(body)
        graph["initial_filter"].append({"candidate_index": index, **result})
        if result["valid"]:
            graph["allowed_indices"].append(index)
            bodies.append(body)
        if progress and (index + 1) % 50 == 0:
            progress(f"candidate geometry {index + 1}/{len(candidates)}; {len(bodies)} allowed")
    count = len(bodies)
    bounds = np.asarray([body["bounds"] for body in bodies]) if count else np.empty((0, 2, 3))
    for i in range(count):
        if expired():
            return graph
        overlaps = np.flatnonzero(np.all(bounds[i, 0] <= bounds[i + 1:, 1], axis=1)
                                 & np.all(bounds[i + 1:, 0] <= bounds[i, 1], axis=1)) + i + 1
        graph["broadphase_pairs"] += len(overlaps)
        for j in overlaps:
            if graph["tested_pairs"] % 128 == 0 and expired():
                return graph
            result = checker.pair(bodies[i], bodies[j])
            graph["tested_pairs"] += 1
            if not result["valid"]:
                graph["conflict_pairs"].append([graph["allowed_indices"][i], graph["allowed_indices"][j]])
                graph["unresolved_pairs"] += int(result["reason"] == "unresolved_contact_query")
        if progress and (i + 1) % 50 == 0:
            progress(f"pair geometry {i + 1}/{count}; {len(graph['conflict_pairs'])} conflicts")
    graph.update(complete=True, status="complete", elapsed_s=time.monotonic() - started,
                 allowed_count=count, total_allowed_pairs=count * (count - 1) // 2)
    return graph


def _graph(graph):
    if not graph.get("complete"):
        raise ValueError("Cannot propose states from an incomplete compatibility graph")
    allowed = list(graph["allowed_indices"])
    if len(set(allowed)) != len(allowed):
        raise ValueError("Duplicate candidate indices in graph")
    adjacency = {index: set() for index in allowed}
    for first, second in graph["conflict_pairs"]:
        if first == second or first not in adjacency or second not in adjacency:
            raise ValueError("Invalid conflict edge")
        adjacency[first].add(second)
        adjacency[second].add(first)
    return allowed, adjacency


def greedy_proposal(graph, seed, target_count=None, excluded_sets=(), restarts=32):
    """Seeded low-degree randomized greedy independent sets, with exact dedupe."""
    allowed, adjacency = _graph(graph)
    if target_count is not None and (int(target_count) != target_count or target_count < 1):
        raise ValueError("target_count must be a positive integer")
    rng = np.random.default_rng(seed)
    excluded = {frozenset(indices) for indices in excluded_sets}
    best = None
    for _ in range(restarts):
        remaining = set(allowed)
        chosen = []
        while remaining and (target_count is None or len(chosen) < target_count):
            order = sorted(remaining)
            degree = np.array([len(adjacency[index] & remaining) for index in order])
            # Randomness produces diverse packings while favouring low degree.
            score = (degree + 1) * rng.uniform(.4, 1.6, len(order))
            selected = order[int(np.argmin(score))]
            chosen.append(selected)
            remaining.difference_update(adjacency[selected] | {selected})
        if frozenset(chosen) in excluded:
            continue
        if target_count is not None and len(chosen) != target_count:
            continue
        if best is None or len(chosen) > len(best):
            best = chosen
    return {"method": "randomized_greedy", "seed": int(seed), "target_count": target_count,
            "selected_indices": sorted(best) if best is not None else None,
            "count": len(best) if best is not None else 0, "status": "proposal" if best is not None else "no_new_proposal"}


def milp_proposal(graph, seed=0, time_limit_s=60., excluded_sets=(), target_count=None, preferred_indices=()):
    """Maximum-cardinality finite-catalog independent set using SciPy/HiGHS.

    Exact no-good: sum(x in S) - sum(x outside S) <= |S|-1. Unlike the
    conventional subset cut this permits supersets which may add support.
    """
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import coo_matrix
    allowed, adjacency = _graph(graph)
    if not np.isfinite(time_limit_s) or not 0 < time_limit_s <= 60:
        raise ValueError("MILP time limit must be positive and at most 60 seconds")
    if target_count is not None and (int(target_count) != target_count or target_count < 1):
        raise ValueError("target_count must be a positive integer")
    if not allowed:
        return {"method": "scipy_milp", "seed": int(seed), "selected_indices": None, "count": 0,
                "status": "empty_catalog", "geometric_cardinality_upper_bound": 0}
    lookup = {value: index for index, value in enumerate(allowed)}
    preferred = set(preferred_indices)
    if not preferred.issubset(lookup):
        raise ValueError("Preferred indices must belong to the eligible finite catalog")
    rows, cols, values, upper, lower = [], [], [], [], []
    for first, second in graph["conflict_pairs"]:
        row = len(upper)
        rows.extend((row, row)); cols.extend((lookup[first], lookup[second])); values.extend((1., 1.))
        lower.append(-np.inf); upper.append(1.)
    unique_excluded = {frozenset(indices) for indices in excluded_sets}
    active_excluded = []
    for indices in unique_excluded:
        if not indices.issubset(lookup):
            continue
        row = len(upper)
        rows.extend([row] * len(allowed)); cols.extend(range(len(allowed)))
        values.extend(1. if index in indices else -1. for index in allowed)
        lower.append(-np.inf); upper.append(len(indices) - 1.)
        active_excluded.append(indices)
    if target_count is not None:
        row = len(upper)
        rows.extend([row] * len(allowed)); cols.extend(range(len(allowed))); values.extend([1.] * len(allowed))
        lower.append(float(target_count)); upper.append(float(target_count))
    matrix = coo_matrix((values, (rows, cols)), shape=(len(upper), len(allowed))).tocsc()
    rng = np.random.default_rng(seed)
    # A whole extra object always dominates the complete tie break (<0.001).
    # One retained preferred object also dominates *all* random tie weights.
    size = len(allowed)
    preference_weight = np.asarray([index in preferred for index in allowed], dtype=float) * (.0009 / size)
    random_weight = rng.random(size) * (.0001 / (size * size))
    objective = -np.ones(size) - preference_weight - random_weight
    started = time.monotonic()
    result = milp(objective, integrality=np.ones(len(allowed)), bounds=Bounds(0., 1.),
                  constraints=LinearConstraint(matrix, np.asarray(lower), np.asarray(upper)),
                  options={"time_limit": float(time_limit_s), "mip_rel_gap": 0.})
    selected = None
    if result.x is not None:
        proposed = [index for index, value in zip(allowed, result.x) if value > .5]
        proposal_set = frozenset(proposed)
        if (not any(adjacency[index] & proposal_set for index in proposed)
                and proposal_set not in active_excluded
                and (target_count is None or len(proposed) == target_count)):
            selected = sorted(proposed)
    dual = getattr(result, "mip_dual_bound", None)
    upper_bound = None
    if dual is not None and np.isfinite(dual):
        upper_bound = max(0, min(len(allowed), int(math.floor(-float(dual) + 1e-7))))
    if target_count is None and upper_bound is not None and unique_excluded:
        # Restore a bound over the full graph after exact assignments were cut.
        upper_bound = max([upper_bound, *[len(indices) for indices in active_excluded]])
    gap = getattr(result, "mip_gap", None)
    return {"method": "scipy_milp", "seed": int(seed), "selected_indices": selected,
            "count": len(selected) if selected is not None else 0, "status": int(result.status),
            "message": str(result.message), "elapsed_s": time.monotonic() - started,
            "time_limit_s": float(time_limit_s), "target_count": target_count,
            "mip_gap": float(gap) if gap is not None and np.isfinite(gap) else None,
            "mip_dual_bound": float(dual) if dual is not None and np.isfinite(dual) else None,
            "geometric_cardinality_upper_bound": upper_bound,
            "bound_scope": "full finite compatibility graph" if target_count is None else "fixed target-count MILP only",
            "excluded_exact_sets": len(active_excluded), "preferred_count": len(preferred),
            "selected_preferred_count": len(preferred.intersection(selected or [])),
            "objective": "cardinality, then preferred overlap, then seeded randomness; total tiebreak weight below 0.001"}
