"""Explicit organization rules for unchanged Frigidaire dish geometry.

Opening normals are local +Z and mug handles local +X. Geometry tests use
actual collision pieces for separation/nesting and authored visual triangles
for the explicitly limited opening-exposure proxy. No Kit imports occur.
"""
from __future__ import annotations
from collections import Counter, defaultdict
from copy import deepcopy
import math
import numpy as np
from .random_poses import quaternion_matrix_xyzw


def default_policy():
    return {"schema_version": 1, "mug_max_downward_angle_deg": 45.,
            "bowl_max_downward_angle_deg": 75., "plate_max_vertical_deviation_deg": 15.,
            "minimum_dish_clearance_m": .005, "opening_samples": 64,
            "opening_ray_length_m": .100, "minimum_opening_exposure": .80,
            "require_direct_rack_support": True, "allow_nesting": False,
            "allow_stacking": False, "clearance_preference_cap_m": .015,
            "definition": "Geometric organization; not measured cleaning performance or loading accessibility"}


def orientation_metrics(kind, rack, pose, policy=None):
    policy = default_policy() if policy is None else policy
    normal = quaternion_matrix_xyzw(pose["quaternion_xyzw"])[:, 2]
    down_angle = math.degrees(math.acos(float(np.clip(-normal[2], -1., 1.))))
    if kind in ("mug", "bowl"):
        limit = policy[f"{kind}_max_downward_angle_deg"]
        metric = down_angle
        valid = metric <= limit + 1e-8
    elif kind == "dinner_plate":
        metric = math.degrees(math.asin(float(np.clip(abs(normal[2]), 0., 1.))))
        limit = policy["plate_max_vertical_deviation_deg"]
        valid = rack == "LowerRack" and metric <= limit + 1e-8
    else:
        raise ValueError(f"Unsupported organized dish kind: {kind}")
    return {"valid": bool(valid), "opening_normal_world": normal.tolist(),
            "downward_angle_deg": down_angle, "orientation_error_deg": metric,
            "limit_deg": limit, "required_rack": "LowerRack" if kind == "dinner_plate" else None}


def _pose_for(obj, poses):
    return obj["pose_world"] if poses is None else poses[obj["object_id"]]


def _bounds_distance(a, b):
    gap = np.maximum(0., np.maximum(a[0]-b[1], b[0]-a[1]))
    return float(np.linalg.norm(gap))


def _convex(fcl, vertices, faces):
    faces = np.column_stack((np.full(len(faces), 3), faces)).astype(np.int32)
    return fcl.Convex(np.asarray(vertices, dtype=float), len(faces), faces.ravel())


class OrganizationGeometry:
    """Cached authored geometry, with exact FCL minimum-distance queries."""
    def __init__(self, asset_dir=None, checker=None):
        import fcl
        from .tableware import tableware_geometry
        from .initial_state_candidates import InitialCollisionChecker
        from .loading import Part, rotation
        from scipy.spatial import ConvexHull
        self.fcl = fcl
        self.checker = checker
        if self.checker is None and asset_dir is not None:
            self.checker = InitialCollisionChecker(asset_dir)
        self.parts, self.visuals, self.vertices, self.bounds, self.cavities = {}, {}, {}, {}, {}
        for kind in ("dinner_plate", "bowl", "mug"):
            geometry = tableware_geometry(kind)
            self.visuals[kind] = np.concatenate([v[0][v[2]] for v in geometry["visuals"]])
            self.vertices[kind] = np.concatenate([v[0] for v in geometry["visuals"]])
            if self.checker is not None:
                self.bounds[kind] = self.checker.bounds[kind]
            else:
                # The convex pieces and handle capsule endpoints define a safe
                # collider box; visual surface facets are never a certificate.
                collision_points = [v for v, _ in geometry["convexes"]]
                for a, b, radius in geometry["capsules"]:
                    collision_points.extend([np.asarray([a,b])-radius, np.asarray([a,b])+radius])
                collision_points = np.concatenate(collision_points)
                self.bounds[kind] = (collision_points.min(0), collision_points.max(0))
            if self.checker is not None:
                self.parts[kind] = self.checker.parts[kind]
            else:
                parts = [Part(_convex(fcl, v, fs), np.eye(3), np.zeros(3), f"shell{i}")
                         for i, (v, fs) in enumerate(geometry["convexes"])]
                for i, (a, b, radius) in enumerate(geometry["capsules"]):
                    direction = (b-a)/np.linalg.norm(b-a)
                    axis = np.cross([0., 0., 1.], direction)
                    sine = float(np.linalg.norm(axis)); cosine = float(direction[2])
                    if sine < 1e-12:
                        orient = np.eye(3) if cosine > 0 else np.diag([1., -1., -1.])
                    else:
                        axis /= sine
                        skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
                        orient = np.eye(3) + sine*skew + (1.-cosine)*(skew@skew)
                    parts.append(Part(fcl.Capsule(radius, float(np.linalg.norm(b-a))), orient, (a+b)/2, f"handle{i}"))
                self.parts[kind] = parts
            if kind == "bowl":
                profile = [(0., .0062), (.0238, .0062), (.0398, .025), (.0598, .052), (.0663, .0648)]
            elif kind == "mug":
                profile = [(0., -.0438), (.0358, -.0438), (.0378, -.035), (.0388, .0498)]
            else:
                continue
            angles = np.arange(96)*2*np.pi/96
            vertices = np.asarray([[r*np.cos(a), r*np.sin(a), z] for r,z in profile for a in angles])
            hull = ConvexHull(vertices)
            faces = hull.simplices.copy()
            for face, equation in zip(faces, hull.equations):
                a, b, c = vertices[face]
                if np.dot(np.cross(b-a, c-a), equation[:3]) < 0:
                    face[1], face[2] = face[2], face[1]
            self.cavities[kind] = _convex(fcl, vertices, faces)

    def body(self, kind, pose, object_id="dish"):
        rot = quaternion_matrix_xyzw(pose["quaternion_xyzw"])
        pos = np.asarray(pose["position_m"], dtype=float)
        objects = [self.fcl.CollisionObject(part.geometry,
                   self.fcl.Transform(rot @ part.rotation, rot @ part.translation + pos)) for part in self.parts[kind]]
        manager = self.fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(objects); manager.setup()
        lo, hi = self.bounds[kind]
        corners = np.asarray([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
        vertices = corners @ rot.T + pos
        return {"id": object_id, "kind": kind, "objects": objects, "manager": manager,
                "bounds": (vertices.min(0), vertices.max(0)), "pose": pose,
                "triangles": self.visuals[kind] @ rot.T + pos}

    def distance(self, first, second):
        data = self.fcl.DistanceData(request=self.fcl.DistanceRequest(enable_nearest_points=False))
        first["manager"].distance(second["manager"], data, self.fcl.defaultDistanceCallback)
        result = float(data.result.min_distance)
        if not np.isfinite(result):
            raise ValueError("Nonfinite FCL separation query")
        return max(0., result)

    def nested(self, vessel, other):
        kind = vessel["kind"]
        if kind not in self.cavities:
            return False
        p = vessel["pose"]
        cavity = self.fcl.CollisionObject(self.cavities[kind], self.fcl.Transform(
                   quaternion_matrix_xyzw(p["quaternion_xyzw"]), np.asarray(p["position_m"])))
        data = self.fcl.CollisionData(request=self.fcl.CollisionRequest(num_max_contacts=1))
        other["manager"].collide(cavity, data, self.fcl.defaultCollisionCallback)
        return bool(data.result.is_collision)

    def opening_rays(self, kind, pose, samples=64):
        if samples != 64:
            raise ValueError("The organization protocol fixes 64 opening rays")
        # Deterministic equal-area Fibonacci disk, slightly inside the inner lip.
        radius, z = {"mug": (.038, .05001), "bowl": (.0655, .06501)}[kind]
        k = np.arange(samples)
        radial = radius*np.sqrt((k+.5)/samples)
        angles = k*(np.pi*(3.-np.sqrt(5.)))
        points = np.column_stack((radial*np.cos(angles), radial*np.sin(angles), np.full(samples,z)))
        rot = quaternion_matrix_xyzw(pose["quaternion_xyzw"])
        return points @ rot.T + np.asarray(pose["position_m"]), rot[:,2]


def ray_blocked(origins, direction, triangles, length):
    """Two-sided finite Moller-Trumbore test; no optional ray backend needed."""
    if not len(triangles):
        return np.zeros(len(origins), dtype=bool)
    ends = origins + np.asarray(direction)*length
    lower = np.minimum(origins.min(0), ends.min(0)); upper = np.maximum(origins.max(0), ends.max(0))
    keep = np.all(triangles.min(1) <= upper+1e-12, axis=1) & np.all(triangles.max(1) >= lower-1e-12, axis=1)
    triangles = triangles[keep]
    if not len(triangles):
        return np.zeros(len(origins), dtype=bool)
    edge1 = triangles[:,1]-triangles[:,0]
    edge2 = triangles[:,2]-triangles[:,0]
    h = np.cross(direction, edge2)
    det = np.einsum("ij,ij->i", edge1,h)
    valid = np.abs(det)>1e-12
    inv = np.zeros_like(det); inv[valid] = 1./det[valid]
    blocked = np.zeros(len(origins),dtype=bool)
    # Small chunks bound temporary memory for a full 35-object load.
    for start in range(0,len(origins),8):
        s = origins[start:start+8,None,:]-triangles[None,:,0,:]
        u = np.einsum("rtj,tj->rt",s,h)*inv
        q = np.cross(s,edge1)
        v = np.einsum("rtj,j->rt",q,direction)*inv
        t = np.einsum("rtj,tj->rt",q,edge2)*inv
        hit = valid & (u>=-1e-10) & (v>=-1e-10) & (u+v<=1+1e-10) & (t>=0) & (t<=length)
        blocked[start:start+8] = np.any(hit,axis=1)
    return blocked


def evaluate_organization(objects, poses=None, geometry=None, policy=None,
                          component_frames=None, direct_support=None):
    """Evaluate measured world poses, never proposal rotations as a fallback.

    ``direct_support`` maps every object ID to a measured bool. Omitting it
    intentionally fails the support gate. ``component_frames`` is accepted for
    evaluator compatibility; row axes are transformed from their assigned rack.
    """
    policy = default_policy() if policy is None else deepcopy(policy)
    geometry = OrganizationGeometry() if geometry is None else geometry
    ids = [obj["object_id"] for obj in objects]
    if len(ids)!=len(set(ids)):
        raise ValueError("Unique object IDs are required")
    bodies, per_object, violations = [], {}, []
    for obj in objects:
        oid = obj["object_id"]
        pose = _pose_for(obj,poses)
        orientation = orientation_metrics(obj["kind"],obj["rack"],pose,policy)
        support = bool(direct_support.get(oid,False)) if direct_support is not None else None
        per_object[oid] = {"kind":obj["kind"],"rack":obj["rack"],"orientation":orientation,
                           "direct_rack_support":support}
        if not orientation["valid"]:
            violations.append({"rule":"orientation","object_id":oid})
        if policy["require_direct_rack_support"] and support is not True:
            violations.append({"rule":"direct_rack_support","object_id":oid,"assessed":direct_support is not None})
        bodies.append(geometry.body(obj["kind"],pose,oid))
    min_clearance = None
    distances, nestings = [], []
    # Far pairs are certified by an AABB lower bound; close ones use exact FCL.
    cap = policy["clearance_preference_cap_m"]
    for i,first in enumerate(bodies):
        for second in bodies[i+1:]:
            lower = _bounds_distance(first["bounds"],second["bounds"])
            if lower>cap:
                distance = lower; exact=False
            else:
                distance=geometry.distance(first,second); exact=True
            min_clearance=distance if min_clearance is None else min(min_clearance,distance)
            if distance<policy["minimum_dish_clearance_m"]-1e-9:
                violations.append({"rule":"separation","objects":[first["id"],second["id"]],"distance_m":distance})
            if distance<=cap:
                distances.append({"objects":[first["id"],second["id"]],"distance_m":distance,"exact":exact})
            if lower<=0:
                for vessel,other in ((first,second),(second,first)):
                    if geometry.nested(vessel,other):
                        entry={"vessel":vessel["id"],"intruder":other["id"]}
                        nestings.append(entry); violations.append({"rule":"nesting",**entry})
    exposure=[]
    for vessel in bodies:
        if vessel["kind"] not in ("mug","bowl"):
            continue
        origins,direction=geometry.opening_rays(vessel["kind"],vessel["pose"],policy["opening_samples"])
        ends=origins+direction*policy["opening_ray_length_m"]
        bounds=(np.minimum(origins.min(0),ends.min(0)),np.maximum(origins.max(0),ends.max(0)))
        blocked=np.zeros(len(origins),dtype=bool)
        blockers=[]
        for other in bodies:
            if other is vessel or _bounds_distance(bounds,other["bounds"])>0:
                continue
            hits=ray_blocked(origins,direction,other["triangles"],policy["opening_ray_length_m"])
            if np.any(hits): blockers.append(other["id"])
            blocked |= hits
        fraction=float(np.mean(~blocked))
        entry={"object_id":vessel["id"],"unobstructed_fraction":fraction,
               "unobstructed_rays":int(np.sum(~blocked)),"total_rays":len(origins),"blockers":blockers}
        exposure.append(entry); per_object[vessel["id"]]["opening_exposure"]=entry
        if fraction+1e-12<policy["minimum_opening_exposure"]:
            violations.append({"rule":"opening_exposure",**entry})
    rows=defaultdict(list)
    handle_errors=[]; normal_errors=[]
    for obj,body in zip(objects,bodies):
        metadata=obj.get("row_metadata",{})
        row=metadata.get("row_id",obj.get("row_id","unassigned"))
        coordinate=float(metadata.get("coordinate_m",body["pose"]["position_m"][1]))
        rows[(obj["rack"],row)].append((coordinate,obj["kind"]))
        intended=metadata.get("opening_normal_rack")
        if intended is not None and component_frames is not None:
            rackrot=quaternion_matrix_xyzw(component_frames[obj["rack"]]["quaternion_xyzw"])
            actual=quaternion_matrix_xyzw(body["pose"]["quaternion_xyzw"])
            dot=float(np.dot(actual[:,2],rackrot@np.asarray(intended)))
            if obj["kind"]=="dinner_plate":dot=abs(dot)
            normal_errors.append(math.degrees(math.acos(float(np.clip(dot,-1,1)))))
            handle=metadata.get("handle_direction_rack")
            if obj["kind"]=="mug" and handle is not None:
                dot=float(np.dot(actual[:,0],rackrot@np.asarray(handle)))
                handle_errors.append(math.degrees(math.acos(float(np.clip(dot,-1,1)))))
    fragments=sum(sum(a[1]!=b[1] for a,b in zip(sorted(row),sorted(row)[1:]))+1 for row in rows.values())
    preferred=sum(o["kind"]=="mug" and o["rack"]=="UpperRack" for o in objects)
    return {"valid":not violations,"passed":not violations,"policy":policy,"violations":violations,
            "per_object":per_object,"separation":{"minimum_certified_clearance_m":min_clearance,
               "far_pairs_use_aabb_lower_bound":True,"close_pairs":distances},
            "nesting":{"intrusions":nestings},"opening_exposure":exposure,
            "preferences":{"upper_rack_mugs":preferred,"type_group_fragments":fragments,
                "mean_row_normal_error_deg":float(np.mean(normal_errors)) if normal_errors else None,
                "mean_handle_error_deg":float(np.mean(handle_errors)) if handle_errors else None}}


class OrganizationEvaluator:
    """Convenience runtime adapter retaining one geometry cache."""
    def __init__(self, asset_dir=None, policy=None, checker=None):
        self.geometry=OrganizationGeometry(asset_dir,checker)
        self.policy=default_policy() if policy is None else deepcopy(policy)

    def evaluate(self, objects, poses=None, direct_support=None, component_frames=None):
        return evaluate_organization(objects,poses,self.geometry,self.policy,component_frames,direct_support)


def preference_rank(evaluation):
    """Sort passing arrangements lexicographically; lower tuple is preferred.

    This ranks the arrangements actually considered, not all possible placements.
    Missing semantic row information ranks after measured aligned arrangements.
    """
    p = evaluation["preferences"]
    clearance = evaluation["separation"]["minimum_certified_clearance_m"]
    cap = evaluation["policy"]["clearance_preference_cap_m"]
    return (not evaluation["valid"], -p["upper_rack_mugs"], p["type_group_fragments"],
            p["mean_row_normal_error_deg"] if p["mean_row_normal_error_deg"] is not None else 180.,
            p["mean_handle_error_deg"] if p["mean_handle_error_deg"] is not None else 180.,
            -min(cap, clearance if clearance is not None else cap))
