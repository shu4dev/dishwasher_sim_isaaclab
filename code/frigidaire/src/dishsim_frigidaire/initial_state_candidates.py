"""Collision checker for Frigidaire load states (FCL over the authored component frames).

No Isaac/Kit import is performed. FCL and USD load only when the collision checker is
constructed. World and rack poses use metres, Z up and XYZW. The pose-template catalog,
compatibility graph and packing proposals that shared this module (generate_catalog,
build_compatibility, greedy_proposal, milp_proposal) were retired with the v3-era
experiments on 2026-09-29; they remain in git history.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import os
from pathlib import Path

import numpy as np

from .random_poses import OBJECT_KINDS


COMPONENT_NAMES = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")

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
    def __init__(self, asset_dir, penetration_limit_m=.001, tableware=None):
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
                     **{kind: directory / "tableware" / f"{kind}.usdc" for kind in OBJECT_KINDS}}
        filenames.update({kind: Path(path) for kind, path in (tableware or {}).items()})   # e.g. the HOTEC set
        self.parts = {name: collision_parts(path) for name, path in filenames.items()}
        self.bounds = {name: authored_collision_bounds(path) for name, path in filenames.items()}
        self.asset_sha256 = {os.path.relpath(path, directory): hashlib.sha256(path.read_bytes()).hexdigest()
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
        if candidate["kind"] not in self.parts or candidate["kind"] in COMPONENT_NAMES:
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
