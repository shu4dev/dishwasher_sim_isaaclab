"""Deterministic, full-size mixed loading for the independent Frigidaire asset.

The finite candidate pattern establishes scenario saturation, not a global packing
maximum. FCL reads the exact authored contact primitives, without benchmark caches.
All recorded poses are component-local metres with XYZW quaternions.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np


ORDER = ("dinner_plate", "salad_plate", "saucer", "mug", "tumbler", "bowl",
         "fork", "knife", "tablespoon", "teaspoon")
CUTLERY = ORDER[-4:]


def rotation(axis, angle):
    """Right-handed rotation, with radians at this interface."""
    c, s = math.cos(angle), math.sin(angle)
    if axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quaternion_xyzw(matrix):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(matrix).as_quat().tolist()


def matrix_xyzw(quaternion):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(quaternion).as_matrix()


@dataclass
class Part:
    geometry: object
    rotation: np.ndarray
    translation: np.ndarray
    name: str


def collision_parts(filename):
    """Extract actual USD box, cylinder, capsule and convex-mesh colliders."""
    import fcl
    import trimesh
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    result = []
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim) * root_inverse).T
        scale = np.linalg.norm(matrix[:3, :3], axis=0)
        orient = matrix[:3, :3] / scale
        if prim.IsA(UsdGeom.Cube):
            size = float(UsdGeom.Cube(prim).GetSizeAttr().Get())
            geometry = fcl.Box(*(scale * size))
        elif prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule):
            shape = UsdGeom.Cylinder(prim) if prim.IsA(UsdGeom.Cylinder) else UsdGeom.Capsule(prim)
            assert np.allclose(scale, 1), str(prim.GetPath())
            constructor = fcl.Cylinder if prim.IsA(UsdGeom.Cylinder) else fcl.Capsule
            geometry = constructor(float(shape.GetRadiusAttr().Get()), float(shape.GetHeightAttr().Get()))
            axis = shape.GetAxisAttr().Get()
            orient = orient @ ({"X": rotation("Y", math.pi / 2),
                                "Y": rotation("X", -math.pi / 2), "Z": np.eye(3)}[axis])
        elif prim.IsA(UsdGeom.Mesh):
            approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            if approximation not in ("convexHull", "none", None):
                raise ValueError(f"Unsupported collision approximation {approximation}: {prim.GetPath()}")
            points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float) * scale
            hull = trimesh.convex.convex_hull(points)
            faces = np.column_stack((np.full(len(hull.faces), 3), hull.faces)).astype(np.int32)
            geometry = fcl.Convex(np.asarray(hull.vertices), len(faces), faces.ravel())
        else:
            raise ValueError(f"Unsupported collider {prim.GetTypeName()}: {prim.GetPath()}")
        result.append(Part(geometry, orient, matrix[:3, 3], str(prim.GetPath())))
    if not result:
        raise ValueError(f"No colliders in {filename}")
    return result


def visual_points(filename):
    """Geometry points used only for support-height candidates and size checking."""
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    points = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim) * root_inverse).T
        p = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        points.append(p @ matrix[:3, :3].T + matrix[:3, 3])
    if not points:
        raise ValueError(f"No visual meshes in {filename}")
    return np.concatenate(points)


class CollisionWorld:
    """FCL broadphase scoped to this asset and the accepted load only."""
    def __init__(self, asset_dir):
        import fcl
        from .asset import BODY_POSITIONS, COMPONENT_FILES
        self.fcl = fcl
        self.parts = {}
        self.points = {}
        self.static_objects = []
        self.placed_objects = []
        self.manager = fcl.DynamicAABBTreeCollisionManager()
        self.names = {}
        for body, file in COMPONENT_FILES.items():
            parts = collision_parts(Path(asset_dir) / file)
            objects = self.transform(parts, np.eye(3), BODY_POSITIONS[body])
            self.static_objects.extend(objects)
            for obj in objects:
                self.names[id(obj)] = body
        self.manager.registerObjects(self.static_objects)
        self.manager.setup()
        for kind in ORDER:
            filename = Path(asset_dir) / "tableware" / (kind + ".usdc")
            self.parts[kind] = collision_parts(filename)
            self.points[kind] = visual_points(filename)

    def transform(self, parts, orient, position):
        fcl = self.fcl
        return [fcl.CollisionObject(part.geometry, fcl.Transform(orient @ part.rotation,
                    orient @ part.translation + position)) for part in parts]

    def candidate_objects(self, candidate):
        from .asset import BODY_POSITIONS
        position = np.asarray(candidate["position"]) + BODY_POSITIONS[candidate["rack"]]
        orient = matrix_xyzw(candidate["quaternion_xyzw"])
        return self.transform(self.parts[candidate["kind"]], orient, position)

    def collides(self, objects):
        fcl = self.fcl
        data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
        for obj in objects:
            self.manager.collide(obj, data, fcl.defaultCollisionCallback)
            if data.result.is_collision:
                return True
        return False

    def add(self, candidate):
        objects = self.candidate_objects(candidate)
        self.manager.registerObjects(objects)
        self.manager.update()
        self.placed_objects.extend(objects)

    def reset_load(self, entries):
        for obj in self.placed_objects:
            self.manager.unregisterObject(obj)
        self.placed_objects = []
        self.manager.update()
        for entry in entries:
            self.add(entry)


def geometry_hashes(directory):
    directory = Path(directory)
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(directory.rglob("*.usdc"))}


def _candidate(kind, rack, slot, x, y, orient, points, floor, variant):
    z = float(floor + .002 - (points @ orient.T)[:, 2].min())
    return {"kind": kind, "rack": rack, "slot": slot, "variant": variant,
            "position": [float(x), float(y), z], "quaternion_xyzw": quaternion_xyzw(orient)}


def _sloped_candidate(kind, slot, mouth_x, y, lean, points, floor, yaw=0):
    side = np.sign(mouth_x)
    angle = math.radians(side * lean)
    orient = rotation("Y", angle) @ rotation("Z", math.radians(yaw)) @ rotation("X", math.pi)
    normal = np.array([math.sin(angle), 0., math.cos(angle)])
    half_height = .080 if kind == "tumbler" else .050
    x = mouth_x + half_height * math.sin(angle)
    z = (floor * normal[2] + .002 - ((points @ orient.T) @ normal).min()
         - normal[0] * (x-mouth_x)) / normal[2]
    return {"kind": kind, "rack": "UpperRack", "slot": slot,
            "variant": f"yaw{yaw}_lean{lean}", "position": [float(x), float(y), float(z)],
            "quaternion_xyzw": quaternion_xyzw(orient)}


def candidates(world):
    """Finite photo-based loading patterns; sizes never change during packing."""
    from .geometry import lower_tine_positions, upper_tine_positions

    result = {kind: [] for kind in ORDER}
    # Use the outer pairs of rows as front/rear plate supports. The middle
    # combs remain part of the collision geometry for every candidate.
    teeth, rows = lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    lower_banks = (("front", float((rows[0]+rows[1])/2)),
                   ("rear", float((rows[-2]+rows[-1])/2)))
    for kind in ("dinner_plate", "salad_plate"):
        for bank, y in lower_banks:
            for index, x in enumerate(mids):
                # Reserve the photographed front-right bowl area.
                if bank == "front" and x > .105:
                    continue
                for lean, offset in ((-4, .008 if kind == "dinner_plate" else .004), (-8, .010)):
                    orient = rotation("Y", math.radians(90 - lean))
                    result[kind].append(_candidate(kind, "LowerRack", f"lower_{bank}_{index:02d}",
                        x + offset, y, orient, world.points[kind], .006, f"lean{lean}_offset{offset}"))
    _, upper_ys = upper_tine_positions()
    for index, y in enumerate((upper_ys[:-1] + upper_ys[1:])/2):
        for lean, offset in ((8, .010), (4, .008)):
            orient = rotation("X", math.radians(90 - lean))
            result["saucer"].append(_candidate("saucer", "UpperRack", f"saucer_{index:02d}",
                0, y + offset, orient, world.points["saucer"], -.0081, f"lean{lean}_offset{offset}"))
    for side in (-1, 1):
        for index, y in enumerate(np.arange(-.210, .2151, .085)):
            for x, lean, lift in ((.190, 8, .010), (.180, 16, .010), (.190, 4, .012), (.160, 24, .014)):
                c = _sloped_candidate("tumbler", f"glass_{side}_{index}",
                    side * x, y, lean, world.points["tumbler"], .001233)
                c["position"][2] += lift
                c["variant"] += f"_x{x}_lift{lift}"
                result["tumbler"].append(c)
    for side in (-1, 1):
        for index, y in enumerate(np.arange(-.205, .2051, .125)):
            for yaw in (90, 270, 0, 180):
                for x, lean, lift in ((.100, 11, .006), (.090, 15, .006)):
                    c = _sloped_candidate("mug", f"mug_{side}_{index}",
                        side * x, y, lean, world.points["mug"], -.0096, yaw)
                    c["position"][2] += lift
                    c["variant"] += f"_x{x}_lift{lift}"
                    result["mug"].append(c)
    # Bowls only in the visible front-right zone; back is occupied by the basket.
    # Base origins are left of their tilted mouths. This pair has separated
    # depth slabs along its common normal, so neither bowl nests in the other.
    for index, x in enumerate((.075, .155)):
        result["bowl"].append(_candidate("bowl", "LowerRack", f"bowl_pair_{index}",
            x, -.140, rotation("Y", math.radians(65)), world.points["bowl"], .007, "tilt65"))
    for ix, x in enumerate(np.arange(.125, .226, .020)):
        for iy, y in enumerate(np.arange(-.205, -.084, .020)):
            for tilt in (55, 65, 75):
                orient = rotation("Y", math.radians(tilt))
                result["bowl"].append(_candidate("bowl", "LowerRack", f"bowl_{ix}_{iy}",
                    x, y, orient, world.points["bowl"], .007, f"tilt{tilt}"))
    cutlery_pattern = json.loads(Path(__file__).with_name("cutlery_candidates.json").read_text())
    for kind in CUTLERY:
        result[kind] = cutlery_pattern["patterns"][kind]
    return result


def plan_full_load(asset_dir, output, banned=()):
    """Greedy first-fit over the frozen finite candidate pattern, with an audit log."""
    from .tableware import CATALOG
    from .asset import BODY_POSITIONS
    asset_dir, output = Path(asset_dir), Path(output)
    world = CollisionWorld(asset_dir)
    pools = candidates(world)
    accepted, rejected, occupied = [], [], set()
    banned = set(banned)
    visited = set()

    def attempt(kind):
        for candidate in pools[kind]:
            key = f'{kind}:{candidate["slot"]}:{candidate["variant"]}'
            if key in visited:
                continue
            visited.add(key)
            if candidate["slot"] in occupied or key in banned:
                rejected.append({"candidate": key, "reason": "occupied_slot" if key not in banned else "physics_rejected"})
                continue
            objects = world.candidate_objects(candidate)
            if world.collides(objects):
                rejected.append({"candidate": key, "reason": "authored_collider_overlap"})
                continue
            entry = dict(candidate, id=f"{kind}_{sum(o['kind']==kind for o in accepted)+1:03d}", candidate_key=key)
            accepted.append(entry)
            occupied.add(candidate["slot"])
            world.add(candidate)
            return True
        return False

    attempt("bowl")
    attempt("bowl")
    # Complete balanced rounds establish variety before residual saturation.
    for _ in range(6):
        start = len(accepted)
        rejection_start = len(rejected)
        previous_visited, previous_occupied = set(visited), set(occupied)
        complete = True
        for kind in ORDER:
            if kind != "bowl":
                complete = attempt(kind) and complete
        if not complete:
            del accepted[start:]
            del rejected[rejection_start:]
            visited.clear()
            visited.update(previous_visited)
            occupied.clear()
            occupied.update(previous_occupied)
            world.reset_load(accepted)
            break
    passes = 0
    cutlery_saturated = False
    incomplete_bundles = []
    while True:
        passes += 1
        added = sum(attempt(kind) for kind in ORDER if kind not in CUTLERY)
        if not cutlery_saturated:
            start, rejection_start = len(accepted), len(rejected)
            previous_visited, previous_occupied = set(visited), set(occupied)
            outcomes = [attempt(kind) for kind in CUTLERY]
            if all(outcomes):
                added += len(CUTLERY)
            else:
                incomplete_bundles.append({"pass": passes, "available": dict(zip(CUTLERY, outcomes)),
                    "reason": "No complete fork/knife/tablespoon/teaspoon bundle fits; provisional items rolled back."})
                del accepted[start:]
                del rejected[rejection_start:]
                visited.clear()
                visited.update(previous_visited)
                occupied.clear()
                occupied.update(previous_occupied)
                world.reset_load(accepted)
                cutlery_saturated = True
        if not added:
            break
    counts = dict(Counter(item["kind"] for item in accepted))
    manifest = {"schema_version": 1, "asset_dir": str(asset_dir.resolve()),
                "coordinate_system": "component-local metres; quaternion XYZW; initial closed appliance",
                "objects": accepted, "catalog": CATALOG, "counts": counts,
                "body_positions_m": BODY_POSITIONS, "geometry_hashes": geometry_hashes(asset_dir),
                "packing": {"status": "FCL candidates only; physics validation required",
                            "claim": "saturated within the frozen candidate patterns using individual dishes and complete four-piece cutlery bundles; not a global maximum",
                            "candidate_pattern_sha256": hashlib.sha256(Path(__file__).with_name("cutlery_candidates.json").read_bytes()).hexdigest(),
                            "candidate_count": sum(map(len, pools.values())), "residual_passes": passes,
                            "visited_count": len(visited), "rejections": rejected,
                            "cutlery_policy": "complete four-piece bundles only",
                            "incomplete_cutlery_bundles": incomplete_bundles,
                            "forbidden_shortcuts": ["resizing props", "nested bowls", "stacked cups", "fixed dishes", "invisible supports"]}}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest
