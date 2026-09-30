"""FCL model of the UR5e + 2F-85 for collision-aware IK choice and joint planning in the Frigidaire scene.

Built inside the Kit process from the SPAWNED stage: each arm link's collision meshes (convex hull per mesh, in
the Isaac link frame) and one hull of the whole open gripper (in wrist_3_link's frame). The fixed offsets between
the Isaac link frames and the analytic kinematics' frames (``kin.fk_all_links``) are calibrated once from the
simulator at a known joint state, so FK alone places the geometry afterwards (no simulator in the loop).
Checked against: the appliance components of an ``InitialCollisionChecker`` (tub, door, racks, basket at their
current frames), the counter slab, every dish body except ``ignore``, and optionally a held dish carried with
the TCP. A ``margin`` inflates nothing: FCL distance < margin counts as a collision (clearance).
"""
from __future__ import annotations

import numpy as np

from . import kin

ARM_LINKS = ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
MARGIN_M = .015


def _hull(points):
    import fcl
    import trimesh
    hull = trimesh.convex.convex_hull(np.asarray(points, dtype=float))
    faces = np.column_stack((np.full(len(hull.faces), 3), hull.faces)).astype(np.int32)
    return fcl.Convex(np.asarray(hull.vertices), len(faces), faces.ravel()), hull.vertices


def link_hulls(stage, robot_root="/World/Robot", robot=None):
    """{link: [(fcl.Convex, vertices)]} in each Isaac link prim's frame; the gripper's hull goes to wrist_3_link.

    The gripper hangs off wrist_3 by a PHYSICS joint, so its authored USD transforms do not place it (measured:
    a hull built from them sat sideways at the flange). With ``robot`` (after ``sim.reset``), each gripper
    collider is placed by its rigid body's LIVE pose and expressed in the live wrist_3 frame."""
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    out = {}

    def rel(frame_prim, prim):
        return np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(frame_prim)).T) @ \
            np.asarray(cache.GetLocalToWorldTransform(prim)).T

    def pts_of(prim):
        pts = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        return np.c_[pts, np.ones(len(pts))]

    for link in ARM_LINKS:
        lp = stage.GetPrimAtPath(f"{robot_root}/{link}")
        hulls = []
        for prim in Usd.PrimRange(lp, Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh):
                hulls.append(_hull((rel(lp, prim) @ pts_of(prim).T).T[:, :3]))
        out[link] = hulls
    if robot is None:
        return out
    from .rig import T_of
    paths = list(robot.root_physx_view.link_paths[0])

    def live(i):
        p = robot.data.body_link_pos_w[0, i].cpu().numpy()
        qw = robot.data.body_link_quat_w[0, i].cpu().numpy()
        return T_of(p, (qw[1], qw[2], qw[3], qw[0]))
    w3 = live(paths.index(f"{robot_root}/wrist_3_link"))
    grip = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{robot_root}/Gripper"), Usd.TraverseInstanceProxies()):
        if not (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh)):
            continue
        body = prim
        while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body or str(body.GetPath()) not in paths:
            continue
        world = live(paths.index(str(body.GetPath()))) @ rel(body, prim) @ pts_of(prim).T
        grip.append((np.linalg.inv(w3) @ world).T[:, :3])
    if grip:                                          # one hull PER collider: a single hull would fill the jaw gap
        out["gripper"] = [_hull(g) for g in grip]
    return out


class ArmWorld:
    def __init__(self, checker, slab, hulls, X_link, T_wb, T_w3_tcp, margin=MARGIN_M):
        """``X_link[link]`` = inv(T_wb @ fk_link(q)) @ T_isaac_link (constant); ``gripper`` rides on wrist_3_link."""
        import fcl
        self.fcl, self.checker, self.slab, self.hulls, self.X = fcl, checker, slab, hulls, X_link
        self.T_wb, self.T_w3_tcp, self.margin = np.asarray(T_wb), np.asarray(T_w3_tcp), margin
        self.dishes, self.n_checks, self.statics = {}, 0, {}

    def add_static_box(self, name, center, size):
        """A static box body (e.g. the pedestal column, the floor) in the checker's body format."""
        fcl = self.fcl
        c, h = np.asarray(center, dtype=float), np.asarray(size, dtype=float) / 2
        o = fcl.CollisionObject(fcl.Box(*size), fcl.Transform(np.eye(3), c))
        m = fcl.DynamicAABBTreeCollisionManager()
        m.registerObjects([o])
        m.setup()
        self.statics[name] = {"id": name, "kind": "static", "objects": [o], "manager": m, "bounds": (c - h, c + h)}

    def set_dishes(self, bodies):
        self.dishes = dict(bodies)

    def link_poses(self, q):
        fk = kin.fk_all_links(np.asarray(q, dtype=float))
        poses = {link: self.T_wb @ fk[link] @ self.X[link] for link in ARM_LINKS}
        poses["gripper"] = poses["wrist_3_link"]
        return poses

    def arm_objects(self, q, skip_links=()):
        objs = []
        for link, T in self.link_poses(q).items():
            if link in skip_links:
                continue
            for geom, verts in self.hulls.get(link, ()):
                o = self.fcl.CollisionObject(geom, self.fcl.Transform(T[:3, :3], T[:3, 3]))
                pts = verts @ T[:3, :3].T + T[:3, 3]
                objs.append((link, o, pts.min(0), pts.max(0)))
        return objs

    def _near(self, obj, body):
        req = self.fcl.DistanceRequest()
        for other in body["objects"]:
            res = self.fcl.DistanceResult()
            if self.fcl.distance(obj, other, req, res) < self.margin:
                return True
        return False

    def collisions(self, q, ignore=(), held=None, skip_links=("shoulder_link",), first=True, gripper_may_touch=()):
        """[(link, what)] of arm parts within the margin of the scene; ``held`` = (body_kind, M_tcp_obj, id);
        ``gripper_may_touch``: dishes only the GRIPPER may touch (the one being grasped), never the arm links."""
        self.n_checks += 1
        hits = []
        targets = []
        if self.checker.components:
            targets += [(name, b) for name, b in self.checker.components.items()]
        if self.slab is not None:
            targets.append(("Counter", self.slab))
        targets += list(self.statics.items())
        targets += [(oid, b) for oid, b in self.dishes.items() if oid not in ignore]
        for link, o, lo, hi in self.arm_objects(q, skip_links):
            for name, body in targets:
                blo, bhi = body["bounds"]
                if np.any(hi + self.margin < blo) or np.any(bhi + self.margin < lo):
                    continue
                if link == "gripper" and name in gripper_may_touch:
                    continue
                if self._near(o, body):
                    hits.append((link, name))
                    if first:
                        return hits
        if held is not None:
            kind, M, hid = held
            T_tcp = self.link_poses(q)["wrist_3_link"] @ self.T_w3_tcp
            T = T_tcp @ M
            from .rig import T_of  # noqa: F401  (pose dict below)
            body = self.checker._body(kind, {"position_m": T[:3, 3].tolist(), "quaternion_xyzw": _quat(T[:3, :3])}, hid)
            for name, other in targets:
                if name == hid:
                    continue
                if not self.checker.pair(body, other)["valid"]:
                    hits.append(("held", name))
                    if first:
                        return hits
            # the carried dish vs the robot's own links (it hangs ~7 cm beside the pads and swings)
            blo, bhi = body["bounds"]
            for link, o, lo, hi in self.arm_objects(q, ("shoulder_link",)):
                if link == "gripper" or np.any(hi + self.margin < blo) or np.any(bhi + self.margin < lo):
                    continue
                if self._near(o, body):
                    hits.append(("held", link))
                    if first:
                        return hits
        return hits

    def free(self, q, **kw):
        return not self.collisions(q, **kw)

    def path_free(self, q0, q1, step=.04, **kw):
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        n = max(2, int(np.ceil(float(np.abs(q1 - q0).max()) / step)) + 1)
        return all(self.free(q0 + (q1 - q0) * s, **kw) for s in np.linspace(0., 1., n))


def _quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat().tolist()        # xyzw


def calibrate(robot, q, T_wb):
    """X_link from the simulator's link poses at joint state ``q`` (arm joints in ARM order)."""
    from .rig import T_of
    fk = kin.fk_all_links(np.asarray(q, dtype=float))
    bodies = list(robot.body_names)
    X = {}
    for link in ARM_LINKS:
        i = bodies.index(link)
        p = robot.data.body_link_pos_w[0, i].cpu().numpy()
        qw = robot.data.body_link_quat_w[0, i].cpu().numpy()
        T_sim = T_of(p, (qw[1], qw[2], qw[3], qw[0]))
        X[link] = np.linalg.inv(np.asarray(T_wb) @ fk[link]) @ T_sim
    return X


def rrt_connect(world, q0, goals, time_s=10., seed=1, collide_kw=None, resolution=.03):
    """OMPL RRT-Connect in joint space; returns a list of joint waypoints (simplified, interpolated) or None."""
    from ompl import base as ob, geometric as og, util as ou
    collide_kw = collide_kw or {}
    ou.setLogLevel(ou.LOG_WARN)
    try:
        ou.RNG.setSeed(int(seed) or 1)
    except Exception:
        pass
    space = ob.RealVectorStateSpace(6)
    bounds = ob.RealVectorBounds(6)
    for i in range(6):                                   # the real joint limits (the elbow is +-pi, not +-2 pi)
        bounds.setLow(i, float(kin.JOINT_LIMITS[i, 0]) + .02)
        bounds.setHigh(i, float(kin.JOINT_LIMITS[i, 1]) - .02)
    space.setBounds(bounds)
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(lambda s: world.free(np.array([s[i] for i in range(6)]), **collide_kw))
    si = ss.getSpaceInformation()
    si.setStateValidityCheckingResolution(float(resolution / space.getMaximumExtent()))
    st = space.allocState()
    for i in range(6):
        st[i] = float(q0[i])
    ss.setStartState(st)
    goal = ob.GoalStates(si)
    for g in goals:
        gs = space.allocState()
        for i in range(6):
            gs[i] = float(g[i])
        goal.addState(gs)
    ss.setGoal(goal)
    planner = og.RRTConnect(si)
    planner.setRange(.3)
    ss.setPlanner(planner)
    if not ss.solve(float(time_s)) or not ss.haveExactSolutionPath():
        return None
    ss.simplifySolution(1.)
    path = ss.getSolutionPath()
    path.interpolate(max(2, int(path.length() / .02)))
    return [np.array([path.getState(k)[i] for i in range(6)]) for k in range(path.getStateCount())]


def load_model(path):
    """(hulls, X_link) from results/robot/arm_model.json (frigidaire/scripts/setup/frigidaire_arm_model.py)."""
    import json
    d = json.loads(open(path).read())
    hulls = {k: [_hull(np.asarray(v)) for v in lst] for k, lst in d["hulls"].items()}
    return hulls, {k: np.asarray(v) for k, v in d["X_link"].items()}
