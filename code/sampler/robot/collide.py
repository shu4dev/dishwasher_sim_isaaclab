"""FCL model of the UR5e + 2F-85 for collision-aware IK choice and joint planning in the Frigidaire scene.

Built inside the Kit process from the SPAWNED stage: each arm link's collision meshes (convex hull per mesh, in
the Isaac link frame) and one hull of the whole open gripper (in wrist_3_link's frame). The fixed offsets between
the Isaac link frames and the analytic kinematics' frames (``kin.fk_all_links``) are calibrated once from the
simulator at a known joint state, so FK alone places the geometry afterwards (no simulator in the loop).
Checked against: the appliance components of an ``InitialCollisionChecker`` (tub, door, racks, basket at their
current frames), the counter slab, every dish body except ``ignore``, and optionally a held dish carried with
the TCP. A ``margin`` inflates nothing: FCL distance < margin counts as a collision (clearance). The held dish is checked by
the bench checker's pair rule (up to 1 mm penetration passes), or, with ``held_clear_m`` set (plans/2026-11-04-bowl-carry-
clearance.md, amended: the carry clearance rule), by an FCL distance of at least that much to every scene body.
"""
from __future__ import annotations

import numpy as np

from . import kin

ARM_LINKS = ("shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
MARGIN_M = .015
HELD_ARM_MARGIN_M = .002      # plans/2026-10-27-bowls-lip-plates-fix.md: the welded dish may come this close to the arm links


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
    paths = list(robot.root_view.link_paths[0])

    def live(i):
        p = robot.data.body_link_pos_w.torch[0, i].cpu().numpy()
        q = robot.data.body_link_quat_w.torch[0, i].cpu().numpy()   # XYZW (Isaac Lab 3.0)
        return T_of(p, q)
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
    def __init__(self, checker, slab, hulls, X_link, T_wb, T_w3_tcp, margin=MARGIN_M, held_margin=None):
        """``X_link[link]`` = inv(T_wb @ fk_link(q)) @ T_isaac_link (constant); ``gripper`` rides on wrist_3_link.
        ``held_margin``: the clearance of the held dish against the arm's own links (default ``margin``; the arm keeps
        ``margin`` against the scene)."""
        import fcl
        self.fcl, self.checker, self.slab, self.hulls, self.X = fcl, checker, slab, hulls, X_link
        self.T_wb, self.T_w3_tcp, self.margin = np.asarray(T_wb), np.asarray(T_w3_tcp), margin
        self.held_margin = margin if held_margin is None else held_margin
        self.held_clear_m = None      # the held dish's scene clearance (m); None = the checker's pair rule (1 mm overlap)
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

    def _near(self, obj, body, margin=None):
        req, margin = self.fcl.DistanceRequest(), self.margin if margin is None else margin
        for other in body["objects"]:
            res = self.fcl.DistanceResult()
            if self.fcl.distance(obj, other, req, res) < margin:
                return True
        return False

    def _within(self, body, other, d):
        """True when some piece of ``body`` is closer than ``d`` to some piece of ``other`` (FCL distance; the tree traversal
        is pruned at ``d``, so a clear pair costs about what a collision query costs)."""
        fcl, req, hit = self.fcl, self.fcl.DistanceRequest(), [False]

        def cb(o1, o2, _data):
            dd = fcl.distance(o1, o2, req)
            if dd < d:
                hit[0] = True
                return True, dd
            return False, d                           # the pruning bound stays d
        body["manager"].distance(other["manager"], None, cb)
        return hit[0]

    def targets(self, ignore=()):
        """[(name, body)]: the appliance components, the counter slab, the static boxes and the dishes not in ``ignore``."""
        out = list((self.checker.components or {}).items())
        if self.slab is not None:
            out.append(("Counter", self.slab))
        out += list(self.statics.items())
        return out + [(oid, b) for oid, b in self.dishes.items() if oid not in ignore]

    def held_body(self, q, held):
        """The held dish's checker body at joint state q (``held`` = (kind, M_tcp_obj, id))."""
        kind, M, hid = held
        T = self.link_poses(q)["wrist_3_link"] @ self.T_w3_tcp @ np.asarray(M)
        return self.checker._body(kind, {"position_m": T[:3, 3].tolist(), "quaternion_xyzw": _quat(T[:3, :3])}, hid)

    def held_clearance(self, q, held, ignore=(), cutoff=.10):
        """(d, name): the smallest FCL distance of the held dish at q to the scene (<= 0 touching; inf = nothing within
        ``cutoff`` by box)."""
        body, hid = self.held_body(q, held), held[2]
        blo, bhi = body["bounds"]
        best = (np.inf, None)
        for name, other in self.targets(ignore):
            if name == hid:
                continue
            olo, ohi = other["bounds"]
            gap = float(max(0., np.max(np.r_[olo - bhi, blo - ohi])))
            if gap > min(cutoff, best[0]):
                continue
            data = self.fcl.DistanceData(request=self.fcl.DistanceRequest())
            body["manager"].distance(other["manager"], data, self.fcl.defaultDistanceCallback)
            dd = float(data.result.min_distance)
            if dd < best[0]:
                best = (dd, name)
        return best

    def collisions(self, q, ignore=(), held=None, skip_links=("shoulder_link",), first=True, gripper_may_touch=(),
                   held_scene=True):
        """[(link, what)] of arm parts within the margin of the scene; ``held`` = (body_kind, M_tcp_obj, id);
        ``gripper_may_touch``: dishes only the GRIPPER may touch (the one being grasped), never the arm links;
        ``held_scene`` False: the held dish only against the arm's links (its scene contacts are not asked for)."""
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
            hc = self.held_clear_m
            for name, other in (targets if held_scene else ()):
                if name == hid:
                    continue
                if hc is not None:                    # the carry clearance rule: hc from every scene body (FCL distance)
                    (blo_, bhi_), (olo_, ohi_) = body["bounds"], other["bounds"]
                    if np.any(bhi_ + hc < olo_) or np.any(ohi_ + hc < blo_) or not self._within(body, other, hc):
                        continue
                elif self.checker.pair(body, other)["valid"]:
                    continue
                hits.append(("held", name))
                if first:
                    return hits
            # the carried dish vs the robot's own links (it hangs ~7 cm beside the pads and swings), at held_margin
            blo, bhi, hm = body["bounds"][0], body["bounds"][1], self.held_margin
            for link, o, lo, hi in self.arm_objects(q, ("shoulder_link",)):
                if link == "gripper" or np.any(hi + hm < blo) or np.any(bhi + hm < lo):
                    continue
                if self._near(o, body, hm):
                    hits.append(("held", link))
                    if first:
                        return hits
        return hits

    def free(self, q, **kw):
        ok = getattr(self, "carry_ok", None)          # D-ring carry (2026-10-29): a callable of q, the held bowl mouth down and
        if ok is not None and kw.get("held") is not None and not ok(q):       # the hand above it, on every planned config
            return False
        return not self.collisions(q, **kw)

    def fall_clearance(self, T_tcp, held, drops, q=None):
        """plans/2026-10-27-bowls-plates-down.md, the bowls' fall check: the held dish released at the TCP pose ``T_tcp``
        and displaced straight down (world -z) by each of ``drops`` (m), against the open gripper's hulls and wrist_3 (both
        rigid with the TCP), plus wrist_1 and wrist_2 at joint state ``q`` (the drop posture) when given. FCL distance,
        no margin (0 = touching or overlapping). {"min_mm", "at_drop_mm", "link", "per_link_mm"}."""
        fcl = self.fcl
        kind, M, hid = held
        W3 = np.asarray(T_tcp) @ np.linalg.inv(self.T_w3_tcp)
        poses = {"gripper": W3, "wrist_3_link": W3}
        if q is not None:
            lp = self.link_poses(q)
            poses.update({k: lp[k] for k in ("wrist_1_link", "wrist_2_link")})
        mgrs = {}
        for link, T in poses.items():
            objs = [fcl.CollisionObject(g, fcl.Transform(T[:3, :3], T[:3, 3])) for g, _ in self.hulls.get(link, ())]
            if objs:
                m = fcl.DynamicAABBTreeCollisionManager()
                m.registerObjects(objs)
                m.setup()
                mgrs[link] = m
        best, per = (np.inf, None, None), {}
        for d in drops:
            T = np.asarray(T_tcp) @ np.asarray(M)
            body = self.checker._body(kind, {"position_m": [float(T[0, 3]), float(T[1, 3]), float(T[2, 3] - d)],
                                             "quaternion_xyzw": _quat(T[:3, :3])}, hid)
            for link, m in mgrs.items():
                data = fcl.DistanceData(request=fcl.DistanceRequest())
                body["manager"].distance(m, data, fcl.defaultDistanceCallback)
                dist = max(0., float(data.result.min_distance))
                per[link] = min(per.get(link, np.inf), dist)
                if dist < best[0]:
                    best = (dist, d, link)
        return {"min_mm": round(best[0] * 1e3, 1), "at_drop_mm": round(best[1] * 1e3), "link": best[2],
                "per_link_mm": {k: round(v * 1e3, 1) for k, v in per.items()}}

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
        p = robot.data.body_link_pos_w.torch[0, i].cpu().numpy()
        q = robot.data.body_link_quat_w.torch[0, i].cpu().numpy()   # XYZW (Isaac Lab 3.0)
        T_sim = T_of(p, q)
        X[link] = np.linalg.inv(np.asarray(T_wb) @ fk[link]) @ T_sim
    return X


def _bounds():
    """The planners' joint bounds: rig.POSTURE_LIMITS (+-190 deg, elbow +-180; plans/2026-10-27-posture-fix.md), 0.02 in."""
    from .rig import LIMIT_MARGIN_RAD, POSTURE_LIMITS
    return POSTURE_LIMITS[:, 0] + LIMIT_MARGIN_RAD, POSTURE_LIMITS[:, 1] - LIMIT_MARGIN_RAD


EDGE_RES_RAD = .02            # plans/2026-11-04-bowl-carry-clearance.md (amended): edges checked at the followed 0.02 rad (was 0.03)
NARROW_RETRY_S = 180.         # plans/2026-11-04-fifteen-of-fifteen.md: the narrow passages' retry budget (rrt_connect's retry_s)


def rrt_connect_np(world, q0, goals, time_s=10., seed=1, collide_kw=None, resolution=EDGE_RES_RAD, rng_range=.3, shortcut_s=2.,
                   all_goals=False):
    """RRT-Connect in joint space without OMPL (the dishsim-l4 container has no ompl, 2026-10-23): same bounds, range
    and edge resolution as ``rrt_connect``; random shortcuts for ``shortcut_s`` (at most 30 tries per second), then
    waypoints <= 0.02 rad apart (each edge checked at the same 0.02 rad points that are followed). None on timeout.
    ``all_goals`` (plans/2026-11-04-fifteen-of-fifteen.md): every goal is a root of the goal tree (the path ends at the goal it
    reached, ``reached_goal``); False (the default) = the first goal only."""
    if resolution > EDGE_RES_RAD:
        raise ValueError(f"RRT-Connect edge resolution {resolution} rad is coarser than the followed {EDGE_RES_RAD} rad")
    import time
    collide_kw = collide_kw or {}
    rng = np.random.default_rng(int(seed) or 1)
    lo, hi = _bounds()

    def edge_free(a, b):
        n = max(2, int(np.ceil(float(np.abs(b - a).max()) / resolution)) + 1)
        return all(world.free(a + (b - a) * s, **collide_kw) for s in np.linspace(0., 1., n)[1:])

    def extend(tree, q):
        nodes = np.asarray([n for n, _ in tree])
        i = int(np.argmin(np.linalg.norm(nodes - q, axis=1)))
        d = q - nodes[i]
        dist = float(np.linalg.norm(d))
        new = q if dist <= rng_range else nodes[i] + d * rng_range / dist
        if not edge_free(nodes[i], new):
            return "trapped"
        tree.append((new, i))
        return "reached" if dist <= rng_range else "advanced"

    def branch(tree, k):
        out = []
        while k >= 0:
            out.append(tree[k][0])
            k = tree[k][1]
        return out

    ta = [(np.asarray(q0, dtype=float), -1)]
    tb = [(np.asarray(g, dtype=float), -1) for g in (goals if all_goals else goals[:1])]   # several roots: a forest
    a_is_start = True
    t_end = time.monotonic() + float(time_s)
    path = None
    while time.monotonic() < t_end and path is None:
        if extend(ta, rng.uniform(lo, hi)) != "trapped":
            q_new = ta[-1][0]
            r = "advanced"
            while r == "advanced" and time.monotonic() < t_end:
                r = extend(tb, q_new)
            if r == "reached":
                pa, pb = branch(ta, len(ta) - 1), branch(tb, len(tb) - 1)
                path = pa[::-1] + pb[1:] if a_is_start else pb[::-1] + pa[1:]
        ta, tb, a_is_start = tb, ta, not a_is_start
    if path is None:
        return None
    t_sc = time.monotonic() + min(float(shortcut_s), float(time_s))   # shortcuts get their own budget (a long edge with a
    for _ in range(max(60, int(30 * shortcut_s))):                     # carried dish costs hundreds of checks: unbounded,
                                                                       # it ran > 10 min; 2 s and 60 tries until 2026-10-27)
        if len(path) < 3 or time.monotonic() > t_sc:
            break
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i > 1 and edge_free(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(float(np.abs(b - a).max()) / .02)))
        out += [a + (b - a) * s for s in np.linspace(0., 1., n + 1)[1:]]
    return out


def rrt_connect(world, q0, goals, time_s=10., seed=1, collide_kw=None, resolution=EDGE_RES_RAD, shortcut_s=2., retry_s=0.):
    """OMPL RRT-Connect in joint space; returns a list of joint waypoints (simplified, interpolated) or None.
    Without OMPL: ``rrt_connect_np`` (first goal only). ``retry_s`` > 0 (plans/2026-11-04-fifteen-of-fifteen.md, the narrow
    passages; NARROW_RETRY_S): no path in ``time_s`` -> one more search of ``retry_s`` seconds (RNG seed + 1) rooted at EVERY
    goal (the caller lists the other passing descent starts after the first; ``reached_goal`` says which one the path ends
    at), the margins and the edge resolution unchanged; 0 (the default) = the single search of before."""
    path = _rrt_connect(world, q0, goals, time_s, seed, collide_kw, resolution, shortcut_s, False)
    if path is None and retry_s > 0:
        path = _rrt_connect(world, q0, goals, retry_s, int(seed) + 1, collide_kw, resolution, shortcut_s, True)
    return path


def reached_goal(path, goals):
    """The index of the goal a planned path ends at (the nearest by the largest joint difference); None without a path."""
    if path is None:
        return None
    return int(np.argmin([float(np.abs(np.asarray(path[-1], dtype=float) - np.asarray(g, dtype=float)).max()) for g in goals]))


def _rrt_connect(world, q0, goals, time_s, seed, collide_kw, resolution, shortcut_s, all_goals):
    """rrt_connect's single search (OMPL's goal states always hold every goal; ``all_goals`` roots rrt_connect_np at each)."""
    try:
        from ompl import base as ob, geometric as og, util as ou
    except ImportError:
        return rrt_connect_np(world, q0, goals, time_s=time_s, seed=seed, collide_kw=collide_kw, resolution=resolution,
                              shortcut_s=shortcut_s, all_goals=all_goals)
    collide_kw = collide_kw or {}
    ou.setLogLevel(ou.LOG_WARN)
    try:
        ou.RNG.setSeed(int(seed) or 1)
    except Exception:
        pass
    space = ob.RealVectorStateSpace(6)
    bounds = ob.RealVectorBounds(6)
    lo, hi = _bounds()
    for i in range(6):                                   # the posture limits (+-190 deg, the elbow +-180)
        bounds.setLow(i, float(lo[i]))
        bounds.setHigh(i, float(hi[i]))
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
    """(hulls, X_link) from data/results/robot/arm_model.json (code/sampler/robot/scripts/frigidaire_arm_model.py)."""
    import json
    d = json.loads(open(path).read())
    hulls = {k: [_hull(np.asarray(v)) for v in lst] for k, lst in d["hulls"].items()}
    return hulls, {k: np.asarray(v) for k, v in d["X_link"].items()}
