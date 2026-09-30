"""Kit-side test harness of the robot episode (plan 2026-09-29-easy-s0, Phase 0.3, 0.4, 0.6 + media).

Construct AFTER Kit boots (every Kit import below is lazy). One ``Harness`` per trial; the episode's backend calls
``on_tick(frames)`` after EVERY physics step (robot moves, rack ramps, holds, the end check alike), so contacts,
invariants, samples and video frames cover the whole simulated timeline.

- Sleep: ``author_sleep_and_reports`` (before ``sim.reset``) writes ``physxRigidBody:sleepThreshold = 0`` on every
  dish prim (plan decision D10: on the prims, the benchmark backend is untouched) and on every robot link, and a
  zero-threshold ``PhysxContactReportAPI`` on the dishes and robot links; ``is_sleeping`` is polled as an invariant.
- Contacts: PhysX contact REPORTS (``get_contact_report`` per step): every pair TOUCHING (impulse > 0 or separation
  <= 0; pairs merely inside the contact offset are "near" and ignored), resolved to colliders (pads = the
  ``fingertips`` colliders; the rest of a finger is a finger side / knuckle), with the normal force (sum over the
  points of |impulse . normal| / dt), the total |impulse| / dt and the deepest separation. A pair begins at its
  first touching report and ends after ``CONTACT_GAP_TICKS`` without one (resting contacts flicker).
- Invariants (auto-fail): arm or palm touching anything (except the mount pairs present at reset), the carried dish
  touching anything outside pads, counter, racks and its at-close partners (D4), a teleport after reset, a sleeping
  body, a NaN frame, a joint-limit violation, a pad normal force above 235 N.
- Settle: the benchmark per-move settle (150 ticks, drift window of the last 60) lives in the episode; the plan's
  velocity routine (< 1 mm/s and 0.01 rad/s for 1 s, cap 5 s) is ``diag_settle`` (D3: a diagnostic column only).
- Media: a FIXED camera, 1280x720, one video frame per 8 physics steps (15 fps of simulated time) with the
  simulated time overlaid; key frames on events and on every first invariant trigger.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

PAD_FORCE_MAX_N = 235.           # Robotiq 2F-85 maximum grip force (plan Section 3)
SAMPLE_EVERY = 6                 # ticks between time-series rows (20 Hz of simulated time)
FRAME_EVERY = 8                  # ticks between video frames (15 fps of simulated time at 120 Hz)
SLEEP_CHECK_EVERY = 12           # ticks between is_sleeping polls
CONTACT_GAP_TICKS = 6            # a pair ends after this many ticks without a report (resting contacts flicker)
TOUCH_FORCE_N = 1e-3             # a reported pair TOUCHES when it carries impulse or its separation is <= 0; PhysX also
                                 # reports pairs that are merely within the contact offset (+1-2 mm, 0 N): those are
                                 # "near" pairs and never count as contacts (measured 2026-09-30, p0_upright3_legacy)
FINGER_LIMIT_TOL_RAD = .01       # PhysX joint-limit overshoot allowed on the gripper joints
DIAG_SETTLE = {"v_max_m_s": 1e-3, "w_max_rad_s": .01, "hold_s": 1., "cap_s": 5.}   # plan Phase 0.3 routine
CARRIED_ALLOWED = ("pad", "counter", "rack", "basket")   # D4 default list; the silverware basket counts as rack
                                                          # furniture (the benchmark's support rule seats it on the
                                                          # lower rack) -- an INTERPRETATION, flagged in the report
MOUNT_LINKS = ("base_link", "base_link_inertia", "base")                            # may rest on the pedestal
GRIPPER = "/World/Robot/Gripper/Robotiq_2F_85/"
DISHES = "/World/InitialStateDishes/"
APPLIANCE = "/World/InitialStateDishwasher/"
INVARIANTS = ("arm_or_palm_contact", "carried_contact", "teleport_after_reset", "sleeping_body", "nan_frame",
              "joint_limit", "pad_force")


class InvariantAbort(RuntimeError):
    """Raised from the tick hook when an auto-fail invariant fires and the profile aborts on it."""


def classify(path, roots=("/World/Robot",)):
    """(label, category) of a collider or rigid-body path. Categories: pad, finger, palm, arm, dish, rack, basket,
    appliance, counter, pedestal, ground, other. Robot labels carry a "<root name>:" prefix unless the root is the
    episode's /World/Robot."""
    p = str(path)
    for root in roots:
        if p == root or p.startswith(root + "/"):
            tag = "" if root == "/World/Robot" else root.rsplit("/", 1)[-1] + ":"
            rest = p[len(root) + 1:]
            if rest.startswith("Gripper/Robotiq_2F_85/"):
                body = rest[len("Gripper/Robotiq_2F_85/"):].split("/")[0]
                if body == "base_link":
                    return tag + "palm", "palm"
                side = "L" if body.startswith("left") else "R" if body.startswith("right") else "?"
                if "inner_finger" in body and "fingertips" in p:
                    return f"{tag}pad_{side}", "pad"
                return tag + body, "finger"
            return tag + (rest.split("/")[0] or "root"), "arm"
    if p.startswith(DISHES):
        return p.split("/")[3], "dish"
    if p.startswith(APPLIANCE):
        comp = p.split("/")[3]
        return comp, ("rack" if comp in ("LowerRack", "UpperRack") else "basket" if comp == "SilverwareBasket"
                      else "appliance")
    for name, cat in (("/World/Counter", "counter"), ("/World/Pedestal", "pedestal"), ("/World/Ground", "ground")):
        if p == name or p.startswith(name + "/"):
            return name.split("/")[-1], cat
    return p, "other"


def author_sleep_and_reports(stage, dish_paths, robot_root="/World/Robot", sleep_threshold=0.):
    """Before sim.reset(): never-sleeping (``sleep_threshold`` None = leave the asset values: ANALYSIS only),
    zero-threshold-reporting dishes and robot links. Returns the counts."""
    from pxr import PhysxSchema, Usd, UsdPhysics
    n = {"dish_sleep": 0, "dish_report": 0, "robot_sleep": 0, "robot_report": 0, "articulation_sleep": 0,
         "sleep_threshold": sleep_threshold}
    for path in dish_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"no rigid body at {path}")
        if sleep_threshold is not None:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(float(sleep_threshold))
            n["dish_sleep"] += 1
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        n["dish_report"] += 1
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if sleep_threshold is not None:
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(float(sleep_threshold))
                n["robot_sleep"] += 1
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
            n["robot_report"] += 1
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI) and sleep_threshold is not None:
            PhysxSchema.PhysxArticulationAPI(prim).CreateSleepThresholdAttr().Set(float(sleep_threshold))
            n["articulation_sleep"] += 1
    return n


def deinstance(stage, root):
    """SetInstanceable(False) on every instance under ``root`` (plan decision D12 widens ur5e.deinstance_gripper's
    root from the gripper to the whole robot when the arm-link press test shows instanced colliders make no
    contact). Returns the number of prims de-instanced."""
    from pxr import Usd
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.IsInstance():
            prim.SetInstanceable(False)
            n += 1
    return n


class ContactMonitor:
    """Per-step PhysX contact reports -> active pairs, per-pad normal force, begin/end events."""

    def __init__(self, dt, roots=("/World/Robot",)):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        self.iface = get_physx_simulation_interface()
        self.to_path = PhysicsSchemaTools.intToSdfPath
        self.dt = dt
        self.roots = tuple(roots)
        self._labels = {}
        self.active = {}          # key -> {"since": tick, "last": tick, "peak_N": f, "cats": (c0, c1), "colliders": set}
        self.step_pairs = {}      # key -> force of the latest step
        self.pad_N = {"pad_L": 0., "pad_R": 0.}
        self.pad_F = {"pad_L": 0., "pad_R": 0.}
        self.pad_peak = {"pad_L": 0., "pad_R": 0.}
        self.step_total = {}
        self.counts = {"FOUND": 0, "PERSIST": 0, "LOST": 0, "headers": 0, "points": 0, "near": 0}

    def _label(self, pid):
        lab = self._labels.get(pid)
        if lab is None:
            path = str(self.to_path(pid))
            lab = (*classify(path, self.roots), path)
            self._labels[pid] = lab
        return lab

    def poll(self, tick):
        """Read this step's report. Returns (begun, ended) lists of pair keys."""
        rep = self.iface.get_contact_report()
        headers, data = rep[0], rep[1]
        seen = {}
        pads = {k: 0. for k in self.pad_N}
        pads_total = {k: 0. for k in self.pad_N}
        for h in headers:
            kind = int(h.type)
            self.counts["headers"] += 1
            self.counts[("FOUND", "LOST", "PERSIST")[kind] if kind in (0, 1, 2) else "PERSIST"] += 1
            if kind == 1:                       # CONTACT_LOST: the gap rule below ends the pair
                continue
            a = self._label(h.collider0 or h.actor0)
            b = self._label(h.collider1 or h.actor1)
            if a[0] == b[0]:
                continue
            off, n = int(h.contact_data_offset), int(h.num_contact_data)
            force, normal, sep = 0., 0., math.inf
            for i in range(off, off + n):
                d = data[i]
                imp, nrm = d.impulse, d.normal
                force += math.sqrt(imp[0] * imp[0] + imp[1] * imp[1] + imp[2] * imp[2])
                normal += abs(imp[0] * nrm[0] + imp[1] * nrm[1] + imp[2] * nrm[2])
                sep = min(sep, float(d.separation))
            self.counts["points"] += n
            force /= self.dt
            normal /= self.dt
            if force < TOUCH_FORCE_N and sep > 0.:            # within the contact offset only: not a touch
                self.counts["near"] = self.counts.get("near", 0) + 1
                continue
            (x, y) = sorted((a, b), key=lambda t: t[0])
            key = (x[0], y[0])
            row = seen.setdefault(key, {"N": 0., "F": 0., "sep": math.inf, "cats": (x[1], y[1]), "colliders": set()})
            row["N"] += normal
            row["F"] += force
            row["sep"] = min(row["sep"], sep)
            row["colliders"].update((x[2].rsplit("/", 1)[-1], y[2].rsplit("/", 1)[-1]))
            for lab in (a, b):
                if lab[1] == "pad":
                    pads[lab[0]] = pads.get(lab[0], 0.) + normal
                    pads_total[lab[0]] = pads_total.get(lab[0], 0.) + force
        begun, ended = [], []
        for key, row in seen.items():
            act = self.active.get(key)
            if act is None:
                self.active[key] = {"since": tick, "last": tick, "peak_N": row["N"], "peak_F": row["F"], "cats": row["cats"],
                                    "min_sep_m": row["sep"], "colliders": set(row["colliders"])}
                begun.append(key)
            else:
                act["last"] = tick
                act["peak_N"] = max(act["peak_N"], row["N"])
                act["peak_F"] = max(act["peak_F"], row["F"])
                act["min_sep_m"] = min(act["min_sep_m"], row["sep"])
                act["colliders"].update(row["colliders"])
        for key in [k for k, v in self.active.items() if tick - v["last"] > CONTACT_GAP_TICKS]:
            ended.append((key, self.active.pop(key)))
        self.step_pairs = {k: v["N"] for k, v in seen.items()}      # normal force of the step, per touching pair
        self.step_total = {k: v["F"] for k, v in seen.items()}      # |impulse| / dt (normal + friction)
        self.pad_N = pads                                           # normal component on each pad collider
        self.pad_F = pads_total
        for k, v in pads.items():
            self.pad_peak[k] = max(self.pad_peak.get(k, 0.), v)
        return begun, ended

    def partners(self, label):
        return {b if a == label else a for (a, b) in self.active if label in (a, b)}

    def pairs_with(self, label):
        return {k: v for k, v in self.active.items() if label in k}


class Harness:
    """Contacts, invariants, samples, video and key frames of ONE trial."""

    def __init__(self, *, backend, robot, tlog, flags, ids, arm_joint_ids, joint_limits_arm, media=None):
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        self.backend, self.robot, self.tlog, self.flags, self.ids = backend, robot, tlog, flags, list(ids)
        self.media = media
        self.dt = backend.dt
        self.contacts = ContactMonitor(self.dt)
        self.arm = list(arm_joint_ids)
        self.arm_limits = np.asarray(joint_limits_arm, dtype=float)
        names = list(robot.joint_names)
        self.names = names
        self.gripper_joints = [i for i, n in enumerate(names) if i not in self.arm]
        lim = getattr(robot.data, "joint_pos_limits", None)
        self.all_limits = None if lim is None else lim[0].detach().cpu().numpy().astype(float)
        self.stage_id = omni.usd.get_context().get_stage_id()
        self.sim_iface = get_physx_simulation_interface()
        self.sleep_bodies = [(oid, PhysicsSchemaTools.sdfPathToInt(DISHES + oid)) for oid in self.ids]
        self.sleep_bodies += [(f"robot:{b}", PhysicsSchemaTools.sdfPathToInt(p)) for b, p in self._robot_body_paths()]
        self.sleep_unsupported = []
        self.armed = False
        self.mount_pairs = set()
        self.carried = None               # {"oid", "partners": set of labels allowed until they break}
        self.violations = []              # first occurrences: [{"name", "move", "sig", "line", "tick", "detail"}]
        self.first = {}                   # (name, move, sig) -> trial-log line
        self.tick_counts = {}             # name -> ticks in violation
        self.abort_on = bool(flags.invariants_abort)
        self.aborted = False
        self.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        self.last_frames = None
        self.tick0 = backend.tick_index
        self.robot_update = True
        self.scoring = False              # True during the benchmark end check: its put-back is scoring, not a trial teleport

    # ------------------------------------------------------------------ setup
    def _robot_body_paths(self):
        import omni.usd
        from pxr import Usd, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        out = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot")):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                out.append((prim.GetName(), str(prim.GetPath())))
        return out

    def arm_guard(self):
        """Called once the reset is complete: from now on a teleport is a violation; the robot-pedestal pairs present
        now (the base standing on its column) are the only exempt arm contacts."""
        self.mount_pairs = {k for k in self.contacts.active
                            if "pedestal" in self.contacts.active[k]["cats"] and "arm" in self.contacts.active[k]["cats"]}
        self.armed = True
        return sorted(self.mount_pairs)

    def trial_time(self):
        return (self.backend.tick_index - self.tick0) * self.dt

    # ------------------------------------------------------------------ carried dish (D4)
    def set_carried(self, oid):
        partners = {p for p in self.contacts.partners(oid)}
        self.carried = {"oid": oid, "partners": partners, "since_tick": self.backend.tick_index}
        return sorted(partners)

    def clear_carried(self):
        self.carried = None

    # ------------------------------------------------------------------ violations
    def violate(self, name, detail, frame=True, sig=None):
        """Record an auto-fail invariant. One trial-log row (+ a key frame) per (name, move, sig) -- sig separates e.g.
        two contact partners; ``tick_counts`` counts every tick in violation. Raises InvariantAbort when the profile
        aborts on invariants (headline) and this is the first one."""
        move = self.tlog.ctx.get("move")
        k = (name, move, sig)
        self.tick_counts[name] = self.tick_counts.get(name, 0) + 1
        if k not in self.first:
            shot = None
            if frame and self.media is not None:
                shot = self.media.key_frame(f"invariant_{name}", f"AUTO-FAIL: {name} {'' if sig is None else sig}")
            line = self.tlog.write("invariant", name=name, detail=detail, frame=shot, sig=sig, first_in_move=True)
            self.first[k] = line
            self.violations.append({"name": name, "tick": self.backend.tick_index, "move": move, "sig": sig, "line": line,
                                    "detail": detail})
            print(f"[INVARIANT] {name}: {detail}", flush=True)
        if self.abort_on and not self.aborted:
            self.aborted = True
            raise InvariantAbort(f"{name}: {detail}")

    def on_teleport(self, what):
        if self.scoring:
            self.tlog.write("note", what="scoring teleport (the benchmark end check's put-back after a rack-speed flake)", call=what)
            return
        if self.armed:
            self.violate("teleport_after_reset", {"call": what})

    # ------------------------------------------------------------------ per tick
    def on_tick(self, frames):
        tick = self.backend.tick_index
        if self.robot_update:
            self.robot.update(self.dt)
        self.last_frames = frames
        begun, ended = self.contacts.poll(tick)
        for key in begun:
            act = self.contacts.active[key]
            self.tlog.write("contact_begin", pair=list(key), cats=list(act["cats"]), normal_N=round(act["peak_N"], 3),
                            force_N=round(act["peak_F"], 3),
                            sep_m=round(act["min_sep_m"], 5) if math.isfinite(act["min_sep_m"]) else None,
                            colliders=sorted(act["colliders"])[:6])
        for key, act in ended:
            self.tlog.write("contact_end", pair=list(key), cats=list(act["cats"]), since_tick=act["since"],
                            duration_s=round((act["last"] - act["since"] + 1) * self.dt, 4),
                            peak_N=round(act["peak_N"], 3), peak_F=round(act["peak_F"], 3), colliders=sorted(act["colliders"])[:8],
                            min_sep_m=round(act["min_sep_m"], 5) if math.isfinite(act["min_sep_m"]) else None)
        for k, v in self.contacts.pad_N.items():
            self.pad_max_interval[k] = max(self.pad_max_interval.get(k, 0.), v)
        if self.armed:
            self._check(tick)
        if tick % SAMPLE_EVERY == 0:
            self._sample(frames)
        if self.media is not None and tick % FRAME_EVERY == 0:
            self.media.video_frame()

    def _check(self, tick):
        q = self.robot.data.joint_pos[0].detach().cpu().numpy()
        qd = self.robot.data.joint_vel[0].detach().cpu().numpy()
        if not (np.isfinite(q).all() and np.isfinite(qd).all()):
            self.violate("nan_frame", {"joint_pos_finite": bool(np.isfinite(q).all()), "joint_vel_finite": bool(np.isfinite(qd).all())})
        qa = q[self.arm]
        bad = [i for i in range(6) if qa[i] < self.arm_limits[i, 0] or qa[i] > self.arm_limits[i, 1]]
        if bad:
            self.violate("joint_limit", {"arm": {self.names[self.arm[i]]: round(float(qa[i]), 4) for i in bad},
                                         "limits": "UR description (kin.JOINT_LIMITS)"})
        if self.all_limits is not None:
            gbad = {self.names[i]: round(float(q[i]), 4) for i in self.gripper_joints
                    if q[i] < self.all_limits[i, 0] - FINGER_LIMIT_TOL_RAD or q[i] > self.all_limits[i, 1] + FINGER_LIMIT_TOL_RAD}
            if gbad:
                self.violate("joint_limit", {"gripper": gbad, "limits": "PhysX joint limits + 0.01 rad"})
        for pad, f in self.contacts.pad_N.items():
            if f > PAD_FORCE_MAX_N:
                self.violate("pad_force", {"pad": pad, "normal_N": round(f, 1), "max_N": PAD_FORCE_MAX_N}, sig=pad)
        for key, act in list(self.contacts.active.items()):
            if act["last"] != tick:
                continue
            cats = act["cats"]
            if ("arm" in cats or "palm" in cats) and key not in self.mount_pairs:
                if all(c in ("arm", "palm", "pad", "finger") for c in cats):          # robot self-pairs are off
                    continue
                if "pedestal" in cats and any(lab in MOUNT_LINKS for lab in key):    # the base standing on its column
                    continue
                self.violate("arm_or_palm_contact", {"pair": list(key), "cats": list(cats),
                                                     "force_N": round(self.contacts.step_pairs.get(key, 0.), 3)},
                             sig="|".join(key))
        if self.carried is not None:
            oid = self.carried["oid"]
            for key, act in self.contacts.pairs_with(oid).items():
                if act["last"] != tick:
                    continue
                other = key[1] if key[0] == oid else key[0]
                cat = act["cats"][1] if key[0] == oid else act["cats"][0]
                if cat in CARRIED_ALLOWED:
                    continue
                if other in self.carried["partners"]:
                    continue
                self.violate("carried_contact", {"dish": oid, "partner": other, "category": cat,
                                                 "allowed": list(CARRIED_ALLOWED) + sorted(self.carried["partners"]),
                                                 "force_N": round(self.contacts.step_pairs.get(key, 0.), 3),
                                                 "colliders": sorted(act["colliders"])[:6]},
                             sig=other)
            # an at-close partner stops being allowed once the pair has broken
            self.carried["partners"] = {p for p in self.carried["partners"] if p in self.contacts.partners(oid)}
        if tick % SLEEP_CHECK_EVERY == 0:
            asleep = []
            for name, pid in self.sleep_bodies:
                try:
                    if self.sim_iface.is_sleeping(self.stage_id, pid):
                        asleep.append(name)
                except Exception as exc:          # noqa: BLE001
                    if name not in self.sleep_unsupported:
                        self.sleep_unsupported.append(name)
                        self.tlog.write("note", what="is_sleeping unsupported", body=name, error=repr(exc))
            if asleep:
                self.violate("sleeping_body", {"bodies": asleep}, sig=",".join(asleep[:3]))

    def _sample(self, frames):
        r = self.robot
        q = r.data.joint_pos[0].detach().cpu().numpy()
        qt = r.data.joint_pos_target[0].detach().cpu().numpy() if getattr(r.data, "joint_pos_target", None) is not None else q
        row = {"t_trial_s": round(self.trial_time(), 4),
               "q": [round(float(v), 5) for v in q[self.arm]], "q_cmd": [round(float(v), 5) for v in qt[self.arm]],
               "lag": [round(float(v), 5) for v in (q[self.arm] - qt[self.arm])],
               "fingers": {self.names[i]: round(float(q[i]), 5) for i in self.gripper_joints},
               "pad_N": {k: round(v, 3) for k, v in self.contacts.pad_N.items()},
               "pad_F": {k: round(v, 3) for k, v in self.contacts.pad_F.items()},
               "pad_N_max": {k: round(v, 3) for k, v in self.pad_max_interval.items()},
               "carried": None if self.carried is None else self.carried["oid"],
               "dishes": {oid: [round(v, 5) for v in (*frames[oid]["position_m"], *frames[oid]["quaternion_xyzw"])]
                          for oid in self.ids if oid in frames},
               "racks": {k: round(v, 5) for k, v in self.backend.joints().items()},
               "n_contacts": len(self.contacts.active)}
        self.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        self.tlog.write("sample", **row)

    # ------------------------------------------------------------------ settle routines
    def dish_speeds(self):
        out = {}
        for oid in self.ids:
            v = self.backend.objects[oid].data.root_vel_w[0].detach().cpu().numpy()
            out[oid] = (float(np.linalg.norm(v[:3])), float(np.linalg.norm(v[3:])))
        return out

    def diag_settle(self, step):
        """The plan's settle routine (Phase 0.3): step until every dish is under 1 mm/s and 0.01 rad/s for 1 s of
        simulated time, capped at 5 s. D3: logged as a diagnostic column, never a gate. ``step(n)`` advances physics
        (the rig, so the arm keeps its targets)."""
        need = round(DIAG_SETTLE["hold_s"] / self.dt)
        cap = round(DIAG_SETTLE["cap_s"] / self.dt)
        quiet, worst = 0, (0., 0., None)
        for n in range(1, cap + 1):
            step(1)
            sp = self.dish_speeds()
            v = max(sp.values(), key=lambda t: t[0])[0]
            w = max(sp.values(), key=lambda t: t[1])[1]
            loud = [oid for oid, (a, b) in sp.items() if a >= DIAG_SETTLE["v_max_m_s"] or b >= DIAG_SETTLE["w_max_rad_s"]]
            quiet = 0 if loud else quiet + 1
            worst = (max(worst[0], v), max(worst[1], w), loud[:3] if loud else worst[2])
            if quiet >= need:
                return {"settled": True, "sim_s": round(n * self.dt, 4), "rule": dict(DIAG_SETTLE),
                        "peak_v_m_s": round(worst[0], 5), "peak_w_rad_s": round(worst[1], 5)}
        return {"settled": False, "sim_s": round(cap * self.dt, 4), "rule": dict(DIAG_SETTLE),
                "peak_v_m_s": round(worst[0], 5), "peak_w_rad_s": round(worst[1], 5), "still_moving": worst[2]}


class Media:
    """Fixed-camera video (15 fps of simulated time) with the simulated time overlaid, and key-frame PNGs."""

    def __init__(self, *, backend, cams, cam_name, trial_dir, overlay_text, fps=15):
        from dishsim.media import VideoWriter
        self.backend, self.cams, self.cam = backend, cams, cam_name
        self.dir = Path(trial_dir)
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self.overlay_text = overlay_text          # () -> list of lines
        self.video = VideoWriter(str(self.dir / "video.mp4"), fps=fps)
        self.n_key = 0
        self.font = self._font()
        self.frames = 0

    @staticmethod
    def _font():
        from PIL import ImageFont
        import matplotlib
        path = Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
        try:
            return ImageFont.truetype(str(path), 22)
        except Exception:                       # noqa: BLE001
            return ImageFont.load_default()

    def _grab(self, renders=1):
        for _ in range(renders):
            self.backend.sim.render()
        self.cams.update(self.backend.sim.get_physics_dt())
        return self.cams.grab_one(self.cam)

    def overlay(self, img, extra=None):
        from PIL import Image, ImageDraw
        im = Image.fromarray(img)
        draw = ImageDraw.Draw(im, "RGBA")
        lines = list(self.overlay_text()) + ([extra] if extra else [])
        pad, lh = 10, 28
        w = max(draw.textlength(s, font=self.font) for s in lines) + 2 * pad
        draw.rectangle([0, 0, w, pad * 2 + lh * len(lines)], fill=(0, 0, 0, 150))
        for i, s in enumerate(lines):
            draw.text((pad, pad + i * lh), s, font=self.font, fill=(255, 255, 255, 255))
        return np.asarray(im)

    def video_frame(self):
        self.video.add(self.overlay(self._grab(1)))
        self.frames += 1

    def key_frame(self, tag, label=None, renders=4):
        from PIL import Image
        self.n_key += 1
        name = f"{self.n_key:03d}_{tag.replace(' ', '_').replace('/', '_')}.png"
        Image.fromarray(self.overlay(self._grab(renders), extra=label or tag)).save(self.dir / "frames" / name)
        return f"frames/{name}"

    def close(self):
        if self.video is not None:
            self.video.close()
            self.video = None
