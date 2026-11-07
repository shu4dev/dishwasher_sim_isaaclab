"""Kit-side UR5e + 2F-85 controller: joint moves, straight TCP lines (analytic IK), gripper, pick and place.

Created AFTER Kit boots (imports torch / Isaac Lab objects passed in). Position control: every physics step
writes the arm joint targets of the current waypoint and the gripper targets (inner fingers mimic-consistent).
A move aborts (returns False) when the arm lags its target by more than ``LAG_MAX_RAD`` (a collision stops it)
or IK loses branch continuity. No weld, no attachment: a dish is held only by the pads.
"""
from __future__ import annotations

import numpy as np

from . import kin, ur5e

LAG_MAX_RAD = .15            # measured-vs-commanded arm lag that counts as blocked (with velocity feed-forward)
SETTLED_LAG_RAD = .03        # lag left 20 steps after the last waypoint: something holds the arm
IK_JUMP_MAX_RAD = .6         # joint jump between consecutive line waypoints (branch continuity)
CLOSE_STEPS, OPEN_STEPS = 90, 60
# plans/2026-10-27-posture-fix.md: IK solutions come wrapped to (-180, 180] deg; a joint is unwrapped toward the seed only
# while it stays inside +-190 deg (the elbow +-180), so no joint winds past a turn (round 6 wound wrist 1 to -353 deg)
POSTURE_LIMITS = np.radians([[-190., 190.], [-190., 190.], [-180., 180.], [-190., 190.], [-190., 190.], [-190., 190.]])
LIMIT_MARGIN_RAD = .02


def unwind(s, seed, margin=LIMIT_MARGIN_RAD):
    """A wrapped IK solution moved by 2 pi toward ``seed`` per joint, only where that stays inside POSTURE_LIMITS."""
    s, seed = np.asarray(s, dtype=float), np.asarray(seed, dtype=float)
    u = s + np.round((seed - s) / (2 * np.pi)) * 2 * np.pi
    ok = (u >= POSTURE_LIMITS[:, 0] + margin) & (u <= POSTURE_LIMITS[:, 1] - margin)
    return np.where(ok, u, s)


def T_of(pos, quat_xyzw):
    """Pose matrix; the quaternion is normalised first (plans/2026-10-27-bowls-lip-plates-fix.md: load.json goal quaternions
    sit 1.6-1.9e-7 off unit length, and kin.ik_wrist3_all's 1e-6 FK check then dropped every IK branch of the target)."""
    q = np.asarray(quat_xyzw, dtype=float)
    x, y, z, w = q / np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = pos
    return T


def orthonormal(T):
    """``T`` with its rotation block replaced by the nearest rotation (SVD): an IK target built from measured or float32
    frames is off orthonormal by ~1e-7, enough for kin.ik_wrist3_all's fk_tol 1e-6 to reject every branch."""
    T = np.array(T, dtype=float)
    U, _, Vt = np.linalg.svd(T[:3, :3])
    T[:3, :3] = U @ Vt
    return T


class Rig:
    def __init__(self, sim, robot, T_world_base, on_step=None, grip_close=.8, physics_step=None,
                 lag_max=LAG_MAX_RAD, settled_lag=SETTLED_LAG_RAD, update_robot=True, on_teleport=None):
        """``lag_max`` / ``settled_lag``: the blocked-motion limits (run flags R7/D8; the defaults are the pre-R7
        values). ``update_robot=False`` when ``physics_step`` already refreshes the articulation's buffers (the
        episode's harness tick). ``on_teleport(what)`` is called before every state write (the teleport guard)."""
        import torch
        self.torch, self.sim, self.robot = torch, sim, robot
        self.T_wb = np.asarray(T_world_base, dtype=float)
        self.T_bw = np.linalg.inv(self.T_wb)
        self.T_w3_tcp = T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
        self.names, self.bodies = list(robot.joint_names), list(robot.body_names)
        self.arm = [self.names.index(j) for j in ur5e.ARM_JOINTS]
        self.fj = self.names.index("finger_joint")
        self.w3 = self.bodies.index("wrist_3_link")
        self.target = robot.data.default_joint_pos.torch.clone()
        self.on_step = on_step or (lambda: None)
        self.physics_step = physics_step or (lambda: sim.step())   # e.g. the Frigidaire backend's tick()
        self.grip_close, self.theta = grip_close, 0.
        self.lag_max, self.settled_lag = float(lag_max), float(settled_lag)
        self.update_robot = update_robot
        self.on_teleport = on_teleport
        self.log = []

    # ------------------------------------------------------------------ basics
    def q(self):
        return self.robot.data.joint_pos.torch[0, self.arm].cpu().numpy().astype(float)

    def set_arm(self, q):
        self.target[0, self.arm] = self.torch.tensor(np.asarray(q, dtype=float), dtype=self.target.dtype)

    def step(self, n=1):
        for _ in range(n):
            ur5e.set_gripper(self.target, self.names, self.theta)
            self.robot.actuators.target_command.set_position_index(value=self.target)
            self.robot.write_data_to_sim()
            self.physics_step()
            if self.update_robot:
                self.robot.update(self.sim.get_physics_dt())
            self.on_step()

    def teleport_arm(self, q):
        if self.on_teleport is not None:
            self.on_teleport("teleport_arm")
        self.set_arm(q)
        self.robot.write_joint_state_to_sim_index(position=self.target, velocity=self.torch.zeros_like(self.target))

    def tcp(self):
        p = self.robot.data.body_link_pos_w.torch[0, self.w3].cpu().numpy()
        q = self.robot.data.body_link_quat_w.torch[0, self.w3].cpu().numpy()   # XYZW (Isaac Lab 3.0)
        return T_of(p, q) @ self.T_w3_tcp

    def lag(self):
        return float(np.abs(self.q() - self.target[0, self.arm].cpu().numpy()).max())

    # ------------------------------------------------------------------ IK
    def ik(self, T_tcp_world, seed):
        sols = kin.ik_wrist3_all(orthonormal(self.T_bw @ np.asarray(T_tcp_world) @ np.linalg.inv(self.T_w3_tcp)),
                                 q_seed=np.asarray(seed))
        if len(sols) == 0:
            return None
        sols = [unwind(s, seed) for s in sols]
        ok = [s for s in sols if self.within_limits(s) and self.branch_ok(s)]
        if not ok:
            return None
        return min(ok, key=lambda s: float(np.abs(s - np.asarray(seed)).max()))

    def ik_all(self, T_tcp_world, seed):
        """Every elbow-up IK solution (unwound toward ``seed`` inside POSTURE_LIMITS), nearest first."""
        sols = kin.ik_wrist3_all(orthonormal(self.T_bw @ np.asarray(T_tcp_world) @ np.linalg.inv(self.T_w3_tcp)),
                                 q_seed=np.asarray(seed))
        sols = [unwind(s, seed) for s in sols]
        ok = [s for s in sols if self.within_limits(s) and self.branch_ok(s)]
        return sorted(ok, key=lambda s: float(np.abs(s - np.asarray(seed)).max()))

    @staticmethod
    def within_limits(q, margin=LIMIT_MARGIN_RAD):
        """Inside POSTURE_LIMITS (+-190 deg, elbow +-180; the hardware's kin.JOINT_LIMITS are +-360, elbow +-180)."""
        return bool(np.all(q >= POSTURE_LIMITS[:, 0] + margin) and np.all(q <= POSTURE_LIMITS[:, 1] - margin))

    ELBOW_UP_ONLY = True     # plans/2026-10-27-posture-fix.md (was False since 2026-10-23: every branch with RRT-Connect)

    @classmethod
    def branch_ok(cls, q):
        """Elbow-up family (as HOME_Q): shoulder lift in (-pi, 0), elbow in (0, pi). The nearest-solution rule alone
        drifted into an elbow-down branch that folded the forearm into the lower rack (easy_s0, 2026-09-29)."""
        if not cls.ELBOW_UP_ONLY:
            return True
        lift = (q[1] + np.pi) % (2 * np.pi) - np.pi
        elbow = (q[2] + np.pi) % (2 * np.pi) - np.pi
        return -np.pi < lift < 0. and 0. < elbow < np.pi

    def line_q(self, T_from, T_to, n, seed):
        """Joint waypoints along a straight TCP segment (rotation interpolated by slerp-free linear blend of
        the two frames, re-orthonormalised); None if the IK branch jumps."""
        qs, prev = [], np.asarray(seed, dtype=float)
        R0, R1 = np.asarray(T_from)[:3, :3], np.asarray(T_to)[:3, :3]
        u = np.linspace(0., 1., n)
        for s in u * u * (3. - 2. * u):                       # smoothstep: zero speed at both ends
            T = np.eye(4)
            U, _, Vt = np.linalg.svd((1 - s) * R0 + s * R1)
            T[:3, :3] = U @ Vt
            T[:3, 3] = (1 - s) * np.asarray(T_from)[:3, 3] + s * np.asarray(T_to)[:3, 3]
            q = self.ik(T, prev)
            if q is None or float(np.abs(q - prev).max()) > IK_JUMP_MAX_RAD:
                return None
            qs.append(q)
            prev = q
        return qs

    # ------------------------------------------------------------------ motions
    def follow(self, qs, steps_per=2, settle_tol=None):
        """Track joint waypoints with velocity feed-forward (the implicit PD drive otherwise lags by
        damping * velocity / stiffness: 0.16 rad on the wrists at 1.2 rad/s, measured 2026-09-29)."""
        dt = self.sim.get_physics_dt() * steps_per
        vel = self.robot.data.default_joint_vel.torch.clone()
        prev = self.q()
        for i, q in enumerate(qs):
            v = (np.asarray(q, dtype=float) - prev) / dt
            vel[0, self.arm] = self.torch.tensor(v, dtype=vel.dtype)
            self.robot.actuators.target_command.set_velocity_index(value=vel)
            self.set_arm(q)
            self.step(steps_per)
            prev = np.asarray(q, dtype=float)
            if self.lag() > self.lag_max:
                self._stop_vel(vel)
                self._fail(i, len(qs))
                return False
        self._stop_vel(vel)
        self.step(20)
        if self.lag() > (self.settled_lag if settle_tol is None else settle_tol):
            self._fail(len(qs), len(qs))
            return False
        return True

    def _stop_vel(self, vel):
        vel[0, self.arm] = 0.
        self.robot.actuators.target_command.set_velocity_index(value=vel)

    def _fail(self, i, n):
        lag = self.q() - self.target[0, self.arm].cpu().numpy()
        self.last_fail = {"waypoint": [i, n], "lag_rad": [round(float(v), 3) for v in lag],
                          "q": [round(float(v), 3) for v in self.q()], "tcp": [round(float(v), 3) for v in self.tcp()[:3, 3]]}

    def move_joint(self, q_goal, max_step=.008, settle_tol=None):   # 0.008 rad per 2 steps at 120 Hz = 0.5 rad/s
        q0 = self.q()
        n = max(2, int(np.ceil(1.5 * float(np.abs(np.asarray(q_goal) - q0).max()) / max_step)))
        u = np.linspace(0., 1., n)
        return self.follow([q0 + (np.asarray(q_goal) - q0) * s for s in u * u * (3. - 2. * u)], settle_tol=settle_tol)

    def move_line(self, T_to, n=60):
        qs = self.line_q(self.tcp(), T_to, n, self.q())
        return qs is not None and self.follow(qs)

    def set_grip_drive(self, stiffness, damping):
        """finger_joint's PhysX drive gains at runtime (plans/2026-11-04-fifteen-of-fifteen.md, flags R2 per hold): Isaac Lab
        3.0's implicit-actuator gains are articulation-owned, so the whole joint_stiffness / joint_damping rows are read, only
        finger_joint's entry changed, and written back (write_joint_stiffness_to_sim_index / write_joint_damping_to_sim_index,
        as dishsim_frigidaire.asset.apply_mode does) only where the value differs; the effort limit stays the spawn's
        (ur5e.robot_cfg). Returns {before, after: [stiffness, damping] read back, written: the gains written}."""
        def read():
            return [float(self.robot.data.joint_stiffness.torch[0, self.fj]), float(self.robot.data.joint_damping.torch[0, self.fj])]
        before, written = read(), []
        for name, val, cur in (("stiffness", stiffness, before[0]), ("damping", damping, before[1])):
            if abs(cur - float(val)) > 1e-6 * max(1., abs(float(val))):
                row = getattr(self.robot.data, f"joint_{name}").torch.clone()
                row[0, self.fj] = float(val)
                getattr(self.robot, f"write_joint_{name}_to_sim_index")(**{name: row})
                written.append(name)
        return {"before": before, "after": read(), "written": written}

    def gripper(self, closed, to=None):
        """Ramp finger_joint's target to the close (``grip_close``) or the open, then 30 steps; returns finger_joint's measured
        position. ``to`` (an open only; plans/2026-11-04-fifteen-of-fifteen.md, the partial open): the open's finger_joint target
        (rad, 0 = the full open; None = the full open)."""
        a, b = self.theta, (self.grip_close if closed else (0. if to is None else float(to)))
        for s in np.linspace(a, b, CLOSE_STEPS if closed else OPEN_STEPS):
            self.theta = float(s)
            self.step(1)
        self.step(30)
        return float(self.robot.data.joint_pos.torch[0, self.fj])
