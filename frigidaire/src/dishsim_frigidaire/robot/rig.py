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


def T_of(pos, quat_xyzw):
    x, y, z, w = quat_xyzw
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = pos
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
        self.target = robot.data.default_joint_pos.clone()
        self.on_step = on_step or (lambda: None)
        self.physics_step = physics_step or (lambda: sim.step())   # e.g. the Frigidaire backend's tick()
        self.grip_close, self.theta = grip_close, 0.
        self.lag_max, self.settled_lag = float(lag_max), float(settled_lag)
        self.update_robot = update_robot
        self.on_teleport = on_teleport
        self.log = []

    # ------------------------------------------------------------------ basics
    def q(self):
        return self.robot.data.joint_pos[0, self.arm].cpu().numpy().astype(float)

    def set_arm(self, q):
        self.target[0, self.arm] = self.torch.tensor(np.asarray(q, dtype=float), dtype=self.target.dtype)

    def step(self, n=1):
        for _ in range(n):
            ur5e.set_gripper(self.target, self.names, self.theta)
            self.robot.set_joint_position_target(self.target)
            self.robot.write_data_to_sim()
            self.physics_step()
            if self.update_robot:
                self.robot.update(self.sim.get_physics_dt())
            self.on_step()

    def teleport_arm(self, q):
        if self.on_teleport is not None:
            self.on_teleport("teleport_arm")
        self.set_arm(q)
        self.robot.write_joint_state_to_sim(self.target, self.torch.zeros_like(self.target))

    def tcp(self):
        p = self.robot.data.body_link_pos_w[0, self.w3].cpu().numpy()
        qw = self.robot.data.body_link_quat_w[0, self.w3].cpu().numpy()
        return T_of(p, (qw[1], qw[2], qw[3], qw[0])) @ self.T_w3_tcp

    def lag(self):
        return float(np.abs(self.q() - self.target[0, self.arm].cpu().numpy()).max())

    # ------------------------------------------------------------------ IK
    def ik(self, T_tcp_world, seed):
        sols = kin.ik_wrist3_all(self.T_bw @ np.asarray(T_tcp_world) @ np.linalg.inv(self.T_w3_tcp), q_seed=np.asarray(seed))
        if len(sols) == 0:
            return None
        sols = [s + np.round((np.asarray(seed) - s) / (2 * np.pi)) * 2 * np.pi for s in sols]
        ok = [s for s in sols if self.within_limits(s) and self.branch_ok(s)]
        if not ok:
            return None
        return min(ok, key=lambda s: float(np.abs(s - np.asarray(seed)).max()))

    def ik_all(self, T_tcp_world, seed):
        """Every elbow-up IK solution (2 pi-unwrapped toward ``seed``), nearest first."""
        sols = kin.ik_wrist3_all(self.T_bw @ np.asarray(T_tcp_world) @ np.linalg.inv(self.T_w3_tcp), q_seed=np.asarray(seed))
        sols = [s + np.round((np.asarray(seed) - s) / (2 * np.pi)) * 2 * np.pi for s in sols]
        ok = [s for s in sols if self.within_limits(s) and self.branch_ok(s)]
        return sorted(ok, key=lambda s: float(np.abs(s - np.asarray(seed)).max()))

    @staticmethod
    def within_limits(q, margin=.02):
        return bool(np.all(q >= kin.JOINT_LIMITS[:, 0] + margin) and np.all(q <= kin.JOINT_LIMITS[:, 1] - margin))

    ELBOW_UP_ONLY = False    # with the FCL arm model + RRT-Connect every branch is usable; True = the old heuristic

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
        vel = self.robot.data.default_joint_vel.clone()
        prev = self.q()
        for i, q in enumerate(qs):
            v = (np.asarray(q, dtype=float) - prev) / dt
            vel[0, self.arm] = self.torch.tensor(v, dtype=vel.dtype)
            self.robot.set_joint_velocity_target(vel)
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
        self.robot.set_joint_velocity_target(vel)

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

    def gripper(self, closed):
        a, b = self.theta, (self.grip_close if closed else 0.)
        for s in np.linspace(a, b, CLOSE_STEPS if closed else OPEN_STEPS):
            self.theta = float(s)
            self.step(1)
        self.step(30)
        return float(self.robot.data.joint_pos[0, self.fj])
