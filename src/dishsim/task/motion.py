# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The motion layer: "move from configuration A to configuration B", and nothing else.

This module is the whole of what the task layer is allowed to ask the robot to do. It owns the
collision world, the IK, and path execution; it does not know what an object is. Everything it
is told about one arrives as raw geometry — a list of convex pieces plus a transform — filed
under a key the caller chose and this module never interprets. There is no object registry
import here, no placement mode, no notion of an order. ``tests/test_layer_boundary.py`` asserts
that mechanically, because the property is invisible at runtime and one convenient import would
end it.

Below this sits :mod:`dishsim.planners`, entirely untouched: :meth:`MotionService.plan` forwards
``(world, start_q, goal_qs, seed, debug)`` and gets a ``PlanResult`` back. Swapping RRT-Connect
for PRM changes nothing here and nothing above.

Isaac itself is reached through :class:`ExecContext`, a protocol the runner implements once Kit
is booted. That keeps this module importable — and therefore testable — in the plain venv, which
is how the sequencer tests run a whole episode with no simulator at all.
"""

from dataclasses import dataclass, field
from typing import Protocol, Sequence

import numpy as np

from .. import config
from ..planners.base import PlanDebug, PlanResult
from ..transforms import T_inv
from ..ur5e_kin import ik_wrist3_all


class ExecContext(Protocol):
    """Everything the motion layer needs from a live simulator, and no more.

    The runner implements this after ``AppLauncher`` has booted; this module only ever holds the
    protocol, which is why it imports no Isaac module. All poses are robot-base frame, all
    angles radians.
    """

    def step(self, n: int = 1) -> None:
        """Advance physics ``n`` steps, sampling any active trajectory recorder."""

    def hold(self, arm_q: np.ndarray | None = None, aperture: float | None = None) -> None:
        """Write drive targets for this step (arm pose and/or gripper aperture)."""

    def arm_q(self) -> np.ndarray:
        """Measured arm configuration [rad], shape [6]."""

    def contact_peak(self, exclude: Sequence[str] = ()) -> tuple[float, str]:
        """Largest unexpected contact force on any robot body [N] and the body's name."""

    def ramp_aperture(self, aperture_rad: float, steps: int, arm_q: np.ndarray | None = None,
                      on_step=None) -> float:
        """Ramp the gripper to an aperture over ``steps`` physics steps; returns the final one."""

    def set_phase(self, name: str) -> None:
        """Tag subsequent recorded steps with a phase name."""

    def set_rack_target(self, targets: dict | None) -> None:
        """Override the machine's rack drive targets, or ``None`` to restore the scene default.

        A rack is an actuator of the scene, like the gripper aperture — not object or task
        knowledge — so driving it belongs here. ``targets`` maps prismatic joint name to
        extension [m].
        """

    def set_payload(self, key: str | None) -> None:
        """Note which payload handle is currently carried, or ``None``.

        The handle stays opaque to this layer — it is forwarded so the context can subtract the
        expected grip reaction from its contact readings. Without it, the gripper's own hold on
        the object it is carrying reads as an unexpected collision the moment the transfer
        starts.
        """


@dataclass
class ExecResult:
    """Outcome of executing one motion segment.

    Contact is reported rather than raised: the sequencer decides whether a graze is fatal, and
    for a multi-object scene that decision depends on what was touched.

    Attributes:
        ok: True when the segment ran to completion within the contact gate.
        detail: Human-readable cause when ``ok`` is False, else ``None``.
        peak_contact_n: Largest unexpected contact force observed [N].
        n_steps: Physics steps consumed.
        q_end: Final arm configuration [rad], shape [6], or ``None`` if nothing ran.
    """

    ok: bool
    detail: str | None = None
    peak_contact_n: float = 0.0
    n_steps: int = 0
    q_end: np.ndarray | None = None


@dataclass
class MotionStats:
    """Counters worth carrying into the episode record."""

    n_plans: int = 0
    n_plan_failures: int = 0
    plan_time_s: float = 0.0
    n_exec_steps: int = 0
    per_phase_steps: dict = field(default_factory=dict)


class MotionService:
    """Object-agnostic motion execution over a pluggable planner and a mutable collision world.

    Args:
        planner: Any :class:`~dishsim.planners.base.Planner`; used only through ``plan()``.
        world: Collision world used for validity queries and planning.
        ctx: Live-simulator access (see :class:`ExecContext`).
        contact_limit_n: Unexpected-contact force above which a segment is reported failed [N].
    """

    def __init__(self, planner, world, ctx: ExecContext, *, contact_limit_n: float | None = None) -> None:
        self.planner = planner
        self.world = world
        self.ctx = ctx
        self.contact_limit_n = float(
            config.RETRACT_GRAZE_MAX_N if contact_limit_n is None else contact_limit_n
        )
        self.stats = MotionStats()
        self._T_w3_tcp = np.array(world.manifest["t_wrist3_tcp"])
        self._payload_key: str | None = None

    # ---- planning ----------------------------------------------------------------------------

    def plan(self, start_q, goal_qs, *, seed: int | None = None,
             debug: PlanDebug | None = None) -> PlanResult:
        """Plan a collision-free path from one configuration to any of a goal set.

        A direct forward to the planner layer — the world and seed are per-call, exactly as
        :class:`~dishsim.planners.base.Planner` requires, so successive picks may plan against
        successively different worlds with one planner instance.
        """
        goal_qs = np.atleast_2d(np.asarray(goal_qs, dtype=float))
        res = self.planner.plan(self.world, np.asarray(start_q, dtype=float), goal_qs,
                                seed=seed, debug=debug)
        self.stats.n_plans += 1
        self.stats.plan_time_s += float(res.plan_time_s)
        if res.status != "solved":
            self.stats.n_plan_failures += 1
        return res

    def reachable(self, T_base_tcp: np.ndarray, *, q_seed=None, check_collision: bool = True) -> np.ndarray:
        """Collision-free arm configurations achieving a TCP pose.

        The repository's de-facto reachability test, which was previously copy-pasted at three
        call sites: analytic IK, then the world's validity filter. An empty result is the answer
        "not reachable here" — it is signal, not an error.

        Args:
            T_base_tcp: Target TCP pose in the robot-base frame, shape [4, 4].
            q_seed: Optional seed configuration for branch selection, shape [6].
            check_collision: Apply the world's validity filter (set False for pure kinematics).

        Returns:
            Configurations [rad], shape [K, 6]; ``K`` may be 0.
        """
        seed = np.array(config.HOME_Q) if q_seed is None else np.asarray(q_seed, dtype=float)
        sols = ik_wrist3_all(np.asarray(T_base_tcp) @ T_inv(self._T_w3_tcp), q_seed=seed)
        if not check_collision:
            return np.asarray(sols)
        keep = [q for q in sols if not self.world.in_collision(q)]
        return np.array(keep) if keep else np.empty((0, 6))

    def line_configs(self, T_from: np.ndarray, T_to: np.ndarray, n: int, q_seed) -> np.ndarray | None:
        """Branch-continuous IK along a straight TCP translation (rotation held).

        Used for scripted descend/lift/retreat moves, where a planner is the wrong tool: the
        path is prescribed and what matters is that consecutive configurations stay on the same
        IK branch. Returns ``None`` when no branch stays continuous, which is a real failure
        mode and not the same as a collision.

        Args:
            T_from: Start TCP pose, shape [4, 4].
            T_to: End TCP pose, shape [4, 4]; only its position is used.
            n: Number of waypoints.
            q_seed: Configuration to stay continuous with, shape [6].

        Returns:
            Waypoints [rad], shape [n, 6], or ``None``.
        """
        qs, q_prev = [], np.asarray(q_seed, dtype=float)
        T_tcp_w3 = T_inv(self._T_w3_tcp)
        for s in np.linspace(0.0, 1.0, n):
            T = np.array(T_from, dtype=float)
            T[:3, 3] = (1 - s) * np.asarray(T_from)[:3, 3] + s * np.asarray(T_to)[:3, 3]
            sols = []
            for sol in ik_wrist3_all(T @ T_tcp_w3, q_seed=q_prev):
                sols.append(sol + np.round((q_prev - sol) / (2.0 * np.pi)) * 2.0 * np.pi)
            sols.sort(key=lambda q: float(np.abs(q - q_prev).max()))
            if not sols or float(np.abs(sols[0] - q_prev).max()) > 0.8:
                return None
            qs.append(sols[0])
            q_prev = sols[0]
        return np.array(qs)

    # ---- execution ---------------------------------------------------------------------------

    def execute(self, path_q, *, phase: str, aperture: float | None = None,
                exclude: Sequence[str] = (), on_step=None, gate=None,
                on_before_step=None) -> ExecResult:
        """Track a joint path under time parameterization, gating on unexpected contact.

        Args:
            path_q: Waypoints [rad], shape [N, 6].
            phase: Phase tag recorded against every step of this segment.
            aperture: Gripper aperture to hold [rad]; ``None`` leaves it untouched.
            exclude: Robot bodies whose contact is expected and must not count.
            on_step: Optional per-step callback (media capture, say).
            gate: Optional per-step veto returning a reason string to abort, else ``None``.
                This is how a caller adds checks this layer cannot make itself — whether the
                CARRIED object is still held, for instance, which is object knowledge and so
                belongs above. Robot-body contact is checked here regardless.
            on_before_step: Optional callback taking the segment progress fraction in ``[0, 1]``,
                invoked BEFORE each step's drive write. It exists for actuation that must ramp in
                lockstep with the arm — a rack sliding while the tool tracks its handle. The
                fraction, not a waypoint index, is what is passed: the path is time-parameterized
                here, so it is resampled and the caller's own indices would not line up.

        Returns:
            An :class:`ExecResult`; ``ok`` is False on the first contact over the limit or the
            first step the gate rejects.
        """
        from .. import trajectory as dtraj  # noqa: PLC0415  (Kit-free, but keeps imports lean)

        self.ctx.set_phase(phase)
        peak_seen, n = 0.0, 0
        q_last = None
        traj = list(dtraj.time_parameterize(np.asarray(path_q, dtype=float)))
        last = max(len(traj) - 1, 1)
        for i, wp in enumerate(traj):
            if on_before_step is not None:
                on_before_step(i / last)
            self.ctx.hold(arm_q=wp, aperture=aperture)
            self.ctx.step(1)
            n += 1
            q_last = wp
            if on_step is not None:
                on_step(n)
            peak, body = self.ctx.contact_peak(exclude=exclude)
            peak_seen = max(peak_seen, float(peak))
            if peak > self.contact_limit_n:
                self._tally(phase, n)
                return ExecResult(False, f"{phase}: {body} ({peak:.1f} N)", peak_seen, n, q_last)
            if gate is not None:
                reason = gate()
                if reason:
                    self._tally(phase, n)
                    return ExecResult(False, f"{phase}: {reason}", peak_seen, n, q_last)
        self._tally(phase, n)
        return ExecResult(True, None, peak_seen, n, q_last)

    def servo_line(self, T_from, T_to, n: int, q_seed, *, phase: str,
                   aperture: float | None = None, exclude: Sequence[str] = (),
                   on_step=None) -> ExecResult:
        """:meth:`line_configs` followed by :meth:`execute` — the scripted straight-line move."""
        seg = self.line_configs(T_from, T_to, n, q_seed)
        if seg is None:
            return ExecResult(False, f"{phase}: IK discontinuity along the straight segment")
        return self.execute(seg, phase=phase, aperture=aperture, exclude=exclude, on_step=on_step)

    def hold(self, steps: int, *, phase: str, arm_q=None, aperture: float | None = None,
             on_step=None) -> ExecResult:
        """Hold a pose for ``steps`` physics steps (settling, force stabilisation)."""
        self.ctx.set_phase(phase)
        peak_seen = 0.0
        for i in range(steps):
            self.ctx.hold(arm_q=arm_q, aperture=aperture)
            self.ctx.step(1)
            if on_step is not None:
                on_step(i + 1)
            peak, _ = self.ctx.contact_peak()
            peak_seen = max(peak_seen, float(peak))
        self._tally(phase, steps)
        return ExecResult(True, None, peak_seen, steps, self.ctx.arm_q())

    def set_aperture(self, aperture_rad: float, steps: int, *, phase: str, arm_q=None,
                     on_step=None) -> float:
        """Ramp the gripper to an aperture. Returns the achieved aperture [rad].

        ``on_step`` matters more here than it looks: the close and the release are the two
        moments the choreography exists to show on camera, and without a capture hook they are
        the only phases absent from the video.
        """
        self.ctx.set_phase(phase)
        out = self.ctx.ramp_aperture(aperture_rad, steps, arm_q=arm_q, on_step=on_step)
        self._tally(phase, steps)
        return out

    def current_q(self) -> np.ndarray:
        """Measured arm configuration [rad], shape [6]."""
        return self.ctx.arm_q()

    def set_rack_target(self, targets: dict | None) -> None:
        """Drive the machine's rack joints, or restore the scene default with ``None``.

        Args:
            targets: Prismatic joint name -> extension [m].
        """
        self.ctx.set_rack_target(targets)

    def tcp_pose(self, q=None) -> np.ndarray:
        """TCP pose in the robot-base frame for a configuration (default: the measured one)."""
        from ..ur5e_kin import fk_wrist3  # noqa: PLC0415

        q = self.ctx.arm_q() if q is None else np.asarray(q, dtype=float)
        return fk_wrist3(q) @ self._T_w3_tcp

    def _tally(self, phase: str, n: int) -> None:
        self.stats.n_exec_steps += n
        self.stats.per_phase_steps[phase] = self.stats.per_phase_steps.get(phase, 0) + n

    # ---- world state: geometry in, no identity -----------------------------------------------

    def set_obstacle(self, key: str, pieces, T_base_obj: np.ndarray) -> None:
        """Register (or re-pose) a free-standing obstacle under a caller-chosen key.

        ``key`` is opaque: the caller may use item ids, this layer only uses it as a handle.
        Re-registering an existing key moves it, which is what re-reading world state after a
        pick amounts to.
        """
        if self.world.has_object(key):
            self.world.set_object_pose(key, np.asarray(T_base_obj))
        else:
            self.world.add_object(key, pieces, np.asarray(T_base_obj))

    def clear_obstacle(self, key: str) -> None:
        """Drop an obstacle if present; a no-op when the key is unknown."""
        if self.world.has_object(key):
            self.world.remove_object(key)

    def attach_payload(self, key: str, pieces, T_wrist3_obj: np.ndarray) -> None:
        """Make a payload ride the wrist for planning purposes.

        The carried geometry has to move with the gripper or every plan while carrying is
        computed against a stale world. Any previously attached payload is detached first, so a
        pick-place-pick loop cannot silently accumulate two.
        """
        self.detach_payload()
        self.world.attach_object(pieces, np.asarray(T_wrist3_obj))
        self._payload_key = key
        setter = getattr(self.ctx, "set_payload", None)
        if setter is not None:
            setter(key)

    def detach_payload(self) -> None:
        """Drop the carried payload from the moving cluster; a no-op when nothing is carried."""
        if self._payload_key is not None:
            self.world.detach_carried_object()
            self._payload_key = None
            setter = getattr(self.ctx, "set_payload", None)
            if setter is not None:
                setter(None)

    @property
    def payload(self) -> str | None:
        """Key of the currently attached payload, or ``None``."""
        return self._payload_key
