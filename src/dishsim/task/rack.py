# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack manipulation as a task-layer primitive: engage a handle, slide the rack, let go.

An episode that starts from ``both_in`` cannot place anything — the machine's slots have no
reachable goal configurations while the rack is stowed (0 of 15, measured). The rack has to come
out first, and that is a *machine* operation, not an object one: nothing here knows what is being
loaded, only which joint moves and how far. It therefore sits beside :class:`PickPlace` in the
task layer and executes through :class:`~dishsim.task.motion.MotionService`, never touching a
planner directly.

The choreography mirrors the carried-object methodology — visible, gated contact plus guaranteed
actuation. The gripper engages the rack's front handle while the rack joint's DRIVE target ramps
to the action target and the tool tracks the moving handle. The pinch is real; the drive is what
guarantees the rack actually arrives.

All geometry comes from :mod:`dishsim.rack_ops` (handle poses, engage poses, the
collision-validated slide path) — this module is choreography and gating only.

Two things are deliberately parameters rather than lookups, so the module stays Kit-free and
cache-free: the rack's cached body pose (read from the collision manifest by the caller) and a
``measure_ext`` callback for the settle check, which needs live simulator state.
"""

from dataclasses import dataclass, field

import numpy as np

from .. import config, rack_ops
from ..transforms import T_inv
from ..ur5e_kin import ik_wrist3_all


@dataclass
class RackOutcome:
    """Result of one rack action.

    Attributes:
        ok: Whether the rack reached its target within :data:`dishsim.config.RACK_SLIDE_TOL_M`.
        joint: The prismatic joint that was driven.
        stage: Failure taxonomy value when ``ok`` is False, else ``None``.
        detail: Human-readable cause when ``ok`` is False, else ``None``.
        target_m: Commanded final extension [m].
        achieved_m: Measured final extension [m], or ``None`` if never measured.
        error_m: ``|achieved - target|`` [m], or ``None``.
        q_end: Arm configuration after the disengage, shape [6], or ``None``.
    """

    ok: bool
    joint: str = ""
    stage: str | None = None
    detail: str | None = None
    target_m: float = 0.0
    achieved_m: float | None = None
    error_m: float | None = None
    q_end: np.ndarray | None = None

    def to_json(self) -> dict:
        return {
            "ok": bool(self.ok),
            "joint": self.joint,
            "stage": self.stage,
            "detail": self.detail,
            "target_m": round(float(self.target_m), 5),
            "achieved_m": None if self.achieved_m is None else round(float(self.achieved_m), 5),
            "error_m": None if self.error_m is None else round(float(self.error_m), 5),
        }


@dataclass
class RackSpec:
    """One rack action, resolved from a scenario's ``rack_action`` entry.

    Attributes:
        joint: Prismatic joint to drive.
        body: Rack body the handle belongs to (a key of :data:`dishsim.config.RACK_GEN`).
        to: Target extension [m].
        mode: ``"pull"`` (wrap the rod) or ``"push"`` (closed-fist press).
        T_base_body: The rack body's pose at the CACHED extension, shape [4, 4].
        cached_ext: The extension ``T_base_body`` was cached at [m].
    """

    joint: str
    body: str
    to: float
    mode: str
    T_base_body: np.ndarray
    cached_ext: float

    @classmethod
    def from_action(cls, action: dict, world) -> "RackSpec":
        """Build from a scenario ``rack_action`` plus the collision world holding the rack pose."""
        body = action["body"]
        state = world.manifest["statics_state"]
        cached = state["rack_upper_m"] if action["joint"].endswith("_up") else state["rack_lower_m"]
        return cls(joint=action["joint"], body=body, to=float(action["to"]), mode=action["mode"],
                   T_base_body=np.array(world.manifest["statics"][body]["T_base_body"]),
                   cached_ext=float(cached))


class RackAction:
    """Engages a rack handle and slides the rack to a target extension.

    Args:
        motion: The motion layer. Its ``world`` must be the collision world for the rack's
            CURRENT state — the slide path is validated against it at every intermediate
            extension.
        gripper_bodies: Robot bodies whose contact with the rack is the point of the exercise
            and must not trip the unexpected-contact gate.
        measure_ext: Callable returning the rack joint's measured extension [m]. Live simulator
            state, so it is injected rather than read here.
        seed: Base seed for planner calls.
        on_step: Optional per-physics-step callback (media capture).
    """

    def __init__(self, motion, *, gripper_bodies=(), measure_ext=None, seed: int = 0,
                 on_step=None) -> None:
        self.motion = motion
        self.gripper_bodies = tuple(gripper_bodies)
        self.measure_ext = measure_ext
        self.seed = int(seed)
        self.on_step = on_step
        self._n_calls = 0
        #: Joint -> extension that must stay commanded for the REST of the episode. The scene's
        #: standing targets come from the SCENARIO, which is the pre-action state, so releasing
        #: the override after a successful pull would drive the rack straight back in. It also
        #: has to accumulate: a second action that replaced this dict would drop the first
        #: rack's target and let it slide shut behind the arm.
        self._latched: dict[str, float] = {}

    def _seed(self, salt: int) -> int:
        self._n_calls += 1
        return self.seed * 1000 + self._n_calls * 17 + salt

    def _latch(self, joint: str, value: float) -> None:
        self._latched[joint] = float(value)
        self.motion.set_rack_target(dict(self._latched))

    def run(self, spec: RackSpec) -> RackOutcome:
        """Execute one rack action end to end.

        On success the action's target stays latched (see :attr:`_latched`). On failure the
        override is rolled back to whatever earlier actions latched, so a half-done pull cannot
        leave this one's partial target commanded.

        Returns:
            A :class:`RackOutcome`.
        """
        try:
            out = self._run(spec)
        except Exception:
            self.motion.set_rack_target(dict(self._latched) or None)
            raise
        if out.ok:
            self._latch(spec.joint, spec.to)
        else:
            self.motion.set_rack_target(dict(self._latched) or None)
        return out

    def _run(self, spec: RackSpec) -> RackOutcome:
        m = self.motion
        out = RackOutcome(ok=False, joint=spec.joint, target_m=spec.to)
        params = config.RACK_GEN[spec.body]
        T_w3_tcp = rack_ops.t_wrist3_tcp()
        approach = rack_ops.approach_dir(spec.T_base_body)
        open_aperture = (config.GRIPPER_APERTURE_OPEN_RAD if spec.mode == "pull"
                         else config.RACK_PUSH_APERTURE_RAD)
        slide_aperture = (config.RACK_HANDLE_APERTURE_RAD if spec.mode == "pull"
                          else config.RACK_PUSH_APERTURE_RAD)

        # A human reaches a rack handle horizontally from the front; the rod-axis sign is a free
        # choice, so try both wrist flips and keep the first that yields BOTH a reachable
        # pre-engage pose and a valid slide path. Checking the slide here — before anything
        # moves — is what stops the arm engaging a handle it cannot then follow.
        plan_r = None
        for flip in (False, True):
            T_engage = rack_ops.engage_pose(spec.T_base_body, params, _action(spec),
                                            spec.cached_ext, flip=flip)
            T_pre = T_engage.copy()
            T_pre[:3, 3] = T_engage[:3, 3] - approach * config.RACK_APPROACH_HOVER_M
            pre_sols = [q for q in ik_wrist3_all(T_pre @ T_inv(T_w3_tcp),
                                                 q_seed=np.array(config.HOME_Q))
                        if not m.world.in_collision(q)]
            if not pre_sols:
                continue
            arm_path = rack_ops.slide_arm_path(m.world, spec.T_base_body, params, _action(spec),
                                               spec.cached_ext, pre_sols[0], flip=flip)
            if arm_path is None:
                continue
            plan_r = (flip, T_engage, T_pre, pre_sols, arm_path)
            break
        if plan_r is None:
            out.stage = "rack-slide-plan"
            out.detail = "no wrist-flip variant yields a valid engage pose plus slide path"
            return out
        flip, T_engage, T_pre, pre_sols, arm_path = plan_r

        # ---- approach ---------------------------------------------------------------------
        res = m.plan(m.current_q(), np.array(pre_sols), seed=self._seed(1))
        if res.status != "solved":
            out.stage = "rack-approach-plan"
            out.detail = f"planner returned {res.status} for the handle approach"
            return out
        step = m.execute(res.path_q, phase="rack-approach", aperture=open_aperture,
                         on_step=self.on_step)
        if not step.ok:
            out.stage = "rack-approach-plan"
            out.detail = step.detail
            return out

        # ---- engage: scripted reach-in onto the handle -------------------------------------
        # gripper-rack contact is the POINT here, so the gripper bodies leave the gate; the arm
        # stays in it, because an ARM collision on the way in is still a real fault
        step = m.servo_line(T_pre, T_engage, 15, step.q_end, phase="rack-engage",
                            aperture=open_aperture, exclude=self.gripper_bodies,
                            on_step=self.on_step)
        if not step.ok:
            out.stage = "rack-slide-fault"
            out.detail = step.detail
            return out
        q_engaged = step.q_end
        if spec.mode == "pull":
            m.set_aperture(config.RACK_HANDLE_APERTURE_RAD, 45, phase="rack-grip-close",
                           arm_q=q_engaged, on_step=self.on_step)

        # Re-seed the slide from the configuration actually REACHED, not the one planned: the
        # engage reach-in tracks IK and lands slightly off, and following a path seeded from the
        # planned pose would fight the drive for the whole slide.
        arm_path2 = rack_ops.slide_arm_path(m.world, spec.T_base_body, params, _action(spec),
                                            spec.cached_ext, q_engaged, flip=flip)
        if arm_path2 is not None:
            arm_path = arm_path2

        # ---- slide: the rack drive ramps while the tool tracks the handle -------------------
        e0, e1 = spec.cached_ext, spec.to

        def ramp(frac: float) -> None:
            m.set_rack_target({**self._latched, spec.joint: e0 + (e1 - e0) * frac})

        step = m.execute(arm_path, phase="rack-slide", aperture=slide_aperture,
                         exclude=self.gripper_bodies, on_step=self.on_step, on_before_step=ramp)
        if not step.ok:
            out.stage = "rack-slide-fault"
            out.detail = step.detail
            return out
        ramp(1.0)
        m.hold(60, phase="rack-settle", arm_q=arm_path[-1], aperture=slide_aperture,
               on_step=self.on_step)

        if self.measure_ext is not None:
            out.achieved_m = float(self.measure_ext())
            out.error_m = abs(out.achieved_m - e1)
            if out.error_m > config.RACK_SLIDE_TOL_M:
                out.stage = "rack-slide-fault"
                out.detail = (f"rack settled {out.error_m * 1e3:.1f} mm off target "
                              f"(tol {config.RACK_SLIDE_TOL_M * 1e3:.0f} mm)")
                return out

        # ---- disengage: let go and back straight off the handle ------------------------------
        if spec.mode == "pull":
            m.set_aperture(config.GRIPPER_APERTURE_OPEN_RAD, 45, phase="rack-release",
                           arm_q=arm_path[-1], on_step=self.on_step)
        T_now = m.tcp_pose(arm_path[-1])
        T_off = T_now.copy()
        T_off[:3, 3] = T_now[:3, 3] - approach * config.RACK_APPROACH_HOVER_M
        step = m.servo_line(T_now, T_off, 12, arm_path[-1], phase="rack-disengage",
                            aperture=config.GRIPPER_APERTURE_OPEN_RAD,
                            exclude=self.gripper_bodies, on_step=self.on_step)
        # A failed back-off is not fatal: the rack is already where it needs to be, and the
        # episode's return-home handles an arm left near the handle.
        out.q_end = step.q_end if step.q_end is not None else arm_path[-1]
        out.ok = True
        return out


def _action(spec: RackSpec) -> dict:
    """The plain dict shape :mod:`dishsim.rack_ops` expects."""
    return {"joint": spec.joint, "body": spec.body, "to": spec.to, "mode": spec.mode}


@dataclass
class RackPhase:
    """Every rack action an episode ran, for the episode record."""

    outcomes: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(o.ok for o in self.outcomes)

    def to_json(self) -> list:
        return [o.to_json() for o in self.outcomes]


def run_sequence(action_specs, action_runner) -> RackPhase:
    """Run rack actions in order, stopping at the first failure.

    Reaching ``placement`` from ``both_in`` is one pull; reaching ``placement_open`` is two. Both
    are this function with a different list.

    Args:
        action_specs: :class:`RackSpec` values, in execution order.
        action_runner: A :class:`RackAction`.

    Returns:
        A :class:`RackPhase` whose ``ok`` is False if any action failed.
    """
    phase = RackPhase()
    for spec in action_specs:
        outcome = action_runner.run(spec)
        phase.outcomes.append(outcome)
        if not outcome.ok:
            break
    return phase
