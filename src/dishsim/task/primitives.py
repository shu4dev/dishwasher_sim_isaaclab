# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pick and place for ONE object — the middle of the sandwich.

The sequencer decides *which* object; this module executes *a* pick-and-place; the motion layer
below moves the arm. Splitting it this way is what keeps :mod:`dishsim.task.motion` free of
object knowledge: everything class-specific (the grasp transform, the calibrated aperture and
force band, the weld, which contact partners are expected) is resolved here and handed down as
plain numbers and geometry.

The choreography reproduces the validated single-object sequence in
``scripts/experiment/run_trials.py`` phase for phase, including the gates that took a debugging
cycle each to get right — contact before the close, the per-pad force band, the weld-error
check, and the airborne external-load check after the lift. The multi-object differences are:

* the collision world is mutated as the scene changes — the object being picked stops being an
  obstacle the moment it is in hand, and becomes one again where it was placed;
* every object still on the counter is an obstacle for every pick, which is the whole reason a
  grasp can be blocked in Stage C;
* the payload is attached to the planning cluster per pick, so plans while carrying a cup are
  computed against a cup-shaped cluster rather than a stale one.
"""

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .. import config, placement
from ..transforms import T_inv, make_T
from .sequencer import GraspCandidate, PickOutcome, TaskItem


class SceneAccess(Protocol):
    """Object-side simulator access. Implemented by the runner once Kit is booted.

    Objects are addressed by ``item_id``, never by the per-class ``instance`` index: a scene
    holding ``mug_00`` and ``cup_00`` has two items whose instance is 0, so an instance-keyed
    lookup would silently read one object's pose while welding the other's.
    """

    def object_pose_base(self, item_id: str) -> np.ndarray:
        """Measured object pose in the robot-base frame, shape [4, 4]."""

    def set_weld(self, item_id: str, enabled: bool) -> None:
        """Enable or disable the hidden wrist weld for one object."""

    def grip_forces(self, item_id: str) -> dict:
        """``{"partners", "pads_n", "external_n", "net_n"}`` for one object."""

    def grasp_origin_base(self, arm_q: np.ndarray, object_class: str) -> np.ndarray:
        """BASE-frame position the object's origin should occupy when grasped at ``arm_q``.

        Base frame, matching :meth:`object_pose_base`. Mixing the two frames here costs exactly
        the robot base height (250 mm) and reads as a catastrophic weld error.
        """

    def gripper_bodies(self) -> tuple:
        """Robot body names whose contact with the object is expected during a grasp."""

    def place_in_gripper(self, item_id: str, arm_q: np.ndarray, object_class: str) -> None:
        """Teleport an object to the calibrated carry pose for ``arm_q``, at rest.

        Only used by the weld-acquire path, for grasp families that have no calibrated
        countertop pick. Velocities must be zeroed with the pose or the object arrives carrying
        its countertop settle motion and fights the weld.
        """


@dataclass
class GraspProfile:
    """Everything class-specific about holding one object, resolved from its registry entry.

    Passing this down rather than letting lower layers read the registry is what keeps the
    motion layer object-agnostic — it receives an aperture and a force band, not a mug.
    """

    object_class: str
    T_tcp_obj: np.ndarray
    aperture_grasp_rad: float
    aperture_open_rad: float
    force_min_n: float
    force_max_n: float
    force_exec_max_n: float
    pieces: list
    #: How the object gets into the hand. ``"pick"`` is the calibrated descend-and-close.
    #: ``"weld"`` is for grasp families with no calibrated countertop pick (edge_pinch,
    #: rim_edge, handle_pinch — a flat disc or a 30 g fork cannot be pinched off a table without
    #: dragging it): the arm still flies to the object's hover, but the object is then snapped
    #: to the calibrated carry transform rather than acquired by friction. It is NOT a pick and
    #: the records say so.
    acquire: str = "pick"

    @staticmethod
    def build(object_class: str, scenario: str) -> "GraspProfile":
        """Resolve a class's grasp parameters and collision geometry from config + its cache."""
        spec = config.OBJECTS[object_class]
        g = spec.grasp
        if g.aperture_rad is None:
            raise RuntimeError(
                f"{object_class!r} has no calibrated grasp aperture — run "
                f"scripts/setup/calibrate_grasp.py --object {object_class} and freeze it first"
            )
        weld = g.family in config.WELD_ACQUIRE_FAMILIES
        if not weld and None in (g.force_min_n, g.force_max_n, g.force_exec_max_n):
            raise RuntimeError(
                f"{object_class!r} has no calibrated pad-force band — run "
                f"scripts/setup/calibrate_grasp.py --object {object_class} and freeze it first"
            )
        # A weld-acquire class may have no calibrated pinch band (the jaws never close on the
        # object, so there is nothing to calibrate): the pinch gate is vacuous for it and the
        # carry is protected by the external-load gate alone.
        return GraspProfile(
            object_class=object_class,
            T_tcp_obj=make_T(*config.grasp_transform(spec)),
            aperture_grasp_rad=float(g.aperture_rad),
            aperture_open_rad=float(config.GRIPPER_APERTURE_OPEN_RAD),
            force_min_n=0.0 if g.force_min_n is None else float(g.force_min_n),
            force_max_n=float("inf") if g.force_max_n is None else float(g.force_max_n),
            force_exec_max_n=float("inf") if g.force_exec_max_n is None else float(g.force_exec_max_n),
            pieces=_pieces_for(object_class, scenario),
            acquire="weld" if weld else "pick",
        )


def _pieces_for(object_class: str, scenario: str) -> list:
    """Convex pieces for a class, read under that class's own cache-hash context."""
    from ..collision_world import load_object_pieces  # noqa: PLC0415

    with config.active_object(object_class):
        return load_object_pieces(config.scenario_cache_dir(scenario, object_name=object_class))


class PickPlace:
    """Executes one pick-and-place, mutating the world as the scene changes.

    Args:
        motion: The motion layer.
        scene: Object-side simulator access.
        profiles: ``{object_class: GraspProfile}``.
        goal_sets: ``{object_class: {slot_id: [K, 6] configs}}`` from ``goal_configs.py``.
        seed: Base seed for planner calls.
        on_step: Optional per-physics-step callback (media capture).
        T_w3_tcp: Wrist-to-TCP transform, shape [4, 4].
    """

    def __init__(self, motion, scene: SceneAccess, *, profiles, goal_sets, T_w3_tcp,
                 seed: int = 0, on_step=None) -> None:
        self.motion = motion
        self.scene = scene
        self.profiles = profiles
        self.goal_sets = goal_sets
        self.T_w3_tcp = np.asarray(T_w3_tcp)
        self.seed = int(seed)
        self.on_step = on_step
        self._n_calls = 0
        self.n_replans = 0
        #: Optional Stage C grasp oracle; falls back to the built-in single-yaw check.
        self.grasp_finder = None

    # ---- grasp candidates ---------------------------------------------------------------------

    #: Points sampled along the descent, as fractions of the hover-to-grasp travel. The last
    #: stretch is deliberately excluded: near the grasp the pads are *inside* the object's
    #: inflated hull by design, so any check there would veto every valid grasp.
    DESCENT_SAMPLES = (0.0, 0.25, 0.5, 0.75)

    def grasp_candidate(self, item: TaskItem) -> GraspCandidate | None:
        """A collision-free way to pick ``item`` in the CURRENT world, or ``None``.

        Checks the pre-grasp hover *and* the descent beneath it. The single-object runner checks
        only the hover, which is safe when the object is alone on an empty counter — the descent
        is a short straight drop into free space. With neighbours it is not: a hover can be
        perfectly reachable while the forearm sweeps through the object next to it on the way
        down, which shows up as a 56 N contact mid-descent rather than as an unreachable pose.

        The object being picked is temporarily removed from the world for the check. It must be:
        it is registered as an obstacle like every other item, and an object cannot be allowed to
        veto its own grasp.

        Stage C extends this with a yaw sweep over alternative grasps and the recovery ladder;
        the state-dependence is already here.
        """
        if self.grasp_finder is not None:
            return self.grasp_finder(item)  # Stage C: yaw sweep + rejection funnel
        prof = self.profiles[item.object_class]
        T_obj = np.asarray(item.T_base_obj)
        T_grasp = T_obj @ T_inv(prof.T_tcp_obj)
        T_hover = T_grasp.copy()
        T_hover[2, 3] += config.PICK_HOVER_M

        had_obstacle = self.motion.world.has_object(item.item_id)
        if had_obstacle:
            self.motion.clear_obstacle(item.item_id)
        try:
            sols = self.motion.reachable(T_hover, q_seed=self.motion.current_q())
            if len(sols) == 0:
                return None
            q = self._descent_clear(T_hover, T_grasp, sols)
        finally:
            if had_obstacle:
                self.motion.set_obstacle(item.item_id, prof.pieces, T_obj)
        if q is None:
            return None
        return GraspCandidate(
            item_id=item.item_id, object_class=item.object_class,
            T_base_obj=T_obj, T_base_grasp=T_grasp, grasp_q=q,
        )

    def _descent_clear(self, T_hover, T_grasp, hover_sols):
        """First hover configuration whose whole descent stays collision-free, or ``None``."""
        for q_hover in hover_sols:
            seg = self.motion.line_configs(T_hover, T_grasp, len(self.DESCENT_SAMPLES) + 2, q_hover)
            if seg is None:
                continue
            n_check = int(round(self.DESCENT_SAMPLES[-1] * (len(seg) - 1))) + 1
            if not any(self.motion.world.in_collision(qq) for qq in seg[:n_check]):
                return q_hover
        return None

    # ---- the pick-and-place -------------------------------------------------------------------

    def _pose_is_stale(self, T_then, T_now) -> bool:
        """Has the object moved enough since the grasp was computed to invalidate it?"""
        import numpy.linalg as npl  # noqa: PLC0415

        d_m = float(npl.norm(np.asarray(T_now)[:3, 3] - np.asarray(T_then)[:3, 3]))
        R = np.asarray(T_then)[:3, :3].T @ np.asarray(T_now)[:3, :3]
        d_deg = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))
        return (d_m > float(config.TASK["pose_stale_tol_m"])
                or d_deg > float(config.TASK["pose_stale_tol_deg"]))

    def __call__(self, item: TaskItem, candidate: GraspCandidate) -> PickOutcome:
        """Pick ``item`` off the counter and place it into its allocated slot.

        A failed pick must leave the world exactly as it found it. Without that, a failure
        midway through the grasp strands the weld enabled and the payload attached, so the arm
        is physically holding an object the planner still believes is sitting on the counter —
        and every subsequent plan starts inside an obstacle. That shows up as a mystifying
        "start tree could not be initialized" on the NEXT object rather than here.
        """
        self._n_calls += 1
        prof = self.profiles[item.object_class]
        t0 = float(self.motion.stats.plan_time_s)
        replans0 = self.n_replans
        try:
            q_carry = (self._acquire_welded(item, candidate, prof) if prof.acquire == "weld"
                       else self._pick(item, candidate, prof))
            self._place(item, prof, q_carry)
        except _Failure as f:
            self._abandon(item, prof)
            return PickOutcome(False, f.stage, f.detail,
                               plan_time_s=float(self.motion.stats.plan_time_s) - t0,
                               n_replans=self.n_replans - replans0)
        return PickOutcome(True, plan_time_s=float(self.motion.stats.plan_time_s) - t0,
                           n_replans=self.n_replans - replans0)

    def _abandon(self, item, prof) -> None:
        """Restore world, physics and arm state after a failed pick-and-place.

        Three things have to be undone, in order, or the failure propagates to the next object:

        1. the weld and the jaws — otherwise the arm is still physically holding an object the
           planner believes is on the counter;
        2. the object's own resting place — it has to fall and settle before its pose means
           anything, so the obstacle is registered where it actually ended up;
        3. the arm itself — a failure can leave it deep inside the machine, and every later plan
           starts from ``current_q``. A start configuration in collision surfaces as an
           unexplained "start tree could not be initialized" on a completely different object.
        """
        try:
            self.scene.set_weld(item.item_id, False)
            self.motion.set_aperture(prof.aperture_open_rad, 20, phase="abandon-open",
                                     on_step=self.on_step)
            self.motion.hold(60, phase="abandon-settle", on_step=self.on_step)
        except Exception:  # noqa: BLE001 — cleanup must not mask the original failure
            pass
        if self.motion.payload == item.item_id:
            self.motion.detach_payload()
        try:
            self.motion.set_obstacle(item.item_id, prof.pieces,
                                     self.scene.object_pose_base(item.item_id))
        except Exception:  # noqa: BLE001
            pass
        # settle_steps=0: the abandon path already held for 60 steps above, and a full
        # SETTLE_STEPS pause after every failed pick would stretch a bad episode for no evidence
        self.return_home(phase="abandon-home", settle_steps=0)

    def home_error(self) -> float:
        """Largest per-joint deviation of the arm from :data:`dishsim.config.HOME_Q` [rad]."""
        return float(np.abs(self.motion.current_q() - np.array(config.HOME_Q, dtype=float)).max())

    def return_home(self, *, phase: str = "return-home", settle_steps: int | None = None) -> str:
        """Bring the arm back to the home configuration, which is known collision-free.

        Planned if possible. If the arm is wedged somewhere the planner cannot escape — the
        usual case after an execution failure, since the start state is what failed — fall back
        to a direct joint-space interpolation: empty-handed and jaws open, the retreat is far
        safer than leaving the arm where it is, and the contact gate still reports what it
        brushes.

        The jaws are opened and any payload released *before* the arm moves. Without that guard
        an interrupted carry would drag its object home and drop it somewhere arbitrary — and
        the planner, which believes the payload is still attached, would plan the retreat around
        a cluster that no longer matches the hardware.

        Args:
            phase: Phase label recorded for the motion; the settle hold appends ``"-settle"``.
            settle_steps: Physics steps to hold at home afterwards. ``None`` uses
                :data:`dishsim.config.SETTLE_STEPS`.

        Returns:
            ``"skipped"`` if already within ``TASK["home_tol_rad"]``, ``"planned"`` if a planned
            path was executed, or ``"lerp"`` if the uncollision-checked fallback ran. Callers
            record this: a ``"lerp"`` retreat is the one that can disturb a placed load.
        """
        self._release_payload(phase)
        home = np.array(config.HOME_Q, dtype=float)
        q_now = self.motion.current_q()
        settle = config.SETTLE_STEPS if settle_steps is None else int(settle_steps)
        if float(np.abs(q_now - home).max()) < float(config.TASK["home_tol_rad"]):
            return "skipped"
        status = "lerp"
        if not self.motion.world.in_collision(q_now):
            res = self.motion.plan(q_now, home[None, :], seed=self._seed(9))
            if res.status == "solved":
                self.motion.execute(res.path_q, phase=phase, on_step=self.on_step)
                status = "planned"
        if status == "lerp":
            n = 60
            lerp = q_now[None, :] + np.linspace(0.0, 1.0, n)[:, None] * (home - q_now)[None, :]
            self.motion.execute(lerp, phase=phase, on_step=self.on_step)
        if settle > 0:
            self.motion.hold(settle, phase=f"{phase}-settle", arm_q=home, on_step=self.on_step)
        return status

    def _release_payload(self, phase: str) -> None:
        """Drop the weld, open the jaws and let the object settle, if one is still held.

        A no-op on the normal path — ``_place`` releases, and ``_abandon`` has already run its
        own release by the time it calls back here. It only bites on the interrupted-carry path.
        """
        held = self.motion.payload
        if held is None:
            return
        try:
            self.scene.set_weld(held, False)
            self.motion.set_aperture(config.GRIPPER_APERTURE_OPEN_RAD, 20,
                                     phase=f"{phase}-open", on_step=self.on_step)
            self.motion.hold(60, phase=f"{phase}-drop", on_step=self.on_step)
        except Exception:  # noqa: BLE001 — the retreat matters more than a tidy release
            pass
        if self.motion.payload == held:
            self.motion.detach_payload()

    def _pick(self, item, candidate, prof) -> np.ndarray:
        m, cap = self.motion, self.on_step
        # measured, not the layout pose: settling and neighbour removal both move objects
        T_obj = self.scene.object_pose_base(item.item_id)

        # Stage C: the candidate was computed against a pose read a moment ago. If the object has
        # shifted since — a neighbour was removed and it rolled, or the last pick nudged it — the
        # grasp is aimed at where it used to be, and executing it drives the gripper into empty
        # space or into the object's side. Recompute rather than proceed on a stale aim.
        if candidate is not None and self._pose_is_stale(candidate.T_base_obj, T_obj):
            self.n_replans += 1
            item.T_base_obj = T_obj
            candidate = self.grasp_candidate(item)
            if candidate is None:
                raise _Failure("pick-plan",
                               "object moved after the grasp was computed and no grasp is "
                               "available at its new pose")
        T_grasp = T_obj @ T_inv(prof.T_tcp_obj)
        T_hover = T_grasp.copy()
        T_hover[2, 3] += config.PICK_HOVER_M

        # Prefer the configuration whose descent grasp_candidate already validated, so the
        # descent that runs is the one that was checked. Falling back to a fresh IK solve would
        # quietly execute an unvalidated descent — which is how a 56 N forearm contact appears
        # at a pose the feasibility check had approved.
        sols = np.atleast_2d(candidate.grasp_q) if candidate.grasp_q is not None else \
            m.reachable(T_hover, q_seed=m.current_q())
        if len(sols) == 0:
            raise _Failure("pick-plan", "no collision-free IK at the pre-grasp hover")
        res = m.plan(m.current_q(), sols, seed=self._seed(1))
        if res.status != "solved":
            raise _Failure("pick-plan", f"planner returned {res.status} for the pick approach")
        self._run(m.execute(res.path_q, phase="pick-approach",
                            aperture=prof.aperture_open_rad, on_step=cap), "pick-plan")

        # descent is scripted and NOT collision-checked: at the grasp pose the pads are inside
        # the object's inflated hull by design, so FCL would veto every valid grasp
        self._run(m.servo_line(T_hover, T_grasp, config.PICK_DESCEND_STEPS // 6, res.path_q[-1],
                               phase="pick-descend", aperture=prof.aperture_open_rad,
                               exclude=self.scene.gripper_bodies(), on_step=cap),
                  "pick-grasp-fault")
        q_at_grasp = m.current_q()

        m.hold(30, phase="pick-pregrasp-settle", arm_q=q_at_grasp,
               aperture=prof.aperture_open_rad, on_step=cap)
        f0 = self.scene.grip_forces(item.item_id)
        if f0["partners"] and max(f0["partners"].values()) > 0.05:
            raise _Failure("pick-grasp-fault",
                           f"contact before the close ({max(f0['partners'].values()):.2f} N)")

        m.set_aperture(prof.aperture_grasp_rad, config.GRIPPER_CLOSE_RAMP_STEPS,
                       phase="pick-close", arm_q=q_at_grasp, on_step=cap)
        m.hold(60, phase="pick-hold", arm_q=q_at_grasp, on_step=cap)
        self._check_pinch(item, prof)

        # the visible pinch is real; the hidden weld carries the load during transport
        self.scene.set_weld(item.item_id, True)
        m.hold(30, phase="pick-weld-verify", arm_q=q_at_grasp, on_step=cap)
        # both sides in the BASE frame — see SceneAccess.grasp_origin_base
        err_mm = float(np.linalg.norm(
            self.scene.object_pose_base(item.item_id)[:3, 3]
            - self.scene.grasp_origin_base(q_at_grasp, item.object_class)
        )) * 1e3
        if err_mm > 5.0:
            raise _Failure("pick-weld-fault", f"weld error {err_mm:.1f} mm after enable")

        # in hand now: stop being a world obstacle, start being the carried payload
        m.clear_obstacle(item.item_id)
        m.attach_payload(item.item_id, prof.pieces, self.T_w3_tcp @ prof.T_tcp_obj)

        T_lift = T_grasp.copy()
        T_lift[2, 3] += config.PICK_LIFT_M
        self._run(m.servo_line(T_grasp, T_lift, 15, q_at_grasp, phase="pick-lift",
                               exclude=self.scene.gripper_bodies(), on_step=cap),
                  "pick-grasp-fault")
        if self.scene.grip_forces(item.item_id)["external_n"] > config.CONTACT_FORCE_THRESH_N:
            raise _Failure("pick-grasp-fault", "object still externally loaded after the lift")
        return m.current_q()

    def _acquire_welded(self, item, candidate, prof) -> np.ndarray:
        """Take a weld-carried class into the hand without a calibrated countertop pick.

        Grasp families in :data:`dishsim.config.WELD_ACQUIRE_FAMILIES` contact the pads one-sided
        and drag a free-lying object during the close, so no descend-and-pinch exists for them.
        The single-object runner sidesteps this by SPAWNING the object already welded at the carry
        pose — which works exactly once per episode and so is useless when four items have to be
        cleared in turn.

        What runs instead: the arm plans and flies to the object's pre-grasp hover as it would for
        a real pick, the object is then snapped to the calibrated carry transform, the weld takes
        it, and the jaws close on camera. The descent is deliberately NOT attempted — it is the
        uncalibrated part, and faking it with an unvalidated straight line is how a 56 N forearm
        contact gets past a feasibility check that approved the hover.

        This is an acquisition, not a pick, and it is labelled that way everywhere it is recorded.
        """
        m, cap = self.motion, self.on_step
        T_obj = self.scene.object_pose_base(item.item_id)
        T_grasp = T_obj @ T_inv(prof.T_tcp_obj)
        T_hover = T_grasp.copy()
        T_hover[2, 3] += config.PICK_HOVER_M

        sols = np.atleast_2d(candidate.grasp_q) if candidate.grasp_q is not None else \
            m.reachable(T_hover, q_seed=m.current_q())
        if len(sols) == 0:
            raise _Failure("pick-plan", "no collision-free IK at the pre-grasp hover")
        # The object MATERIALIZES at the carry transform the moment the weld takes it, so an
        # approach goal is only valid if it stays collision-free WITH the payload attached.
        # The empty-hand feasibility that chose the candidate cannot see that: a bowl hover
        # can be fine bare-handed and clip the countertop hull the instant the bowl snaps in
        # ("carry start configuration is in collision", measured 2026-08-14). If the finder's
        # single config dies this way, widen back to every hover branch before giving up.
        m.world.attach_object(prof.pieces, self.T_w3_tcp @ prof.T_tcp_obj)
        try:
            payload_ok = [q for q in np.atleast_2d(sols) if not m.world.in_collision(q)]
            if not payload_ok and candidate.grasp_q is not None:
                payload_ok = [q for q in m.reachable(T_hover, q_seed=m.current_q())
                              if not m.world.in_collision(q)]
        finally:
            m.world.detach_carried_object()
        if not payload_ok:
            raise _Failure("pick-plan",
                           "no hover configuration stays collision-free with the payload attached")
        sols = np.asarray(payload_ok)
        res = m.plan(m.current_q(), sols, seed=self._seed(1))
        if res.status != "solved":
            raise _Failure("pick-plan", f"planner returned {res.status} for the approach")
        self._run(m.execute(res.path_q, phase="acquire-approach",
                            aperture=prof.aperture_open_rad, on_step=cap), "pick-plan")
        q_at_grasp = m.current_q()

        # snap to the carry transform, then let the weld take it before the jaws move
        self.scene.place_in_gripper(item.item_id, q_at_grasp, item.object_class)
        self.scene.set_weld(item.item_id, True)
        m.hold(30, phase="acquire-weld", arm_q=q_at_grasp, aperture=prof.aperture_open_rad,
               on_step=cap)
        err_mm = float(np.linalg.norm(
            self.scene.object_pose_base(item.item_id)[:3, 3]
            - self.scene.grasp_origin_base(q_at_grasp, item.object_class)
        )) * 1e3
        if err_mm > 5.0:
            raise _Failure("pick-weld-fault", f"weld error {err_mm:.1f} mm after the snap")

        m.set_aperture(prof.aperture_grasp_rad, config.GRIPPER_CLOSE_RAMP_STEPS,
                       phase="acquire-close", arm_q=q_at_grasp, on_step=cap)
        m.hold(60, phase="acquire-hold", arm_q=q_at_grasp, on_step=cap)

        m.clear_obstacle(item.item_id)
        m.attach_payload(item.item_id, prof.pieces, self.T_w3_tcp @ prof.T_tcp_obj)
        return q_at_grasp

    def _place(self, item, prof, q_carry) -> None:
        m, cap = self.motion, self.on_step
        slot = item.goal_slot
        goals = self.goal_sets.get(item.object_class, {}).get(slot.slot_id)
        if goals is None or len(goals) == 0:
            raise _Failure("no-goal-config", f"slot {slot.slot_id} has no goal configurations")
        # Goal sets are precomputed by goal_configs.py against an EMPTY rack. By the time the
        # second object is placed the rack holds the first, so a goal that was collision-free
        # offline may not be now — re-filter against the live world rather than handing the
        # planner goals it can never reach.
        goals = np.asarray(goals)
        rng = np.random.default_rng(self._seed(2))
        order = rng.permutation(len(goals))
        keep, budget = [], min(config.GOALS_PER_PLAN, len(goals))
        for i in order:
            if not self.motion.world.in_collision(goals[i]):
                keep.append(goals[i])
                if len(keep) >= budget:
                    break
        if not keep:
            raise _Failure("no-goal-config",
                           f"all {len(goals)} goal configs for slot {slot.slot_id} are blocked "
                           f"in the current world (already-placed objects)")
        sub = np.asarray(keep)

        if m.world.in_collision(q_carry):
            raise _Failure("pick-plan", "carry start configuration is in collision")
        res = m.plan(q_carry, sub, seed=self._seed(3))
        if res.status != "solved":
            raise _Failure("planner-timeout", f"planner returned {res.status} for the transfer")
        # The validated single-object transfer gates on THREE conditions, not one: unexpected
        # robot contact (handled inside execute), the carried object picking up an external
        # load, and the pads squeezing past their dynamic limit. Dropping the latter two means a
        # transfer that scrapes the object along a rack, or crushes it, runs to completion and
        # is judged only at the end — by which point the evidence of what went wrong is gone.
        out = m.execute(res.path_q, phase="place-execute", on_step=cap,
                        gate=lambda: self._carry_gate(item, prof))
        if not out.ok:
            raise _Failure("execution-collision", out.detail)
        q_goal = np.asarray(res.path_q[-1], dtype=float)

        m.hold(30, phase="place-hold", arm_q=q_goal, on_step=cap)
        # jaws open on camera BEFORE the weld drops, so the release is visibly real
        m.set_aperture(prof.aperture_open_rad, config.GRIPPER_CLOSE_RAMP_STEPS,
                       phase="place-open", arm_q=q_goal, on_step=cap)
        m.hold(30, phase="post-open-hold", arm_q=q_goal, on_step=cap)
        gf = self.scene.grip_forces(item.item_id)
        if gf["partners"] and max(gf["partners"].values()) > 0.05:
            raise _Failure("release-fault", "pads still load the object after opening")
        self.scene.set_weld(item.item_id, False)
        m.hold(150, phase="release-settle", arm_q=q_goal, on_step=cap)

        # placed: it is world geometry again, at wherever it actually came to rest
        m.detach_payload()
        T_placed = self.scene.object_pose_base(item.item_id)
        m.set_obstacle(item.item_id, prof.pieces, T_placed)

        T_goal_tcp = m.tcp_pose(q_goal)
        T_back = T_goal_tcp.copy()
        if config.placement_mode_params(slot.mode).get("retract_world_up"):
            # deep-release modes (flat lay between tray rims): straight up in the base frame —
            # the tool-z direction is pitched and would drag the open finger through the wires
            T_back[:3, 3] += np.array([0.0, 0.0, config.RETRACT_DIST_M])
        else:
            T_back[:3, 3] -= T_goal_tcp[:3, 2] * config.RETRACT_DIST_M
        out = m.servo_line(T_goal_tcp, T_back, 12, q_goal, phase="retract",
                           aperture=prof.aperture_open_rad, on_step=cap)
        # Only a real contact fails the trial. A retract with no continuous IK branch leaves the
        # tool where it is, which is untidy but harmless — the same judgement the single-object
        # runner makes when it records "skipped-no-valid-path" and still evaluates the placement.
        # Deep-release modes may raise the graze bar (retract_graze_max_n): leaving a tray
        # brushes wires transiently, and the post-retract settle verdict is the real judge.
        graze_max = float(config.placement_mode_params(slot.mode).get(
            "retract_graze_max_n", config.RETRACT_GRAZE_MAX_N))
        if not out.ok and out.peak_contact_n >= graze_max:
            raise _Failure("retract-collision", out.detail)

        m.hold(config.SETTLE_STEPS, phase="final-settle", on_step=cap)
        # evaluate_placement reads the ACTIVE object's axis convention and bbox, so it must run
        # with this item's class active — judging a Z-up cup against the Y-up mug's convention
        # reports a perfectly seated object as lying on its side at 86 degrees of tilt
        with config.active_object(item.object_class):
            ev = placement.evaluate_placement(slot, self.scene.object_pose_base(item.item_id))
        if not ev.pop("ok"):
            raise _Failure("unstable-after-release", f"placement out of tolerance: {ev}")

    # ---- helpers -------------------------------------------------------------------------------

    def _carry_gate(self, item, prof) -> str | None:
        """Per-step check that the object is still properly held during the transfer."""
        gf = self.scene.grip_forces(item.item_id)
        if gf["external_n"] > config.CONTACT_FORCE_THRESH_N:
            return f"carried object externally loaded ({gf['external_n']:.2f} N)"
        pads = gf["pads_n"]
        if pads and max(pads) > prof.force_exec_max_n:
            return f"pad squeeze {max(pads):.2f} N over the dynamic limit {prof.force_exec_max_n:.2f} N"
        return None

    def _check_pinch(self, item, prof) -> None:
        """Per-pad force band plus non-pad silence — the calibrated grasp gate."""
        gf = self.scene.grip_forces(item.item_id)
        for pad, mag in zip(config.GRIP_PAD_BODIES, gf["pads_n"]):
            if not prof.force_min_n <= mag <= prof.force_max_n:
                raise _Failure("pick-grasp-fault",
                               f"{pad} force {mag:.2f} N outside the calibrated band "
                               f"[{prof.force_min_n:.2f}, {prof.force_max_n:.2f}]")
        for name, mag in gf["partners"].items():
            if name not in config.GRIP_PAD_BODIES and mag > config.CONTACT_FORCE_THRESH_N:
                raise _Failure("pick-grasp-fault",
                               f"non-pad partner {name} touches the object ({mag:.2f} N)")

    def _run(self, result, stage: str) -> None:
        if not result.ok:
            raise _Failure(stage, result.detail)

    def _seed(self, k: int) -> int:
        """Distinct seeds per planner call.

        The single-object runner reused one seed across two different plan calls; harmless there
        because the problems differed, but a multi-object episode makes many more calls and
        colliding seeds would correlate them.
        """
        return self.seed * 1009 + self._n_calls * 17 + k


class _Failure(Exception):
    """Internal control flow: a failed stage with its taxonomy value."""

    def __init__(self, stage: str, detail: str | None) -> None:
        super().__init__(f"{stage}: {detail}")
        self.stage = stage
        self.detail = detail
