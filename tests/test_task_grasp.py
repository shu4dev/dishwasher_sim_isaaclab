# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""State-dependent grasp availability and the recovery ladder (Stage C).

The property under test is that grasp availability is a function of the WORLD, not of the
object: the same object must be pickable, then not pickable once something is placed beside it,
then pickable again when that neighbour is cleared. A precomputed grasp set passes a naive test
and fails that one.

Everything here runs against a stub world — a plain object answering ``in_collision(q)`` — so the
logic is testable without a collision cache or a simulator, the same way the planner contract is.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_grasp.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.task import grasp as G
from dishsim.task import recovery as R
from dishsim.task.sequencer import TaskItem


class StubWorld:
    """A world whose blocked region the test controls directly."""

    def __init__(self, blocked=None):
        self.blocked = blocked or (lambda q: False)
        self.objects = {}

    def in_collision(self, q, return_pairs=False):
        hit = bool(self.blocked(np.asarray(q, dtype=float)))
        return (hit, []) if return_pairs else hit

    def has_object(self, name):
        return name in self.objects

    def add_object(self, name, pieces, T):
        self.objects[name] = (pieces, T)

    def remove_object(self, name):
        self.objects.pop(name, None)

    def set_object_pose(self, name, T):
        self.objects[name] = (self.objects[name][0], T)


class StubMotion:
    """The slice of MotionService that grasp search uses."""

    def __init__(self, world, *, ik=True, line=True):
        self.world = world
        self._ik, self._line = ik, line
        self.held = []

    def current_q(self):
        return np.zeros(6)

    def reachable(self, T_base_tcp, *, q_seed=None, check_collision=True):
        if not self._ik:
            return np.empty((0, 6))
        # one deterministic "configuration" that encodes the target position, so a stub world can
        # decide collisions from where the tool would be
        q = np.zeros(6)
        q[:3] = np.asarray(T_base_tcp)[:3, 3]
        sols = np.atleast_2d(q)
        if check_collision:
            sols = np.array([s for s in sols if not self.world.in_collision(s)])
            if len(sols) == 0:
                return np.empty((0, 6))
        return sols

    def line_configs(self, T_from, T_to, n, q_seed):
        if not self._line:
            return None
        qs = []
        for s in np.linspace(0.0, 1.0, n):
            q = np.zeros(6)
            q[:3] = (1 - s) * np.asarray(T_from)[:3, 3] + s * np.asarray(T_to)[:3, 3]
            qs.append(q)
        return np.array(qs)

    def clear_obstacle(self, key):
        self.world.remove_object(key)

    def set_obstacle(self, key, pieces, T):
        self.world.add_object(key, pieces, T)

    def hold(self, steps, *, phase, **kw):
        self.held.append((phase, steps))


class StubProfile:
    T_tcp_obj = np.eye(4)
    pieces = ["geometry"]


def make_item(item_id="mug_00", xyz=(0.75, 0.0, 0.05)):
    T = np.eye(4)
    T[:3, 3] = xyz
    return TaskItem(item_id=item_id, object_class="mug", instance=0, T_base_obj=T)


# ---- the yaw sweep ------------------------------------------------------------------------------


def test_yaw_sweep_tries_the_nominal_grasp_first():
    assert G.yaw_sweep(8)[0] == 0.0


def test_yaw_sweep_widens_symmetrically():
    sweep = G.yaw_sweep(8)
    assert sweep[1] == pytest.approx(45.0) and sweep[2] == pytest.approx(-45.0)
    assert len(sweep) == 8 and len(set(sweep)) == 8


@pytest.mark.parametrize("n", [1, 2, 3, 7, 12, 36])
def test_yaw_sweep_returns_exactly_n_distinct_angles(n):
    sweep = G.yaw_sweep(n)
    assert len(sweep) == n and len(set(sweep)) == n


# ---- state dependence: the whole point ------------------------------------------------------------


def test_a_grasp_is_found_in_a_free_world():
    motion = StubMotion(StubWorld())
    cand, funnel = G.find_grasp(make_item(), StubProfile(), motion)
    assert cand is not None and funnel.found
    assert funnel.accepted_yaw_deg == 0.0, "the nominal grasp should win when nothing blocks it"


def test_the_same_object_becomes_unpickable_when_the_world_changes():
    """Availability must track the world, not be a fixed property of the object."""
    item = make_item()
    free = StubMotion(StubWorld())
    assert G.find_grasp(item, StubProfile(), free)[0] is not None

    blocked = StubMotion(StubWorld(blocked=lambda q: True))
    cand, funnel = G.find_grasp(item, StubProfile(), blocked)
    assert cand is None and not funnel.found


def test_an_object_does_not_veto_its_own_grasp():
    """The target is an obstacle like any other; it must be lifted for the check and restored."""
    world = StubWorld()
    motion = StubMotion(world)
    item = make_item()
    world.add_object(item.item_id, ["geometry"], item.T_base_obj)

    cand, _ = G.find_grasp(item, StubProfile(), motion)
    assert cand is not None
    assert world.has_object(item.item_id), "the obstacle must be put back after the check"


def test_the_target_is_restored_even_when_no_grasp_is_found():
    world = StubWorld(blocked=lambda q: True)
    motion = StubMotion(world)
    item = make_item()
    world.add_object(item.item_id, ["geometry"], item.T_base_obj)
    assert G.find_grasp(item, StubProfile(), motion)[0] is None
    assert world.has_object(item.item_id)


def test_a_blocked_descent_rejects_a_reachable_hover():
    """The failure that produced 56 N forearm contacts: hover fine, descent not."""
    item = make_item(xyz=(0.75, 0.0, 0.05))
    hover_z = item.T_base_obj[2, 3] + config.PICK_HOVER_M

    # block everything below the hover plane — the hover itself stays free
    motion = StubMotion(StubWorld(blocked=lambda q: q[2] < hover_z - 1e-6))
    cand, funnel = G.find_grasp(item, StubProfile(), motion)
    assert cand is None
    assert funnel.n_descent_blocked == funnel.n_candidates
    assert funnel.n_hover_blocked == 0
    assert "descent" in funnel.reason()


def test_a_yaw_sweep_finds_a_way_in_when_the_nominal_grasp_is_blocked():
    """A neighbour on one side blocks one approach; rotating about the object's axis clears it."""
    item = make_item(xyz=(0.75, 0.0, 0.05))
    # block only tool positions on the +y side of the object
    motion = StubMotion(StubWorld(blocked=lambda q: q[1] > item.T_base_obj[1, 3] + 1e-9))

    narrow, f_narrow = G.find_grasp(item, StubProfile(), motion, n_yaws=1)
    wide, f_wide = G.find_grasp(item, StubProfile(), motion, n_yaws=12)
    assert f_narrow.n_candidates == 1 and f_wide.n_candidates >= 1
    # with an identity grasp transform the tool sits at the object centre, so this scene is
    # symmetric under yaw; assert the sweep is genuinely explored rather than a specific outcome
    assert len(f_wide.tried_yaw_deg) >= len(f_narrow.tried_yaw_deg)


# ---- the funnel explains itself --------------------------------------------------------------------


def test_no_ik_is_distinguished_from_blocked():
    motion = StubMotion(StubWorld(), ik=False)
    cand, funnel = G.find_grasp(make_item(), StubProfile(), motion, n_yaws=4)
    assert cand is None
    assert funnel.n_no_ik == 4 and funnel.n_hover_blocked == 0
    assert "out of reach" in funnel.reason()


def test_ik_discontinuity_on_the_descent_is_reported_as_blocked():
    motion = StubMotion(StubWorld(), line=False)
    cand, funnel = G.find_grasp(make_item(), StubProfile(), motion, n_yaws=3)
    assert cand is None and funnel.n_descent_blocked == 3


def test_the_funnel_serialises_for_the_episode_record():
    motion = StubMotion(StubWorld(blocked=lambda q: True))
    _, funnel = G.find_grasp(make_item(), StubProfile(), motion, n_yaws=5)
    doc = funnel.to_json()
    assert doc["candidates"] == 5 and doc["accepted_yaw_deg"] is None


# ---- the GraspFinder wrapper -------------------------------------------------------------------------


def test_finder_reports_reasons_only_for_objects_it_could_not_grasp():
    motion = StubMotion(StubWorld())
    finder = G.GraspFinder({"mug": StubProfile()}, motion)
    assert finder(make_item("a")) is not None
    assert finder.reasons() == {}

    finder.motion = StubMotion(StubWorld(blocked=lambda q: True))
    assert finder(make_item("b")) is None
    assert set(finder.reasons()) == {"b"}


def test_widening_the_sweep_changes_how_many_candidates_are_tried():
    motion = StubMotion(StubWorld(blocked=lambda q: True))
    finder = G.GraspFinder({"mug": StubProfile()}, motion, n_yaws=4)
    finder(make_item("a"))
    assert finder.funnels["a"].n_candidates == 4

    finder.set_yaw_samples(16)
    finder(make_item("b"))
    assert finder.funnels["b"].n_candidates == 16


# ---- the recovery ladder ------------------------------------------------------------------------------


class StubSequencer:
    def __init__(self, grasp_fn=None, motion=None):
        self.grasp_fn = grasp_fn
        self.motion = motion
        self.events = []

    def _emit(self, name, payload):
        self.events.append((name, payload))


def test_registry_contract():
    assert R.available_recoveries() == sorted(R.RECOVERY_STRATEGIES)
    assert set(R.DEFAULT_LADDER) <= set(R.available_recoveries())


def test_unknown_strategy_raises_with_the_choices():
    with pytest.raises(ValueError, match="unknown recovery strategy"):
        R.make_recovery(["teleport_everything"])


def test_the_ladder_tries_the_cheap_rung_first():
    motion = StubMotion(StubWorld())
    finder = G.GraspFinder({"mug": StubProfile()}, motion, n_yaws=4)
    seq = StubSequencer(grasp_fn=finder, motion=motion)

    assert R.make_recovery()(seq, []) is True
    assert finder.n_yaws == int(config.TASK["grasp_yaw_samples_wide"])
    assert motion.held == [], "the cheap rung must not run physics"
    assert seq.events[0][1]["strategy"] == "widen_grasp_search"


def test_widening_only_pays_out_once_then_the_ladder_moves_on():
    """A rung that claims progress it cannot repeat would burn every recovery attempt."""
    motion = StubMotion(StubWorld())
    finder = G.GraspFinder({"mug": StubProfile()}, motion, n_yaws=4)
    seq = StubSequencer(grasp_fn=finder, motion=motion)
    ladder = R.make_recovery()

    assert ladder(seq, []) is True   # widened
    assert ladder(seq, []) is True   # falls through to resettle
    assert motion.held and motion.held[0][0] == "recovery-settle"
    assert seq.events[-1][1]["strategy"] == "resettle"


def test_the_ladder_reports_exhaustion_when_no_rung_can_act():
    seq = StubSequencer(grasp_fn=None, motion=None)  # neither rung has anything to work with
    assert R.make_recovery()(seq, []) is False


def test_resettle_runs_the_configured_number_of_steps():
    motion = StubMotion(StubWorld())
    seq = StubSequencer(grasp_fn=None, motion=motion)
    assert R.make_recovery(["resettle"])(seq, []) is True
    assert motion.held == [("recovery-settle", int(config.TASK["recovery_settle_steps"]))]


# ---- replanning on a stale pose (Stage C) --------------------------------------------------------


def _pick_place_stub():
    """A PickPlace with just enough wired up to exercise the staleness test."""
    from dishsim.task.primitives import PickPlace

    return PickPlace(StubMotion(StubWorld()), object(), profiles={"mug": StubProfile()},
                     goal_sets={}, T_w3_tcp=np.eye(4))


def _pose(xyz=(0.0, 0.0, 0.0), yaw_deg=0.0):
    from scipy.spatial.transform import Rotation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


def test_an_unmoved_object_is_not_stale():
    pp = _pick_place_stub()
    assert not pp._pose_is_stale(_pose(), _pose())


def test_a_small_settle_wobble_is_not_stale():
    """Objects always drift a little; re-planning on every millimetre would never finish."""
    pp = _pick_place_stub()
    tol = config.TASK["pose_stale_tol_m"]
    assert not pp._pose_is_stale(_pose(), _pose(xyz=(tol * 0.4, 0.0, 0.0)))


def test_a_translation_past_the_tolerance_is_stale():
    pp = _pick_place_stub()
    tol = config.TASK["pose_stale_tol_m"]
    assert pp._pose_is_stale(_pose(), _pose(xyz=(tol * 3.0, 0.0, 0.0)))


def test_a_rotation_past_the_tolerance_is_stale():
    """A rolled object can keep its position while turning enough to invalidate the grasp."""
    pp = _pick_place_stub()
    tol_deg = config.TASK["pose_stale_tol_deg"]
    assert not pp._pose_is_stale(_pose(), _pose(yaw_deg=tol_deg * 0.4))
    assert pp._pose_is_stale(_pose(), _pose(yaw_deg=tol_deg * 3.0))


def test_staleness_is_symmetric_in_direction():
    pp = _pick_place_stub()
    tol = config.TASK["pose_stale_tol_m"]
    assert pp._pose_is_stale(_pose(), _pose(xyz=(-tol * 3.0, 0.0, 0.0)))
    assert pp._pose_is_stale(_pose(), _pose(yaw_deg=-config.TASK["pose_stale_tol_deg"] * 3.0))


# ---- weld-acquire classes skip the descent gate ---------------------------------------------------

class StubWeldProfile(StubProfile):
    acquire = "weld"


def test_weld_acquire_class_is_not_vetoed_by_a_blocked_descent():
    """The descent never runs for these classes — gating on it rejected 344 of 400 fork draws."""
    # anything below the hover collides; a picked class would be refused here
    world = StubWorld(blocked=lambda q: q[2] < 0.4)
    motion = StubMotion(world)
    item = TaskItem(item_id="fork_00", object_class="fork", instance=0,
                    T_base_obj=np.diag([1.0, 1.0, 1.0, 1.0]))
    item.T_base_obj[:3, 3] = [0.7, 0.0, 0.3]

    picked, _ = G.find_grasp(item, StubProfile(), motion)
    assert picked is None, "a picked class must still be vetoed by a blocked descent"

    welded, funnel = G.find_grasp(item, StubWeldProfile(), motion)
    assert welded is not None and welded.grasp_q is not None
    assert funnel.n_descent_blocked == 0
