# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The task sequencer, driven against a stub motion layer — no Kit, no physics, no planner.

That this is possible at all is the point of the layer split: the sequencer's decisions (which
object next, when something is blocked, what to do when nothing is pickable) are pure logic over
injected callables, so they can be tested exhaustively in milliseconds instead of via a
simulator. The same property is what lets Stage B and Stage C be validated before they are ever
run in Isaac.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_sequencer.py -v
"""

import numpy as np
import pytest

from dishsim.task.sequencer import GraspCandidate, PickOutcome, TaskItem, TaskSequencer


class StubMotion:
    """Satisfies everything the sequencer asks of a motion layer: where the arm is."""

    def __init__(self, q=None):
        self.q = np.zeros(6) if q is None else np.asarray(q, dtype=float)

    def current_q(self):
        return self.q

    def tcp_pose(self, q=None):
        T = np.eye(4)
        T[:3, 3] = (0.0, 0.0, 0.0)  # tool parked at the base origin: cost == distance to object
        return T


def make_items(spec):
    """``spec`` maps item_id -> (x, y, z); poses are all that the stub grasp/cost need."""
    items = []
    for i, (item_id, xyz) in enumerate(spec.items()):
        T = np.eye(4)
        T[:3, 3] = xyz
        cls = item_id.rsplit("_", 1)[0]
        items.append(TaskItem(item_id=item_id, object_class=cls, instance=i, T_base_obj=T))
    return items


def grasp_at_object(item):
    """Trivially pickable: the grasp is the object pose."""
    return GraspCandidate(item.item_id, item.object_class, item.T_base_obj, item.T_base_obj,
                          grasp_q=np.zeros(6))


def always_succeeds(item, candidate):
    return PickOutcome(success=True, plan_time_s=0.5)


def slot_counter():
    """Hands out a distinct slot object per call, like the real allocator."""
    n = {"i": 0}

    def alloc(item):
        n["i"] += 1
        return type("Slot", (), {"slot_id": n["i"]})()

    return alloc


# ---- Stage A: everything pickable, order decided by cost ---------------------------------------


def test_every_item_is_picked_exactly_once():
    items = make_items({"mug_00": (0.3, 0, 0), "cup_00": (0.1, 0, 0), "tumbler_00": (0.2, 0, 0)})
    seq = TaskSequencer(items, StubMotion(), pick_place=always_succeeds,
                        grasp_fn=grasp_at_object, slot_fn=slot_counter(), cost_fn="nearest_first")
    res = seq.run(episode_id="ep0", seed=0)
    assert res.status == "cleared"
    assert sorted(p.item_id for p in res.picks) == ["cup_00", "mug_00", "tumbler_00"]
    assert len(res.picks) == 3 and res.n_placed == 3
    assert res.unplaced == []


def test_nearest_first_orders_by_distance():
    items = make_items({"mug_00": (0.3, 0, 0), "cup_00": (0.1, 0, 0), "tumbler_00": (0.2, 0, 0)})
    seq = TaskSequencer(items, StubMotion(), pick_place=always_succeeds,
                        grasp_fn=grasp_at_object, slot_fn=slot_counter(), cost_fn="nearest_first")
    res = seq.run()
    assert [p.item_id for p in res.picks] == ["cup_00", "tumbler_00", "mug_00"]


def test_swapping_the_cost_function_changes_the_order_and_nothing_else():
    """The ordering layer is swappable: same scene, same outcome, different sequence."""
    spec = {"mug_00": (0.3, 0, 0), "cup_00": (0.1, 0, 0), "tumbler_00": (0.2, 0, 0)}
    near = TaskSequencer(make_items(spec), StubMotion(), pick_place=always_succeeds,
                         grasp_fn=grasp_at_object, slot_fn=slot_counter(),
                         cost_fn="nearest_first").run()
    far = TaskSequencer(make_items(spec), StubMotion(), pick_place=always_succeeds,
                        grasp_fn=grasp_at_object, slot_fn=slot_counter(),
                        cost_fn="farthest_first").run()
    assert [p.item_id for p in near.picks] == ["cup_00", "tumbler_00", "mug_00"]
    assert [p.item_id for p in far.picks] == ["mug_00", "tumbler_00", "cup_00"]
    assert near.status == far.status == "cleared"
    assert near.cost_fn == "nearest_first" and far.cost_fn == "farthest_first"


def test_a_failed_pick_costs_one_object_not_the_episode():
    items = make_items({"mug_00": (0.1, 0, 0), "cup_00": (0.2, 0, 0), "tumbler_00": (0.3, 0, 0)})

    def fails_the_cup(item, candidate):
        if item.object_class == "cup":
            return PickOutcome(False, "pick-grasp-fault", "pad force out of band")
        return PickOutcome(True)

    res = TaskSequencer(items, StubMotion(), pick_place=fails_the_cup, grasp_fn=grasp_at_object,
                        slot_fn=slot_counter()).run()
    assert res.status == "partial"
    assert len(res.picks) == 3 and res.n_placed == 2
    assert res.unplaced == ["cup_00"]
    assert [p.failure_stage for p in res.picks if not p.success] == ["pick-grasp-fault"]


def test_running_out_of_slots_is_recorded_not_crashed():
    items = make_items({"mug_00": (0.1, 0, 0), "cup_00": (0.2, 0, 0)})
    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp_at_object,
                        slot_fn=lambda item: None).run()
    assert res.status == "partial"
    assert all(p.failure_stage == "no-goal-config" for p in res.picks)


def test_the_episode_record_captures_order_cost_and_candidate_count():
    items = make_items({"mug_00": (0.3, 0, 0), "cup_00": (0.1, 0, 0)})
    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp_at_object,
                        slot_fn=slot_counter()).run(episode_id="ep7", seed=42)
    doc = res.to_json()
    assert doc["episode_id"] == "ep7" and doc["seed"] == 42
    assert doc["pick_order"] == ["cup_00", "mug_00"]
    assert [p["order"] for p in doc["picks"]] == [0, 1]
    assert doc["picks"][0]["n_candidates"] == 2 and doc["picks"][1]["n_candidates"] == 1
    assert doc["picks"][0]["cost"] == pytest.approx(0.1)


# ---- Stage B: the support graph gates pickability -----------------------------------------------


def test_an_object_under_another_is_not_pickable():
    items = make_items({"plate_00": (0.1, 0, 0), "mug_00": (0.1, 0, 0.05)})
    # mug rests on plate
    support = lambda remaining: {"plate_00": {"mug_00"}} if any(i.item_id == "mug_00" for i in remaining) else {}
    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp_at_object,
                        slot_fn=slot_counter(), support_fn=support).run()
    assert [p.item_id for p in res.picks] == ["mug_00", "plate_00"], "must clear top-down"
    assert res.status == "cleared"


def test_a_stack_is_cleared_strictly_top_down():
    items = make_items({"a": (0, 0, 0), "b": (0, 0, 0.05), "c": (0, 0, 0.10)})
    # c on b on a; the graph is rebuilt from whatever is still remaining
    def support(remaining):
        ids = {i.item_id for i in remaining}
        g = {}
        if "b" in ids and "c" in ids:
            g["b"] = {"c"}
        if "a" in ids and "b" in ids:
            g["a"] = {"b"}
        return g

    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp_at_object,
                        slot_fn=slot_counter(), support_fn=support).run()
    assert [p.item_id for p in res.picks] == ["c", "b", "a"]


def test_the_support_graph_is_rebuilt_between_picks_not_cached():
    """Objects shift when neighbours are removed; planning on a stale graph is the bug."""
    calls = {"n": 0}
    items = make_items({"a": (0, 0, 0), "b": (0, 0, 0.05)})

    def support(remaining):
        calls["n"] += 1
        ids = {i.item_id for i in remaining}
        return {"a": {"b"}} if {"a", "b"} <= ids else {}

    TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp_at_object,
                  slot_fn=slot_counter(), support_fn=support).run()
    assert calls["n"] == 2, "support graph must be rebuilt before every pick"


def test_measured_poses_are_re_read_before_every_pick():
    items = make_items({"a": (0.5, 0, 0), "b": (0.2, 0, 0)})
    moved = {"n": 0}

    def refresh():
        moved["n"] += 1
        # after the first pick 'a' has shifted much closer than it was at spawn
        T = np.eye(4)
        T[:3, 3] = (0.01, 0, 0)
        return {"a": T} if moved["n"] > 1 else {}

    seen = []

    def record_pose(item, candidate):
        seen.append((item.item_id, float(item.T_base_obj[0, 3])))
        return PickOutcome(True)

    TaskSequencer(items, StubMotion(), pick_place=record_pose, grasp_fn=grasp_at_object,
                  slot_fn=slot_counter(), refresh_fn=refresh).run()
    assert moved["n"] == 2
    assert seen == [("b", 0.2), ("a", 0.01)], "the second pick must use the re-read pose"


# ---- Stage C: unpickable objects, recovery, deadlock --------------------------------------------


def test_an_object_with_no_grasp_is_not_pickable_even_with_nothing_on_top():
    items = make_items({"mug_00": (0.1, 0, 0), "cup_00": (0.2, 0, 0)})
    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds,
                        grasp_fn=lambda it: None if it.item_id == "cup_00" else grasp_at_object(it),
                        slot_fn=slot_counter()).run()
    assert res.status == "deadlock"
    assert [p.item_id for p in res.picks] == ["mug_00"]
    assert res.unplaced == ["cup_00"]
    assert "no collision-free grasp" in res.blocked_reasons["cup_00"]


def test_deadlock_names_every_remaining_object_and_its_reason():
    items = make_items({"a": (0.1, 0, 0), "b": (0.2, 0, 0)})
    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds,
                        grasp_fn=lambda it: None, slot_fn=slot_counter()).run()
    assert res.status == "deadlock"
    assert set(res.blocked_reasons) == {"a", "b"}
    assert res.picks == [] and sorted(res.unplaced) == ["a", "b"]
    assert "none is pickable" in res.detail


def test_recovery_is_tried_before_declaring_deadlock_and_is_capped():
    items = make_items({"a": (0.1, 0, 0)})
    attempts = {"n": 0}

    def recovery(seq, remaining):
        attempts["n"] += 1
        return True  # claims progress but never actually unblocks anything

    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=lambda it: None,
                        slot_fn=slot_counter(), recovery=recovery, max_recoveries=2).run()
    assert attempts["n"] == 2, "recovery must stop at the cap rather than spin forever"
    assert res.status == "deadlock" and res.n_recoveries == 2


def test_successful_recovery_lets_the_episode_finish():
    items = make_items({"a": (0.1, 0, 0)})
    unblocked = {"yes": False}

    def grasp(item):
        return grasp_at_object(item) if unblocked["yes"] else None

    def recovery(seq, remaining):
        unblocked["yes"] = True
        return True

    res = TaskSequencer(items, StubMotion(), pick_place=always_succeeds, grasp_fn=grasp,
                        slot_fn=slot_counter(), recovery=recovery, max_recoveries=2).run()
    assert res.status == "cleared" and res.n_recoveries == 1
    assert [p.item_id for p in res.picks] == ["a"]


def test_ties_break_deterministically():
    """Same world state must always yield the same choice, or seeds stop reproducing episodes."""
    spec = {"b_00": (0.2, 0, 0), "a_00": (0.2, 0, 0)}  # identical cost
    orders = set()
    for _ in range(5):
        res = TaskSequencer(make_items(spec), StubMotion(), pick_place=always_succeeds,
                            grasp_fn=grasp_at_object, slot_fn=slot_counter()).run()
        orders.add(tuple(p.item_id for p in res.picks))
    assert orders == {("a_00", "b_00")}
