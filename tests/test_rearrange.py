# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The rearrangement episode driver + greedy baseline, through a 1-D toy oracle (no Kit/FCL).

``run_episode`` never touches slot JSON itself — only the oracle's ``at_goal`` does — which is
what lets a toy oracle exercise the entire Kit-free surface: greedy pass-1/pass-2/buffer
logic, the FCL pre-check path, budget and fault aborts, and the bookkeeping.
"""

import numpy as np

from dishsim.rearrange import MAX_CONSEC_REFUSALS, Greedy, Instance, Move, run_episode
from dishsim.transforms import make_T


def T(x):
    return make_T((float(x), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class ToyWorld:
    """1-D: a commanded pose collides when any OTHER item sits within 0.5 of its x."""

    def __init__(self):
        self.poses = {}

    def clear(self):
        self.poses = {}

    def sync(self, poses, classes):
        self.poses.update({k: np.asarray(v) for k, v in poses.items()})

    def blockers(self, item_id, T_cmd, object_class=None):
        x = np.asarray(T_cmd)[0, 3]
        return sorted(k for k, p in self.poses.items()
                      if k != item_id and abs(p[0, 3] - x) < 0.5)

    def move_collides(self, item_id, T_cmd, object_class=None):
        return bool(self.blockers(item_id, T_cmd))

    def buffer_poses(self, object_class):
        return [T(100.0), T(101.0)]

    def in_counter(self, T_cmd):
        # matches the buffer cells at x 100/101: everything past 90 is "the counter band"
        return bool(np.asarray(T_cmd)[0, 3] >= 90.0)


class ToyOracle:
    """Perfect physics (settled == commanded), with optional scripted fault injection.

    ``fail_settle_at=k`` makes executed move number ``k`` a NON-fatal failed settle: per the
    teleport-back contract the move is NOT applied and the pre-move poses come back.
    """

    def __init__(self, instance, fault_at=None, fail_settle_at=None):
        self.targets = {it["item_id"]: it["target"]["T_base_obj"] for it in instance.items}
        self.poses = {it["item_id"]: np.asarray(it["T_base_init"]) for it in instance.items}
        self.fault_at, self.fail_settle_at, self.n = fault_at, fail_settle_at, 0

    def at_goal(self, item, T_obj):
        return abs(np.asarray(T_obj)[0, 3] - self.targets[item["item_id"]][0, 3]) < 1e-6

    def execute(self, move):
        self.n += 1
        if self.n == self.fail_settle_at:
            return dict(self.poses), "failed-settle", {"settle_dev_mm": 99.0}
        self.poses[move.item_id] = np.asarray(move.T_base_obj)
        fault = "disturbed" if self.n == self.fault_at else None
        return dict(self.poses), fault, {}


def swap_instance():
    """The canonical blocked case: A and B must trade cells."""
    items = [{"item_id": "A", "object_class": "toy", "T_base_init": T(0),
              "target": {"T_base_obj": T(1)}},
             {"item_id": "B", "object_class": "toy", "T_base_init": T(1),
              "target": {"T_base_obj": T(0)}}]
    return Instance(name="swap", machine="toy", base_placement="toy", state="toy",
                    items=items, meta={})


def test_greedy_solves_the_swap_via_the_buffer():
    inst = swap_instance()
    rec = run_episode(inst, Greedy(), ToyWorld(), ToyOracle(inst), budget=10,
                      algorithm_name="greedy")
    assert rec["solved"] and rec["abort"] is None
    assert rec["moves_used"] == 3  # B -> buffer, A -> home, B -> home
    assert rec["at_goal_initial"] == 0 and rec["fraction_at_goal"] == 1.0
    assert len(rec["planning_time_s"]) == 3  # one planning call per executed move


def test_budget_aborts_unsolved():
    inst = swap_instance()
    rec = run_episode(inst, Greedy(), ToyWorld(), ToyOracle(inst), budget=1)
    assert not rec["solved"] and rec["abort"] == "budget"
    assert rec["moves_used"] == 1


def test_injected_fault_aborts_with_state():
    inst = swap_instance()
    rec = run_episode(inst, Greedy(), ToyWorld(), ToyOracle(inst, fault_at=2), budget=10)
    assert not rec["solved"] and rec["abort"] == "disturbed"
    assert rec["moves_used"] == 2 and len(rec["moves"]) == 2


class Scripted:
    """Replays a fixed move list; None when exhausted. Captures every obs it saw."""

    def __init__(self, moves):
        self.moves, self.seen = list(moves), []

    def reset(self, instance, world):
        pass

    def next_move(self, obs):
        self.seen.append({"counter_cap": obs.get("counter_cap"),
                          "counter_count": obs.get("counter_count")})
        return self.moves.pop(0) if self.moves else None


def test_counter_full_refusal_is_logged_and_nonfatal():
    inst = swap_instance()
    algo = Scripted([Move("A", T(100.0))])  # park on the counter under cap 0
    rec = run_episode(inst, algo, ToyWorld(), ToyOracle(inst), budget=10, counter_cap=0)
    assert rec["abort"] == "give-up"  # the refusal itself is non-fatal; the script ran dry
    assert rec["counter_full_refusals"] == 1 and rec["moves_used"] == 0
    assert rec["moves"][0]["kind"] == "counter-full"
    assert algo.seen[0] == {"counter_cap": 0, "counter_count": 0}


def test_greedy_gives_up_at_cap_instead_of_spamming():
    inst = swap_instance()
    rec = run_episode(inst, Greedy(), ToyWorld(), ToyOracle(inst), budget=10, counter_cap=0)
    assert rec["abort"] == "give-up"
    assert rec["counter_full_refusals"] == 0  # cap-aware: never even commands the park


def test_refusal_loop_guard():
    inst = swap_instance()

    class Spammer:
        def reset(self, instance, world):
            pass

        def next_move(self, obs):
            return Move("A", T(1.0))  # collides with B forever

    rec = run_episode(inst, Spammer(), ToyWorld(), ToyOracle(inst), budget=10)
    assert rec["abort"] == "refusal-loop"
    assert len(rec["moves"]) == MAX_CONSEC_REFUSALS and rec["moves_used"] == 0


def test_failed_settle_is_nonfatal_and_counted():
    inst = swap_instance()
    # greedy's move 2 is A -> home; it fails to settle once, greedy retries it next turn
    rec = run_episode(inst, Greedy(), ToyWorld(), ToyOracle(inst, fail_settle_at=2),
                      budget=10)
    assert rec["solved"] and rec["abort"] is None
    assert rec["moves_used"] == 4 and rec["failed_settles"] == 1
    assert rec["moves"][1]["kind"] == "failed-settle"
