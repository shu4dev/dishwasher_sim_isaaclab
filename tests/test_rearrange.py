# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The rearrangement episode driver + greedy baseline, through a 1-D toy oracle (no Kit/FCL).

``run_episode`` never touches slot JSON itself — only the oracle's ``at_goal`` does — which is
what lets a toy oracle exercise the entire Kit-free surface: greedy pass-1/pass-2/buffer
logic, the FCL pre-check path, budget and fault aborts, and the bookkeeping.
"""

import numpy as np

from dishsim.rearrange import Greedy, Instance, run_episode
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


class ToyOracle:
    """Perfect physics (settled == commanded), with optional scripted fault injection."""

    def __init__(self, instance, fault_at=None):
        self.targets = {it["item_id"]: it["target"]["T_base_obj"] for it in instance.items}
        self.poses = {it["item_id"]: np.asarray(it["T_base_init"]) for it in instance.items}
        self.fault_at, self.n = fault_at, 0

    def at_goal(self, item, T_obj):
        return abs(np.asarray(T_obj)[0, 3] - self.targets[item["item_id"]][0, 3]) < 1e-6

    def execute(self, move):
        self.n += 1
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
