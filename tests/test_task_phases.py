# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Full-load phase machinery (task/phases.py).

The structural invariant worth its own test class: every transition target dict contains
EVERY rack joint of the machine. The scene-side rack override is replace-not-merge with a
fallback to the BUILD scenario's targets, so an incomplete dict silently drives an omitted
(loaded) rack back to its build-scenario extension — the latch-safety property is enforced
by shape, not convention.
"""

import json

import numpy as np
import pytest

from dishsim import config
from dishsim.task import phases


@pytest.fixture
def bosch():
    config.apply_machine("bosch800")
    config.apply_base_placement("side_winner")
    yield
    config.apply_machine(config.MACHINE_BASELINE_NAME)
    config.apply_scenario("both_out")  # apply_machine keeps a surviving scenario name


class TestTransitionTargets:
    def test_every_dict_contains_every_rack_joint(self, bosch):
        joints = set(phases.state_joint_targets("placement"))
        assert len(joints) == 3  # down, up, third
        for a, b in [("placement", "middle_out"), ("middle_out", "third_out"),
                     ("placement", "third_out"), ("third_out", "placement")]:
            for d in phases.transition_targets(a, b):
                assert set(d) == joints, (a, b, d)

    def test_stow_first_order(self, bosch):
        seq = phases.transition_targets("placement", "middle_out")
        assert len(seq) == 2
        # step 1: the loaded lower rack stows while the middle stays in
        assert seq[0]["PrismaticJoint_dishwasher_2_down"] == 0.0
        assert seq[0]["PrismaticJoint_dishwasher_2_up"] == 0.0
        # step 2: the middle rack extends
        assert seq[1]["PrismaticJoint_dishwasher_2_up"] == pytest.approx(-0.51)
        assert seq[1]["PrismaticJoint_dishwasher_2_down"] == 0.0

    def test_middle_to_third(self, bosch):
        seq = phases.transition_targets("middle_out", "third_out")
        assert len(seq) == 2
        assert seq[0]["PrismaticJoint_dishwasher_2_up"] == 0.0
        assert seq[0][config.RACK_THIRD_JOINT] == 0.0
        assert seq[1][config.RACK_THIRD_JOINT] == pytest.approx(-0.51)

    def test_identity_transition_is_a_single_noop_dict(self, bosch):
        seq = phases.transition_targets("placement", "placement")
        assert seq == [phases.state_joint_targets("placement")]

    def test_baseline_machine_has_two_joints(self):
        d = phases.state_joint_targets("placement")
        assert set(d) == {"PrismaticJoint_dishwasher_2_down", "PrismaticJoint_dishwasher_2_up"}


class TestParsePhasesArg:
    def test_round_trip(self, bosch):
        out = phases.parse_phases_arg("placement:plate=2,cup=1; third_out:fork=2")
        assert out == [("placement", {"plate": 2, "cup": 1}), ("third_out", {"fork": 2})]

    def test_rack_action_scenario_rejected(self, bosch):
        with pytest.raises(ValueError, match="rack action"):
            phases.parse_phases_arg("both_in:cup=1")

    def test_unknown_class_rejected(self, bosch):
        with pytest.raises(ValueError, match="unknown object class"):
            phases.parse_phases_arg("placement:goblet=1")

    def test_bare_class_counts_one(self, bosch):
        assert phases.parse_phases_arg("placement:cup") == [("placement", {"cup": 1})]

    def test_empty_spec_rejected(self, bosch):
        with pytest.raises(ValueError):
            phases.parse_phases_arg(" ; ")


def _items(*pairs):
    return [phases.PhaseItem(f"{c}_{i:02d}", c, i, i, f"s{i}", "floor_stand", "pick")
            for i, c in enumerate(pairs)]


class TestPlanWaves:
    RECT = {"x_min": 0.0, "x_max": 0.30, "y_min": 0.0, "y_max": 0.30}  # 0.09 m^2

    def test_area_budget_splits(self):
        # radius 0.05 + sep/2 = 0.0575 -> disc area ~0.0104; budget 0.45*0.09 = 0.0405
        # -> 3 discs (0.0312) fit, the 4th (0.0415) overflows -> waves of 3
        items = _items(*(["cup"] * 9))
        waves = phases.plan_waves(items, rect=self.RECT, min_separation_m=0.015,
                                  radius_fn=lambda c: 0.05, fill_factor=0.45)
        assert [len(w) for w in waves] == [3, 3, 3]
        assert [i.item_id for w in waves for i in w] == [i.item_id for i in items]  # order kept

    def test_max_wave_cap(self):
        items = _items(*(["cup"] * 5))
        waves = phases.plan_waves(items, rect=self.RECT, min_separation_m=0.015,
                                  radius_fn=lambda c: 0.01, fill_factor=0.45, max_wave=2)
        assert [len(w) for w in waves] == [2, 2, 1]

    def test_single_oversized_item_still_gets_a_wave(self):
        items = _items("plate")
        waves = phases.plan_waves(items, rect=self.RECT, min_separation_m=0.015,
                                  radius_fn=lambda c: 1.0, fill_factor=0.45)
        assert [len(w) for w in waves] == [1]

    def test_deterministic(self):
        items = _items("cup", "plate", "bowl", "cup")
        kw = dict(rect=self.RECT, min_separation_m=0.015,
                  radius_fn=lambda c: {"cup": 0.03, "plate": 0.08, "bowl": 0.07}[c],
                  fill_factor=0.45)
        assert [[i.item_id for i in w] for w in phases.plan_waves(items, **kw)] == \
            [[i.item_id for i in w] for w in phases.plan_waves(items, **kw)]


class TestLoadFullLoadPlan:
    def _doc(self, machine="bosch800", placement="side_winner", state="placement"):
        return {"machine": machine, "base_placement": placement,
                "phases": [{"state": state,
                            "items": [{"item_id": "cup_00", "object_class": "cup",
                                       "instance": 0, "slot_id": 3, "slot_name": "near",
                                       "mode": "floor_stand", "acquire": "pick",
                                       "T_base_slot": np.eye(4).tolist()}]}]}

    def test_loads(self, bosch, tmp_path):
        p = tmp_path / "plan.json"
        p.write_text(json.dumps(self._doc()))
        out = phases.load_full_load_plan(str(p), machine="bosch800",
                                         base_placement="side_winner")
        assert len(out) == 1 and out[0].state == "placement"
        assert out[0].items[0].item_id == "cup_00"
        assert out[0].items[0].T_base_slot.shape == (4, 4)

    def test_machine_mismatch_fails(self, bosch, tmp_path):
        p = tmp_path / "plan.json"
        p.write_text(json.dumps(self._doc(machine="artvip_compact")))
        with pytest.raises(SystemExit, match="re-run"):
            phases.load_full_load_plan(str(p), machine="bosch800",
                                       base_placement="side_winner")

    def test_placement_mismatch_fails(self, bosch, tmp_path):
        p = tmp_path / "plan.json"
        p.write_text(json.dumps(self._doc(placement="front")))
        with pytest.raises(SystemExit, match="re-run"):
            phases.load_full_load_plan(str(p), machine="bosch800",
                                       base_placement="side_winner")

    def test_rack_action_phase_state_fails(self, bosch, tmp_path):
        p = tmp_path / "plan.json"
        p.write_text(json.dumps(self._doc(state="both_in")))
        with pytest.raises(SystemExit, match="rack action"):
            phases.load_full_load_plan(str(p), machine="bosch800",
                                       base_placement="side_winner")

    def test_empty_plan_fails(self, bosch, tmp_path):
        p = tmp_path / "plan.json"
        doc = self._doc()
        doc["phases"][0]["items"] = []
        p.write_text(json.dumps(doc))
        with pytest.raises(SystemExit, match="no items"):
            phases.load_full_load_plan(str(p), machine="bosch800",
                                       base_placement="side_winner")
