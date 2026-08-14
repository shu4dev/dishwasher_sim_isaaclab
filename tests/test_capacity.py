# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reachable-capacity planner (dishsim/capacity.py).

The joint-certification loop needs real collision caches, so those paths run only when the
restored assets are present (they are gitignored); everything else — the z-budget gate that
decides the middle-rack class policy, the policy streams, serialization, the figure — is
Kit-free and cache-free.
"""

import json
import os

import numpy as np
import pytest

from dishsim import capacity, config, placement

BOSCH_PLACEMENT_CACHE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "assets", "cache", "machines", "bosch800", "objects", "cup", "placement", "slots")


@pytest.fixture
def bosch():
    config.apply_machine("bosch800")
    config.apply_base_placement("side_winner")
    yield
    config.apply_machine(config.MACHINE_BASELINE_NAME)
    config.apply_scenario("both_out")  # apply_machine keeps a surviving scenario name


class TestZBudget:
    """The measured transition-clearance gate (values from the 2026-08-14 z-budget table)."""

    def test_middle_rack_cup_clears_tumbler_does_not(self, bosch):
        cup = capacity.z_budget_clearance_m("cup", "middle_out", last_state=False)
        tumbler = capacity.z_budget_clearance_m("tumbler", "middle_out", last_state=False)
        # cup ~ +20.6 mm, tumbler ~ -15.8 mm against the third rack's underside
        assert cup is not None and cup >= capacity.Z_BUDGET_MARGIN_M
        assert tumbler is not None and tumbler < 0.0
        assert cup == pytest.approx(0.0206, abs=0.002)
        assert tumbler == pytest.approx(-0.0158, abs=0.002)

    def test_lower_rack_riders_all_clear(self, bosch):
        for cls in ("cup", "tumbler", "bowl", "plate"):
            clr = capacity.z_budget_clearance_m(cls, "placement", last_state=False)
            assert clr is not None and clr > 0.10, cls  # >= 113 mm measured

    def test_final_phase_has_no_gate(self, bosch):
        assert capacity.z_budget_clearance_m("fork", "third_out", last_state=True) is None

    def test_baseline_machine_has_no_rack_model(self):
        assert capacity.z_budget_clearance_m("cup", "placement", last_state=False) is None


class TestPolicyStreams:
    def test_round_robin_groups_and_filtering(self):
        streams = capacity._class_streams(("plate", ("cup", "tumbler")),
                                          ["plate", "cup", "tumbler"])
        assert streams == [["plate"], ["cup", "tumbler"]]
        # a class outside the pool drops out; an emptied group disappears
        assert capacity._class_streams(("plate", ("cup",)), ["cup"]) == [["cup"]]

    def test_capacity_classes_excludes_mug_on_bosch(self):
        config.apply_machine("bosch800")
        try:
            assert "mug" not in capacity.capacity_classes()
            assert set(capacity.capacity_classes()) == {"plate", "bowl", "cup", "tumbler",
                                                        "fork"}
        finally:
            config.apply_machine(config.MACHINE_BASELINE_NAME)
            config.apply_scenario("both_out")
        assert "mug" in capacity.capacity_classes()


def _fake_tables(state="placement"):
    def slot(sid, xyz):
        T = np.eye(4)
        T[:3, 3] = xyz
        return placement.SlotFrame(sid, T, 0.06, "derived")

    slots = {0: slot(0, (0.3, 0.0, 0.0)), 1: slot(1, (0.4, 0.0, 0.0))}
    return capacity._StateTables(
        state, "deadbeef", {"cup": slots},
        {"cup": {0: np.zeros((3, 6)), 1: np.zeros((0, 6))}},
        {"cup": {"near": 0, "far": 1}})


class TestSerialization:
    def _plan(self):
        item = capacity.PlannedPlacement(
            "cup_00", "cup", 0, "placement", 0, "near", "floor_stand", "lower",
            np.eye(4), np.eye(4), "pick", 12)
        phase = capacity.PhaseCapacity("placement", [item], {
            "cup": {"slots_total": 2, "reachable": 1, "assigned": 1, "stopped_by": None}})
        return capacity.CapacityPlan("bosch800", "side_winner", "plates_first",
                                     {"placement": "deadbeef"}, ["cup"], [phase])

    def test_round_trip(self, tmp_path):
        plan = self._plan()
        path = capacity.write_plan(plan, str(tmp_path / "plan.json"), extra={"figure": "x"})
        loaded = json.load(open(path))
        assert loaded["schema_version"] == capacity.CAPACITY_SCHEMA_VERSION
        assert loaded["total_items"] == 1
        assert loaded["figure"] == "x"
        it = loaded["phases"][0]["items"][0]
        assert it["item_id"] == "cup_00" and it["acquire"] == "pick"
        assert np.array(it["T_base_obj"]).shape == (4, 4)

    def test_counts_and_total(self):
        plan = self._plan()
        assert plan.total_items == 1
        assert plan.phases[0].counts() == {"cup": 1}

    def test_figure_smoke(self, tmp_path):
        plan = self._plan()
        out = capacity.render_reachability_figure(
            plan, {"placement": _fake_tables()}, str(tmp_path / "fig.png"))
        assert os.path.getsize(out) > 10_000


@pytest.mark.skipif(not os.path.isdir(BOSCH_PLACEMENT_CACHE),
                    reason="restored Bosch caches not present")
class TestAgainstRealCaches:
    def test_load_state_tables_validates_hashes(self, bosch):
        entry = config.SCENARIO_NAME
        config.apply_scenario("placement")
        try:
            tables = capacity.load_state_tables("placement", ["cup"])
            assert len(tables.slots["cup"]) == 56
            assert sum(1 for v in tables.goal_sets["cup"].values() if len(v)) > 30
        finally:
            config.apply_scenario(entry)

    def test_wrong_placement_is_rejected(self):
        config.apply_machine("bosch800")  # default placement: front, caches are side_winner
        try:
            config.apply_scenario("placement")
            with pytest.raises(RuntimeError, match="config_hash"):
                capacity.load_state_tables("placement", ["cup"])
        finally:
            config.apply_machine(config.MACHINE_BASELINE_NAME)
            config.apply_scenario("both_out")


class TestSettleReliabilityGate:
    """Measured drinkware verdict (2026-08-14 probes): the OEM lattice cannot stand the
    scaled cups/tumblers up dependably; bowls can. The gate is data, not policy."""

    def test_measured_values_gate_correctly(self):
        assert capacity.MEASURED_SETTLE_RELIABILITY[("placement", "bowl")] \
            >= capacity.SETTLE_RELIABILITY_BAR
        for pair in (("placement", "cup"), ("placement", "tumbler"), ("middle_out", "cup")):
            assert capacity.MEASURED_SETTLE_RELIABILITY[pair] < capacity.SETTLE_RELIABILITY_BAR

    def test_unmeasured_pairs_pass(self):
        assert ("third_out", "fork") not in capacity.MEASURED_SETTLE_RELIABILITY
        assert ("placement", "plate") not in capacity.MEASURED_SETTLE_RELIABILITY
