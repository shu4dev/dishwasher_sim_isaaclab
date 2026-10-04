"""Host tests for measured-trial scheduling and contact evidence, without Kit."""
from copy import deepcopy
import math
import unittest

import numpy as np

from dishsim_frigidaire.random_pose_experiment import BudgetExpired, execute_trials
from dishsim_frigidaire.random_pose_runtime import (
    CONTACT_CAPACITY, ContactTracker, supported, surface_motion_bound,
)
from dishsim_frigidaire.random_poses import (
    CELLS, KINDS, RACKS, cell_generators, sample_pose,
)


class FakeClock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


class FakeReport:
    def __init__(self):
        self.trials = []

    def record_trial(self, trial):
        self.trials.append(deepcopy(trial))


class FakeBackend:
    def __init__(self, clock):
        self.clock = clock
        self.points = {kind: np.array([[-.04, -.03, -.02], [.05, .03, .07]]) for kind in KINDS}
        self.baseline_results = {rack: {"result": "PASS"} for rack in RACKS}
        self.baseline_calls = []
        self.trial_calls = []
        self.baseline_hook = None
        self.trial_hook = None

    def prepare_baseline(self, rack):
        self.baseline_calls.append(rack)
        if self.baseline_hook is not None:
            self.baseline_hook(rack)
        return deepcopy(self.baseline_results[rack])

    def evaluate_trial(self, trial):
        self.trial_calls.append(deepcopy(trial))
        if self.trial_hook is not None:
            return self.trial_hook(trial)
        self.clock.now += .1
        return {"outcome": "accepted", "final_pose": deepcopy(trial["sampled_pose"])}


class TrialExecutionTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.backend = FakeBackend(self.clock)
        self.report = FakeReport()
        self.domains = {"sampling_bounds_by_rack": {
            rack: {"lower_m": [-.2, -.2, 0.], "upper_m": [.2, .2, .3]}
            for rack in RACKS}}

    def run_trials(self, **kwargs):
        return execute_trials(self.backend, self.report, domains=self.domains,
                              clock=self.clock, progress=lambda message: None, **kwargs)

    def test_round_robin_records_each_raw_proposal_once(self):
        result = self.run_trials(samples_per_cell=3, seed=7)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(self.backend.baseline_calls, list(RACKS))
        self.assertEqual(len(self.report.trials), 18)
        actual = [(trial["kind"], trial["rack"], trial["sample_index"]) for trial in self.report.trials]
        self.assertEqual(actual, [(kind, rack, index) for index in range(3) for kind, rack in CELLS])
        self.assertTrue(all(trial["outcome"] == "accepted" for trial in self.report.trials))
        self.assertEqual(len({trial["trial_id"] for trial in self.report.trials}), 18)
        self.assertTrue(all(trial["wall_seconds"] >= 0 for trial in self.report.trials))

    def test_failed_baseline_skips_that_rack_without_failed_dish_trials(self):
        self.backend.baseline_results["LowerRack"] = {"result": "FAIL", "reason": "empty closure blocked"}
        result = self.run_trials(samples_per_cell=2)
        self.assertEqual(result["status"], "baseline_blocked")
        self.assertEqual(result["baselines"]["LowerRack"]["reason"], "empty closure blocked")
        self.assertEqual(len(self.report.trials), 6)
        self.assertTrue(all(trial["rack"] == "UpperRack" for trial in self.report.trials))
        self.assertTrue(all(trial["outcome"] == "accepted" for trial in self.report.trials))

    def test_both_failed_baselines_generate_no_proposals(self):
        self.backend.baseline_results = {rack: {"result": "FAIL"} for rack in RACKS}
        result = self.run_trials(samples_per_cell=20)
        self.assertEqual(result["status"], "baseline_blocked")
        self.assertEqual(self.backend.trial_calls, [])
        self.assertEqual(self.report.trials, [])

    def test_collision_rejections_consume_samples_without_redrawing(self):
        self.backend.trial_hook = lambda trial: {"outcome": "initial_collision"}
        self.run_trials(samples_per_cell=2, seed=42)
        self.assertEqual(len(self.report.trials), 12)
        streams = cell_generators(42)
        for trial in self.report.trials:
            cell = trial["kind"], trial["rack"]
            expected = sample_pose(streams[cell], self.backend.points[trial["kind"]],
                                   self.domains["sampling_bounds_by_rack"][trial["rack"]])
            self.assertEqual(trial["sampled_pose"], expected)
            self.assertEqual(trial["outcome"], "initial_collision")

    def original_trials(self, samples_per_cell=4, seed=31):
        clock, report = FakeClock(), FakeReport()
        backend = FakeBackend(clock)
        execute_trials(backend, report, domains=self.domains, samples_per_cell=samples_per_cell,
                       seed=seed, clock=clock, progress=lambda message: None)
        return report.trials

    def test_resume_preserves_every_original_draw_and_never_records_duplicates(self):
        original = self.original_trials()
        # Uneven histories across all independent cells expose accidental RNG
        # reseeding, advancing only uncompleted cells, or only skipping a prefix.
        completed = {row["trial_id"]: deepcopy(row) for index, row in enumerate(original)
                     if index % 3 == 0 or (row["kind"] == "mug" and row["sample_index"] < 2)}
        for index, row in enumerate(completed.values()):
            if index % 2 == 0:
                row["outcome"] = "initial_collision"
                row["final_pose"] = None
        self.report.trials = deepcopy(list(completed.values()))
        result = self.run_trials(samples_per_cell=4, seed=31, completed_trials=completed)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(self.backend.baseline_calls, list(RACKS))
        self.assertEqual(len(self.backend.trial_calls), len(original)-len(completed))
        self.assertEqual(len(self.report.trials), len(original))
        self.assertEqual(len({row["trial_id"] for row in self.report.trials}), len(original))
        expected = {row["trial_id"]: row for row in original}
        for row in self.report.trials:
            self.assertEqual(row["sampled_pose"], expected[row["trial_id"]]["sampled_pose"])
        called_ids = {row["trial_id"] for row in self.backend.trial_calls}
        self.assertFalse(called_ids.intersection(completed))
        for row in self.report.trials:
            if row["trial_id"] in completed:
                self.assertEqual(row, completed[row["trial_id"]])

    def test_resume_retries_omitted_interrupted_proposal_with_identical_pose(self):
        original = self.original_trials(samples_per_cell=2)
        interrupted = deepcopy(original[7])
        interrupted["outcome"] = "interrupted"
        interrupted["final_pose"] = None
        # The resume loader deliberately excludes this unresolved row.
        completed = {row["trial_id"]: deepcopy(row) for row in original[:7]}
        self.report.trials = deepcopy(list(completed.values()))
        self.run_trials(samples_per_cell=2, seed=31, completed_trials=completed)
        self.assertEqual(self.backend.trial_calls[0]["trial_id"], interrupted["trial_id"])
        self.assertEqual(self.backend.trial_calls[0]["sampled_pose"], interrupted["sampled_pose"])
        retried = [row for row in self.report.trials if row["trial_id"] == interrupted["trial_id"]]
        self.assertEqual(len(retried), 1)
        self.assertEqual(retried[0]["outcome"], "accepted")
        self.assertEqual(len(self.report.trials), 12)

    def test_resume_of_fully_classified_run_only_rechecks_baselines(self):
        original = self.original_trials(samples_per_cell=1)
        completed = {row["trial_id"]: row for row in original}
        self.report.trials = deepcopy(original)
        result = self.run_trials(samples_per_cell=1, seed=31, completed_trials=completed)
        self.assertEqual(result["status"], "complete")
        self.assertEqual(self.backend.baseline_calls, list(RACKS))
        self.assertEqual(self.backend.trial_calls, [])
        self.assertEqual(self.report.trials, original)

    def test_resume_rejects_changed_sampled_pose_without_new_physics(self):
        original = self.original_trials(samples_per_cell=1)
        for key, delta in (("position_m", 2e-12), ("quaternion_xyzw", 1e-4), ("bbox_center_m", 1e-4)):
            with self.subTest(key=key):
                completed = {original[0]["trial_id"]: deepcopy(original[0])}
                completed[original[0]["trial_id"]]["sampled_pose"][key][0] += delta
                with self.assertRaisesRegex(ValueError, "sampled pose mismatch"):
                    self.run_trials(samples_per_cell=1, seed=31, completed_trials=completed)
        self.assertEqual(self.backend.trial_calls, [])
        self.assertEqual(self.report.trials, [])

    def test_resume_rechecks_rack_baselines_without_changing_other_streams(self):
        original = self.original_trials(samples_per_cell=3)
        completed = {row["trial_id"]: deepcopy(row) for row in original[:8]}
        self.backend.baseline_results["LowerRack"] = {"result": "FAIL"}
        self.report.trials = deepcopy(list(completed.values()))
        result = self.run_trials(samples_per_cell=3, seed=31, completed_trials=completed)
        self.assertEqual(result["status"], "baseline_blocked")
        expected = {row["trial_id"]: row for row in original}
        self.assertTrue(self.backend.trial_calls)
        for row in self.backend.trial_calls:
            self.assertEqual(row["rack"], "UpperRack")
            self.assertNotIn(row["trial_id"], completed)
            self.assertEqual(row["sampled_pose"], expected[row["trial_id"]]["sampled_pose"])

    def test_expired_deadline_before_first_baseline_has_no_attempts(self):
        result = self.run_trials(samples_per_cell=1, deadline=0.)
        self.assertEqual(result["status"], "budget_exhausted")
        self.assertEqual(self.backend.baseline_calls, [])
        self.assertEqual(self.report.trials, [])
        self.assertTrue(all(baseline["result"] == "NOT_RUN" for baseline in result["baselines"].values()))

    def test_deadline_after_baselines_does_not_generate_first_trial(self):
        def advance(rack):
            if rack == "UpperRack":
                self.clock.now = 10.
        self.backend.baseline_hook = advance
        result = self.run_trials(samples_per_cell=1, deadline=10.)
        self.assertEqual(result["status"], "budget_exhausted")
        self.assertEqual(self.backend.baseline_calls, list(RACKS))
        self.assertEqual(self.backend.trial_calls, [])
        self.assertEqual(self.report.trials, [])

    def test_deadline_between_trials_does_not_count_an_extra_proposal(self):
        def finish_at_deadline(trial):
            self.clock.now = 3.
            return {"outcome": "accepted"}
        self.backend.trial_hook = finish_at_deadline
        result = self.run_trials(samples_per_cell=2, deadline=3.)
        self.assertEqual(result["status"], "budget_exhausted")
        self.assertEqual(len(self.backend.trial_calls), 1)
        self.assertEqual(len(self.report.trials), 1)
        self.assertEqual(self.report.trials[0]["outcome"], "accepted")

    def test_midtrial_timeout_is_interrupted_and_preserves_observations(self):
        def expire(trial):
            trial["settled_pose"] = {"position_m": [0, 0, .1], "quaternion_xyzw": [0, 0, 0, 1]}
            trial["contacts"] = {"peak_penetration_m": .0002}
            self.clock.now = 4.
            raise BudgetExpired("deadline during rack retraction")
        self.backend.trial_hook = expire
        result = self.run_trials(samples_per_cell=5, deadline=4.)
        self.assertEqual(result["status"], "budget_exhausted")
        self.assertEqual(len(self.report.trials), 1)
        trial = self.report.trials[0]
        self.assertEqual(trial["outcome"], "interrupted")
        self.assertEqual(trial["settled_pose"]["position_m"], [0, 0, .1])
        self.assertEqual(trial["contacts"]["peak_penetration_m"], .0002)
        self.assertIsNone(trial["final_pose"])
        self.assertEqual(trial["wall_seconds"], 4.)

    def test_baseline_timeout_does_not_create_a_dish_trial(self):
        def expire(rack):
            raise BudgetExpired("deadline while checking empty rack")
        self.backend.baseline_hook = expire
        result = self.run_trials(samples_per_cell=1)
        self.assertEqual(result["status"], "budget_exhausted")
        self.assertEqual(self.report.trials, [])

    def test_runtime_exception_counts_error_and_stops_shared_simulation(self):
        def fail(trial):
            trial["phase"] = "closure"
            raise RuntimeError("invalid sensor buffer")
        self.backend.trial_hook = fail
        result = self.run_trials(samples_per_cell=2)
        self.assertEqual(result["status"], "error")
        self.assertEqual(len(self.report.trials), 1)
        self.assertEqual(self.report.trials[0]["outcome"], "simulation_error")
        self.assertEqual(self.report.trials[0]["phase"], "closure")
        self.assertIn("invalid sensor buffer", self.report.trials[0]["reason"])

    def test_baseline_sensor_exception_stops_all_trials(self):
        def fail(rack):
            raise RuntimeError("missing contact data")
        self.backend.baseline_hook = fail
        result = self.run_trials(samples_per_cell=1)
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["baselines"]["LowerRack"]["result"], "ERROR")
        self.assertEqual(result["baselines"]["UpperRack"]["result"], "NOT_RUN")
        self.assertEqual(self.report.trials, [])

    def test_user_interruption_preserves_active_trial_as_unresolved(self):
        def interrupt(trial):
            raise KeyboardInterrupt()
        self.backend.trial_hook = interrupt
        result = self.run_trials(samples_per_cell=2)
        self.assertEqual(result["status"], "interrupted")
        self.assertEqual(len(self.report.trials), 1)
        self.assertEqual(self.report.trials[0]["outcome"], "interrupted")


class ContactEvidenceTests(unittest.TestCase):
    names = ("mug", "LowerRack", "UpperRack", "SilverwareBasket")

    def setUp(self):
        self.radii = {name: .2 for name in self.names}
        self.tracker = ContactTracker(self.names, self.radii)
        self.poses = {name: {"position_m": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
                      for name in self.names}

    def contacts(self, first=None, second=None, separation=-.0005):
        size = len(self.names)
        counts = np.zeros((size, size), dtype=np.int64)
        starts = np.zeros_like(counts)
        count = 0 if first is None else 1
        if count:
            counts[self.names.index(first), self.names.index(second)] = 1
        return [np.zeros((count, 1)), np.zeros((count, 3)), np.zeros((count, 3)),
                np.full((count, 1), separation), counts, starts]

    @staticmethod
    def pairs(*pairs):
        return {tuple(sorted(pair)) for pair in pairs}

    def test_support_requires_selected_rack_or_seated_lower_basket_chain(self):
        lower = self.pairs(("mug", "LowerRack"))
        upper = self.pairs(("mug", "UpperRack"))
        self.assertTrue(supported(lower, "mug", "LowerRack"))
        self.assertFalse(supported(lower, "mug", "UpperRack"))
        self.assertTrue(supported(upper, "mug", "UpperRack"))
        self.assertFalse(supported(upper, "mug", "LowerRack"))
        basket_only = self.pairs(("mug", "SilverwareBasket"))
        chain = basket_only | self.pairs(("SilverwareBasket", "LowerRack"))
        self.assertFalse(supported(basket_only, "mug", "LowerRack"))
        self.assertTrue(supported(chain, "mug", "LowerRack"))
        self.assertFalse(supported(chain, "mug", "UpperRack"))
        self.assertFalse(supported(self.pairs(("mug", "Cabinet")), "mug", "LowerRack"))

    def test_sleeping_contacts_retain_witnessed_support_and_depth(self):
        observed = self.tracker.update(self.contacts("mug", "LowerRack"), self.poses)
        self.assertTrue(supported(observed["pairs"], "mug", "LowerRack"))
        self.assertEqual(observed["peak_m"], .0005)
        sleeping = self.tracker.update(self.contacts(), deepcopy(self.poses))
        self.assertEqual(sleeping["count"], 0)
        self.assertEqual(sleeping["pairs"], observed["pairs"])
        self.assertEqual(sleeping["peak_m"], observed["peak_m"])
        self.tracker.clear()
        self.assertEqual(self.tracker.update(self.contacts(), self.poses)["pairs"], set())

    def test_sleeping_support_expires_after_either_surface_moves(self):
        for moved_body in ("mug", "LowerRack"):
            with self.subTest(moved_body=moved_body):
                self.tracker.clear()
                self.tracker.update(self.contacts("mug", "LowerRack"), deepcopy(self.poses))
                near = deepcopy(self.poses)
                near[moved_body]["position_m"][0] = .7e-6
                self.assertTrue(self.tracker.update(self.contacts(), near)["pairs"])
                far = deepcopy(self.poses)
                far[moved_body]["position_m"][0] = 1.4e-6
                self.assertEqual(self.tracker.update(self.contacts(), far)["pairs"], set())

    def test_quaternion_sign_is_unchanged_but_actual_rotation_expires_support(self):
        self.tracker.update(self.contacts("mug", "LowerRack"), deepcopy(self.poses))
        flipped = deepcopy(self.poses)
        flipped["mug"]["quaternion_xyzw"] = [0., 0., 0., -1.]
        self.assertEqual(surface_motion_bound(self.poses["mug"], flipped["mug"], .2), 0.)
        self.assertTrue(self.tracker.update(self.contacts(), flipped)["pairs"])
        rotated = deepcopy(self.poses)
        rotated["mug"]["quaternion_xyzw"] = [0., math.sin(.0005), 0., math.cos(.0005)]
        self.assertEqual(self.tracker.update(self.contacts(), rotated)["pairs"], set())

    def test_fresh_contact_after_motion_restores_support(self):
        self.tracker.update(self.contacts("mug", "LowerRack"), deepcopy(self.poses))
        moved = deepcopy(self.poses)
        moved["mug"]["position_m"][2] = .01
        observed = self.tracker.update(self.contacts("LowerRack", "mug", -.0008), moved)
        self.assertTrue(supported(observed["pairs"], "mug", "LowerRack"))
        self.assertEqual(observed["peak_m"], .0008)

    def test_self_contacts_do_not_establish_support(self):
        observed = self.tracker.update(self.contacts("mug", "mug"), self.poses)
        self.assertEqual(observed["pairs"], set())
        self.assertEqual(observed["peak_m"], 0.)
        self.assertFalse(supported(observed["pairs"], "mug", "LowerRack"))

    def test_positive_separation_is_not_penetration(self):
        observed = self.tracker.update(self.contacts("mug", "LowerRack", .0001), self.poses)
        self.assertEqual(observed["peak_m"], 0.)

    def test_nonfinite_active_contact_values_fail_closed(self):
        for buffer_index in range(4):
            for nonfinite in (float("nan"), float("inf")):
                with self.subTest(buffer=buffer_index, value=nonfinite):
                    data = self.contacts("mug", "LowerRack")
                    data[buffer_index][0, 0] = nonfinite
                    with self.assertRaisesRegex(RuntimeError, "Nonfinite"):
                        self.tracker.update(data, self.poses)

    def test_nonfinite_unused_buffer_capacity_does_not_invent_error(self):
        data = self.contacts()
        data[:4] = [np.full((3, 1), np.nan), np.full((3, 3), np.nan),
                    np.full((3, 3), np.nan), np.full((3, 1), np.nan)]
        self.assertEqual(self.tracker.update(data, self.poses)["pairs"], set())

    def test_contact_capacity_or_slice_overflow_fails_closed(self):
        data = self.contacts()
        data[4][0, 1] = CONTACT_CAPACITY
        with self.assertRaisesRegex(RuntimeError, "overflow"):
            self.tracker.update(data, self.poses)
        data = self.contacts("mug", "LowerRack")
        data[5][0, 1] = 1  # one-past-end start for a one-element buffer
        with self.assertRaisesRegex(RuntimeError, "overflow"):
            self.tracker.update(data, self.poses)

    def test_invalid_counts_indices_and_missing_buffers_fail_closed(self):
        for index, value in ((4, -.1), (4, .5), (4, np.nan), (5, -.1), (5, .5), (5, np.inf)):
            with self.subTest(index=index, value=value):
                data = self.contacts("mug", "LowerRack")
                data[index] = data[index].astype(float)
                data[index][0, 1] = value
                with self.assertRaisesRegex(RuntimeError, "Invalid"):
                    self.tracker.update(data, self.poses)
        for data in (None, [], [0]*5):
            with self.assertRaisesRegex(RuntimeError, "unavailable"):
                self.tracker.update(data, self.poses)
        data = self.contacts()
        data[4] = np.zeros((2, 2))
        with self.assertRaisesRegex(RuntimeError, "Invalid"):
            self.tracker.update(data, self.poses)


if __name__ == "__main__":
    unittest.main()
