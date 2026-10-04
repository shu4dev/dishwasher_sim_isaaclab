"""Trial scheduling for the random-pose experiment, without importing Isaac.

The backend owns physics; this module owns proposal counts and interruption semantics.
An unavailable baseline is not evidence that a dish pose is impossible.
"""
from __future__ import annotations

import time

import numpy as np

from .random_poses import RACKS, cell_generators, round_robin_cells, sample_pose


class BudgetExpired(TimeoutError):
    """The experiment's wall-clock budget expired, including an active trial."""


def check_deadline(deadline, clock=time.monotonic):
    if clock() >= deadline:
        raise BudgetExpired("Experiment wall-time limit reached")


def execute_trials(backend, report, *, domains, samples_per_cell=100, seed=0,
                   deadline=float("inf"), clock=time.monotonic, progress=print,
                   completed_trials=None):
    """Run raw proposals round-robin; return status and all baseline observations.

    Backend protocol: ``points[kind]``, ``prepare_baseline(rack) -> report`` and
    ``evaluate_trial(trial) -> fields``. The latter may update trial in place so
    measurements survive a timeout or exception. Physics steps must also check
    the same deadline, not merely this outer loop.

    ``completed_trials`` maps trial IDs to externally validated classified rows
    already copied into ``report``. Replay their random draws and verify sampled
    poses, but never simulate or record them twice. Omit unresolved prior rows
    from the mapping to retry those same proposals. Baselines always run again.
    """
    completed_trials = {} if completed_trials is None else completed_trials
    generators = cell_generators(seed)
    baselines = {rack: {"result": "NOT_RUN"} for rack in RACKS}
    result = {"status": "complete", "baselines": baselines}
    for rack in RACKS:
        try:
            check_deadline(deadline, clock)
            baselines[rack] = backend.prepare_baseline(rack)
            if baselines[rack].get("result") not in {"PASS", "FAIL", "ERROR"}:
                raise ValueError("Backend returned an invalid baseline result")
            progress("[BASELINE] {}: {}".format(rack, baselines[rack]["result"]))
            if baselines[rack]["result"] == "ERROR":
                result.update(status="error", error=baselines[rack].get("error", "Baseline runtime error"))
                return result
        except BudgetExpired as exc:
            result.update(status="budget_exhausted", error=str(exc))
            return result
        except KeyboardInterrupt:
            result.update(status="interrupted")
            return result
        except Exception as exc:
            baselines[rack] = {"result": "ERROR", "error": repr(exc)}
            # Runtime/sensor failures invalidate the shared simulation, unlike a
            # measured FAIL of only one rack's empty closure.
            result.update(status="error", error=repr(exc))
            return result

    skipped = {rack for rack in RACKS if baselines[rack]["result"] != "PASS"}
    if skipped:
        result["status"] = "baseline_blocked"
    for kind, rack, index in round_robin_cells(samples_per_cell):
        try:
            check_deadline(deadline, clock)
        except BudgetExpired:
            result["status"] = "budget_exhausted"
            break
        trial = {"trial_id": "{}_{}_{:05d}".format(kind, rack, index),
                 "kind": kind, "rack": rack, "sample_index": index,
                 "sampled_pose": sample_pose(generators[(kind, rack)], backend.points[kind],
                                             domains["sampling_bounds_by_rack"][rack]),
                 "settled_pose": None, "final_pose": None}
        previous = completed_trials.get(trial["trial_id"])
        if previous is not None:
            for key in ("position_m", "quaternion_xyzw", "bbox_center_m"):
                recorded = np.asarray(previous["sampled_pose"][key], dtype=float)
                generated = np.asarray(trial["sampled_pose"][key], dtype=float)
                if (recorded.shape != generated.shape
                        or not np.allclose(recorded, generated, atol=1e-12, rtol=0.)):
                    raise ValueError("Completed trial sampled pose mismatch: {} ({})".format(trial["trial_id"], key))
            continue
        if rack in skipped:
            continue
        started = clock()
        try:
            trial.update(backend.evaluate_trial(trial))
        except BudgetExpired as exc:
            trial.update(outcome="interrupted", reason=str(exc))
            result["status"] = "budget_exhausted"
        except KeyboardInterrupt:
            trial.update(outcome="interrupted", reason="User interrupted the active trial")
            result["status"] = "interrupted"
        except Exception as exc:
            trial.update(outcome="simulation_error", reason=repr(exc))
            result.update(status="error", error=repr(exc))
        trial["wall_seconds"] = max(0., clock()-started)
        report.record_trial(trial)
        progress("[TRIAL] {}: {}".format(trial["trial_id"], trial["outcome"]))
        if result["status"] in {"budget_exhausted", "interrupted", "error"}:
            break
    return result
