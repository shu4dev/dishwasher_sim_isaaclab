"""Budget and evidence gates for the multi-object experiment driver."""
import importlib.util
from pathlib import Path
import sys

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/experiment/frigidaire_initial_state_experiment.py'
spec = importlib.util.spec_from_file_location('initial_state_driver', SCRIPT)
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


def test_budget_deadlines_and_no_negative_remaining(monkeypatch):
    monkeypatch.setattr(driver.time, 'monotonic', lambda: 110.)
    budget = driver.Budget(20., started=100.)
    assert budget.remaining() == 10.
    assert budget.remaining(115.) == 5.
    assert budget.remaining(105.) == 0.
    assert budget.remaining(500.) == 10.


def test_process_timeout_retains_written_evidence(tmp_path):
    logfile = tmp_path/'run.log'
    result = driver.run_process([sys.executable, '-u', '-c',
        'import time; print("began"); time.sleep(10)'], logfile, .3)
    assert result['timed_out']
    assert 'began' in logfile.read_text()
    assert result['exit_code'] != 0


def test_successful_exit_alone_is_not_physics_result(tmp_path):
    result = driver.run_process([sys.executable, '-c', 'pass'], tmp_path/'run.log', 10)
    assert result == {'exit_code': 0, 'timed_out': False}
    assert not (tmp_path/'result.json').exists()


def test_counts_keep_type_and_rack_assignments():
    result = driver.counts([{'kind': 'mug', 'rack': 'LowerRack'},
                           {'kind': 'mug', 'rack': 'UpperRack'},
                           {'kind': 'dinner_plate', 'rack': 'UpperRack'}])
    assert result == {'total': 3, 'by_kind': {'mug': 2, 'dinner_plate': 1},
                      'by_rack': {'LowerRack': 1, 'UpperRack': 2}}
