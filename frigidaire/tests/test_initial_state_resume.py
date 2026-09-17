"""Read-only restoration and scheduling guards for budget-preserving resumption."""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/experiment'))
import frigidaire_initial_state_resume as resume


def pose(x=0):
    return {'position_m': [x, 0., 0.], 'quaternion_xyzw': [0., 0., 0., 1.]}


def write(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj))


@pytest.fixture
def run(tmp_path):
    frames = {key: pose() for key in ('Cabinet', 'Door', 'LowerRack', 'UpperRack', 'SilverwareBasket')}
    objects = [{'candidate_id': f'c{i}', 'object_id': f'c{i}', 'kind': 'bowl', 'rack': 'LowerRack',
                'pose_world': pose(i), 'rack_local_pose': pose(i)} for i in range(3)]
    catalog = {'candidates': objects, 'candidate_count': 3, 'baseline_components': frames}
    graph = {'complete': True, 'unresolved_pairs': 0, 'allowed_indices': [0, 1, 2],
             'candidate_count': 3, 'conflict_pairs': [], 'candidate_catalog_sha256': resume.canonical_digest(catalog)}
    summary = {'schema_version': 1, 'started_utc': '2026-09-11T04:00:00+00:00', 'budget_seconds': 7200,
               'seed': 0, 'input_hashes': {}, 'search': {'solver_records': [
                {'bound_scope': 'full finite compatibility graph', 'target_count': None,
                 'status': 0, 'count': 3, 'geometric_cardinality_upper_bound': 3}]}}
    write(tmp_path / 'summary.json', summary)
    write(tmp_path / 'candidates.json', catalog)
    write(tmp_path / 'compatibility.json', graph)
    write(tmp_path / 'baseline/result.json', {'outcome': 'baseline_ready', 'poses': frames, 'snapshot': {'poses': frames}})
    write(tmp_path / 'input_validation.json', {})
    index = []
    for i, (indices, purpose, seed) in enumerate([([0, 1], 'capacity_search', 5), ([2], 'randomized', 100000)]):
        selected = [deepcopy(objects[j]) for j in indices]
        manifest = {'attempt_id': f'attempt_{i:04d}', 'objects': selected, 'candidate_indices': indices,
                    'purpose': purpose, 'order': 'upper_first', 'seed': seed, 'counts': resume.counts(selected)}
        snapshot = {'poses': {**frames, **{obj['object_id']: pose(j + .2) for j, obj in enumerate(selected)}}}
        result = {'outcome': 'accepted', 'object_count': len(selected), 'initial_snapshot': snapshot,
                  'proposed_objects': selected}
        write(tmp_path / f'attempts/attempt_{i:04d}/manifest.json', manifest)
        write(tmp_path / f'attempts/attempt_{i:04d}/result.json', result)
        if i == 0:
            index.append({'attempt_id': manifest['attempt_id'], 'outcome': 'accepted', 'count': len(selected),
                          'purpose': purpose, 'order': 'upper_first', 'seed': seed})
            highest = resume.state_from_completed_attempt(manifest, result, {}, state_id='highest', purpose='highest')
            write(tmp_path / 'states/highest.json', highest)
    write(tmp_path / 'attempt_index.json', index)
    return tmp_path


def restore(run):
    # Physics evidence validation is tested in the report suite; here inject a
    # seam to isolate restoring records, transforms, IDs and the original clock.
    return resume.restore_run(run, now_epoch=datetime(2026, 9, 11, 4, 20, tzinfo=timezone.utc).timestamp(),
                              now_monotonic=500., validate_state_fn=lambda state: state['accepted'])


def test_restore_keeps_elapsed_budget_and_recovers_unindexed_measured_state(run):
    data = restore(run)
    assert data['restored_elapsed_seconds'] == 1200
    assert data['budget'].started == -700
    assert data['budget'].deadline == 6500
    assert len(data['attempts']) == 2 and len(data['excluded']) == 2
    random = data['recovered_states'][0]
    assert random['state_id'] == 'random_00' and random['seed'] == 100000
    assert random['source_attempt'] == 'attempt_0001'
    assert random['objects'][0]['pose_world']['position_m'] == [.2, 0., 0.]
    assert random['objects'][0]['candidate_pose_world']['position_m'] == [2, 0., 0.]
    assert not (run / 'states/random_00.json').exists(), 'Read-only restore must not publish recovery'
    assert len(json.loads((run / 'attempt_index.json').read_text())) == 1


def test_pending_attempt_is_unresolved_and_exact_sets_are_deduplicated(run):
    manifest = json.loads((run / 'attempts/attempt_0001/manifest.json').read_text())
    manifest['attempt_id'] = 'attempt_0002'
    write(run / 'attempts/attempt_0002/manifest.json', manifest)
    data = restore(run)
    assert data['attempts'][-1]['outcome'] == 'interrupted'
    assert len(data['attempts']) == 3 and len(data['excluded']) == 2
    assert data['pending'][0]['attempt_id'] == 'attempt_0002'
    assert not (run / 'attempts/attempt_0002/result.json').exists()


def test_unknown_outcome_and_missing_indexed_evidence_fail_closed(run):
    result_path = run / 'attempts/attempt_0001/result.json'
    result = json.loads(result_path.read_text())
    result['outcome'] = 'maybe'
    write(result_path, result)
    with pytest.raises(ValueError, match='Unknown or unfinished'):
        restore(run)
    result['outcome'] = 'accepted'
    write(result_path, result)
    (run / 'attempts/attempt_0000/result.json').unlink()
    with pytest.raises(ValueError, match='Indexed completed attempt'):
        restore(run)


def test_catalog_tampering_and_attempt_gaps_fail_closed(run):
    catalog = json.loads((run / 'candidates.json').read_text())
    catalog['candidates'][0]['pose_world']['position_m'][0] = 99
    write(run / 'candidates.json', catalog)
    with pytest.raises(ValueError, match='graph hash'):
        restore(run)


def test_bound_ignores_fixed_target_unknown_and_unresolved_graph(run):
    summary = json.loads((run / 'summary.json').read_text())
    graph = json.loads((run / 'compatibility.json').read_text())
    assert resume.finite_bound(summary, graph) == 3
    summary['search']['solver_records'][0]['target_count'] = 2
    assert resume.finite_bound(summary, graph) is None
    summary['search']['solver_records'][0]['target_count'] = None
    graph['unresolved_pairs'] = 1
    assert resume.finite_bound(summary, graph) is None


def test_process_guard_detects_actual_owner_but_ignores_own_shell_wrapper(tmp_path):
    proc = tmp_path / 'proc'
    proc.mkdir()
    run = tmp_path / 'run'
    for pid, program in [(42, '/bin/bash'), (43, '/isaac-sim/kit/python/bin/python3')]:
        (proc / str(pid)).mkdir()
        (proc / str(pid) / 'cmdline').write_bytes('\0'.join([program, 'frigidaire_initial_state_resume.py', '--out-dir', str(run)]).encode())
    resume.assert_no_active_run(run, proc_root=proc, current_pid=43)
    with pytest.raises(RuntimeError, match='43'):
        resume.assert_no_active_run(run, proc_root=proc, current_pid=99)


def test_capacity_does_not_stop_on_empty_greedy_proposal(monkeypatch):
    experiment = resume.ResumedExperiment.__new__(resume.ResumedExperiment)
    calls = []
    class Budget:
        ticks = 0
        def remaining(self, deadline):
            self.ticks += 1
            return 1000 if self.ticks < 9 else 0
    experiment.budget = Budget()
    experiment.search_deadline = 999
    experiment.args = SimpleNamespace(minimum_attempt_seconds=70, seed=1)
    experiment.summary = {}
    experiment.solver_records = []
    experiment.graph = {'complete': True, 'allowed_indices': [0, 1, 2], 'unresolved_pairs': 0}
    experiment.best = {'objects': [{}, {}], 'candidate_indices': [0, 1]}
    experiment.attempts = []
    experiment.excluded = []
    experiment.update = lambda: None
    experiment.validate = lambda *args, **kwargs: calls.append('validate')
    monkeypatch.setattr(resume, 'greedy_proposal', lambda *args, **kwargs: {'selected_indices': None})
    def milp(*args, **kwargs):
        calls.append('milp')
        return {'selected_indices': [0, 1, 2], 'count': 3}
    monkeypatch.setattr(resume, 'milp_proposal', milp)
    experiment.run_capacity()
    assert calls.count('milp') >= 2


def test_capacity_skips_nonimproving_physics_and_falls_back_to_milp(monkeypatch):
    experiment = resume.ResumedExperiment.__new__(resume.ResumedExperiment)
    calls = []
    class Budget:
        ticks = 0
        def remaining(self, deadline):
            self.ticks += 1
            return 1000 if self.ticks < 5 else 0
    experiment.budget = Budget()
    experiment.search_deadline = 999
    experiment.args = SimpleNamespace(minimum_attempt_seconds=70, seed=1)
    experiment.summary = {}
    experiment.solver_records = []
    experiment.graph = {'complete': True, 'allowed_indices': [0, 1, 2], 'unresolved_pairs': 0}
    experiment.best = {'objects': [{}, {}], 'candidate_indices': [0, 1]}
    experiment.attempts, experiment.excluded = [], []
    experiment.update = lambda: None
    experiment.validate = lambda *args, **kwargs: calls.append('physics')
    monkeypatch.setattr(resume, 'greedy_proposal', lambda *args, **kwargs: {'selected_indices': [0], 'count': 1})
    def milp(*args, **kwargs):
        calls.append(('milp', kwargs['preferred_indices']))
        return {'selected_indices': [0, 1], 'count': 2}
    monkeypatch.setattr(resume, 'milp_proposal', milp)
    experiment.run_capacity()
    assert ('milp', [0, 1]) in calls
    assert 'physics' not in calls
