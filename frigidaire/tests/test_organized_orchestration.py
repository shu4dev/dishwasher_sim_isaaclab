"""Inventory and truthful-delivery contracts for the organized experiment."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'frigidaire/scripts/experiment'))
spec = importlib.util.spec_from_file_location('organized_driver', ROOT / 'frigidaire/scripts/experiment/frigidaire_organized_experiment.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


def obj(identity, kind='mug', rack='UpperRack', x=0):
    pose = {'position_m': [x, 0., 0.], 'quaternion_xyzw': [0., 0., 0., 1.]}
    return {'object_id': identity, 'candidate_id': 'candidate_' + identity,
            'kind': kind, 'rack': rack, 'pose_world': pose, 'rack_local_pose': deepcopy(pose)}


def test_assignment_preserves_ids_and_minimizes_transfers():
    originals = [obj('original_lower', rack='LowerRack', x=1), obj('original_upper', x=0)]
    placements = [obj('new_lower', rack='LowerRack', x=0), obj('new_upper', x=1)]
    before = deepcopy(originals), deepcopy(placements)
    mapped = driver.assign_identities(placements, originals, {})
    assert {o['object_id']: o['rack'] for o in mapped} == {'original_lower': 'LowerRack', 'original_upper': 'UpperRack'}
    assert (originals, placements) == before


@pytest.mark.parametrize('placements', [[obj('new')], [obj('new_a'), obj('new_b', kind='bowl')]])
def test_assignment_rejects_missing_or_substituted_dishes(placements):
    with pytest.raises(ValueError, match='exact source inventory'):
        driver.assign_identities(placements, [obj('a'), obj('b')], {})


def test_compact_subset_retains_exact_kind_counts_and_is_repeatable():
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.accepted = {'highest': {'objects': [obj('m1'), obj('m2'), obj('b1', kind='bowl'), obj('b2', kind='bowl')]}}
    result = experiment.subset({'mug': 1, 'bowl': 2}, 17)
    assert driver.inventory(result) == {'mug': 1, 'bowl': 2}
    assert result == experiment.subset({'mug': 1, 'bowl': 2}, 17)
    result[0]['rack'] = 'changed'
    assert all(o['rack'] != 'changed' for o in experiment.accepted['highest']['objects'])
    assert experiment.subset({'dinner_plate': 1}, 17) is None


def test_no_proposal_is_unresolved_and_never_accepted(monkeypatch, tmp_path):
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.solve_inventory = lambda *a, **k: {'selected_indices': None, 'status': 'infeasible'}
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.args = types.SimpleNamespace(seed=1)
    experiment.sources = {'highest': (tmp_path / 'source.json', {'objects': [obj('a')]})}
    experiment.excluded = {}
    experiment.accepted = {}
    experiment.catalog = {'candidates': []}
    experiment.graph = {}
    experiment.summary = {'solver_records': [], 'catalog_level': 0}
    experiment.remaining = lambda deadline=None: 100
    experiment.update = lambda: None
    experiment.attempt = lambda *a, **k: pytest.fail('No candidates must not trigger an empty physics test')
    row = {'source_state_id': 'highest', 'status': 'pending', 'attempt_ids': []}
    experiment.organize(row, 100)
    assert row['status'] == 'unresolved'
    assert experiment.accepted == {}
    assert len(experiment.summary['solver_records']) == 1


def test_expired_allocation_does_not_launch_simulation(monkeypatch, tmp_path):
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.solve_inventory = lambda *a, **k: pytest.fail('Expired allocation must not start a solver')
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.sources = {'highest': (tmp_path / 'source.json', {'objects': [obj('a')]})}
    experiment.excluded = {}
    experiment.remaining = lambda deadline=None: 0
    experiment.update = lambda: None
    row = {'source_state_id': 'highest', 'status': 'pending', 'attempt_ids': []}
    experiment.organize(row, 0)
    assert row['status'] == 'unresolved'


@pytest.mark.parametrize('replay_outcome', [None, 'accepted', 'organization_failure', 'timeout'])
def test_recovery_preserves_primary_and_requires_independent_success(monkeypatch, tmp_path, replay_outcome):
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.directory = tmp_path
    experiment.attempts = []
    experiment.accepted = {}
    experiment.excluded = {}
    experiment.catalog = {'candidates': [obj('a')]}
    experiment.sources = {'highest': (tmp_path / 'source.json', {})}
    row = {'source_state_id': 'highest', 'status': 'pending', 'attempt_ids': []}
    experiment.summary = {'states': [row]}
    manifest = {'objects': [obj('a')], 'source_state_id': 'highest', 'purpose': 'organized',
                'seed': 1, 'order': 'upper_first', 'attempt_id': 'attempt_0000', 'reproduction_of': None}
    driver.save(tmp_path / 'attempts/attempt_0000/manifest.json', manifest)
    driver.save(tmp_path / 'attempts/attempt_0000/result.json', {'outcome': 'accepted'})
    original_bytes = (tmp_path / 'attempts/attempt_0000/result.json').read_bytes()
    monkeypatch.setattr(driver, 'measured_state', lambda manifest, result, path:
        dict(deepcopy(manifest), validation=result, organization={'valid': True}, accepted=False))
    module = types.ModuleType('dishsim_frigidaire.organization')
    module.preference_rank = lambda result: (0,)
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organization', module)
    if replay_outcome:
        driver.save(tmp_path / 'attempts/attempt_0001/manifest.json', dict(manifest,
            attempt_id='attempt_0001', purpose='reproduction', reproduction_of='attempt_0000'))
        driver.save(tmp_path / 'attempts/attempt_0001/result.json', {'outcome': replay_outcome})
    experiment.recover_attempts()
    assert (tmp_path / 'attempts/attempt_0000/result.json').read_bytes() == original_bytes
    assert len(experiment.attempts) == (2 if replay_outcome else 1)
    assert bool(experiment.accepted) == (replay_outcome == 'accepted')
    if replay_outcome in (None, 'timeout'):
        assert (tmp_path / row['provisional_state']).is_file()
    elif replay_outcome == 'organization_failure':
        assert not row.get('provisional_state')
        assert experiment.excluded['highest'] == [[0]]
    else:
        assert experiment.accepted['highest']['source_attempt'] == 'attempt_0000'
        assert experiment.accepted['highest']['reproduction']['source_attempt'] == 'attempt_0001'


@pytest.mark.parametrize('changed_baseline', [False, True])
def test_expansion_combines_only_compatible_completed_screens(monkeypatch, tmp_path, changed_baseline):
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.build_compatibility = lambda catalog, *a, **k: {'allowed_indices': list(range(len(catalog['candidates'])))}
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.directory = tmp_path
    experiment.summary = {'screening': {'catalog_adopted': True}}
    experiment.args = types.SimpleNamespace(usd=tmp_path / 'asset.usdc')
    experiment.highest_deadline, experiment.search_deadline = 1, 2
    experiment.remaining = lambda deadline=None: 600
    experiment.update = lambda: None
    base = dict(seed=7, baseline_components={'rack': 1}, asset_sha256='abc', policy={}, candidates=[obj('a')])
    extra = deepcopy(base)
    extra['candidates'].append(obj('b'))
    if changed_baseline:
        extra['baseline_components']['rack'] = 2
    driver.save(tmp_path / 'candidate_screening/screened_candidates.json', base)
    driver.save(tmp_path / 'candidate_screening_expansion/screened_candidates.json', extra)
    driver.save(tmp_path / 'candidate_screening_expansion/result.json', {'outcome': 'screening_partial'})
    if changed_baseline:
        with pytest.raises(ValueError, match='baseline_components'):
            experiment.screen_catalog()
    else:
        experiment.screen_catalog()
        assert [o['candidate_id'] for o in experiment.catalog['candidates']] == ['candidate_a', 'candidate_b']
        assert experiment.summary['screening']['expansion_adopted']
        assert experiment.summary['screening']['expansion_additional_proposals'] == 1
        assert len(experiment.catalog['screening_catalog_sources']) == 2


@pytest.mark.parametrize('modified', [False, True])
def test_input_bundle_preserves_original_bytes_or_rejects_drift(tmp_path, modified):
    source = tmp_path / 'source'
    asset = tmp_path / 'assets'
    source.mkdir(); asset.mkdir()
    original = source / 'state.json'
    original.write_bytes(b'{"objects": []}\n')
    usd = asset / 'machine.usdc'
    usd.write_bytes(b'asset')
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.directory = tmp_path / 'run'
    experiment.args = types.SimpleNamespace(source_run=source, usd=usd)
    experiment.summary = {'input_hashes': {'state.json': driver.digest(original)}}
    if modified:
        original.write_bytes(b'changed')
        with pytest.raises(ValueError, match='Input changed'):
            experiment.copy_inputs()
    else:
        experiment.copy_inputs()
        assert (experiment.directory / 'inputs/source_run/state.json').read_bytes() == original.read_bytes()
        assert (experiment.directory / 'inputs/assets/machine.usdc').read_bytes() == usd.read_bytes()


def precheck_experiment(tmp_path, inventories):
    experiment = driver.Organizer.__new__(driver.Organizer)
    experiment.args = types.SimpleNamespace(seed=9)
    experiment.sources = {}
    rows = []
    for identity, kinds in inventories.items():
        experiment.sources[identity] = (tmp_path / (identity + '.json'),
            {'objects': [obj(identity + str(i), kind=kind) for i, kind in enumerate(kinds)]})
        rows.append({'source_state_id': identity, 'status': 'pending', 'attempt_ids': []})
    experiment.summary = {'states': rows}
    experiment.catalog = {'candidates': [obj('a')]}
    experiment.graph = {'complete': True, 'full_catalog_complete': True,
                        'allowed_indices': [0], 'conflict_pairs': []}
    experiment.search_deadline = 1000
    experiment.remaining = lambda deadline=None: 100
    experiment.update = lambda: None
    return experiment, rows


def test_geometric_proof_is_uncut_cached_and_excluded_from_allocation(monkeypatch, tmp_path):
    experiment, rows = precheck_experiment(tmp_path, {'first': ['mug', 'mug'], 'same': ['mug', 'mug'], 'unknown': ['bowl']})
    experiment.excluded = {'first': [[0]]}
    calls = []
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    def solve(catalog, graph, desired, **kwargs):
        calls.append((desired, kwargs))
        if 'mug' in desired:
            return {'status': 2, 'selected_indices': None, 'geometric_feasibility': 'infeasible_finite_catalog'}
        return {'status': 1, 'selected_indices': None, 'geometric_feasibility': 'unresolved'}
    module.solve_inventory = solve
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment.precheck_inventories()
    experiment.precheck_inventories()
    assert len(calls) == 2  # One solve per distinct inventory, including unknowns.
    assert all(c[1]['excluded_sets'] == () and c[1]['time_limit_s'] <= 10 for c in calls)
    assert [row['source_state_id'] for row in experiment.allocation_rows(rows)] == ['unknown']
    assert rows[0]['status'] == rows[1]['status'] == 'unresolved'
    assert 'finite candidate' in rows[0]['reason']
    experiment.attempt = lambda *a, **k: pytest.fail('Proven infeasibility must not launch Kit')
    experiment.organize(rows[0], 100)


@pytest.mark.parametrize('status,classification,selected', [
    (1, 'unresolved', None), ('infeasible', 'infeasible_finite_catalog', None),
    (2, 'unresolved', None), (2, 'infeasible_finite_catalog', [0])])
def test_unknown_or_inconsistent_solver_result_remains_scheduled(monkeypatch, tmp_path, status, classification, selected):
    experiment, rows = precheck_experiment(tmp_path, {'trial': ['mug']})
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.solve_inventory = lambda *a, **k: {'status': status, 'selected_indices': selected,
        'geometric_feasibility': classification}
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment.precheck_inventories()
    assert experiment.allocation_rows(rows) == rows
    assert not rows[0]['geometric_precheck']['proven_infeasible']


@pytest.mark.parametrize('change', ['catalog', 'graph'])
def test_changed_catalog_or_graph_invalidates_cached_proof(monkeypatch, tmp_path, change):
    experiment, rows = precheck_experiment(tmp_path, {'trial': ['mug']})
    calls = []
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    def solve(*a, **k):
        calls.append(1)
        return ({'status': 2, 'selected_indices': None, 'geometric_feasibility': 'infeasible_finite_catalog'}
                if len(calls) == 1 else {'status': 0, 'selected_indices': [0], 'geometric_feasibility': 'feasible_proposal'})
    module.solve_inventory = solve
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment.precheck_inventories()
    assert experiment.geometrically_infeasible(rows[0])
    if change == 'catalog':
        experiment.catalog['candidates'][0]['pose_world']['position_m'][0] += .01
    else:
        experiment.graph['revision'] = 'recomputed'
    assert not experiment.geometrically_infeasible(rows[0])
    experiment.precheck_inventories()
    assert len(calls) == 2 and rows[0]['status'] == 'pending'
    assert experiment.allocation_rows(rows) == rows


def test_partial_graph_cannot_remove_inventory_allocation(monkeypatch, tmp_path):
    experiment, rows = precheck_experiment(tmp_path, {'trial': ['mug']})
    experiment.graph['full_catalog_complete'] = False
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.solve_inventory = lambda *a, **k: pytest.fail('Partial graph cannot certify full catalog infeasibility')
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment.precheck_inventories()
    assert experiment.allocation_rows(rows) == rows


def test_pending_replay_keeps_its_allocation_despite_catalog_proof(monkeypatch, tmp_path):
    experiment, rows = precheck_experiment(tmp_path, {'trial': ['mug']})
    module = types.ModuleType('dishsim_frigidaire.organized_candidates')
    module.solve_inventory = lambda *a, **k: {'status': 2, 'selected_indices': None,
        'geometric_feasibility': 'infeasible_finite_catalog'}
    monkeypatch.setitem(sys.modules, 'dishsim_frigidaire.organized_candidates', module)
    experiment.precheck_inventories()
    rows[0]['provisional_state'] = 'primary.json'
    assert experiment.allocation_rows(rows) == rows
    experiment.replay_provisional = lambda row, deadline: True
    experiment.organize(rows[0], 100)
