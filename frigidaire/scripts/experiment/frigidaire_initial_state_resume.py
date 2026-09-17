#!/usr/bin/env python3
"""Resume an existing packing experiment without resetting its wall-clock budget.

The default phase starts randomized validation only after the highest physical
load matches a recorded finite-catalog bound. All proposals still undergo the
original joint Isaac validation. No running experiment is stopped by this tool.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import time
import traceback
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'frigidaire/src'), str(Path(__file__).parent),
                str(ROOT / 'frigidaire/scripts/evaluation')]
from frigidaire_initial_state_experiment import Experiment, Budget, counts, digest, run_process, save, utc
from dishsim_frigidaire.initial_state_candidates import greedy_proposal, milp_proposal


KNOWN_OUTCOMES = {'accepted', 'initial_collision', 'settle_failure', 'penetration_failure',
                  'closure_failure', 'outside_dishwasher', 'timeout', 'simulation_error',
                  'interrupted'}
ATTEMPT_PATTERN = re.compile(r'attempt_(\d{4,})')


def read_json(path):
    return json.loads(Path(path).read_text())


def canonical_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def finite_bound(summary, graph):
    """Return only valid whole-graph bounds, never fixed-target solver bounds."""
    if not graph.get('complete') or graph.get('unresolved_pairs', 0) != 0:
        return None
    bounds = []
    for record in summary.get('search', {}).get('solver_records', []):
        bound = record.get('geometric_cardinality_upper_bound')
        if (record.get('bound_scope') == 'full finite compatibility graph'
                and record.get('target_count') is None and record.get('status') in (0, 1)
                and isinstance(bound, (int, float)) and not isinstance(bound, bool)
                and math.isfinite(bound) and bound == int(bound)
                and record.get('count', 0) <= bound <= len(graph['allowed_indices'])):
            bounds.append(int(bound))
    return min(bounds, default=None)


def assert_no_active_run(directory, *, proc_root=Path('/proc'), current_pid=None):
    """Fail before writing if an old controller/validator still owns this run."""
    directory = Path(directory).resolve()
    current_pid = os.getpid() if current_pid is None else current_pid
    owners = []
    names = {'frigidaire_initial_state_experiment.py', 'frigidaire_initial_state_resume.py',
             'frigidaire_initial_state_validate.py'}
    for entry in Path(proc_root).iterdir():
        if not entry.name.isdigit() or int(entry.name) == current_pid:
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().decode().rstrip('\0').split('\0')
        except (OSError, UnicodeError):
            continue
        # Isaac's shell wrappers repeat the script path in their argv. The
        # actual Python process is sufficient to detect an active owner and
        # avoids mistaking our own parent wrapper for a competing resume.
        if not argv or not Path(argv[0]).name.startswith('python'):
            continue
        if not any(Path(arg).name in names for arg in argv):
            continue
        for option in ('--out-dir', '--manifest'):
            if option not in argv:
                continue
            try:
                path = Path(argv[argv.index(option) + 1])
                if not path.is_absolute():
                    path = (entry / 'cwd').resolve() / path
                path = path.resolve()
                if path == directory or directory in path.parents:
                    owners.append(int(entry.name))
                    break
            except (IndexError, OSError):
                continue
    if owners:
        raise RuntimeError(f'Run still has active controller/validator processes {owners}; stop them at a completed boundary before resuming')


def _indices(manifest, catalog, allowed):
    indices = manifest.get('candidate_indices')
    if (not isinstance(indices, list) or not indices
            or any(not isinstance(i, int) or isinstance(i, bool) or i not in allowed for i in indices)
            or len(set(indices)) != len(indices)):
        raise ValueError('Attempt candidate indices must be unique eligible catalog integers')
    objects = manifest.get('objects', [])
    expected_ids = {catalog['candidates'][index]['candidate_id'] for index in indices}
    if len(objects) != len(indices) or {obj.get('object_id') for obj in objects} != expected_ids:
        raise ValueError('Attempt objects disagree with candidate indices')
    return sorted(indices)



def state_from_completed_attempt(manifest, result, input_hashes, *, state_id, purpose):
    """Recover measured state if the old controller stopped after result write."""
    from dishsim_frigidaire.random_poses import relative_pose
    prepared = {obj['object_id']: obj for obj in result['proposed_objects']}
    objects = []
    for original in manifest['objects']:
        measured = result['initial_snapshot']['poses'][original['object_id']]
        rack = result['initial_snapshot']['poses'][original['rack']]
        position, quaternion = relative_pose(measured['position_m'], measured['quaternion_xyzw'],
                                             rack['position_m'], rack['quaternion_xyzw'])
        objects.append(dict(original, candidate_pose_world=original['pose_world'],
            candidate_rack_local_pose=original['rack_local_pose'],
            proposed_pose_world=prepared[original['object_id']]['pose_world'], pose_world=measured,
            rack_local_pose={'position_m': position.tolist(), 'quaternion_xyzw': quaternion.tolist()}))
    return dict(manifest, schema_version=1, state_id=state_id, purpose=purpose,
                accepted=True, initial_snapshot=result['initial_snapshot'], objects=objects,
                validation=result, source_attempt=manifest['attempt_id'], input_hashes=input_hashes,
                recovery={'reason': 'Completed accepted result was saved before the previous controller consumed it.',
                          'source_manifest_purpose': manifest['purpose'],
                          'selection_method': 'original_controller_randomized_low_degree_graph_greedy'
                          if manifest['purpose'] == 'randomized' else 'original_capacity_search'})


def restore_run(directory, *, now_epoch=None, now_monotonic=None, validate_state_fn=None):
    """Read and validate resumable state; this function never writes artifacts."""
    directory = Path(directory).resolve()
    summary = read_json(directory / 'summary.json')
    if summary.get('schema_version') != 1:
        raise ValueError('Unsupported summary schema')
    started = datetime.fromisoformat(summary['started_utc'])
    if started.tzinfo is None:
        raise ValueError('Original start time must include a timezone')
    now_epoch = time.time() if now_epoch is None else now_epoch
    now_monotonic = time.monotonic() if now_monotonic is None else now_monotonic
    elapsed = now_epoch - started.timestamp()
    budget_seconds = summary['budget_seconds']
    if (not isinstance(budget_seconds, (int, float)) or not math.isfinite(budget_seconds)
            or not 0 < budget_seconds <= 7200 or elapsed < -1):
        raise ValueError('Invalid original budget or future start time')
    catalog = read_json(directory / 'candidates.json')
    graph = read_json(directory / 'compatibility.json')
    if not graph.get('complete'):
        raise ValueError('Cannot resume from an incomplete compatibility graph')
    if graph.get('candidate_catalog_sha256') not in {canonical_digest(catalog), digest(directory / 'candidates.json')}:
        raise ValueError('Candidate catalog does not match its graph hash')
    candidates = catalog['candidates']
    if len(candidates) != catalog['candidate_count'] or len(candidates) != graph['candidate_count']:
        raise ValueError('Candidate catalog/graph counts disagree')
    allowed = set(graph['allowed_indices'])
    if len(allowed) != len(graph['allowed_indices']) or any(not isinstance(i, int) or not 0 <= i < len(candidates) for i in allowed):
        raise ValueError('Invalid eligible graph indices')
    if len({candidate['candidate_id'] for candidate in candidates}) != len(candidates):
        raise ValueError('Duplicate candidate IDs')
    for edge in graph['conflict_pairs']:
        if len(edge) != 2 or edge[0] == edge[1] or any(index not in allowed for index in edge):
            raise ValueError('Invalid graph conflict edge')
    baseline = read_json(directory / 'baseline/result.json')
    if baseline.get('outcome') != 'baseline_ready' or baseline['snapshot']['poses'] != baseline['poses']:
        raise ValueError('A completed measured baseline is required')
    if baseline['poses'] != catalog['baseline_components']:
        raise ValueError('Catalog baseline differs from measured baseline')
    inputs = read_json(directory / 'input_validation.json')
    old_index = read_json(directory / 'attempt_index.json') if (directory / 'attempt_index.json').exists() else []
    if len({item['attempt_id'] for item in old_index}) != len(old_index):
        raise ValueError('Attempt index contains duplicate IDs')
    indexed = {item['attempt_id']: item for item in old_index}
    attempts, excluded, pending, completed = [], [], [], {}
    directories = []
    for path in (directory / 'attempts').iterdir():
        match = ATTEMPT_PATTERN.fullmatch(path.name)
        if path.is_dir() and match:
            directories.append((int(match[1]), path))
        elif path.is_dir():
            raise ValueError(f'Unknown attempt directory: {path.name}')
    directories.sort()
    if [number for number, _ in directories] != list(range(len(directories))):
        raise ValueError('Attempt directory IDs must be contiguous; refusing to risk an overwrite')
    for number, path in directories:
        manifest = read_json(path / 'manifest.json')
        if manifest.get('attempt_id') != path.name:
            raise ValueError('Attempt directory and manifest ID disagree')
        indices = _indices(manifest, catalog, allowed)
        if tuple(indices) not in {tuple(old) for old in excluded}:
            excluded.append(indices)
        result_path = path / 'result.json'
        if result_path.exists():
            result = read_json(result_path)
            outcome = result.get('outcome')
            if outcome not in KNOWN_OUTCOMES:
                raise ValueError(f'Unknown or unfinished saved outcome in {path.name}: {outcome!r}')
            completed[path.name] = (manifest, result)
        else:
            if path.name in indexed:
                raise ValueError('Indexed completed attempt is missing its result; refusing to rewrite lost evidence')
            outcome = 'interrupted'
            pending.append({'attempt_id': path.name, 'manifest_sha256': digest(path / 'manifest.json'),
                            'candidate_indices': indices, 'prior_files': sorted(item.name for item in path.iterdir())})
        record = {'attempt_id': path.name, 'outcome': outcome, 'count': len(indices),
                  'purpose': manifest['purpose'], 'order': manifest['order'], 'seed': manifest['seed']}
        if path.name in indexed and any(indexed[path.name].get(key) != value for key, value in record.items()):
            raise ValueError(f'Attempt index disagrees with saved evidence: {path.name}')
        attempts.append(record)
    if set(indexed) - {item['attempt_id'] for item in attempts}:
        raise ValueError('Attempt index references missing directories')
    if validate_state_fn is None:
        from frigidaire_initial_state_report import validate_state
        validate_state_fn = validate_state
    states = []
    state_sets = set()
    for path in sorted((directory / 'states').glob('*.json')):
        state = read_json(path)
        validate_state_fn(state)
        if state['state_id'] != path.stem:
            raise ValueError('State filename and ID disagree')
        _indices(state, catalog, allowed)
        attempt = completed.get(state['source_attempt'])
        if attempt is None or attempt[1] != state['validation'] or attempt[1].get('outcome') != 'accepted':
            raise ValueError('Saved state does not match a completed accepted source attempt')
        key = tuple(sorted(state['candidate_indices']))
        if state.get('purpose') == 'randomized' and key in state_sets:
            raise ValueError('Randomized states repeat an identical candidate set')
        state_sets.add(key)
        states.append(state)
    highest = [state for state in states if state['purpose'] == 'highest']
    if len(highest) != 1:
        raise ValueError('Exactly one saved highest accepted state is required for resumption')
    best = highest[0]
    if any(len(state['objects']) > len(best['objects']) for state in states):
        raise ValueError('Saved highest state is smaller than another saved state')
    if not any(state['state_id'] == 'highest' for state in highest):
        raise ValueError('Highest state must use the established highest.json path')
    if len([state for state in states if state['purpose'] == 'randomized']) > 10:
        raise ValueError('More than ten randomized states are already published')
    recovered_states = []
    published_attempts = {state['source_attempt'] for state in states}
    for name, (manifest, result) in completed.items():
        if result.get('outcome') != 'accepted' or name in published_attempts:
            continue
        if manifest['purpose'] == 'randomized':
            occupied = {state['state_id'] for state in states}
            number = (manifest['seed'] - summary['seed'] - 100000) // 1000
            if not 0 <= number < 10 or f'random_{number:02d}' in occupied:
                # Keep accepted evidence without inventing its intended state slot.
                continue
            recovered = state_from_completed_attempt(manifest, result, summary['input_hashes'],
                state_id=f'random_{number:02d}', purpose='randomized')
            recovered.update(requested_count=manifest['counts']['total'],
                             attempted_target_count=manifest['counts']['total'], density_retries=0)
            validate_state_fn(recovered)
            if tuple(sorted(recovered['candidate_indices'])) in state_sets:
                continue
            state_sets.add(tuple(sorted(recovered['candidate_indices'])))
            states.append(recovered)
            recovered_states.append(recovered)
        elif result.get('object_count', 0) > len(best['objects']):
            recovered = state_from_completed_attempt(manifest, result, summary['input_hashes'],
                state_id='highest', purpose='highest')
            validate_state_fn(recovered)
            states = [state for state in states if state['purpose'] != 'highest'] + [recovered]
            best = recovered
            recovered_states = [state for state in recovered_states if state['purpose'] != 'highest'] + [recovered]
    return {'directory': directory, 'summary': summary, 'catalog': catalog, 'graph': graph,
            'baseline': baseline['snapshot'], 'inputs': inputs, 'attempts': attempts,
            'excluded': excluded, 'pending': pending, 'states': states, 'best': best,
            'budget': Budget(float(budget_seconds), started=now_monotonic - max(0., elapsed)),
            'restored_elapsed_seconds': max(0., elapsed), 'finite_bound': finite_bound(summary, graph),
            'recovered_states': recovered_states}


class ResumedExperiment(Experiment):
    def __init__(self, args):
        assert_no_active_run(args.out_dir)
        restored = restore_run(args.out_dir)
        for name in ('directory', 'summary', 'catalog', 'graph', 'baseline', 'inputs', 'attempts',
                     'excluded', 'states', 'best', 'budget'):
            setattr(self, name, restored[name])
        bound = restored['finite_bound']
        highest = len(self.best['objects'])
        if args.phase == 'randomized' and bound != highest and not args.early_stop_reason:
            raise ValueError(f'Early randomized phase needs highest count equal to a valid finite-graph bound (highest={highest}, bound={bound}); an explicit --early-stop-reason is required to end capacity search earlier')
        if bound is not None and highest > bound:
            raise ValueError('Physical candidate count exceeds recorded geometric bound')
        if self.budget.remaining() <= 0:
            raise ValueError('Original two-hour experiment budget is exhausted; resumption cannot reset it')
        self.args = SimpleNamespace(**vars(args))
        self.args.seed = self.summary['seed']
        self.args.budget_seconds = self.summary['budget_seconds']
        self.args.accepted = Path(self.catalog['source_accepted_json'])
        self.search_deadline = self.budget.started + self.budget.seconds * 2 / 3
        self.random_deadline = self.budget.started + self.budget.seconds * 11 / 12
        self.solver_records = deepcopy(self.summary.get('search', {}).get('solver_records', []))
        self.inputs = read_json(self.directory / 'input_validation.json')
        # Validate immutable geometry/source-pool provenance before writing.
        if digest(self.args.accepted) != self.catalog['source_accepted_sha256']:
            raise ValueError('Source accepted pool has changed')
        from dishsim_frigidaire.random_pose_assets import validate_inputs
        actual_inputs = validate_inputs(args.usd)
        if actual_inputs['asset_hashes'] != self.inputs['asset_hashes']:
            raise ValueError('Experiment appliance/tableware geometry changed')
        event_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.event_directory = self.directory / 'resume_history' / event_id
        self.event_directory.mkdir(parents=True, exist_ok=False)
        shutil.copy2(self.directory / 'summary.json', self.event_directory / 'summary_before_resume.json')
        if (self.directory / 'attempt_index.json').exists():
            shutil.copy2(self.directory / 'attempt_index.json', self.event_directory / 'attempt_index_before_resume.json')
        sources = [Path(__file__), Path(sys.modules[Experiment.__module__].__file__),
                   ROOT / 'frigidaire/src/dishsim_frigidaire/initial_state_candidates.py',
                   ROOT / 'frigidaire/src/dishsim_frigidaire/initial_state_runtime.py',
                   ROOT / 'frigidaire/scripts/experiment/frigidaire_initial_state_validate.py',
                   ROOT / 'frigidaire/scripts/evaluation/frigidaire_initial_state_render.py',
                   ROOT / 'frigidaire/scripts/evaluation/frigidaire_initial_state_report.py']
        source_hashes = {}
        for path in sources:
            relative = path.resolve().relative_to(ROOT)
            source_hashes[str(relative)] = digest(path)
            destination = self.event_directory / 'source_snapshot' / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)
        self.event = {'schema_version': 1, 'resumed_utc': utc(), 'phase': args.phase,
            'original_started_utc': self.summary['started_utc'], 'original_budget_seconds': self.budget.seconds,
            'elapsed_before_resume_seconds': restored['restored_elapsed_seconds'],
            'remaining_before_resume_seconds': self.budget.remaining(),
            'highest_at_resume': highest, 'finite_graph_upper_bound': bound,
            'reason': getattr(args, 'resume_reason', None) or args.early_stop_reason or ('Resume capacity after the prior controller treated an empty greedy proposal as completion; keep searching until finite-bound saturation or the original capacity deadline.' if args.phase in {'capacity', 'capacity_then_randomized'} else 'Highest jointly validated count reaches the finite-catalog geometric upper bound; reuse unused capacity-search time for randomized states.'),
            'pending_attempts': restored['pending'],
            'recovered_states': [{'state_id': state['state_id'], 'source_attempt': state['source_attempt']}
                                 for state in restored['recovered_states']], 'source_hashes': source_hashes,
            'input_hashes': {name: digest(self.directory / name) for name in
                            ('candidates.json', 'compatibility.json', 'baseline/result.json', 'input_validation.json')},
            'capacity_proposal_priority': 'Physically test only counts above the highest validated load; alternate maximum-cardinality MILP preferences for retaining known-good candidates with unbiased ties, using improving greedy proposals when available.',
            'randomized_sampling': 'Alternate uniformly sampled subsets of the highest validated candidate set with randomized low-degree graph greedy; condition on uniqueness and full joint physics validation.'}
        for pending in restored['pending']:
            result_path = self.directory / 'attempts' / pending['attempt_id'] / 'result.json'
            if result_path.exists():
                raise RuntimeError('An interrupted attempt gained a result during restoration; refusing concurrent mutation')
            save(result_path, {'outcome': 'interrupted', 'result': 'INCOMPLETE', 'finished_utc': utc(),
                 'error': 'No completed result existed after the prior process stopped; no physical outcome is inferred.',
                 'resume_recovery': {'event': str(self.event_directory.relative_to(self.directory)), **pending}})
        for state in restored['recovered_states']:
            destination = self.directory / f"states/{state['state_id']}.json"
            if destination.exists():
                shutil.copy2(destination, self.event_directory / f"{state['state_id']}_before_recovery.json")
            save(destination, state)
        save(self.event_directory / 'resume.json', self.event)
        save(self.directory / 'attempt_index.json', self.attempts)
        self.summary.setdefault('resumptions', []).append(str((self.event_directory / 'resume.json').relative_to(self.directory)))
        self.summary['status'] = 'running'
        self.summary.pop('finished_utc', None)
        self.summary['resume_phase'] = args.phase
        self.summary['capacity_stop'] = {'highest_count': highest, 'finite_graph_upper_bound': bound,
                                         'reason': self.event['reason'], 'recorded_utc': utc()}
        self.proposal_number = 0
        self.update()

    def run_capacity(self):
        iteration = 0
        while self.budget.remaining(self.search_deadline) >= self.args.minimum_attempt_seconds:
            if finite_bound({'search': {'solver_records': self.solver_records}}, self.graph) == len(self.best['objects']):
                break
            seed = self.args.seed + 500000 + len(self.attempts) * 100 + iteration
            best_count = len(self.best['objects'])
            proposal = None
            if iteration % 3 == 0:
                proposal = greedy_proposal(self.graph, seed=seed, excluded_sets=self.excluded)
            if proposal is None or len(proposal.get('selected_indices') or []) <= best_count:
                preferred = self.best['candidate_indices'] if iteration % 3 != 2 else ()
                proposal = milp_proposal(self.graph, seed=seed, excluded_sets=self.excluded,
                    preferred_indices=preferred,
                    time_limit_s=min(60., self.budget.remaining(self.search_deadline) / 3))
                self.solver_records.append(dict(proposal, selected_indices=None))
            if len(proposal.get('selected_indices') or []) > best_count:
                self.validate(proposal['selected_indices'], seed=seed, purpose='capacity_search', deadline=self.search_deadline)
            else:
                print(f'[SEARCH] No larger proposal this restart; highest={best_count}, proposed={proposal.get("count", 0)}', flush=True)
            iteration += 1
            self.update()
        self.summary['capacity_stop'] = {'highest_count': len(self.best['objects']),
            'finite_graph_upper_bound': finite_bound({'search': {'solver_records': self.solver_records}}, self.graph),
            'reason': 'Resumed capacity phase reached its finite bound or original phase deadline.', 'recorded_utc': utc()}
        self.update()

    def random_proposal(self, *, state_number, target, retry, seed):
        """Record the nonuniform mixture explicitly; subsets are uniform draws."""
        import numpy as np
        excluded = {frozenset(indices) for indices in self.excluded}
        if retry % 2 == 0 and target <= len(self.best['candidate_indices']):
            rng = np.random.default_rng(seed)
            selected = sorted(int(i) for i in rng.choice(self.best['candidate_indices'], size=target, replace=False))
            if frozenset(selected) in excluded:
                selected = None
            method = 'uniform_subset_of_highest_validated_candidate_set'
        else:
            selected = greedy_proposal(self.graph, seed=seed, target_count=target,
                                       excluded_sets=self.excluded)['selected_indices']
            method = 'randomized_low_degree_graph_greedy'
        return {'method': method, 'state_number': state_number, 'seed': seed, 'retry': retry,
                'target_count': target, 'selected_indices': selected,
                'highest_source_attempt': self.best['source_attempt']}

    def run_randomized(self):
        maximum = len(self.best['objects'])
        targets = [max(1, math.ceil(maximum * fraction)) for fraction in [.25] * 3 + [.5] * 4 + [.75] * 3]
        existing = {state['state_id'] for state in self.states if state['purpose'] == 'randomized'}
        desired = {f'random_{number:02d}' for number in range(10)}
        if existing - desired:
            raise ValueError('Existing randomized state IDs do not use random_00 through random_09')
        remaining_numbers = [number for number in range(10) if f'random_{number:02d}' not in existing]
        for offset, number in enumerate(remaining_numbers):
            original_target = targets[number]
            target, retry = original_target, 0
            # Redistribute unused earlier time fairly among still-missing states;
            # keep the last 10 minutes for the two exact-pose renders/report.
            remaining_states = len(remaining_numbers) - offset
            share = self.budget.remaining(self.random_deadline) / remaining_states
            state_deadline = min(self.random_deadline, time.monotonic() + share)
            used_seeds = {attempt['seed'] for attempt in self.attempts}
            while self.budget.remaining(state_deadline) >= self.args.minimum_attempt_seconds:
                seed = self.args.seed + 100000 + number * 1000 + retry
                if seed in used_seeds:
                    retry += 1
                    continue
                proposal = self.random_proposal(state_number=number, target=target, retry=retry, seed=seed)
                proposal['requested_count'] = original_target
                proposal_path = self.event_directory / 'proposals' / f'proposal_{self.proposal_number:04d}.json'
                self.proposal_number += 1
                save(proposal_path, proposal)
                proposal_reference = str(proposal_path.relative_to(self.directory))
                print(f'[RANDOM] state=random_{number:02d} target={target} seed={seed} method={proposal["method"]}', flush=True)
                state = self.validate(proposal['selected_indices'], seed=seed, purpose='randomized', deadline=state_deadline)
                used_seeds.add(seed)
                if state:
                    state.update(state_id=f'random_{number:02d}', requested_count=original_target,
                        attempted_target_count=target, density_retries=retry,
                        generation=dict(proposal, proposal_record=proposal_reference))
                    save(self.directory / f'states/{state["state_id"]}.json', state)
                    self.states.append(state)
                    self.update()
                    break
                retry += 1
                if retry % 3 == 0:
                    target = max(1, target - 1)
            if self.budget.remaining(self.random_deadline) < self.args.minimum_attempt_seconds:
                break

    def run(self):
        if self.args.phase in {'capacity', 'capacity_then_randomized'}:
            self.run_capacity()
        self.run_randomized()

    def finish(self):
        """Finalize summary before report hashing; never rewrite it afterwards."""
        randomized = [state for state in self.states if state['purpose'] == 'randomized']
        chosen = [self.best]
        if randomized:
            chosen.append(min(randomized, key=lambda state: abs(len(state['objects']) - len(self.best['objects']) / 2)))
        for state in chosen:
            state_path = self.directory / f'states/{state["state_id"]}.json'
            evidence_path = self.directory / 'renders' / f'{state["state_id"]}_render_evidence.json'
            valid = False
            if evidence_path.exists():
                evidence = read_json(evidence_path)
                image_path = self.directory / 'renders' / evidence.get('image_file', '')
                valid = (evidence.get('result') == 'PASS' and evidence.get('state_sha256') == digest(state_path)
                         and image_path.is_file() and evidence.get('image_sha256') == digest(image_path))
            if valid or self.budget.remaining() < 60:
                continue
            command = [str(ROOT / 'scripts/run_kit.sh'), str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_initial_state_render.py'),
                '--state', str(state_path), '--usd', str(self.args.usd), '--out-dir', str(self.directory / 'renders'),
                '--headless', '--enable_cameras']
            run_process(command, self.event_directory / f'render_{state["state_id"]}.log',
                        min(270., max(1., self.budget.remaining() - 30)))
        from frigidaire_initial_state_report import image_records, load_experiment
        self.summary['renders'] = [{'path': str(path.relative_to(self.directory)), **read_json(path)}
                                  for path in sorted((self.directory / 'renders').glob('*_render_evidence.json'))]
        self.event['finished_utc'] = utc()
        self.event['randomized_count_after_resume'] = len(randomized)
        save(self.event_directory / 'resume.json', self.event)
        self.summary['status'] = 'incomplete'
        self.summary['finished_utc'] = utc()
        self.summary['wall_seconds_scope'] = 'Through final summary after rendering; report completion time is in completion.json.'
        self.update()
        evidence = load_experiment(self.directory)
        complete = len(randomized) == 10 and len(image_records(evidence)) == 2 and not evidence['problems']
        self.summary['status'] = 'complete' if complete else 'incomplete'
        self.summary['finished_utc'] = utc()
        self.update()
        report_started = time.monotonic()
        command = [str(ROOT / 'scripts/run_py.sh'), str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_initial_state_report.py'),
                   '--out-dir', str(self.directory)]
        report = run_process(command, self.event_directory / 'report.log', max(.1, min(60., self.budget.remaining())))
        completion = {'schema_version': 1, 'finished_utc': utc(),
            'original_started_utc': self.summary['started_utc'], 'budget_seconds': self.budget.seconds,
            'total_wall_seconds': time.monotonic() - self.budget.started,
            'report_wall_seconds': time.monotonic() - report_started, 'report_process': report,
            'summary_sha256': digest(self.directory / 'summary.json'), 'status': self.summary['status']}
        audit = self.directory / 'report_audit.json'
        if audit.exists():
            completion['report_audit_sha256'] = digest(audit)
            completion['report_audit_matches_summary'] = read_json(audit).get('input_hashes', {}).get('summary.json') == completion['summary_sha256']
        save(self.directory / 'completion.json', completion)
        print(f'[RESULT] {self.summary["status"]} highest={len(self.best["objects"])} randomized={len(randomized)} out={self.directory}', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', required=True, type=Path)
    parser.add_argument('--usd', type=Path, default=ROOT / 'build/frigidaire_collection/usd/fdpc4221as.usdc')
    parser.add_argument('--phase', choices=('randomized', 'capacity', 'capacity_then_randomized'), default='randomized')
    parser.add_argument('--early-stop-reason', help='Explicit recorded reason to start randomized validation before reaching the finite bound')
    parser.add_argument('--resume-reason', help='Provenance explanation for this controller/source revision; does not waive the finite-bound guard')
    parser.add_argument('--attempt-seconds', type=float, default=240.)
    parser.add_argument('--minimum-attempt-seconds', type=float, default=70.)
    args = parser.parse_args(argv)
    if not 5 < args.minimum_attempt_seconds <= args.attempt_seconds <= 600:
        parser.error('Require 5 < minimum-attempt-seconds <= attempt-seconds <= 600')
    experiment = None
    try:
        experiment = ResumedExperiment(args)
        experiment.run()
    except Exception as exc:
        traceback.print_exc()
        if experiment is None:
            return 1
        experiment.summary['resume_error'] = repr(exc)
    finally:
        if experiment is not None:
            experiment.finish()
    return 0 if experiment.summary['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
