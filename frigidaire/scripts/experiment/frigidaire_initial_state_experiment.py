#!/usr/bin/env python3
"""Budgeted multi-object packing search; run with scripts/run_py.sh in Isaac container."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'frigidaire/src')]


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def counts(objects):
    return {'total': len(objects), 'by_kind': dict(Counter(x['kind'] for x in objects)),
            'by_rack': dict(Counter(x['rack'] for x in objects))}


def utc():
    return datetime.now(timezone.utc).isoformat()


class Budget:
    def __init__(self, seconds, *, started=None):
        self.started = time.monotonic() if started is None else started
        self.seconds = seconds
        self.deadline = self.started+seconds

    def remaining(self, deadline=None):
        return max(0., min(self.deadline, deadline or self.deadline)-time.monotonic())


def run_process(command, log_path, timeout):
    """Terminate only our child process group, retaining its log on timeout."""
    Path(log_path).parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, 'w') as log:
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            return {'exit_code': process.wait(timeout=max(.1, timeout)), 'timed_out': False}
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return {'exit_code': process.returncode, 'timed_out': True}


class Experiment:
    def __init__(self, args):
        self.args = args
        self.directory = args.out_dir.resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        prior_elapsed = max(0., time.time()-args.started_epoch) if args.started_epoch else 0.
        self.budget = Budget(args.budget_seconds, started=time.monotonic()-prior_elapsed)
        self.search_deadline = self.budget.started+args.budget_seconds*2/3
        self.random_deadline = self.budget.started+args.budget_seconds*11/12
        self.excluded, self.attempts, self.states, self.solver_records = [], [], [], []
        self.best, self.catalog, self.graph = None, None, None
        self.inputs = {}
        self.summary = {'schema_version': 1, 'status': 'running', 'started_utc': utc(),
            'seed': args.seed, 'budget_seconds': args.budget_seconds,
            'allocation_seconds': {'capacity': args.budget_seconds*2/3,
                'randomized': args.budget_seconds/4, 'render_report': args.budget_seconds/12},
            'source_pool': str(args.accepted), 'input_hashes': {'accepted_poses.json': digest(args.accepted)},
            'source_hashes': {}, 'limits': {}, 'notes': [
                'Packing capacity only: door remains open; no insertion paths or cleaning assessment.',
                'Highest validated load found is not a global capacity certificate.',
                'Physics failures exclude only exact candidate sets; supersets can gain support.']}
        if args.started_epoch:
            self.summary['started_utc'] = datetime.fromtimestamp(args.started_epoch, timezone.utc).isoformat()
            self.summary['prelaunch_elapsed_seconds'] = prior_elapsed
        sources = list((ROOT/'frigidaire/src/dishsim_frigidaire').glob('initial_state*.py'))
        sources += list((ROOT/'frigidaire/scripts/experiment').glob('frigidaire_initial_state*.py'))
        sources += list((ROOT/'frigidaire/scripts/evaluation').glob('frigidaire_initial_state*.py'))
        sources += [ROOT/'frigidaire/src/dishsim_frigidaire'/f'{name}.py' for name in
                    ('random_poses', 'random_pose_runtime', 'random_pose_experiment', 'random_pose_assets',
                     'loading', 'load_validation', 'asset', 'geometry', 'tableware')]
        sources += [ROOT/'src/dishsim/quats.py', ROOT/'src/dishsim/media.py']
        for path in sources:
            self.summary['source_hashes'][str(path.relative_to(ROOT))] = digest(path)
            snapshot = self.directory/'source_snapshot'/path.relative_to(ROOT)
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            snapshot.write_bytes(path.read_bytes())
        self.update()

    def update(self):
        self.summary.update(wall_seconds=time.monotonic()-self.budget.started,
            highest_count=len(self.best['objects']) if self.best else 0,
            randomized_count=sum(s['purpose'] == 'randomized' for s in self.states),
            attempt_count=len(self.attempts), outcomes=dict(Counter(a['outcome'] for a in self.attempts)),
            states=[f"states/{s['state_id']}.json" for s in self.states],
            search={'solver_records': self.solver_records, 'excluded_exact_sets': len(self.excluded)})
        save(self.directory/'summary.json', self.summary)

    def validate(self, indices, *, seed, purpose, deadline):
        if not indices:
            return None
        indices = sorted(set(int(i) for i in indices))
        if not indices or tuple(indices) in {tuple(sorted(s)) for s in self.excluded}:
            return None
        if self.budget.remaining(deadline) < self.args.minimum_attempt_seconds:
            return None
        objects = [dict(self.catalog['candidates'][i], object_id=self.catalog['candidates'][i]['candidate_id'])
                   for i in indices]
        manifest = {'schema_version': 1, 'seed': seed, 'purpose': purpose,
                    'candidate_indices': indices, 'objects': objects, 'counts': counts(objects),
                    'baseline': self.baseline}
        state = None
        for order in ('upper_first', 'lower_first'):
            if self.budget.remaining(deadline) < self.args.minimum_attempt_seconds:
                break
            attempt_id = f'attempt_{len(self.attempts):04d}'
            directory = self.directory/'attempts'/attempt_id
            save(directory/'manifest.json', dict(manifest, attempt_id=attempt_id, order=order))
            seconds = min(self.args.attempt_seconds, self.budget.remaining(deadline)-4)
            command = [str(ROOT/'scripts/run_kit.sh'), str(ROOT/'frigidaire/scripts/experiment/frigidaire_initial_state_validate.py'),
                '--usd', str(self.args.usd), '--out-dir', str(directory), '--manifest', str(directory/'manifest.json'),
                '--order', order, '--max-wall-seconds', str(seconds), '--headless', '--device', 'cpu']
            print(f'[ATTEMPT] {attempt_id} purpose={purpose} count={len(objects)} order={order}', flush=True)
            process = run_process(command, directory/'run.log', seconds+3)
            path = directory/'result.json'
            result = json.loads(path.read_text()) if path.exists() else {
                'outcome': 'timeout' if process['timed_out'] else 'simulation_error',
                'error': 'No completed result.json; inspect run.log'}
            result['process'] = process
            save(path, result)
            outcome = result.get('outcome', 'simulation_error')
            self.attempts.append({'attempt_id': attempt_id, 'outcome': outcome, 'count': len(objects),
                                  'purpose': purpose, 'order': order, 'seed': seed})
            save(self.directory/'attempt_index.json', self.attempts)
            print(f'[OUTCOME] {attempt_id} {outcome}', flush=True)
            self.update()
            if outcome == 'accepted':
                from dishsim_frigidaire.random_poses import relative_pose
                measured_objects = []
                prepared = {o['object_id']: o for o in result['proposed_objects']}
                for obj in objects:
                    measured = result['initial_snapshot']['poses'][obj['object_id']]
                    rack = result['initial_snapshot']['poses'][obj['rack']]
                    local_p, local_q = relative_pose(measured['position_m'], measured['quaternion_xyzw'],
                                                    rack['position_m'], rack['quaternion_xyzw'])
                    measured_objects.append(dict(obj, candidate_pose_world=obj['pose_world'],
                        candidate_rack_local_pose=obj['rack_local_pose'],
                        proposed_pose_world=prepared[obj['object_id']]['pose_world'], pose_world=measured,
                        rack_local_pose={'position_m': local_p.tolist(), 'quaternion_xyzw': local_q.tolist()}))
                state = dict(manifest, accepted=True, initial_snapshot=result['initial_snapshot'],
                             objects=measured_objects, validation=result, source_attempt=attempt_id,
                             input_hashes=self.summary['input_hashes'])
                break
            # A different order can only address a failure after initial settling.
            if not result.get('initial_snapshot'):
                break
        self.excluded.append(indices)
        if state and (self.best is None or len(objects) > len(self.best['objects'])):
            self.best = dict(state, state_id='highest', purpose='highest')
            save(self.directory/'states/highest.json', self.best)
            self.states = [s for s in self.states if s['purpose'] != 'highest']+[self.best]
            print(f'[BEST] highest_validated_count={len(objects)}', flush=True)
        self.update()
        return state

    def run(self):
        from dishsim_frigidaire.initial_state_candidates import generate_catalog, build_compatibility, greedy_proposal, milp_proposal
        from dishsim_frigidaire.random_pose_assets import validate_inputs
        from dishsim_frigidaire.random_poses import LIMITS
        self.summary['limits'] = dict(LIMITS, continuous_rest_observation_s=5.,
            initial_penetration_limit_m=.001, perturbation_translation_radius_m=.01,
            perturbation_rotation_max_deg=10., allow_stacking=True)
        self.inputs = validate_inputs(self.args.usd)
        save(self.directory/'input_validation.json', self.inputs)
        self.summary['input_hashes'].update(self.inputs['asset_hashes'])
        self.summary['input_hashes']['asset_hashes'] = self.inputs['asset_hashes']
        baseline_dir = self.directory/'baseline'
        command = [str(ROOT/'scripts/run_kit.sh'), str(ROOT/'frigidaire/scripts/experiment/frigidaire_initial_state_validate.py'),
                   '--usd', str(self.args.usd), '--out-dir', str(baseline_dir), '--headless', '--device', 'cpu',
                   '--max-wall-seconds', str(min(240., self.budget.remaining(self.search_deadline)))]
        if self.args.baseline_from:
            shutil.copytree(self.args.baseline_from, baseline_dir)
            self.summary['baseline_reused_from'] = str(self.args.baseline_from)
        else:
            run_process(command, baseline_dir/'run.log', min(245., self.budget.remaining(self.search_deadline)))
        baseline = json.loads((baseline_dir/'result.json').read_text())
        if baseline['outcome'] != 'baseline_ready':
            raise RuntimeError('Empty baseline failed: '+str(baseline))
        self.baseline = baseline['snapshot']
        self.catalog = generate_catalog(self.args.accepted, baseline['poses'], seed=self.args.seed, variants_per_template=8)
        save(self.directory/'candidates.json', self.catalog)
        print(f"[CATALOG] generated {len(self.catalog['candidates'])} candidates", flush=True)
        self.graph = build_compatibility(self.catalog, self.args.usd.parent, deadline=self.search_deadline,
                                        progress=lambda msg: print('[GEOMETRY] '+str(msg), flush=True))
        save(self.directory/'compatibility.json', self.graph)
        if not self.graph.get('complete') or not self.graph.get('allowed_indices'):
            raise RuntimeError('Compatibility graph is incomplete or has no eligible candidates')
        self.summary['catalog'] = {k: v for k, v in self.graph.items()
                                   if k not in {'conflict_pairs', 'candidate_checks', 'unary_checks', 'initial_filter'}}
        self.summary['catalog']['conflict_count'] = len(self.graph['conflict_pairs'])
        self.update()
        iteration = 0
        while self.budget.remaining(self.search_deadline) >= self.args.minimum_attempt_seconds:
            seed = self.args.seed+1000+iteration
            if iteration == 0:
                proposal = greedy_proposal(self.graph, seed=seed, target_count=2, excluded_sets=self.excluded)
            elif iteration % 4 == 1:
                proposal = milp_proposal(self.graph, seed=seed, time_limit_s=min(60., self.budget.remaining(self.search_deadline)/3), excluded_sets=self.excluded)
                self.solver_records.append(dict(proposal, selected_indices=None))
            else:
                # Alternate full greedy packings with incremental physical improvements.
                target = None if iteration % 4 == 2 else max(3, (len(self.best['objects']) if self.best else 1)+1)
                proposal = greedy_proposal(self.graph, seed=seed, target_count=target, excluded_sets=self.excluded)
            indices = proposal['selected_indices']
            if indices:
                self.validate(indices, seed=seed, purpose='capacity_search', deadline=self.search_deadline)
            bounds = [r['geometric_cardinality_upper_bound'] for r in self.solver_records
                      if r.get('bound_scope') == 'full finite compatibility graph'
                      and r.get('geometric_cardinality_upper_bound') is not None]
            if self.best and bounds and len(self.best['objects']) == min(bounds):
                self.summary['capacity_stop_reason'] = 'Validated load reaches finite catalog geometric upper bound'
                break
            iteration += 1
            self.update()
        if not self.best:
            raise RuntimeError('No jointly validated arrangement found within capacity phase')
        maximum = len(self.best['objects'])
        targets = [max(1, math.ceil(maximum*f)) for f in [.25]*3+[.5]*4+[.75]*3]
        for state_number, original_target in enumerate(targets):
            target, retry = original_target, 0
            state_deadline = time.monotonic()+self.budget.remaining(self.random_deadline)/(10-state_number)
            while self.budget.remaining(state_deadline) >= self.args.minimum_attempt_seconds:
                seed = self.args.seed+100000+state_number*1000+retry
                proposal = greedy_proposal(self.graph, seed=seed, target_count=target, excluded_sets=self.excluded)
                state = self.validate(proposal['selected_indices'], seed=seed, purpose='randomized', deadline=state_deadline)
                if state:
                    state.update(state_id=f'random_{state_number:02d}', requested_count=original_target,
                                 attempted_target_count=target, density_retries=retry)
                    save(self.directory/f"states/{state['state_id']}.json", state)
                    self.states.append(state)
                    self.update()
                    break
                retry += 1
                if retry % 3 == 0:
                    target = max(1, target-1)
            if self.budget.remaining(self.random_deadline) < self.args.minimum_attempt_seconds:
                break

    def finish(self):
        if self.best:
            randoms = [s for s in self.states if s['purpose'] == 'randomized']
            chosen = [self.best]
            if randoms:
                chosen.append(min(randoms, key=lambda s: abs(len(s['objects'])-len(self.best['objects'])/2)))
            for state in chosen:
                if self.budget.remaining() < 30:
                    break
                command = [str(ROOT/'scripts/run_kit.sh'), str(ROOT/'frigidaire/scripts/evaluation/frigidaire_initial_state_render.py'),
                    '--state', str(self.directory/f"states/{state['state_id']}.json"), '--usd', str(self.args.usd),
                    '--out-dir', str(self.directory/'renders'), '--headless', '--enable_cameras']
                run_process(command, self.directory/f"renders/{state['state_id']}.log", min(270., self.budget.remaining()-10))
        renders = []
        for path in (self.directory/'renders').glob('*_render_evidence.json'):
            renders.append({'path': str(path.relative_to(self.directory)), **json.loads(path.read_text())})
        self.summary['renders'] = renders
        self.summary.update(finished_utc=utc(), status='complete' if self.best and
            sum(s['purpose'] == 'randomized' for s in self.states) == 10 and
            sum(r.get('result') == 'PASS' for r in renders) == 2 else 'incomplete')
        self.update()
        command = [str(ROOT/'scripts/run_py.sh'), str(ROOT/'frigidaire/scripts/evaluation/frigidaire_initial_state_report.py'),
                   '--out-dir', str(self.directory)]
        run_process(command, self.directory/'report.log', max(1., min(60., self.budget.remaining())))
        print(f"[RESULT] {self.summary['status']} highest={self.summary['highest_count']} randomized={self.summary['randomized_count']} out={self.directory}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--accepted', type=Path, default=ROOT/'results/random_poses/frigidaire/complete_20260910_seed0/accepted_poses.json')
    parser.add_argument('--usd', type=Path, default=ROOT/'build/frigidaire_collection/usd/fdpc4221as.usdc')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--budget-seconds', type=float, default=7200.)
    parser.add_argument('--attempt-seconds', type=float, default=240.)
    parser.add_argument('--minimum-attempt-seconds', type=float, default=70.)
    parser.add_argument('--seed', type=int, default=20260911)
    parser.add_argument('--baseline-from', type=Path)
    parser.add_argument('--started-epoch', type=float, help='Include preceding baseline/smoke time in total wall budget')
    args = parser.parse_args()
    experiment = Experiment(args)
    try:
        experiment.run()
    except Exception as exc:
        traceback.print_exc()
        experiment.summary['error'] = repr(exc)
    finally:
        experiment.finish()


if __name__ == '__main__':
    main()
