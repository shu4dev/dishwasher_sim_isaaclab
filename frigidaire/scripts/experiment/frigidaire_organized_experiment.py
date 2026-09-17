#!/usr/bin/env python3
"""Organize fixed inventories and verify complete appliance cycles in Isaac Sim."""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'frigidaire/src')]
from frigidaire_initial_state_experiment import counts, digest, run_process, save, utc

UNRESOLVED_PROCESS_OUTCOMES = {'timeout', 'interrupted', 'simulation_error', 'shutdown_timeout', 'budget_exhausted'}


def inventory(objects):
    return dict(Counter(obj['kind'] for obj in objects))


def assign_identities(selected, source, baseline):
    """Preserve physical identities; prefer fewer rack transfers, then shorter moves."""
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    if inventory(selected) != inventory(source):
        raise ValueError('Organized assignment must preserve the exact source inventory')
    assigned = []
    for kind in sorted(inventory(source)):
        originals = sorted((o for o in source if o['kind'] == kind), key=lambda o: o['object_id'])
        placements = sorted((o for o in selected if o['kind'] == kind), key=lambda o: o['candidate_id'])
        costs = np.zeros((len(originals), len(placements)))
        for i, original in enumerate(originals):
            for j, placement in enumerate(placements):
                distance = np.linalg.norm(np.asarray(original['pose_world']['position_m']) -
                                          placement['pose_world']['position_m'])
                costs[i, j] = 1000 * (original['rack'] != placement['rack']) + distance + j * 1e-10
        rows, cols = linear_sum_assignment(costs)
        for i, j in zip(rows, cols):
            original, placement = originals[i], deepcopy(placements[j])
            placement.update(object_id=original['object_id'], source_object_id=original['object_id'],
                             source_rack=original['rack'], source_pose_world=original['pose_world'],
                             source_rack_local_pose=original['rack_local_pose'])
            assigned.append(placement)
    return sorted(assigned, key=lambda o: o['object_id'])


def measured_state(manifest, result, source_path):
    from dishsim_frigidaire.random_poses import relative_pose
    objects = []
    for original in manifest['objects']:
        obj = deepcopy(original)
        measured = result['initial_snapshot']['poses'][obj['object_id']]
        rack = result['initial_snapshot']['poses'][obj['rack']]
        p, q = relative_pose(measured['position_m'], measured['quaternion_xyzw'],
                             rack['position_m'], rack['quaternion_xyzw'])
        obj.update(candidate_pose_world=obj['pose_world'], candidate_rack_local_pose=obj['rack_local_pose'],
                   pose_world=measured, rack_local_pose={'position_m': p.tolist(), 'quaternion_xyzw': q.tolist()})
        objects.append(obj)
    return dict(manifest, objects=objects, accepted=False, status='pending_reproduction',
                source_state_sha256=digest(source_path), initial_snapshot=result['initial_snapshot'],
                validation=result, organization=result.get('organization_initial'), counts=counts(objects))


class Organizer:
    def __init__(self, args):
        self.args = args
        self.directory = args.out_dir.resolve()
        self.directory.mkdir(parents=True, exist_ok=args.resume)
        self.lock = (self.directory / 'run.lock').open('a+')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.sources = {}
        self.attempts = []
        self.catalog = self.graph = self.baseline = None
        self.excluded = {}
        self.accepted = {}
        self.started_epoch = time.time()
        if args.resume:
            self.summary = json.loads((self.directory / 'summary.json').read_text())
            self.started_epoch = self.summary['started_epoch']
            args.budget_seconds = self.summary['budget_seconds']
            args.seed = self.summary['seed']
            args.source_run = Path(self.summary['source_run'])
            self.attempts = json.loads((self.directory / 'attempt_index.json').read_text())
            self.excluded = self.summary.get('excluded', {})
        else:
            self.summary = {'schema_version': 1, 'experiment': 'organized_fixed_inventories',
                'status': 'running', 'started_epoch': self.started_epoch, 'started_utc': utc(),
                'source_run': str(args.source_run.resolve()), 'seed': args.seed,
                'budget_seconds': args.budget_seconds, 'states': [], 'controls': {}, 'solver_records': [],
                'input_hashes': {}, 'allocation_seconds': {'preparation': args.budget_seconds / 12,
                'highest_search': args.budget_seconds * 55 / 120, 'other_states': args.budget_seconds / 3,
                'reporting': args.budget_seconds / 8}, 'notes': [
                    'Exact original inventories; no removal, resizing, or relaxed organization gates.',
                    'Unresolved means not validated in this finite search and budget, not physically impossible.',
                    'Geometric washing suitability only: no spray arms, water, dispenser, or loading-path simulation.']}
            for path in sorted(args.source_run.rglob('*')):
                if path.is_file():
                    self.summary['input_hashes'][str(path.relative_to(args.source_run))] = digest(path)
        self.deadline = self.started_epoch + args.budget_seconds
        self.highest_deadline = self.started_epoch + args.budget_seconds * 65 / 120
        self.search_deadline = self.started_epoch + args.budget_seconds * 105 / 120
        for path in sorted((args.source_run / 'states').glob('*.json')):
            state = json.loads(path.read_text())
            self.sources[path.stem] = (path, state)
            if not args.resume:
                self.summary['states'].append({'source_state_id': path.stem, 'source_state': str(path.resolve()),
                    'source_state_sha256': digest(path), 'counts': counts(state['objects']),
                    'status': 'pending', 'accepted_state': None, 'attempt_ids': []})
        if len(self.sources) != 11 or 'highest' not in self.sources:
            raise ValueError('Expected highest plus ten saved randomized states')
        for row in self.summary['states']:
            if row['status'] == 'accepted':
                self.accepted[row['source_state_id']] = json.loads((self.directory / row['accepted_state']).read_text())
        if args.resume:
            if self.summary.get('candidate_file') and self.summary.get('compatibility_file'):
                self.catalog = json.loads((self.directory / self.summary['candidate_file']).read_text())
                self.graph = json.loads((self.directory / self.summary['compatibility_file']).read_text())
            self.recover_attempts()
        self.snapshot_sources()
        self.update()

    def recover_attempts(self):
        """Keep completed children, including successful primaries awaiting replay."""
        indexed = {a['attempt_id']: a for a in self.attempts}
        rows = {row['source_state_id']: row for row in self.summary['states']}
        provisional = {}
        for folder in sorted((self.directory / 'attempts').glob('attempt_*')):
            manifest_path, result_path = folder / 'manifest.json', folder / 'result.json'
            if not manifest_path.exists():
                continue
            manifest = json.loads(manifest_path.read_text())
            if not result_path.exists():
                save(result_path, {'outcome': 'interrupted', 'reason': 'Previous process ended without a completed result; partial evidence retained'})
            result = json.loads(result_path.read_text())
            attempt_id, identity = folder.name, manifest['source_state_id']
            row = rows[identity]
            if attempt_id not in indexed:
                indexed[attempt_id] = {'attempt_id': attempt_id, 'source_state_id': identity,
                    'purpose': manifest['purpose'], 'order': manifest['order'], 'seed': manifest['seed'],
                    'count': len(manifest['objects']), 'outcome': result['outcome'],
                    'reason': result.get('reason'), 'recovered_completed_child': True}
            if attempt_id not in row['attempt_ids']:
                row['attempt_ids'].append(attempt_id)
            if row['status'] == 'accepted':
                continue
            if manifest.get('reproduction_of'):
                primary = provisional.get(manifest['reproduction_of'])
                if primary:
                    if result.get('outcome') == 'accepted':
                        self.promote(row, primary, manifest, result)
                    else:
                        unresolved = result.get('outcome') in UNRESOLVED_PROCESS_OUTCOMES
                        primary['reproduction'] = {'result': 'UNRESOLVED' if unresolved else 'FAIL',
                                                  'source_attempt': attempt_id, 'validation': result}
                        path = self.directory / 'provisional_states' / f'{identity}_{primary["source_attempt"]}.json'
                        save(path, primary)
                        if unresolved:
                            row['provisional_state'] = str(path.relative_to(self.directory))
                            continue
                        row.pop('provisional_state', None)
                        lookup = {c['candidate_id']: i for i, c in enumerate(self.catalog['candidates'])} if self.catalog else {}
                        if all(o['candidate_id'] in lookup for o in primary['objects']):
                            self.excluded.setdefault(identity, []).append(sorted(lookup[o['candidate_id']] for o in primary['objects']))
                continue
            if result.get('outcome') != 'accepted':
                if result.get('outcome') not in UNRESOLVED_PROCESS_OUTCOMES and self.catalog:
                    lookup = {c['candidate_id']: i for i, c in enumerate(self.catalog['candidates'])}
                    if all(o['candidate_id'] in lookup for o in manifest['objects']):
                        selected = sorted(lookup[o['candidate_id']] for o in manifest['objects'])
                        if selected not in self.excluded.setdefault(identity, []):
                            self.excluded[identity].append(selected)
                continue
            state = measured_state(manifest, result, self.sources[identity][0])
            state.update(state_id=identity, source_attempt=attempt_id)
            path = self.directory / 'provisional_states' / f'{identity}_{attempt_id}.json'
            if path.exists():
                state = json.loads(path.read_text())
            else:
                save(path, state)
            provisional[attempt_id] = state
            if state.get('reproduction', {}).get('result') != 'FAIL':
                row['provisional_state'] = str(path.relative_to(self.directory))
        self.attempts = [indexed[key] for key in sorted(indexed)]

    def promote(self, row, state, replay_manifest, replay):
        from dishsim_frigidaire.organization import preference_rank
        state['reproduction'] = {'result': 'PASS', 'source_attempt': replay_manifest['attempt_id'], 'validation': replay}
        state.update(accepted=True, status='accepted')
        state['organization_preference_rank'] = preference_rank(state['organization'])
        state['preference_optimality_proven'] = False
        path = self.directory / 'states' / f'{row["source_state_id"]}.json'
        save(path, state)
        row.update(status='accepted', accepted_state=str(path.relative_to(self.directory)),
            organization=state['organization'], source_attempt=state['source_attempt'],
            reproduction_attempt=replay_manifest['attempt_id'],
            reason='Full inventory passed organization, full cycle, and independent replay')
        row.pop('provisional_state', None)
        self.accepted[row['source_state_id']] = state

    def replay_deadline(self, row, deadline, primary):
        if row['source_state_id'] == 'highest':
            return deadline
        estimate = min(self.args.attempt_seconds, max(90., primary.get('wall_seconds', 120.) * 1.3))
        untried = sum(not r['attempt_ids'] for r in self.allocation_rows(self.summary['states']) if r is not row)
        available = self.search_deadline - 60. * untried
        extended = max(deadline, min(available, time.time() + estimate))
        if extended > deadline:
            self.summary.setdefault('allocation_adjustments', []).append({'source_state_id': row['source_state_id'],
                'reason': 'Finish independent replay using observed primary duration; retain time for untried inventories',
                'added_seconds': extended - deadline})
        return min(extended, self.search_deadline)

    def replay_provisional(self, row, deadline):
        path = self.directory / row['provisional_state']
        state = json.loads(path.read_text())
        deadline = self.replay_deadline(row, deadline, state['validation'])
        if self.remaining(deadline) < 60:
            return False
        manifest, result = self.attempt(row, state['objects'], state['seed'], deadline,
            reproduction_of=state['source_attempt'], order=state['order'])
        if result['outcome'] == 'accepted':
            self.promote(row, state, manifest, result)
            self.update()
            return True
        unresolved = result.get('outcome') in UNRESOLVED_PROCESS_OUTCOMES
        state['reproduction'] = {'result': 'UNRESOLVED' if unresolved else 'FAIL',
                                'source_attempt': manifest['attempt_id'], 'validation': result}
        save(path, state)
        if unresolved:
            self.update()
            return False
        row.pop('provisional_state', None)
        lookup = {c['candidate_id']: i for i, c in enumerate(self.catalog['candidates'])}
        if all(o['candidate_id'] in lookup for o in state['objects']):
            self.excluded.setdefault(row['source_state_id'], []).append(sorted(lookup[o['candidate_id']] for o in state['objects']))
        self.update()
        return False

    def remaining(self, deadline=None):
        return max(0., min(deadline or self.deadline, self.deadline) - time.time())

    def snapshot_sources(self):
        hashes = {}
        paths = list((ROOT / 'frigidaire/src/dishsim_frigidaire').glob('*.py'))
        paths += list((ROOT / 'frigidaire/scripts').rglob('*.py'))
        paths += [ROOT / 'src/dishsim/quats.py', ROOT / 'src/dishsim/media.py']
        for path in paths:
            data = path.read_bytes()
            sha = hashlib.sha256(data).hexdigest()
            rel = str(path.relative_to(ROOT))
            hashes[rel] = sha
            dest = self.directory / 'source_versions' / sha / path.name
            if not dest.exists():
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
        self.summary['current_source_hashes'] = hashes

    def update(self):
        self.summary.update(wall_seconds=time.time() - self.started_epoch, attempt_count=len(self.attempts),
            outcomes=dict(Counter(a['outcome'] for a in self.attempts)), accepted_count=len(self.accepted),
            excluded=self.excluded)
        save(self.directory / 'attempt_index.json', self.attempts)
        save(self.directory / 'summary.json', self.summary)

    def invoke(self, directory, *, manifest=None, control=None, order='upper_first', deadline=None):
        seconds = min(self.args.attempt_seconds if manifest else 240., self.remaining(deadline) - 5.)
        if seconds < 30:
            return {'outcome': 'budget_exhausted', 'reason': 'Insufficient remaining time to start a fresh process'}
        self.snapshot_sources()
        directory.mkdir(parents=True, exist_ok=True)
        command = [str(ROOT / 'scripts/run_kit.sh'),
            str(ROOT / 'frigidaire/scripts/experiment/frigidaire_organized_validate.py'),
            '--usd', str(self.args.usd), '--out-dir', str(directory), '--max-wall-seconds', str(seconds),
            '--headless', '--device', 'cpu', '--order', order]
        if manifest is not None:
            manifest['execution_source_hashes'] = self.summary['current_source_hashes']
            save(directory / 'manifest.json', manifest)
            command += ['--manifest', str(directory / 'manifest.json')]
        else:
            command += ['--control', control]
        process = run_process(command, directory / 'run.log', seconds + 3)
        path = directory / 'result.json'
        result = json.loads(path.read_text()) if path.exists() else {
            'outcome': 'timeout' if process['timed_out'] else 'simulation_error',
            'reason': 'No completed result; inspect run.log'}
        result['process'] = process
        if process['timed_out'] and result.get('outcome') in ('accepted', 'control_passed'):
            result.update(outcome='shutdown_timeout', reason='Completed record but simulator failed to finish within deadline')
        save(path, result)
        return result

    def run_controls(self):
        for control in ('empty_cycle', 'blocked_door'):
            old = self.summary['controls'].get(control)
            if not old:
                # Recover a completed child if the controller was stopped at a
                # process boundary. Its evidence and original clock are retained.
                for path in sorted((self.directory / 'controls').glob(control + '_*/result.json'), reverse=True):
                    completed = json.loads(path.read_text())
                    if completed.get('outcome') == 'control_passed':
                        old = {'outcome': 'control_passed', 'result': str(path.relative_to(self.directory)),
                               'recovered_completed_control': True}
                        self.summary['controls'][control] = old
                        break
            if old and old.get('outcome') == 'control_passed':
                result = json.loads((self.directory / old['result']).read_text())
            else:
                print('[CONTROL] ' + control, flush=True)
                folder = self.directory / 'controls' / f'{control}_{len(list((self.directory / "controls").glob(control+"_*"))) if (self.directory / "controls").exists() else 0:02d}'
                result = self.invoke(folder, control=control, deadline=self.highest_deadline)
                self.summary['controls'][control] = {'outcome': result['outcome'],
                    'result': str((folder / 'result.json').relative_to(self.directory))}
                self.update()
            if result.get('outcome') != 'control_passed':
                raise RuntimeError('Current-geometry control failed: ' + control + ': ' + str(result.get('reason')))
            if control == 'empty_cycle':
                self.baseline = result.get('baseline', {}).get('snapshot', result.get('snapshot'))
                if self.baseline is None:
                    raise RuntimeError('Empty control omitted measured extended baseline snapshot')
        save(self.directory / 'baseline.json', self.baseline)

    def create_catalog(self, level=0):
        from dishsim_frigidaire.organized_candidates import generate_catalog, build_compatibility
        if self.remaining(self.highest_deadline) < 60:
            return False
        phase_end = min(self.highest_deadline - 45., time.time() + 900.)
        log = lambda message: print('[GEOMETRY] ' + str(message), flush=True)
        old_catalog, old_excluded = self.catalog, deepcopy(self.excluded)
        self.catalog = generate_catalog(self.args.usd.parent, self.baseline['poses'], seed=self.args.seed,
            deadline=time.monotonic() + max(1., min(300., (phase_end - time.time()) / 2)),
            progress=log, refinement_level=level, previous_catalog=old_catalog)
        save(self.directory / f'candidates_level{level}.json', self.catalog)
        self.graph = build_compatibility(self.catalog, self.args.usd.parent,
            deadline=time.monotonic() + max(1., phase_end - time.time()), progress=log)
        save(self.directory / f'compatibility_level{level}.json', self.graph)
        self.summary.update(catalog_level=level, candidate_file=f'candidates_level{level}.json',
                            compatibility_file=f'compatibility_level{level}.json')
        lookup = {c['candidate_id']: i for i, c in enumerate(self.catalog['candidates'])}
        self.excluded = {}
        if old_catalog:
            for identity, sets in old_excluded.items():
                for selected in sets:
                    names = [old_catalog['candidates'][i]['candidate_id'] for i in selected]
                    if all(name in lookup for name in names):
                        self.excluded.setdefault(identity, []).append(sorted(lookup[name] for name in names))
        self.update()
        return bool(self.graph.get('allowed_indices'))

    def assess_originals(self):
        from dishsim_frigidaire.organization import OrganizationGeometry, evaluate_organization
        geometry = OrganizationGeometry(self.args.usd.parent)
        for row in self.summary['states']:
            if row.get('initial_organization') is not None:
                continue
            state = self.sources[row['source_state_id']][1]
            paths = state.get('validation', {}).get('settled', {}).get('support_paths', {})
            support = {obj['object_id']: paths.get(obj['object_id']) == [obj['object_id'], obj['rack']]
                       for obj in state['objects']} if paths else None
            row['initial_organization'] = evaluate_organization(state['objects'],
                poses=state['initial_snapshot']['poses'], geometry=geometry,
                policy=self.summary['policy'], component_frames=state['initial_snapshot']['poses'], direct_support=support)
            row['initial_organization']['support_evidence'] = 'original saved rest-window contact paths; not a new physics replay'
        self.update()

    def screen_catalog(self):
        """Use isolated measured resting poses as proposals for joint validation."""
        from dishsim_frigidaire.organized_candidates import build_compatibility
        expansion_folder = self.directory / 'candidate_screening_expansion'
        expansion_ready = (expansion_folder / 'result.json').exists() and (expansion_folder / 'screened_candidates.json').exists()
        expansion_pending = expansion_ready and not self.summary.get('screening', {}).get('expansion_adopted')
        if self.summary.get('screening', {}).get('catalog_adopted') and not expansion_pending:
            return
        folder = self.directory / 'candidate_screening'
        aggregate = folder / 'screened_candidates.json'
        if not aggregate.exists():
            seconds = min(900., self.remaining(self.highest_deadline) - 600.)
            if seconds < 60:
                self.summary['screening'] = {'status': 'not_started', 'reason': 'Insufficient remaining highest-search allocation'}
                return
            self.snapshot_sources()
            command = [str(ROOT / 'scripts/run_kit.sh'),
                str(ROOT / 'frigidaire/scripts/experiment/frigidaire_organized_screen.py'),
                '--catalog', str(self.directory / self.summary['candidate_file']),
                '--baseline', str(self.directory / 'baseline.json'), '--usd', str(self.args.usd),
                '--out-dir', str(folder), '--max-wall-seconds', str(seconds), '--max-candidates', '300',
                '--headless', '--device', 'cpu']
            self.summary['screening'] = {'status': 'running', 'budget_seconds': seconds,
                'source_catalog': self.summary['candidate_file'],
                'reason': 'First-contact geometric seeds tipped during joint settling; screen isolated measured resting poses'}
            self.update()
            print('[SCREENING] isolated candidates; max_wall_seconds=' + str(seconds), flush=True)
            process = run_process(command, folder / 'run.log', seconds + 3)
            self.summary['screening']['process'] = process
        if not aggregate.exists():
            raise RuntimeError('Candidate screening did not produce its partial/final catalog')
        catalog = json.loads(aggregate.read_text())
        if expansion_pending:
            extra = json.loads((expansion_folder / 'screened_candidates.json').read_text())
            for key in ('seed', 'baseline_components', 'asset_sha256', 'policy'):
                if extra[key] != catalog[key]:
                    raise ValueError('Expanded screening catalog changes ' + key)
            known = {c['candidate_id'] for c in catalog['candidates']}
            additional = [c for c in extra['candidates'] if c['candidate_id'] not in known]
            catalog['candidates'].extend(additional)
            catalog['candidate_count'] = len(catalog['candidates'])
            catalog['count_by_kind'] = dict(Counter(c['kind'] for c in catalog['candidates']))
            catalog['screening_catalog_sources'] = [
                {'path': str(aggregate.relative_to(self.directory)), 'sha256': digest(aggregate)},
                {'path': 'candidate_screening_expansion/screened_candidates.json',
                 'sha256': digest(expansion_folder / 'screened_candidates.json')}]
            self.summary.setdefault('screening', {}).update(expansion_adopted=True,
                expansion_result='candidate_screening_expansion/result.json',
                expansion_additional_proposals=len(additional))
        if not catalog.get('candidates'):
            raise RuntimeError('No individual organized candidate passed measured settling')
        self.catalog = catalog
        save(self.directory / 'candidates_screened.json', catalog)
        graph_deadline = self.search_deadline if expansion_pending else self.highest_deadline
        self.graph = build_compatibility(catalog, self.args.usd.parent,
            deadline=time.monotonic() + max(1., min(600., self.remaining(graph_deadline) - 120.)),
            progress=lambda message: print('[SCREENED GEOMETRY] ' + str(message), flush=True))
        save(self.directory / 'compatibility_screened.json', self.graph)
        self.excluded = {}
        self.summary.update(candidate_file='candidates_screened.json', compatibility_file='compatibility_screened.json')
        self.summary.setdefault('screening', {}).update(status='completed', catalog_adopted=True,
            result='candidate_screening/result.json', catalog='candidate_screening/screened_candidates.json',
            accepted_proposals=len(catalog['candidates']), graph_allowed=len(self.graph.get('allowed_indices', [])),
            full_state_validation_claim=False)
        self.update()

    def geometric_domain_key(self):
        """Fingerprint the actual in-memory catalog and graph, including changes."""
        catalog, graph = getattr(self, 'catalog', None), getattr(self, 'graph', None)
        if catalog is None or graph is None:
            return None
        encode = lambda value: hashlib.sha256(json.dumps(value, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        return encode(catalog), encode(graph)

    def geometrically_infeasible(self, row):
        """A finite-catalog proof never cancels an accepted state or its replay."""
        if row.get('status') == 'accepted' or row.get('provisional_state'):
            return False
        domain = self.geometric_domain_key()
        if domain is None:
            return False
        catalog_sha, graph_sha = domain
        entry = getattr(self, 'summary', {}).get('geometric_prechecks', {}).get(catalog_sha, {})
        if entry.get('graph_sha256') != graph_sha:
            return False
        proof = row.get('geometric_precheck', {})
        if (proof.get('catalog_sha256'), proof.get('graph_sha256')) != domain:
            return False
        record = entry.get('inventories', {}).get(proof.get('inventory_key'), {})
        return record.get('proven_infeasible') is True

    def allocation_rows(self, rows):
        """Unknown feasibility retains its allocation; only proven cases skip."""
        return [row for row in rows if row.get('status') != 'accepted'
                and not self.geometrically_infeasible(row)]

    def precheck_inventories(self):
        """Bounded, uncut geometric solves protect time for viable inventories.

        A solver timeout is never an infeasibility proof. No physical failure
        cuts are included, and a partially checked graph cannot prove the full
        candidate catalog infeasible. Old catalog entries remain as history.
        """
        from dishsim_frigidaire.organized_candidates import solve_inventory
        domain = self.geometric_domain_key()
        if domain is None:
            return
        catalog_sha, graph_sha = domain
        cache = self.summary.setdefault('geometric_prechecks', {})
        entry = cache.get(catalog_sha)
        if entry is None or entry.get('graph_sha256') != graph_sha:
            if entry is not None:
                self.summary.setdefault('geometric_precheck_graph_history', []).append(deepcopy(entry))
            entry = {'catalog_sha256': catalog_sha, 'graph_sha256': graph_sha,
                'inventories': {}, 'scope': 'Current finite catalog only; no physical impossibility claim'}
            cache[catalog_sha] = entry
        complete_graph = bool(self.graph.get('complete') and self.graph.get('full_catalog_complete', True))
        for row in self.summary['states']:
            if row.get('status') == 'accepted' or row.get('provisional_state'):
                continue
            desired = inventory(self.sources[row['source_state_id']][1]['objects'])
            inventory_key = json.dumps(desired, sort_keys=True, separators=(',', ':'))
            record = entry['inventories'].get(inventory_key)
            if record is None:
                seconds = min(10., max(0., self.remaining(self.search_deadline) - 1.))
                if not complete_graph or seconds < 1.:
                    solver = {'status': 'unassessed', 'selected_indices': None,
                        'geometric_feasibility': 'unresolved',
                        'reason': 'Incomplete catalog graph' if not complete_graph else 'Insufficient precheck time'}
                else:
                    solver = solve_inventory(self.catalog, self.graph, desired,
                        seed=self.args.seed, time_limit_s=seconds, excluded_sets=())
                proven = bool(complete_graph and solver.get('status') == 2
                    and solver.get('selected_indices') is None
                    and solver.get('geometric_feasibility') == 'infeasible_finite_catalog')
                record = {'inventory': desired, 'solver': solver, 'proven_infeasible': proven,
                    'checked_utc': utc(), 'excluded_sets_used': False, 'maximum_solver_seconds': 10.}
                entry['inventories'][inventory_key] = record
            previous_proof = row.get('geometric_precheck', {})
            row['geometric_precheck'] = {'catalog_sha256': catalog_sha,
                'graph_sha256': graph_sha, 'inventory_key': inventory_key,
                'proven_infeasible': record['proven_infeasible'],
                'solver_status': record['solver'].get('status')}
            if record['proven_infeasible']:
                row.update(status='unresolved', reason=(
                    'Exact inventory is geometrically infeasible in the current finite candidate '
                    'catalog (' + catalog_sha[:12] + '); this is not a proof of physical impossibility'))
            elif previous_proof.get('proven_infeasible'):
                row.update(status='pending', reason='Catalog changed or proof invalidated; inventory remains scheduled for search')
        self.update()

    def attempt(self, row, objects, seed, deadline, *, reproduction_of=None, order='upper_first'):
        previous = [int(p.name.removeprefix('attempt_')) for p in (self.directory / 'attempts').glob('attempt_*')]
        attempt_id = f'attempt_{max(previous, default=-1) + 1:04d}'
        folder = self.directory / 'attempts' / attempt_id
        manifest = {'schema_version': 1, 'attempt_id': attempt_id, 'seed': seed,
            'purpose': 'reproduction' if reproduction_of else 'organized',
            'source_state_id': row['source_state_id'], 'source_state': row['source_state'],
            'policy': self.summary['policy'], 'objects': objects, 'counts': counts(objects),
            'baseline': self.baseline, 'order': order, 'reproduction_of': reproduction_of}
        print(f"[ATTEMPT] {attempt_id} {row['source_state_id']} {manifest['purpose']} n={len(objects)} {order}", flush=True)
        result = self.invoke(folder, manifest=manifest, order=order, deadline=deadline)
        self.attempts.append({'attempt_id': attempt_id, 'source_state_id': row['source_state_id'],
            'purpose': manifest['purpose'], 'order': order, 'seed': seed, 'count': len(objects),
            'outcome': result['outcome'], 'reason': result.get('reason')})
        row['attempt_ids'].append(attempt_id)
        print(f"[OUTCOME] {attempt_id} {result['outcome']} {result.get('reason', '')}", flush=True)
        self.update()
        return manifest, result

    def subset(self, desired, seed):
        import numpy as np
        rng = np.random.default_rng(seed)
        for state in sorted(self.accepted.values(), key=lambda s: -len(s['objects'])):
            available = inventory(state['objects'])
            if any(available.get(kind, 0) < count for kind, count in desired.items()):
                continue
            selected = []
            for kind, count in sorted(desired.items()):
                pool = [o for o in state['objects'] if o['kind'] == kind]
                # Pick a row direction once, then keep occupied runs compact.
                reverse = bool(rng.integers(2))
                pool.sort(key=lambda o: (o['rack'], str(o.get('row_id', '')),
                    o['rack_local_pose']['position_m'][1], o['rack_local_pose']['position_m'][0]), reverse=reverse)
                selected.extend(deepcopy(pool[:count]))
            return selected
        return None

    def organize(self, row, deadline):
        from dishsim_frigidaire.organized_candidates import solve_inventory
        identity = row['source_state_id']
        if row['status'] == 'accepted':
            return
        if row.get('provisional_state'):
            if self.replay_provisional(row, deadline):
                return
            if row.get('provisional_state'):
                row.update(status='unresolved', reason='Primary passed; independent replay remains unresolved')
                self.update()
                return
        if self.geometrically_infeasible(row):
            row.update(status='unresolved', reason='Exact inventory is infeasible in the current finite candidate catalog; no new physics attempt is warranted')
            self.update()
            return
        source_path, source = self.sources[identity]
        desired = inventory(source['objects'])
        iteration = len(row['attempt_ids'])
        failures = self.excluded.setdefault(identity, [])
        while self.remaining(deadline) >= 60:
            seed = self.args.seed + 1000 * list(self.sources).index(identity) + iteration
            selected = self.subset(desired, seed) if iteration % 2 == 0 else None
            indices = None
            if selected is None:
                proposal = solve_inventory(self.catalog, self.graph, desired, seed=seed,
                    time_limit_s=min(60., max(1., self.remaining(deadline) / 4)), excluded_sets=failures)
                self.summary['solver_records'].append(dict(proposal, source_state_id=identity,
                                                          catalog_level=self.summary['catalog_level']))
                self.update()
                indices = proposal.get('selected_indices')
                if not indices:
                    row['reason'] = 'No full-inventory proposal found in the current organized candidate catalog'
                    break
                selected = [self.catalog['candidates'][i] for i in indices]
            objects = assign_identities(selected, source['objects'], self.baseline)
            for order in ('upper_first', 'lower_first'):
                manifest, result = self.attempt(row, objects, seed, deadline, order=order)
                if result['outcome'] == 'accepted':
                    state = measured_state(manifest, result, source_path)
                    state.update(state_id=identity, source_attempt=manifest['attempt_id'])
                    primary = self.directory / 'provisional_states' / f'{identity}_{manifest["attempt_id"]}.json'
                    save(primary, state)
                    row['provisional_state'] = str(primary.relative_to(self.directory))
                    replay_deadline = self.replay_deadline(row, deadline, result)
                    if self.remaining(replay_deadline) < 60:
                        row['reason'] = 'Primary cycle passed; insufficient time for independent reproduction'
                        row['provisional_state'] = str(primary.relative_to(self.directory))
                        break
                    replay_manifest, replay = self.attempt(row, state['objects'], seed, replay_deadline,
                        reproduction_of=manifest['attempt_id'], order=order)
                    state['reproduction'] = {'result': 'PASS' if replay['outcome'] == 'accepted' else 'FAIL',
                        'source_attempt': replay_manifest['attempt_id'], 'validation': replay}
                    if replay['outcome'] == 'accepted':
                        self.promote(row, state, replay_manifest, replay)
                        self.update()
                        return
                    save(primary, state)
                    if replay['outcome'] in UNRESOLVED_PROCESS_OUTCOMES:
                        state['reproduction']['result'] = 'UNRESOLVED'
                        save(primary, state)
                    else:
                        row.pop('provisional_state', None)
                    row['reason'] = 'Primary cycle passed but independent reproduction failed'
                    break
                if not result.get('initial_snapshot') or result['outcome'] != 'closure_failure':
                    break
                if self.remaining(deadline) < 60:
                    break
            if indices:
                failures.append(sorted(indices))
            iteration += 1
            self.update()
        if row['status'] != 'accepted':
            row.update(status='unresolved', reason=row.get('reason', 'Search allocation exhausted without a reproducible full-inventory state'))
        self.update()

    def verify_sources(self):
        changed = [rel for rel, sha in self.summary['input_hashes'].items()
                   if not (self.args.source_run / rel).is_file() or digest(self.args.source_run / rel) != sha]
        result = {'result': 'PASS' if not changed else 'FAIL', 'verified_files': len(self.summary['input_hashes']),
                  'changed_or_missing': changed}
        save(self.directory / 'source_preservation_audit.json', result)
        return result

    def copy_inputs(self):
        """Bundle byte-identical input evidence so source links survive extraction."""
        records = []
        groups = [(self.args.source_run, self.directory / 'inputs/source_run', self.summary['input_hashes'])]
        asset_root = self.args.usd.parent
        asset_hashes = {str(path.relative_to(asset_root)): digest(path)
                        for path in asset_root.rglob('*') if path.is_file()}
        groups.append((asset_root, self.directory / 'inputs/assets', asset_hashes))
        for source_root, destination, hashes in groups:
            for relative, expected in hashes.items():
                source, target = source_root / relative, destination / relative
                data = source.read_bytes()
                if hashlib.sha256(data).hexdigest() != expected:
                    raise ValueError('Input changed before bundling: ' + str(source))
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
                records.append({'source': str(source), 'copy': str(target.relative_to(self.directory)), 'sha256': expected})
        save(self.directory / 'input_copies.json', {'result': 'PASS', 'files': records})

    def deliver(self):
        for row in self.summary['states']:
            if row['status'] == 'pending':
                row.update(status='unresolved', reason='Experiment stopped before this inventory could be validated')
        self.summary.update(status='complete', finished_utc=utc())
        self.summary['source_preservation'] = self.verify_sources()
        self.copy_inputs()
        self.update()
        if not self.args.skip_render and self.remaining() > 90:
            command = [str(ROOT / 'scripts/run_kit.sh'),
                str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_organized_render.py'),
                '--batch', str(self.directory / 'summary.json'), '--out-dir', str(self.directory / 'renders'),
                '--headless', '--enable_cameras']
            self.summary['render_process'] = run_process(command, self.directory / 'render.log', max(1., self.remaining() - 100))
            self.update()
        command = [str(ROOT / 'scripts/run_py.sh'),
            str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_organized_report.py'),
            '--out-dir', str(self.directory), '--pdf']
        self.summary['report_process'] = run_process(command, self.directory / 'report.log', max(1., self.remaining() - 60))
        self.summary['delivery_wall_seconds'] = time.time() - self.started_epoch
        self.summary['within_budget'] = self.summary['delivery_wall_seconds'] <= self.args.budget_seconds
        # Do not change report inputs after report generation: completion is separate.
        save(self.directory / 'completion.json', {'status': 'complete', 'finished_utc': utc(),
            'summary_sha256': digest(self.directory / 'summary.json'),
            'wall_seconds': self.summary['delivery_wall_seconds'], 'within_budget': self.summary['within_budget'],
            'render_process': self.summary.get('render_process'), 'report_process': self.summary['report_process']})
        command = [str(ROOT / 'scripts/run_py.sh'),
            str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_organized_package.py'), '--out-dir', str(self.directory)]
        # Keep the package log outside its own input tree so ZIP checksums cannot
        # race a log being written while it is archived.
        package_log = self.directory.with_name(self.directory.name + '_package.log')
        packaging = run_process(command, package_log, max(1., self.remaining() - 1))
        print('[PACKAGE] ' + json.dumps(packaging), flush=True)

    def run(self):
        from dishsim_frigidaire.organization import default_policy
        from dishsim_frigidaire.random_pose_assets import validate_inputs
        self.summary['policy'] = default_policy()
        save(self.directory / 'input_validation.json', validate_inputs(self.args.usd))
        try:
            self.run_controls()
            self.assess_originals()
            if self.catalog is None and not self.create_catalog(0):
                raise RuntimeError('No geometrically eligible organized candidates were generated')
            self.screen_catalog()
            self.precheck_inventories()
            highest = next(r for r in self.summary['states'] if r['source_state_id'] == 'highest')
            for level in range(self.summary.get('catalog_level', 0), 3):
                self.organize(highest, self.highest_deadline)
                if highest['status'] == 'accepted' or self.remaining(self.highest_deadline) < 180:
                    break
                if self.summary.get('screening', {}).get('catalog_adopted'):
                    # Further first-contact variants are not a substitute for
                    # the measured-pose refinement just performed.
                    break
                if level < 2 and not self.create_catalog(level + 1):
                    break
            others = [r for r in self.summary['states'] if r['source_state_id'] != 'highest']
            for index, row in enumerate(others):
                if not self.allocation_rows([row]):
                    continue
                remaining_rows = len(self.allocation_rows(others[index:]))
                allocation = self.remaining(self.search_deadline) / max(1, remaining_rows)
                self.organize(row, min(self.search_deadline, time.time() + allocation))
            # Reuse unspent allocations without denying another inventory its first pass.
            for row in self.summary['states']:
                if self.allocation_rows([row]) and self.remaining(self.search_deadline) >= 120:
                    self.organize(row, self.search_deadline)
        except Exception as exc:
            traceback.print_exc()
            self.summary['execution_error'] = {'error': repr(exc), 'traceback': traceback.format_exc()}
        finally:
            self.deliver()
        print('[RESULT] complete ' + str(self.directory / 'summary.json'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, default=ROOT / 'results/initial_states/frigidaire/packing_20260911_seed20260911')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--usd', type=Path, default=ROOT / 'build/frigidaire_collection/usd/fdpc4221as.usdc')
    parser.add_argument('--seed', type=int, default=20260911)
    parser.add_argument('--budget-seconds', type=float, default=7200.)
    parser.add_argument('--attempt-seconds', type=float, default=480.)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--skip-render', action='store_true')
    args = parser.parse_args()
    if args.budget_seconds < 120 or not math.isfinite(args.budget_seconds):
        parser.error('budget-seconds must be finite and at least 120')
    Organizer(args).run()


if __name__ == '__main__':
    main()
