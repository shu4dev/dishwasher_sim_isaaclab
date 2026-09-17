#!/usr/bin/env python3
"""Measure independent resting candidate poses before joint organized search.

One reused Isaac scene is a proposal screening accelerator. A screened candidate
is never a validated complete state: fresh joint and replay cycles remain required.
"""
from __future__ import annotations
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def active_entry(candidate):
    entry = deepcopy(candidate)
    entry['object_id'] = 'screen_'+entry['kind']
    return entry


def screened_candidate(candidate, snapshot, baseline, index, trial_path):
    """Convert a measured pose through its own rack into the common frame."""
    from dishsim_frigidaire.random_poses import relative_pose, compose_pose
    entry = deepcopy(candidate)
    measured = snapshot['poses']['screen_'+entry['kind']]
    rack = snapshot['poses'][entry['rack']]
    p, q = relative_pose(measured['position_m'], measured['quaternion_xyzw'],
                         rack['position_m'], rack['quaternion_xyzw'])
    reference = baseline['poses'][entry['rack']]
    wp, wq = compose_pose(reference['position_m'], reference['quaternion_xyzw'], p, q)
    source_id = entry['candidate_id']
    entry.update(candidate_id=source_id+'_settled', object_id=source_id+'_settled',
        source_candidate_id=source_id, source_candidate_index=entry.get('candidate_index'),
        source_proposed_pose_world=deepcopy(entry['pose_world']),
        source_proposed_rack_local_pose=deepcopy(entry['rack_local_pose']),
        candidate_index=index, position_xy_m=p[:2].tolist(), quaternion_xyzw=q.tolist(),
        rack_local_pose={'position_m': p.tolist(), 'quaternion_xyzw': q.tolist()},
        pose_world={'position_m': wp.tolist(), 'quaternion_xyzw': wq.tolist()},
        support_screen={'outcome': 'screening_passed', 'result_file': trial_path,
                   'scope': 'one dish, reused scene, initial settling only; fresh joint full-cycle validation still required'})
    return entry


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--usd', type=Path, default=ROOT/'build/frigidaire_collection/usd/fdpc4221as.usdc')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=900.)
    parser.add_argument('--max-candidates', type=int, default=300)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device='cpu', headless=True)
    args = parser.parse_args()
    if not math.isfinite(args.max_wall_seconds) or args.max_wall_seconds <= 0 or args.max_candidates <= 0:
        parser.error('Screening time and candidate count must be positive')
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if (args.out_dir/'screened_candidates.json').exists() or (args.out_dir/'trials').exists():
        raise FileExistsError('Refusing to overwrite previous screening evidence')
    source = json.loads(args.catalog.read_text())
    baseline = json.loads(args.baseline.read_text())
    baseline = baseline.get('snapshot', baseline)
    output = deepcopy(source)
    output.update(candidates=[], candidate_count=0, complete=False, screening_only=True,
        source_candidate_count=len(source['candidates']), screening_test_cap=args.max_candidates,
        screening_plate_diverse_slot_pass_cap=6, screening_skips=[],
        source_catalog=str(args.catalog), source_catalog_sha256=hashlib.sha256(args.catalog.read_bytes()).hexdigest(),
        source_baseline_sha256=hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
        generation='independently settled candidates; reused screening scene, fresh complete-state cycles required',
        screening_attempts=[], screening_status='running',
        screening_started_utc=datetime.now(timezone.utc).isoformat())
    source_paths = [Path(__file__), ROOT/'frigidaire/src/dishsim_frigidaire/organization_runtime.py',
        ROOT/'frigidaire/src/dishsim_frigidaire/organization.py',
        ROOT/'frigidaire/src/dishsim_frigidaire/organized_candidates.py',
        ROOT/'frigidaire/src/dishsim_frigidaire/initial_state_runtime.py']
    output['screening_execution_source_hashes'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    save(args.out_dir/'screened_candidates.json', output)
    app, backend = None, None
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT/'src'), str(ROOT/'frigidaire/src')]
        from dishsim_frigidaire.organization_runtime import IsaacOrganizedStateBackend
        from dishsim_frigidaire.organized_candidates import screening_order
        from dishsim_frigidaire.random_pose_runtime import COMPONENTS
        from dishsim_frigidaire.random_pose_experiment import BudgetExpired
        from dishsim_frigidaire.random_poses import source_geometry_domains, RACKS, LIMITS
        representatives = []
        for kind in ('bowl', 'mug', 'dinner_plate'):
            example = next((c for c in source['candidates'] if c['kind'] == kind), None)
            if example is not None:
                representatives.append(active_entry(example))
        backend = IsaacOrganizedStateBackend(args.usd, args.out_dir, device=args.device,
            domains=source_geometry_domains(), deadline=started+args.max_wall_seconds-3.,
            app=app, candidates=representatives, organization_policy=source.get('policy'))
        backend.runtime_settings.update(fresh_scene_per_arrangement=False,
            screening_reused_scene=True, isolated_active_dish=True,
            parked_objects=list(backend.objects), no_loaded_rack_or_door_cycle=True,
            door_cycle_enabled=False, reopen_and_extend_validation=False)
        output['runtime_settings'] = deepcopy(backend.runtime_settings)
        checker = backend.organization_geometry.checker
        ordered = screening_order(source)
        passed_plate_slots = set()
        for ordinal, candidate_index in enumerate(ordered):
            if len(output['screening_attempts']) >= args.max_candidates:
                break
            if time.monotonic() >= backend.deadline-5.:
                break
            candidate = source['candidates'][candidate_index]
            if candidate['kind'] == 'dinner_plate' and (len(passed_plate_slots) >= 6 or candidate['slot_id'] in passed_plate_slots):
                output['screening_skips'].append({'source_candidate_id': candidate['candidate_id'],
                    'reason': 'six_diverse_plate_slots_passed' if len(passed_plate_slots) >= 6 else 'plate_slot_already_passed'})
                continue
            entry = active_entry(candidate)
            trial_id = f"screen_{len(output['screening_attempts']):04d}"
            trial_dir = args.out_dir/'trials'/trial_id
            trial_dir.mkdir(parents=True, exist_ok=False)
            backend.directory = trial_dir
            backend.entries = [entry]
            backend.assignments = {entry['object_id']: entry['rack']}
            backend._organization_cache_key = backend._organization_cache = None
            backend.last_organization = None
            trial = {'outcome': 'unresolved', 'trial_id': trial_id,
                'source_candidate_index': candidate_index, 'source_candidate_id': candidate['candidate_id'],
                'candidate': deepcopy(candidate), 'active_object_id': entry['object_id'],
                'parked_object_ids': [key for key in backend.objects if key != entry['object_id']],
                'execution_source_hashes': output['screening_execution_source_hashes'],
                'asset_sha256': output['asset_sha256'],
                'screening_only': True, 'trace_file': 'trace.jsonl'}
            trial_started = time.monotonic()
            with (trial_dir/'trace.jsonl').open('w', buffering=1) as trace:
                backend.trace = trace
                try:
                    prepared_baseline = backend.prepare_baseline(baseline)
                    trial['baseline'] = prepared_baseline
                    if prepared_baseline['result'] != 'PASS':
                        trial.update(outcome='baseline_failure', reason='Restored empty baseline failed')
                    else:
                        frames = backend.poses()
                        prepared = backend._prepare_objects(frames)
                        geometry = checker.check_arrangement(prepared, {name: frames[name] for name in COMPONENTS})
                        trial['proposed_objects'] = prepared
                        trial['initial_geometry'] = geometry
                        if not geometry['valid']:
                            trial.update(outcome='initial_collision', reason='Measured initialized component geometry failed')
                        else:
                            backend.set_rigid_pose(backend.objects[entry['object_id']], prepared[0]['pose_world'])
                            backend.loaded = True
                            backend.tracker.clear()
                            backend.previous_frames = None
                            backend.maximum_penetration_m = 0.
                            backend.peak_event = backend.global_peak_event = None
                            hold = backend.hold(phase='loaded_initial_settle_and_observation',
                                goal=backend.goal(RACKS), timeout=12., observation=5., abort_on_peak=True)
                            trial['settled'] = hold
                            trial['measured_snapshot'] = backend.snapshot()
                            trial['maximum_settle_penetration_m'] = backend.maximum_penetration_m
                            trial['maximum_settle_penetration_event'] = backend.peak_event
                            if backend.maximum_penetration_m >= LIMITS['peak_penetration_m']:
                                trial.update(outcome='penetration_failure', reason='Initial peak penetration limit exceeded')
                            elif not hold['passed']:
                                trial.update(outcome='settle_or_organization_failure', reason=hold.get('reason'))
                            else:
                                trial.update(outcome='screening_passed', reason='Independent initial rest and organization passed')
                                measured = screened_candidate(candidate, trial['measured_snapshot'], baseline,
                                    len(output['candidates']), f'trials/{trial_id}/result.json')
                                output['candidates'].append(measured)
                                if candidate['kind'] == 'dinner_plate':
                                    passed_plate_slots.add(candidate['slot_id'])
                except BudgetExpired as exc:
                    trial.update(outcome='timeout', reason=str(exc))
                except Exception as exc:
                    trial.update(outcome='simulation_error', reason=repr(exc), traceback=traceback.format_exc())
                finally:
                    backend.trace = None
                    trial['wall_seconds'] = time.monotonic()-trial_started
                    save(trial_dir/'result.json', trial)
            output['screening_attempts'].append({'trial_id': trial_id, 'source_candidate_id': candidate['candidate_id'],
                'outcome': trial['outcome'], 'wall_seconds': trial['wall_seconds'],
                'result_file': f'trials/{trial_id}/result.json'})
            output.update(candidate_count=len(output['candidates']), screening_wall_seconds=time.monotonic()-started)
            save(args.out_dir/'screened_candidates.json', output)
            print(f"[SCREEN] {trial_id} {candidate['kind']} {candidate['rack']} {trial['outcome']} passing={len(output['candidates'])}", flush=True)
            if trial['outcome'] in ('timeout', 'simulation_error', 'baseline_failure'):
                break
        output['complete'] = len(output['screening_attempts']) == len(ordered)
        schedule_complete = (len(output['screening_attempts']) >= args.max_candidates or
                             len(output['screening_attempts'])+len(output['screening_skips']) == len(ordered))
        output['screening_status'] = 'complete' if schedule_complete else 'budget_or_runtime_stopped'
    except Exception as exc:
        traceback.print_exc()
        output.update(screening_status='simulation_error', screening_error=repr(exc))
    finally:
        output.update(candidate_count=len(output['candidates']), screening_wall_seconds=time.monotonic()-started,
                      screening_finished_utc=datetime.now(timezone.utc).isoformat())
        save(args.out_dir/'screened_candidates.json', output)
        outcome = ('screening_complete' if output['screening_status'] == 'complete' else
                   'screening_partial' if output['candidates'] else 'screening_failure')
        save(args.out_dir/'result.json', {'outcome': outcome, 'candidate_count': len(output['candidates']),
            'attempt_count': len(output['screening_attempts']), 'wall_seconds': time.monotonic()-started,
            'failure_count': len(output['screening_attempts'])-len(output['candidates']),
            'outcome_counts': dict(Counter(item['outcome'] for item in output['screening_attempts'])),
            'source_candidate_count': len(source['candidates']), 'skipped_count': len(output['screening_skips']),
            'catalog_file': 'screened_candidates.json',
            'catalog_sha256': hashlib.sha256((args.out_dir/'screened_candidates.json').read_bytes()).hexdigest(),
            'execution_source_hashes': output['screening_execution_source_hashes'],
            'screening_status': output['screening_status'], 'error': output.get('screening_error')})
        print('[RESULT] '+output['screening_status']+' '+str(args.out_dir/'screened_candidates.json'), flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == '__main__':
    main()
