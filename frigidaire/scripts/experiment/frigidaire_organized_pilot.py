#!/usr/bin/env python3
"""A coordinated parallel full-state pilot; the main coordinator must be paused.

Uses the same validator, original wall deadline, and standard attempt evidence.
The resumed coordinator imports completed attempts instead of repeating them.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import types

from frigidaire_organized_experiment import Organizer, measured_state, ROOT, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    directory = args.run_dir.resolve()
    summary = json.loads((directory / 'summary.json').read_text())
    original = json.loads(args.manifest.read_text())
    runner = Organizer.__new__(Organizer)
    runner.directory = directory
    runner.deadline = summary['started_epoch'] + summary['budget_seconds']
    runner.args = types.SimpleNamespace(usd=ROOT / 'build/frigidaire_collection/usd/fdpc4221as.usdc', attempt_seconds=480.)
    runner.summary = {}
    deadline = summary['started_epoch'] + summary['budget_seconds'] * 105 / 120
    outcomes = []

    def invoke(manifest, order):
        ids = [int(p.name.removeprefix('attempt_')) for p in (directory / 'attempts').glob('attempt_*')]
        identity = f'attempt_{max(ids, default=-1) + 1:04d}'
        manifest.update(attempt_id=identity, order=order)
        print('[PILOT ATTEMPT] ' + identity + ' ' + manifest['purpose'], flush=True)
        result = runner.invoke(directory / 'attempts' / identity, manifest=manifest, order=order, deadline=deadline)
        outcomes.append({'attempt_id': identity, 'outcome': result['outcome'], 'purpose': manifest['purpose']})
        save(args.manifest.parent / 'pilot_result.json', {'attempts': outcomes, 'finalized_by_main_coordinator': False})
        print('[PILOT OUTCOME] ' + identity + ' ' + result['outcome'], flush=True)
        return result

    for order in ('upper_first', 'lower_first'):
        manifest = deepcopy(original)
        manifest.update(purpose='organized', reproduction_of=None)
        result = invoke(manifest, order)
        if result['outcome'] == 'accepted':
            source = Path(manifest['source_state'])
            state = measured_state(manifest, result, source)
            state.update(state_id=manifest['source_state_id'], source_attempt=manifest['attempt_id'])
            provisional = directory / 'provisional_states' / f'{state["state_id"]}_{manifest["attempt_id"]}.json'
            save(provisional, state)
            replay_manifest = deepcopy(manifest)
            replay_manifest.update(objects=state['objects'], purpose='reproduction', reproduction_of=manifest['attempt_id'])
            replay = invoke(replay_manifest, order)
            state['reproduction'] = {'result': 'PASS' if replay['outcome'] == 'accepted' else 'FAIL',
                                    'source_attempt': replay_manifest['attempt_id'], 'validation': replay}
            save(provisional, state)
            break
        if result['outcome'] != 'closure_failure' or not result.get('initial_snapshot'):
            break
    print('[RESULT] pilot_complete', flush=True)


if __name__ == '__main__':
    main()
