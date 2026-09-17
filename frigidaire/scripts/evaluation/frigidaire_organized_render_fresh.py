#!/usr/bin/env python3
"""Render each measured view in its own Isaac process, with no shared image history."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'frigidaire/scripts/experiment'))
from frigidaire_initial_state_experiment import run_process, save, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    directory = args.out_dir.resolve()
    summary = json.loads((directory / 'summary.json').read_text())
    deadline = summary['started_epoch'] + summary['budget_seconds']
    jobs = []
    for row in summary['states']:
        jobs.append((row['source_state_id'], 'before', Path(row['source_state']), Path(row['source_state'])))
        if row['status'] == 'accepted':
            jobs.append((row['source_state_id'], 'after', directory / row['accepted_state'], Path(row['source_state'])))
    def render(job):
        identity, view, state, source = job
        name = identity + '_' + view
        command = [str(ROOT / 'scripts/run_kit.sh'), str(ROOT / 'frigidaire/scripts/evaluation/frigidaire_organized_render.py'),
                   '--state', str(state), '--source-state', str(source), '--view', view,
                   '--out-dir', str(directory / 'renders'), '--headless', '--enable_cameras', '--device', 'cpu']
        process = run_process(command, directory / 'fresh_render_logs' / (name + '.log'),
                              max(1., min(150., deadline - time.time() - 90.)))
        path = directory / 'renders' / (name + '_render_evidence.json')
        evidence = json.loads(path.read_text()) if path.exists() else {}
        passed = evidence.get('result') == 'PASS' and not process.get('timed_out') and process.get('exit_code') == 0
        record = {'source_state_id': identity, 'view': view, 'result': 'PASS' if passed else 'FAIL',
                  'process': process, 'command': command, 'evidence': str(path.relative_to(directory)),
                  'evidence_sha256': digest(path) if path.exists() else None}
        print('[FRESH RENDER] ' + name + ' ' + record['result'], flush=True)
        return record
    records = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(render, job) for job in jobs]):
            records.append(future.result())
            save(directory / 'fresh_render_process.json', {'result': 'RUNNING', 'views': records})
    result = {'result': 'PASS' if all(r['result'] == 'PASS' for r in records) else 'FAIL',
              'method': 'One fresh Isaac process per view; two concurrent workers; no shared temporal image history',
              'view_count': len(records), 'views': records, 'finished_epoch': time.time(),
              'within_original_budget': time.time() <= deadline}
    save(directory / 'fresh_render_process.json', result)
    print('[RESULT] ' + result['result'], flush=True)


if __name__ == '__main__':
    main()
