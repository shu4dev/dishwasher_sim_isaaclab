#!/usr/bin/env python3
"""Audit delivered fixed inventories and archive the organized experiment."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'frigidaire/scripts/experiment'))
from frigidaire_initial_state_experiment import digest, save, utc


def resolve(value):
    path = Path(value)
    if not path.exists() and str(path).startswith('/workspace/dishsim/'):
        path = ROOT / str(path).removeprefix('/workspace/dishsim/')
    return path


def audit_report(directory, report, accepted, source_count):
    """Tie the published presentation to the exact evidence being packaged."""
    errors = []
    def check(path, expected, label):
        if not expected or not path.is_file() or digest(path) != expected:
            errors.append('Report hash mismatch or missing file: ' + label)
    if report.get('result') != 'PASS' or report.get('problems') or report.get('missing_views'):
        errors.append('Report saved-evidence audit did not pass')
    for key, expected in (('accepted_count', len(accepted)),
                          ('unresolved_count', source_count - len(accepted)),
                          ('source_state_count', source_count)):
        if report.get(key) != expected:
            errors.append('Report count mismatch: ' + key)
    if sorted(report.get('accepted_state_ids', [])) != sorted(accepted):
        errors.append('Report accepted inventory IDs differ from packaged states')
    inputs = report.get('input_hashes', {})
    if 'summary.json' not in inputs:
        errors.append('Report does not bind the experiment summary')
    for name, sha in inputs.items():
        path = resolve(name) if Path(name).is_absolute() else directory / name
        check(path, sha, name)
    outputs = report.get('output_hashes', {})
    for name in ('organized_report.md', 'organized_report.html', 'index.html'):
        if name not in outputs:
            errors.append('Missing report output hash: ' + name)
    for name, sha in outputs.items():
        check(directory / name, sha, name)
    for view in report.get('render_evidence', []):
        check(directory / view['path'], view.get('image_sha256'), view['path'])
        check(directory / view['evidence'], view.get('evidence_sha256'), view['evidence'])
    pdf = report.get('pdf', {})
    if pdf.get('result') != 'PASS' or not pdf.get('all_images_loaded') or pdf.get('javascript_errors') or pdf.get('remote_requests'):
        errors.append('PDF export validation is missing or failed')
    check(directory / 'technical_report.pdf', pdf.get('pdf_sha256'), 'technical_report.pdf')
    check(directory / 'organized_report.html', pdf.get('html_sha256'), 'PDF source HTML')
    validation = directory / 'pdf_validation.json'
    if not validation.is_file() or json.loads(validation.read_text()) != pdf:
        errors.append('PDF validation differs from report audit')
    completion = directory / 'completion.json'
    if completion.is_file():
        check(directory / 'summary.json', json.loads(completion.read_text()).get('summary_sha256'), 'completion summary')
    return errors


def audit(directory):
    directory = Path(directory).resolve()
    summary = json.loads((directory / 'summary.json').read_text())
    errors = []
    sources = resolve(summary['source_run'])
    for rel, sha in summary['input_hashes'].items():
        path = sources / rel
        if not path.is_file() or digest(path) != sha:
            errors.append('source changed or missing: ' + rel)
    copies = json.loads((directory / 'input_copies.json').read_text())
    for record in copies.get('files', []):
        path = directory / record['copy']
        if not path.is_file() or digest(path) != record['sha256']:
            errors.append('bundled input differs: ' + record['copy'])
    expected_copies = {'inputs/source_run/' + rel for rel in summary['input_hashes']}
    if not expected_copies.issubset({record['copy'] for record in copies.get('files', [])}):
        errors.append('Bundled source evidence is incomplete')
    if len(summary['states']) != 11:
        errors.append('Expected all eleven source inventories in report')
    accepted = []
    for row in summary['states']:
        original = json.loads(resolve(row['source_state']).read_text())
        identities = {(o['object_id'], o['kind']) for o in original['objects']}
        if row['counts']['total'] != len(identities):
            errors.append(row['source_state_id'] + ': count does not match source')
        if row['status'] not in ('accepted', 'unresolved'):
            errors.append(row['source_state_id'] + ': unfinalized status')
        if row['status'] != 'accepted':
            if row.get('accepted_state'):
                errors.append(row['source_state_id'] + ': unresolved state has accepted artifact')
            continue
        path = directory / row['accepted_state']
        state = json.loads(path.read_text())
        if len(state['objects']) != len(identities) or {(o['object_id'], o['kind']) for o in state['objects']} != identities:
            errors.append(row['source_state_id'] + ': objects removed or replaced')
        originals = {o['object_id']: o for o in original['objects']}
        for obj in state['objects']:
            for name in ('mass_kg', 'size_m'):
                if obj.get(name) != originals[obj['object_id']].get(name):
                    errors.append(row['source_state_id'] + ': changed ' + name)
        primary_id = state['source_attempt']
        replay_id = state['reproduction']['source_attempt']
        if primary_id == replay_id or state['reproduction']['result'] != 'PASS':
            errors.append(row['source_state_id'] + ': lacks independent passing replay')
        for attempt_id, validation in ((primary_id, state['validation']), (replay_id, state['reproduction']['validation'])):
            saved = json.loads((directory / 'attempts' / attempt_id / 'result.json').read_text())
            if saved != validation or saved.get('outcome') != 'accepted':
                errors.append(attempt_id + ': validation differs from saved accepted result')
            for name in ('organization_initial', 'organization_closed', 'organization_reopened'):
                if not saved.get(name, {}).get('valid'):
                    errors.append(attempt_id + ': missing passing ' + name)
            for name in ('settled', 'door_closed_hold', 'reopened_hold'):
                hold = saved.get(name, {})
                if not hold.get('passed') or hold.get('observation_completed_s', 0) < 5 - 1e-9:
                    errors.append(attempt_id + ': missing continuous observation ' + name)
            for name in ('door_close_motion', 'door_open_motion'):
                if not saved.get(name, {}).get('passed'):
                    errors.append(attempt_id + ': motion failed ' + name)
            for rel, sha in saved.get('execution_source_hashes', {}).items():
                archived = directory / 'source_versions' / sha / Path(rel).name
                if not archived.is_file() or digest(archived) != sha:
                    errors.append(attempt_id + ': missing executed source ' + rel)
        accepted.append(row['source_state_id'])
    report = json.loads((directory / 'report_audit.json').read_text())
    errors.extend(audit_report(directory, report, accepted, len(summary['states'])))
    for filename in ('index.html', 'technical_report.pdf', 'completion.json'):
        if not (directory / filename).is_file():
            errors.append('Missing deliverable: ' + filename)
    return {'schema_version': 1, 'result': 'PASS' if not errors else 'FAIL', 'created_utc': utc(),
            'accepted_state_ids': accepted, 'accepted_count': len(accepted),
            'unresolved_count': len(summary['states']) - len(accepted),
            'source_files_unchanged': len(summary['input_hashes']), 'errors': errors}


def package(directory):
    directory = Path(directory).resolve()
    result = audit(directory)
    save(directory / 'delivery_audit.json', result)
    if result['result'] != 'PASS':
        raise RuntimeError('Delivery audit failed: ' + '; '.join(result['errors']))
    summary = json.loads((directory / 'summary.json').read_text())
    for relative, sha in summary.get('current_source_hashes', {}).items():
        archived = directory / 'source_versions' / sha / Path(relative).name
        if not archived.is_file() or digest(archived) != sha:
            raise ValueError('Missing current source snapshot: ' + relative)
        target = directory / 'source' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(archived.read_bytes())
    accepted_rows = [row for row in summary['states'] if row['status'] == 'accepted']
    example = max(accepted_rows, key=lambda row: row['counts']['total']) if accepted_rows else None
    instructions = ['# Reproducing the organized experiment', '',
        'Use the existing dishwasher_sim_isaaclab checkout and pinned dishsim-isaac:4.5.0 container. '
        'The package contains experiment source and evidence; the repository framework and container image are separate prerequisites.', '',
        '`source/` restores the final source snapshot to its repository paths. `source_versions/` retains '
        'the earlier executed versions; each attempt records its exact execution hashes. '
        '`inputs/source_run/` contains byte-identical original packing evidence and `inputs/assets/` contains the USD inputs.', '',
        'Run commands from the repository root. Set DISHSIM_ORGANIZED_RUN to the extracted run directory.', '']
    if example:
        example_state = json.loads((directory / example['accepted_state']).read_text())
        recorded_order = example_state['validation']['order']
        replay_order = {'UpperRack': 'upper_first', 'LowerRack': 'lower_first'}[recorded_order[0]] if isinstance(recorded_order, list) else recorded_order
        instructions += ['To replay the largest accepted inventory in a fresh Isaac process:', '', '```bash',
            'scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_organized_validate.py \\',
            '  --manifest "$DISHSIM_ORGANIZED_RUN/' + example['accepted_state'] + '" \\',
            '  --usd "$DISHSIM_ORGANIZED_RUN/inputs/assets/fdpc4221as.usdc" \\',
            '  --out-dir outputs/organized_independent_replay --max-wall-seconds 480 \\',
            '  --order ' + replay_order + ' --headless --device cpu', '```', '']
    instructions += ['A passing replay must report `outcome: accepted` and retain all organization and full-cycle observations. '
        'The existing accepted records already include distinct primary and reproduction processes. '
        'An unresolved inventory has no accepted counterpart in this bounded search.', '',
        '`checksums.sha256` covers the delivered files. The ZIP has a separate SHA-256 sidecar. '
        'HTML, PDF and images are linked to the saved results by the report and delivery audits.', '']
    (directory / 'REPRODUCE.md').write_text('\n'.join(instructions))
    # Archive the packager actually executing, including repairs after the report
    # froze its input summary. Keep that report-bound summary unchanged.
    package_relative = str(Path(__file__).resolve().relative_to(ROOT))
    package_sha = digest(__file__)
    package_archive = directory / 'source_versions' / package_sha / Path(__file__).name
    package_archive.parent.mkdir(parents=True, exist_ok=True)
    package_archive.write_bytes(Path(__file__).read_bytes())
    (directory / 'source' / package_relative).write_bytes(Path(__file__).read_bytes())
    elapsed = time.time() - summary['started_epoch']
    save(directory / 'delivery.json', dict(result, elapsed_seconds_through_delivery_preparation=elapsed,
        packaged_source_overrides={package_relative: package_sha},
        budget_seconds=summary['budget_seconds'], within_original_budget=elapsed <= summary['budget_seconds']))
    paths = sorted(p for p in directory.rglob('*') if p.is_file() and p.name != 'checksums.sha256')
    manifest = ''.join(f'{digest(p)}  {p.relative_to(directory)}\n' for p in paths)
    (directory / 'checksums.sha256').write_text(manifest)
    archive = directory.with_suffix('.zip')
    if archive.exists():
        raise FileExistsError('Refusing to overwrite finalized archive: ' + str(archive))
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for path in sorted(p for p in directory.rglob('*') if p.is_file()):
            z.write(path, arcname=str(Path(directory.name) / path.relative_to(directory)))
    with zipfile.ZipFile(archive) as z:
        corrupt = z.testzip()
        if corrupt:
            raise RuntimeError('Archive CRC error: ' + corrupt)
        count = len(z.infolist())
    archive.with_suffix('.zip.sha256').write_text(digest(archive) + '  ' + archive.name + '\n')
    print(json.dumps({'result': 'PASS', 'archive': str(archive), 'files': count, 'bytes': archive.stat().st_size,
                      'accepted_count': result['accepted_count'], 'unresolved_count': result['unresolved_count']}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    package(args.out_dir)
