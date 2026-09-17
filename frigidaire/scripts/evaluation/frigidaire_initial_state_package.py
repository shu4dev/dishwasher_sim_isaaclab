#!/usr/bin/env python3
"""Package audited initial states, reports, images and executable provenance."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time
import zipfile

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            sha.update(block)
    return sha.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--pdf-dir', type=Path, required=True)
    args = parser.parse_args()
    directory = args.out_dir.resolve()
    summary = json.loads((directory/'summary.json').read_text())
    assert summary['status'] == 'complete'
    for name in ('report_audit.json', 'delivery_audit.json'):
        assert json.loads((directory/name).read_text())['result'] == 'PASS', name
    pdf_audit = json.loads((args.pdf_dir/'pdf_validation.json').read_text())
    assert pdf_audit['result'] == 'PASS'
    assert pdf_audit['html_sha256'] == digest(directory/'experiment_report.html')
    assert pdf_audit['pdf_sha256'] == digest(args.pdf_dir/'technical_report.pdf')
    for name in ('technical_report.pdf', 'pdf_validation.json', 'report_preview.png'):
        shutil.copy2(args.pdf_dir/name, directory/name)
    shutil.copy2(directory/'experiment_report.html', directory/'index.html')
    sources = list((ROOT/'frigidaire/src/dishsim_frigidaire').glob('initial_state*.py'))
    sources += list((ROOT/'frigidaire/scripts/experiment').glob('frigidaire_initial_state*.py'))
    sources += list((ROOT/'frigidaire/scripts/evaluation').glob('frigidaire_initial_state*.py'))
    sources += list((ROOT/'frigidaire/tests').glob('test_initial_state*.py'))
    sources += [ROOT/'frigidaire/tests/test_frigidaire_initial_state_reporting.py',
                ROOT/'frigidaire/scripts/experiment/run_initial_states.sh',
                ROOT/'frigidaire/docs/initial_state_experiment.md']
    for source in sources:
        target = directory/'final_tools'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    original = ROOT/'results/random_poses/frigidaire/complete_20260910_seed0'
    checked = 0
    for line in (original/'checksums.sha256').read_text().splitlines():
        expected, relative = line.split(None, 1)
        path = (original/relative.lstrip(' *')).resolve()
        assert original.resolve() in path.parents
        assert digest(path) == expected, relative
        checked += 1
    elapsed = time.time()-datetime.fromisoformat(summary['started_utc']).timestamp()
    delivery = {'schema_version': 1, 'result': 'PASS', 'created_utc': datetime.now(timezone.utc).isoformat(),
                'highest_count': summary['highest_count'], 'randomized_count': summary['randomized_count'],
                'original_single_dish_files_unchanged': checked, 'copied_source_files': len(sources),
                'elapsed_seconds_through_delivery_preparation': elapsed,
                'budget_seconds': summary['budget_seconds'], 'within_original_budget': elapsed <= summary['budget_seconds']}
    (directory/'delivery.json').write_text(json.dumps(delivery, indent=2)+'\n')
    files = sorted(path for path in directory.rglob('*') if path.is_file() and path.name != 'checksums.sha256')
    checksum = directory/'checksums.sha256'
    checksum.write_text(''.join(f'{digest(path)}  {path.relative_to(directory)}\n' for path in files))
    archive = directory.with_suffix('.zip')
    if archive.exists():
        raise FileExistsError(f'Refusing to overwrite {archive}')
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as output:
        for path in [*files, checksum]:
            output.write(path, Path(directory.name)/path.relative_to(directory))
    with zipfile.ZipFile(archive) as check:
        assert check.testzip() is None
    archive.with_suffix('.zip.sha256').write_text(f'{digest(archive)}  {archive.name}\n')
    print(json.dumps(dict(delivery, artifact_files=len(files)+1, archive=str(archive), archive_bytes=archive.stat().st_size)))


if __name__ == '__main__':
    main()
