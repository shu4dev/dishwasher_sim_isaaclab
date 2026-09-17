"""A PASS label cannot hide stale presentation artifacts at delivery."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'frigidaire/scripts/evaluation'))
from frigidaire_organized_package import audit_report, digest


def presentation(tmp_path):
    for name in ('summary.json', 'organized_report.md', 'organized_report.html',
                 'index.html', 'technical_report.pdf', 'before.png', 'render.json'):
        (tmp_path / name).write_text(name)
    pdf = dict(result='PASS', all_images_loaded=True, javascript_errors=[], remote_requests=[],
               pdf_sha256=digest(tmp_path / 'technical_report.pdf'),
               html_sha256=digest(tmp_path / 'organized_report.html'))
    (tmp_path / 'pdf_validation.json').write_text(json.dumps(pdf))
    (tmp_path / 'completion.json').write_text(json.dumps({'summary_sha256': digest(tmp_path / 'summary.json')}))
    return dict(result='PASS', problems=[], missing_views=[], accepted_count=1, unresolved_count=10,
                source_state_count=11, accepted_state_ids=['random_00'],
                input_hashes={'summary.json': digest(tmp_path / 'summary.json')},
                output_hashes={name: digest(tmp_path / name) for name in
                               ('organized_report.md', 'organized_report.html', 'index.html')},
                render_evidence=[dict(path='before.png', evidence='render.json',
                                      image_sha256=digest(tmp_path / 'before.png'),
                                      evidence_sha256=digest(tmp_path / 'render.json'))], pdf=pdf)


def test_current_presentation_passes(tmp_path):
    assert audit_report(tmp_path, presentation(tmp_path), ['random_00'], 11) == []


def test_stale_summary_rejected(tmp_path):
    report = presentation(tmp_path)
    (tmp_path / 'summary.json').write_text('later physics result')
    assert any('summary' in error for error in audit_report(tmp_path, report, ['random_00'], 11))


def test_stale_pdf_and_image_rejected(tmp_path):
    report = presentation(tmp_path)
    (tmp_path / 'technical_report.pdf').write_text('old report')
    (tmp_path / 'before.png').write_text('different pose')
    errors = audit_report(tmp_path, report, ['random_00'], 11)
    assert any('technical_report.pdf' in error for error in errors)
    assert any('before.png' in error for error in errors)


def test_changed_acceptance_rejected(tmp_path):
    report = presentation(tmp_path)
    errors = audit_report(tmp_path, report, ['random_01', 'random_02'], 11)
    assert any('accepted_count' in error for error in errors)
    assert any('inventory IDs' in error for error in errors)


def test_detached_pdf_validation_rejected(tmp_path):
    report = presentation(tmp_path)
    (tmp_path / 'pdf_validation.json').write_text('{}')
    assert any('PDF validation differs' in error for error in audit_report(tmp_path, report, ['random_00'], 11))
