#!/usr/bin/env python3
"""Export the completed local HTML report using the existing offline browser."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--out-dir', required=True, type=Path)
    args = parser.parse_args()
    support = ROOT/'outputs/viewer_validation'
    os.environ['PLAYWRIGHT_BROWSERS_PATH'] = str(support/'browsers')
    os.environ['XDG_CACHE_HOME'] = str(support/'runtime_cache')
    os.environ['TMPDIR'] = str(support/'tmp')
    sys.path.insert(0, str(support/'deps'))
    from playwright.sync_api import sync_playwright
    source = args.run_dir.resolve()/'experiment_report.html'
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir/'technical_report.pdf'
    errors, requests = [], []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, args=['--no-sandbox'])
        context = browser.new_context(offline=True, viewport={'width': 1280, 'height': 1100})
        page = context.new_page()
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.on('request', lambda request: requests.append(request.url))
        page.goto(source.as_uri(), wait_until='load')
        assert page.locator('h1').inner_text() == 'Multi-dish initial-state experiment'
        assert page.locator('img').count() == 2
        assert page.evaluate('Array.from(document.images).every(i=>i.complete && i.naturalWidth===1920 && i.naturalHeight===1440)')
        assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
        page.screenshot(path=str(args.out_dir/'report_preview.png'))
        page.add_style_tag(content='''@media print {
          @page {size:A4;margin:12mm} body {margin:0;padding:0;max-width:none;font-size:10pt;line-height:1.35;background:white}
          h1 {font-size:22pt} table {font-size:8pt} th,td {padding:4px}
          figure {break-inside:avoid;margin:10mm 0} img {max-height:185mm;object-fit:contain}
          p {orphans:3;widows:3} pre {font-size:7pt;overflow-wrap:anywhere}
        }''')
        page.pdf(path=str(output), format='A4', print_background=True, prefer_css_page_size=True,
                 display_header_footer=True, header_template='<span></span>',
                 footer_template='<div style="font-size:8px;color:#667;width:100%;text-align:center">Joint dishwasher initial states · <span class="pageNumber"></span> / <span class="totalPages"></span></div>')
        browser.close()
    remote = [url for url in requests if url.startswith(('http://', 'https://'))]
    assert not errors and not remote, (errors, remote)
    digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    result = {'result': 'PASS', 'html_sha256': digest(source), 'pdf_sha256': digest(output),
              'export_source_sha256': digest(__file__), 'pdf_bytes': output.stat().st_size,
              'all_two_isaac_images_loaded': True, 'javascript_errors': errors, 'remote_requests': remote}
    (args.out_dir/'pdf_validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
