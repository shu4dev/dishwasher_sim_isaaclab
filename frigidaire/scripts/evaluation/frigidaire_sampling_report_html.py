#!/usr/bin/env python3
"""Render the sampling technical report as a standalone, printable HTML file.

Uses existing container Markdown. Embeds figures and rewrites source links to
the shared workspace. Does not modify finalized experiment evidence.
"""
from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import quote

import markdown

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "frigidaire/docs/random_pose_sampling_technical_report.md"
OUTPUT = ROOT / "outputs/sampling_technical_report"

CSS = """
*{box-sizing:border-box}html{background:#edf1ee}body{max-width:1000px;margin:30px auto;
padding:48px 60px;background:white;color:#243632;font:15px/1.7 system-ui,sans-serif;
box-shadow:0 2px 25px #28463210}h1{font-size:30px;line-height:1.2;letter-spacing:-.7px;
font-weight:600;border-bottom:3px solid #217366;padding-bottom:22px;margin:0 0 20px}
p{margin:14px 0}p:has(>strong:first-child){margin-top:25px}strong{font-weight:650}
a{color:#217366;text-underline-offset:3px}code{font:12px/1.5 ui-monospace,monospace;
overflow-wrap:anywhere}pre{background:#f2f5f1;padding:14px 18px;border-left:3px solid #85a894;
white-space:pre-wrap;overflow-wrap:anywhere;line-height:1.5}pre code{font-size:12px}
table{border-collapse:collapse;width:100%;margin:20px 0;font-size:12px;line-height:1.5;
table-layout:auto}th{background:#e9f0e9;text-align:left;font-weight:650}th,td{padding:9px 10px;
border-bottom:1px solid #dce3dd;vertical-align:top}tr:nth-child(even) td{background:#f8faf7}
img{max-width:100%;display:block;margin:25px auto}footer{border-top:1px solid #dce3dd;
padding-top:16px;margin-top:32px;font-size:11px;color:#637a6a}.print-button{border:1px solid #217366;
padding:9px 16px;background:#217366;color:#fff;border-radius:5px;cursor:pointer;float:right;
margin:0 0 20px 20px}::selection{background:#d4e6da}
@media(max-width:700px){body{margin:0;padding:25px 18px;font-size:14px}h1{font-size:26px}
table{font-size:10px}th,td{padding:6px}pre code{font-size:10px}}
@page{size:A4;margin:16mm 15mm 18mm}
@media print{html{background:white}body{margin:0;padding:0;max-width:none;box-shadow:none;
font-family:Arial,sans-serif;font-size:10pt;line-height:1.48}h1{font-size:21pt}p{orphans:3;widows:3;
margin:9pt 0}p:has(>strong:first-child){margin-top:15pt;break-after:avoid}pre{font-size:8pt;
padding:8pt 10pt;break-inside:avoid}pre code,code{font-size:8pt}table{font-size:8pt;margin:12pt 0;
line-height:1.35}th,td{padding:5pt 6pt}thead{display:table-header-group}tr{break-inside:avoid}
img{max-height:90mm;object-fit:contain;break-inside:avoid;margin:12pt auto}.print-button{display:none}
a{text-decoration:none;color:#246257}footer{font-size:8pt}*{-webkit-print-color-adjust:exact}}
"""


def main():
    content = markdown.markdown(SOURCE.read_text(), extensions=["tables", "fenced_code", "sane_lists"])

    def rewrite(match):
        attr, target = match.group(1), match.group(2)
        if target.startswith(("https://", "http://", "#")):
            return match.group(0)
        target_path = (SOURCE.parent / target).resolve()
        if not target_path.is_file():
            raise FileNotFoundError(target_path)
        if attr == "src":
            encoded = base64.b64encode(target_path.read_bytes()).decode("ascii")
            return f'src="data:image/png;base64,{encoded}"'
        # Docker and host see the same files at different paths. Links in the
        # user-facing document point to the known shared host checkout.
        try:
            relative = target_path.relative_to(ROOT.resolve())
        except ValueError:
            original_relative = (SOURCE.parent / target).absolute().relative_to(ROOT)
            relative = original_relative
        host_path = Path("/home/brianshu/dishwasher_sim_isaaclab") / relative
        return 'href="file://' + quote(str(host_path)) + '"'

    content = re.sub(r'(href|src)="([^"]+)"', rewrite, content)
    title = "Technical report: sampling dish poses in a dishwasher"
    html = ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>{title}</title><style>{CSS}</style></head><body>'
            '<button class="print-button" onclick="window.print()">Print / Save PDF</button>'
            + content + '<footer>Derived technical analysis of completed simulation evidence. '
            'Run complete_20260910_seed0. No new physical trials.</footer></body></html>')
    OUTPUT.mkdir(parents=True, exist_ok=True)
    output = OUTPUT / "index.html"
    output.write_text(html)
    (OUTPUT / "technical_report.md").write_text(SOURCE.read_text())
    record = {"html_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
              "markdown_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              "statistics_sha256": hashlib.sha256((OUTPUT / "statistics.json").read_bytes()).hexdigest(),
              "figure_sha256": hashlib.sha256((OUTPUT / "acceptance_by_cell.png").read_bytes()).hexdigest(),
              "html_bytes": output.stat().st_size, "word_count_approx": len(SOURCE.read_text().split()),
              "new_simulation": False}
    (OUTPUT / "report_build.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
