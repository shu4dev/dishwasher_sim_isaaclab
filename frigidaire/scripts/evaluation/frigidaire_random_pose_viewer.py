#!/usr/bin/env python3
"""Build a single offline HTML file from the verified pose/mesh export.

Generate data first with frigidaire_random_pose_viewer_data.py. Vendor Three.js
0.160.1 and its MIT license under the output directory's vendor/ directory.
No finalized experiment artifact is modified by this builder.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TEMPLATE = Path(__file__).with_name("pose_viewer")


def build(data_file: Path, output: Path, vendor: Path) -> dict:
    data = json.loads(data_file.read_text())
    if data.get("schema_version") != 1 or not data.get("trials"):
        raise ValueError("Expected verified pose viewer schema version 1")
    if output.resolve().is_relative_to((ROOT / "results").resolve()):
        raise ValueError("Keep the viewer outside finalized experiment results")
    library = (vendor / "three.min.js").read_text()
    license_text = (vendor / "three.LICENSE").read_text()
    if "160" not in library[:10000] or "MIT License" not in license_text:
        raise ValueError("Expected the bundled Three.js r160 renderer and MIT license")
    # Escaping '<' makes embedded JSON safe even if labels contain '</script>'.
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    replacements = {
        "/* VIEWER_CSS */": (TEMPLATE / "viewer.css").read_text(),
        "/* VIEWER_JS */": (TEMPLATE / "viewer.js").read_text(),
        "/* VIEWER_DATA */": payload,
        "/* THREE_LIBRARY */": "/*\n" + license_text.replace("*/", "* /") + "\n*/\n" + library,
    }
    html = (TEMPLATE / "template.html").read_text()
    for marker, content in replacements.items():
        if html.count(marker) != 1:
            raise ValueError("Missing or duplicate template marker: " + marker)
        html = html.replace(marker, content)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)
    result = {"output": str(output.resolve()), "bytes": output.stat().st_size,
              "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
              "data_sha256": hashlib.sha256(data_file.read_bytes()).hexdigest(),
              "renderer": "Three.js 0.160.1 (MIT)",
              "renderer_sha256": hashlib.sha256((vendor / "three.min.js").read_bytes()).hexdigest(),
              "trials": len(data["trials"]), "offline": True}
    output.with_name("viewer_build.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT / "outputs/random_pose_viewer/viewer_data.json")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/random_pose_viewer/index.html")
    parser.add_argument("--vendor", type=Path, default=ROOT / "outputs/random_pose_viewer/vendor")
    args = parser.parse_args()
    print(json.dumps(build(args.data, args.out, args.vendor), indent=2))


if __name__ == "__main__":
    main()
