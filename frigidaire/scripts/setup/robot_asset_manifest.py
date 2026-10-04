#!/usr/bin/env python3
"""Write assets/robots/MANIFEST.sha256 (plan 2026-09-29-easy-s0, decision D11; Kit-free, no download).

    scripts/run_py.sh frigidaire/scripts/setup/robot_asset_manifest.py [--check]

One line per mirrored file (sha256, size, path relative to assets/robots), plus a header naming the two layers that
mirror_robot_usd.sh converted from crate to text usda (their md5 too, for the values recorded in
plans/2026-09-29-easy-s0-map.md) and the usd-core version that converted them when it is recorded. --check verifies
the tree against the manifest and prints [RESULT] PASS/FAIL.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
DEST = ROOT / "assets/robots"
MANIFEST = DEST / "MANIFEST.sha256"
CONVERTED = ("Assets/Isaac/6.0/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
             "Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_robot_schema.usd")
VERSION_FILE = DEST / "CONVERSION.json"      # written by mirror_robot_usd.sh when it converts (usd-core version, date)


def digest(path, algo="sha256"):
    h = hashlib.new(algo)
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def files():
    return sorted(p for p in DEST.rglob("*") if p.is_file() and p.name not in (MANIFEST.name, VERSION_FILE.name))


def build():
    lines = ["# sha256 manifest of the mirrored Isaac 6.0 UR5e + Robotiq 2F-85 (frigidaire/scripts/setup/mirror_robot_usd.sh)",
             "# converted-to-usda layers (crate 0.9 -> text): " + ", ".join(CONVERTED)]
    for relp in CONVERTED:
        p = DEST / relp
        if p.exists():
            head = p.read_bytes()[:8]
            lines.append(f"#   {relp}: {'text usda' if head.startswith(b'#usda') else 'binary crate'}, md5 {digest(p, 'md5')}, {p.stat().st_size} B")
    if VERSION_FILE.exists():
        lines.append("# conversion: " + json.dumps(json.loads(VERSION_FILE.read_text())))
    else:
        lines.append("# conversion: usd-core version UNRECORDED (the 2026-09-29 throwaway venv was deleted; D11 forbids a re-download)")
    for p in files():
        lines.append(f"{digest(p)}  {p.stat().st_size:>9d}  {p.relative_to(DEST)}")
    return "\n".join(lines) + "\n"


def check():
    want = {}
    for line in MANIFEST.read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        sha, size, relp = line.split(maxsplit=2)
        want[relp] = (sha, int(size))
    have = {str(p.relative_to(DEST)): (digest(p), p.stat().st_size) for p in files()}
    missing = sorted(set(want) - set(have))
    extra = sorted(set(have) - set(want))
    changed = sorted(k for k in set(want) & set(have) if want[k] != have[k])
    ok = not (missing or extra or changed)
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} manifest check: {len(want)} listed, missing {missing}, extra {extra}, changed {changed}")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    if a.check:
        sys.exit(check())
    MANIFEST.write_text(build())
    print(f"[RESULT] PASS wrote {MANIFEST.relative_to(ROOT)} ({len(files())} files)")
