#!/usr/bin/env python3
"""Trim a conventions record into the pytest fixture (Kit-free; plan 2026-09-29-easy-s0 Phase 0.5).

    code/scripts/run_py.sh code/frigidaire/scripts/setup/frigidaire_robot_conventions_fixture.py data/artifacts/<run-id>/conventions.json

Writes code/frigidaire/tests/fixtures/robot/conventions.json: every measured value of the live-asset record, minus the
per-step trajectories and contact-pair lists, plus a "source" block with the full record's sha256.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "code/frigidaire/tests/fixtures/robot/conventions.json"


def trim(src):
    raw = Path(src).read_bytes()
    r = json.loads(raw)
    if r.get("result") != "PASS":
        raise SystemExit(f"[RESULT] FAIL {src} is not a completed conventions record")
    keep = {k: r[k] for k in ("run_id", "started_utc", "label", "args", "joint_names", "deinstanced", "pad_colliders", "tcp", "pads")}
    keep["unloaded_close"] = {k: v for k, v in r["unloaded_close"].items() if k != "trajectory"}
    keep["loaded_close"] = {k: v for k, v in r["loaded_close"].items() if k not in ("trajectory", "bowl_pairs")}
    keep["press"] = {n: {k: v for k, v in p.items() if k != "link_radii"} for n, p in r["press"].items()}
    rel = Path(src).resolve().relative_to(ROOT.resolve()) if str(Path(src).resolve()).startswith(str(ROOT.resolve())) else Path(src)
    keep["source"] = {"file": str(rel), "sha256": hashlib.sha256(raw).hexdigest(),
                      "producer": "code/frigidaire/scripts/setup/frigidaire_robot_conventions.py (Kit, live asset)",
                      "trimmed": "trajectories and contact-pair lists dropped; every other value copied verbatim"}
    return keep


if __name__ == "__main__":
    OUT.write_text(json.dumps(trim(sys.argv[1]), indent=1) + "\n")
    print(f"[RESULT] PASS fixture {OUT.relative_to(ROOT)} from {sys.argv[1]}")
