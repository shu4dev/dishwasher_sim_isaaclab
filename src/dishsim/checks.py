# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared pass/fail gate for entry-point scripts (Kit-free, stdlib-only).

``./isaaclab.sh -p`` exits 0 even when the wrapped script crashes, so the ``[RESULT]`` log
line — not the exit code — is the verdict a run is judged by (see CLAUDE.md). Scripts import
``FAILURES``/``check``/``finish`` from here; ``FAILURES`` is mutated in place (never rebound),
so ``from dishsim.checks import FAILURES`` keeps every importer on the same list.
"""

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def finish() -> None:
    """Print the ``[RESULT]`` verdict line; exit 1 if any check failed."""
    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)
