"""JSONL trial log of the robot harness (Kit-free: the report script reads the same schema).

One file per trial: ``data/artifacts/<run-id>/<trial-id>/trial.jsonl``. Every row carries the run and trial identity,
the context (attempt, move, dish, phase), the physics tick, the SIMULATED time (``sim_s`` = tick x 1/120 s, from
the backend's tick counter) and the WALL time (``wall_s``, monotonic seconds since the trial log opened) as two
separate labelled fields, the event name and a payload. Attempts (``attempt``) and distinct start configurations
(``config_id``, a digest of the reset poses) are separate fields, so a report can never count one as the other.
"""
from __future__ import annotations

import hashlib
import json
import math
import time

SCHEMA_VERSION = 1
KEY_FRAMES = ("pre-grasp", "close", "lift-off", "insertion start", "release", "after settle")
EVENTS = ("meta", "reset", "sample", "contact_begin", "contact_end", "key_frame", "invariant", "attempt", "move",
          "settle", "rack", "refusal", "hold", "end", "note")


def _clean(value):
    """JSON-safe copy: numpy -> python, non-finite floats -> the strings 'NaN' / 'Infinity' / '-Infinity'."""
    if hasattr(value, "tolist") and not isinstance(value, (str, bytes)):
        value = value.tolist()
    if isinstance(value, float):
        return value if math.isfinite(value) else ("NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity"))
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_clean(v) for v in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def config_id(poses, ids):
    """Digest of the start poses of ``ids`` (0.1 mm / 1e-4 quaternion): equal only for the same configuration."""
    rows = [[oid, [round(float(v), 4) for v in poses[oid]["position_m"]],
             [round(float(v), 4) for v in poses[oid]["quaternion_xyzw"]]] for oid in sorted(ids)]
    return hashlib.sha256(json.dumps(rows).encode()).hexdigest()[:12]


class TrialLog:
    """Append-only JSONL writer; ``write`` returns the 1-based line number (the report cites ``trial.jsonl:L<n>``)."""

    def __init__(self, path, *, run_id, trial_id, config, profile, clock):
        self.path = path
        self._f = open(path, "w", buffering=1)
        self.base = {"v": SCHEMA_VERSION, "run_id": run_id, "trial_id": trial_id, "config_id": config,
                     "profile": profile}
        self.clock = clock                  # () -> (tick, sim_s)
        self.t0 = time.monotonic()
        self.ctx = {"attempt": None, "move": None, "dish": None, "phase": None}
        self.lines = 0

    def set(self, **ctx):
        self.ctx.update(ctx)

    def wall(self):
        return time.monotonic() - self.t0

    def write(self, event, **payload):
        tick, sim_s = self.clock()
        row = {**self.base, **self.ctx, "tick": int(tick), "sim_s": round(float(sim_s), 5),
               "wall_s": round(self.wall(), 3), "event": event, "payload": _clean(payload)}
        self._f.write(json.dumps(row, allow_nan=False) + "\n")
        self.lines += 1
        return self.lines

    def close(self):
        if not self._f.closed:
            self._f.close()


def read(path):
    """[(line number, row)] of a trial log."""
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if line:
                out.append((n, json.loads(line)))
    return out
