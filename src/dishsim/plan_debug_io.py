# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Persist a planning query and its search tree, so the planner visual reads an artifact.

Without this, the only way to draw a trial's search tree is to re-run the planner and hope the
query is reproduced — which means duplicating the runner's seed arithmetic, its goal-subset
RNG, and its collision-world settings in the visualization tool. That duplication silently
drifted in this project once already (the visual built its world with a different
``merged_cluster`` than the runner), so the query is recorded instead.

Kit-free: numpy only.
"""

import json

import numpy as np

SCHEMA_VERSION = 1

_ARRAY_FIELDS = (
    "checked_q", "checked_valid", "tree_q", "tree_tag", "tree_edges", "raw_path_q",
)


def save_plan_debug(path: str, debug, result, start_q, goal_qs, meta: dict) -> None:
    """Write one planning query + its instrumentation to ``path`` (``.npz``).

    Args:
        path: Destination ``.npz``.
        debug: The :class:`~dishsim.planners.base.PlanDebug` filled by the planner.
        result: The :class:`~dishsim.planners.base.PlanResult` it returned.
        start_q: Start configuration [6].
        goal_qs: The exact goal set handed to the planner [K, 6].
        meta: Provenance (trial, planner block, seed, cache dir, merged_cluster, object,
            scenario) — everything the visual needs to rebuild the same world.
    """
    import os  # noqa: PLC0415

    arrays = {
        "start_q": np.asarray(start_q, dtype=np.float64),
        "goal_qs": np.asarray(goal_qs, dtype=np.float64),
        "path_q": np.asarray(result.path_q if result.path_q is not None else np.zeros((0, 6))),
    }
    for name in _ARRAY_FIELDS:
        value = getattr(debug, name, None)
        if value is not None:
            arrays[name] = np.asarray(value)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "status": result.status,
        "plan_time_s": result.plan_time_s,
        "path_len_rad": result.path_len_rad,
        "goal_index": result.goal_index,
        "n_checks": int(getattr(debug, "n_checks", 0)),
        "planner_data_ok": bool(getattr(debug, "planner_data_ok", False)),
        "planner_data_error": getattr(debug, "planner_data_error", ""),
        **meta,
    }
    arrays["meta"] = np.array(json.dumps(payload))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez_compressed(path, **arrays)


def load_plan_debug(path: str) -> tuple[dict, dict]:
    """Load a saved query.

    Returns:
        ``(arrays, meta)`` — arrays keyed as written, meta as a dict.
    """
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        if meta.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(f"{path}: unsupported plan-debug schema {meta.get('schema_version')}")
        arrays = {k: z[k] for k in z.files if k != "meta"}
    return arrays, meta
