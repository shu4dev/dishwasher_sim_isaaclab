# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The pivot's core boundary, enforced mechanically: planning stays Kit-free.

Only the scene-construction modules may import ``isaaclab``/``pxr``/``omni`` at module
scope; everything else (config, geometry readers, collision world, placement, capacity,
fill_plan, task/*) must import in any plain Python process. Function-local Kit imports
(geometry's extraction half, media's camera rig) stay legal.
"""

import ast
import os

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "dishsim")
KIT_ROOTS = ("isaaclab", "isaaclab_physx", "pxr", "omni", "carb")
ALLOWED = {"scene.py", "machine.py"}


def _module_scope_imports(path: str) -> set[str]:
    tree = ast.parse(open(path).read(), filename=path)
    roots = set()
    for node in tree.body:  # module scope only — function-local imports are legal
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            roots.add(node.module.split(".")[0])
    return roots


def test_only_scene_modules_import_kit_at_module_scope():
    offenders = {}
    for dirpath, _, files in os.walk(SRC):
        for fname in files:
            if not fname.endswith(".py"):
                continue
            path = os.path.join(dirpath, fname)
            rel = os.path.relpath(path, SRC)
            hits = _module_scope_imports(path) & set(KIT_ROOTS)
            if hits and rel not in ALLOWED:
                offenders[rel] = sorted(hits)
    assert not offenders, f"Kit imports at module scope outside {sorted(ALLOWED)}: {offenders}"


def test_the_allowed_set_is_honest():
    # if scene/machine ever go Kit-free, shrink ALLOWED instead of leaving a stale exemption
    for rel in ALLOWED:
        hits = _module_scope_imports(os.path.join(SRC, rel)) & set(KIT_ROOTS)
        assert hits, f"{rel} no longer imports Kit at module scope — remove it from ALLOWED"
