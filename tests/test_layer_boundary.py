# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The layer boundary is enforced here, not by convention.

Three layers, each allowed to call only downward: the task sequencer decides *what* to do, the
motion service turns that into "move from configuration A to configuration B", and the planner
package does pure configuration-space search. The whole point is that a planner cannot acquire
task knowledge, so a planner swap can never change which layouts are solvable.

That property is invisible at runtime and easy to erode with one convenient import, so these
tests read the import graph directly out of the source AST — no execution, no Kit, no mocks.
"""

import ast
import os

import pytest

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "dishsim")
PLANNERS_DIR = os.path.join(SRC, "planners")
TASK_DIR = os.path.join(SRC, "task")

#: Modules a planner may import. `config` supplies planning scalars (budgets, resolutions,
#: bounds margin) and `ur5e_kin` the joint limits that bound the state space; both are about the
#: ROBOT, not the task. Anything else means the boundary has been crossed.
PLANNER_ALLOWED = {"config", "ur5e_kin", "base", "ompl_base", "registry",
                   "rrt_connect", "rrt_star", "bit_star", "prm"}

#: Task-level names the motion layer must never see. Being handed an object registry, a slot, or
#: a placement mode is exactly how "move A to B" quietly becomes "pick up the mug".
TASK_SYMBOLS = {"OBJECTS", "TASK", "active_object_spec", "set_active_object", "ACTIVE_OBJECT",
                "SlotFrame", "derive_slots", "evaluate_placement", "OBJECT_COUNTERTOP_POSES_W"}


def _module_files(directory):
    return sorted(
        os.path.join(directory, f) for f in os.listdir(directory)
        if f.endswith(".py") and f != "__init__.py"
    )


def _imports(path):
    """Every module referenced and symbol imported, at any depth in a source file.

    Relative imports need care: ``from .. import config`` carries no module name at all, only
    an imported name, so counting ``node.module`` alone would miss exactly the package-sibling
    imports this file exists to police.

    Function-local imports count too. This package uses them deliberately to keep ``pxr``/Kit
    out of module scope, so a boundary violation could easily hide inside a function body.

    Returns:
        ``(modules, symbols)`` — referenced module names (relative ones flattened to their
        final component) and the bare names pulled from them.
    """
    tree = ast.parse(open(path).read(), filename=path)
    modules, symbols = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                modules.add(node.module)
            for a in node.names:
                symbols.add(a.name)
                # `from .. import X` / `from . import X`: X may itself be a sibling module
                if node.level:
                    modules.add(a.name)
    return {m for m in modules if m}, symbols


@pytest.mark.parametrize("path", _module_files(PLANNERS_DIR), ids=os.path.basename)
def test_planner_never_imports_task_or_scene_modules(path):
    """A planner may know about the robot and its own package. Nothing else."""
    modules, _ = _imports(path)
    local = {m.rsplit(".", 1)[-1] for m in modules if m.startswith(".") or "." not in m}
    # third-party and stdlib are fine; only intra-project imports are constrained
    project = {m for m in local if os.path.exists(os.path.join(SRC, m + ".py"))
               or os.path.isdir(os.path.join(SRC, m))}
    leaked = project - PLANNER_ALLOWED
    assert not leaked, (
        f"{os.path.basename(path)} imports {sorted(leaked)} — planners must stay task-agnostic. "
        f"If a planner needs world information, it belongs behind the world's "
        f"in_collision(q) contract, not behind a new import."
    )


@pytest.mark.parametrize("path", _module_files(PLANNERS_DIR), ids=os.path.basename)
def test_planner_never_imports_the_task_package(path):
    """The downward-only rule, stated the other way round: no planner may reach up."""
    modules, _ = _imports(path)
    assert not [m for m in modules if "task" in m.split(".")], \
        f"{os.path.basename(path)} imports the task layer — the dependency runs the other way"


def test_planner_plan_signature_is_world_start_goals():
    """The contract itself: a planner is handed a world, a start and a goal SET — no objects."""
    import inspect

    from dishsim.planners.base import Planner

    params = list(inspect.signature(Planner.plan).parameters)
    assert params[:4] == ["self", "world", "start_q", "goal_qs"], params
    assert set(params[4:]) <= {"seed", "debug"}, params


@pytest.mark.skipif(not os.path.isdir(TASK_DIR), reason="task package not present yet")
def test_motion_layer_never_imports_the_object_registry():
    """The motion service is handed geometry, never identity.

    Everything it needs about an object arrives as convex pieces plus a transform under a
    caller-chosen key, so it cannot special-case a mug, consult a placement mode, or decide an
    order. Reading ``config.OBJECTS`` here would be the first step back to a single-object
    runner with the task logic inlined.
    """
    path = os.path.join(TASK_DIR, "motion.py")
    if not os.path.exists(path):
        pytest.skip("motion.py not present yet")
    _, symbols = _imports(path)
    leaked = symbols & TASK_SYMBOLS
    assert not leaked, f"task/motion.py imports task-level symbols {sorted(leaked)}"
    src = open(path).read()
    for name in ("config.OBJECTS", "active_object_spec", "set_active_object", "config.TASK"):
        assert name not in src, f"task/motion.py references {name} — it must stay object-agnostic"


@pytest.mark.skipif(not os.path.isdir(TASK_DIR), reason="task package not present yet")
def test_sequencer_never_imports_a_planner_directly():
    """The sequencer talks to the motion layer, which owns the planner.

    Left unchecked, a sequencer that constructs its own planner starts passing task context into
    it — the exact coupling the three-layer split exists to prevent.
    """
    path = os.path.join(TASK_DIR, "sequencer.py")
    if not os.path.exists(path):
        pytest.skip("sequencer.py not present yet")
    modules, symbols = _imports(path)
    assert not [m for m in modules if "planners" in m.split(".")], \
        "sequencer.py imports dishsim.planners — go through the motion layer"
    assert "make_planner" not in symbols, "sequencer.py constructs a planner directly"


# ---- protocol conformance ------------------------------------------------------------------------
#
# The runner's ExecContext/SceneAccess implementations live in a Kit-only script, so they cannot
# be imported here. Their signatures can still be compared against the protocols by reading the
# AST — which is enough to catch the failure mode that actually happens: a parameter added to the
# protocol and to the caller, but not to the concrete implementation. That mismatch is invisible
# until a simulator run reaches the one line that uses it, minutes in.

RUNNER = os.path.join(os.path.dirname(SRC), "..", "scripts", "experiment", "run_task.py")


def _class_methods(path, class_name):
    """``{method_name: [arg names]}`` for one class in a source file."""
    tree = ast.parse(open(path).read(), filename=path)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {
                m.name: [a.arg for a in m.args.args] + [a.arg for a in m.args.kwonlyargs]
                for m in node.body if isinstance(m, ast.FunctionDef)
            }
    return None


@pytest.mark.skipif(not os.path.exists(RUNNER), reason="run_task.py not present")
@pytest.mark.parametrize(
    ("protocol_file", "protocol", "impl"),
    [
        (os.path.join(TASK_DIR, "motion.py"), "ExecContext", "_Ctx"),
        (os.path.join(TASK_DIR, "primitives.py"), "SceneAccess", "_SceneAccess"),
    ],
)
def test_runner_implements_the_protocol_signature_for_signature(protocol_file, protocol, impl):
    want = _class_methods(protocol_file, protocol)
    got = _class_methods(RUNNER, impl)
    assert want is not None, f"protocol {protocol} not found"
    assert got is not None, f"implementation {impl} not found in run_task.py"

    missing = sorted(set(want) - set(got))
    assert not missing, f"{impl} does not implement {protocol} method(s): {missing}"

    for name, params in want.items():
        assert got[name] == params, (
            f"{impl}.{name}{tuple(got[name])} does not match {protocol}.{name}{tuple(params)} — "
            f"a parameter added to the protocol but not the implementation raises TypeError only "
            f"once a simulator run reaches that call"
        )
