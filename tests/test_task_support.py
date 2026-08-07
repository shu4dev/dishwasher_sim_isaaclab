# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The support graph — including the negative cases, which are where it earns its keep.

A support test that only checks "A on B is detected" passes trivially with a rule as crude as
"their footprints overlap". What makes the graph usable is that it does NOT fire for objects
merely standing next to each other, and does not invent an ordering between two things resting
on the counter. Those are the cases below.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_support.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.task import layout as L
from dishsim.task import support as S
from dishsim.task.sequencer import TaskItem

COUNTER_Z = L.countertop_top_z_w() - config.ROBOT_BASE_POS_W[2]


def item(item_id, object_class, x, y, z_bottom, yaw=0.0):
    """An item of ``object_class`` whose lowest point sits at ``z_bottom`` (base frame)."""
    spec = config.OBJECTS[object_class]
    R = L.stand_rotation(spec, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (x, y, z_bottom + L.bottom_offset(spec, R))
    return TaskItem(item_id=item_id, object_class=object_class, instance=0, T_base_obj=T)


def top_of(it):
    return S.object_extent(it.object_class, it.T_base_obj)[3]


# ---- positives ----------------------------------------------------------------------------------


def test_a_stacked_object_is_detected():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    g = S.build_support_graph([a, b], backend="geometric")
    assert g.supports["a"] == {"b"}
    assert g.rests_on["b"] == {"a"}
    assert not g.pickable("a") and g.pickable("b")


def test_a_slightly_offset_stack_still_counts():
    """Physics never lands an object perfectly centred; a few mm of offset must not break it."""
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.757, 0.004, top_of(a))
    g = S.build_support_graph([a, b], backend="geometric")
    assert g.supports["a"] == {"b"}


def test_an_object_resting_on_two_others_records_both():
    a = item("a", "cup", 0.74, -0.02, COUNTER_Z)
    b = item("b", "cup", 0.74, 0.02, COUNTER_Z)
    c = item("c", "cup", 0.74, 0.0, max(top_of(a), top_of(b)))
    g = S.build_support_graph([a, b, c], backend="geometric")
    assert g.rests_on["c"] == {"a", "b"}
    assert g.supports["a"] == {"c"} and g.supports["b"] == {"c"}
    assert not g.pickable("a") and not g.pickable("b") and g.pickable("c")


def test_three_high_stack_records_only_direct_edges():
    """Direct-only edges are what make "pickable == empty entry" a lookup rather than a search."""
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    c = item("c", "cup", 0.75, 0.0, top_of(b))
    g = S.build_support_graph([a, b, c], backend="geometric")
    assert g.supports["a"] == {"b"}, "a must list only its DIRECT dependant"
    assert g.supports["b"] == {"c"}
    assert g.supports["c"] == set()
    assert S.clearing_order(g) == ["c", "b", "a"]


# ---- negatives: where a naive rule goes wrong ----------------------------------------------------


def test_objects_side_by_side_do_not_support_each_other():
    a = item("a", "cup", 0.75, 0.00, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.08, COUNTER_Z)
    g = S.build_support_graph([a, b], backend="geometric")
    assert g.supports["a"] == set() and g.supports["b"] == set()
    assert g.grounded == {"a", "b"}
    assert g.pickable("a") and g.pickable("b")


def test_touching_objects_on_the_counter_do_not_support_each_other():
    """Footprints overlapping while BOTH sit on the counter is jostling, not stacking."""
    a = item("a", "cup", 0.75, 0.00, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.03, COUNTER_Z)  # heavily overlapping footprints
    g = S.build_support_graph([a, b], backend="geometric")
    assert g.supports["a"] == set() and g.supports["b"] == set()


def test_a_tall_object_beside_a_short_one_is_not_supported_by_it():
    """The trap: a tall neighbour's TOP can sit at the same height as another object's bottom."""
    short = item("short", "cup", 0.75, 0.00, COUNTER_Z)
    tall = item("tall", "tumbler", 0.75, 0.09, COUNTER_Z)
    g = S.build_support_graph([short, tall], backend="geometric")
    assert g.supports["short"] == set() and g.supports["tall"] == set()
    assert g.grounded == {"short", "tall"}


def test_an_object_floating_above_another_is_not_resting_on_it():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a) + 0.05)  # well clear of the gap tolerance
    g = S.build_support_graph([a, b], backend="geometric")
    assert g.supports["a"] == set()


def test_the_vertical_gap_tolerance_is_respected_on_both_sides():
    gap = config.TASK["support_gap_m"]
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    just_on = item("b", "cup", 0.75, 0.0, top_of(a) + gap * 0.5)
    just_off = item("c", "cup", 0.75, 0.0, top_of(a) + gap * 3.0)
    assert S.build_support_graph([a, just_on], backend="geometric").supports["a"] == {"b"}
    assert S.build_support_graph([a, just_off], backend="geometric").supports["a"] == set()


# ---- structural guarantees ------------------------------------------------------------------------


def test_the_graph_is_always_acyclic():
    """Orienting by height makes a cycle impossible however the poses are arranged."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        items = [item(f"i{k}", "cup", float(rng.uniform(0.70, 0.80)),
                      float(rng.uniform(-0.05, 0.05)),
                      COUNTER_Z + float(rng.uniform(0.0, 0.15))) for k in range(4)]
        g = S.build_support_graph(items, backend="geometric")
        S.clearing_order(g)  # raises on a cycle


def test_clearing_order_rejects_a_hand_made_cycle():
    """The cycle guard must actually fire, or it is decoration."""
    g = S.SupportGraph(supports={"a": {"b"}, "b": {"a"}}, rests_on={"a": {"b"}, "b": {"a"}})
    with pytest.raises(ValueError, match="cycle"):
        S.clearing_order(g)


def test_an_empty_scene_is_handled():
    g = S.build_support_graph([], backend="geometric")
    assert g.supports == {} and S.clearing_order(g) == []


# ---- the contact backend ---------------------------------------------------------------------------


def test_contact_backend_orients_pairs_by_height():
    """Contact is symmetric; support is not. The higher object rests on the lower."""
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    forces = {"a": {"b": 2.0}, "b": {"a": 2.0}}
    g = S.build_support_graph([a, b], backend="contact", forces=forces)
    assert g.supports["a"] == {"b"} and g.rests_on["b"] == {"a"}


def test_contact_backend_ignores_forces_below_threshold():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    g = S.build_support_graph([a, b], backend="contact", forces={"a": {"b": 1e-4}})
    assert g.supports["a"] == set()


def test_contact_backend_ignores_unknown_peers():
    """A sensor also filters against gripper prims; those must not become support edges."""
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    g = S.build_support_graph([a], backend="contact",
                              forces={"a": {"left_inner_finger": 9.0, "ghost": 3.0}})
    assert g.supports["a"] == set() and g.grounded == {"a"}


def test_both_backends_report_disagreement_rather_than_hiding_it():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    # geometry sees the stack; contacts report nothing (a dropped/undetected pair)
    g = S.build_support_graph([a, b], backend="both", forces={})
    assert g.supports["a"] == {"b"}, "geometry stays authoritative"
    assert any(d["supporter"] == "a" and d["resting"] == "b" for d in g.disagreements)


def test_both_backends_agree_when_contacts_confirm_the_stack():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    b = item("b", "cup", 0.75, 0.0, top_of(a))
    g = S.build_support_graph([a, b], backend="both", forces={"a": {"b": 2.0}, "b": {"a": 2.0}})
    assert g.disagreements == []


def test_nested_objects_are_detected_as_supported():
    """Cups stack by NESTING: the upper cup's bottom ends up well below the lower cup's rim.

    A rule that requires "upper bottom level with lower top" calls a real, settled, physically
    supported stack unsupported — measured against PhysX contacts on an actual run, where the
    two backends disagreed on exactly this pair.
    """
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    bot_a, top_a = S.object_extent("cup", a.T_base_obj)[2:]
    nested = item("b", "cup", 0.75, 0.0, bot_a + 0.4 * (top_a - bot_a))  # sunk 40% into a
    g = S.build_support_graph([a, nested], backend="geometric")
    assert g.supports["a"] == {"b"}, "a nested cup must count as supported"
    assert g.rests_on["b"] == {"a"}


def test_nesting_and_contact_backends_agree():
    a = item("a", "cup", 0.75, 0.0, COUNTER_Z)
    bot_a, top_a = S.object_extent("cup", a.T_base_obj)[2:]
    nested = item("b", "cup", 0.75, 0.0, bot_a + 0.4 * (top_a - bot_a))
    g = S.build_support_graph([a, nested], backend="both",
                              forces={"a": {"b": 3.0}, "b": {"a": 3.0}})
    assert g.disagreements == [], f"backends still disagree: {g.disagreements}"
