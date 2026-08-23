# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The task layer: decides *what* to do, and calls the motion layer to do it.

Three layers, each calling only downward:

.. code-block:: text

    TaskSequencer   which object next, in what order, into which slot
      |             owns the item list, support graph, grasp availability, recovery
      v
    MotionService   "move from configuration A to configuration B" + the robot-side
      |             primitives that are not planning (servo, gripper, payload, obstacles)
      v             knows NOTHING about object identity, stacking, or task goals
    Planner         pure configuration-space search (dishsim.planners) — never edited here

The boundary is enforced by ``tests/test_layer_boundary.py``, not by convention: the planner
package may not import task or scene modules, and :mod:`dishsim.task.motion` may not import the
object registry. Anything the motion layer is told about an object arrives as raw geometry
(convex pieces plus a transform) under a caller-chosen opaque key.
"""
