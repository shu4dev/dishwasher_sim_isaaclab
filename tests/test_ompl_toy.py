# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""OMPL smoke test: RRT-Connect in a toy 2-D world with a circular obstacle."""

import math

OBSTACLE_C = (0.5, 0.5)
OBSTACLE_R = 0.25


def _is_valid(state) -> bool:
    dx, dy = state[0] - OBSTACLE_C[0], state[1] - OBSTACLE_C[1]
    return math.hypot(dx, dy) > OBSTACLE_R


def test_rrt_connect_toy_2d():
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import util as ou

    ou.setLogLevel(ou.LOG_WARN)

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)
    # ompl >= 2.0 (nanobind bindings): plain Python callables are accepted directly
    ss.setStateValidityChecker(_is_valid)
    ss.getSpaceInformation().setStateValidityCheckingResolution(0.005)

    start, goal = space.allocState(), space.allocState()
    start[0], start[1] = 0.05, 0.05
    goal[0], goal[1] = 0.95, 0.95
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))

    solved = ss.solve(5.0)
    assert solved, "RRT-Connect failed on a trivial 2-D problem"
    assert ss.haveExactSolutionPath(), "only an approximate solution was found"

    ss.simplifySolution()
    path = ss.getSolutionPath()
    length = path.length()
    straight = math.hypot(0.9, 0.9)
    assert straight <= length < 3.0, f"suspicious path length {length}"

    # densify and verify the simplified path respects the obstacle
    path.interpolate(200)
    for i in range(path.getStateCount()):
        assert _is_valid(path.getState(i)), "simplified path passes through the obstacle"
    print(f"\nOMPL toy plan OK: length {length:.3f} (straight-line {straight:.3f})")
