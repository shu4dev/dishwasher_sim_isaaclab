# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack-manipulation geometry: engaging a rack's front handle and sliding it (Kit-free).

The both_in / both_out trials start with a ``rack_action``: the robot pulls the lower rack out
(or pushes the upper rack in) before picking and placing. The methodology mirrors the mug weld
— visible, gated contact with guaranteed actuation: the gripper engages the rack's front
handle while the rack joint's DRIVE target ramps to the action target and the TCP tracks the
moving handle. This module supplies the pure geometry: handle poses, engage/pre-engage TCP
poses, and a collision-validated arm-configuration path synchronized with the slide (using
:meth:`CollisionWorld.set_static_offset` to pose the rack at each intermediate extension).

Frames: everything in the robot-base frame, meters, XYZW. A rack's cached ``T_base_body``
(manifest) is its pose at the CACHED extension; sliding by ``delta`` along the prismatic axis
moves the body by ``delta`` along its own +y (joint localRot is identity).
"""

import math

import numpy as np

from . import config
from .transforms import T_inv, make_T
from .ur5e_kin import ik_wrist3_all


def t_wrist3_tcp() -> np.ndarray:
    """wrist_3_link -> TCP transform from the frozen config constants (Kit-free twin of
    scene.t_wrist3_tcp — scene.py imports Isaac Lab at module scope and cannot load here)."""
    return make_T(config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT)


def handle_point_body(params: dict) -> np.ndarray:
    """Grip-rod center of a rack's front handle, in the rack BODY (= design) frame."""
    h = params["handle"]
    return np.array([(h["x"][0] + h["x"][1]) / 2.0, 0.0025, h["grip_z"]])


def slide_axis_base(T_base_body: np.ndarray) -> np.ndarray:
    """The rack's prismatic axis (+body-y) in the base frame."""
    return T_base_body[:3, :3] @ np.array([0.0, 1.0, 0.0])


def engage_pose(
    T_base_body: np.ndarray, params: dict, action: dict, cached_ext: float, flip: bool = False
) -> np.ndarray:
    """TCP pose engaging the handle at the rack's CURRENT (cached-extension) pose.

    A human reaches a rack handle HORIZONTALLY from the front (a top-down wrist would stand in
    the stowed upper rack's band — measured to collide for every IK branch). Tool z points
    machine-inward along the slide axis, tool x runs along the grip rod (the jaw-free axis of
    the calibrated grasp), so the pads close vertically across the rod; the TCP sits 20 mm on
    the robot side of the rod (the rim-vs-TCP relation of the calibrated pinch). ``flip``
    mirrors the rod-axis sign (wrist-flip alternative for IK). In ``push`` mode the closed
    fist is instead offset 18 mm to the trailing side of the rod so the fingertips press it
    along the slide direction instead of wrapping it.
    """
    p_rod = (T_base_body @ np.append(handle_point_body(params), 1.0))[:3]
    inward = slide_axis_base(T_base_body)  # body +y == machine-inward for this asset
    z_tcp = inward - np.array([0.0, 0.0, 1.0]) * inward[2]
    z_tcp = z_tcp / np.linalg.norm(z_tcp)
    x_rod = T_base_body[:3, :3] @ np.array([1.0, 0.0, 0.0])
    x_tcp = x_rod - np.dot(x_rod, z_tcp) * z_tcp
    x_tcp = x_tcp / np.linalg.norm(x_tcp)
    if flip:
        x_tcp = -x_tcp
    y_tcp = np.cross(z_tcp, x_tcp)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x_tcp, y_tcp, z_tcp
    T[:3, 3] = p_rod - z_tcp * 0.020
    if action["mode"] == "push":
        direction = slide_axis_base(T_base_body) * math.copysign(1.0, float(action["to"]) - cached_ext)
        T[:3, 3] = T[:3, 3] - direction * 0.018
    return T


def approach_dir(T_base_body: np.ndarray) -> np.ndarray:
    """The horizontal machine-inward unit vector — pre-engage poses back off along MINUS this."""
    inward = slide_axis_base(T_base_body)
    d = inward - np.array([0.0, 0.0, 1.0]) * inward[2]
    return d / np.linalg.norm(d)


def slide_arm_path(
    world,
    T_base_body: np.ndarray,
    params: dict,
    action: dict,
    cached_ext: float,
    q_near: np.ndarray,
    waypoints: int = 21,
    flip: bool = False,
) -> np.ndarray | None:
    """Collision-validated arm path tracking the handle through the slide.

    For each interpolated extension the rack's static pieces are re-posed
    (``world.set_static_offset``) and the tracking TCP pose is solved with the analytic IK,
    keeping the branch closest to the previous waypoint. Returns [waypoints, 6] arm
    configurations (s = 0 is the engage pose), or None if any waypoint has no valid branch.
    The world's rack pose is restored to the cached extension before returning.
    """
    body = action["body"]
    e0 = float(cached_ext)
    e1 = float(action["to"])
    T_w3_tcp_inv = T_inv(t_wrist3_tcp())
    T_engage = engage_pose(T_base_body, params, action, e0, flip=flip)
    axis = slide_axis_base(T_base_body)

    path = []
    q_prev = np.asarray(q_near, dtype=float)
    try:
        # the engaged hand wraps/presses the moved rack's handle by design — that body is
        # dropped from the FCL checks here (the live contact gate covers arm-vs-rack during
        # execution); every OTHER obstacle stays checked at each intermediate extension
        world.set_static_enabled(body, False)
        for i in range(waypoints):
            s = i / (waypoints - 1)
            delta = (e1 - e0) * s
            T_tcp = T_engage.copy()
            T_tcp[:3, 3] = T_engage[:3, 3] + axis * delta
            sols = ik_wrist3_all(T_tcp @ T_w3_tcp_inv, q_seed=q_prev)
            # the DRIVE lags its ramp: the real rack rides anywhere between the target
            # extension and RACK_LAG_BAND_M behind it (toward the start) — the arm must
            # clear the moved rack across that whole band, not at the nominal pose only
            lag = math.copysign(min(config.RACK_LAG_BAND_M, abs(delta)), -(e1 - e0))
            best = None
            for q in sorted(sols, key=lambda q: float(np.abs(q - q_prev).max())):
                if i > 0 and float(np.abs(q - q_prev).max()) > config.SLIDE_MAX_STEP_RAD:
                    break  # sorted by closeness: everything after is farther still
                if world.in_collision(q):
                    continue
                clear = True
                for d_chk in (delta, delta + lag):
                    world.set_static_offset(body, d_chk)
                    if _arm_hits_moved_rack(world, body):
                        clear = False
                        break
                if clear:
                    best = q
                    break
            if best is None:
                return None
            path.append(best)
            q_prev = best
    finally:
        world.set_static_offset(body, 0.0)
        world.set_static_enabled(body, True)
    return np.array(path)


#: arm links that must clear the MOVED rack during the slide (the gripper cluster is exempt
#: by design — it wraps the handle; the pads press it)
_SLIDE_ARM_LINKS = ("forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")


def _arm_hits_moved_rack(world, body: str) -> bool:
    """Distal arm links vs the manager-disabled moving rack, at the arm's LAST-POSED config.

    ``slide_arm_path`` drops the moved rack from the manager because the wrapping gripper
    contacts its handle by design — but the forearm/wrist links get no such license. v1's
    200 mm pull never swept the rack's raised rear rim under the wrist; the Bosch 560 mm
    pull does (measured: 103.8 N on ``wrist_2_link`` in the first live episode), so the
    slide gate checks these pairs explicitly against the per-waypoint rack pose.
    """
    import fcl  # noqa: PLC0415

    rack_objs = world._static_objs.get(body, ())
    margin = float(config.SLIDE_ARM_CLEARANCE_M)
    for link in _SLIDE_ARM_LINKS:
        arm_obj = world._arm.get(link)
        if arm_obj is None:
            continue
        for so in rack_objs:
            d = fcl.distance(arm_obj, so, fcl.DistanceRequest(), fcl.DistanceResult())
            if d < margin:  # touching OR within the tracking-error margin
                return True
    return False
