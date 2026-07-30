# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Small homogeneous-transform helpers (numpy + scipy, XYZW quaternions everywhere)."""

import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_mat(quat_xyzw) -> np.ndarray:
    """Rotation matrix from an XYZW quaternion, shape [3, 3]."""
    return Rotation.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """XYZW quaternion from a rotation matrix, shape [4]."""
    return Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()


def make_T(pos, quat_xyzw) -> np.ndarray:
    """Homogeneous transform from position [m] and XYZW quaternion, shape [4, 4]."""
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(quat_xyzw)
    T[:3, 3] = np.asarray(pos, dtype=float)
    return T


def T_to_pos_quat(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split a homogeneous transform into (position [m], XYZW quaternion)."""
    T = np.asarray(T, dtype=float)
    return T[:3, 3].copy(), mat_to_quat(T[:3, :3])


def T_inv(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid homogeneous transform, shape [4, 4]."""
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    R = T[:3, :3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def rot_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices [deg]."""
    R = np.asarray(R_a)[:3, :3].T @ np.asarray(R_b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))
