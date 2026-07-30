# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Analytic UR5e kinematics: closed-form FK and 8-branch IK, no simulator dependencies.

Frames
------
The canonical end-effector frame is **``wrist_3_link``** (it exists in the Isaac articulation's
body list, and the Robotiq TCP is a fixed offset from it — see :mod:`dishsim.config`).

The math runs in the classic UR DH convention (Hawkins 2013 / ROS-Industrial ``ur_kin``), with
nominal UR5e parameters taken from the ``ur_description`` URDF that ships with Isaac Sim
(``motion_policy_configs/universal_robots/ur5e/ur5e.urdf``). In that URDF the chain from
``base_link_inertia`` to ``wrist_3_link`` is *exactly* the DH chain (verified symbolically: the
wrist_3 origin ``rpy=(pi/2, pi, pi)`` collapses to ``Rx(-pi/2)``, matching the DH tail), and
``base_link`` differs from the DH base only by ``Rz(pi)``. Hence:

    T_base_link->wrist_3_link(q) = Rz(pi) @ dh_fk(q)

This identity is re-verified numerically at import time (raises ``AssertionError`` on drift) and
against Pinocchio in ``tests/test_ur5e_kin.py``.

Joint order everywhere: ``[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]``.
"""

import numpy as np

# Nominal UR5e DH parameters [m] (ur_description / vendor values).
D1 = 0.1625
A2 = -0.425
A3 = -0.3922
D4 = 0.1333
D5 = 0.0997
D6 = 0.0996

# Joint limits [rad] (URDF: +-2*pi except elbow +-pi).
JOINT_LIMITS = np.array(
    [
        [-2.0 * np.pi, 2.0 * np.pi],
        [-2.0 * np.pi, 2.0 * np.pi],
        [-np.pi, np.pi],
        [-2.0 * np.pi, 2.0 * np.pi],
        [-2.0 * np.pi, 2.0 * np.pi],
        [-2.0 * np.pi, 2.0 * np.pi],
    ]
)

# base_link -> DH base frame corrector (Rz(pi)).
_RZ_PI = np.diag([-1.0, -1.0, 1.0, 1.0])

_EPS = 1e-10


def _dh_mat(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


_DH_TABLE = (  # (d, a, alpha) per joint
    (D1, 0.0, np.pi / 2),
    (0.0, A2, 0.0),
    (0.0, A3, 0.0),
    (D4, 0.0, np.pi / 2),
    (D5, 0.0, -np.pi / 2),
    (D6, 0.0, 0.0),
)


def _dh_fk(q: np.ndarray) -> np.ndarray:
    """FK in the DH convention: DH base frame -> DH frame 6 (== wrist_3_link orientation/origin)."""
    T = np.eye(4)
    for qi, (d, a, alpha) in zip(q, _DH_TABLE):
        T = T @ _dh_mat(qi, d, a, alpha)
    return T


def fk_wrist3(q: np.ndarray) -> np.ndarray:
    """Pose of ``wrist_3_link`` in the ``base_link`` frame for joint vector ``q`` [rad].

    Args:
        q: Joint positions, shape [6], order shoulder_pan..wrist_3.

    Returns:
        Homogeneous transform, shape [4, 4].
    """
    return _RZ_PI @ _dh_fk(np.asarray(q, dtype=float))


#: Link frame reached after each successive joint (URDF link names, robot base frame).
LINK_NAMES = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]


def fk_all_links(q: np.ndarray) -> dict[str, np.ndarray]:
    """Pose of every arm link frame in the ``base_link`` frame (URDF chain, exact).

    Includes ``base_link`` (identity). Gripper links are rigid relative to ``wrist_3_link`` in
    v0 (frozen fingers) — compose their cached offsets externally.

    Args:
        q: Joint positions, shape [6].

    Returns:
        Mapping link name -> homogeneous transform, shape [4, 4].
    """
    q = np.asarray(q, dtype=float)
    out = {"base_link": np.eye(4)}
    T = _RZ_PI.copy()  # base_link -> base_link_inertia (start of the joint chain)
    # per-joint (fixed origin, then Rz(q)) — origins hard-coded from the shipped URDF
    c, s = np.cos, np.sin

    def rz(a):
        return np.array([[c(a), -s(a), 0, 0], [s(a), c(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])

    def rx(a):
        return np.array([[1, 0, 0, 0], [0, c(a), -s(a), 0], [0, s(a), c(a), 0], [0, 0, 0, 1.0]])

    def tr(x, y, z):
        T = np.eye(4)
        T[:3, 3] = (x, y, z)
        return T

    origins = [
        tr(0, 0, 0.1625),
        rx(np.pi / 2),
        tr(-0.425, 0, 0),
        tr(-0.3922, 0, 0.1333),
        tr(0, -0.0997, 0) @ rx(np.pi / 2),
        tr(0, 0.0996, 0) @ rx(-np.pi / 2),
    ]
    for name, origin, qi in zip(LINK_NAMES, origins, q):
        T = T @ origin @ rz(qi)
        out[name] = T.copy()
    return out


def wrap_to_pi(q: np.ndarray) -> np.ndarray:
    """Wrap angles to (-pi, pi]."""
    return -((-np.asarray(q) + np.pi) % (2.0 * np.pi) - np.pi)


def ik_wrist3_all(
    T_base_wrist3: np.ndarray,
    q_seed: np.ndarray | None = None,
    fk_tol: float = 1e-6,
) -> np.ndarray:
    """All analytic IK branches for a target ``wrist_3_link`` pose in the ``base_link`` frame.

    Up to 8 solutions (shoulder left/right x elbow up/down x wrist flip). Singular branches
    (wrist center on the shoulder axis; ``|sin(q5)| ~ 0`` where q4/q6 are coupled) return one
    representative with q6 taken from ``q_seed`` (or 0). Every returned solution is verified
    against :func:`fk_wrist3` and discarded if the reconstruction error exceeds ``fk_tol``.

    Args:
        T_base_wrist3: Target pose, shape [4, 4].
        q_seed: Optional reference configuration used to resolve the free angle at wrist
            singularities, shape [6].
        fk_tol: Max allowed FK reconstruction error [m] (translation + rotation matrix norm).

    Returns:
        Solutions wrapped to (-pi, pi], shape [K, 6] with K <= 8 (possibly K == 0).
    """
    T = _RZ_PI @ np.asarray(T_base_wrist3, dtype=float)  # into DH convention (Rz(pi) inverse == itself)
    q6_free = 0.0 if q_seed is None else float(q_seed[5])

    T00, T01, T02, T03 = T[0]
    T10, T11, T12, T13 = T[1]
    T20, T21, T22, T23 = T[2]

    sols: list[np.ndarray] = []

    # ---- q1: shoulder rotation (2 branches) --------------------------------------------------
    A = D6 * T12 - T13
    B = D6 * T02 - T03
    R2 = A * A + B * B
    if R2 < D4 * D4 - _EPS:
        return np.zeros((0, 6))
    arg = D4 / max(np.sqrt(R2), _EPS)
    arccos = np.arccos(np.clip(arg, -1.0, 1.0))
    arctan = np.arctan2(-B, A)
    q1_branches = (arccos + arctan, -arccos + arctan)

    for q1 in q1_branches:
        c1, s1 = np.cos(q1), np.sin(q1)

        # ---- q5: wrist "lift" (2 branches) ---------------------------------------------------
        numer = T03 * s1 - T13 * c1 - D4
        div = np.clip(numer / D6, -1.0, 1.0)
        acos5 = np.arccos(div)
        for q5 in (acos5, -acos5):
            c5, s5 = np.cos(q5), np.sin(q5)

            # ---- q6 from orientation (free at wrist singularity) -----------------------------
            if abs(s5) < 1e-8:
                q6 = q6_free
            else:
                sgn = np.sign(s5)
                q6 = np.arctan2(sgn * -(T01 * s1 - T11 * c1), sgn * (T00 * s1 - T10 * c1))
            c6, s6 = np.cos(q6), np.sin(q6)

            # ---- planar 2R subchain for q2, q3, then q4 --------------------------------------
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1))
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = (
                D5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1))
                - D6 * (T02 * c1 + T12 * s1)
                + T03 * c1
                + T13 * s1
            )
            p13y = T23 - D1 - D6 * T22 + D5 * (T21 * c6 + T20 * s6)

            c3 = (p13x * p13x + p13y * p13y - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
            if abs(c3) > 1.0 + 1e-9:
                continue  # target out of the planar subchain's reach for this branch
            c3 = np.clip(c3, -1.0, 1.0)
            s3 = np.sqrt(max(0.0, 1.0 - c3 * c3))
            denom = A2 * A2 + A3 * A3 + 2.0 * A2 * A3 * c3
            AA = A2 + A3 * c3
            BB = A3 * s3

            for q3, sign3 in ((np.arccos(c3), 1.0), (-np.arccos(c3), -1.0)):
                b = sign3 * BB
                q2 = np.arctan2((AA * p13y - b * p13x) / denom, (AA * p13x + b * p13y) / denom)
                c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
                q4 = np.arctan2(c23 * x04y - s23 * x04x, x04x * c23 + x04y * s23)
                sols.append(wrap_to_pi(np.array([q1, q2, q3, q4, q5, q6])))

    if not sols:
        return np.zeros((0, 6))

    # keep only solutions that actually reconstruct the target (guards against numerically
    # degenerate branches) and deduplicate
    T_target = np.asarray(T_base_wrist3, dtype=float)
    good: list[np.ndarray] = []
    for q in sols:
        T_re = fk_wrist3(q)
        err = np.linalg.norm(T_re[:3, 3] - T_target[:3, 3]) + np.linalg.norm(T_re[:3, :3] - T_target[:3, :3])
        if err < fk_tol:
            if not any(np.max(np.abs(wrap_to_pi(q - g))) < 1e-8 for g in good):
                good.append(q)
    return np.array(good) if good else np.zeros((0, 6))


def expand_2pi_wraps(
    sols: np.ndarray,
    limits: np.ndarray = JOINT_LIMITS,
    margin: float = 0.05,
    max_out: int = 256,
) -> np.ndarray:
    """Expand IK solutions with +-2*pi-wrapped duplicates that stay within joint limits.

    Args:
        sols: IK solutions, shape [K, 6].
        limits: Joint limits, shape [6, 2].
        margin: Keep-out margin from the limits [rad].
        max_out: Hard cap on the number of returned configurations.

    Returns:
        Expanded configurations (including the originals), shape [M, 6], M <= max_out.
    """
    out: list[np.ndarray] = []
    for q in np.atleast_2d(sols):
        variants: list[list[float]] = [[]]
        for j in range(6):
            vals = []
            for k in (0.0, 2.0 * np.pi, -2.0 * np.pi):
                v = q[j] + k
                if limits[j, 0] + margin <= v <= limits[j, 1] - margin:
                    vals.append(v)
            if not vals:
                variants = []
                break
            variants = [prefix + [v] for prefix in variants for v in vals]
        out.extend(np.array(v) for v in variants)
        if len(out) >= max_out:
            break
    if not out:
        return np.zeros((0, 6))
    arr = np.array(out[:max_out])
    # deduplicate
    keep = []
    for q in arr:
        if not any(np.max(np.abs(q - k)) < 1e-9 for k in keep):
            keep.append(q)
    return np.array(keep)


def dls_ik(
    T_base_wrist3: np.ndarray,
    q0: np.ndarray,
    max_iters: int = 200,
    tol: float = 1e-8,
    damping: float = 1e-3,
) -> np.ndarray | None:
    """Damped-least-squares numeric IK fallback (finite-difference Jacobian).

    Args:
        T_base_wrist3: Target pose, shape [4, 4].
        q0: Initial guess, shape [6].
        max_iters: Iteration cap.
        tol: Convergence threshold on the 6-D pose error norm.
        damping: DLS damping factor.

    Returns:
        Converged joint vector (shape [6]) or None.
    """
    T_t = np.asarray(T_base_wrist3, dtype=float)
    q = np.asarray(q0, dtype=float).copy()

    def pose_err(qv: np.ndarray) -> np.ndarray:
        T = fk_wrist3(qv)
        e_p = T_t[:3, 3] - T[:3, 3]
        R_err = T_t[:3, :3] @ T[:3, :3].T
        # rotation-vector (log map) of R_err
        cos_a = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(cos_a)
        if angle < 1e-9:
            e_r = np.zeros(3)
        else:
            axis = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
            e_r = angle * axis / (2.0 * np.sin(angle))
        return np.concatenate([e_p, e_r])

    for _ in range(max_iters):
        e = pose_err(q)
        if np.linalg.norm(e) < tol:
            return wrap_to_pi(q)
        # finite-difference Jacobian of the pose (e(q) = target - pose(q), so
        # (e(q) - e(q+h))/h = d(pose)/dq, which is what the DLS update wants)
        J = np.zeros((6, 6))
        h = 1e-6
        for j in range(6):
            dq = q.copy()
            dq[j] += h
            J[:, j] = (pose_err(q) - pose_err(dq)) / h
        JT = J.T
        q = q + JT @ np.linalg.solve(J @ JT + damping * np.eye(6), e)
    return None


def _self_check() -> None:
    """Verify the Rz(pi) base-corrector identity against the raw URDF chain at fixed configs."""

    def rx(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])

    def rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])

    def tr(x, y, z):
        T = np.eye(4)
        T[:3, 3] = (x, y, z)
        return T

    def urdf_fk(q):
        # exact joint origins from the shipped ur_description UR5e URDF
        T = rz(np.pi)  # base_link -> base_link_inertia
        T = T @ tr(0, 0, 0.1625) @ rz(q[0])
        T = T @ rx(np.pi / 2) @ rz(q[1])
        T = T @ tr(-0.425, 0, 0) @ rz(q[2])
        T = T @ tr(-0.3922, 0, 0.1333) @ rz(q[3])
        T = T @ tr(0, -0.0997, 0) @ rx(np.pi / 2) @ rz(q[4])
        T = T @ tr(0, 0.0996, 0) @ rx(-np.pi / 2) @ rz(q[5])
        return T

    for q in (
        np.zeros(6),
        np.array([0.3, -1.2, 1.9, -2.2, -1.3, 0.7]),
        np.array([-2.1, 0.4, -0.8, 1.1, 2.5, -3.0]),
    ):
        err = np.max(np.abs(fk_wrist3(q) - urdf_fk(q)))
        assert err < 1e-9, f"UR5e FK corrector identity violated (err={err:.3e}) — check DH table"


_self_check()
