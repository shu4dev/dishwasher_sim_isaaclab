# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Quaternion order conversion at the Isaac Lab boundary — and nowhere else.

The project convention is XYZW everywhere (asserted in code; every recording, cache, and
config value). Isaac Lab 2.1 (this machine's runtime) is WXYZ everywhere: ``rot=`` config
tuples, ``data.*_quat_w`` buffers, and the 7-D poses passed to ``write_root_pose_to_sim``.
Every read of an isaaclab quaternion goes through :func:`wxyz_to_xyzw`; every value handed
to isaaclab goes through :func:`xyzw_to_wxyz`. Internal math never converts.

Kit-free on purpose (tuples / numpy / torch via duck typing) so both the planning stack and
Kit-side scripts can import it.
"""

_TO_WXYZ = (3, 0, 1, 2)
_TO_XYZW = (1, 2, 3, 0)


def _reorder(q, idx):
    if isinstance(q, (tuple, list)):
        return type(q)(q[i] for i in idx)
    return q[..., list(idx)]  # numpy array or torch tensor, quat on the last axis


def xyzw_to_wxyz(q):
    """Project order -> Isaac Lab 2.1 order, for values handed TO isaaclab."""
    return _reorder(q, _TO_WXYZ)


def wxyz_to_xyzw(q):
    """Isaac Lab 2.1 order -> project order, for values read FROM isaaclab."""
    return _reorder(q, _TO_XYZW)
