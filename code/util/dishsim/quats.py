# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Quaternion order at the Isaac Lab boundary — and nowhere else.

The project convention is XYZW everywhere (asserted in code; every recording, cache, and
config value). Isaac Lab 3.0 (Isaac Sim 6.2, the Brev box's runtime) is XYZW too: ``rot=``
config tuples (``InitialStateCfg.rot`` defaults to ``(0, 0, 0, 1)``), ``data.*_quat_w`` buffers,
and the 7-D poses passed to ``write_root_pose_to_sim_index``. Both functions are therefore the
identity; they stay at every boundary crossing so the crossings remain findable (Isaac Lab 2.1,
corallab's runtime on the isaac45-final commit, was WXYZ and reordered here). Never use them for
``pxr`` (``Gf.Quat*`` takes the real part first in every version).

Kit-free on purpose (tuples / numpy / torch via duck typing) so both the planning stack and
Kit-side scripts can import it.
"""


def xyzw_to_wxyz(q):
    """Project order -> Isaac Lab order, for values handed TO isaaclab (3.0: the same order)."""
    return q


def wxyz_to_xyzw(q):
    """Isaac Lab order -> project order, for values read FROM isaaclab (3.0: the same order)."""
    return q
