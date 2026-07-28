# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive an RL-ready copy of an ArtVIP dishwasher USD.

The ArtVIP assets ship two properties that break relocated, cloned RL scenes:

1. A ``PhysicsFixedJoint`` with ``body1`` but **no** ``body0`` — a world-space weld that pins the
   machine at its originally authored pose. Spawning the articulation anywhere else (which every
   cloned environment does) makes this weld fight the articulation's fixed base and the sim blows
   up within a few seconds. The RL copy removes it; ``fix_root_link=True`` in the
   :class:`~isaaclab.assets.ArticulationCfg` keeps the base fixed at the per-env spawn pose.
2. An active door drive (stiffness 20 per-degree ≈ 1146 N·m/rad toward 90 deg, acceleration
   type). Actuator configs override the gains after initialization, but the authored drive still
   acts during the physics-reset step and kicks the door. The RL copy zeroes the revolute drive
   gains/targets and switches the drives to force type so configured gains mean N·m/rad.

The derived file is written next to the source (so its relative asset references keep resolving)
with an ``_rl`` suffix; the original is never modified.
"""

import os


def make_dishwasher_rl_usd(src_path: str, force: bool = False) -> str:
    """Create (or reuse) the RL-ready derived copy of a dishwasher USD.

    Args:
        src_path: Path to the original ``model_<variant>.usd(a)`` file.
        force: Regenerate even if the derived file already exists.

    Returns:
        Path to the derived ``model_<variant>_rl.usda`` file.
    """
    root, _ = os.path.splitext(src_path)
    dst_path = f"{root}_rl.usda"
    if os.path.isfile(dst_path) and not force:
        return dst_path

    # deferred: importing pxr at module scope breaks Kit if this package is imported before
    # SimulationApp starts (which happens — the task registry walks all config modules)
    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    stage = Usd.Stage.Open(src_path)

    # 1) remove world-weld fixed joints (body1 set, body0 empty)
    removed = []
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                removed.append(str(prim.GetPath()))
    for path in removed:
        stage.RemovePrim(path)

    # 2) neutralize revolute (door) drives and make all joint drives force-type
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.CreateStiffnessAttr(0.0)
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTypeAttr("force")
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
            if drive:
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTypeAttr("force")

    stage.GetRootLayer().Export(dst_path)
    print(f"[INFO] RL copy written to {dst_path} (removed world welds: {removed or 'none'})")
    return dst_path
