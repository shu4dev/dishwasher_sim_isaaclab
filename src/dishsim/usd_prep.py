# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive prepared copies of the ArtVIP dishwasher USD.

The ArtVIP assets ship two properties that break relocated spawns:

1. A ``PhysicsFixedJoint`` with ``body1`` but **no** ``body0`` — a world-space weld that pins the
   machine at its originally authored pose. Spawning the articulation anywhere else makes this
   weld fight the articulation's fixed base and the sim blows up within a few seconds. The
   derived copy removes it; ``fix_root_link=True`` in the :class:`~isaaclab.assets.ArticulationCfg`
   keeps the base fixed at the spawn pose.
2. An active door drive (stiffness 20 per-degree ≈ 1146 N·m/rad toward 90 deg, acceleration
   type). Actuator configs override the gains after initialization, but the authored drive still
   acts during the physics-reset step and kicks the door. The passive-door copy zeroes the
   revolute drive gains/targets and switches the drives to force type so configured gains mean
   N·m/rad.

Derived files are written next to the source (so relative asset references keep resolving); the
original is never modified. Two derivations exist:

- :func:`make_dishwasher_rl_usd` (``_rl`` suffix): passive door — used by the archived RL task
  and by the inspection script's stability/door tests.
- :func:`make_dishwasher_v0_usd` (``_v0`` suffix): the v0 planning scene — door joint limits
  clamped to a narrow band at the open position and rack drive targets set to the configured
  extensions, so the machine is a *static* obstacle. The lock itself is belt-and-braces: the
  clamped USD limits make the open pose inescapable even if drive gains misbehave, and the
  actuator config (``DISHWASHER_V0_CFG``) holds the joints with stiff position drives on top.
"""

import os


def make_dishwasher_rl_usd(src_path: str, force: bool = False) -> str:
    """Create (or reuse) the passive-door derived copy of a dishwasher USD.

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
    # SimulationApp starts (which happens — Kit walks importable modules at boot)
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
    print(f"[INFO] Passive-door copy written to {dst_path} (removed world welds: {removed or 'none'})")
    return dst_path


def make_dishwasher_v0_usd(
    src_path: str,
    door_open_deg: float = 90.0,
    door_band_deg: float = 5.0,
    rack_targets: dict[str, float] | None = None,
    force: bool = False,
) -> str:
    """Create (or reuse) the v0 derived copy: static machine, door locked open, racks pinned.

    Transformations relative to the original:

    1. World-weld fixed joints removed (same reason as :func:`make_dishwasher_rl_usd`).
    2. Every revolute (door) joint: limits clamped to
       ``[door_open_deg - door_band_deg, door_open_deg]`` (USD revolute limits are in degrees),
       drive neutralized (stiffness 0, force type) — the actuator config supplies the hold; the
       clamped limits plus gravity (the bottom-hinged door falls into the upper limit) make the
       pose static regardless.
    3. Every prismatic (rack) joint: drive switched to force type with the target set from
       ``rack_targets`` (joint name -> extension [m]) so the authored spring no longer pulls the
       rack to the stowed position during the physics-reset step.

    Args:
        src_path: Path to the original ``model_<variant>.usd(a)`` file.
        door_open_deg: Door-open angle the limits clamp around [deg].
        door_band_deg: Width of the allowed band below the open angle [deg].
        rack_targets: Prismatic drive targets by joint name [m]; joints not listed keep their
            authored target.
        force: Regenerate even if the derived file already exists.

    Returns:
        Path to the derived ``model_<variant>_v0.usda`` file.
    """
    root, _ = os.path.splitext(src_path)
    dst_path = f"{root}_v0.usda"
    if os.path.isfile(dst_path) and not force:
        return dst_path

    # deferred pxr import — same Kit-boot reason as above
    from pxr import Usd, UsdPhysics  # noqa: PLC0415

    stage = Usd.Stage.Open(src_path)

    removed = []
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() and joint.GetBody1Rel().GetTargets():
                removed.append(str(prim.GetPath()))
    for path in removed:
        stage.RemovePrim(path)

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.GetLowerLimitAttr().Set(door_open_deg - door_band_deg)
            joint.GetUpperLimitAttr().Set(door_open_deg)
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.CreateStiffnessAttr(0.0)
                drive.CreateTargetPositionAttr(door_open_deg)
                drive.CreateTypeAttr("force")
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
            if drive:
                target = (rack_targets or {}).get(prim.GetName())
                if target is not None:
                    drive.CreateTargetPositionAttr(float(target))
                drive.CreateTypeAttr("force")

    stage.GetRootLayer().Export(dst_path)
    print(
        f"[INFO] v0 copy written to {dst_path} (door clamped to "
        f"[{door_open_deg - door_band_deg}, {door_open_deg}] deg; removed world welds: {removed or 'none'})"
    )
    return dst_path
