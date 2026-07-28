# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive a physics-enabled copy of the YCB mug from the Isaac Sim asset library.

The Isaac Sim 6.0 asset library ships ``Props/YCB/Axis_Aligned_Physics`` with only four objects
(cracker box, sugar box, soup can, mustard bottle). The mug (``025_mug.usd``) exists only in the
plain ``Axis_Aligned`` folder without any physics schemas, and Isaac Lab's spawner can only modify
existing physics APIs, not add missing ones. This script downloads the plain mug USD, applies
``RigidBodyAPI`` + ``MassAPI`` on the root prim and a convex-decomposition collision approximation
on every mesh prim, and saves the result to ``assets/props/025_mug_physics.usd``.

Convex decomposition (rather than convex hull) matters for the stretch task: a convex hull would
"fill in" the mug's cavity so it could neither nest between rack tines nor be grasped by the rim.

Run with:
    /workspace/isaaclab/isaaclab.sh -p scripts/01_make_mug_physics_usd.py
"""

import argparse
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="Derive a physics-enabled YCB mug USD.")
parser.add_argument(
    "--output",
    type=str,
    default=os.path.join(PROJECT_ROOT, "assets", "props", "025_mug_physics.usd"),
    help="Output path for the derived USD.",
)
parser.add_argument("--mass", type=float, default=0.118, help="Mug mass [kg] (YCB 025_mug is 118 g).")
args = parser.parse_args()


def main():
    # isaaclab.utils.assets resolves the pinned S3 asset root without launching the full app.
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, retrieve_file_path

    from pxr import Usd, UsdGeom, UsdPhysics

    src_url = f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned/025_mug.usd"
    download_dir = os.path.join(PROJECT_ROOT, "assets", "props", "_download")
    print(f"[INFO] Retrieving {src_url}")
    local_src = retrieve_file_path(src_url, download_dir=download_dir)
    print(f"[INFO] Local copy: {local_src}")

    stage = Usd.Stage.Open(local_src)
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        default_prim = next(iter(stage.GetPseudoRoot().GetChildren()))
        stage.SetDefaultPrim(default_prim)
    print(f"[INFO] Default prim: {default_prim.GetPath()}")

    # Rigid body + mass on the root prim.
    UsdPhysics.RigidBodyAPI.Apply(default_prim)
    mass_api = UsdPhysics.MassAPI.Apply(default_prim)
    mass_api.CreateMassAttr(args.mass)

    # Convex-decomposition collision on every mesh.
    mesh_count = 0
    for prim in Usd.PrimRange(default_prim):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr("convexDecomposition")
            mesh_count += 1
    print(f"[INFO] Applied convex-decomposition collision to {mesh_count} mesh prim(s)")
    if mesh_count == 0:
        raise RuntimeError("No mesh prims found under the default prim — unexpected asset layout.")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    stage.GetRootLayer().Export(args.output)
    print(f"[INFO] Wrote {args.output}")

    # Sanity check: reopen and verify the schemas exist.
    check = Usd.Stage.Open(args.output)
    root = check.GetDefaultPrim()
    assert root.HasAPI(UsdPhysics.RigidBodyAPI), "RigidBodyAPI missing after export"
    assert root.HasAPI(UsdPhysics.MassAPI), "MassAPI missing after export"
    print("[OK] RigidBodyAPI + MassAPI verified on", root.GetPath())


if __name__ == "__main__":
    main()
