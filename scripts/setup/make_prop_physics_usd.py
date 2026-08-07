# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Derive a physics-enabled copy of a YCB prop from the Isaac Sim asset library.

The Isaac Sim 6.0 asset library ships ``Props/YCB/Axis_Aligned_Physics`` with only four objects
(cracker box, sugar box, soup can, mustard bottle). Everything else — including the plate
(``029_plate.usd``) and the mug (``025_mug.usd``) — exists only in the plain ``Axis_Aligned``
folder without any physics schemas, and Isaac Lab's spawner can only modify existing physics
APIs, not add missing ones. This script downloads the plain USD, applies ``RigidBodyAPI`` +
``MassAPI`` on the root prim and a convex-decomposition collision approximation on every mesh
prim, and saves the result to ``assets/props/<object>_physics.usd``.

Convex decomposition (rather than convex hull) matters here: a convex hull would "fill in" the
plate's dish or the mug's cavity, so the object could not nest between rack tines.

Run with:
    /workspace/isaaclab/isaaclab.sh -p scripts/setup/make_prop_physics_usd.py --object 029_plate
    /workspace/isaaclab/isaaclab.sh -p scripts/setup/make_prop_physics_usd.py --object 025_mug
"""

import argparse
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

# Measured YCB object masses [kg].
DEFAULT_MASS = {
    "029_plate": 0.279,
    "025_mug": 0.118,
}

parser = argparse.ArgumentParser(description="Derive a physics-enabled YCB prop USD.")
parser.add_argument(
    "--object",
    type=str,
    default="029_plate",
    help="YCB object name in Props/YCB/Axis_Aligned (e.g. 029_plate, 025_mug).",
)
parser.add_argument(
    "--output",
    type=str,
    default=None,
    help="Output path for the derived USD (default: assets/props/<object>_physics.usd).",
)
parser.add_argument(
    "--mass",
    type=float,
    default=None,
    help="Object mass [kg]; defaults to the measured YCB mass for known objects.",
)
args = parser.parse_args()


def main():
    # isaaclab.utils.assets resolves the pinned S3 asset root without launching the full app.
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, retrieve_file_path

    from pxr import Usd, UsdGeom, UsdPhysics

    mass = args.mass if args.mass is not None else DEFAULT_MASS.get(args.object)
    if mass is None:
        raise ValueError(f"No default mass known for '{args.object}' — pass --mass explicitly.")
    output = args.output or os.path.join(PROJECT_ROOT, "assets", "props", f"{args.object}_physics.usd")

    src_url = f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned/{args.object}.usd"
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
    mass_api.CreateMassAttr(mass)

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

    os.makedirs(os.path.dirname(output), exist_ok=True)
    stage.GetRootLayer().Export(output)
    print(f"[INFO] Wrote {output} (mass {mass} kg)")

    # Sanity check: reopen and verify the schemas exist.
    check = Usd.Stage.Open(output)
    root = check.GetDefaultPrim()
    assert root.HasAPI(UsdPhysics.RigidBodyAPI), "RigidBodyAPI missing after export"
    assert root.HasAPI(UsdPhysics.MassAPI), "MassAPI missing after export"
    print("[OK] RigidBodyAPI + MassAPI verified on", root.GetPath())


if __name__ == "__main__":
    main()
