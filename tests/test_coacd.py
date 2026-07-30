# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""CoACD smoke test: decompose a concave mesh into multiple convex pieces."""

import numpy as np
import trimesh


def test_coacd_decomposes_torus():
    import coacd

    coacd.set_log_level("error")

    torus = trimesh.creation.torus(major_radius=0.5, minor_radius=0.15)
    assert not torus.is_convex

    parts = coacd.run_coacd(coacd.Mesh(torus.vertices, torus.faces), threshold=0.05)
    assert len(parts) > 1, "a torus cannot be a single convex piece"

    total_volume = 0.0
    for verts, faces in parts:
        piece = trimesh.Trimesh(vertices=np.asarray(verts), faces=np.asarray(faces))
        hull = piece.convex_hull
        assert hull.volume > 0.0
        total_volume += hull.volume

    # decomposition must roughly cover the solid (pieces overlap, so >= ~torus volume)
    assert total_volume > torus.volume * 0.8, (
        f"convex pieces cover only {total_volume:.4f} of {torus.volume:.4f} m^3"
    )
    print(f"\nCoACD: {len(parts)} pieces, sum of hull volumes {total_volume:.4f} vs torus {torus.volume:.4f}")
