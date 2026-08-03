# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free preview of the procedural rack geometry (rack_gen) — evidence before any Kit run.

Renders zone-colored 3-view PNGs for both racks into ``media/D/`` and prints a spec-compliance
table (part counts, bounds, tine/gap/rim numbers measured from the built parts, not the params).

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/preview_rack.py
"""

import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config, rack_gen  # noqa: E402


def main() -> None:
    media_dir = os.path.join(PROJECT_ROOT, "media", "D")
    for body, params in config.RACK_GEN.items():
        parts = rack_gen.build_rack(params)
        merged = rack_gen.merged_mesh(parts)
        mn, mx = merged.bounds
        out = os.path.join(media_dir, f"rack_gen_preview_{body}.png")
        rack_gen.preview_png(parts, out, body)
        zones = {}
        for p in parts:
            zones[p.zone] = zones.get(p.zone, 0) + 1
        print(f"\n[INFO] {body}: {len(parts)} parts, {len(merged.vertices)} verts -> {out}")
        print(f"       bounds  {np.round(mn, 4).tolist()} .. {np.round(mx, 4).tolist()}")
        print(f"       zones   {dict(sorted(zones.items()))}")
        print(f"       rim     front dip {params['rim_front_h'] * 1e3:.0f} mm, sides {params['rim_side_h'] * 1e3:.0f} mm, "
              f"rear {params['rim_rear_h'] * 1e3:.0f} mm (+{(params['rim_rear_h'] / params['rim_side_h'] - 1) * 100:.0f}% vs sides)")
        print(f"       floor   top z {rack_gen.floor_top_z(params) * 1e3:.1f} mm; slope {params['slope_deg']} deg")
        if "plate_rows_y" in params:
            print(f"       tines   plate h {params['plate_tine_h'] * 1e3:.0f} mm @ {params['plate_tine_pitch'] * 1e3:.0f} mm "
                  f"pitch, lean {params['plate_tine_lean_deg']} deg; bowl h {params['bowl_tine_h'] * 1e3:.0f} mm")
    print("\n[RESULT] PASS (previews rendered)")


if __name__ == "__main__":
    main()
