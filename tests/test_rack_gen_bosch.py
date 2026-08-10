# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free checks on the Bosch 800 rack generator path.

Covers: the two big Bosch wire racks build through the hardened :func:`rack_gen.build_rack`
(optional features skipped when absent or None), the third-rack cutlery tray builds through
the NEW :func:`rack_gen.build_tray` (perimeter rim, fine mesh floor, dropped center channel),
the ``build`` dispatch routes on the ``"builder"`` key — and the v1 (artvip_compact) geometry
stays byte-identical, since the frozen mug baseline and every baked collision cache key on it.

``config.apply_machine`` mutates module globals in place, so every Bosch test restores the
baseline machine through the fixture.
"""

import hashlib

import numpy as np
import pytest

from dishsim import config
from dishsim import rack_gen

LOWER, UPPER, THIRD = "E_shelf_1_04", "E_shelf_03", "E_shelf_third"


@pytest.fixture()
def bosch():
    """Activate the Bosch machine; ALWAYS restore the byte-stable v1 baseline afterwards."""
    config.apply_machine("bosch800")
    yield config.RACK_GEN
    config.apply_machine(config.MACHINE_BASELINE_NAME)


# ---------------------------------------------------------------------------------------------
# Bosch wire racks through the hardened build_rack
# ---------------------------------------------------------------------------------------------


def test_bosch_wire_racks_assemble_to_footprint(bosch):
    """Both big Bosch racks build; assembled bounds land exactly on the configured footprint
    with wire bottoms at z=0 (the 12-gon bbox contract shared with the v1 racks)."""
    for name in (LOWER, UPPER):
        p = bosch[name]
        assert p["usd_scale_x"] == 1.0  # self-authored Bosch USD: no inherited Xform scale
        parts = rack_gen.build_rack(p)
        mn, mx = rack_gen.merged_mesh(parts).bounds
        np.testing.assert_allclose(mn, (0.0, 0.0, 0.0), atol=1e-9)
        np.testing.assert_allclose(mx[:2], p["footprint"], atol=1e-9)


def test_bosch_lower_has_no_basket_or_insert(bosch):
    """The Bosch lower rack carries no cutlery basket, no second tine bank, and no fold-down
    insert hardware — everything lands on the single ``frame`` mesh."""
    parts = rack_gen.build_rack(bosch[LOWER])
    assert set(rack_gen.parts_by_group(parts)) == {"frame"}
    names = {q.name for q in parts}
    assert not any(n.startswith(("basket", "insert_")) for n in names)
    # single robot bank at the real ~50 mm pitch: (0.460 - 0.060) / 0.050 + 1 = 9 tine columns
    assert len(rack_gen._plate_tine_xs(bosch[LOWER])) == 9
    assert any(n.startswith("plate_tine_") for n in names)


def test_build_rack_skips_absent_optional_features(bosch):
    """Hardening: optional feature keys may be ABSENT entirely; the rack still assembles to
    its footprint without those parts."""
    p = dict(bosch[UPPER])
    for key in ("handle", "guard_mid_rail", "wheels", "cup_shelf", "rackmatic_blocks",
                "divider_tine_x", "divider_tine_ys", "divider_tine_h",
                "slope_zones", "slope_deg", "slope_sign", "baluster_pitch"):
        p.pop(key, None)
    parts = rack_gen.build_rack(p)
    assert len(parts) < len(rack_gen.build_rack(bosch[UPPER]))
    names = {q.name for q in parts}
    assert not any(n.startswith(("handle_", "wheel", "shelf_", "rackmatic_", "divider_", "bal_"))
                   for n in names)
    mn, mx = rack_gen.merged_mesh(parts).bounds
    np.testing.assert_allclose(mn, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(mx[:2], p["footprint"], atol=1e-9)


def test_build_rack_skips_none_optional_features(bosch):
    """Hardening: optional feature keys may be present but None (the config idiom for 'this
    machine variant drops the feature') — same skip as an absent key."""
    p = dict(bosch[LOWER])
    p.update({"handle": None, "wheels": None, "candy_cane": None,
              "tine_bead": None, "tine_fillet": None, "rib_amplitude": None})
    parts = rack_gen.build_rack(p)
    names = {q.name for q in parts}
    assert not any(n.startswith(("handle_", "wheel", "cane_", "bead_", "fillet_", "rib_"))
                   for n in names)
    # the tine shafts survive, now full-height straight rods on every column
    assert any(n.startswith("plate_tine_") for n in names)
    mn, mx = rack_gen.merged_mesh(parts).bounds
    np.testing.assert_allclose(mn, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(mx[:2], p["footprint"], atol=1e-9)


# ---------------------------------------------------------------------------------------------
# third-rack tray builder
# ---------------------------------------------------------------------------------------------


def test_tray_assembles_to_footprint_with_rim(bosch):
    """The tray's assembled bounds land exactly on the footprint, wire bottoms at z=0, and
    the rim top exactly ``rim_h`` above the wing floor plane."""
    p = bosch[THIRD]
    parts = rack_gen.build_tray(p)
    mn, mx = rack_gen.merged_mesh(parts).bounds
    np.testing.assert_allclose(mn, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(
        mx, (*p["footprint"], rack_gen.tray_floor_z(p) + p["rim_h"]), atol=1e-9
    )
    # plumbing contract: single frame mesh, only preview-known zones
    assert set(rack_gen.parts_by_group(parts)) == {"frame"}
    assert {q.zone for q in parts} <= set(rack_gen.ZONE_COLORS)
    # every part is convex-usable: watertight with positive volume (the FCL piece contract)
    assert all(q.mesh.volume > 0 for q in parts)


def test_tray_channel_floor_drop(bosch):
    """The dropped center channel's floor sits exactly ``channel.drop`` below the wing floor
    plane, and the channel-span wires carry the global z=0 bottoms."""
    p = bosch[THIRD]
    parts = rack_gen.build_tray(p)
    drop = p["channel"]["drop"]
    assert abs(rack_gen.tray_floor_z(p) - rack_gen.tray_channel_floor_z(p) - drop) < 1e-12
    floor = np.concatenate(
        [q.mesh.vertices for q in parts if q.zone in ("floor_open", "channel", "slope")]
    )
    # wing floor plane = the highest floor-wire surface; lowest wire bottoms exactly at 0
    assert abs(floor[:, 2].max() - rack_gen.tray_floor_z(p)) < 1e-9
    assert abs(floor[:, 2].min()) < 1e-12
    # dropped floor: the channel-zone crossbar segments top out exactly on the channel floor
    # plane, and the channel-span runners carry the global z=0 wire bottoms
    cross_ch = np.concatenate(
        [q.mesh.vertices for q in parts if q.zone == "channel" and q.name.startswith("tray_cross")]
    )
    assert abs(cross_ch[:, 2].max() - rack_gen.tray_channel_floor_z(p)) < 1e-9
    run_ch = np.concatenate(
        [q.mesh.vertices for q in parts if q.zone == "channel" and q.name.startswith("tray_runner")]
    )
    assert abs(run_ch[:, 2].min()) < 1e-12
    # the channel runners rest tangent under their crossbars, drop below the wing runners
    assert abs(run_ch[:, 2].max() - cross_ch[:, 2].min()) < 1e-9
    cx0, cx1 = p["channel"]["x"]
    assert cx0 < run_ch[:, 0].min() and run_ch[:, 0].max() < cx1


def test_tray_mesh_pitches(bosch):
    """The mesh floor follows the configured pitches: adjacent runners ``runner_pitch``
    apart (fine, so cutlery cannot fall through), crossbar rows at ``crossbar_pitch``."""
    p = bosch[THIRD]
    parts = rack_gen.build_tray(p)
    runner_xs = sorted(q.mesh.vertices[:, 0].mean() for q in parts if q.name.startswith("tray_runner"))
    gaps = np.diff(runner_xs)
    np.testing.assert_allclose(gaps, p["runner_pitch"], atol=1e-9)
    cross_ys = sorted({round(float(q.mesh.vertices[:, 1].mean()), 6) for q in parts
                       if q.name.startswith("tray_cross")})
    np.testing.assert_allclose(np.diff(cross_ys), p["crossbar_pitch"], atol=1e-9)


def test_build_dispatch(bosch):
    """``rack_gen.build`` routes on the ``"builder"`` key: tray -> build_tray, default (the
    key is absent on every wire-rack dict) -> build_rack; typos raise."""
    tray_parts = rack_gen.build(bosch[THIRD])
    assert [q.name for q in tray_parts] == [q.name for q in rack_gen.build_tray(bosch[THIRD])]
    assert "builder" not in bosch[LOWER]
    rack_parts = rack_gen.build(bosch[LOWER])
    assert [q.name for q in rack_parts] == [q.name for q in rack_gen.build_rack(bosch[LOWER])]
    with pytest.raises(ValueError, match="unknown rack builder"):
        rack_gen.build({**bosch[THIRD], "builder": "hovercraft"})


# ---------------------------------------------------------------------------------------------
# v1 byte-stability (the frozen-baseline invariant)
# ---------------------------------------------------------------------------------------------

# Recorded 2026-08-10 against the pre-Bosch generator (RACK_GEN_VERSION 4, v1 machine). The
# v1 racks are FROZEN: any drift here silently invalidates every baked collision cache and
# the frozen mug baseline — this test must only ever fail because someone changed v1 geometry
# or the builder's v1 code path, and both are bugs.
_V1_DIGESTS = {
    "E_shelf_1_04": "95146089757e0b34b2bfe51caa322af9061a47512cdfe50dd78c6d73c56a2597",
    "E_shelf_03": "d9d80e69f36043485aa4c19838fdbd80b2b2e27af4367124152661d9fffe9438",
}
_V1_PARAMS_HASH = "d6df107a9bca8510"
_V1_PART_COUNTS = {"E_shelf_1_04": 421, "E_shelf_03": 166}


def test_v1_geometry_byte_identical():
    """build_rack on the v1 dicts reproduces the recorded pre-Bosch geometry byte-for-byte,
    and params_hash of the v1 generator config is unchanged."""
    config.apply_machine(config.MACHINE_BASELINE_NAME)  # hermetic: never trust test order
    for name, want in _V1_DIGESTS.items():
        parts = rack_gen.build_rack(config.RACK_GEN[name])
        assert len(parts) == _V1_PART_COUNTS[name], f"{name}: v1 part count drifted"
        h = hashlib.sha256()
        for part in parts:
            h.update(part.name.encode())
            h.update(part.zone.encode())
            h.update(part.group.encode())
            h.update(part.mesh.vertices.tobytes())
            h.update(part.mesh.faces.tobytes())
        assert h.hexdigest() == want, f"{name}: v1 rack geometry drifted"
    assert rack_gen.params_hash(config.RACK_GEN, config.RACK_GEN_VERSION) == _V1_PARAMS_HASH
