# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free validation of the procedural rack generator (rack_gen v2).

Covers: reference-appliance dimensional compliance (measured from the built parts, not just the
params — 3-gauge wires, 30 mm Whirlpool tine rows, candy canes, beads, fillets, wheels, shelves),
per-part convexity/watertightness, exact assembled bounds, the floor-datum contract that
placement.derive_slots_from_rack depends on, the FCL feasibility probes exactly as decompose_meshes runs
them, the Phase-E slot guarantee (>= 3 mug-feasible slot columns), the insert-group split, the
upper-rack raise wrist-clearance arithmetic, and determinism plus the USD x-scale
pre-compensation round-trip.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/test_rack_gen.py
"""

import math

import fcl
import numpy as np
import pytest
import trimesh

from dishsim import config, geometry, rack_gen

LOWER = "E_shelf_1_04"
UPPER = "E_shelf_03"


@pytest.fixture(scope="module")
def racks():
    return {name: rack_gen.build_rack(params) for name, params in config.RACK_GEN.items()}


def _fcl_convex(mesh: trimesh.Trimesh) -> fcl.Convex:
    hull = mesh.convex_hull
    faces = np.hstack([np.full((len(hull.faces), 1), 3, dtype=np.int64), hull.faces.astype(np.int64)])
    return fcl.Convex(hull.vertices.astype(np.float64), len(hull.faces), faces.flatten())


def _manager(parts):
    objs = [fcl.CollisionObject(_fcl_convex(p.mesh)) for p in parts]
    mgr = fcl.DynamicAABBTreeCollisionManager()
    mgr.registerObjects(objs)
    mgr.setup()
    return mgr


def _hits(mgr, extents, center) -> bool:
    probe = fcl.CollisionObject(fcl.Box(*extents), fcl.Transform(np.asarray(center, dtype=float)))
    cdata = fcl.CollisionData()
    mgr.collide(probe, cdata, fcl.defaultCollisionCallback)
    return bool(cdata.result.is_collision)


# ---------------------------------------------------------------------------------------------
# 1. reference-appliance dimensional compliance
# ---------------------------------------------------------------------------------------------


def test_spec_param_ranges():
    for name, p in config.RACK_GEN.items():
        # rear guard vs SIDE rails (the lower rack's front rail deliberately dips for loading)
        ratio = p["rim_rear_h"] / p["rim_side_h"]
        assert 1.20 <= ratio <= 1.30, f"{name}: rear rim +{(ratio - 1) * 100:.0f}% outside 20-30%"
        assert 0.015 <= p["corner_r"] <= 0.020
        assert 5.0 <= p["slope_deg"] <= 8.0
        # 3-gauge hierarchy (frame >> runners > tines)
        assert p["wire_dia_heavy"] == 0.005
        assert 0.0034 <= p["wire_dia_load"] <= 0.005
        assert 0.0020 <= p["wire_dia_light"] <= 0.0030
        assert p["wire_dia_heavy"] / p["wire_dia_light"] >= 2.0
    p = config.RACK_GEN[LOWER]
    assert 0.060 <= p["plate_tine_h"] <= 0.090
    # v4 ROBOT bank: 40 mm pitch — the margin-driven minimum (a 5 mm-inflated 14.4 mm disc
    # plus release tolerances cannot thread the Whirlpool 30 mm pitch, measured)
    assert 0.036 <= p["plate_tine_pitch"] <= 0.048
    assert 5.0 <= p["plate_tine_lean_deg"] <= 10.0
    # rows split for true two-point plate support
    assert 0.045 <= p["plate_rows_y"][1] - p["plate_rows_y"][0] <= 0.065
    # the fill-only REAR bank keeps the Whirlpool 12-tine/11-slot row verbatim
    p2 = p["plate_bank2"]
    assert 0.028 <= p2["plate_tine_pitch"] <= 0.032
    assert 0.06 <= p["wire_dia_light"] / p2["plate_tine_pitch"] <= 0.09
    # v4: the bowl lean fixture is gone (bowls stand upright — see config v4 note)
    assert "bowl_tine_xs" not in p


def test_measured_zone_gaps(racks):
    p = config.RACK_GEN[LOWER]
    r_load, r_light, r_heavy = p["wire_dia_load"] / 2, p["wire_dia_light"] / 2, p["wire_dia_heavy"] / 2
    runner_r = {x: (r_heavy if i in (0, len(p["runner_xs"]) - 1) else r_load) for i, x in enumerate(p["runner_xs"])}
    # dense zone: one light runner per span midpoint, surface gaps 15-20 mm
    dense_xs = sorted(
        {round(float(np.mean(part.mesh.vertices[:, 0])), 4) for part in racks[LOWER] if part.name.startswith("dense_runner")}
    )
    assert len(dense_xs) == 6  # 7 spans, one bridged by the channel (both banks share the grid)
    combined = sorted([(x, runner_r[x]) for x in p["runner_xs"]] + [(x, r_light) for x in dense_xs])
    for (xa, ra), (xb, rb) in zip(combined[:-1], combined[1:]):
        gap = xb - xa - ra - rb
        if xb - xa > 0.030:  # the channel-bridged span — excluded by design
            continue
        assert 0.0145 <= gap <= 0.0205, f"dense-zone surface gap {gap * 1e3:.1f} mm outside 15-20 mm"
    # open zone: load-runner surface gaps ~40 mm (real racks' upper bound), crossbars ~42 mm
    for (xa, ra), (xb, rb) in zip(
        [(x, runner_r[x]) for x in p["runner_xs"][:-1]], [(x, runner_r[x]) for x in p["runner_xs"][1:]]
    ):
        if xb - xa > 0.045:  # channel span
            continue
        assert 0.036 <= (xb - xa - ra - rb) <= 0.044
    cross_gap = float(np.diff(p["crossbar_ys"]).min()) - p["wire_dia_light"]
    assert 0.040 <= cross_gap <= 0.060
    # plate-tine surface gaps: robot bank at the margin-driven 40 mm pitch, rear fill bank
    # at the Whirlpool 30 mm
    assert 0.034 <= p["plate_tine_pitch"] - p["wire_dia_light"] <= 0.046
    assert 0.026 <= p["plate_bank2"]["plate_tine_pitch"] - p["wire_dia_light"] <= 0.030


def test_measured_tine_geometry(racks):
    p = config.RACK_GEN[LOWER]
    # v4: TWO banks — the robot front bank (main keys) and the fill-only rear bank
    xs = rack_gen._plate_tine_xs(p)
    xs2 = rack_gen._plate_tine_xs({**p, **p["plate_bank2"]})
    straights = [part for part in racks[LOWER] if part.name.startswith("plate_tine_")]
    canes = [part for part in racks[LOWER] if part.name.startswith("cane_shaft_")]
    arcs = [part for part in racks[LOWER] if part.name.startswith("cane_arc_")]
    beads = [part for part in racks[LOWER] if part.name.startswith("bead_")]
    fillets = [part for part in racks[LOWER] if part.name.startswith("fillet_")]
    assert len(straights) == 2 * (len(xs) - 2) + 2 * (len(xs2) - 2)
    assert len(canes) == 8 and len(arcs) == 8 * p["candy_cane"]["segments"]
    assert len(beads) == len(straights)
    assert len(fillets) == 2 * (len(xs) + len(xs2)) * p["tine_fillet"]["segments"]
    zl = rack_gen._z_levels(p)
    top_expect = zl["tine_base"] + p["plate_tine_h"]
    # bead caps define the exact tine-top plane (Frigidaire push-on caps)
    for part in beads:
        assert abs(part.mesh.bounds[1][2] - top_expect) < 1e-9
        assert part.mesh.bounds[1][0] - part.mesh.bounds[0][0] > (p["wire_dia_light"] * 1.4)
    # candy-cane apex wire surface reaches the same plane to within the 12-gon facet sag
    # (apex-adjacent segments are ~15 deg off horizontal: r * (1 - cos 15) ~ 0.04 mm)
    cane_top = max(part.mesh.bounds[1][2] for part in arcs)
    assert abs(cane_top - top_expect) < 1e-4
    # backward lean: shaft top sheared toward +y by tan(lean) * height
    part = straights[0]
    mn, mx = part.mesh.bounds
    top = part.mesh.vertices[part.mesh.vertices[:, 2] > mx[2] - 1e-6]
    base = part.mesh.vertices[part.mesh.vertices[:, 2] < mn[2] + 1e-6]
    dy = float(np.mean(top[:, 1]) - np.mean(base[:, 1]))
    assert abs(dy - math.tan(math.radians(p["plate_tine_lean_deg"])) * (mx[2] - mn[2])) < 1e-4
    # tie wires: only the fill-only rear bank has them (the robot bank is tie-less — a tie
    # crosses every gap through the disc plane, measured sole plate blocker)
    ties = [part for part in racks[LOWER] if part.zone == "tie"]
    assert len(ties) == 2
    tz = float(np.mean(ties[0].mesh.vertices[:, 2]))
    tie_frac = p["plate_bank2"]["tine_tie_frac"]
    assert abs(tz - (zl["tine_base"] + tie_frac * p["plate_tine_h"])) < 1e-3
    # v4: no bowl tines (bowls stand upright on the open floor)
    assert not [part for part in racks[LOWER] if part.zone == "bowl_tines"]


def test_feature_presence(racks):
    for name, parts in racks.items():
        zones = {p.zone for p in parts}
        assert "wheels" in zones, f"{name}: no roller wheels"
        wheels = [p for p in parts if p.name.startswith("wheel_") and "hub" not in p.name]
        assert len(wheels) == 4
        W = config.RACK_GEN[name]["footprint"][0]
        for w in wheels:
            mn, mx = w.mesh.bounds
            assert mn[2] > -1e-9  # bottoms exactly at/above z=0
            assert mn[0] > -1e-9 and mx[0] < W + 1e-9  # flush inside the footprint
    lower_zones = {p.zone for p in racks[LOWER]}
    assert "handle" in lower_zones and "insert" in lower_zones
    upper_zones = {p.zone for p in racks[UPPER]}
    assert "cup_shelf" in upper_zones and "rackmatic" in upper_zones
    # dipped front rail: the lower front rail's minimum height < the side-rail height
    p = config.RACK_GEN[LOWER]
    front_parts = [q for q in racks[LOWER] if q.name.startswith("rail_top_f_front_")]
    assert len(front_parts) == 5  # stub, drop, dip, drop, stub
    dip_min = min(q.mesh.bounds[0][2] for q in front_parts)
    assert dip_min < p["rim_front_h"] - 0.002


# ---------------------------------------------------------------------------------------------
# 2. convexity / watertightness
# ---------------------------------------------------------------------------------------------


def test_parts_convex_watertight(racks):
    assert len(racks[LOWER]) > 300
    for name, parts in racks.items():
        for part in parts:
            m = part.mesh
            assert m.is_volume, f"{name}/{part.name}: not a watertight, consistently wound volume"
            hull_vol = m.convex_hull.volume
            assert hull_vol <= m.volume * 1.01 + 1e-12, (
                f"{name}/{part.name}: not convex (hull {hull_vol:.3e} vs mesh {m.volume:.3e})"
            )


# ---------------------------------------------------------------------------------------------
# 3. assembled bounds / envelope
# ---------------------------------------------------------------------------------------------


def test_bounds_and_envelope(racks):
    for name, parts in racks.items():
        p = config.RACK_GEN[name]
        mn, mx = rack_gen.merged_mesh(parts).bounds
        assert np.abs(mn).max() < 1e-6, f"{name}: min corner {mn} not at the origin"
        assert abs(mx[0] - p["footprint"][0]) < 1e-6
        assert abs(mx[1] - p["footprint"][1]) < 1e-6
    lo_top = rack_gen.merged_mesh(racks[LOWER]).bounds[1][2]
    p = config.RACK_GEN[LOWER]
    # v4: the handleless basket's wall top is the bbox top (the arch bar left with the handle)
    b = p["basket"]
    wall_top = rack_gen.floor_top_z(p) + b["h"]
    assert abs(lo_top - wall_top) < 1e-6
    assert lo_top <= 0.14
    assert rack_gen.merged_mesh(racks[UPPER]).bounds[1][2] <= 0.10


# ---------------------------------------------------------------------------------------------
# 4. floor-datum contract (placement.derive_slots_from_rack)
# ---------------------------------------------------------------------------------------------


def test_floor_datum(racks):
    floor_zones = {"floor_open", "floor_dense", "channel", "ribs", "slope"}
    for name, parts in racks.items():
        p = config.RACK_GEN[name]
        f_top = rack_gen.floor_top_z(p)
        for part in parts:
            if part.zone in floor_zones:
                z_max = part.mesh.bounds[1][2]
                assert z_max <= 0.015 + 1e-9, f"{name}/{part.name}: floor part top {z_max:.4f} above the 15 mm band"
                # recess surfaces (wire TOPS — what an object rests on) stay within 8 mm of the datum
                assert z_max >= f_top - 0.008 - 1e-9, f"{name}/{part.name}: resting surface deeper than 8 mm below floor top"
    # the 95th-percentile datum (exactly as placement.py computes it) must land on the open
    # floor top of the LOWER rack — wheels/fillets/insert hardware must not drag it upward
    merged = rack_gen.merged_mesh(racks[LOWER])
    verts = merged.vertices
    mn = merged.bounds[0]
    bottom = verts[verts[:, 2] < mn[2] + 0.015]
    datum = float(np.percentile(bottom[:, 2], 95))
    f_top = rack_gen.floor_top_z(config.RACK_GEN[LOWER])
    assert f_top - 0.0015 <= datum <= f_top + 0.0005, f"slot datum {datum:.4f} off the open floor top {f_top:.4f}"


# ---------------------------------------------------------------------------------------------
# 5. FCL probes (exactly as scripts/setup/decompose_meshes.py runs them)
# ---------------------------------------------------------------------------------------------


def test_probes(racks):
    for name, parts in racks.items():
        p = config.RACK_GEN[name]
        mgr = _manager(parts)
        for i, (ext, ctr) in enumerate(rack_gen.mug_probes(p)):
            assert not _hits(mgr, ext, ctr), f"{name}: mug probe #{i} collides"
        if "plate_rows_y" in p:
            for i, (ext, ctr) in enumerate(rack_gen.plate_gap_probes(p)):
                assert not _hits(mgr, ext, ctr), f"{name}: plate-gap probe #{i} collides"
            ext, ctr = rack_gen.plate_tine_negative_probe(p)
            assert _hits(mgr, ext, ctr), f"{name}: negative control on a tine is unexpectedly free"


# ---------------------------------------------------------------------------------------------
# 6. Phase-E slot guarantee
# ---------------------------------------------------------------------------------------------


def test_slot_feasibility_guarantee(racks):
    """The open floor must accept a standing object — the same probe the bake gate runs.

    v4 retired the old 147 mm clearance-cube grid sweep: the quadrant rack is deliberately
    fixture-dense, and real feasibility is yaw-dependent (the goal funnel measures 5 cup
    cells where the cube proxy finds none). What this test guards Kit-free is the layout
    contract the pipeline enforces at bake time: every configured open zone holds an
    object-sized free column (mirrors scripts/setup/decompose_meshes.py's mug-zone probes;
    the end guarantee — ``min_feasible_slots`` per state — is asserted by goal_configs).
    """
    parts = racks[LOWER]
    p = config.RACK_GEN[LOWER]
    mgr = _manager(parts)
    for i, (ext, ctr) in enumerate(rack_gen.mug_probes(p)):
        assert not _hits(mgr, ext, ctr), f"open zone {i} obstructed for a standing object"
    # and the zones must genuinely sit on the open floor (not over a fixture footprint)
    b = p["basket"]
    for x0, x1, y0, y1 in p["open_zones"]:
        assert x1 <= b["x"][0] or x0 >= b["x"][1] or y0 >= b["y"][1], (
            "open zone overlaps the basket footprint"
        )


# ---------------------------------------------------------------------------------------------
# 7. insert group / two-mesh split
# ---------------------------------------------------------------------------------------------


def test_insert_group(racks):
    groups = rack_gen.parts_by_group(racks[LOWER])
    assert set(groups) == {"frame", "insert", "basket"}
    insert = groups["insert"]
    p = config.RACK_GEN[LOWER]
    # v4: the fold-down insert hardware lives on the fill-only rear bank (plate_bank2)
    row = p["plate_bank2"]["insert"]["row"]
    # the insert = the configured row's tine hardware + spine/bosses/clip/tie, nothing else
    for part in insert:
        assert part.zone in ("plate_tines", "insert", "tie"), f"unexpected insert part {part.name}"
        if part.name[-1].isdigit() and f"_r{row}_" in part.name:
            continue
    row_tags = {f"_r{row}_", "insert_", f"tine_tie_{row}"}
    for part in insert:
        assert any(t in part.name or part.name.startswith(t) for t in row_tags), (
            f"{part.name} in the insert group but not row-{row} hardware"
        )
    # the OTHER row stays in the frame
    other = 1 - row
    assert all(f"_r{other}_" not in part.name for part in insert)
    # all groups merged reproduce the full bounds
    full = rack_gen.merged_mesh(racks[LOWER]).bounds
    bounds = [rack_gen.merged_mesh(g).bounds for g in groups.values()]
    union_lo = np.min([b[0] for b in bounds], axis=0)
    union_hi = np.max([b[1] for b in bounds], axis=0)
    assert np.allclose(full[0], union_lo) and np.allclose(full[1], union_hi)
    # the upper rack has neither insert nor basket
    assert set(rack_gen.parts_by_group(racks[UPPER])) == {"frame"}


# ---------------------------------------------------------------------------------------------
# 7b. cutlery basket (v3)
# ---------------------------------------------------------------------------------------------


def test_basket_geometry_contracts(racks):
    p = config.RACK_GEN[LOWER]
    b = p["basket"]
    W, D = p["footprint"]
    # v4: inside the footprint, clear of the open floor zone (its own region now) and IN
    # FRONT of the robot plate bank's corridor
    ox0, ox1, oy0, oy1 = p["open_zones"][0]
    assert b["x"][0] >= ox1 and b["x"][1] <= W
    assert 0.0 < b["y"][0] < b["y"][1] <= min(p["plate_zone_y"][0], D)
    # 3 bays; each must hold the inflated cutlery cross-section in its narrow axis
    # (>= 34 mm, measured: fork head 16.4 mm + 2x5 mm margin + drop tolerance) and the
    # item line in its long axis (>= 44 mm)
    bays = rack_gen.basket_bays(p)
    assert len(bays) == 3
    for x0, x1, y0, y1 in bays:
        assert min(x1 - x0, y1 - y0) >= 0.034 and max(x1 - x0, y1 - y0) >= 0.044
    # basket parts stay inside the assembled bounds (exact-bounds test covers the rest)
    basket_parts = [part for part in racks[LOWER] if part.group == "basket"]
    assert len(basket_parts) == 7  # floor + 4 walls + 2 dividers (handleless in v4)
    merged = rack_gen.merged_mesh(basket_parts)
    assert merged.bounds[0][2] >= rack_gen.floor_top_z(p) - 1e-9
    assert merged.bounds[1][2] <= 0.14  # under the lower-rack height budget


def test_basket_probes(racks):
    p = config.RACK_GEN[LOWER]
    mgr = _manager(racks[LOWER])
    # a cutlery box in every bay is free
    for i, (ext, ctr) in enumerate(rack_gen.basket_probes(p)):
        assert not _hits(mgr, ext, ctr), f"basket bay {i} obstructed"
    # negative control: a box straddling a divider collides
    ext, ctr = rack_gen.basket_divider_negative_probe(p)
    assert _hits(mgr, ext, ctr), "divider negative control unexpectedly free"
    # the clipped open zone excludes the basket footprint
    zones = rack_gen.open_zones_effective(p)
    assert all(x1 <= p["basket"]["x"][0] for _, x1, _, _ in zones)


# ---------------------------------------------------------------------------------------------
# 8. rack manipulation geometry (rack_ops) — both scenarios' actions must be reachable
# ---------------------------------------------------------------------------------------------


def _rack_T_base_body(body: str, extension: float) -> np.ndarray:
    """Rack body pose in the base frame from the measured spawn + body offsets (no cache)."""
    offsets = {
        "E_shelf_1_04": np.array([-0.1746608, -0.1854389, 0.0875132]),
        "E_shelf_03": np.array([-0.1829912, -0.1854389, 0.2417593]),
    }
    T_w_m = np.eye(4)
    c, s = np.cos(-np.pi / 2), np.sin(-np.pi / 2)
    T_w_m[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T_w_m[:3, 3] = np.array(config.DISHWASHER_POS_W)
    T_m_b = np.eye(4)
    T_m_b[:3, 3] = offsets[body] + np.array([0.0, extension, 0.0])
    T_base_w = np.eye(4)
    T_base_w[:3, 3] = -np.array(config.ROBOT_BASE_POS_W)
    return T_base_w @ T_w_m @ T_m_b


def test_rack_action_handles_reachable():
    """Every scenario's rack_action handle must have analytic IK at engage and both slide ends."""
    from dishsim import rack_ops
    from dishsim.transforms import T_inv
    from dishsim.ur5e_kin import ik_wrist3_all

    T_w3_tcp_inv = T_inv(rack_ops.t_wrist3_tcp())
    for name, sc in config.SCENARIOS.items():
        action = sc["rack_action"]
        params = config.RACK_GEN[action["body"]]
        assert "handle" in params, f"{action['body']} has no handle for the {name} rack_action"
        e0 = sc["rack_upper_m"] if action["joint"].endswith("_up") else sc["rack_lower_m"]
        T_body = _rack_T_base_body(action["body"], float(e0))
        T_engage = rack_ops.engage_pose(T_body, params, action, float(e0))
        axis = rack_ops.slide_axis_base(T_body)
        for s, tag in ((0.0, "engage"), (1.0, "slide-end")):
            T = T_engage.copy()
            T[:3, 3] = T_engage[:3, 3] + axis * (float(action["to"]) - float(e0)) * s
            sols = ik_wrist3_all(T @ T_w3_tcp_inv, q_seed=np.array(config.HOME_Q))
            assert len(sols) > 0, f"{name}: no IK at the {tag} pose of {action['body']}"


def test_mug_countertop_pick_reachable():
    """The countertop mug's pre-grasp and grasp TCP poses must have analytic IK branches."""
    from dishsim import rack_ops
    from dishsim.transforms import T_inv, make_T
    from dishsim.ur5e_kin import ik_wrist3_all
    from scipy.spatial.transform import Rotation

    (pos_w, yaw_deg) = config.OBJECT_COUNTERTOP_POSES_W[0]
    pos_b = np.array(pos_w) - np.array(config.ROBOT_BASE_POS_W)
    rot = Rotation.from_euler("z", np.radians(yaw_deg)) * Rotation.from_euler("x", np.pi / 2)
    T_base_obj = make_T(pos_b, rot.as_quat())
    T_tcp_obj = make_T(config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT)
    T_grasp = T_base_obj @ T_inv(T_tcp_obj)
    T_w3_tcp_inv = T_inv(rack_ops.t_wrist3_tcp())
    for dz, tag in ((config.PICK_HOVER_M, "pre-grasp"), (0.0, "grasp")):
        T = T_grasp.copy()
        T[2, 3] += dz
        sols = ik_wrist3_all(T @ T_w3_tcp_inv, q_seed=np.array(config.HOME_Q))
        assert len(sols) > 0, f"no IK at the countertop {tag} pose"


# ---------------------------------------------------------------------------------------------
# 9. determinism, pre-compensation, cache invalidation
# ---------------------------------------------------------------------------------------------


def test_determinism(racks):
    rebuilt = rack_gen.build_rack(config.RACK_GEN[LOWER])
    assert len(rebuilt) == len(racks[LOWER])
    a = rack_gen.merged_mesh(racks[LOWER])
    b = rack_gen.merged_mesh(rebuilt)
    assert np.array_equal(a.vertices, b.vertices)
    assert np.array_equal(a.faces, b.faces)


def test_usd_scale_precompensation(racks):
    p = config.RACK_GEN[LOWER]
    for group, gparts in rack_gen.parts_by_group(racks[LOWER]).items():
        points, counts, indices = rack_gen.mesh_arrays_usd(gparts, p["usd_scale_x"])
        merged = rack_gen.merged_mesh(gparts)
        assert counts.sum() == len(indices) == 3 * len(merged.faces)
        restored = points[:, 0].astype(np.float64) * p["usd_scale_x"]
        assert np.abs(restored - merged.vertices[:, 0]).max() < 1e-5, group  # float32 round-trip
        assert np.abs(points[:, 1] - merged.vertices[:, 1].astype(np.float32)).max() == 0.0


def test_params_hash_and_config_hash_sensitivity():
    h0 = rack_gen.params_hash(config.RACK_GEN, config.RACK_GEN_VERSION)
    assert h0 == rack_gen.params_hash(config.RACK_GEN, config.RACK_GEN_VERSION)
    assert h0 != rack_gen.params_hash(config.RACK_GEN, config.RACK_GEN_VERSION + 1)

    g0 = geometry.config_hash()
    old = config.RACK_GEN[LOWER]["plate_tine_h"]
    config.RACK_GEN[LOWER]["plate_tine_h"] = old + 0.001
    try:
        assert geometry.config_hash() != g0, "config_hash blind to rack-shape changes"
    finally:
        config.RACK_GEN[LOWER]["plate_tine_h"] = old
    assert geometry.config_hash() == g0
