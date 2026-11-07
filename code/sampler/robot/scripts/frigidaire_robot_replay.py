#!/usr/bin/env python3
"""Warp replay of a logged robot trial (the box's RTX renderer crashes with cameras): the arm and gripper as the
convex hulls of their USD collision meshes at the logged body poses, the dish (HOTEC visual mesh), the racks, the
counter and the pedestal; two views (the episode camera, a close view following the dish), 10 fps mp4.

    code/util/run_py.sh code/sampler/robot/scripts/frigidaire_robot_replay.py \\
        --traj data/results/robot/limits_v1/<run>/trials/traj_bowl_gate_t00.npz --kind bowl --label "..." --out x.mp4

``--strip`` writes a six-frame PNG strip instead of (or with) the mp4. Reads robot_meshes.json beside the trajectory.
``--load-dir data/results/robot/placing_v1/<load> --out <media dir>`` (variant D, plans/2026-10-27-drop-test.md): one mp4 of
the load's drops in order plus the goal and as-landed scene stills, entries upserted into <out>/videos.json. ``--variant
D-side`` (plans/2026-10-27-cup-side-drop.md): those rows, files drop_/landed_<load>_side (the goal still is the D one).
``--variant D-flip`` / ``I`` (plans/2026-10-27-bowls-plates-down.md): the bowls' partial-flip drops (drop_/landed_<load>_flip)
and the plates' top-edge insertions (insert_<load>.mp4, landed_<load>_insert.png); the goal stills stay the D ones.
``--variant D-lip`` / ``I2`` (plans/2026-10-27-bowls-lip-plates-fix.md): the bowls' lip-pinch drops (drop_/landed_<load>_lip)
and the plates' insertions with the fixes (insert_<load>_2.mp4, landed_<load>_insert_2.png). ``--variant D-lip2`` / ``I3``
(plans/2026-10-27-carry-rules.md): the bowls carried from the nearest home branch (drop_/landed_<load>_lip2) and the plates
with the checked retreat (insert_<load>_3.mp4, landed_<load>_insert_3.png). ``--variant D-perch``
(plans/2026-10-28-perch-release.md): the bowls carried tilted steeper than their goal, lowered until the emulated flange force
sensor tripped and let go there (drop_/landed_<load>_perch).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util"), str(ROOT / "code/sampler/scorer/scripts")]
import arrangement_optimization.frigidaire_bench as B  # noqa: E402  (COUNTER, HX, BENCH_CAMERA; frigidaire_dot_rules_render (Warp) is imported to draw)

EPISODE_EYE, EPISODE_TARGET, EPISODE_FOV = (-1.9, -2.7, 1.95), (.05, -.25, .62), 46.4   # ROBOT_CAMERA (42 mm / 36 mm)
CLOSE_OFFSET = np.array([-.55, -.75, .45])
COLORS = {"rack": (.80, .80, .82), "counter": (.72, .70, .66), "pedestal": (.35, .36, .40), "arm": (.55, .62, .72),
          "gripper": (.25, .25, .28), "bowl": (217 / 255, 72 / 255, 1 / 255), "cup": (35 / 255, 139 / 255, 69 / 255),
          "plate": (33 / 255, 113 / 255, 181 / 255)}                       # #d94801, #238b45, #2171b5
# --load, the stills' look (plans/2026-11-05-replay-like-stills.md): the twin's material colours (source/asset.py MATERIALS),
# the door open at 90 deg (the baseline snapshot's Door: the hinge, +90 deg about X), the stills' dark background and ground
MATERIAL_RGB = {"Stainless": (.57, .60, .63), "TubPlastic": (.55, .58, .59), "BlackPlastic": (.025, .031, .037),
                "WheelPlastic": (.047, .055, .064), "Rubber": (.012, .015, .018), "CoatedWire": (.61, .65, .68)}
DOOR_OPEN = ((0., -.27425, .19125), (np.sqrt(.5), 0., 0., np.sqrt(.5)))
STILL_LOOK = {"bg": (.20, .20, .21), "ground": (52 / 255, 52 / 255, 53 / 255)}


def quat_wxyz_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_tris(center, size):
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4),
         (1, 5, 7), (1, 7, 3)]
    return v[np.array(f)]


def component_tris(name):
    """A twin component's visual triangles by material, component frame (components.json via geometry.components()): its
    meshes, its box solids and its cylinders as 24-gon prisms on their axis; ``visual: False`` solids skipped."""
    from asset.geometry import components
    out, c = {}, components()[name]
    for m in c["meshes"].values():
        out.setdefault(m.material, []).append(np.asarray(m.points, float)[np.asarray(m.faces)])
    for s in c["solids"]:
        if s.get("visual") is False:
            continue
        if s["kind"] == "box":
            t = box_tris(s["center"], s["size"])
        else:
            h, a = s["height"] / 2, np.linspace(0., 2 * np.pi, 25)
            lo = np.c_[s["radius"] * np.cos(a), s["radius"] * np.sin(a), np.full(25, -h)]
            hi, c0, c1 = lo * [1, 1, -1], np.tile([0., 0., -h], (24, 1)), np.tile([0., 0., h], (24, 1))
            t = np.concatenate([np.stack(v, 1) for v in ((lo[:-1], lo[1:], hi[1:]), (lo[:-1], hi[1:], hi[:-1]),
                                                         (c0, lo[1:], lo[:-1]), (c1, hi[:-1], hi[1:]))])
            t = t[..., {"X": [2, 0, 1], "Y": [1, 2, 0], "Z": [0, 1, 2]}[s["axis"]]] + s["center"]
        out.setdefault(s["material"], []).append(t)
    return {k: np.concatenate(v) for k, v in out.items()}


def hull_tris(pts):
    from scipy.spatial import ConvexHull
    pts = np.asarray(pts, float)
    if len(pts) < 4:
        return np.zeros((0, 3, 3))
    try:
        h = ConvexHull(pts)
    except Exception:                                    # noqa: BLE001  (flat collider)
        return np.zeros((0, 3, 3))
    return pts[h.simplices]


def robot_tris(meshes_json):
    meshes = json.loads(Path(meshes_json).read_text())
    body_tris = {int(i): np.concatenate([hull_tris(p) for p in lst]) for i, lst in meshes["hulls"].items()}
    return body_tris, {i for i, p in enumerate(meshes["body_paths"]) if "/Gripper/" in p}


def statics(lower, upper, mount=None, twin=False):
    """The racks at the given poses (xyz + quat xyzw), the counter and the pedestal (``mount`` = its base x, y, z; default
    the episode profile's mount, data/results/robot/mount/easy_s0_any.json). ``twin``: the racks in coated-wire grey, the
    cabinet and the open door by material (no basket: lifted out for every robot run)."""
    from sampler.scorer import exposure as E
    mount = mount or json.loads((ROOT / "data/results/robot/mount/easy_s0_any.json").read_text())["best"]["base"]
    rack = MATERIAL_RGB["CoatedWire"] if twin else COLORS["rack"]
    parts, cols = ([E.posed(E.component_triangles("LowerRack"), lower[:3], lower[3:7]),
                    E.posed(E.component_triangles("UpperRack"), upper[:3], upper[3:7]),
                    box_tris(B.COUNTER["center_m"], B.COUNTER["size_m"]),
                    box_tris((mount[0], mount[1], mount[2] / 2), (.16, .16, mount[2]))],
                   [rack, rack, COLORS["counter"], COLORS["pedestal"]])
    for name, pose in (("Cabinet", ((0., 0., 0.), E.IDENTITY)), ("Door", DOOR_OPEN)) if twin else ():
        for mat, t in component_tris(name).items():
            parts.append(E.posed(t, *pose))
            cols.append(MATERIAL_RGB[mat])
    return parts, cols


def sample_parts(d, k, body_tris, grip, dish_local, kind, base, dish=True, others_local=None, tints=None):
    """``base`` (statics) + the arm and gripper hulls at logged sample k (+ the dish and the earlier dishes; ``others_local``
    = one visual mesh per earlier dish, their own kinds: a mixed samples_v1 load; ``tints`` = the stills' colours of the
    dish, then of the earlier dishes, none lightened)."""
    from sampler.scorer import exposure as E
    parts, cols = list(base[0]), list(base[1])
    for i, tri in body_tris.items():
        if len(tri) == 0:
            continue
        R = quat_wxyz_R(d["body_quat_wxyz"][k, i])
        parts.append(tri @ R.T + d["body_pos"][k, i])
        cols.append(COLORS["gripper"] if i in grip else COLORS["arm"])
    if dish:
        parts.append(E.posed(dish_local, d["dish"][k][:3], d["dish"][k][3:7]))
        cols.append(tints[0] if tints else COLORS[kind])
        for j, o in enumerate(d["others"][k] if "others" in d.files else ()):   # placing_v1: the dishes placed earlier
            loc, kd = (others_local[j] if others_local else (dish_local, kind))
            parts.append(E.posed(loc, o[:3], o[3:7]))
            cols.append(tints[1 + j] if tints else tuple(.55 * c + .45 for c in COLORS[kd]))
    return parts, cols


def frame(parts, cols, tgt, W, H, top, sub, ss=1, inset=False):
    """The clips' frame: the episode camera beside a close view of ``tgt``, under a two-line header; ``inset``: the stills'
    camera and look at W x H, the close view a W/4 x H/4 inset at the bottom right. (image, close view)"""
    import frigidaire_dot_rules_render as RD
    from PIL import Image, ImageDraw
    face_obj = np.concatenate([np.full(len(p), j, np.int32) for j, p in enumerate(parts)])
    tri = np.concatenate(parts).astype(np.float32)
    cols_a = np.asarray(cols, np.float32)
    if inset:
        eye, at, cam = B.BENCH_CAMERA["initial"]                         # the vertical fov: 30.23 deg at 4:3
        fov = np.degrees(2 * np.arctan(cam["horizontal_aperture"] / 2 / cam["focal_length"] * H / W))
        view = RD.render(tri, face_obj, cols_a, eye, at, W=W, H=H, fov_deg=fov, ss=ss, **STILL_LOOK)
        close = RD.render(tri, face_obj, cols_a, np.asarray(tgt) + CLOSE_OFFSET, tgt, W=W // 4, H=H // 4, fov_deg=40., ss=ss,
                          **STILL_LOOK)
        view[-(H // 4):, -(W // 4):] = close
    else:
        wide = RD.render(tri, face_obj, cols_a, EPISODE_EYE, EPISODE_TARGET, W=W, H=H, fov_deg=EPISODE_FOV, ss=ss)
        close = RD.render(tri, face_obj, cols_a, np.asarray(tgt) + CLOSE_OFFSET, tgt, W=W, H=H, fov_deg=40., ss=ss)
        view = np.concatenate([wide, close], axis=1)
    img = Image.new("RGB", (view.shape[1], H + 56), (18, 28, 40))     # header bar above the views
    img.paste(Image.fromarray(view), (0, 56))
    dr = ImageDraw.Draw(img)
    if inset:
        dr.rectangle((W - W // 4 - 2, H + 56 - H // 4 - 2, W + 1, H + 57), outline=(18, 28, 40), width=2)   # inset edge
    dr.text((10, 6), top[:150], fill=(246, 249, 252), font=RD.font(18, True))
    dr.text((10, 32), sub, fill=(190, 207, 222), font=RD.font(14))
    return img, close


def drop_outcome(r):
    if r.get("placed_at_goal"):
        return f"{r.get('failed_at')}: placed at its goal pose (no arm motion)"
    if r.get("variant") in ("D-ring", "D-ring-h100") and r.get("stop"):   # plans/2026-10-29-ring-place.md, -30-bowl-drop100.md
        st, sc, sd, jc = r.get("stop") or {}, r.get("stop_check") or {}, r.get("seated") or {}, r.get("jaw_contact") or {}
        stop = (f"{st.get('kind')} stop {st.get('force_N')} N at {sc.get('measured_mm')} mm (pred {sc.get('pred_mm')})"
                if st.get("kind") not in (None, "none") else f"no contact down to {st.get('bound_mm')} mm")
        if r.get("variant") == "D-ring-h100":                      # the release height: planned, measured, by what
            stop = (f"let go {sc.get('measured_mm')} mm above the goal (plan {sc.get('plan_mm')}, {sc.get('by')}"
                    + (f", {st.get('force_N')} N" if st.get("kind") not in (None, "none") else "") + ")")
        return (f"{'EXTRA, arm margin %s mm · ' % r.get('arm_margin_mm') if r.get('extra') else ''}foot ring, spin "
                f"{r.get('spin_deg', 0):.0f}° · {stop} · open, 0.5 s, up 100 mm · "
                f"{'JAW CONTACT after the open' if jc.get('after_open') else 'no jaw contact after the open'} · "
                f"{'SEATED' if sd.get('seated') else 'not seated'} · mouth {r.get('mouth')} · "
                f"{'AT_GOAL' if r.get('at_goal') else 'off goal'}")
    if r.get("variant") == "D-roll" and r.get("stop"):
        st, pm, td, sd, jc = (r.get("stop") or {}, r.get("perch_mm") or {}, r.get("tilt_deg") or {}, r.get("seated") or {},
                              r.get("jaw_contact") or {})
        stop = (f"sensed stop {st.get('force_N')} N at {pm.get('measured')} mm (perch {pm.get('pred')})" if st.get("kind") != "none"
                else f"no contact down to {st.get('bound_mm')} mm")
        return (f"tilt {td.get('carried')}°, spin {r.get('spin_deg', 0):+.0f}° · {stop} · open at contact, backed out · "
                f"{'JAW CONTACT after the open' if jc.get('after_open') else 'no jaw contact after the open'} · "
                f"{'SEATED' if sd.get('seated') else 'not seated'} · mouth {r.get('mouth')}")
    if r.get("variant") in ("I5", "D-sampler") and r.get("failed_at") not in ("no reach", "no seat"):
        st, sc, sd, rel = r.get("stop") or {}, r.get("stop_check") or {}, r.get("seated") or {}, r.get("sampler_release") or {}
        stop = (f"{st.get('kind')} stop {st.get('force_N')} N at {sc.get('measured_mm')} mm (pred {sc.get('pred_mm')})"
                if st.get("kind") not in (None, "none") else f"no contact (at {sc.get('measured_mm')} mm)")
        return (f"{rel.get('hold')} · {stop} · up 20, let go at {r.get('release_h_mm')} mm (sampler {rel.get('above_goal_mm')}) · "
                f"{'SEATED' if sd.get('seated') else 'not seated'} · {'settled' if r.get('settled') else 'NOT settled'}"
                + (f" · mouth {r.get('mouth')}" if r.get("mouth") else ""))
    if r.get("failed_at") == "no perch":
        return "no perch: no tilt, spin and arm branch lets the bowl touch the rack before the hand"
    if r.get("failed_at") == "no reach":
        return ("no reach: no collision-free IK at any drop height or yaw" if "no collision-free IK" in str(r.get("why"))
                else "no reach: collision-free drop poses, but no path to them")
    if r.get("variant") == "D-seat":
        st, sp, td, sd = r.get("stop") or {}, r.get("stop_pred") or {}, r.get("tilt_deg") or {}, r.get("seated") or {}
        rp = sp.get("re_pred") or {}
        stop = (f"{st.get('kind')} stop {st.get('force_N')} N at {st.get('height_mm')} mm (pred {rp.get('kind')} "
                f"{rp.get('height_mm')})" if st.get("kind") not in (None, "none") else f"no contact (at {st.get('height_mm')} mm)")
        return (f"tilt {td.get('carried')}° (goal {td.get('goal')}°) · {stop} · "
                f"{(r.get('retreat') or {}).get('outcome', '?')} · {'SEATED' if sd.get('seated') else 'not seated'} · "
                f"mouth {r.get('mouth')}")
    if r.get("variant") == "I4":
        st, sm, sd = r.get("stop") or {}, r.get("seat_mm") or {}, r.get("seated") or {}
        hp = (r.get("hand_stop_mm") or {}).get("pred")
        stop = (f"{st.get('kind')} stop {st.get('force_N')} N at {sm.get('measured')} mm (seat {sm.get('pred')}, hand "
                f"{hp if hp is not None else '-'})" if st.get("kind") not in (None, "none") else
                f"no contact, let go at {sm.get('measured')} mm (seat {sm.get('pred')})")
        return (f"tine axis, rim {r.get('hold_depth_mm', 0):.0f} mm · {stop} · {'SEATED' if sd.get('seated') else 'not seated'}"
                f" · {'settled' if r.get('settled') else 'NOT settled'}")
    if r.get("variant") == "I3":
        rt = r.get("retreat") or {}
        rise = rt.get("rise")
        return (f"{'settled' if r.get('settled') else 'NOT settled'} · {r.get('side')} face · lowered {r.get('lowered_mm')} mm "
                f"from +{r['start_height_m'] * 1e3:.0f} mm, stop: {r.get('stop_reason')} · back {rt.get('back_mm')} mm, "
                + ("rose" if rise is True else "rise blocked" if rise is False else "no rise (fingers at the rim)")
                + f" · {'in' if r.get('in_slot') else 'NOT in'} its slot")
    if r.get("variant") in ("I", "I2"):
        return (f"{'settled' if r.get('settled') else 'NOT settled'} · gripper on the {r.get('side')} face · lowered "
                f"{r.get('lowered_mm')} mm from +{r['start_height_m'] * 1e3:.0f} mm, stop: {r.get('stop_reason')} · "
                f"{'in' if r.get('in_slot') else 'NOT in'} its slot · landed on {', '.join(r.get('landed_on') or []) or 'nothing'}")
    if r.get("variant") == "D-perch":
        st, pm, td = r.get("stop") or {}, r.get("perch_mm") or {}, r.get("tilt_deg") or {}
        pair = (st.get("pairs") or [["none"]])[0][0].replace("|", " vs ")
        stop = (f"sensed stop {st.get('force_N')} N ({pair}) at {pm.get('measured')} mm (predicted {pm.get('pred')} mm)"
                if st.get("kind") != "none" else f"no contact down to {st.get('bound_mm')} mm (bowl at {pm.get('measured')} mm)")
        return (f"tilt {td.get('carried')}° (goal {td.get('goal')}°), spin {r.get('spin_deg', 0):+.0f}° · {stop} · roll "
                f"predicted {(r.get('roll_pred') or {}).get('sign')} · {'settled' if r.get('settled') else 'NOT settled'} · "
                f"mouth {r.get('mouth')}")
    if r.get("variant") == "D-lip2":
        lg = r.get("legs") or {}
        return (f"spin {r.get('spin_deg', 0):+.0f}° · largest joint travel {r.get('carry_cost_deg', 0):.0f}° (predicted "
                f"{r.get('carry_cost_pred', 0):.0f}°, legs {lg.get('leg1')} / {lg.get('leg2')}) · let go "
                f"{r['drop_height_m'] * 1e3:.0f} mm above the goal · {'settled' if r.get('settled') else 'NOT settled'} · "
                f"mouth {r.get('mouth')}")
    if r.get("variant") == "D-lip":
        return (f"lip pinch, spin {r.get('spin_deg', 0):+.0f}° · {'settled' if r.get('settled') else 'NOT settled'} · let go "
                f"{r['drop_height_m'] * 1e3:.0f} mm above the goal · landed on {', '.join(r.get('landed_on') or []) or 'nothing'}"
                f" · mouth {r.get('mouth')}")
    if r.get("variant") == "D-flip":
        return (f"flip tilt {r.get('flip_tilt_deg', 0):.0f}°{'' if r.get('flip_clear') else ' (palm)'} · "
                f"{'settled' if r.get('settled') else 'NOT settled'} · let go {r['drop_height_m'] * 1e3:.0f} mm above the "
                f"goal · landed on {', '.join(r.get('landed_on') or []) or 'nothing'} · mouth {r.get('mouth')}")
    return (f"{'settled' if r.get('settled') else 'NOT settled'} · let go {r['drop_height_m'] * 1e3:.0f} mm above the goal · "
            f"landed on {', '.join(r.get('landed_on') or []) or 'nothing'}" + (f" · mouth {r['mouth']}" if r.get("mouth") else ""))


def drop_load(a):
    """Variant D of one load: drop_<load>.mp4 (the D rows' trajectories in the witness order, each dish's last frame held
    1 s), goal_<load>.png (every dish at its goal pose) and landed_<load>.png (the D rows' final poses), both with the racks
    extended (the goal records' frames; a dish's own rack is extended when it drops) and the arm at home (the first
    logged sample); the clips' two views."""
    import imageio
    from sampler.scorer import exposure as E
    load = json.loads((a.load_dir / "load.json").read_text())
    L, kind = load["load"], load["kind"]
    var, sfx = a.variant, {"D": "", "D-side": "_side", "D-side-m2": "_side_m2", "D-flip": "_flip", "I": "_insert",
                           "D-lip": "_lip", "I2": "_insert_2", "D-lip2": "_lip2", "I3": "_insert_3", "D-perch": "_perch",
                           "I4": "_insert_4", "D-seat": "_seat", "I5": "_insert_5", "D-sampler": "_sampler",
                           "D-roll": "_roll", "D-ring": "_ring", "D-ring-h100": "_ring_h100"}[a.variant]
    ins = var in ("I", "I2", "I3", "I4", "I5")
    rows = {}
    for line in (a.load_dir / "dishes.jsonl").read_text().splitlines():
        r = json.loads(line) if line.strip() else None
        if r and r.get("variant") == var:
            rows[r["dish"]] = r                                    # the last row of the variant per dish
    names = [x["name"] for x in sorted(load["dishes"], key=lambda x: x["order_index"])]
    done = [n for n in names if n in rows]
    trajs = [n for n in done if rows[n].get("traj")]
    if not trajs:
        raise SystemExit(f"[RESULT] FAIL {L}: no variant {var} trajectory in {a.load_dir}/dishes.jsonl")
    fy = rows[trajs[0]].get("counter_front_y")
    if fy is not None:                                             # the run's counter edge (the D rows: -0.30, unrecorded)
        from sampler.robot import grasp as G
        G.pull_counter(B, fy)
    body_tris, grip = robot_tris((ROOT / rows[trajs[0]]["traj"]).parent / "robot_meshes.json")
    home = np.load(ROOT / rows[trajs[0]]["traj"])                  # sample 0: the arm at home
    dish_local = np.concatenate(E.dish_visuals(f"hotec_{kind}"))
    W, H = (int(v) for v in a.wh.split("x"))
    a.out.mkdir(parents=True, exist_ok=True)
    n_set = sum(bool(rows[n].get("settled")) for n in done)
    n_down = sum(bool(rows[n].get("settled")) and rows[n].get("mouth") == "down" for n in done)
    n_seat = sum(bool((rows[n].get("seated") or {}).get("seated")) for n in done)
    ran = [n for n in done if not rows[n].get("placed_at_goal")]
    at_goal = [n for n in done if rows[n].get("placed_at_goal")]
    out_ = (f"seated {n_seat}/{len(ran)}, settled {n_set}/{len(ran)}, in slot {sum(bool(rows[n].get('in_slot')) for n in ran)}/"
            f"{len(ran)}; {len(at_goal)} placed at their goals (sampler release infeasible)" if var == "I5" else
            f"seated mouth down {n_seat}/{len(ran)}, settled {n_set}/{len(ran)}; {len(at_goal)} placed at their goals "
            f"(sampler release infeasible)" if var == "D-sampler" else
            f"seated mouth down {n_seat}/{len(ran)}, jaw contact after the open "
            f"{sum(bool((rows[n].get('jaw_contact') or {}).get('after_open')) for n in ran)}/{len(ran)}; {len(at_goal)} placed "
            f"at their goals (not gated or not releasable)" if var == "D-roll" else
            f"seated mouth down {n_seat}/{len(ran)}, at goal {sum(bool(rows[n].get('at_goal')) for n in ran)}/{len(ran)}, jaw contact "
            f"after the open {sum(bool((rows[n].get('jaw_contact') or {}).get('after_open')) for n in ran)}/{len(ran)}; "
            f"{len(at_goal)} placed at their goals (no reach)" if var in ("D-ring", "D-ring-h100") else
            f"seated {n_seat}/{len(done)}, settled {n_set}/{len(done)}, in slot "
            f"{sum(bool(rows[n].get('in_slot')) for n in done)}/{len(done)}" if var == "I4" else
            f"seated mouth down {n_seat}/{len(done)}, settled {n_set}/{len(done)}, hand away "
            f"{sum(bool(rows[n].get('hand_away')) for n in done)}/{len(done)}" if var == "D-seat" else
            f"settled mouth down {n_down}/{len(done)}, settled {n_set}/{len(done)}" if sfx and not ins else
            f"settled {n_set}/{len(done)}, in slot {sum(bool(rows[n].get('in_slot')) for n in done)}/{len(done)}" if ins
            else f"settled {n_set}/{len(done)}")
    gone = [n for n in done if rows[n].get("failed_at") in ("no reach", "no perch") and not rows[n].get("placed_at_goal")]
    fr = load["frames"]
    mount = (rows[trajs[0]].get("mount") or [None])[:3] or None       # the run's pedestal (rows since the posture fix)
    mount = mount if mount and mount[0] is not None else None
    ext = statics(*[np.r_[fr[r]["position_m"], fr[r]["quaternion_xyzw"]] for r in ("LowerRack", "UpperRack")], mount=mount)
    tgt = np.mean([x["goal_pose"]["pos"] for x in load["dishes"]], axis=0)
    if getattr(a, "goals", None):                                  # D-ring: the refreshed goals (goals_v4.json)
        gv = json.loads(Path(a.goals).read_text())["bowls"]
        for x in load["dishes"]:
            if x["name"] in gv:
                x["goal_pose"] = gv[x["name"]]["goal_pose"]
    stills = (("goal", [(x["goal_pose"]["pos"], x["goal_pose"]["quat_xyzw"]) for x in load["dishes"]],
               f"goal arrangement: the {len(names)} {kind}s at their goal poses"),
              ("landed", [(lp["position_m"], lp["quaternion_xyzw"]) for n in done
                          for lp in [(rows[n].get("after") or {}).get("pose") or rows[n].get("final_pose")] if lp],
               f"as landed (variant {var}{f', counter edge y {fy:+.2f} m' if fy is not None else ''}): {out_}"
               + (f"; {'no perch' if var == 'D-perch' else 'no reach'}, not drawn: {', '.join(gone)}" if gone else "")
               + (f"; at their goal poses by teleport: {', '.join(at_goal)}" if at_goal else "")
               + "".join(f"; {n}: the flagged {rows[n].get('arm_margin_mm'):.0f} mm try (in the clip), then teleported to its goal"
                         for n in done if rows[n].get("extra") and rows[n].get("after"))))
    for tag, poses, what in stills[1:] if sfx else stills:        # D-side reuses the D goal still
        parts, cols = sample_parts(home, 0, body_tris, grip, dish_local, kind, ext, dish=False)
        for p_, q_ in poses:
            parts.append(E.posed(dish_local, p_, q_))
            cols.append(COLORS[kind])
        img, _ = frame(parts, cols, tgt, W, H, f"{L}: {what}", "left: episode camera, right: close view of the load · arm at "
                       "home · racks extended (the goal records' frames) · Warp render (collision hulls)", ss=2)
        img.save(a.out / f"{tag}_{L}{sfx}.png")
    mp4 = a.out / (f"insert_{L}{ {'I2': '_2', 'I3': '_3', 'I4': '_4', 'I5': '_5'}.get(var, '')}.mp4" if ins else f"drop_{L}{sfx}.mp4")
    writer = imageio.get_writer(str(mp4), fps=10, codec="libx264", quality=7, macro_block_size=None)
    nf = 0
    for n in trajs:
        r, d = rows[n], np.load(ROOT / rows[n]["traj"])
        base = statics(d["lower"][0], d["upper"][0], mount=mount)
        top = f"{L}{f' ({var})' if sfx else ''} · {names.index(n) + 1}/{len(names)} {n}: {drop_outcome(r)}"
        ks = list(range(0, len(d["t"]), a.every))
        for k in ks:
            img, _ = frame(*sample_parts(d, k, body_tris, grip, dish_local, kind, base), d["dish"][k][:3], W, H, top,
                           f"simulated t = {d['t'][k] - d['t'][0]:6.2f} s   jaw {d['jaw'][k]:.2f} rad   left: episode camera, "
                           f"right: following the dish   Warp replay of the logged Isaac trial (collision hulls)")
            for _ in range(10 if k == ks[-1] else 1):              # the outcome held 1 s at the end of each dish
                writer.append_data(np.asarray(img))
                nf += 1
    writer.close()
    man_p = a.out / "videos.json"
    man = json.loads(man_p.read_text()) if man_p.is_file() else []
    held = {"_flip": "held by the rim pinch and turned about the closing axis to fall mouth down",
            "_insert": "held at the assumed top-edge rim hold at its goal orientation, lowered until it touches",
            "_lip": "held by the rim pinch at the lip, carried at its goal lean",
            "_insert_2": "held at the assumed top-edge rim hold at its goal orientation, lowered until it touches (bugs fixed)",
            "_lip2": "held by the rim pinch at the lip, carried from the home branch and rim spin with the smallest joint travel",
            "_perch": "held by the rim pinch at the lip, carried tilted steeper than its goal, lowered at 20 mm/s until the "
                      "emulated flange force sensor tripped",
            "_insert_3": "held at the assumed top-edge rim hold, lowered until it touches, the open hand backing out through a "
                         "checked line and rising only clear of the rim",
            "_insert_4": "held on its base face at the assumed top-edge hold (rim 6 mm into the pads), lowered along the tine "
                         "axis at 20 mm/s until the emulated flange force sensor tripped, let go there, the open hand backing "
                         "out through a checked line",
            "_seat": "held by the rim pinch at the lip, carried at the tilt whose predicted stop is lowest above its seat, "
                     "lowered at 20 mm/s until the emulated flange force sensor tripped, let go there, the hand taken away "
                     "before the settle",
            "_insert_5": "held upright (the sampler's release orientation) at the mirrored top-edge hold (rim 6 mm into the "
                         "pads), carried to 50 mm above the sampler's release pose, lowered straight down at 20 mm/s until the "
                         "emulated flange force sensor tripped, raised 20 mm, let go, the open hand straight up 0.10 m",
            "_roll": "held by the rim pinch at the lip, carried at the perch-roll curve's best tilt and a spin whose open hand "
                     "the recorded rolling bowl never meets, lowered at 20 mm/s until the emulated flange force sensor tripped on "
                     "the tine tip, let go there, the open hand backing out along its approach",
            "_sampler": "held by the rim pinch at the lip in its packed pose (the sampler's release orientation), carried to "
                        "50 mm above the sampler's release pose, lowered straight down at 20 mm/s until the emulated flange "
                        "force sensor tripped, raised 20 mm, let go, the open hand straight up 0.10 m",
            "_ring_h100": "picked mouth down by the foot ring, carried mouth down with the hand above, lowered at 20 mm/s by "
                          "position to its release height above the goal (100 mm; the flange sensor as a guard), let go there, "
                          "0.5 s, the open hand straight up 100 mm"}.get(
        sfx, "held by the side pinch (wrist toward the base), flipped mouth down" if sfx else "carried as held") + (
        f", the pedestal at ({', '.join(f'{v:.2f}' for v in mount)})" if mount and sfx in ("_side_m2", "_flip", "_insert",
                                                                                          "_lip", "_insert_2", "_lip2",
                                                                                          "_insert_3", "_perch",
                                                                                          "_insert_4", "_seat", "_insert_5",
                                                                                          "_sampler", "_roll") else "")
    what_ = "insertions" if ins else "drops"
    new = [{"file": mp4.name, "kind": kind, "load": L, "variant": var, "outcome": out_,
            "caption": f"{L}: the {what_} in the witness order, each {kind} {held}, let go and settled "
                       f"(Warp replay of the logged Isaac run; {out_})"},
           {"file": f"goal_{L}.png", "kind": kind, "load": L, "variant": "D", "outcome": "goal",
            "caption": f"{L}: goal arrangement, every {kind} at its goal pose"},
           {"file": f"landed_{L}{sfx}.png", "kind": kind, "load": L, "variant": var, "outcome": out_,
            "caption": f"{L}: as landed after the {what_} ({out_}" + (f"; {'no perch' if var == 'D-perch' else 'no reach'}, not "
                                                                      f"drawn: {', '.join(gone)}" if gone else "") + ")"}]
    new = new[::2] if sfx else new                                 # D-side: the D goal entry stays as it is
    files = {e["file"] for e in new}
    man_p.write_text(json.dumps([e for e in man if e["file"] not in files] + new, indent=1) + "\n")
    print(f"[RESULT] PASS drop replay {L} {var}: {mp4.name} ({nf} frames, {len(trajs)} dishes), "
          f"{'' if sfx else f'goal_{L}.png, '}landed_{L}{sfx}.png; {out_}", flush=True)


def pose_still(kind, site_name, target, yaw, out, label):
    """Kit-free still of the arm at the first IK solution of a packer-pose pre-place (the colliding pose): the arm
    model's hulls at the analytic link poses, the carried dish, the racks of the cell, the counter and the pedestal."""
    import warp as wp
    from PIL import Image, ImageDraw
    import frigidaire_dot_rules_render as RD
    sys.path.insert(0, str(ROOT / "code/sampler/robot/scripts"))
    import frigidaire_robot_reach as RC
    from sampler.scorer import exposure as E
    from sampler.robot import grasp as G
    from sampler.robot.rig import T_of
    B = RD.B
    wp.init()
    B.HX().register_hotec(B.ASSETS)
    W = RC.World()
    reach = json.loads((RC.OUT / "reach.json").read_text())
    site = next(x for x in reach["sites"][kind] if x["site"] == site_name)
    arm = W.arms[site["rack"]]
    T_o = RC.counter_obj()
    g = G.tilted_grasps(T_o, RC.N_YAW, kind)[yaw]
    M = np.linalg.inv(g["T_tcp"]) @ T_o
    pose = site["world"][target]
    T_rel = T_of(pose["position_m"], pose["quaternion_xyzw"]) @ np.linalg.inv(M)
    T_rel[2, 3] += RC.RELEASE_ABOVE_M
    T_pp = G.hover_along(T_rel, RC.HOVER_M)
    qs = W.kr.ik_all(T_pp, W.seed)
    if not qs:
        raise SystemExit(f"no IK at the pre-place of {kind} {site_name} yaw {yaw}")
    q = qs[0]
    hits = arm.collisions(q, first=False, held=(kind, M, "dish"))
    parts, cols = [], []
    for link, T in arm.link_poses(q).items():
        for _, verts in arm.hulls.get(link, ()):
            tri = hull_tris(verts)
            if len(tri):
                parts.append(tri @ T[:3, :3].T + T[:3, 3])
                cols.append(COLORS["gripper"] if link == "gripper" else COLORS["arm"])
    frames = RC.frames_for(B, site["rack"])
    for name in ("LowerRack", "UpperRack"):
        f = frames[name]
        parts.append(E.posed(E.component_triangles(name), f["position_m"], f["quaternion_xyzw"]))
        cols.append(COLORS["rack"])
    parts.append(box_tris(B.COUNTER["center_m"], B.COUNTER["size_m"]))
    cols.append(COLORS["counter"])
    m = json.loads(RC.MOUNT.read_text())["best"]["base"]
    parts.append(box_tris((m[0], m[1], m[2] / 2), (.16, .16, m[2])))
    cols.append(COLORS["pedestal"])
    T_d = T_pp @ M
    from scipy.spatial.transform import Rotation
    parts.append(E.posed(np.concatenate(E.dish_visuals(f"hotec_{kind}")), T_d[:3, 3], Rotation.from_matrix(T_d[:3, :3]).as_quat()))
    cols.append((.95, .2, .2))                                      # the carried dish in red
    face_obj = np.concatenate([np.full(len(p_), j, np.int32) for j, p_ in enumerate(parts)])
    tri = np.concatenate(parts).astype(np.float32)
    cols_a = np.asarray(cols, np.float32)
    tgt = T_d[:3, 3]
    views = [RD.render(tri, face_obj, cols_a, EPISODE_EYE, EPISODE_TARGET, W=640, H=480, fov_deg=EPISODE_FOV, ss=2),
             RD.render(tri, face_obj, cols_a, tgt + np.array([-.6, -.8, .5]), tgt, W=640, H=480, fov_deg=45., ss=2)]
    img = Image.new("RGB", (1280, 536), (18, 28, 40))
    img.paste(Image.fromarray(np.concatenate(views, axis=1)), (0, 56))
    dr = ImageDraw.Draw(img)
    dr.text((10, 6), label[:150], fill=(246, 249, 252), font=RD.font(18, True))
    dr.text((10, 32), f"pre-place IK solution 1 of {len(qs)} · contacts within 15 mm: {hits[:4]} · Kit-free render (arm model hulls)",
            fill=(190, 207, 222), font=RD.font(14))
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"[RESULT] PASS still {out} hits {hits[:6]}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--still", default=None, help="kind,site,target,yaw: a Kit-free still of the pre-place pose instead")
    ap.add_argument("--traj", type=Path, default=None)
    ap.add_argument("--kind", default=None, help="dish kind (not needed with --load-dir: the load's)")
    ap.add_argument("--load-dir", type=Path, default=None,
                    help="variant D: a placing_v1/<load> folder; --out is then the media folder (drop_/goal_/landed_<load>)")
    ap.add_argument("--variant", choices=("D", "D-side", "D-side-m2", "D-flip", "I", "D-lip", "I2", "D-lip2", "I3", "D-perch",
                                          "I4", "D-seat", "I5", "D-sampler", "D-roll", "D-ring", "D-ring-h100"),
                    default="D",
                    help="--load-dir: whose rows (D-side: _side files; D-side-m2, the posture fix at its mount: _side_m2 files; "
                         "D-flip, the bowls' partial flip: _flip files; I, the plates' insertions: insert_<load>.mp4, "
                         "landed_<load>_insert.png)")
    ap.add_argument("--goals", type=Path, default=None, help="--load-dir: the goal still from these refreshed goals (goals_v4.json)")
    ap.add_argument("--label", default="")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--strip", action="store_true", help="also write a six-frame strip (<out>.png)")
    ap.add_argument("--strip-only", action="store_true")
    ap.add_argument("--wh", default=None, help="each view (default 640x480); with --load the main view (default 1280x960)")
    ap.add_argument("--ss", type=int, default=1, help="supersampling per pixel side")
    ap.add_argument("--every", type=int, default=1, help="mp4: every k-th logged sample (plays k x faster)")
    ap.add_argument("--release-still", action="store_true",
                    help="one PNG (two views) of the logged trial at the release step (the last frame with the jaw closed "
                         "before it opens) instead of the mp4")
    ap.add_argument("--landed-still", action="store_true",
                    help="one PNG (two views) of the logged trial's last frame (the dish as it landed and settled) instead of "
                         "the mp4 (plans/2026-11-04-grasp-place-s0.md)")
    ap.add_argument("--load", type=Path, default=None,
                    help="the stills' look (plans/2026-11-05-replay-like-stills.md): the cabinet and the open door, the "
                         "dishes in item_tints over this load.json's dishes, the bench camera, the dish-following view as an inset")
    ap.add_argument("--rows", type=Path, default=None, help="--load: the load's rows.jsonl (the failed dishes' final_pose)")
    ap.add_argument("--states", type=Path, default=None, help="--load: the chains' state folder (after_NN_<dish>.json)")
    ap.add_argument("--video-order", default=None,
                    help="--load, --states: the clips' order (comma list); the other rack shows what its chain had placed "
                         "before this clip, either rack the dishes that failed before it (at their landed poses)")
    ap.add_argument("--check-scene", action="store_true",
                    help="--load: print this clip's scene (triangles, dishes and their sources, tints, projections, overlaps) "
                         "and stop before Warp")
    a = ap.parse_args()
    a.wh = a.wh or ("1280x960" if a.load else "640x480")
    if a.still:
        kind, site, target, yaw = a.still.split(",")
        return pose_still(kind, site, target, int(yaw), a.out, a.label)
    from sampler.scorer import exposure as E
    B.HX().register_hotec(B.ASSETS)
    if a.load_dir:
        import warp as wp
        wp.init()
        return drop_load(a)
    W, H = (int(v) for v in a.wh.split("x"))
    d = np.load(a.traj)
    body_tris, grip = robot_tris(a.traj.parent / "robot_meshes.json")
    if "counter_front_y" in d.files:                   # samples_v1 trajectories carry the run's counter edge and pedestal
        from sampler.robot import grasp as G
        G.pull_counter(B, float(d["counter_front_y"]))
    base = statics(d["lower"][0], d["upper"][0], mount=[float(v) for v in d["mount"]] if "mount" in d.files else None,
                   twin=a.load is not None)
    for fx in (d["fixtures"] if "fixtures" in d.files else ()):   # the plate presentation's backstop (center, size, axes)
        fx = np.asarray(fx, dtype=float)
        base[0].append(box_tris((0., 0., 0.), fx[3:6]) @ fx[6:15].reshape(3, 3).T + fx[:3])
        base[1].append(COLORS["counter"])
    a.kind = a.kind or (str(d["kind"]) if "kind" in d.files else None)
    dish_local = np.concatenate(E.dish_visuals(f"hotec_{a.kind}"))
    others_local = None
    if "others_kinds" in d.files:                      # a mixed load: each earlier dish drawn with its own kind's mesh
        mesh = {k_: np.concatenate(E.dish_visuals(f"hotec_{k_}")) for k_ in set(map(str, d["others_kinds"]))}
        others_local = [(mesh[str(k_)], str(k_)) for k_ in d["others_kinds"]]
    tints, extra = None, []                            # extra: (dish, rack, source, position, quaternion) the trial did not hold
    if a.load:
        sys.path.insert(0, str(ROOT / "code/sampler/scripts"))
        from frigidaire_initial_state_render import item_tints
        from sampler.random_poses import compose_pose, relative_pose
        load = json.loads(a.load.read_text())
        rack_of, kind_of = ({x["name"]: x[f] for x in load["dishes"]} for f in ("rack", "kind"))
        T = item_tints([{"object_id": n_} for n_ in rack_of])                    # the stills' colours
        name = next(n_ for n_ in rack_of if a.traj.name.startswith(f"traj_{n_}_"))  # traj_<dish>_<move>.npz
        others = list(map(str, d["others_names"])) if "others_names" in d.files else []
        tints, seen = [T[name]] + [T[n_] for n_ in others], {name, *others}
        order = a.video_order.split(",") if a.video_order else []
        rows = {r["dish"]: r for r in map(json.loads, filter(str.strip, a.rows.read_text().splitlines()))} if a.rows else {}
        now = {"LowerRack": d["lower"][0], "UpperRack": d["upper"][0]}          # this clip's rack frames
        for rk in ("UpperRack", "LowerRack"):           # each rack's chain: its latest state file before this clip
            prev = [n_ for n_ in order[:order.index(name)] if rack_of[n_] == rk] if a.states and name in order else []
            fs = [f for n_ in prev for f in sorted(a.states.glob(f"after_*_{n_}.json"))]
            if not fs:
                continue
            st = json.loads(fs[-1].read_text())
            fr = st["rack_frames"][rk]                  # shifted into this clip's rack frame: F_now . F_state^-1 . p
            src = [(n_, f"its chain's {fs[-1].name}", st["placed"][n_]) for n_ in st["placed"]] if rk != rack_of[name] else []
            src += [(n_, "failed: rows final_pose", rows[n_]["final_pose"]) for n_ in st["failed"]
                    if (rows.get(n_) or {}).get("final_pose")]
            for n_, why, pose in src:
                if n_ in seen or rack_of[n_] != rk:
                    continue
                p_, q_ = compose_pose(now[rk][:3], now[rk][3:7], *relative_pose(pose["position_m"], pose["quaternion_xyzw"],
                                                                                 fr["position_m"], fr["quaternion_xyzw"]))
                extra.append((n_, rk, why, p_, q_))
                seen.add(n_)
        mesh = {k_: np.concatenate(E.dish_visuals(f"hotec_{k_}")) for k_ in set(kind_of.values())}
        for n_, _, _, p_, q_ in extra:
            base[0].append(E.posed(mesh[kind_of[n_]], p_, q_))
            base[1].append(T[n_])
    if a.check_scene:                                  # no Warp: what the clip will draw (plans/2026-11-05-replay-like-stills.md)
        import fcl
        from asset.loading import collision_parts
        from sampler.random_poses import quaternion_matrix_xyzw
        n, last = len(d["t"]), len(d["t"]) - 1
        hexc = lambda c: "#%02x%02x%02x" % tuple(round(v * 255) for v in c)        # noqa: E731
        print(f"[scene] {name} ({rack_of[name]}): clip {order.index(name) + 1 if name in order else '-'} of {len(order)} in the "
              f"video order, {n} frames; main view {W} x {H}, inset {W // 4} x {H // 4}, 56 px header")
        cnt = {"cabinet": sum(map(len, component_tris("Cabinet").values())),
               "door": sum(map(len, component_tris("Door").values())),
               "lower rack": len(base[0][0]), "upper rack": len(base[0][1]), "counter": len(base[0][2]),
               "pedestal": len(base[0][3]), "fixtures": 12 * len(d["fixtures"]) if "fixtures" in d.files else 0,
               "arm + gripper hulls": sum(map(len, body_tris.values())), "ground": 2,
               "total at frame 0": sum(map(len, sample_parts(d, 0, body_tris, grip, dish_local, a.kind, base,
                                                            others_local=others_local, tints=tints)[0])) + 2}
        print("[scene] triangles: " + ", ".join(f"{k_} {v}" for k_, v in cnt.items()))
        drawn = ([(name, rack_of[name], f"npz dish (frame {last})", d["dish"][last][:3], d["dish"][last][3:7])]
                 + [(o, rack_of[o], f"npz others (frame {last})", d["others"][last][j][:3], d["others"][last][j][3:7])
                    for j, o in enumerate(others)] + extra)
        fr = load["frames"]                             # the goal poses' rack frames (both racks extended)
        for n_, rk, why, p_, q_ in drawn:
            g = next(x["goal_pose"] for x in load["dishes"] if x["name"] == n_)
            gp, _ = compose_pose(now[rk][:3], now[rk][3:7], *relative_pose(g["pos"], g["quat_xyzw"], fr[rk]["position_m"],
                                                                           fr[rk]["quaternion_xyzw"]))
            print(f"[dish] {rk} {n_} {hexc(T[n_])} {len(mesh[kind_of[n_]])} tris, {why}: position {np.round(p_, 4).tolist()} "
                  f"quat {np.round(q_, 4).tolist()}, {1e3 * np.linalg.norm(np.asarray(p_) - gp):.1f} mm from its goal")
        eye, at, cam = B.BENCH_CAMERA["initial"]        # the main view's projection, as frigidaire_dot_rules_render.render
        eye = np.asarray(eye, float)
        f_ = (np.asarray(at) - eye) / np.linalg.norm(np.asarray(at) - eye)
        r_ = np.cross(f_, [0, 0, 1.]) / np.linalg.norm(np.cross(f_, [0, 0, 1.]))
        u_, th = np.cross(r_, f_), cam["horizontal_aperture"] / 2 / cam["focal_length"] * H / W
        px = lambda v: np.c_[((v - eye) @ r_ / ((v - eye) @ f_) / (th * W / H) + 1) * W / 2 - .5,   # noqa: E731
                             (1 - (v - eye) @ u_ / ((v - eye) @ f_) / th) * H / 2 - .5]
        arm = lambda k: np.concatenate([t @ quat_wxyz_R(d["body_quat_wxyz"][k, i]).T + d["body_pos"][k, i]   # noqa: E731
                                        for i, t in body_tris.items() if len(t)]).reshape(-1, 3)
        pts = {"lower rack centre": base[0][0].reshape(-1, 3), "upper rack centre": base[0][1].reshape(-1, 3),
               "pedestal top": d["mount"], "pick spot": [*load["pick_spot_xy"], B.COUNTER["top_z_m"]]}
        for k_, v in pts.items():
            v = (v.min(0) + v.max(0)) / 2 if np.ndim(v) == 2 else np.asarray(v, float)
            print(f"[view] {k_} {np.round(v, 3).tolist()} -> pixel {np.round(px(v[None])[0]).astype(int).tolist()} of {W} x {H}")
        for k in (0, last):
            q = px(arm(k))
            print(f"[view] arm + gripper at frame {k}: pixels x {q[:, 0].min():.0f}..{q[:, 0].max():.0f}, "
                  f"y {q[:, 1].min():.0f}..{q[:, 1].max():.0f}")
        hulls = {k_: collision_parts(B.ASSETS / f"{k_}.usda") for k_ in set(kind_of.values())}
        def mgr(n_, p_, q_):                            # one dish's collision hulls (the USD's convex pieces) at a pose
            R, m = quaternion_matrix_xyzw(q_), fcl.DynamicAABBTreeCollisionManager()
            m.registerObjects([fcl.CollisionObject(h.geometry, fcl.Transform(R @ h.rotation, R @ h.translation + np.asarray(p_)))
                               for h in hulls[kind_of[n_]]])
            m.setup()
            return m
        def gap(m1, m2):                                # mm; negative: the deepest penetration of two overlapping dishes
            cd = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1000, enable_contact=True))
            m1.collide(m2, cd, fcl.defaultCollisionCallback)
            if cd.result.is_collision:
                return -1e3 * max(c.penetration_depth for c in cd.result.contacts)
            dd = fcl.DistanceData()
            m1.distance(m2, dd, fcl.defaultDistanceCallback)
            return 1e3 * dd.result.min_distance
        ms = [mgr(x[0], x[3], x[4]) for x in drawn]
        near = [(drawn[i][0], drawn[j][0], gap(ms[i], ms[j])) for i in range(len(drawn)) for j in range(i + 1, len(drawn))
                if drawn[i][1] == drawn[j][1]]
        print("[overlap] dish pairs closer than 0 mm at the last frame: " + (", ".join(f"{a_} vs {b_} {g_:.2f} mm" for a_, b_, g_
                                                                                      in near if g_ < 0) or "none"))
        print("[overlap] nearest pairs: " + ", ".join(f"{a_} vs {b_} {g_:.2f} mm"
                                                      for a_, b_, g_ in sorted(near, key=lambda x: x[2])[:6]))
        for i in range(len(drawn) - len(extra), len(drawn)):   # the moving dish against each added dish (every 5th frame)
            if drawn[i][1] == rack_of[name]:
                g_ = [(gap(mgr(name, d["dish"][k][:3], d["dish"][k][3:7]), ms[i]), k) for k in sorted({*range(0, n, 5), last})]
                neg = [k for v, k in g_ if v < 0]
                print(f"[overlap] {name} (moving) vs {drawn[i][0]} ({drawn[i][2]}): closest {min(g_)[0]:.2f} mm at frame "
                      f"{min(g_)[1]}" + (f", below 0 mm from frame {neg[0]} to {neg[-1]}" if neg else ""))
        return
    import warp as wp                                  # the Mac stops here (no Warp): the arguments and the scene were accepted
    from PIL import Image
    wp.init()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    n = len(d["t"])
    picks = set(np.linspace(0, n - 1, 6).round().astype(int).tolist())
    k_rel = None
    if a.release_still:                                # the last frame with the jaw closed (a retried pick reopens earlier)
        jaw = np.asarray(d["jaw"])
        k_rel = int(np.nonzero(jaw >= .9 * jaw.max())[0][-1])
        a.strip_only, picks = True, {k_rel}
    elif a.landed_still:                               # the last logged frame: the dish as it landed and settled
        k_rel = n - 1
        a.strip_only, picks = True, {k_rel}
    writer = None
    if not a.strip_only:
        import imageio
        writer = imageio.get_writer(str(a.out), fps=10, codec="libx264", quality=7, macro_block_size=None)
    strip = []
    for k in range(n):
        if a.strip_only and k not in picks:
            continue
        if not a.strip_only and k % a.every and k not in picks and k != n - 1:
            continue
        img, close = frame(*sample_parts(d, k, body_tris, grip, dish_local, a.kind, base, others_local=others_local,
                                         tints=tints), d["dish"][k][:3], W, H, a.label,
                           f"simulated t = {d['t'][k] - d['t'][0]:6.2f} s   jaw {d['jaw'][k]:.2f} rad   " + (
                               "inset: following the dish · basket out; the other rack's dishes as placed by their own chain · "
                               "Warp replay (collision hulls)" if a.load else "left: episode camera, right: following the dish   "
                               "Warp replay of the logged Isaac trial (collision hulls)"), ss=a.ss, inset=a.load is not None)
        if k_rel is not None:
            img.save(a.out)
            print(f"[RESULT] PASS {'landed' if a.landed_still else 'release'} still {a.out} (frame {k_rel} of {n}, jaw "
                  f"{d['jaw'][k_rel]:.2f})", flush=True)
            return
        if writer is not None:
            writer.append_data(np.asarray(img))
        if k in picks:
            strip.append(close)
    if writer is not None:
        writer.close()
    if a.strip or a.strip_only:
        Image.fromarray(np.concatenate(strip, axis=1)).resize((len(strip) * W // 2, H // 2)).save(a.out.with_suffix(".png"))
    print(f"[RESULT] PASS replay {a.out.relative_to(ROOT) if a.out.is_relative_to(ROOT) else a.out} ({n} frames)", flush=True)


if __name__ == "__main__":
    main()
