#!/usr/bin/env python3
"""Warp replay of a logged robot trial (the box's RTX renderer crashes with cameras): the arm and gripper as the
convex hulls of their USD collision meshes at the logged body poses, the dish (HOTEC visual mesh), the racks, the
counter and the pedestal; two views (the episode camera, a close view following the dish), 10 fps mp4.

    code/util/run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py \\
        --traj data/results/robot/limits_v1/<run>/trials/traj_bowl_gate_t00.npz --kind bowl --label "..." --out x.mp4

``--strip`` writes a six-frame PNG strip instead of (or with) the mp4. Reads robot_meshes.json beside the trajectory.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
import frigidaire_dot_rules_render as RD  # noqa: E402  (Warp kernel, scene helpers, B)

EPISODE_EYE, EPISODE_TARGET, EPISODE_FOV = (-1.9, -2.7, 1.95), (.05, -.25, .62), 46.4   # ROBOT_CAMERA (42 mm / 36 mm)
CLOSE_OFFSET = np.array([-.55, -.75, .45])
COLORS = {"rack": (.80, .80, .82), "counter": (.72, .70, .66), "pedestal": (.35, .36, .40), "arm": (.55, .62, .72),
          "gripper": (.25, .25, .28), "bowl": RD.hexrgb("#d94801"), "cup": RD.hexrgb("#238b45"), "plate": RD.hexrgb("#2171b5")}


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


def pose_still(kind, site_name, target, yaw, out, label):
    """Kit-free still of the arm at the first IK solution of a packer-pose pre-place (the colliding pose): the arm
    model's hulls at the analytic link poses, the carried dish, the racks of the cell, the counter and the pedestal."""
    import warp as wp
    from PIL import Image, ImageDraw
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_reach as RC
    from dishsim_frigidaire import exposure as E
    from dishsim_frigidaire.robot import grasp as G
    from dishsim_frigidaire.robot.rig import T_of
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
    ap.add_argument("--kind", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--strip", action="store_true", help="also write a six-frame strip (<out>.png)")
    ap.add_argument("--strip-only", action="store_true")
    ap.add_argument("--wh", default="640x480")
    a = ap.parse_args()
    if a.still:
        kind, site, target, yaw = a.still.split(",")
        return pose_still(kind, site, target, int(yaw), a.out, a.label)
    import warp as wp
    from PIL import Image, ImageDraw
    from dishsim_frigidaire import exposure as E
    B = RD.B
    wp.init()
    B.HX().register_hotec(B.ASSETS)
    W, H = (int(v) for v in a.wh.split("x"))
    d = np.load(a.traj)
    meshes = json.loads((a.traj.parent / "robot_meshes.json").read_text())
    names = [p.rsplit("/", 1)[-1] for p in meshes["body_paths"]]
    body_tris = {int(i): np.concatenate([hull_tris(p) for p in lst]) for i, lst in meshes["hulls"].items()}
    grip = {i for i, p in enumerate(meshes["body_paths"]) if "/Gripper/" in p}
    mount = json.loads((ROOT / "data/results/robot/mount/easy_s0_any.json").read_text())["best"]["base"]
    static, scol = [], []
    for name, key in (("LowerRack", "lower"), ("UpperRack", "upper")):
        f = d[key][0]
        static.append(E.posed(E.component_triangles(name), f[:3], f[3:7]))
        scol.append(COLORS["rack"])
    static.append(box_tris(B.COUNTER["center_m"], B.COUNTER["size_m"]))
    scol.append(COLORS["counter"])
    static.append(box_tris((mount[0], mount[1], mount[2] / 2), (.16, .16, mount[2])))
    scol.append(COLORS["pedestal"])
    dish_local = np.concatenate(E.dish_visuals(f"hotec_{a.kind}"))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    n = len(d["t"])
    picks = set(np.linspace(0, n - 1, 6).round().astype(int).tolist())
    writer = None
    if not a.strip_only:
        import imageio
        writer = imageio.get_writer(str(a.out), fps=10, codec="libx264", quality=7, macro_block_size=None)
    strip = []
    font, small = RD.font(18, True), RD.font(14)
    for k in range(n):
        if a.strip_only and k not in picks:
            continue
        parts, cols = list(static), list(scol)
        for i, tri in body_tris.items():
            if len(tri) == 0:
                continue
            R = quat_wxyz_R(d["body_quat_wxyz"][k, i])
            parts.append(tri @ R.T + d["body_pos"][k, i])
            cols.append(COLORS["gripper"] if i in grip else COLORS["arm"])
        dish = d["dish"][k]
        parts.append(E.posed(dish_local, dish[:3], dish[3:7]))
        cols.append(COLORS[a.kind])
        face_obj = np.concatenate([np.full(len(p), j, np.int32) for j, p in enumerate(parts)])
        tri = np.concatenate(parts).astype(np.float32)
        cols_a = np.asarray(cols, np.float32)
        wide = RD.render(tri, face_obj, cols_a, EPISODE_EYE, EPISODE_TARGET, W=W, H=H, fov_deg=EPISODE_FOV, ss=1)
        tgt = dish[:3]
        close = RD.render(tri, face_obj, cols_a, tgt + CLOSE_OFFSET, tgt, W=W, H=H, fov_deg=40., ss=1)
        img = Image.new("RGB", (2 * W, H + 56), (18, 28, 40))             # header bar above the two views
        img.paste(Image.fromarray(np.concatenate([wide, close], axis=1)), (0, 56))
        dr = ImageDraw.Draw(img)
        dr.text((10, 6), a.label[:150], fill=(246, 249, 252), font=font)
        dr.text((10, 32), f"simulated t = {d['t'][k] - d['t'][0]:6.2f} s   jaw {d['jaw'][k]:.2f} rad   left: episode camera, "
                          f"right: following the dish   Warp replay of the logged Isaac trial (collision hulls)",
                fill=(190, 207, 222), font=small)
        frame = np.asarray(img)
        if writer is not None:
            writer.append_data(frame)
        if k in picks:
            strip.append(close)
    if writer is not None:
        writer.close()
    if a.strip or a.strip_only:
        Image.fromarray(np.concatenate(strip, axis=1)).resize((len(strip) * W // 2, H // 2)).save(a.out.with_suffix(".png"))
    print(f"[RESULT] PASS replay {a.out.relative_to(ROOT) if a.out.is_relative_to(ROOT) else a.out} ({n} frames)", flush=True)


if __name__ == "__main__":
    main()
