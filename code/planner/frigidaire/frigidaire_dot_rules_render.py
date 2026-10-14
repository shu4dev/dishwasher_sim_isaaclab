#!/usr/bin/env python3
"""Ray-traced stills of the 5 best and 5 worst loads under New+J, from the Isaac scene geometry (Warp, GPU).

The same HOTEC visual meshes and rack/basket meshes the Isaac scene loads, at the scored rack-local poses (racks in),
appliance shell hidden. Isaac's RTX renderer does not start on this box (segfault when Kit enables the Hydra engine,
NVIDIA driver 595.71 with Isaac Sim 4.5.0), so this stands in for frigidaire_hotec_load.py's stills.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_render.py --picks data/media/dot_rules/isaac/rand_picks.json
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
B = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(B)

import warp as wp  # noqa: E402

TINTS = {"plate": ["#08519c", "#2171b5", "#4292c6", "#6baed6", "#9ecae1", "#3182bd", "#1c5a99", "#5a9bd0"],
         "bowl": ["#a63603", "#d94801", "#f16913", "#fd8d3c", "#fdae6b", "#e6550d", "#c4520f", "#f9a05a"],
         "cup": ["#006d2c", "#238b45", "#41ab5d", "#74c476", "#a1d99b", "#31a354", "#1f7a3c", "#5cb86e"]}


@wp.kernel
def shade(mesh: wp.uint64, eye: wp.vec3, fwd: wp.vec3, right: wp.vec3, up: wp.vec3, tan_h: float, W: int, H: int,
          face_obj: wp.array(dtype=wp.int32), colors: wp.array(dtype=wp.vec3), light: wp.vec3, bg: wp.vec3,
          out: wp.array(dtype=wp.vec3)):
    i, j = wp.tid()
    u = (2.0 * (float(j) + 0.5) / float(W) - 1.0) * tan_h * float(W) / float(H)
    v = (1.0 - 2.0 * (float(i) + 0.5) / float(H)) * tan_h
    d = wp.normalize(fwd + u * right + v * up)
    q = wp.mesh_query_ray(mesh, eye, d, 50.0)
    c = bg
    if q.result:
        n = wp.normalize(q.normal)
        if wp.dot(n, d) > 0.0:
            n = -n
        p = eye + d * q.t + n * 0.0003
        lam = wp.max(wp.dot(n, light), 0.0)
        s = wp.mesh_query_ray(mesh, p, light, 50.0)
        if s.result:
            lam = lam * 0.3
        head = wp.max(-wp.dot(n, d), 0.0)
        c = colors[face_obj[q.face]] * (0.28 + 0.52 * lam + 0.30 * head)
    out[i * W + j] = c


def hexrgb(h):
    return np.array([int(h[k:k + 2], 16) / 255. for k in (1, 3, 5)])


def scene(entries, hide_upper=False):
    from dishsim_frigidaire import exposure as E
    from dishsim_frigidaire import dot_rules as DR
    from dishsim_frigidaire.asset import BODY_POSITIONS
    parts, cols = [], []
    for name, col in (("LowerRack", (.80, .80, .82)), ("UpperRack", (.80, .80, .82))):
        if hide_upper and name == "UpperRack":
            continue
        parts.append(E.component_triangles(name) + np.asarray(BODY_POSITIONS[name]))
        cols.append(col)
    parts.append(E.posed(E.component_triangles("SilverwareBasket"), np.asarray(BODY_POSITIONS["SilverwareBasket"], float),
                         np.asarray(E.IDENTITY, float)))
    cols.append((.62, .62, .66))
    for e, o in zip(entries, DR.world_objects(entries)):
        if hide_upper and e["rack"] == "UpperRack":
            continue
        n = int(e["id"].rsplit("_", 1)[1])
        tri = np.concatenate([E.posed(t, o["position_m"], o["quaternion_xyzw"]) for t in E.dish_visuals(o["kind"])])
        parts.append(tri)
        cols.append(hexrgb(TINTS[e["kind"].replace("hotec_", "")][(n - 1) % 8]))
    face_obj = np.concatenate([np.full(len(p), k, np.int32) for k, p in enumerate(parts)])
    return np.concatenate(parts).astype(np.float32), face_obj, np.asarray(cols, np.float32)


def render(tri, face_obj, cols, eye, target, W=960, H=720, fov_deg=34., ss=2):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    mesh = wp.Mesh(points=wp.array(tri.reshape(-1, 3), dtype=wp.vec3),
                   indices=wp.array(np.arange(tri.size // 3, dtype=np.int32), dtype=wp.int32))
    Ws, Hs = W * ss, H * ss
    out = wp.zeros(Ws * Hs, dtype=wp.vec3)
    light = np.array([-.35, -.55, 1.]) / np.linalg.norm([-.35, -.55, 1.])
    wp.launch(shade, dim=(Hs, Ws), inputs=[mesh.id, wp.vec3(*eye), wp.vec3(*fwd), wp.vec3(*right), wp.vec3(*up),
                                          float(np.tan(np.radians(fov_deg) / 2)), Ws, Hs, wp.array(face_obj, dtype=wp.int32),
                                          wp.array(cols, dtype=wp.vec3), wp.vec3(*light), wp.vec3(.965, .97, .975), out])
    img = out.numpy().reshape(Hs, Ws, 3).reshape(H, ss, W, ss, 3).mean(axis=(1, 3))
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def font(size, bold=False):
    import matplotlib
    return ImageFont.truetype(str(Path(matplotlib.get_data_path()) / "fonts/ttf" /
                                  ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")), size)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--picks", required=True)
    ap.add_argument("--out", default="data/media/dot_rules/renders")
    a = ap.parse_args()
    wp.init()
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire.asset import BODY_POSITIONS
    picks = json.loads(Path(a.picks).read_text())
    base = Path(a.picks).parent
    out = ROOT / a.out
    out.mkdir(parents=True, exist_ok=True)
    yc = .5 * (BODY_POSITIONS["LowerRack"][1] + BODY_POSITIONS["UpperRack"][1])
    for p in picks:
        lay = json.loads((base / p["tag"] / "layout.json").read_text())
        entries = [{"id": o["object_id"], "kind": o["kind"], "rack": o["rack"],
                    "position": o["rack_local_pose"]["position_m"], "quaternion_xyzw": o["rack_local_pose"]["quaternion_xyzw"]}
                   for o in lay["objects"]]
        views = []
        for hide, tz, k in ((False, .45, 1.), (True, .30, .78)):
            tri, fo, cols = scene(entries, hide_upper=hide)
            target = np.array([0., yc, tz])
            views.append(render(tri, fo, cols, target + k * np.array([.62, -1.05, .95]), target))
        img = Image.fromarray(np.concatenate(views, axis=1))
        canvas = Image.new("RGB", (img.width, img.height + 64), (18, 28, 40))
        canvas.paste(img, (0, 64))
        dr = ImageDraw.Draw(canvas)
        side = "best" if p["side"] == "top" else "worst"
        lab = p.get("label") or ("Completely random" if p["tag"].startswith("rand") else "Restricted sampler")
        dr.text((18, 8), f"{lab} · {side} {p['rank']} of {p['n']} by New+J · {p['load_id']} · S(New+J) = {p['S_NJ']:.3f}",
                fill=(246, 249, 252), font=font(24, True))
        dr.text((18, 38), "left: both racks · right: upper rack hidden · racks in, appliance shell hidden · "
                          "Isaac scene meshes at the scored poses, ray-traced with Warp", fill=(190, 207, 222), font=font(17))
        path = out / f"{p['tag']}.jpg"
        canvas.save(path, quality=88)
        print(f"[render] wrote {path.relative_to(ROOT)}", flush=True)
    print(f"[RESULT] PASS {len(picks)} renders", flush=True)


if __name__ == "__main__":
    main()
