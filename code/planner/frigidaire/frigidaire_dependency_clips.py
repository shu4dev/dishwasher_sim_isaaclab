#!/usr/bin/env python3
"""Clips of an --edges swaps dependency record: the witness build and one clip per edge (Kit-free, Warp on the GPU).

Input: the record (frigidaire_dependency_kit.py --edges swaps) and the tick poses its Kit run kept, one frame per 4 ticks
at 120 Hz, in <record dir>/_kit/<load_id>/traj/. Each frame is ray-traced with Warp like the Dot Score Rules stills
(frigidaire_dot_rules_render.render: the same HOTEC and rack meshes, racks in, appliance shell hidden, the both-racks
view): the world poses (racks extended) go rack-local in the record's rack frames, then into the racks-in model. The
dropped dish and the dishes it affects are tinted, every other dish grey, a parked dish is hidden. 640 x 480 (a 48 px
caption bar), played at 13 fps: a 210-tick drop lasts about 4 s. An edge's clip is the swap that failed: step 1 (b goes
in before a) and, for a step-2 edge, step 2 (a after b); a drop with no clear release is a held still with the dish
shown at its goal pose in red. --release insertion records (2026-10-15): each trajectory starts with the descent (one
frame per 5 mm step from the start pose, recorded by the Kit run), then the fall; a blocked descent plays until the
blocked step with the dish in red, held. The re-posing sampler's records (2026-10-15 b) add load.jpg (the built goal,
every dish tinted) and, per conditional edge, the confirmation drops of each confirmed alternative (cond_<a>__<b>__<c>):
b goes in with c present and a absent. mp4 (H.264 through imageio, as dishsim.media.VideoWriter), else GIF.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_clips.py --record data/results/dot_rules/dependency/sequential/r02_00048.json --out data/media/dot_rules/dependency_sequential/r02_00048

Writes <out>/witness.mp4, <out>/edge_<a>__<b>.mp4, <out>/cond_<a>__<b>__<c>.mp4, <out>/load.jpg, a poster JPEG per
clip and <out>/clips.json. On an --edges physical record, --views front,side renders only the access stills, level
from the door side and from +x along the plate row (access_<a>__<b>_front.png, _side.png; Stage.axis_view):

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_clips.py --record data/results/dot_rules/dependency/samples_v5/r05_00027.json --out data/media/dot_rules/dependency_samples_v5/r05_00027 --views front,side
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
import frigidaire_dot_rules_render as R  # noqa: E402  (Warp; R.B is frigidaire_bench)

W, H, HEAD, FPS, HOLD = 640, 432, 48, 13, 6      # render size, caption bar, playback, frames held at a segment's end
GREY, RED = (.66, .67, .70), (.86, .16, .16)
PARK_X = 2.


def short(i):
    kind, n = i.rsplit("_", 1)
    return f"{kind[0]}{n}"


class Stage:
    """The static racks and basket (racks in) and each kind's visual triangles, built once; frame() poses the dishes."""

    def __init__(self, record):
        import warp as wp
        wp.init()
        R.B.HX().register_hotec(R.B.ASSETS)
        from dishsim_frigidaire import exposure as E
        from dishsim_frigidaire.asset import BODY_POSITIONS
        from dishsim_frigidaire.random_poses import compose_pose, relative_pose
        self.E, self.body, self.compose, self.relative = E, BODY_POSITIONS, compose_pose, relative_pose
        parts = [E.component_triangles(n) + np.asarray(BODY_POSITIONS[n]) for n in ("LowerRack", "UpperRack")]
        parts.append(E.posed(E.component_triangles("SilverwareBasket"),
                             np.asarray(BODY_POSITIONS["SilverwareBasket"], float), np.asarray(E.IDENTITY, float)))
        self.parts = parts
        self.static = np.concatenate(parts).astype(np.float32)
        self.static_obj = np.concatenate([np.full(len(p), k, np.int32) for k, p in enumerate(parts)])
        self.static_cols = [(.80, .80, .82), (.80, .80, .82), (.62, .62, .66)]
        ents = record.get("entries_chosen") or record["entries"]     # a re-posed dish may have changed rack
        self.kind = {e["id"]: e["kind"] for e in ents}
        self.rack = {e["id"]: e["rack"] for e in ents}
        self.frames = record["frames"]
        self.visual = {k: np.concatenate(E.dish_visuals(f"hotec_{k}")) for k in set(self.kind.values())}
        yc = .5 * (BODY_POSITIONS["LowerRack"][1] + BODY_POSITIONS["UpperRack"][1])
        self.target = np.array([0., yc, .45])
        self.eye = self.target + np.array([.62, -1.05, .95])

    def tint(self, i):
        n = int(i.rsplit("_", 1)[1])
        return tuple(R.hexrgb(R.TINTS[self.kind[i]][(n - 1) % 8]))

    def frame(self, poses, colors):
        """One RGB image of the dishes at the world ``poses`` {id: (position, quaternion_xyzw)} (parked ones hidden);
        ``colors`` {id: rgb} overrides the grey."""
        tris, objs, cols = [self.static], [self.static_obj], list(self.static_cols)
        for i, (p, q) in poses.items():
            if p[0] > PARK_X:
                continue
            f = self.frames[self.rack[i]]
            lp, lq = self.relative(p, q, f["position_m"], f["quaternion_xyzw"])
            wp_, wq = self.compose(self.body[self.rack[i]], self.E.IDENTITY, lp, lq)
            tri = self.E.posed(self.visual[self.kind[i]], wp_, wq).astype(np.float32)
            tris.append(tri)
            objs.append(np.full(len(tri), len(cols), np.int32))
            cols.append(colors.get(i, GREY))
        return R.render(np.concatenate(tris), np.concatenate(objs), np.asarray(cols, np.float32), self.eye,
                        self.target, W=W, H=H)

    def views(self, poses, colors):
        """The load from four angles, 2 x 2 (plans/2026-10-16-labelled-views-overlap-rule.md): the oblique view of
        frame(), a top view of the lower rack (with the basket) and of the upper rack (each hides the other rack), a front
        view; every dish labelled with its short id at its visual centroid, white with a dark outline."""
        names = ("LowerRack", "UpperRack", "SilverwareBasket")
        tan_h, out = float(np.tan(np.radians(34.) / 2)), []    # R.render's default field of view
        for title, racks, how in (("oblique", names, "oblique"), ("top: lower rack", ("LowerRack", "SilverwareBasket"), "top"),
                                  ("top: upper rack", ("UpperRack",), "top"), ("front", names, "front")):
            tris, objs, cols, cent = [], [], [], {}
            for k, n in enumerate(names):
                if n in racks:
                    tris.append(self.parts[k].astype(np.float32))
                    objs.append(np.full(len(self.parts[k]), len(cols), np.int32))
                    cols.append(self.static_cols[k])
            for i, (p, q) in poses.items():
                if p[0] > PARK_X or self.rack[i] not in racks:
                    continue
                f = self.frames[self.rack[i]]
                lp, lq = self.relative(p, q, f["position_m"], f["quaternion_xyzw"])
                wp_, wq = self.compose(self.body[self.rack[i]], self.E.IDENTITY, lp, lq)
                tri = self.E.posed(self.visual[self.kind[i]], wp_, wq).astype(np.float32)
                cent[i] = tri.reshape(-1, 3).mean(axis=0)
                tris.append(tri)
                objs.append(np.full(len(tri), len(cols), np.int32))
                cols.append(colors.get(i, GREY))
            pts = np.concatenate(tris).reshape(-1, 3)
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            c, half = .5 * (lo + hi), .5 * (hi - lo)
            if how == "oblique":
                eye, target = self.eye, self.target
            elif how == "top":                                  # straight down (a 3 cm tilt keeps R.render's up defined)
                d = 1.08 * max(half[1] / tan_h, half[0] / (tan_h * W / H))
                eye, target = np.array([c[0], c[1] - .03, hi[2] + d]), np.array([c[0], c[1], lo[2]])
            else:                                               # from the front (-y, the door side), level
                d = 1.08 * max(half[2] / tan_h, half[0] / (tan_h * W / H))
                eye, target = np.array([c[0], lo[1] - d, c[2]]), np.array([c[0], hi[1], c[2]])
            img = Image.fromarray(R.render(np.concatenate(tris), np.concatenate(objs), np.asarray(cols, np.float32),
                                           eye, target, W=W, H=H))
            fwd = (target - eye) / np.linalg.norm(target - eye)      # R.render's camera, to place the labels
            right = np.cross(fwd, [0, 0, 1.])
            right /= np.linalg.norm(right)
            up = np.cross(right, fwd)
            draw, boxes, lab = ImageDraw.Draw(img), [], R.font(13, True)
            for i, x in cent.items():
                r = x - eye
                z = float(r @ fwd)
                u, v = float(r @ right) / z, float(r @ up) / z
                px, py = (u / (tan_h * W / H) + 1) * W / 2, (1 - v / tan_h) * H / 2

                def box(dy):
                    return draw.textbbox((px, py + dy), short(i), font=lab, stroke_width=2, anchor="mm")

                def free(dy):                                   # no label drawn so far overlaps it
                    b = box(dy)
                    return not any(b[0] < c[2] and c[0] < b[2] and b[1] < c[3] and c[1] < b[3] for c in boxes)
                dy = next((d for d in (0, 14, -14, 28, -28, 42, -42) if free(d)), 0)   # nudged off a neighbour's label
                boxes.append(box(dy))
                draw.text((px, py + dy), short(i), fill=(255, 255, 255), font=lab, stroke_width=2,
                          stroke_fill=(16, 20, 28), anchor="mm")
            draw.text((10, 8), title, fill=(255, 255, 255), font=R.font(15, True), stroke_width=2,
                      stroke_fill=(16, 20, 28))
            out.append(np.asarray(img))
        return np.concatenate([np.concatenate(out[:2], axis=1), np.concatenate(out[2:], axis=1)], axis=0)

    def axis_view(self, poses, colors, how, pair):
        """The rack of ``pair`` (a, b) seen nearly level along a rack axis, for the access stills (coordinator,
        2026-10-16): "front" from the door side (-y) looking along +y, "side" from +x looking along -x (along the plate
        row); the eye 6 cm above the rack floor (the lowest dish bottom), aimed at the middle of what is shown. The other
        rack is hidden (the basket goes with the lower rack), and so is any other dish (or the basket) nearer the camera
        than the farther of the pair whose screen box overlaps one of theirs. Returns (image, hidden ids)."""
        rack, names = self.rack[pair[0]], ("LowerRack", "UpperRack", "SilverwareBasket")
        keep = ("LowerRack", "SilverwareBasket") if rack == "LowerRack" else ("UpperRack",)
        items = [(n, self.parts[k].astype(np.float32), self.static_cols[k]) for k, n in enumerate(names) if n in keep]
        for i, (p, q) in poses.items():
            if p[0] > PARK_X or self.rack[i] != rack:
                continue
            f = self.frames[rack]
            lp, lq = self.relative(p, q, f["position_m"], f["quaternion_xyzw"])
            wp_, wq = self.compose(self.body[rack], self.E.IDENTITY, lp, lq)
            items.append((i, self.E.posed(self.visual[self.kind[i]], wp_, wq).astype(np.float32), colors.get(i, GREY)))
        pts = np.concatenate([t for _, t, _ in items]).reshape(-1, 3)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        c, half = .5 * (lo + hi), .5 * (hi - lo)
        floor = min(float(t[..., 2].min()) for i, t, _ in items if i in self.kind)
        tan_h = float(np.tan(np.radians(34.) / 2))             # R.render's default field of view
        depth, lat = (1, 0) if how == "front" else (0, 1)
        d = 1.15 * max(half[2] / tan_h, half[lat] / (tan_h * W / H))
        eye, target = c.copy(), c.copy()
        eye[2] = floor + .06
        eye[depth] = lo[1] - d if how == "front" else hi[0] + d
        fwd = (target - eye) / np.linalg.norm(target - eye)
        right = np.cross(fwd, [0, 0, 1.])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)

        def screen(t):                                          # a part's centroid depth and its screen box
            r = t.reshape(-1, 3) - eye
            z = r @ fwd
            u, v = (r @ right) / z, (r @ up) / z
            return float(r.mean(axis=0) @ fwd), (u.min(), v.min(), u.max(), v.max())
        box = {i: screen(t) for i, t, _ in items if i != rack}
        pair = [x for x in pair if x in box]
        far = max(box[x][0] for x in pair)
        hidden = [i for i, (z, b) in box.items() if i not in pair and z < far - .01 and any(
            b[0] < box[x][1][2] and box[x][1][0] < b[2] and b[1] < box[x][1][3] and box[x][1][1] < b[3] for x in pair)]
        shown = [(i, t, col) for i, t, col in items if i not in hidden]
        img = R.render(np.concatenate([t for _, t, _ in shown]),
                       np.concatenate([np.full(len(t), k, np.int32) for k, (_, t, _) in enumerate(shown)]),
                       np.asarray([col for _, _, col in shown], np.float32), eye, target, W=W, H=H)
        return img, hidden


def captioned(img, title, detail):
    canvas = Image.new("RGB", (img.shape[1], img.shape[0] + HEAD), (18, 28, 40))
    canvas.paste(Image.fromarray(img), (0, HEAD))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 5), title, fill=(246, 249, 252), font=R.font(17, True))
    draw.text((12, 27), detail, fill=(190, 207, 222), font=R.font(13))
    return np.asarray(canvas)


def load_traj(folder, name):
    data = json.loads((folder / name).read_text())
    ids, out = data["ids"], []
    for row in data["frames"]:
        a = np.asarray(row, dtype=float).reshape(len(ids), 7)
        out.append({i: (a[n, :3], a[n, 3:]) for n, i in enumerate(ids)})
    return data, out


class Clip:
    """Frames to an mp4 (imageio, libx264, as dishsim.media.VideoWriter) or, without it, a GIF."""

    def __init__(self, path):
        self.path, self.frames, self.writer = Path(path), [], None
        try:
            import imageio
            self.writer = imageio.get_writer(str(self.path), fps=FPS, codec="libx264", quality=7,
                                             macro_block_size=None, ffmpeg_params=["-movflags", "+faststart"])
        except Exception as exc:                                    # noqa: BLE001
            print(f"[clips] no mp4 writer ({exc!r}): GIF instead", flush=True)
            self.path = self.path.with_suffix(".gif")
        self.count, self.last = 0, None

    def add(self, frame, repeat=1):
        for _ in range(repeat):
            if self.writer is not None:
                self.writer.append_data(frame)
            else:
                self.frames.append(Image.fromarray(frame))
            self.count += 1
        self.last = frame

    def close(self, poster):
        if self.writer is not None:
            self.writer.close()
        else:
            self.frames[0].save(self.path, save_all=True, append_images=self.frames[1:], duration=int(1000 / FPS),
                                loop=0)
        Image.fromarray(self.last).save(poster, quality=88)
        return {"file": self.path.name, "poster": Path(poster).name, "frames": self.count,
                "seconds": round(self.count / FPS, 1), "bytes": self.path.stat().st_size}


def segment(stage, clip, traj, colors, title, detail, ghost=None):
    """Play one trajectory (every frame) into ``clip``, holding its last frame; ``ghost`` = (id, goal pose) drawn in red
    on a still (no clear release)."""
    for poses in traj:
        if ghost is not None:
            gid, g = ghost
            poses = {**poses, gid: (np.asarray(g["position_m"], float), np.asarray(g["quaternion_xyzw"], float))}
        clip.add(captioned(stage.frame(poses, colors), title, detail),
                 repeat=4 * HOLD if ghost is not None or len(traj) == 1 else 1)
    clip.add(clip.last, repeat=HOLD)


def load_image(record, stage, out):
    """load.png: the rebuild's final poses (or the built goal when it did not certify), every dish tinted, four labelled
    views (Stage.views)."""
    cert, gate, lid = record.get("certify") or {}, record.get("gate") or {}, record["load_id"]
    final = cert.get("poses_world") or record["targets_world"]
    goal = {i: (np.asarray(p["position_m"], float), np.asarray(p["quaternion_xyzw"], float)) for i, p in final.items()}
    first = "lower" if list(gate.get("order") or [])[:1] == ["LowerRack"] else "upper"
    gtext = (f"joint gate {gate.get('outcome')} ({first} rack first), peaks {gate.get('settle_peak_mm')} / "
             f"{gate.get('cycle_peak_mm')} mm (limit {gate.get('peak_limit_mm'):g})" if gate else "no gate")
    Image.fromarray(captioned(stage.views(goal, {i: stage.tint(i) for i in stage.kind}),
                              f"Load {lid}: " + ("the certified rebuild's final poses" if cert.get("certified")
                                                 else "the built goal") + ", four views, every dish labelled",
                              gtext)).save(out / "load.png")
    return "load.png"


def access_stills(record, stage, out, views=("oblique",)):
    """access_<a>__<b>.png per access hit: the finished load (the certified rebuild's final poses, else the built goal),
    b tinted, a in red at the first step of its insertion path that b's final pose covers, in the oblique view of
    Stage.frame; --views front,side adds access_<a>__<b>_front.png and _side.png (Stage.axis_view)."""
    final = (record.get("certify") or {}).get("poses_world") or record["targets_world"]
    goal = {i: (np.asarray(p["position_m"], float), np.asarray(p["quaternion_xyzw"], float)) for i, p in final.items()}
    done = {}
    for h in (record.get("access") or {}).get("hits") or []:
        a, b = h["a"], h["b"]
        poses = {**goal, a: (np.asarray(h["pose"]["position_m"], float), np.asarray(h["pose"]["quaternion_xyzw"], float))}
        colors = {a: RED, b: stage.tint(b)}
        for how in views:
            if how == "oblique":
                name = f"access_{a}__{b}.png"
                img = captioned(stage.frame(poses, colors), f"{short(a)} before {short(b)}: {short(b)} covers its "
                                f"insertion path", f"{short(a)} at step {h['step']} of {h['steps']}, {h['lift_mm']} mm "
                                f"above its move pose; {short(b)} at its final pose")
            else:
                name = f"access_{a}__{b}_{how}.png"
                view, hidden = stage.axis_view(poses, colors, how, (a, b))
                img = captioned(view, f"{short(a)} before {short(b)}, {how} view: {short(b)} covers its insertion path",
                                f"{short(a)} at step {h['step']} of {h['steps']}, {h['lift_mm']} mm up; {short(b)} at "
                                f"its final pose; looking along {'+y' if how == 'front' else '-x'}"
                                + (f"; {len(hidden)} in front hidden" if hidden else ""))
                print(f"[clips] {name}: hidden {hidden}", flush=True)
            Image.fromarray(img).save(out / name)
            done[f"{a}>{b}" + ("" if how == "oblique" else f"|{how}")] = name
    return done


def physical(record, stage, folder, out):
    """--edges physical records (plans/2026-10-16-physical-cause-graph.md): witness.mp4 (the fresh-process rebuild that
    certified the witness, poster witness.png), load.png (the rebuild's final poses, or the built goal when it did not
    certify, every dish tinted) for the page; on disk only (coordinator, 2026-10-16): conf_<b>.mp4 (every confirmation drop
    of b, one segment each) and access_<a>__<b>.png (the finished load, b tinted, a in red at the first step of its
    insertion path that b's final pose covers)."""
    t0, lid = time.monotonic(), record["load_id"]
    tinted = {i: stage.tint(i) for i in stage.kind}
    cert, gate = record.get("certify") or {}, record.get("gate") or {}
    order = record["witness"]["order"]
    manifest = {"load_id": lid, "fps": FPS, "size": [W, H + HEAD], "confirmations": {}, "access": {}}
    data, traj = load_traj(folder, cert["traj"])
    clip = Clip(out / "witness.mp4")
    failed = None if cert.get("certified") else cert.get("dish")
    for poses in traj:                                          # the newest dish in the racks is the one dropping
        placed = [i for i in order if poses[i][0][0] <= PARK_X]
        now = placed[-1] if placed else None
        colors = {now: RED if now == failed else tinted[now]} if now else {}
        clip.add(captioned(stage.frame(poses, colors), f"Witness rebuilt in a fresh Kit process: {lid}, {len(placed)} of "
                           f"{len(order)} placed", (f"now {short(now)}" + ("  (did not land)" if now == failed else "")
                                                    if now else "empty racks; the witness order, the insertion move")))
    clip.add(clip.last, repeat=3 * HOLD)
    manifest["witness"] = clip.close(out / "witness.png")
    print(f"[clips] witness: {manifest['witness']} ({time.monotonic() - t0:.0f} s)", flush=True)
    manifest["load"] = load_image(record, stage, out)
    for c in record.get("confirmations") or []:
        b = c["dish"]
        drops = [("without " + "+".join(map(short, c["supports"])), c["without"])]
        drops += [(f"with {short(x)} alone", d) for x, d in c["alone"].items()]
        drops += [("with all of " + "+".join(map(short, c["supports"])), c["all"])] if c.get("all") else []
        drops += [(f"without {short(x)} only", d) for x, d in (c.get("loo") or {}).items()
                  if d["traj"] not in {y["traj"] for _, y in drops}]
        clip = Clip(out / f"conf_{b}.mp4")
        head = f"{short(b)} rests on {', '.join(map(short, c['supports']))}  ·  {c['outcome']}"
        for what, d in drops:
            _, tr = load_traj(folder, d["traj"])
            colors = {i: tinted[i] for i in c["supports"]}
            colors[b] = tinted[b] if d["landed"] else RED
            res = ("blocked by " + ", ".join(map(short, d["blocked_by"])) if d.get("blocked_by") is not None else
                   f"{'lands' if d['landed'] else 'does not land'}: {d['lateral_mm']} mm / {d['dz_mm']} mm / "
                   f"{d['tilt_deg']}°, contact {d['contact_mm']} mm")
            segment(stage, clip, tr, colors, head, f"{short(b)} {what}: {res}")
        manifest["confirmations"][b] = clip.close(out / f"conf_{b}.jpg")
    manifest["access"] = access_stills(record, stage, out)
    (out / "clips.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"[RESULT] PASS {lid} clips: witness, load, {len(manifest['confirmations'])} confirmation clips, "
          f"{len(manifest['access'])} access stills ({time.monotonic() - t0:.0f} s)", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--record", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", default=None, help="comma list: witness and/or edge keys a>b (default: all)")
    ap.add_argument("--views", default=None, help="--edges physical records: render only the access stills, in these "
                    "views (comma list of oblique, front, side); front and side go to access_<a>__<b>_<view>.png")
    a = ap.parse_args()
    record = json.loads(a.record.read_text())
    folder = a.record.parent / "_kit" / record["load_id"] / "traj"
    a.out.mkdir(parents=True, exist_ok=True)
    only = set(a.only.split(",")) if a.only else None
    stage = Stage(record)
    if record.get("procedure") == "physical" and a.views:      # the access stills alone; clips.json untouched
        t0, views = time.monotonic(), a.views.split(",")
        done = access_stills(record, stage, a.out, views)
        print(f"[RESULT] PASS {record['load_id']} access stills: {len(done)} ({'+'.join(views)}, "
              f"{len((record.get('access') or {}).get('hits') or [])} hits) ({time.monotonic() - t0:.0f} s)", flush=True)
        return
    if record.get("procedure") == "physical":
        return physical(record, stage, folder, a.out)
    manifest, t0 = {"load_id": record["load_id"], "fps": FPS, "size": [W, H + HEAD], "edges": {}}, time.monotonic()
    tinted = {i: stage.tint(i) for i in stage.kind}
    wit = record["witness"]
    rinfo = record.get("release") or {"mode": "clear"}
    failed = record.get("status") != "ok"           # a resettle_failed record: its witness is the sampler's last build
    move = ("empty racks; dishes go in one at a time, each released at its lowest clear hover" if rinfo["mode"] != "insertion"
            else "empty racks; each dish is lowered from above the rim (plates upright) and released 20 mm above where "
                 "the rack stops it")
    if only is None or "witness" in only:
        data, traj = load_traj(folder, wit["traj"])
        clip, n_placed = Clip(a.out / "witness.mp4"), 0
        for poses in traj:                                      # the newest dish in the racks is the one dropping
            placed = [i for i in wit["order"] if poses[i][0][0] <= PARK_X]
            n_placed = len(placed)
            now = placed[-1] if placed else None
            colors = {now: RED if failed and now == record.get("failed_dish") else tinted[now]} if now else {}
            title = (f"{'Last sampler build' if failed else 'Witness build'} of {record['load_id']}: {n_placed} of "
                     f"{len(record['entries'])} placed"
                     + ("  ·  insertion move" if rinfo["mode"] == "insertion" else ""))
            detail = (f"now {short(now)} (the sampler's order)" if now else move)
            if failed and now == record.get("failed_dish"):
                s = wit["steps"][-1]
                detail = (f"{short(now)}: did not go in ({s.get('blocked') or 'no landing'}"
                          f"{', ' + ', '.join(s.get('blockers') or []) if s.get('blockers') else ''}); the sampler stopped")
            clip.add(captioned(stage.frame(poses, colors), title, detail))
        clip.add(clip.last, repeat=3 * HOLD)
        manifest["witness"] = clip.close(a.out / "witness.jpg")
        print(f"[clips] witness: {manifest['witness']} ({time.monotonic() - t0:.0f} s)", flush=True)
    if (only is None or "load" in only) and record.get("status") == "ok":     # the built goal, every dish tinted
        goal = {i: (np.asarray(p["position_m"], float), np.asarray(p["quaternion_xyzw"], float))
                for i, p in record["targets_world"].items()}
        n_re = len((record.get("reposing") or {}).get("reposed") or {})
        img = captioned(stage.frame(goal, tinted), f"Load {record['load_id']}: the built goal",
                        f"{len(goal)} dishes, both racks, as the witness left them ({n_re} re-posed by the sampler)")
        Image.fromarray(img).save(a.out / "load.jpg", quality=90)
        manifest["load"] = "load.jpg"
    swaps = {s["swap"]: s for s in record.get("swaps") or []}
    for e in record.get("edges") or []:
        key = f"{e['a']}>{e['b']}"
        if (only is not None and key not in only) or e.get("swap") is None:   # a learned edge has no swap to replay
            continue
        s = swaps[e["swap"]]
        name = f"edge_{e['a']}__{e['b']}"
        clip = Clip(a.out / f"{name}.mp4")
        head = f"{short(e['a'])} before {short(e['b'])}  ·  {e['reason']}" + ("  ·  support" if e["support"] else "")
        for n, tname in enumerate(e["traj"], 1):
            data, traj = load_traj(folder, tname)
            dish, last = data["dish"], n == len(e["traj"])
            colors = {dish: tinted[dish]}
            if last:                                            # the failing drop: the pair and what it moved
                colors.update({i: tinted[i] for i in [e["a"], e["b"], *e["affected"]]})
            if dish == e["b"] and not last:
                detail = f"step 1: {short(e['b'])} goes in first, into the {s['placed_before']} dishes before {short(e['a'])}"
            elif last:
                detail = f"step {e['step']}: {e['detail']}"
            else:
                detail = f"step {n}"
            blocked = bool(data.get("no_clear_release"))
            ghost = (dish, record["targets_world"][dish]) if blocked and not data.get("path_blocked") else None
            if blocked:
                colors[dish] = RED
            segment(stage, clip, traj, colors, head, detail, ghost)
            if data.get("path_blocked"):                            # the blocked step, held
                clip.add(clip.last, repeat=3 * HOLD)
        manifest["edges"][key] = clip.close(a.out / f"{name}.jpg")
        print(f"[clips] {key}: {manifest['edges'][key]} ({time.monotonic() - t0:.0f} s)", flush=True)
    manifest["conditional"] = {}
    for c in ((record.get("refinement") or {}).get("confirmations") or []):    # the conditional-edge clip set
        ea, eb = c["edge"]
        key = f"{ea}>{eb}|{c['c']}"
        if not c["confirmed"] or (only is not None and key not in only):
            continue
        name = f"cond_{ea}__{eb}__{c['c']}"
        clip = Clip(a.out / f"{name}.mp4")
        head = f"{short(eb)} needs one of {short(ea)} or {short(c['c'])}  ·  {short(c['c'])} in, {short(ea)} out"
        for n, tname in enumerate(c["traj"], 1):
            data, traj = load_traj(folder, tname)
            dish = data["dish"]
            colors = {i: tinted[i] for i in (ea, eb, c["c"])}
            detail = (f"{short(eb)} goes in with {short(c['c'])} placed and {short(ea)} absent" if n == 1 else
                      f"then {short(ea)} goes in after {short(eb)}")
            segment(stage, clip, traj, colors, head, detail)
        manifest["conditional"][key] = clip.close(a.out / f"{name}.jpg")
        print(f"[clips] {key}: {manifest['conditional'][key]} ({time.monotonic() - t0:.0f} s)", flush=True)
    (a.out / "clips.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"[RESULT] PASS {record['load_id']} clips: witness {'witness' in manifest}, load {'load' in manifest}, edges "
          f"{len(manifest['edges'])} of {sum(e.get('swap') is not None for e in record.get('edges') or [])} swap-tested, "
          f"conditional {len(manifest['conditional'])} "
          f"({time.monotonic() - t0:.0f} s)", flush=True)


if __name__ == "__main__":
    main()
