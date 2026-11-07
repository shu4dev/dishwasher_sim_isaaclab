#!/usr/bin/env python3
"""Stop-motion video of one recorded bench episode with Isaac RTX, built on the bench kit's scene (Isaac Lab 3.0).

    code/util/run_kit.sh code/paths/scripts/frigidaire_planner_video.py \
        --instance <work>/run/instance.json --episode <work>/run/episodes/<track>/<tier>/<pair>__<algo>.json \
        --out-dir data/media/bench/v2/<pair> --video-width 640 --headless --enable_cameras

The video stage (code/paths/video.py) runs it once per recorded episode that lacks a video. The scene is the bench
kit's (frigidaire_bench_kit.make_backend: the spawned twin, one RigidObject per dish, the kit's per-dish tints, the
counter slab, BENCH_CAMERA) with the dishes kinematic. It restores the instance's start (racks at the start joints,
dishes at initial_snapshot.poses); for each executed move it writes the record's poses_after of every dish that moved,
steps once and renders the kit's STILL_RENDERS, so no physics moves a dish. Captions: the start, one line per move, the
end with S_final and S_ref from the run stage's <record>.fair.json. Writes <pair>__<track>__<algo>.mp4 (as
.partial.mp4 until every check passed; a failed render leaves none), the first and last move frames as PNGs and
<record stem>__<track>_video_evidence.json. An existing mp4 is never overwritten: delete it to render again.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
KINEMATIC = True              # dishes kinematic so a written pose holds; False = dynamic dishes, one tick after each write
POSE_TOL_M = .005             # after the tick every dish sits within this of the pose the frame shows
MOVED_M, MIN_CHANGED_PX = .05, 30   # a dish that travelled this far must change this many pixels (a stale renderer: ~0)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def caption_style(fonts, size, head, lines):
    """One font size for a whole video: the largest (at most size[1] // 30 px) at which the bold head line and every
    caption line fit the frame width. The bar holds the two lines."""
    for px in range(max(12, size[1] // 30), 9, -1):
        bold, regular = fonts["title"].font_variant(size=px), fonts["detail"].font_variant(size=px)
        if bold.getlength(head) <= size[0] - 24 and max(regular.getlength(t) for t in lines) <= size[0] - 24:
            break
    return {"bold": bold, "regular": regular, "px": px, "bar": round(3.3 * px)}


def draw_caption(frame, style, head, text):
    from PIL import ImageDraw
    draw = ImageDraw.Draw(frame)
    draw.rectangle((0, 0, frame.width, style["bar"]), fill=(18, 28, 40))
    draw.text((12, round(.45 * style["px"])), head, fill=(246, 249, 252), font=style["bold"])
    draw.text((12, round(1.85 * style["px"])), text, fill=(218, 229, 239), font=style["regular"])


def captions(inst, episode, fair, in_counter_band):
    """(head line, [start, move 1..n, end]) and the executed moves shown."""
    import numpy as np
    objects, counter = inst["objects"], inst["counter"]
    n_counter = sum(o["start"] == "Counter" for o in objects)
    executed = [mv for mv in episode["moves"] if "poses_after" in mv]   # refusals never ran
    n = len(executed)
    head = f"{inst['instance_id']} · {episode['algorithm']}"
    lines = [f"{episode['track']} track: {n_counter} of {len(objects)} dishes on the counter, "
             f"allowance {counter.get('slack', counter['cap'] - n_counter)} (cap {counter['cap']})"]
    for k, mv in enumerate(executed, 1):
        T = np.eye(4)
        T[:3, 3] = mv["poses_after"][mv["item_id"]]["position_m"]
        where = (" failed settle, put back" if mv.get("kind") == "failed-settle" else
                 " -> counter (buffer)" if in_counter_band(T, counter) else " -> rack")
        lines.append(f"move {k}/{n}: {mv['item_id'].replace('_', ' ')}{where}")
    end = (f"done: {n} moves, every dish racked" if episode.get("success") else
           f"stopped: {episode.get('abort') or (episode.get('end_check') or {}).get('outcome')}")
    if fair:
        end += f" · S = {fair['S_final']:.3f} (S_ref {fair['S_ref']:.3f})"
    return head, lines + [end], executed


def render(args, report):
    import numpy as np
    from PIL import Image
    from dishsim.media import VideoWriter
    from motion_planning.planner import in_counter_band
    import arrangement_optimization.frigidaire_bench as B
    from frigidaire_bench_kit import STILL_RENDERS, make_backend    # code/sampler/scripts
    from frigidaire_initial_state_render import HEIGHT, WIDTH, caption_fonts

    inst = json.loads(args.instance.read_text())
    episode = json.loads(args.episode.read_text())
    if inst.get("purpose") != "hotec_bench_instance" or episode.get("instance") != inst["instance_id"]:
        raise ValueError("Instance and episode do not match")
    fair_path = args.episode.with_name(args.episode.stem + ".fair.json")
    fair = json.loads(fair_path.read_text()) if fair_path.is_file() else None
    head, lines, executed = captions(inst, episode, fair, in_counter_band)
    renders = args.renders_per_frame or STILL_RENDERS
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    saved = inst["initial_snapshot"]["poses"]
    report.update(instance=str(args.instance), episode=str(args.episode), instance_sha256=digest(args.instance),
                  episode_sha256=digest(args.episode), fair=fair, moves_shown=len(executed), camera=B.BENCH_CAMERA,
                  renders_per_frame=renders, kinematic=KINEMATIC,
                  runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip()},
                  scope="Isaac RTX stop-motion of a recorded episode: poses from the record, "
                        + ("kinematic bodies, no physics." if KINEMATIC else "dynamic bodies, one tick after each write."))
    entries = [{"object_id": o["object_id"], "kind": o["kind"], "rack": o.get("rack", "LowerRack"), "pose_world": o["pose_world"]}
               for o in inst["objects"]]
    counter_ids = [o["object_id"] for o in inst["objects"] if o["start"] == "Counter"]
    args.max_wall_seconds = 1800.
    backend, rig = make_backend(args, B, entries, Path(tempfile.mkdtemp(prefix="video_kit_")), counter_ids,
                                cameras=True, kinematic=KINEMATIC)
    rig = rig["rig"]
    backend.restore(inst["initial_snapshot"])               # racks at the start joints, the basket at its start pose
    shown = {oid: saved[oid] for oid in kinds}
    for oid in kinds:
        backend.set_rigid_pose(backend.objects[oid], shown[oid])
    worst = [0.]

    def step():
        """One tick; every dish must sit where the frame says it is."""
        backend.tick()
        now = backend.poses()
        err, oid = max((float(np.linalg.norm(np.subtract(now[i]["position_m"], shown[i]["position_m"]))), i) for i in kinds)
        worst[0] = max(worst[0], err)
        if err > POSE_TOL_M:
            raise ValueError(f"{oid} sits {err * 1e3:.1f} mm off its recorded pose after the tick")

    step()
    joint_error = max(abs(backend.joints()[j] - v) for j, v in inst["initial_snapshot"]["joints"].items())
    if joint_error > POSE_TOL_M:
        raise ValueError(f"the racks sit {joint_error * 1e3:.1f} mm (or mrad) off the start joints after the restore")
    for _ in range(args.warmup_renders):          # the first frames after the boot come back dark
        backend.sim.render()
        rig.update(backend.sim.get_physics_dt())
    size = (args.video_width, args.video_width * HEIGHT // WIDTH)
    fonts, _ = caption_fonts()
    style = caption_style(fonts, size, head, lines)
    args.partial.parent.mkdir(parents=True, exist_ok=True)
    args.partial.unlink(missing_ok=True)
    video = VideoWriter(str(args.partial), fps=args.fps)
    frames_written, stills, changes, previous = 0, {}, [], [None]

    def capture(text, hold, tag=None, travel=0.):
        nonlocal frames_written
        for _ in range(renders):                  # RTX accumulation: fewer renders leave ghosts of moved dishes
            backend.sim.render()
            rig.update(backend.sim.get_physics_dt())
        pixels = rig.grab_one("initial")
        if pixels.std() <= 5 or pixels.max() <= 60:
            raise ValueError("Isaac camera produced an empty frame")
        frame = Image.fromarray(pixels).resize(size, Image.LANCZOS)
        raw = np.array(frame).astype(int)
        if previous[0] is not None:
            changed = int((np.abs(raw - previous[0]).max(axis=2) > 40).sum())
            changes.append(changed)
            print(f"[frame] {text.replace(chr(183), '|')}: {changed} px changed, travel {travel:.3f} m", flush=True)
            if travel > MOVED_M and changed < MIN_CHANGED_PX:
                raise ValueError(f"frame unchanged after a {travel:.2f} m move ({changed} px): the renderer shows stale poses")
        previous[0] = raw
        draw_caption(frame, style, head, text)
        arr = np.asarray(frame)
        for _ in range(hold):
            video.add(arr)
            frames_written += 1
        if tag:
            path = args.out_dir / f"{args.stem}_{tag}.png"
            frame.save(path)
            stills[tag] = path.name

    try:
        capture(lines[0], args.hold_frames * 2, tag="start")
        for k, mv in enumerate(executed, 1):
            after = mv["poses_after"]
            moved = [oid for oid, pose in after.items()
                     if np.abs(np.subtract(pose["position_m"], shown[oid]["position_m"])).max() > 1e-3
                     or np.abs(np.subtract(pose["quaternion_xyzw"], shown[oid]["quaternion_xyzw"])).max() > 1e-3]
            travel = max((float(np.linalg.norm(np.subtract(after[oid]["position_m"], shown[oid]["position_m"]))) for oid in moved),
                         default=0.)
            for oid in moved:
                backend.set_rigid_pose(backend.objects[oid], after[oid])
                shown[oid] = after[oid]
            step()
            capture(lines[k], args.hold_frames, tag=("end" if k == len(executed) else None), travel=travel)
        capture(lines[-1], args.hold_frames * 3)
    finally:
        video.close()
    if video.frames != frames_written or args.partial.stat().st_size < 50_000:
        raise ValueError(f"video incomplete: {video.frames} frames, {args.partial.stat().st_size} bytes")
    os.replace(args.partial, args.video)
    report.update(result="PASS", video_file=args.video.name, video_sha256=digest(args.video), video_bytes=args.video.stat().st_size,
                  frames=frames_written, fps=args.fps, duration_s=round(frames_written / args.fps, 2), resolution=list(size),
                  stills=stills, changed_px=changes, max_pose_error_mm=round(worst[0] * 1e3, 3),
                  start_joint_error=round(joint_error, 6))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--instance", type=Path, required=True, help="the run stage's <work>/run/instance.json")
    parser.add_argument("--episode", type=Path, required=True, help="an episode record of that run")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--hold-frames", type=int, default=18)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--renders-per-frame", type=int, default=None,
                        help="RTX renders before each frame is grabbed (default: the bench kit's STILL_RENDERS)")
    parser.add_argument("--warmup-renders", type=int, default=32)
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit in the try below
    add_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    args.instance, args.episode, args.usd, args.out_dir = (p.resolve() for p in (args.instance, args.episode, args.usd, args.out_dir))
    episode = json.loads(args.episode.read_text())
    args.stem = f"{episode['instance']}__{episode['track']}__{episode['algorithm']}"
    args.video, args.partial = args.out_dir / f"{args.stem}.mp4", args.out_dir / f"{args.stem}.partial.mp4"
    evidence_path = args.out_dir / f"{args.episode.stem}__{episode['track']}_video_evidence.json"
    if args.video.exists():
        print(f"[RESULT] FAIL refusing to overwrite {args.video}", flush=True)
        return 1
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 2, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat()}
    app, started = None, time.monotonic()
    try:
        app = launch(args)
        args.app = app
        sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util"), str(ROOT / "code/sampler/scripts")]
        render(args, report)
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", error=repr(exc))
    finally:
        if report["result"] != "PASS":
            args.partial.unlink(missing_ok=True)
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), wall_seconds=time.monotonic() - started)
        evidence_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")   # a stale one is replaced
        print(f"[RESULT] {report['result']}: {evidence_path}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
