#!/usr/bin/env python3
"""The s0 rerun's stitched video (plans/2026-11-04-s0-rerun.md), Mac side after the copy-back: one 3 s title card per dish in
the executed order (the load's executed_order mapped back onto the dag order: the sample's 15 entries, each rack in the order
its chain ran), each followed by that dish's Warp replay clip when it has one; ffmpeg concat into <media dir>/s0_grasp_place.mp4
(h264, yuv420p, 1280 x 536, 10 fps, the clips' format) and contact_sheet.png of the landed stills. A reject row gets its card
and no clip. Needs PIL (the Mac's /opt/anaconda3/bin/python3) and ffmpeg.

    /opt/anaconda3/bin/python3 code/sampler/robot/scripts/frigidaire_run_video.py \\
        data/results/robot/samples_v1/<sample_id>__final
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
W_, H_, FPS, CARD_S = 1280, 536, 10, 3
NOTES = {"plate_03": "placed before plate_05 for the arm's access: plate_05 at its goal blocks plate_03's descent (wrist 1, the "
                     "forearm and the hand), plate_03 at its goal leaves plate_05 a 115 mm start"}


def font(size, bold=False):
    from PIL import ImageFont
    import matplotlib
    return ImageFont.truetype(str(Path(matplotlib.get_data_path()) / "fonts/ttf" /
                                  ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")), size)


def wrap(text, f, width, draw):
    out, line = [], ""
    for w in str(text).split():
        t = (line + " " + w).strip()
        if draw.textlength(t, font=f) <= width:
            line = t
        else:
            out.append(line)
            line = w
    return out + ([line] if line else [])


def verdict(r):
    if r.get("seated") and r.get("at_goal"):
        return "placed: seated on its goal and at goal", (120, 220, 140)
    if not r.get("ran_isaac"):
        return "reject row: no Isaac run", (235, 190, 90)
    if r.get("at_goal"):
        return "at goal, not seated", (235, 190, 90)
    return "not placed", (240, 120, 110)


def card(r, k, n, path):
    from PIL import Image, ImageDraw
    img = Image.new("RGB", (W_, H_), (18, 28, 40))
    d = ImageDraw.Draw(img)
    x, y = 48, 40
    d.text((x, y), f"{k}/{n}  {r['dish']}", fill=(246, 249, 252), font=font(44, True))
    y += 66
    d.text((x, y), f"{r['kind']}, slot {r['slot']} ({r['rack']})", fill=(190, 207, 222), font=font(24))
    y += 40
    d.text((x, y), f"move: {r['move']}", fill=(190, 207, 222), font=font(24))
    y += 52
    v, col = verdict(r)
    d.text((x, y), v, fill=col, font=font(32, True))
    y += 50
    lg = r.get("landing") or {}
    if r.get("ran_isaac"):
        d.text((x, y), f"picked {r.get('picked')}, carried {r.get('carried')}, seated {r.get('seated')}, at goal {r.get('at_goal')}"
               + (f"; landing {lg.get('lateral_mm')} mm lateral, {lg.get('dz_mm')} mm dz, {lg.get('tilt_deg')} deg"
                  if lg else ""), fill=(220, 228, 236), font=font(20))
        y += 34
    cause = r.get("cause")
    if cause and not (r.get("seated") and r.get("at_goal")):
        f_ = font(20)
        for line in wrap(f"cause: {cause}" + (f" ({r['cause_detail']})" if r.get("cause_detail") else ""), f_, W_ - 2 * x, d)[:6]:
            d.text((x, y), line, fill=(220, 228, 236), font=f_)
            y += 28
    if r["dish"] in NOTES:
        f_ = font(20)
        y += 8
        for line in wrap(NOTES[r["dish"]], f_, W_ - 2 * x, d)[:3]:
            d.text((x, y), line, fill=(235, 190, 90), font=f_)
            y += 28
    d.text((x, H_ - 44), "s0 rerun, plans/2026-11-04-s0-rerun.md" + ("" if r.get("clip") else "  (no clip: no Isaac run)"),
           fill=(120, 140, 160), font=font(16))
    img.save(path)


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        sys.exit(f"[RESULT] FAIL {' '.join(map(str, cmd))[:300]}: {p.stderr[-800:]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("load_dir", type=Path)
    ap.add_argument("--crf", type=int, default=26)
    a = ap.parse_args()
    d = a.load_dir if a.load_dir.is_absolute() else ROOT / a.load_dir
    load = json.loads((d / "load.json").read_text())
    rows = {r["dish"]: r for r in (json.loads(x) for x in (d / "rows.jsonl").read_text().splitlines() if x.strip())}
    media = ROOT / load["media_dir"]
    ex = load.get("executed_order") or load["order"]
    racks = {x["name"]: x["rack"] for x in load["dishes"]}
    seq = {rk: [n for n in ex if racks[n] == rk] for rk in ("UpperRack", "LowerRack")}
    slots = {rk: [n for n in load["order"] if racks[n] == rk and n in ex] for rk in seq}
    swap = {s_: e_ for rk in seq for s_, e_ in zip(slots[rk], seq[rk])}   # each rack's run order on its dag positions
    order = [swap.get(n, n) for n in load["order"]]
    if sorted(order) != sorted(load["order"]):
        sys.exit(f"[RESULT] FAIL the executed order {order} is not a permutation of the dag order")
    work = d / "_video"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir()
    parts, landed = [], []
    for k, n in enumerate(order, 1):
        r = rows[n]
        png, seg = work / f"card_{k:02d}_{n}.png", work / f"card_{k:02d}_{n}.mp4"
        card(r, k, len(order), png)
        run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-framerate", str(FPS), "-i", str(png), "-t", str(CARD_S),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", str(FPS), "-vf", f"scale={W_}:{H_}", str(seg)])
        parts.append(seg)
        if r.get("clip"):
            parts.append(ROOT / r["clip"])
        if r.get("landed_still"):
            landed.append((n, ROOT / r["landed_still"], verdict(r)[0]))
    lst = work / "list.txt"
    lst.write_text("".join(f"file '{p}'\n" for p in parts))
    out = media / "s0_grasp_place.mp4"
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c:v", "libx264", "-pix_fmt",
         "yuv420p", "-r", str(FPS), "-vf", f"scale={W_}:{H_}", "-crf", str(a.crf), "-preset", "medium", "-movflags", "+faststart",
         str(out)])
    from PIL import Image, ImageDraw
    cols, tw, th = 3, W_ // 2, H_ // 2 + 30
    rows_n = max(1, (len(landed) + cols - 1) // cols)
    sheet = Image.new("RGB", (cols * tw, rows_n * th), (18, 28, 40))
    dr = ImageDraw.Draw(sheet)
    for i, (n, p, v) in enumerate(landed):
        im = Image.open(p).convert("RGB").resize((tw, H_ // 2))
        x0, y0 = (i % cols) * tw, (i // cols) * th
        sheet.paste(im, (x0, y0 + 30))
        dr.text((x0 + 8, y0 + 6), f"{n}: {v}", fill=(246, 249, 252), font=font(16, True))
    sheet.save(media / "contact_sheet.png")
    probe = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "stream=codec_name,width,height,r_frame_rate,pix_fmt:format=duration",
                            "-of", "compact=p=0", str(out)], capture_output=True, text=True).stdout.strip().replace("\n", "; ")
    print(f"[RESULT] PASS video {out.relative_to(ROOT)} ({len(order)} cards, {sum(1 for p in parts if 'card_' not in p.name)} clips; "
          f"{probe}); contact sheet of {len(landed)} landed stills -> {(media / 'contact_sheet.png').relative_to(ROOT)}; order {order}")


if __name__ == "__main__":
    main()
