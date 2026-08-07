# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""A/B a replayed trial video against the live capture of the same trial.

The acceptance test for the experiment/evaluation split: if a video rendered from a recorded
trajectory matches the one captured live during physics, then evaluation genuinely does not
need the experiment.

Checks, in order of what they catch:

1. **Frame count** — a stride or capture-mask bug.
2. **Per-frame PSNR** — geometry error. Thresholds are calibrated against a measured-good
   replay (mean 38 dB, p1 31 dB, min 26 dB): the floor sits where RTX sampling, H.264
   compression on both sides, and one step of sub-millimetre settling land, well above the
   ~11-23 dB a real pose or scene-setup error produces.
3. **Alignment** — replay[k] must match live[k] better than live[k±1], which catches an
   off-by-one in where the recorder samples relative to the step.
4. **Head frames** — camera warm-up asymmetry (early frames come back black without it).

Emits a worst-case contact sheet (live | replay | 10x difference) so a real error is
immediately distinguishable from noise: uniform speckle is sampling, a crisp silhouette is a
pose bug.

Venv-side, no Kit:
    env_isaaclab/bin/python scripts/evaluation/compare_videos.py --live A.mp4 --replay B.mp4
"""

import argparse
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Compare a live-captured trial video with its replay.")
parser.add_argument("--live", type=str, required=True)
parser.add_argument("--replay", type=str, required=True)
parser.add_argument("--min_psnr_db", type=float, default=20.0, help="Minimum per-frame PSNR.")
parser.add_argument("--p99_psnr_db", type=float, default=25.0, help="Required 1st-percentile PSNR.")
parser.add_argument("--align_samples", type=int, default=20)
parser.add_argument("--sheet", type=str, default=None, help="Worst-frames contact sheet PNG.")
args = parser.parse_args()

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def psnr(a: np.ndarray, b: np.ndarray) -> float:
    mse = float(np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2))
    return float("inf") if mse == 0 else 10.0 * np.log10(255.0**2 / mse)


def read_frames(path: str) -> list[np.ndarray]:
    import imageio.v2 as imageio  # noqa: PLC0415

    reader = imageio.get_reader(path)
    try:
        return [np.asarray(f) for f in reader]
    finally:
        reader.close()


def main() -> None:
    live, replay = read_frames(args.live), read_frames(args.replay)
    print(f"[INFO] live {len(live)} frames, replay {len(replay)} frames")
    check("frame count matches", len(live) == len(replay), f"{len(live)} vs {len(replay)}")
    n = min(len(live), len(replay))
    if n == 0:
        print("[RESULT] FAIL")
        raise SystemExit(1)

    scores = np.array([psnr(live[i], replay[i]) for i in range(n)])
    finite = scores[np.isfinite(scores)]
    p1 = float(np.percentile(finite, 1)) if len(finite) else float("inf")
    check("per-frame PSNR floor", float(scores.min()) >= args.min_psnr_db,
          f"min {scores.min():.1f} dB (threshold {args.min_psnr_db})")
    check("1st-percentile PSNR", p1 >= args.p99_psnr_db,
          f"p1 {p1:.1f} dB (threshold {args.p99_psnr_db})")
    print(f"[INFO] PSNR mean {scores.mean():.1f} dB, median {np.median(scores):.1f} dB")

    # Alignment: the matching frame must beat its neighbours. Only meaningful where the scene
    # is actually moving — during a settle phase consecutive frames are near-identical and the
    # comparison is noise, so sample the frames with the most inter-frame motion.
    motion = np.array([
        float(np.mean(np.abs(live[k].astype(np.int16) - live[k + 1].astype(np.int16))))
        for k in range(1, n - 2)
    ])
    idx = (np.argsort(motion)[::-1][: args.align_samples] + 1)
    misaligned = [
        int(k) for k in idx
        if psnr(live[k], replay[k]) < max(psnr(live[k - 1], replay[k]), psnr(live[k + 1], replay[k]))
    ]
    check("frame alignment (on the most-moving frames)", not misaligned,
          f"{len(misaligned)}/{len(idx)} match a neighbour better")

    head = scores[: min(10, n)]
    check("head frames (camera warm-up)", float(head.min()) >= args.min_psnr_db,
          f"min {head.min():.1f} dB over the first {len(head)} frames")

    if args.sheet:
        from PIL import Image  # noqa: PLC0415

        from dishsim.media import contact_sheet  # noqa: PLC0415

        worst = np.argsort(scores)[: min(4, n)]
        images, labels = [], []
        for k in worst:
            diff = np.clip(np.abs(live[k].astype(np.int16) - replay[k].astype(np.int16)) * 10, 0, 255)
            images += [live[k], replay[k], diff.astype(np.uint8)]
            labels += [f"live #{k}", f"replay #{k} ({scores[k]:.0f} dB)", f"10x diff #{k}"]
        os.makedirs(os.path.dirname(os.path.abspath(args.sheet)), exist_ok=True)
        contact_sheet(images, labels, args.sheet, cols=3)
        print(f"[INFO] worst-frames sheet: {args.sheet}")

    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
