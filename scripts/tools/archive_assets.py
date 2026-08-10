# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Archive the generated (gitignored) artifacts so a fresh instance restores in minutes.

Builds manifest-stamped tarballs under ``outputs/archive/`` and optionally uploads them to
a PUBLIC Hugging Face dataset repo. Everything archived is redistributable with attribution:
ArtVIP-derived USDs (Apache-2.0), YCB-derived meshes (YCB dataset terms, cited in the
README), and this project's own procedural assets and caches. The NVIDIA-EULA-encumbered
mug was replaced by the public YCB ``025_mug`` scan in the 2026-08-10 migration; the robot
USD is runtime-fetched from NVIDIA's bucket and never archived.

- ``dishsim_assets_<date>_<sha>.tar.gz`` — built prop USDs/textures/meshes/parts
  (``assets/props`` minus the re-downloadable ``_download/``), every geometry cache
  (``assets/cache`` — the ~1.5 h-of-Kit-compute part), the derived dishwasher USDs inside
  ``assets/artvip`` (the originals re-download from HF), and ``results/``.
- ``dishsim_media_<date>_<sha>.tar.gz`` — ``media/`` (run evidence; not regenerable).

Restore with ``scripts/tools/restore_assets.py``. Upload requires a one-time
``huggingface-cli login`` with a write token.

Venv-side, no Kit:
    env_isaaclab/bin/python scripts/tools/archive_assets.py [--upload] [--repo <user>/<name>]
"""

import argparse
import glob
import io
import json
import os
import subprocess
import sys
import tarfile
from datetime import datetime, timezone

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

ARCHIVE_DIR = os.path.join(PROJECT_ROOT, "outputs", "archive")

parser = argparse.ArgumentParser(description="Archive generated assets/media to tarballs (+ HF).")
parser.add_argument("--upload", action="store_true", help="Upload to the public HF dataset repo.")
parser.add_argument("--repo", type=str, default=None,
                    help="HF dataset repo id (default: <whoami>/dishsim-assets).")
parser.add_argument("--skip_media", action="store_true", help="Build/upload only the assets tarball.")
args = parser.parse_args()


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT, text=True
        ).strip()
    except Exception:
        return "nogit"


def cache_stamps() -> dict:
    """config_hash stamps of every cached scene_state.json (restore validates against these)."""
    stamps = {}
    for path in glob.glob(os.path.join(PROJECT_ROOT, "assets", "cache", "**", "scene_state.json"),
                          recursive=True):
        with open(path) as f:
            m = json.load(f)
        rel = os.path.relpath(path, PROJECT_ROOT)
        stamps[rel] = {"config_hash": m.get("config_hash"), "object_name": m.get("object_name"),
                       "scenario": m.get("scenario")}
    return stamps


def iter_members(kind: str):
    """(abs_path, arcname) pairs for one tarball kind ('assets' | 'media')."""
    if kind == "media":
        roots = [("media", None)]
    else:
        roots = [("assets/props", "assets/props/_download"), ("assets/cache", None),
                 ("results", None)]
    for root, exclude in roots:
        abs_root = os.path.join(PROJECT_ROOT, root)
        if not os.path.isdir(abs_root):
            continue
        for dirpath, _, files in os.walk(abs_root):
            if exclude and os.path.commonpath([dirpath, os.path.join(PROJECT_ROOT, exclude)]) == \
                    os.path.join(PROJECT_ROOT, exclude):
                continue
            for fn in files:
                p = os.path.join(dirpath, fn)
                yield p, os.path.relpath(p, PROJECT_ROOT)
    if kind == "assets":
        # derived dishwasher USDs only (the ArtVIP originals re-download from HF)
        for pat in ("*_rl.usda", "*_v0*.usda"):
            for p in glob.glob(os.path.join(PROJECT_ROOT, "assets", "artvip", "**", pat),
                               recursive=True):
                yield p, os.path.relpath(p, PROJECT_ROOT)


def build_tarball(kind: str, tag: str) -> str:
    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    out = os.path.join(ARCHIVE_DIR, f"dishsim_{kind}_{tag}.tar.gz")
    n, total = 0, 0
    manifest = {"kind": kind, "tag": tag, "git_sha": git_sha(),
                "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "cache_stamps": cache_stamps() if kind == "assets" else {}}
    with tarfile.open(out, "w:gz") as tf:
        for p, arc in iter_members(kind):
            tf.add(p, arcname=arc, recursive=False)
            n += 1
            total += os.path.getsize(p)
        manifest["n_files"] = n
        manifest["bytes"] = total
        blob = json.dumps(manifest, indent=2).encode()
        info = tarfile.TarInfo("MANIFEST.json")
        info.size = len(blob)
        tf.addfile(info, io.BytesIO(blob))
    print(f"[INFO] {out}: {n} files, {total / 1e6:.1f} MB raw, "
          f"{os.path.getsize(out) / 1e6:.1f} MB compressed")
    return out


def upload(paths: list[str], tag: str) -> None:
    from huggingface_hub import HfApi  # noqa: PLC0415

    api = HfApi()
    user = api.whoami()["name"]
    repo = args.repo or f"{user}/dishsim-assets"
    api.create_repo(repo_id=repo, repo_type="dataset", private=False, exist_ok=True)
    for p in paths:
        print(f"[INFO] uploading {os.path.basename(p)} ...")
        api.upload_file(path_or_fileobj=p, path_in_repo=os.path.basename(p),
                        repo_id=repo, repo_type="dataset")
    latest = {"tag": tag, "files": {os.path.basename(p).split("_")[1]: os.path.basename(p)
                                    for p in paths}}
    api.upload_file(path_or_fileobj=json.dumps(latest, indent=2).encode(),
                    path_in_repo="latest.json", repo_id=repo, repo_type="dataset")
    print(f"[INFO] uploaded to https://huggingface.co/datasets/{repo} (public)")
    print(f"[INFO] restore on a fresh instance:\n"
          f"    env_isaaclab/bin/python scripts/tools/restore_assets.py --repo {repo} --with_media")


def main() -> None:
    tag = f"{datetime.now(timezone.utc).strftime('%Y%m%d')}_{git_sha()}"
    paths = [build_tarball("assets", tag)]
    if not args.skip_media:
        paths.append(build_tarball("media", tag))
    if args.upload:
        upload(paths, tag)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
