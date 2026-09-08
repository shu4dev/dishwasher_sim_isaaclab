# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Archive gitignored artifacts into manifest-stamped tarballs (+ optional HF upload).

Producer side of ``restore_assets.py``. Every tarball kind maps to a subtree of the data
roots; each tarball carries a ``MANIFEST.json`` with the kind, tag, git SHA, and a sha256
per member so the restore can verify every extracted file. Kinds:

- ``assets``   — built props (minus ``_download/``), every geometry cache, derived
                 dishwasher USDs under ``assets/artvip``, and ``results/`` (the legacy pack).
- ``media``    — ``media/`` (recorded evidence).
- ``models``   — ``assets/models/`` (the standalone Bosch 800 USD asset + textures).
- ``evidence`` — ``assets/evidence/`` (that asset's validation stills/video/report).

Tarballs land in ``outputs/archive/`` (on the 2 TB drive). ``--upload`` pushes them to the
public HF dataset and rewrites ``latest.json`` by MERGING into the remote copy: only the
kinds built in this run change, every other kind keeps pointing at its current tarball.
Upload needs a write token in the environment (``HF_TOKEN``) — pass it to the one command
that uploads, never store it on disk.

Kit-free (inside the container, via run_py.sh):
    scripts/run_py.sh scripts/tools/archive_assets.py --kinds models evidence
    HF_TOKEN=... scripts/run_py.sh scripts/tools/archive_assets.py --kinds models evidence \\
        --upload [--repo <user>/<name>] [--card outputs/archive/hf_README.md]
"""

import argparse
import glob
import hashlib
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
KINDS = ("assets", "media", "models", "evidence")

parser = argparse.ArgumentParser(description="Archive gitignored artifacts to tarballs (+ HF).")
parser.add_argument("--kinds", nargs="+", choices=KINDS, required=True,
                    help="Tarball kinds to build (and upload).")
parser.add_argument("--upload", action="store_true", help="Upload to the public HF dataset repo.")
parser.add_argument("--repo", type=str, default="shu4dev/dishsim-assets",
                    help="HF dataset repo id (default: shu4dev/dishsim-assets).")
parser.add_argument("--card", type=str, default=None,
                    help="Local dataset-card markdown to upload as README.md (with --upload).")
parser.add_argument("--tag", type=str, default=None,
                    help="Override the <date>_<gitsha> tag (default: today + HEAD short SHA).")
args = parser.parse_args()


def git_sha() -> str:
    try:
        # -c safe.directory: the container runs as root on a bind mount owned by the host user
        return subprocess.check_output(
            ["git", "-c", "safe.directory=*", "rev-parse", "--short", "HEAD"],
            cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "nogit"


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


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


def walk(root: str, exclude: str | None = None):
    abs_root = os.path.join(PROJECT_ROOT, root)
    if not os.path.isdir(abs_root):
        return
    abs_excl = os.path.join(PROJECT_ROOT, exclude) if exclude else None
    for dirpath, _, files in os.walk(abs_root):
        if abs_excl and os.path.commonpath([dirpath, abs_excl]) == abs_excl:
            continue
        for fn in sorted(files):
            if fn == ".DS_Store":
                continue
            p = os.path.join(dirpath, fn)
            yield p, os.path.relpath(p, PROJECT_ROOT)


def iter_members(kind: str):
    """(abs_path, arcname) pairs for one tarball kind."""
    if kind == "media":
        yield from walk("media")
    elif kind == "models":
        yield from walk("assets/models")
    elif kind == "evidence":
        yield from walk("assets/evidence")
    else:  # assets (legacy pack)
        yield from walk("assets/props", exclude="assets/props/_download")
        yield from walk("assets/cache")
        yield from walk("results")
        # derived dishwasher USDs only (the ArtVIP originals re-download from HF)
        for pat in ("*_rl.usda", "*_v0*.usda"):
            for p in sorted(glob.glob(os.path.join(PROJECT_ROOT, "assets", "artvip", "**", pat),
                                      recursive=True)):
                yield p, os.path.relpath(p, PROJECT_ROOT)


def build_tarball(kind: str, tag: str) -> str:
    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    out = os.path.join(ARCHIVE_DIR, f"dishsim_{kind}_{tag}.tar.gz")
    n, total, digests = 0, 0, {}
    manifest = {"kind": kind, "tag": tag, "git_sha": git_sha(),
                "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "cache_stamps": cache_stamps() if kind == "assets" else {}}
    with tarfile.open(out, "w:gz") as tf:
        for p, arc in iter_members(kind):
            tf.add(p, arcname=arc, recursive=False)
            digests[arc] = sha256_of(p)
            n += 1
            total += os.path.getsize(p)
        if n == 0:
            raise SystemExit(f"[FAIL] kind '{kind}': nothing to archive")
        manifest["n_files"] = n
        manifest["bytes"] = total
        manifest["sha256"] = digests
        blob = json.dumps(manifest, indent=2, sort_keys=True).encode()
        info = tarfile.TarInfo("MANIFEST.json")
        info.size = len(blob)
        tf.addfile(info, io.BytesIO(blob))
    print(f"[INFO] {out}: {n} files, {total / 1e6:.1f} MB raw, "
          f"{os.path.getsize(out) / 1e6:.1f} MB compressed")
    return out


def upload(paths: dict[str, str], tag: str) -> None:
    from huggingface_hub import HfApi, hf_hub_download  # noqa: PLC0415

    api = HfApi()
    repo = args.repo
    api.create_repo(repo_id=repo, repo_type="dataset", private=False, exist_ok=True)
    # merge into the remote pointer so untouched kinds keep resolving
    try:
        latest = json.load(open(hf_hub_download(repo_id=repo, repo_type="dataset",
                                                filename="latest.json")))
    except Exception as e:  # first upload to a fresh repo
        print(f"[INFO] no remote latest.json ({type(e).__name__}); starting fresh")
        latest = {"files": {}}
    for kind, p in paths.items():
        print(f"[INFO] uploading {os.path.basename(p)} ...")
        api.upload_file(path_or_fileobj=p, path_in_repo=os.path.basename(p),
                        repo_id=repo, repo_type="dataset")
        latest["files"][kind] = os.path.basename(p)
    latest["tag"] = tag
    api.upload_file(path_or_fileobj=json.dumps(latest, indent=2).encode(),
                    path_in_repo="latest.json", repo_id=repo, repo_type="dataset")
    if args.card:
        api.upload_file(path_or_fileobj=os.path.abspath(args.card), path_in_repo="README.md",
                        repo_id=repo, repo_type="dataset")
        print("[INFO] dataset card updated")
    print(f"[INFO] uploaded to https://huggingface.co/datasets/{repo} (public); latest.json = "
          f"{json.dumps(latest['files'])}")
    print(f"[INFO] restore on a fresh instance:\n"
          f"    scripts/run_py.sh scripts/tools/restore_assets.py --repo {repo} "
          f"--kinds {' '.join(paths)}")


def main() -> None:
    tag = args.tag or f"{datetime.now(timezone.utc).strftime('%Y%m%d')}_{git_sha()}"
    paths = {kind: build_tarball(kind, tag) for kind in args.kinds}
    if args.upload:
        upload(paths, tag)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
