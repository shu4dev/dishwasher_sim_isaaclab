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
- ``models``   — ``assets/models/`` (standalone machine USD assets + textures; ``*.zip``
                 duplicates are excluded, see ``EXCLUDE_GLOBS``).
- ``evidence`` — ``assets/evidence/`` (asset validation stills/video/report).

Tarballs land in ``outputs/archive/`` (on the 2 TB drive). ``--upload`` pushes them to the
public HF dataset and rewrites ``latest.json`` by MERGING into the remote copy: only the
kinds built in this run change, every other kind keeps pointing at its current tarball. If
the remote ``latest.json`` cannot be read the upload ABORTS (a blank pointer map would break
``bootstrap.sh`` for every fresh box); ``--fresh`` is the explicit opt-in for a new repo.
Upload needs a write token in the environment (``HF_TOKEN``; ``run_py.sh`` forwards it into
the container only when set) — pass it to the one command that uploads, never store it.

Kit-free (inside the container, via run_py.sh):
    scripts/run_py.sh scripts/tools/archive_assets.py --status              # read-only diff
    scripts/run_py.sh scripts/tools/archive_assets.py --kinds assets models evidence
    HF_TOKEN=... scripts/run_py.sh scripts/tools/archive_assets.py \\
        --kinds assets models evidence --upload --no-build --tag <tag> \\
        [--repo <user>/<name>] [--card outputs/archive/hf_README.md]

``--status`` needs no token: it reads the public ``latest.json``, compares the LFS sha256 of
every referenced tarball against the same-named local file in ``outputs/archive/``, and flags
kinds whose source tree is newer than the local tarball (``stale-build``). It ends with
``[RESULT] SYNCED`` or ``[RESULT] OUT-OF-SYNC`` (exit 1).
"""

import argparse
import fnmatch
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
DEFAULT_REPO = "shu4dev/dishsim-assets"
# basename globs skipped per kind (the zips under assets/models duplicate their sibling dirs)
EXCLUDE_GLOBS = {"models": ("*.zip",)}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Archive gitignored artifacts to tarballs (+ HF).")
    parser.add_argument("--kinds", nargs="+", choices=KINDS, default=None,
                        help="Tarball kinds to build (and upload). Required unless --status.")
    parser.add_argument("--upload", action="store_true", help="Upload to the public HF dataset repo.")
    parser.add_argument("--repo", type=str, default=DEFAULT_REPO,
                        help=f"HF dataset repo id (default: {DEFAULT_REPO}).")
    parser.add_argument("--card", type=str, default=None,
                        help="Local dataset-card markdown to upload as README.md (with --upload).")
    parser.add_argument("--tag", type=str, default=None,
                        help="Override the <date>_<gitsha> tag (default: today + HEAD short SHA).")
    parser.add_argument("--no-build", action="store_true",
                        help="Reuse outputs/archive/dishsim_<kind>_<tag>.tar.gz instead of "
                             "rebuilding (needs --tag); uploads exactly the files already inspected.")
    parser.add_argument("--fresh", action="store_true",
                        help="Allow --upload to start a NEW latest.json when the remote one "
                             "cannot be read (first upload to an empty repo only).")
    parser.add_argument("--status", action="store_true",
                        help="Read-only: diff the remote latest.json + LFS sha256 against the "
                             "local tarballs and source trees; no token needed.")
    return parser


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


def walk(root: str, exclude: str | None = None, exclude_globs: tuple[str, ...] = ()):
    """(abs_path, arcname) for every regular file under PROJECT_ROOT/root.

    ``exclude`` drops one subtree; ``exclude_globs`` drops files by basename pattern.
    """
    abs_root = os.path.join(PROJECT_ROOT, root)
    if not os.path.isdir(abs_root):
        return
    abs_excl = os.path.join(PROJECT_ROOT, exclude) if exclude else None
    for dirpath, _, files in os.walk(abs_root):
        if abs_excl and os.path.commonpath([dirpath, abs_excl]) == abs_excl:
            continue
        for fn in sorted(files):
            if fn == ".DS_Store" or any(fnmatch.fnmatch(fn, g) for g in exclude_globs):
                continue
            p = os.path.join(dirpath, fn)
            yield p, os.path.relpath(p, PROJECT_ROOT)


def iter_members(kind: str):
    """(abs_path, arcname) pairs for one tarball kind."""
    if kind == "media":
        yield from walk("media")
    elif kind == "models":
        yield from walk("assets/models", exclude_globs=EXCLUDE_GLOBS["models"])
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


def tarball_path(kind: str, tag: str) -> str:
    return os.path.join(ARCHIVE_DIR, f"dishsim_{kind}_{tag}.tar.gz")


def newest_source_mtime(kind: str) -> float:
    """mtime of the newest file the kind would archive (0.0 when the kind is empty)."""
    return max((os.path.getmtime(p) for p, _ in iter_members(kind)), default=0.0)


def build_tarball(kind: str, tag: str) -> str:
    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    out = tarball_path(kind, tag)
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


# --------------------------------------------------------------------------- remote helpers

def fetch_remote_latest(repo: str) -> dict | None:
    """The public latest.json, or None when it cannot be read (missing repo/file, network)."""
    from huggingface_hub import hf_hub_download  # noqa: PLC0415

    try:
        with open(hf_hub_download(repo_id=repo, repo_type="dataset", filename="latest.json")) as f:
            return json.load(f)
    except Exception as e:
        print(f"[WARN] could not read remote latest.json: {type(e).__name__}: {e}")
        return None


def remote_sha256(repo: str, names: list[str]) -> dict[str, str | None]:
    """name -> LFS sha256 of the file on the Hub (None when absent or not stored as LFS)."""
    from huggingface_hub import HfApi  # noqa: PLC0415

    out: dict[str, str | None] = {n: None for n in names}
    if not names:
        return out
    for info in HfApi().get_paths_info(repo, names, repo_type="dataset"):
        lfs = getattr(info, "lfs", None)
        if lfs is None:
            continue
        sha = lfs.get("sha256") if isinstance(lfs, dict) else getattr(lfs, "sha256", None)
        out[getattr(info, "path", None) or getattr(info, "rfilename", "")] = sha
    return out


def merge_latest(remote: dict | None, built: dict[str, str], tag: str, fresh: bool = False) -> dict:
    """New latest.json: the remote pointer map with the built kinds replaced.

    ``built`` maps kind -> tarball basename. ``remote=None`` (unreadable) aborts unless
    ``fresh``: silently starting from ``{}`` would drop every untouched kind's pointer.
    """
    if remote is None:
        if not fresh:
            raise SystemExit("[FAIL] remote latest.json unreadable; refusing to overwrite it "
                             "with a partial pointer map (pass --fresh only for a NEW repo)")
        remote = {"files": {}}
    latest = {"files": dict(remote.get("files", {})), "tag": tag}
    latest["files"].update(built)
    return latest


def diff_status(remote_files: dict[str, str], remote_shas: dict[str, str | None],
                local_dir: str, local_kinds: set[str], newest_mtime=newest_source_mtime) -> list[dict]:
    """One row per kind known remotely or locally: {kind, remote, sha, build, ok}.

    sha:   'ok' | 'mismatch' | 'missing-local' | 'no-remote' | 'no-lfs'
    build: 'fresh' | 'stale' | '-'   (source tree newer than the local tarball -> stale)
    """
    rows = []
    for kind in sorted(set(remote_files) | set(local_kinds)):
        name = remote_files.get(kind)
        row = {"kind": kind, "remote": name or "-", "sha": "no-remote", "build": "-"}
        if name:
            local = os.path.join(local_dir, name)
            if not os.path.isfile(local):
                row["sha"] = "missing-local"
            else:
                rsha = remote_shas.get(name)
                row["sha"] = "no-lfs" if rsha is None else (
                    "ok" if rsha == sha256_of(local) else "mismatch")
                row["build"] = "stale" if newest_mtime(kind) > os.path.getmtime(local) else "fresh"
        row["ok"] = row["sha"] == "ok" and row["build"] == "fresh"
        rows.append(row)
    return rows


def local_tarball_kinds(local_dir: str = ARCHIVE_DIR) -> set[str]:
    kinds = set()
    for p in glob.glob(os.path.join(local_dir, "dishsim_*_*.tar.gz")):
        kind = os.path.basename(p).split("_", 2)[1]
        if kind in KINDS:
            kinds.add(kind)
    return kinds


def status(repo: str) -> int:
    remote = fetch_remote_latest(repo)
    if remote is None:
        print("[RESULT] OUT-OF-SYNC (remote latest.json unreadable)")
        return 1
    files = remote.get("files", {})
    shas = remote_sha256(repo, sorted(set(files.values())))
    rows = diff_status(files, shas, ARCHIVE_DIR, local_tarball_kinds())
    print(f"[INFO] remote tag {remote.get('tag')} @ {repo}")
    print(f"{'kind':<9} {'remote tarball':<42} {'sha256':<14} build")
    for r in rows:
        print(f"{r['kind']:<9} {r['remote']:<42} {r['sha']:<14} {r['build']}")
    synced = bool(rows) and all(r["ok"] for r in rows)
    print("[RESULT] SYNCED" if synced else "[RESULT] OUT-OF-SYNC")
    return 0 if synced else 1


def upload(paths: dict[str, str], tag: str, repo: str, card: str | None, fresh: bool) -> None:
    from huggingface_hub import HfApi  # noqa: PLC0415

    api = HfApi()
    api.create_repo(repo_id=repo, repo_type="dataset", private=False, exist_ok=True)
    # merge into the remote pointer so untouched kinds keep resolving (aborts if unreadable)
    latest = merge_latest(fetch_remote_latest(repo),
                          {k: os.path.basename(p) for k, p in paths.items()}, tag, fresh)
    for kind, p in paths.items():
        print(f"[INFO] uploading {os.path.basename(p)} ...")
        api.upload_file(path_or_fileobj=p, path_in_repo=os.path.basename(p),
                        repo_id=repo, repo_type="dataset")
    api.upload_file(path_or_fileobj=json.dumps(latest, indent=2).encode(),
                    path_in_repo="latest.json", repo_id=repo, repo_type="dataset")
    if card:
        api.upload_file(path_or_fileobj=os.path.abspath(card), path_in_repo="README.md",
                        repo_id=repo, repo_type="dataset")
        print("[INFO] dataset card updated")
    print(f"[INFO] uploaded to https://huggingface.co/datasets/{repo} (public); latest.json = "
          f"{json.dumps(latest['files'])}")
    print(f"[INFO] restore on a fresh instance:\n"
          f"    scripts/run_py.sh scripts/tools/restore_assets.py --repo {repo} "
          f"--kinds {' '.join(paths)}")


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.status:
        raise SystemExit(status(args.repo))
    if not args.kinds:
        raise SystemExit("[FAIL] --kinds is required (or --status)")
    if args.no_build and not args.tag:
        raise SystemExit("[FAIL] --no-build needs --tag <tag> naming the existing tarballs")
    tag = args.tag or f"{datetime.now(timezone.utc).strftime('%Y%m%d')}_{git_sha()}"
    paths = {}
    for kind in args.kinds:
        existing = tarball_path(kind, tag)
        if args.no_build:
            if not os.path.isfile(existing):
                raise SystemExit(f"[FAIL] --no-build: {existing} does not exist")
            print(f"[INFO] reusing {existing} ({os.path.getsize(existing) / 1e6:.1f} MB)")
            paths[kind] = existing
        else:
            paths[kind] = build_tarball(kind, tag)
    if args.upload:
        upload(paths, tag, args.repo, args.card, args.fresh)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
