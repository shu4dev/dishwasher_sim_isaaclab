# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Restore archived assets/media on a fresh instance (counterpart of scripts/tools/archive_assets.py).

Downloads the tarballs from the public HF dataset (or takes local paths), safe-extracts
them into the project root, re-downloads the ArtVIP originals (the derived dishwasher
``.usda`` layers reference that tree), and validates every restored geometry cache's
``config_hash`` stamp against the current ``config.py`` before declaring success.

Prerequisites on a fresh box: the planning venv (README setup step 1). The public dataset
downloads without a token (``huggingface-cli login`` is only needed for a private mirror).

    env_isaaclab/bin/python scripts/tools/restore_assets.py [--repo <id>] [--with_media]
    env_isaaclab/bin/python scripts/tools/restore_assets.py --local outputs/archive/dishsim_assets_<tag>.tar.gz
"""

import argparse
import json
import os
import sys
import tarfile

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Restore archived assets/media from HF (or local tarballs).")
parser.add_argument("--repo", type=str, default=None,
                    help="HF dataset repo id (default: <whoami>/dishsim-assets).")
parser.add_argument("--file", type=str, default="latest",
                    help="Assets tarball filename in the repo (default: resolve via latest.json).")
parser.add_argument("--with_media", action="store_true", help="Also restore the media tarball.")
parser.add_argument("--local", type=str, nargs="*", default=None,
                    help="Local tarball path(s) instead of downloading.")
parser.add_argument("--skip_tests", action="store_true", help="Skip the pytest validation pass.")
args = parser.parse_args()

ALLOWED_PREFIXES = ("assets/", "media/", "results/", "MANIFEST.json")


def fetch_tarballs() -> list[str]:
    if args.local:
        return [os.path.abspath(p) for p in args.local]
    from huggingface_hub import HfApi, hf_hub_download  # noqa: PLC0415

    repo = args.repo or f"{HfApi().whoami()['name']}/dishsim-assets"
    print(f"[INFO] restoring from https://huggingface.co/datasets/{repo}")
    if args.file == "latest":
        latest = json.load(open(hf_hub_download(repo_id=repo, repo_type="dataset",
                                                filename="latest.json")))
        names = [latest["files"]["assets"]]
        if args.with_media and "media" in latest["files"]:
            names.append(latest["files"]["media"])
    else:
        names = [args.file]
    return [hf_hub_download(repo_id=repo, repo_type="dataset", filename=n) for n in names]


def safe_extract(tar_path: str) -> dict:
    """Extract into the project root; only whitelisted top-level trees are allowed."""
    manifest = {}
    with tarfile.open(tar_path) as tf:
        for m in tf.getmembers():
            name = os.path.normpath(m.name)
            if name.startswith(("/", "..")) or not name.startswith(ALLOWED_PREFIXES):
                raise SystemExit(f"[FAIL] refusing to extract suspicious member: {m.name}")
            if m.islnk() or m.issym():
                raise SystemExit(f"[FAIL] refusing to extract link member: {m.name}")
        mf = tf.extractfile("MANIFEST.json")
        if mf is not None:
            manifest = json.load(mf)
        # every member is prefix-vetted above; "data" would refuse the deliberate
        # assets/media/results symlinks onto the big disk (OutsideDestinationError)
        tf.extractall(PROJECT_ROOT, members=[m for m in tf.getmembers()
                                             if m.name != "MANIFEST.json"], filter="fully_trusted")
    print(f"[INFO] extracted {os.path.basename(tar_path)} "
          f"({manifest.get('n_files', '?')} files, git {manifest.get('git_sha', '?')})")
    return manifest


def ensure_artvip() -> None:
    probe = os.path.join(PROJECT_ROOT, "assets", "artvip", "Articulated_objects",
                         "major_appliances", "dishwasher", "dishwasher_2",
                         "model_dishwasher_2.usda")
    if os.path.exists(probe):
        print("[INFO] ArtVIP originals present")
        return
    print("[INFO] downloading ArtVIP originals (~85 MB, HF)")
    from huggingface_hub import snapshot_download  # noqa: PLC0415

    snapshot_download(repo_id="X-Humanoid/ArtVIP", repo_type="dataset",
                      allow_patterns=["Articulated_objects/major_appliances/dishwasher/**"],
                      local_dir=os.path.join(PROJECT_ROOT, "assets", "artvip"))


def validate_caches(manifest: dict) -> bool:
    """Recompute config_hash for every restored cache and compare to the archived stamp AND
    the current config.py (a drifted config means the caches need re-extraction)."""
    from dishsim import config  # noqa: PLC0415
    from dishsim.geometry import config_hash  # noqa: PLC0415

    stamps = manifest.get("cache_stamps", {})
    ok = True
    for rel, stamp in sorted(stamps.items()):
        path = os.path.join(PROJECT_ROOT, rel)
        if not os.path.exists(path):
            print(f"[FAIL] missing after extract: {rel}")
            ok = False
            continue
        live = json.load(open(path))
        if live.get("config_hash") != stamp["config_hash"]:
            print(f"[FAIL] {rel}: extracted stamp != archived stamp")
            ok = False
            continue
        # Context reconstruction, in the machine -> object -> scenario -> placement order the
        # hash demands: machine and object come from the cache path (mug caches use the legacy
        # layout, other objects live under .../objects/<object>/<state>). The base placement is
        # not encoded in the path, so every placement the machine defines is tried — the hash
        # covers it, so exactly one can match.
        parts = rel.split(os.sep)
        machine = parts[parts.index("machines") + 1] if "machines" in parts \
            else config.MACHINE_BASELINE_NAME
        active = parts[parts.index("objects") + 1] if "objects" in parts else "mug"
        if active not in config.OBJECTS:
            active = "mug"
        placements = sorted(config.BASE_PLACEMENTS.get(machine, {"front": None}),
                            key=lambda p: p != config.DEFAULT_BASE_PLACEMENT.get(machine, "front"))
        matched = None
        for placement in placements:
            config.apply_machine(machine)  # resets scenario + placement, so it goes first
            config.set_active_object(active)
            config.apply_scenario(stamp.get("scenario") or "both_out")
            config.apply_base_placement(placement)
            if config_hash() == stamp["config_hash"]:
                matched = placement
                break
        if matched is None:
            print(f"[WARN] {rel}: config.py drifted since the archive "
                  f"— re-run scripts/setup/extract_geometry.py for this state")
        else:
            print(f"[OK] {rel} ({stamp['object_name']}, {stamp['scenario']} @ {matched})")
    config.apply_machine(config.MACHINE_BASELINE_NAME)
    config.set_active_object("mug")
    config.apply_scenario("both_out")
    return ok


def main() -> None:
    tarballs = fetch_tarballs()
    manifest = {}
    for p in tarballs:
        m = safe_extract(p)
        if m.get("kind") == "assets" or "assets" in os.path.basename(p):
            manifest = m
    ensure_artvip()
    ok = validate_caches(manifest)
    if not args.skip_tests:
        import subprocess  # noqa: PLC0415

        env = {**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"}
        r = subprocess.run([sys.executable, "-m", "pytest", "tests/", "-q"],
                           cwd=PROJECT_ROOT, env=env)
        ok = ok and r.returncode == 0
    print(f"[RESULT] {'PASS' if ok else 'FAIL'}")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
