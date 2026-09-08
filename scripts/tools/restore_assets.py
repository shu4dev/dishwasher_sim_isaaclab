# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Restore archived assets/media on a fresh instance (counterpart of the retired archive_assets.py, git history).

Downloads the tarballs from the public HF dataset (or takes local paths), safe-extracts
them into the project root, verifies every extracted file's sha256 against the tarball's
``MANIFEST.json`` (tarballs cut before 2026-09 carry no digests and are extracted unverified),
and — for the ``assets`` kind — re-downloads the ArtVIP originals (the derived dishwasher
``.usda`` layers reference that tree) and validates every restored geometry cache's
``config_hash`` stamp against the current ``config.py`` before declaring success.

Kinds (``latest.json["files"][kind]`` names each tarball): ``assets`` (caches, props,
derived USDs — the default), ``media`` (recorded evidence), ``models`` (the standalone
Bosch 800 USD asset under ``assets/models/``), ``evidence`` (its validation report, stills
and video under ``assets/evidence/``). Producer: ``archive_assets.py``.

Prerequisites on a fresh box: the runtime container (README setup step 1). The public dataset
downloads without a token (``huggingface-cli login`` is only needed for a private mirror).

    scripts/run_py.sh scripts/tools/restore_assets.py [--repo <id>] [--kinds assets models evidence]
    scripts/run_py.sh scripts/tools/restore_assets.py --local outputs/archive/dishsim_<kind>_<tag>.tar.gz
"""

import argparse
import hashlib
import json
import os
import sys
import tarfile

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

parser = argparse.ArgumentParser(description="Restore archived assets/media from HF (or local tarballs).")
parser.add_argument("--repo", type=str, default="shu4dev/dishsim-assets",
                    help="HF dataset repo id (default: shu4dev/dishsim-assets, public, no token).")
parser.add_argument("--file", type=str, default="latest",
                    help="Assets tarball filename in the repo (default: resolve via latest.json).")
parser.add_argument("--kinds", nargs="+", default=["assets"],
                    choices=["assets", "media", "models", "evidence"],
                    help="Tarball kinds to restore (default: assets).")
parser.add_argument("--with_media", action="store_true",
                    help="Also restore the media tarball (alias for adding 'media' to --kinds).")
parser.add_argument("--local", type=str, nargs="*", default=None,
                    help="Local tarball path(s) instead of downloading.")
parser.add_argument("--skip_tests", action="store_true", help="Skip the pytest validation pass.")
args = parser.parse_args()

if args.with_media and "media" not in args.kinds:
    args.kinds.append("media")

ALLOWED_PREFIXES = ("assets/", "media/", "results/", "MANIFEST.json")
#: What actually gets EXTRACTED from the archive. The shipped tarball packs `results/` (and a
#: media tarball would pack `media/`) wholesale from the robot era; on this branch results and
#: media are LOCAL artifacts (instances, episode records, renders are regenerated here), so
#: only `assets/` restores — extracting the rest would resurrect ~30 MB of retired robot-era
#: outputs on every run. Robot-era evidence that predates this rule lives OUTSIDE the repo
#: roots at /media/corallab-s1/2tbhdd/brianshu/dishsim/robot_era_evidence/. The ``models``
#: and ``evidence`` kinds live under ``assets/`` too, so this single prefix covers them.
EXTRACT_PREFIXES = ("assets/",)


def fetch_tarballs() -> list[str]:
    if args.local:
        return [os.path.abspath(p) for p in args.local]
    from huggingface_hub import hf_hub_download  # noqa: PLC0415

    repo = args.repo
    print(f"[INFO] restoring from https://huggingface.co/datasets/{repo}")
    if args.file == "latest":
        latest = json.load(open(hf_hub_download(repo_id=repo, repo_type="dataset",
                                                filename="latest.json")))
        missing = [k for k in args.kinds if k not in latest["files"]]
        if missing:
            raise SystemExit(f"[FAIL] latest.json has no tarball for kind(s) {missing}; "
                             f"available: {sorted(latest['files'])}")
        names = [latest["files"][k] for k in args.kinds]
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
        members = [m for m in tf.getmembers()
                   if m.name != "MANIFEST.json"
                   and os.path.normpath(m.name).startswith(EXTRACT_PREFIXES)]
        skipped = sum(1 for m in tf.getmembers()
                      if m.name != "MANIFEST.json"
                      and not os.path.normpath(m.name).startswith(EXTRACT_PREFIXES))
        if skipped:
            print(f"[INFO] skipping {skipped} archived members outside {EXTRACT_PREFIXES} "
                  f"(robot-era results/media — this branch regenerates its own)")
        tf.extractall(PROJECT_ROOT, members=members, filter="fully_trusted")
    print(f"[INFO] extracted {os.path.basename(tar_path)} "
          f"({manifest.get('n_files', '?')} files, git {manifest.get('git_sha', '?')})")
    verify_digests(manifest)
    return manifest


def verify_digests(manifest: dict) -> None:
    """Compare every extracted member against the sha256 recorded at archive time."""
    digests = manifest.get("sha256")
    if not digests:
        print("[INFO] no sha256 manifest in this tarball (pre-2026-09 archive) — skipping verify")
        return
    bad = []
    for rel, want in sorted(digests.items()):
        if not os.path.normpath(rel).startswith(EXTRACT_PREFIXES):
            continue
        path = os.path.join(PROJECT_ROOT, rel)
        h = hashlib.sha256()
        try:
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
        except FileNotFoundError:
            bad.append(f"{rel}: missing after extract")
            continue
        if h.hexdigest() != want:
            bad.append(f"{rel}: sha256 mismatch")
    if bad:
        raise SystemExit("[FAIL] manifest verification:\n  " + "\n  ".join(bad))
    print(f"[INFO] sha256 verified for {len(digests)} extracted files")


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
    manifest = None
    for p in tarballs:
        m = safe_extract(p)
        if m.get("kind") == "assets" or "_assets_" in os.path.basename(p):
            manifest = m
    ok = True
    if manifest is not None:  # the cache pack was restored: ArtVIP tree + config_hash gate
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
