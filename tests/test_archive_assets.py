# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free tests for scripts/tools/archive_assets.py (tarball selection, latest.json merge,
sync status). No network: the remote side is passed in as plain dicts."""

import importlib.util
import os
import sys
import time

import pytest

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "scripts", "tools", "archive_assets.py")
_spec = importlib.util.spec_from_file_location("archive_assets", _PATH)
aa = importlib.util.module_from_spec(_spec)
sys.modules["archive_assets"] = aa
_spec.loader.exec_module(aa)


def _touch(path: str, data: bytes = b"x") -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)
    return path


def test_models_kind_excludes_zip_duplicates(tmp_path, monkeypatch):
    monkeypatch.setattr(aa, "PROJECT_ROOT", str(tmp_path))
    root = tmp_path / "assets" / "models"
    _touch(str(root / "bosch800" / "bosch800.usdc"))
    _touch(str(root / "frigidaire_v2" / "full_load_manifest.json"))
    _touch(str(root / "frigidaire_v2.zip"))
    _touch(str(root / "frigidaire_v2" / "nested.zip"))
    _touch(str(root / ".DS_Store"))
    arcs = sorted(arc for _, arc in aa.iter_members("models"))
    assert arcs == ["assets/models/bosch800/bosch800.usdc",
                    "assets/models/frigidaire_v2/full_load_manifest.json"]
    # the exclusion is kind-specific: evidence keeps zips (previous_asset.zip is shipped)
    _touch(str(tmp_path / "assets" / "evidence" / "bosch800" / "previous_asset.zip"))
    assert [arc for _, arc in aa.iter_members("evidence")] == [
        "assets/evidence/bosch800/previous_asset.zip"]


def test_merge_latest_keeps_untouched_kinds_and_bumps_tag():
    remote = {"tag": "20260814_657fe90", "files": {"assets": "dishsim_assets_20260814_657fe90.tar.gz",
                                                   "media": "dishsim_media_20260810_a9591f0.tar.gz"}}
    built = {"models": "dishsim_models_20260910_abc1234.tar.gz",
             "evidence": "dishsim_evidence_20260910_abc1234.tar.gz"}
    latest = aa.merge_latest(remote, built, "20260910_abc1234")
    assert latest["tag"] == "20260910_abc1234"
    assert latest["files"] == {**remote["files"], **built}
    assert remote["files"] == {"assets": "dishsim_assets_20260814_657fe90.tar.gz",
                               "media": "dishsim_media_20260810_a9591f0.tar.gz"}  # not mutated


def test_merge_latest_refuses_to_blank_remote_pointer_map():
    built = {"models": "dishsim_models_20260910_abc1234.tar.gz"}
    with pytest.raises(SystemExit, match="refusing"):
        aa.merge_latest(None, built, "20260910_abc1234")
    # explicit opt-in for a brand-new repo
    assert aa.merge_latest(None, built, "t", fresh=True) == {"files": built, "tag": "t"}


def test_diff_status_states(tmp_path):
    local = str(tmp_path / "archive")
    os.makedirs(local)
    ok = _touch(os.path.join(local, "dishsim_assets_t.tar.gz"), b"assets-bytes")
    bad = _touch(os.path.join(local, "dishsim_models_t.tar.gz"), b"models-bytes")
    stale = _touch(os.path.join(local, "dishsim_evidence_t.tar.gz"), b"evidence-bytes")
    old = time.time() - 3600
    for p in (ok, bad, stale):
        os.utime(p, (old, old))
    remote_files = {"assets": "dishsim_assets_t.tar.gz", "models": "dishsim_models_t.tar.gz",
                    "evidence": "dishsim_evidence_t.tar.gz", "media": "dishsim_media_t.tar.gz"}
    remote_shas = {"dishsim_assets_t.tar.gz": aa.sha256_of(ok),
                   "dishsim_models_t.tar.gz": "0" * 64,
                   "dishsim_evidence_t.tar.gz": aa.sha256_of(stale),
                   "dishsim_media_t.tar.gz": "1" * 64}
    # evidence's source tree is newer than its tarball; everything else is older
    newest = {"evidence": time.time()}.get
    rows = {r["kind"]: r for r in aa.diff_status(
        remote_files, remote_shas, local, {"assets", "models", "evidence", "frigidaire_only"},
        newest_mtime=lambda k: newest(k, 0.0))}
    assert rows["assets"]["sha"] == "ok" and rows["assets"]["build"] == "fresh" and rows["assets"]["ok"]
    assert rows["models"]["sha"] == "mismatch" and not rows["models"]["ok"]
    assert rows["evidence"]["sha"] == "ok" and rows["evidence"]["build"] == "stale"
    assert not rows["evidence"]["ok"]
    assert rows["media"]["sha"] == "missing-local"
    assert rows["frigidaire_only"]["sha"] == "no-remote"


def test_diff_status_all_synced(tmp_path):
    local = str(tmp_path / "archive")
    p = _touch(os.path.join(local, "dishsim_assets_t.tar.gz"), b"bytes")
    rows = aa.diff_status({"assets": "dishsim_assets_t.tar.gz"},
                          {"dishsim_assets_t.tar.gz": aa.sha256_of(p)}, local, {"assets"},
                          newest_mtime=lambda k: 0.0)
    assert [r["ok"] for r in rows] == [True]


def test_local_tarball_kinds(tmp_path):
    for n in ("dishsim_assets_20260910_abc.tar.gz", "dishsim_models_20260910_abc.tar.gz",
              "dishsim_bogus_1.tar.gz", "hf_README.md"):
        _touch(str(tmp_path / n))
    assert aa.local_tarball_kinds(str(tmp_path)) == {"assets", "models"}


def test_cli_requires_kinds_or_status():
    with pytest.raises(SystemExit, match="--kinds"):
        aa.main([])
    with pytest.raises(SystemExit, match="--no-build"):
        aa.main(["--kinds", "models", "--no-build"])
