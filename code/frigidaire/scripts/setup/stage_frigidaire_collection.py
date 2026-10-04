#!/usr/bin/env python3
"""Stage reference material and unchanged release history; generate current rack diagrams.

This host-side preparation does not generate USD or certify Isaac physics. It
never removes original files or replaces the installed collection.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
from dishsim_frigidaire.paths import COLLECTION_DIR, SOURCE_ROOT


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def copy_verified(source, destination):
    """Copy without overwriting a differing archive, then verify every byte hash."""
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        return
    if source.is_dir():
        if source.resolve() in destination.resolve().parents or destination.resolve() in source.resolve().parents:
            raise ValueError("Archive source and destination directories must not overlap")
        for path in sorted(source.rglob("*")):
            if path.is_file():
                copy_verified(path, destination / path.relative_to(source))
        return
    expected = digest(source)
    if destination.exists() and digest(destination) != expected:
        raise ValueError("Refusing to replace a differing archived file: %s" % destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists():
        with tempfile.NamedTemporaryFile(prefix=".frigidaire-copy-", dir=destination.parent, delete=False) as stream:
            temporary = Path(stream.name)
        try:
            shutil.copy2(source, temporary)
            if digest(temporary) != expected:
                raise ValueError("Copy checksum mismatch: %s" % destination)
            if destination.exists() and digest(destination) != expected:
                raise ValueError("Archive destination changed during copy: %s" % destination)
            temporary.replace(destination)
        finally:
            if temporary.exists():
                temporary.unlink()
    if digest(destination) != expected:
        raise ValueError("Copy checksum mismatch: %s" % destination)


def archive_sources():
    models = ROOT / "data/assets/models"
    for version, name in (("v1", "frigidaire_fdpc4221as"), ("v2", "frigidaire_fdpc4221as_v2")):
        bundle = models / name
        # After installation the unversioned directory is the collection, not v1.
        if bundle.is_dir() and not (bundle / "usd").exists():
            yield bundle, Path(version) / "assets"
        for source, target in (
            (ROOT / "data/media" / name, "gallery"),
            (models / (name + ".zip"), "archives/" + name + ".zip"),
            (ROOT / "data/media" / (name + "_gallery.zip"), "archives/" + name + "_gallery.zip"),
            (ROOT / "data/outputs" / name / "release_manifest.json", "release_manifest.json"),
        ):
            if source.exists():
                yield source, Path(version) / target
    delivery = models / "frigidaire_fdpc4221as_delivery.json"
    if delivery.exists():
        yield delivery, Path("v1/delivery.json")
    if (COLLECTION_DIR / "history").is_dir():
        for path in (COLLECTION_DIR / "history").iterdir():
            yield path, Path(path.name)


HISTORY_VERSIONS = {
    "v1": "first release: 32-tine upper rack, 56-tine lower rack, photo-proportioned basket",
    "v2": "the same 32/56-tine geometry with the 67-object full-load evidence",
    "v3": "52-tine upper rack, 72-tine lower rack, basket shifted 27.5 mm right; claims A/A2/B FCL layouts and settle evidence",
    "v4": "first tape-measured build (48/64 tines, 320 x 95 x 130 mm basket) with centreline rim heights: upper 14.6 cm and lower 11.9 cm "
          "outside, basket 12.85 cm; HOTEC loads v5-v10 and the claims dry run v7 ran on it",
}
STALE_RESULTS = [
    "data/build/frigidaire_collection/validation/claims/", "data/build/frigidaire_collection/validation/composition.json",
    "data/build/frigidaire_collection/validation/release_check.json", "data/build/frigidaire_collection/validation/workspace_checks.json",
    "data/build/frigidaire_collection/images/lower_rack/", "data/build/frigidaire_collection/images/upper_rack/",
    "data/results/planner/frigidaire/", "data/results/initial_states/frigidaire/", "data/results/random_poses/frigidaire/",
    "data/results/exposure/frigidaire/", "data/results/hotec/frigidaire/v1/", "data/build/frigidaire_diagnostics/",
]
STALE_RESULTS_BY_VERSION = {"v4": [
    "data/build/frigidaire_collection/validation/composition.json", "data/build/frigidaire_collection/validation/lower_rack_clearance.json",
    "data/build/frigidaire_collection/images/lower_rack/", "data/build/frigidaire_collection/images/upper_rack/",
    "data/build/frigidaire_collection/images/assembly/", "data/results/hotec/frigidaire/v2/ to v10/",
    "data/results/exposure/frigidaire/hotec/", "data/build/frigidaire_diagnostics/claims_v4/ to claims_v7/",
    "data/build/frigidaire_diagnostics/cutlery_candidates.json", "data/build/frigidaire_diagnostics/cutlery_pose_search.json",
], "v5": [
    "data/build/frigidaire_collection/validation/composition.json", "data/build/frigidaire_collection/validation/lower_rack_clearance.json",
    "data/build/frigidaire_collection/images/lower_rack/", "data/build/frigidaire_collection/images/upper_rack/",
    "data/build/frigidaire_collection/images/assembly/", "data/results/hotec/frigidaire/v11/ to v13/",
    "data/results/exposure/frigidaire/hotec/", "data/results/benchmark/frigidaire_hotec/ (families, instances, goals, plans, episodes)",
    "data/build/frigidaire_diagnostics/claims_v8/", "docs/figures/frigidaire_{upper,lower}_rack_cm.png (regenerated)",
]}


def history_readme():
    """History README naming every archived version and the current source revisions."""
    from dishsim_frigidaire.geometry import PARAMETERS
    current = ", ".join("%s `%s`" % (label, PARAMETERS[key].get("geometry_revision", "fdpc4221as_photo_v1"))
                        for label, key in (("upper rack", "upper_rack"), ("lower rack", "lower_rack"),
                                           ("basket", "silverware_basket")))
    lines = ["# Historical Frigidaire releases", ""]
    lines += ["- `%s/`: %s." % (version, what) for version, what in HISTORY_VERSIONS.items()]
    lines += ["", "These are unchanged earlier USD bundles, galleries, reports and archives. Their source "
              "paths and certification hashes describe their original release. None of them validates the "
              "current source revisions (%s). The copy map and checksums are in "
              "../validation/collection_manifest.json." % current, ""]
    return "\n".join(lines)


def archive_current(out_dir, version):
    """Copy the current staged collection (usd, images, validation, README) into history/<version>.

    Run this BEFORE rebuilding the USD: the build rewrites usd/ in place. Nothing is removed.
    """
    out_dir = Path(out_dir).absolute()
    target = out_dir / "history" / version
    if target.exists():
        raise FileExistsError("Refusing to overwrite an existing history version: %s" % target)
    usd = out_dir / "usd"
    if not (usd / "fdpc4221as.usdc").is_file():
        raise FileNotFoundError("No current USD bundle to archive under %s" % usd)
    origins = []
    for source_name, archived_name in (("usd", "assets"), ("images", "gallery"), ("validation", "validation"),
                                       ("README.md", "README.md")):
        source = out_dir / source_name
        if not source.exists():
            continue
        copy_verified(source, target / archived_name)
        origins.append({"original": str(source.relative_to(ROOT)),
                        "archived": str((target / archived_name).relative_to(out_dir))})
    validation = json.loads((usd / "geometry_validation.json").read_text()) if (usd / "geometry_validation.json").is_file() else {}
    parameters = json.loads((usd / "parameters.json").read_text()) if (usd / "parameters.json").is_file() else {}
    files = {str(p.relative_to(target)): digest(p) for p in sorted(target.rglob("*")) if p.is_file()}
    manifest = {"version": version, "source": str(out_dir.relative_to(ROOT)),
                "geometry_revisions": {name: component.get("geometry_revision")
                                       for name, component in validation.get("components", {}).items()},
                "body_positions_m": parameters.get("body_positions_m"),
                "usdc_sha256": validation.get("sha256"),
                "stale": "Every manifest, result, image and check recorded against these hashes describes this "
                         "archived geometry only; none of it validates a later source revision.",
                "stale_results": STALE_RESULTS_BY_VERSION.get(version, STALE_RESULTS),
                "archive_copy_map": origins, "files": files,
                "bytes": sum(p.stat().st_size for p in target.rglob("*") if p.is_file())}
    (target / "archive_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out_dir / "history/README.md").write_text(history_readme())
    manifest_path = out_dir / "validation/collection_manifest.json"
    if manifest_path.is_file():
        data = json.loads(manifest_path.read_text())
        data.setdefault("archive_copy_map", []).extend(origins)
        data["stale_evidence"] = STALE_RESULTS_BY_VERSION.get(version, STALE_RESULTS)
        manifest_path.write_text(json.dumps(data, indent=2) + "\n")
    print("[RESULT] PASS: archived %s -> %s (%d files, %.1f MB)" % (
        out_dir.relative_to(ROOT), target.relative_to(ROOT), len(files), manifest["bytes"] / 1e6))
    return manifest


def stage(out_dir, reference_dir=None):
    out_dir = Path(out_dir).absolute()
    output_path, installed_path = out_dir.resolve(), COLLECTION_DIR.resolve()
    if (output_path == installed_path or installed_path in output_path.parents
            or output_path in installed_path.parents):
        raise ValueError("Stage into a separate directory before replacing the installed collection")
    reference_dir = Path(reference_dir) if reference_dir else next(
        (p for p in (SOURCE_ROOT, COLLECTION_DIR / "references", out_dir / "references")
         if (p / "spec.pdf").is_file()), None)
    if reference_dir is None:
        raise FileNotFoundError("Supply --reference-dir containing spec.pdf and the supplied photographs")
    archives = list(archive_sources())
    inputs = [source for source, _ in archives]
    if reference_dir.resolve() != (out_dir / "references").resolve():
        inputs.append(reference_dir)
    for source in inputs:
        resolved = source.resolve()
        if resolved == output_path or resolved in output_path.parents or output_path in resolved.parents:
            raise ValueError("Staging output must not overlap an archive or reference input: %s" % source)
    for name in ("usd", "images", "validation", "references", "history"):
        (out_dir / name).mkdir(parents=True, exist_ok=True)
    for name in ("loaded", "lower_rack", "overall", "upper_rack", "spec.pdf", "wire_detail.jpeg"):
        copy_verified(reference_dir / name, out_dir / "references" / name)
    basket = "silverware_basket" if (reference_dir / "silverware_basket").is_dir() else "sliverware_basket"
    copy_verified(reference_dir / basket, out_dir / "references/silverware_basket")
    origins = []
    for source, relative in archives:
        target = out_dir / "history" / relative
        copy_verified(source, target)
        origins.append({"original": str(source.relative_to(ROOT)), "archived": str(target.relative_to(out_dir))})
    (out_dir / "history/README.md").write_text(history_readme())
    scripts = SOURCE_ROOT / "scripts/evaluation"
    for rack in ("upper", "lower"):
        subprocess.run([sys.executable, str(scripts / ("frigidaire_%s_rack_preview.py" % rack)),
                        "--out-dir", str(out_dir / "images" / (rack + "_rack"))], check=True)
    subprocess.run([sys.executable, str(scripts / "frigidaire_lower_rack_clearance.py"),
                    "--out", str(out_dir / "validation/lower_rack_clearance.json")], check=True)
    pdf = out_dir / "references/spec.pdf"
    image = out_dir / "images/dimensions.png"
    subprocess.run(["pdftoppm", "-f", "3", "-l", "3", "-r", "180", "-singlefile", "-png",
                    str(pdf), str(image.with_suffix(""))], check=True)
    dimensions = {"source": "references/spec.pdf", "source_pdf_sha256": digest(pdf), "page": 3,
                  "image": "images/dimensions.png", "image_sha256": digest(image),
                  "method": "pdftoppm; full manufacturer dimension page; 180 dpi"}
    (out_dir / "validation/dimensions.json").write_text(json.dumps(dimensions, indent=2) + "\n")
    readme = SOURCE_ROOT / "docs/collection_readme.md"
    shutil.copy2(readme, out_dir / "README.md")
    files = {str(p.relative_to(out_dir)): digest(p)
             for folder in ("references", "history") for p in sorted((out_dir / folder).rglob("*"))
             if p.is_file()}
    manifest = {"status": "STAGED; run collection inspection and assembly evidence before packaging",
                "intended_collection": "data/assets/models/frigidaire_fdpc4221as",
                "current_usd_present": (out_dir / "usd/fdpc4221as.usdc").is_file(),
                "archive_copy_map": origins, "preserved_file_sha256": files,
                "notes": "Historical certification applies only to the archived geometry. Originals were not removed."}
    (out_dir / "validation/collection_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("[RESULT] PASS: references, history and current source diagrams staged at %s" % out_dir)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "data/build/frigidaire_collection")
    parser.add_argument("--reference-dir", type=Path)
    parser.add_argument("--archive-current", metavar="VERSION",
                        help="copy the current usd/images/validation into history/VERSION instead of staging")
    args = parser.parse_args()
    if args.archive_current:
        archive_current(args.out_dir, args.archive_current)
        return
    stage(args.out_dir, args.reference_dir)


if __name__ == "__main__":
    main()
