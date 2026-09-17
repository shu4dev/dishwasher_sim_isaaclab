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

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "frigidaire/src"))
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
    models = ROOT / "assets/models"
    for version, name in (("v1", "frigidaire_fdpc4221as"), ("v2", "frigidaire_fdpc4221as_v2")):
        bundle = models / name
        # After installation the unversioned directory is the collection, not v1.
        if bundle.is_dir() and not (bundle / "usd").exists():
            yield bundle, Path(version) / "assets"
        for source, target in (
            (ROOT / "media" / name, "gallery"),
            (models / (name + ".zip"), "archives/" + name + ".zip"),
            (ROOT / "media" / (name + "_gallery.zip"), "archives/" + name + "_gallery.zip"),
            (ROOT / "outputs" / name / "release_manifest.json", "release_manifest.json"),
        ):
            if source.exists():
                yield source, Path(version) / target
    delivery = models / "frigidaire_fdpc4221as_delivery.json"
    if delivery.exists():
        yield delivery, Path("v1/delivery.json")
    if (COLLECTION_DIR / "history").is_dir():
        for path in (COLLECTION_DIR / "history").iterdir():
            yield path, Path(path.name)


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
    (out_dir / "history/README.md").write_text(
        "# Historical Frigidaire releases\n\n"
        "v1 and v2 contain unchanged earlier USD bundles, galleries, reports and archives. "
        "Their source paths and certification hashes describe their original release. "
        "They do not validate the current 52/72-tine racks or corrected basket pose. "
        "The copy map and checksums are in ../validation/collection_manifest.json.\n")
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
                "intended_collection": "assets/models/frigidaire_fdpc4221as",
                "current_usd_present": (out_dir / "usd/fdpc4221as.usdc").is_file(),
                "archive_copy_map": origins, "preserved_file_sha256": files,
                "notes": "Historical certification applies only to the archived geometry. Originals were not removed."}
    (out_dir / "validation/collection_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("[RESULT] PASS: references, history and current source diagrams staged at %s" % out_dir)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "build/frigidaire_collection")
    parser.add_argument("--reference-dir", type=Path)
    args = parser.parse_args()
    stage(args.out_dir, args.reference_dir)


if __name__ == "__main__":
    main()
