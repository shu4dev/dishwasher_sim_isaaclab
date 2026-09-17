#!/usr/bin/env python3
"""Check and package the complete Frigidaire asset collection.

Only current authoring, composition, and empty-appliance evidence can authorize
an archive. Historical reports are preserved as files, never reused as gates.
``--check`` reads the collection and prints every missing or stale gate without
creating directories, changing reports, or writing an archive.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
from dishsim_frigidaire.paths import COLLECTION_DIR, REPO_ROOT

USD_SUFFIXES = {".usd", ".usda", ".usdc"}
COMPONENTS = {"Cabinet", "Door", "UpperRack", "LowerRack", "SilverwareBasket"}
COMPOSITION_CHECKS = {
    "dependencies_resolve", "relative_references", "stage_conventions",
    "components_present", "polished_racks", "basket_position",
    "relocated_dependencies_resolve",
}
PACKAGE_SOURCE = Path("frigidaire/src/dishsim_frigidaire")
INSPECT_SOURCE = "frigidaire/scripts/evaluation/frigidaire_collection_inspect.py"
EVIDENCE_SOURCE = "frigidaire/scripts/evaluation/frigidaire_asset_evidence.py"


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def read(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def local_file(directory, name):
    """Resolve a report's relative filename within its declared directory."""
    if not isinstance(name, str) or not name:
        raise ValueError(f"Invalid relative filename: {name!r}")
    relative = Path(name)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"Invalid relative filename: {name!r}")
    path = Path(directory) / relative
    try:
        path.resolve().relative_to(Path(directory).resolve())
    except ValueError:
        raise ValueError(f"File escapes its collection directory: {name}") from None
    if not path.is_file():
        raise ValueError(f"Missing file: {path}")
    return path


def usd_hashes(directory, suffixes=USD_SUFFIXES):
    directory = Path(directory)
    return {path.relative_to(directory).as_posix(): digest(local_file(directory, path.relative_to(directory).as_posix()))
            for path in sorted(directory.rglob("*"))
            if path.is_file() and path.suffix in suffixes}


def verify_hashes(actual, expected, label):
    if not isinstance(expected, dict) or not expected:
        raise ValueError(f"Missing {label} hash map")
    if actual != expected:
        missing = sorted(set(expected) - set(actual))
        added = sorted(set(actual) - set(expected))
        changed = sorted(name for name in set(actual) & set(expected) if actual[name] != expected[name])
        raise ValueError(f"Stale {label}: missing={missing}, added={added}, changed={changed}")


def verify_sources(report, source_root, required):
    recorded = report.get("source_hashes")
    if not isinstance(recorded, dict) or not set(required).issubset(recorded):
        raise ValueError("Missing required current source hashes")
    for name, expected in recorded.items():
        if digest(local_file(source_root, name)) != expected:
            raise ValueError(f"Stale executable source: {name}")


def verify_report(path, assets, source_root, required_sources, assembly=False):
    report = read(path)
    if report.get("result") != "PASS":
        raise ValueError(f"Current passing report required: {path} (result={report.get('result', 'missing')})")
    if assembly and report.get("scope") != "assembly":
        raise ValueError(f"Current assembly-only evidence required: {path}")
    verify_hashes(assets, report.get("asset_hashes"), "current USD geometry")
    verify_sources(report, source_root, required_sources)
    return report


def check_collection(collection_dir, source_root=REPO_ROOT):
    """Return all release gate results without writing anything."""
    collection = Path(collection_dir)
    source_root = Path(source_root)
    usd = collection / "usd"
    assembly = collection / "images/assembly"
    checks, errors = {}, []

    def check(name, operation):
        try:
            operation()
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
            checks[name] = False
            errors.append({"check": name, "message": str(error)})
        else:
            checks[name] = True

    def layout():
        for name in ("README.md", "usd/fdpc4221as.usdc", "usd/example_scene.usda", "usd/parameters.json",
                     "usd/cabinet.usdc", "usd/door.usdc", "usd/upper_rack.usdc",
                     "usd/lower_rack.usdc", "usd/silverware_basket.usdc", "references/spec.pdf"):
            local_file(collection, name)
        if not any(path.is_file() and path.suffix.lower() in {".png", ".jpg", ".jpeg", ".webp"}
                   for path in (collection / "references").rglob("*")):
            raise ValueError("Missing supplied reference photographs")
        for version in ("v1", "v2"):
            if not any(path.is_file() for path in (collection / "history" / version).rglob("*")):
                raise ValueError(f"Missing preserved history/{version} files")
        if any(usd.glob("full_load*")):
            raise ValueError("Historical full_load files belong in history/, outside the current usd/ bundle")
        for path in collection.rglob("*"):
            if path.is_symlink():
                raise ValueError(f"Collection must contain portable files, not symlinks: {path}")

    def authoring():
        report = read(usd / "geometry_validation.json")
        if not str(report.get("result", "")).startswith("PASS"):
            raise ValueError("Current USD authoring report has not passed")
        if not COMPONENTS.issubset(report.get("components", {})):
            raise ValueError("Current authoring report is missing appliance components")
        verify_hashes(usd_hashes(usd, {".usdc"}), report.get("sha256"), "authored USD geometry")

    def composition():
        sources = {INSPECT_SOURCE, str(PACKAGE_SOURCE / "asset.py"), str(PACKAGE_SOURCE / "geometry.py")}
        report = verify_report(collection / "validation/composition.json", usd_hashes(usd), source_root, sources)
        if report.get("parameters_sha256") != digest(local_file(usd, "parameters.json")):
            raise ValueError("Stale composition parameter metadata")
        missing = sorted(name for name in COMPOSITION_CHECKS if report.get("checks", {}).get(name) is not True)
        if missing:
            raise ValueError(f"Composition checks have not passed: {missing}")

    required_evidence_sources = {
        path.relative_to(source_root).as_posix()
        for path in (source_root / PACKAGE_SOURCE).glob("*")
        if path.is_file() and path.suffix in {".py", ".json"}
    } | {EVIDENCE_SOURCE}

    def evidence(kind):
        report = verify_report(assembly / f"{kind}.json", usd_hashes(usd), source_root,
                               required_evidence_sources, assembly=True)
        if kind == "renders":
            images = report.get("images")
            if not isinstance(images, list) or not images:
                raise ValueError("Assembly rendering report has no images")
            sheet = report.get("contact_sheet")
            if not isinstance(sheet, dict):
                raise ValueError("Assembly rendering report has no contact sheet")
            for item in [*images, sheet]:
                if digest(local_file(assembly, item["file"])) != item.get("sha256"):
                    raise ValueError(f"Changed or unhashed assembly image: {item.get('file')}")
            local_file(assembly, "index.html")
        if kind == "evidence":
            for child in ("physics", "renders", "contacts"):
                if report.get("reports", {}).get(child) != read(assembly / f"{child}.json"):
                    raise ValueError(f"Combined evidence is stale: {child}.json changed")

    def rack_images(rack):
        directory = collection / "images" / rack
        report = read(directory / "measurements.json")
        if report.get("source_sha256") != digest(local_file(source_root, str(PACKAGE_SOURCE / "geometry.py"))):
            raise ValueError(f"Stale {rack} source geometry measurements")
        artifacts = report.get("artifacts")
        required = {f"{rack}_overhead.png", f"{rack}_overhead.svg", f"{rack}_oblique.png"}
        if not isinstance(artifacts, list) or not required.issubset(artifacts):
            raise ValueError(f"Missing {rack} layout artifacts")
        verify_hashes({name: digest(local_file(directory, name)) for name in artifacts},
                      report.get("artifact_sha256"), f"{rack} layout images")

    def dimensions():
        report = read(collection / "validation/dimensions.json")
        if report.get("page") != 3:
            raise ValueError("Dimension drawing must come from specification page 3")
        if report.get("source_pdf_sha256") != digest(local_file(collection, "references/spec.pdf")):
            raise ValueError("Stale dimension drawing specification")
        if report.get("image_sha256") != digest(local_file(collection, "images/dimensions.png")):
            raise ValueError("Changed or unhashed dimension image")

    check("collection_layout", layout)
    check("usd_authoring", authoring)
    check("usd_composition", composition)
    for kind in ("physics", "renders", "contacts", "evidence"):
        check(f"assembly_{kind}", lambda kind=kind: evidence(kind))
    for rack in ("upper_rack", "lower_rack"):
        check(f"{rack}_images", lambda rack=rack: rack_images(rack))
    check("dimension_image", dimensions)
    return {"result": "PASS" if all(checks.values()) else "FAIL", "scope": "assembly_collection",
            "collection_dir": str(collection), "checks": checks, "errors": errors,
            "loading_validation": "Not requested; historical load evidence applies only to archived geometry."}


def package_collection(collection_dir, output, source_root=REPO_ROOT):
    """Create one portable archive only after current release gates pass."""
    collection, output = Path(collection_dir), Path(output)
    report = check_collection(collection, source_root)
    if report["result"] != "PASS":
        raise ValueError("Collection release gates failed:\n" + "\n".join(item["message"] for item in report["errors"]))
    try:
        output.resolve().relative_to(collection.resolve())
    except ValueError:
        pass
    else:
        raise ValueError("Archive output must be outside the collection directory")
    files = sorted(path for path in collection.rglob("*") if path.is_file())
    before = {path.relative_to(collection).as_posix(): digest(path) for path in files}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".frigidaire-package-", dir=output.parent) as temporary:
        staged = Path(temporary) / "collection.zip"
        with zipfile.ZipFile(staged, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in files:
                archive.write(path, (Path(collection.name) / path.relative_to(collection)).as_posix())
        with zipfile.ZipFile(staged) as archive:
            if archive.testzip() is not None:
                raise ValueError("Archive integrity check failed")
        after = {path.relative_to(collection).as_posix(): digest(path)
                 for path in sorted(collection.rglob("*")) if path.is_file()}
        if before != after:
            raise ValueError("Collection changed while packaging; archive was not published")
        staged.replace(output)
    return {**report, "archive": {"file": str(output), "sha256": digest(output),
                                   "bytes": output.stat().st_size, "files": len(files)}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-dir", type=Path, default=COLLECTION_DIR)
    parser.add_argument("--output", type=Path, help="ZIP output; defaults to a sibling <collection>.zip")
    parser.add_argument("--check", action="store_true", help="Read and report all release gates without writing")
    args = parser.parse_args(argv)
    if args.check:
        report = check_collection(args.collection_dir)
    else:
        output = args.output or args.collection_dir.parent / (args.collection_dir.name + ".zip")
        try:
            report = package_collection(args.collection_dir, output)
        except (OSError, ValueError) as error:
            report = {"result": "FAIL", "errors": [{"message": str(error)}]}
    print(json.dumps(report, indent=2), flush=True)
    print(f"[RESULT] {report['result']}: Frigidaire collection {'check' if args.check else 'archive'}", flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
