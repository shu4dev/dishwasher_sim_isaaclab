"""Canonical locations for the Frigidaire source and distributable collection.

Tools accept explicit output paths for staging; these defaults point at the
repository asset collection. Historical bundles live outside ASSET_DIR so they
cannot contribute files to current geometry hashes.
"""
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SOURCE_ROOT = REPO_ROOT / "frigidaire"
COLLECTION_DIR = REPO_ROOT / "assets/models/frigidaire_fdpc4221as"
# The current-source USD build lands in the staged collection; the installed
# COLLECTION_DIR holds only the historical v1 bundle (flat, no usd/).
ASSET_DIR = REPO_ROOT / "build/frigidaire_collection/usd"
IMAGE_DIR = COLLECTION_DIR / "images"
VALIDATION_DIR = COLLECTION_DIR / "validation"
