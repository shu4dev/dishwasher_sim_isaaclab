"""Canonical locations of the Frigidaire FDPC4221AS asset folder and its prototypes.

``data/assets`` is a copy of the HF dataset shu4dev/dishsim-assets (``hf download``): the twin's USDs, ``parameters.json`` and
``components.json`` live in ``data/assets/frigidaire_fdpc4221as``, the ten tableware prototypes in
``data/assets/frigidaire_prototypes``. Evidence (images, validation reports) stays out of the asset folder.
"""
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
SOURCE_ROOT = REPO_ROOT / "code/frigidaire"
COLLECTION_DIR = REPO_ROOT / "data/assets/frigidaire_fdpc4221as"
ASSET_DIR = COLLECTION_DIR / "usd"
PARAMETERS_PATH = COLLECTION_DIR / "parameters.json"
COMPONENTS_PATH = COLLECTION_DIR / "components.json"
# The ten tableware prototypes (dinner plate ... teaspoon) live in their own asset folder.
PROTOTYPES_DIR = REPO_ROOT / "data/assets/frigidaire_prototypes/usd"
# evidence written by the generator-side tools (data/assets/frigidaire_fdpc4221as/source/), outside the asset
IMAGE_DIR = REPO_ROOT / "data/media/frigidaire_fdpc4221as/images"
VALIDATION_DIR = REPO_ROOT / "data/media/frigidaire_fdpc4221as/validation"
