"""Data for the planning-note figures that needs the container (USD): the usable pose catalogue.

Runs Kit-free inside the container:
    scripts/run_py.sh docs/planning_note/figures/extract_figdata.py > out.txt
and prints a base64 npz between NPZ_BEGIN / NPZ_END; make_figures.py (host) decodes it into
build/planning_note/figdata.npz. Per usable catalogue pose (families() + family_masks(), the set every
load-building method draws from): kind, rack, slot and rack-local position.
"""
import base64
import io
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "frigidaire/scripts/experiment"))
import frigidaire_bench as B  # noqa: E402

fam, parts, points = B.families()
masks = B.family_masks(fam, parts, points)
rows = [(c["kind"], c["rack"], c["slot"], *c["position"])
        for kind in B.KINDS for (c, _), ok in zip(fam[kind], masks[kind]) if ok]
out = {"kind": np.array([r[0] for r in rows]), "rack": np.array([r[1] for r in rows]),
       "slot": np.array([r[2] for r in rows]), "xyz": np.array([r[3:] for r in rows], dtype=float),
       "n_catalogue": np.array([len(fam[k]) for k in B.KINDS]), "n_usable": np.array([int(masks[k].sum()) for k in B.KINDS])}
buf = io.BytesIO()
np.savez_compressed(buf, **out)
print(f"[INFO] {len(rows)} usable poses; catalogue {out['n_catalogue'].tolist()}, usable {out['n_usable'].tolist()}",
      file=sys.stderr)
print("NPZ_BEGIN")
print(base64.b64encode(buf.getvalue()).decode())
print("NPZ_END")
