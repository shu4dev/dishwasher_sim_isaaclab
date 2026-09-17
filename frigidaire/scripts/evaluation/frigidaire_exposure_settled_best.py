# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Turn a settled exposure-search attempt into a state file and re-score it (Kit-free)."""
import json
import sys
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire.random_poses import relative_pose  # noqa: E402
from dishsim_frigidaire import exposure as E  # noqa: E402

folder = Path(sys.argv[1] if len(sys.argv) > 1 else REPO_ROOT / "results/exposure/frigidaire/search/settle_best")
manifest = json.loads((folder / "manifest.json").read_text())
result = json.loads((folder / "result.json").read_text())
print(f"[OK] outcome: {result['outcome']} {result.get('reason', '')}")
if result["outcome"] != "accepted":
    print("[RESULT] FAIL"); sys.exit(0)
objects = []
for original in manifest["objects"]:
    obj = deepcopy(original)
    measured = result["initial_snapshot"]["poses"][obj["object_id"]]
    rack = result["initial_snapshot"]["poses"][obj["rack"]]
    p, q = relative_pose(measured["position_m"], measured["quaternion_xyzw"], rack["position_m"], rack["quaternion_xyzw"])
    obj.update(candidate_pose_world=obj["pose_world"], candidate_rack_local_pose=obj["rack_local_pose"],
               pose_world=measured, rack_local_pose={"position_m": p.tolist(), "quaternion_xyzw": q.tolist()})
    objects.append(obj)
state = dict(manifest, objects=objects, accepted=True, status="settled_once_no_reproduction",
             initial_snapshot=result["initial_snapshot"], validation=result, state_id="random_06_best")
(folder / "state.json").write_text(json.dumps(state, indent=1) + "\n")
settled = E.score_state(folder / "state.json")
organized = E.score_state(REPO_ROOT / "results/initial_states/frigidaire/organized_20260911_seed20260911/states/random_06.json")
moved = max(float(abs(__import__("numpy").asarray(o["pose_world"]["position_m"]) -
                      __import__("numpy").asarray(o["candidate_pose_world"]["position_m"])).max()) for o in objects)
summary = {"proposal_unsettled": manifest["exposure_score_unsettled"], "proposal_settled": settled["score"],
           "settled_worst": settled["worst"], "settled_feasible": settled["feasible"],
           "settled_pooling": settled["pooling_count"], "organized_settled": organized["score"],
           "max_settle_displacement_m": moved}
(folder / "scores.json").write_text(json.dumps(summary, indent=1) + "\n")
for k, v in summary.items():
    print(f"[OK] {k}: {v:.3f}" if isinstance(v, float) else f"[OK] {k}: {v}")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def draw_top(ax, r, title):
    s = r["samples"]; order = np.argsort(s["points"][:, 2])
    sc = ax.scatter(s["points"][order, 0], s["points"][order, 1], c=s["exposure"][order], cmap="viridis", vmin=0, vmax=1, s=3)
    for o in r["objects"]:
        if o.get("pools"):
            ax.add_patch(Circle(o["position_m"][:2], .05, fill=False, color="red", lw=1.2))
    ax.set_xlim(-.3, .3); ax.set_ylim(-.3, .3); ax.set_aspect("equal"); ax.set_title(title, fontsize=9)
    return sc


fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
draw_top(axes[0], organized, f"settled organized random_06: score {organized['score']:.3f}")
sc = draw_top(axes[1], settled, f"best proposal, SETTLED in Isaac: score {settled['score']:.3f} "
                                f"(was {manifest['exposure_score_unsettled']:.3f} unsettled)")
fig.colorbar(sc, ax=axes, shrink=.8, label="exposure")
fig.suptitle("Same 18 objects; both arrangements physically settled", fontsize=10)
fig.savefig(folder / "settled_best.png", dpi=150); plt.close(fig)
print("[RESULT] PASS")
