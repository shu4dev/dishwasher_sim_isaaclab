"""stills stage (plans/2026-11-02-sample-schema.md): the sample's initial and goal stills (frigidaire_bench_kit.py
--stills, Kit, cameras on) rendered into <media>/<sample>/initial.png and goal.png; their paths go to metadata.media.
PASS when the kit log passed and both PNGs exist. Needs a certified, scored goal (the goal still's title carries S_ref).
"""
from pathlib import Path

from . import ROOT, rel, run

KIT = "code/sampler/scripts/frigidaire_bench_kit.py"


def stage_stills(state, params):
    if not ((state["metadata"]["provenance"].get("certify") or {}).get("valid")
            and ((state.get("goal") or {}).get("score") or {}).get("S_ref") is not None):
        return state, "FAIL", "no certified, scored goal (run certify and score first)"
    path = Path(params["sample"])
    media = Path(params.get("media") or ROOT / "data/media/bench/v1") / path.stem
    out = Path(params["work"]) / "stills"
    ok, _ = run("kit", [KIT, "--stills", "--sample", rel(path), "--out", rel(out), "--media", rel(media), "--headless",
                        "--enable_cameras"], out / "logs" / "stills.log")
    stills = {k: media / f"{k}.png" for k in ("initial", "goal")}
    got = all(p.is_file() for p in stills.values())
    if got:
        state["metadata"]["media"] = {k: rel(p) for k, p in stills.items()}
    return state, "PASS" if ok and got else "FAIL", "initial and goal stills" + (
        "" if got else "; a still is missing") + ("" if ok else "; kit log not PASS")
