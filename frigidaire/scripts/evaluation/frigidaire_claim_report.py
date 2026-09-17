"""Compose REPORT.md for the rack-capacity claim test from the JSON verdicts (Kit-free).

Reads, per variant V in {A, B}, ``claim_V_geometry.json`` (FCL layout verdict) and, when
present, ``V_settle/physics.json`` (settle-only) and ``V/physics.json`` (full cycle) written
by ``frigidaire_full_load_evidence.py``; lists the rack stills copied to ``--media-dir``.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_claim_report.py
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire.tableware import CATALOG  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--claims-dir", type=Path, default=REPO_ROOT / "build/frigidaire_collection/validation/claims")
parser.add_argument("--media-dir", type=Path, default=REPO_ROOT / "media/frigidaire_fdpc4221as/claims")
args = parser.parse_args()

CLAIMS = {
    "upper_tumblers": "Upper rack: 6 tumblers per side in the outer channels (12)",
    "upper_saucers": "Upper rack: 2 saucers at the very front of the centre gap",
    "upper_bowls": "Upper rack: as many bowls as fit in the centre gap behind the saucers (measured)",
    "lower_front": "Lower rack front row",
    "lower_rear": "Lower rack rear row: 6 dinner + 5 salad",
    "lower_bowls": "Lower rack: 2 bowls front-right ahead of the basket",
    "basket": "Basket: 4 each fork, knife, tablespoon, teaspoon",
}
FRONT = {"A": "6 dinner + 2 salad + 2 bowls", "B": "6 dinner + 5 salad"}


def load(path):
    return json.loads(path.read_text()) if path.is_file() else None


def physics_summary(report, label):
    if report is None:
        return f"- {label}: not run"
    result = report.get("result")
    line = f"- {label}: **{result}**"
    if report.get("wall_seconds"):
        line += f" ({report['wall_seconds']:.0f} s wall, {report.get('validation_scope', 'full cycle')})"
    failures = {}
    for oid, obj in report.get("objects", {}).items():
        for hold, reasons in obj.get("failures", {}).items():
            if reasons:
                failures.setdefault(oid, []).append(f"{hold}: {'; '.join(reasons)}")
    if failures:
        line += "\n  - faulted dishes: " + "; ".join(f"`{oid}` ({', '.join(r)})" for oid, r in failures.items())
    if report.get("exception"):
        line += f"\n  - exception: `{report['exception']}`"
    if report.get("validated_counts"):
        line += f"\n  - validated counts by rack: `{json.dumps(report['validated_counts'].get('by_rack', {}))}`"
    return line


def rack_counts(by_rack):
    return ", ".join(f"{rack}: " + ", ".join(f"{k} {v}" for k, v in sorted(kinds.items()))
                     for rack, kinds in by_rack.items())


lines = ["# Frigidaire FDPC4221AS rack-capacity claim test", "",
         "Geometry under test: current source (52-tine upper, 72-tine lower, basket +27.5 mm), "
         "built into `build/frigidaire_collection/usd/`. Dishes: the ten tableware prototypes at "
         "their modeling sizes (dinner 260, salad 205, saucer 150, bowl 140x65, tumbler 80x160 mm, "
         "stated defaults, not measurements). Verdict per dish: collision-free pose (FCL), then "
         "Isaac Sim 4.5 settle + door/rack travel cycle (37 gates, CPU PhysX, CCD on).", ""]
labels = sorted(p.name[len("claim_"):-len("_geometry.json")]
                for p in args.claims_dir.glob("claim_*_geometry.json"))
for label in labels:
    variant = label[0]
    geometry = load(args.claims_dir / f"claim_{label}_geometry.json")
    settle = load(args.claims_dir / f"{label}_settle/physics.json")
    full = load(args.claims_dir / f"{label}/physics.json")
    lines += [f"## Attempt {label}: variant {variant} (front row {FRONT[variant]})", ""]
    if geometry is None:
        lines += ["- geometry: not generated", ""]
        continue
    claimed = geometry["claimed"]
    lines += [f"- geometry (FCL): **{geometry['result']}**, {geometry['object_count']} objects placed; "
              f"{rack_counts(geometry['by_rack'])}",
              f"- upper-gap bowl fill (measured): **{geometry['upper_bowl_fill']}**"]
    if geometry["unplaced"]:
        lines += ["- unplaced (claim FAILED at geometry): " + ", ".join(
            f"`{u['item']}` ({u['tried']} variants tried)" for u in geometry["unplaced"])]
    lines += [physics_summary(settle, "Isaac settle-only (12 s hold)"),
              physics_summary(full, "Isaac full cycle (settle, open, extend, retract, close)"), ""]
    if geometry.get("banned"):
        lines += ["- poses banned after an earlier physics run: " + ", ".join(f"`{k}`" for k in geometry["banned"])]
    stills = sorted((args.media_dir / label).glob("*.png")) if (args.media_dir / label).is_dir() else []
    if stills:
        lines += ["Stills: " + ", ".join(f"`{p.relative_to(REPO_ROOT)}`" for p in stills), ""]
lines += ["## Claim-by-claim", "",
          "Per dish kind: placed/requested, then that kind's own outcome across every hold of the "
          "best full-cycle run (a run's overall verdict fails on any single object).", "",
          "| claim | requested | A | B |", "|---|---|---|---|"]
def best(variant):
    """Latest attempt of a variant with a full-cycle PASS, else its latest attempt."""
    attempts = [l for l in labels if l[0] == variant]
    passed = [l for l in attempts if (load(args.claims_dir / f"{l}/physics.json") or {}).get("result") == "PASS"]
    pick = (passed or attempts or [None])[-1]
    if pick is None:
        return None, None
    return load(args.claims_dir / f"claim_{pick}_geometry.json"), load(args.claims_dir / f"{pick}/physics.json")


(ga, fa), (gb, fb) = best("A"), best("B")


def cell(g, f, rack, kind, want):
    """Placed/requested, then the per-dish physics outcome for THIS kind on THIS rack."""
    if g is None:
        return "-"
    got = g["by_rack"].get(rack, {}).get(kind, 0)
    if f is None:
        return f"{got}/{want}"
    kinds = {kind} if kind != "fork" else {"fork", "knife", "tablespoon", "teaspoon"}
    objs = [o for o in f.get("objects", {}).values() if o["kind"] in kinds and o["rack"] == rack]
    faulted = sum(any(o["failures"].values()) for o in objs)
    phys = "all settled through the cycle" if objs and not faulted else f"{faulted} of {len(objs)} faulted"
    return f"{got}/{want} / {phys}"


rows = [("tumbler, upper", "UpperRack", "tumbler", 12, 12), ("saucer, upper", "UpperRack", "saucer", 2, 2),
        ("bowl, upper (measured)", "UpperRack", "bowl", None, None),
        ("dinner plate, lower", "LowerRack", "dinner_plate", 12, 12), ("salad plate, lower", "LowerRack", "salad_plate", 7, 10),
        ("bowl, lower", "LowerRack", "bowl", 2, 0),
        ("fork / knife / tablespoon / teaspoon, basket", "SilverwareBasket", "fork", 4, 4)]
for label, rack, kind, wa, wb in rows:
    a = cell(ga, fa, rack, kind, wa if wa is not None else "max")
    b = cell(gb, fb, rack, kind, wb if wb is not None else "max")
    lines.append(f"| {label} | {wa if wa is not None else 'as many as fit'} (A), {wb if wb is not None else 'as many as fit'} (B) | {a} | {b} |")
lines += ["", "## Caveats", "",
          "- Dish sizes are the catalog's stated modeling defaults, not measured dishes; a claim that "
          "fails or passes by a few millimetres is inside that uncertainty.",
          "- Rack outer dimensions and tine counts are the user's corrections; only the exterior "
          "envelope, the 8 in upper clearance and the 14 place-setting rating come from the "
          "manufacturer sheet.",
          "- Bowls are placed mouth-down over the tines (rim on the floor rib, base against the row "
          "behind); consecutive bowls must keep disjoint depth slabs along the mouth axis, so no "
          "nested bowls are counted.",
          "- Mugs are not part of any claim and were not loaded.", ""]
out = args.claims_dir / "REPORT.md"
out.write_text("\n".join(lines))
print(f"[INFO] wrote {out}")
print("[RESULT] PASS: claim report composed")
