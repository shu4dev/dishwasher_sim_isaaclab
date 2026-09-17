# Organized counterparts of the saved initial states

This experiment searches for an organized counterpart of each of the eleven
saved states in `results/initial_states/frigidaire/packing_20260911_seed20260911`.
It preserves each inventory, including every object ID, type, dimension and mass.
The highest source state contains 35 items. Objects may move between racks;
removing dishes, shrinking them or weakening a rule is not a successful result.

The run directory is
`results/initial_states/frigidaire/organized_20260911_seed20260911`.
Its `summary.json` is the authoritative current status. An unresolved inventory
means the bounded search has not demonstrated a reproducible organized state;
it does not establish that such a state is impossible. The source packing is
assessed against the new rules but is not required to pass them.

## Organization rules

Mug openings must point within 45 degrees of downward and bowl openings within
75 degrees. Dinner plates must occupy the lower rack with their planes within
15 degrees of vertical. Every dish must directly contact its assigned rack.
Dish stacks, basket support chains and nesting are excluded. Every dish pair
must have at least 5 mm geometric clearance.

Each mug or bowl opening has 64 deterministic, equal-area Fibonacci-disk sample
points. Rays extend 100 mm outward along its opening normal. At least 52 rays
must avoid other dish visual triangles, meeting the 80% exposure threshold.
These rays describe an explicit geometric exposure proxy; they do not model
water jets, detergent, drainage, spray arms, dispenser access or cleaning quality.

Near-pair distances use authored FCL collision geometry. Distant pairs may use
conservative bounding-box distance lower bounds. Nesting tests use an inscribed
convex cavity proxy. These approximations are part of the recorded method.

Soft preferences favor upper-rack mugs, fewer occupied type rows, shared
orientation families and compact centers. Actual row fragments, normal and handle
alignment and clearance are reported separately. Proposal optimization does not
certify globally optimal geometric clearance, contiguity or visual neatness.

## Finite pose generation and combination search

Candidate positions follow the authored rack tine gaps and channel grids.
Lower-rack plates use the two outer tine-bank pairs, with vertical or ±8-degree
lean families. Mug grid slots use 0-, 15- and 30-degree downward tilt families
with 90- and 270-degree handle yaw. Bowl slots follow tine gaps with 30-, 45-,
60- and 75-degree downward tilt in opposite directions. Refinement introduces
5 mm position offsets and additional mug yaw families. The orientations are
structured finite choices, not samples from a uniform continuous rotation law.

For each fixed horizontal slot and orientation, a conservative vertical FCL
distance search approaches the assigned rack. Distance certificates limit each
downward step; a small positive final gap allows gravity to establish support.
Appliance contact and full rack footprint are checked before the pose becomes
a candidate. Finding this geometric first-contact pose does not show that it
will remain stable during settling.

The combination solver uses one binary selection variable per candidate.
Selected counts must exactly match the source inventory's count of each dish
type. Incompatible candidate pairs satisfy `x_i + x_j <= 1`; failed selections
are excluded within that fixed inventory. Pairwise compatibility is only a
proposal filter: several dishes can collectively block a mouth even when every
pair passes, and joint support must still be checked in physics.

After selecting placements, a Hungarian assignment permutes original object
IDs among placements of the same dish type. Its cost prefers fewer rack
transfers, followed by shorter movement. It preserves every physical identity,
mass and dimension. This assignment is distinct from selecting the combination
of occupied positions and orientations.

The report derives current type counts from the actual adopted candidate list,
rather than inherited generation counters. It reports current graph scope,
screening attempts and passing proposals separately from whole-state acceptance.
Initial and expansion screening catalogs retain separate source hashes; expansion
proposals count toward the active catalog only after the merged catalog is
adopted. `screen_catalog_sources` / `screening_catalog_sources` and
`summary.screening.expansion` retain that provenance when present. A running or
unfinished expansion remains explicit. No candidate count, finite MILP result
or screening result establishes a global capacity bound, exhaustively covers
continuous positions and SO(3), or gives uniform samples of valid loads.

The controller can also precheck each distinct inventory for up to ten seconds
against the current complete geometric graph. These solves use no exclusions
from failed physical trials. Only a proved finite-catalog infeasibility result
skips further allocation; that inventory remains unresolved. Unknown feasibility,
solver timeouts and pending independent replays keep their allocation. Cached
proofs bind both catalog and graph hashes, and a changed domain requires another
precheck. `summary.geometric_prechecks` and graph history retain these results.
The report counts actual precheck solver calls separately from proposal-search
`solver_records`; an unassessed record is not counted as a solver call.

## Physics and fresh reproduction

The pinned Isaac Sim 4.5.0 runtime settles all dishes together with both racks
extended and the door open. Accepted states must survive both loaded rack
retractions, actual door closure, a closed-door rest observation, reopening and
extension of both loaded racks. Dish-to-dish and dish-to-door contacts are
forbidden. Empty-appliance and deliberately blocked-door controls precede search.

Orientation, direct support, forbidden contacts and physical rest are checked
at 120 Hz. Full clearance, nesting and exposure geometry are checked at 10 Hz
and at final snapshots. A five-second accepted observation has at least 50
geometric samples and 601 passing physical windows. Geometric clearance between
those samples is not guaranteed. An early failed observation can requalify
within the twelve-second settling allowance; a passing observation still
requires five uninterrupted seconds.

The inherited physical limits include 5 mm rack endpoint error, 1 mm whole-mesh
containment tolerance, peak penetration below 2 mm, median maximum rest
penetration below 1 mm, and rest spans below 5 mm / 3 degrees / 0.03 m/s over
authored visual vertices. Door endpoint error is at most 0.5 degrees; measured
door speed is capped at 0.65 rad/s with the recorded numerical tolerance.

A successful primary cycle produces a provisional state. A separate fresh
Isaac process must reproduce the full cycle from its measured initial poses
before an accepted counterpart is published. The primary and reproduction
attempts retain separate results, logs and traces. A primary-only success
remains unresolved.

The two-hour allocation is 10 minutes for preparation, 55 minutes for the
highest inventory, 40 minutes for the other inventories and 15 minutes for
reporting. Candidate refinement can consume part of the highest-inventory
allocation. Search settings, seeds, solver results, actual wall time and failures
remain in the run summary and attempt records.

When `summary.screening` is present, search also uses measured single-candidate
screening. One shared Isaac session reuses three dish actors, with only one
candidate active and the others parked away. A passing candidate must complete
the unchanged five-second organization, physical-rest and direct-rack-support
observation. Its measured settled rack-local pose supplies a candidate in a new
catalog. This preconditioning addresses motion during initial settling; it does
not establish joint support, clearance under a full load, or successful rack and
door motion.

`screening_passed` is a proposal-screen result. Every complete inventory still
requires fresh primary and reproduction cycles. Screening uses the same original
two-hour budget, keeps the highest target at 35 items, and changes no acceptance
threshold. Failed full-load attempts remain recorded with their original verdicts.

## Reports and real before/after images

The report lists all eleven inventories, their original organization failures,
their accepted or unresolved counterpart, and actual organization measurements.
Every inventory gets an original view. Only accepted reproduced counterparts get
an after view. The highest and `random_03` accepted counterparts are highlighted
when available. An unresolved case is never shown with an accepted label.

One Isaac RTX session can render all views at 1920 × 1440:

```bash
scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_organized_render.py \
  --batch results/initial_states/frigidaire/organized_20260911_seed20260911/summary.json \
  --out-dir outputs/organized_render_review \
  --headless --enable_cameras
```

Use a fresh output directory. Add `--state-ids highest,random_03` for a small
camera review. Single-state mode accepts `--state`, `--view before|after` and
`--source-state`; an after view requires its original source state. Rendering
uses measured saved poses with dynamics disabled only in the render scene.
It never advances physics or changes a validation verdict. Each item has a
distinct tint, preserved between its before and after views.

Regenerate reports after the summary and canonical `renders/` directory are final:

```bash
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_organized_report.py \
  --out-dir results/initial_states/frigidaire/organized_20260911_seed20260911 --pdf
```

Outputs are `organized_report.md`, `organized_report.html`, the identical
`index.html`, `technical_report.pdf`, `report_preview.png`, `pdf_validation.json`
and `report_audit.json`. PDF export uses the existing offline Playwright browser.
Render evidence records image and source-state hashes, exact saved-versus-rendered
transforms, source and asset hashes, font hashes and runtime version. The report
audits accepted inventories, controls, both primary and reproduction evidence,
organization thresholds, actual door endpoints and every image transform.

The original run is read only. Its preservation audit and hashes are retained
separately from all new search, physics, render and reporting artifacts.
