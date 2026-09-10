# Bosch 800 standalone dishwasher asset — SHP78CM5N reference

`assets/models/bosch800/bosch800.usdc` is a standalone, independently authored
approximation of the Bosch 800 Series pocket-handle dishwasher. It includes mesh geometry,
PBR materials, relative texture paths, rigid bodies, collision shapes and driven joints. No
online asset server, ArtVIP assets, robot models, or planning archive is needed to use it.
The binaries (`bosch800.usdc`, the rack USDs, `textures/`) and the asset's evidence under
`assets/evidence/bosch800/` are not in git; only this document is tracked. They come from
the public HF archive as two opt-in tarball kinds — see "Fetch the prebuilt asset" below.

Open the USD directly in Isaac Sim, or reference its default prim `/Bosch800` into another
USD stage. Keep the `textures/` directory beside the asset. Units are metres and kilograms;
Z is up, X is width, and the front faces negative Y. The origin is at the centre of the
appliance's floor footprint. The asset starts closed.

**Scope on this branch.** The asset is separate from the benchmark's own Bosch 800 machine
(`usd_prep.make_bosch800_usd`, re-authored on import into `assets/machines/bosch800/`) and
its cached collision worlds. `--machine bosch800` still selects that procedural machine; the
new geometry needs fresh planning caches and slot calibration before it can back the
benchmark. Until then it ships as a redistributable asset only.

## Model details

- 598 mm wide, 602 mm deep, approximately 860 mm high at the authored leg setting.
- Folded outer case, thin stainless tub, narrow frame and rubber door seal.
- Recessed pocket handle, satin steel fascia with procedural brushed roughness, hidden top
  controls and a display, raised door liner and visible fasteners.
- Detergent dispenser, rinse-aid cap, sump/filter screen, shaped lower PowerControl arm,
  middle-rack spray arm and ceiling spray head.
- Lower rack traced from a product photo of the bottom rack: gray coated wire, tall sparse
  walls, three front rails, separate front/rear tine banks, eight wheels, a broad gray
  grip, and a right-side lattice cutlery basket.
- Middle rack traced from a product photo of the middle rack: stepped floor channels,
  dipped front rail, paired curved bowl supports, left glass-rest loops, one raised slotted
  right shelf, gray RackMatic hardware and an open BOSCH grip.
- The photos superseded the earlier generic rack styling. Unsupported red rack adjustment
  tabs and the duplicated flat cup shelves were removed.
- Third cutlery tray with a dropped centre, fine open lattice and red wing tabs.
- Separate convex collision shapes preserve rack openings for object interaction. Visual
  meshes are grouped by body/material to keep stage overhead manageable.

## Articulation

| Joint | Body | Axis | Limits | Isaac Lab units |
|---|---|---|---|---|
| `door_hinge` | `Door` | +X | 0–90° | radians |
| `lower_slide` | `LowerRack` | Y | −0.53–0 | metres |
| `middle_slide` | `MiddleRack` | Y | −0.46–0 | metres |
| `third_slide` | `ThirdRack` | Y | −0.43–0 | metres |

Negative slide positions extend the racks. Retract all racks before closing the door. The
shell is fixed to the world by `Joints/base_fixed`. When placing the asset away from the
origin, update that joint's world-side `physics:localPos0` and `physics:localRot0` to the
desired base pose. The articulation root API is on `Cabinet`. Five link masses sum to
43.5 kg; link mass distribution, inertias, motor gains and travel limits are engineering
estimates.

Self-collision is disabled for the constrained appliance links. Door/rack interlocking,
foldable tines, adjustable RackMatic height, tray wing articulation, spray-arm rotation,
detergent dispensing, fluid flow and washing/drying physics are not simulated. Details such
as spray heads and fasteners are visual geometry; collision uses shell/door boxes and rack
capsules/boxes. This is a visual and manipulation asset, not manufacturer CAD or a
metrologically verified digital twin.

`lower_rack.usdc` and `middle_rack.usdc` are standalone rigid-body exports, using the same
textures. Their link origin is the rack wire-plane reference; spawn a loose lower rack at
least 30 mm above a floor so its wheels do not initially penetrate it.

A revision of the lower rack based on the four local `bottom_rack` photographs is
staged separately. See [lower rack polish](../frigidaire/docs/lower_rack_polish.md) for its geometry,
installation and Isaac Sim capture scripts, and current validation status. It has
not yet replaced the prebuilt binaries described here.

`LowerRack/Manipulation` and `MiddleRack/Manipulation` provide named local handle and
candidate dish sites. These are starting hints, not a complete slot library or a
collision-free grasp guarantee. Use the actual dish size and gripper geometry in the
planner. Handles and basket ribs have collision geometry; no invisible solid box fills
either handle pocket, the cutlery cells, or rack floors. Contacts have explicit friction
(static 0.55, dynamic 0.40) and zero restitution. The cutlery basket and adjustment
mechanisms are attached to their rack body.

For vertical lower-rack access, retract the middle/third racks and extend the lower rack.
For middle-rack access, retract the third rack and extend the middle rack to −0.46 m.
Extending all racks is a visual overview, not a valid unobstructed robot approach to every
level. The 62 mm test cup uses the broad left-centre channel; the far-left channel's fixed
glass-rest loops obstruct a straight lift of that cup (the contact test caught this). The
authored motor drives support scripted rack positioning; reduce or disable their gains when
a robot should physically pull a passive rack.

## Fetch the prebuilt asset

From the repository root, inside the runtime container (README §2.2); the dataset is public
and needs no token. Each tarball carries a `MANIFEST.json` with a sha256 per file, and
`restore_assets.py` verifies every extracted file against it:

```bash
# standalone machine assets (~6 MB, seconds): this Bosch 800 asset plus the preliminary
# Frigidaire FDPC4221AS revisions under assets/models/frigidaire_fdpc4221as{,_v2}/
scripts/run_py.sh scripts/tools/restore_assets.py --kinds models
# add its validation evidence (~74 MB: stills, video, validation.json, previous-asset zip)
scripts/run_py.sh scripts/tools/restore_assets.py --kinds models evidence
```

Without the repo (any machine with `curl` + `python3`); the archive extracts to
`assets/models/bosch800/` relative to the current directory:

```bash
B=https://huggingface.co/datasets/shu4dev/dishsim-assets/resolve/main
N=$(curl -sL $B/latest.json | python3 -c 'import json,sys;print(json.load(sys.stdin)["files"]["models"])')
curl -L -o $N $B/$N && tar xzf $N
```

Re-cutting the tarballs after an asset revision (producer side, Kit-free). Build first, inspect
the tarballs (`tar -tzf`; `*.zip` duplicates under `assets/models/` are excluded by design), then
upload exactly those files with `--no-build --tag`:

```bash
scripts/run_py.sh scripts/tools/archive_assets.py --status                          # what differs?
scripts/run_py.sh scripts/tools/archive_assets.py --kinds models evidence            # build only
HF_TOKEN=<write token> scripts/run_py.sh scripts/tools/archive_assets.py \
    --kinds models evidence --upload --no-build --tag <date>_<gitsha> \
    --card outputs/archive/hf_README.md                                             # + publish
scripts/run_py.sh scripts/tools/archive_assets.py --status                          # -> SYNCED
```

`run_py.sh` forwards `HF_TOKEN` into the container only when it is set in that shell; the token
is never written anywhere. The upload merges into the remote `latest.json`, so kinds not in
`--kinds` keep resolving unchanged — and it aborts rather than blanking the pointer map when
the remote file cannot be read (`--fresh` is the opt-in for an empty repo).

## Runs under this repo's stack (corallab, Isaac Sim 4.5.0 / Isaac Lab 2.1.1)

`scripts/evaluation/bosch800_asset_evidence.py` loads the USD as an Isaac Lab 2.1
`Articulation` with the asset's own drives and base joint (nothing overridden), reads the
dish sites the asset names under `<Rack>/Manipulation/`, and records a full
open -> extend -> load -> retract -> close cycle with this repo's plate and cup props:

```bash
scripts/run_kit.sh scripts/evaluation/bosch800_asset_evidence.py --headless --enable_cameras
scripts/run_kit.sh scripts/evaluation/bosch800_asset_evidence.py --headless --probe   # stage dump only
```

Output: `media/bosch800_asset/{closed,racks_extended,loaded_extended}_{iso,front}.png`,
`articulation.mp4` (15 s, 30 fps) and `evidence.json`. Run of record 2026-09-08:
`[RESULT] PASS` — every hold within 0.2 mm / 0.01 deg of target, the plate settled in
`LowerRack/plate_slot_0` and the cup on `MiddleRack/cup_drop`, and both rode their racks
back in with zero vertical drift.

## Validation evidence

The asset was built and validated outside this repo on Isaac Sim 6.0.1-rc.7 / Isaac Lab 3.0
(the build and validation scripts are not part of this branch; the asset is accepted here
as a prebuilt binary). The validation checked stage units, mesh indices, the joint set, a
complete motor-driven opening/extension/retraction/closing cycle, position errors, and a
block dropped onto the lower rack. It also released a 240 mm plate, a hollow 62 × 80 mm
inverted cup, and a 12 mm cutlery probe; checked settling and retention during loaded rack
motion; and checked contact-aware vertical retrieval using an ideal velocity-controlled
grasp fixture. This verifies asset support/access, not a robot arm, finger grasp, or
pick-and-place policy. Images and video come from Isaac Sim's RTX camera while PhysX drives
the articulation. Its `[RESULT] PASS` report ships as `validation.json`.

`assets/evidence/bosch800/`:

- `validation.json`: measured settling, loaded travel and retrieval (result `PASS`).
- `run.log`: the validation run's console log.
- `closed.png`, `open.png`, `racks_extended.png`, `interior_detail.png`: appliance states.
- `lower_rack_reference.png`, `middle_rack_reference.png`: isolated racks.
- `lower_rack_loaded.png`, `loaded_racks.png`, `dish_settle_detail.png`: dish support.
- `articulation.mp4`: the motor-driven open/extend/retract/close cycle.
- `test_dishes/{plate,cup}.usdc`: the dishes the validation released.
- `studio.usda`: the inspection scene (floor, lights, camera). It references the asset and
  the test dishes by absolute `/workspace/...` paths from the build machine, so it is
  inspection-only and non-portable.
- `previous_asset.zip`: the superseded asset (no basket rim, flat middle floor) kept for
  provenance, with `rack_collision_{tests,baseline}.log`: the spatial acceptance checks
  passed on the revised USD and failed on the previous one.

## References and accuracy

Bosch's [SHP78CM5N product page](https://www.bosch-home.com/us/en/product/SHP78CM5N) and
[official specification sheet](https://media3.bosch-home.com/Documents/20595211_SHP78CM5N%20Spec%20Sheet.pdf)
provide the exterior envelope, net mass, pocket-handle style, stainless tub and three-rack
configuration. Product imagery informs the visual styling. This repo's
[bosch800_source_data.md](bosch800_source_data.md) provides additional rack reference
dimensions.

Interior dimensions are estimates: liner width 552 mm, floor about 183 mm above the
ground, ceiling about 812 mm, lower rack origin 222 mm, middle 545 mm, third 757 mm. The
original prototype's 730 mm ceiling and thick upper housing were replaced with a taller tub
to better reflect the product photographs. Rack layouts follow two product photos. Exact
wire spacing, local floor steps, basket proportions, pocket/dispenser dimensions, control
artwork and concealed mechanical parts remain estimates; no physical appliance was measured.

The geometry and generated label/roughness textures are authored for this project under
BSD-3-Clause. Bosch names identify the reference product; this asset is not made or endorsed
by Bosch. No downloaded product photographs are included in the asset or its tarballs; the
two rack reference photos the geometry was traced from are not redistributed.
