# FDPC4221AS current geometry (tape re-measured 2026-09-28)

Coordinates are metres in USD: X left/right, -Y front, +Y rear, Z up. Rack layout
images and this page report millimetres in the rack-local frame, with tine base
centres measured to the outer wire-rim edge. Wheel and grip projections are outside
the rim footprint.

The user tape-measured the real racks and basket on 2026-09-21 and re-measured the
racks on 2026-09-28 (the first tape's uniform pitches and margins over-determined both
racks by 3 to 6 cm; the re-measurement found the real column and row spacings are not
uniform). The source (`code/frigidaire/src/dishsim_frigidaire/geometry.py`, `PARAMETERS`)
holds those numbers and the USD collection was rebuilt on 2026-09-28. The basket is the
part that matters; the rest of the appliance may relax in realism. Trusted inputs are the
outer rim size, the tine counts, the pitches (per gap) and the FRONT and LEFT margins,
which are the user's counting datums ("tape" rule, chosen 2026-09-28). The rear and right
margins are the leftover of the rim and are reported as derived next to the tape figure.
Wire diameters, tine leans, floor depths and every basket lattice count remain estimates.

## Revisions

| Component | Revision | Basis |
|---|---|---|
| UpperRack | `upper_tines_4x13_v5` | tape 2026-09-28: 480 x 515 rim, 125 mm outside height, 4 columns at 92 / 86 / 92 mm, 13 positions at 33 mm from an 82 mm front margin; v3 five-channel floor unchanged |
| LowerRack | `lower_tines_6x12_v4` | tape 2026-09-28: 525 x 561 rim, 108 mm outside height, 12 columns at 31.8 mm from a 73 mm left margin, 6 rows at 81 / 73 / 67 / 73 / 81 mm from a 101 mm front margin, 95 / 45 mm tines |
| SilverwareBasket | `basket_1x4_320x95_v4` | unchanged: 320 long x 95 wide top rim, 130 body and 220 mm handle top measured from the lowest wire, four compartments in a row; seat `[184, 96, 9]` |
| Cabinet, Door | `fdpc4221as_photo_v1` | unchanged photo fit |

Archived builds under `data/build/frigidaire_collection/history/`: `v3` (photo-fitted racks and
basket), `v4` (first tape build, centreline rim heights, 2026-09-23), `v5` (the
2026-09-23 build with the outside heights fixed: `upper_tines_4x13_v4`, `lower_tines_6x12_v3`,
204 files, 344.5 MB, archived 2026-09-28). Each holds an `archive_manifest.json` with the
revision ids, usdc sha256 and a `stale_results` list.

## Outside heights

The user measured every height outside, bottom to top. The rim wire centre is set so
the generated wires reach the tape height:

| Part | Model | Outside |
|---|---|---:|
| Upper rack | rim wire centre 104.1 mm above z = 0 (floor dips to -18.7) | 125.0 (lowest floor wire to rim top) |
| Lower rack | rim wire centre 103.6 mm above the floor-wire centre (was 110.6 for the 115 mm of 2026-09-21) | 108.0 (floor wire underside to rim top) |
| Basket | lowest wire z = 0 | 130 / 220 (top rim, handle top) |

`test_outer_heights_match_the_tape` pins all three (`usd/parameters.json` holds `rim_height`
and `outer_height_tape`). The lower rack's under-floor rails and wheel axles hang below the
floor wires and are not part of the measured height. The 95 mm tines of the lower rack
now reach z 101, 2.6 mm under the rim wire centre.

## Upper rack

Values verified against `data/build/frigidaire_collection/images/upper_rack/measurements.json`
(regenerated 2026-09-28 for `upper_tines_4x13_v5`).

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Outer rim W x D, height outside | 48 x 51.5 x 12.5 | 480 x 515 x 125 (rim wire centre z 104.1) | measured |
| Rim centreline half-widths | | 237.8 / 255.3 | derived (4.4 mm rim wire) |
| Tine columns, gaps | 4 at 9.2 / 8.6 / 9.2 | x = -135, -43, +43, +135 (symmetric) | measured |
| Positions per column, pitch | 13 at 3.3 | y = -175.5 + 33 i, i = 0..12 | measured |
| Tines per column | 13 / 11 / 11 / 13 | 48 | measured |
| Front margin | 8.2 | 82 | measured (applied) |
| Side margins | 11.6 / 11.6 | 105 / 105 | derived: leftover, half per side |
| Rear margin | 4.4 | 37 | derived: leftover |
| Rear tip-centre margin | | 29 | derived |
| Tine diameter | | 3.6 | estimate |
| Tine rise / rearward lean | | 91 / 8 | estimate |

Derivation: the tape is symmetric across the rack (11.6 both sides), so the columns are
placed symmetrically at the tape gaps and the side margin is (480 - 270) / 2 = 105; the
tape margins exceed the rim by 22 mm. Positions run from the tape front margin at the
tape pitch, so the rear margin is 515 - 82 - 12 x 33 = 37; the tape margins exceed the
rim by 7 mm. The outer columns sit where they did (x = +-135), so the five-channel floor
is unchanged; the inner columns moved from +-45 to +-43 on the central ridge.

Absent tines: the two middle columns (x = +-43) lack positions 6 and 7 of 13 from the
front (y = -10.5 and 22.5). The surviving neighbours at y = -43.5 and 55.5 leave one wide
centre slot of 99 mm (was 111), centred at y = 6. The regular gaps are 33 mm (were 37).

### Floor section: the photo-fitted five channels inside the tape rim

Half-section from the centre to the rim (wire centreline, mm; nominal corners before the
9 mm fillets). `PARAMETERS["upper_rack"]["channel_profile"]` in `geometry.py` holds the
valley and ridge offsets from the outer tine column, the ridge height, and the trough and
wall-foot insets from the rim centreline (the v3 offsets).

| x | z | Feature | Anchor |
|---:|---:|---|---|
| 0 to 53 | -10 | central ridge, carries the inner tine columns (x = +-43) | |
| 62 | +5 | ridge shoulder crest | |
| 72 | -5 | shoulder foot; the cup channel slopes down from here | |
| 135 | -16 | outer tine column, on the cup-channel slope | tape column |
| 145.5 | -18 | mug-cradle valley | column + 10.5 |
| 155.5 | +4 (+0.3 after the fillet) | low ridge between the mug cradle and the glass slope | column + 20.5 |
| 220 | -18 | glass trough, against the wall | rim - 17.8 |
| 231 | +2 | wall foot | rim - 6.8 |
| 235 | 61 | wall at the mid rim | rim - 2.8 |
| 237.8 | 104.1 | top rim centreline | |

Sites: cup channel at x = +-108.75 (mid cup slope), glass channel at x = +-187.75 (mid glass
slope, the tumbler mouth centre).

History of this section (2026-09-22). The first tape rebuild (`upper_tines_4x13_v2`, one day)
replaced the five channels with one V-shelf per side so an 80 mm tumbler could stand upright:
in the 480 mm rim the photo-fitted +18 mm crest at 155.5 left 79.5 mm to the wall. The user
found that outer wire structure further from the real rack than v3, so `upper_tines_4x13_v3`
restored the v3 shape inside the tape rim: mug valley and ridge anchored to the outer column,
the glass slope descending outward to a deep trough against the wall (v3 insets from the rim),
and the wall and rim as measured (the rim was lowered to 104.1 mm on 2026-09-23 so the rack is
125 mm outside). The one deliberate departure from v3 is the ridge height, +4 mm instead of +18
(about +0.3 mm after the fillet), so an inverted 80 mm tumbler stands near-upright over it with
the ridge wire inside its open mouth. The claims dry run places all ten tumblers (upright, 12 mm
hover, mouth centre x 182.75).

## Lower rack

Values verified against `data/build/frigidaire_collection/images/lower_rack/measurements.json`
(regenerated 2026-09-28 for `lower_tines_6x12_v4`).

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Outer rim W x D, height outside | 52.5 x 56.1 x 10.8 | 525 x 561 x 108 (rim wire centre z 103.6) | measured |
| Rim centreline half-widths | | 260.1 / 278.1 | derived (4.8 mm rim wire) |
| Columns, pitch | 12 at 3.18 | x = -189.5 + 31.8 i, i = 0..11 | measured |
| Rows, gaps front to back | 6 at 8.1 / 7.3 / 6.7 / 7.3 / 8.1 | y = -179.5, -98.5, -25.5, 41.5, 114.5, 195.5 | measured |
| Left / front margins | 7.3 / 10.1 | 73 / 101 | measured (applied) |
| Right margin | 10.2 | 102.2 | derived: leftover (closes with the tape) |
| Rear margin | 11.3 | 85 | derived: leftover (the tape over-determines the depth by 28 mm) |
| Tine height, rows 1, 2, 5, 6 | 9.5 | 95 | measured |
| Tine height, rows 3, 4 | 4.5 | 45 | measured |
| Tine base height | | z = 6 | estimate |
| Tine diameter / rightward lean | | 3.9 / 8 | estimate |
| Tines present | | 64 (12, 12, 10, 10, 10, 10) | derived from the basket bay |

Derivation: columns run from the tape left margin at the tape pitch, 73 + 11 x 31.8 = 422.8,
leaving 102.2 of the 525 (tape 102: the width closes). Rows run from the tape front margin
at the five tape gaps, 101 + 375 = 476, leaving 85 of the 561 (tape 113). The 28 mm depth
residual is the one remaining tape conflict; it is left in the rear margin, behind the rear
plate bank, where nothing is placed.

Removed tines: the basket footprint x [136.5, 231.5], y [-64, 256] (the 95 x 320 top rim
around the seat), grown by 6 mm to the reserved bay x [130.5, 237.5], y [-70, 262], removes
columns 11 and 12 (1-based; x = 128.5 and 160.3) of rows 3 to 6: 8 tines. Column 11's base
centre sits 2 mm ahead of the bay, but its tip leans 8 mm toward the basket, so the bay
rule now tests the leaning tip as well as the base (`lower_tine_mask`, 2026-09-28). The base
rails of rows 3 to 6 end at the last remaining tine (column 10, x = 96.7), so the basket
rests on the floor wires.

Plate banks: front, between rows 1 and 2 at y = -139 (11 gaps, of which 10 lie left of
the bowl zone beside the basket bay; 9 at the old 36 mm pitch); rear, between rows 5 and 6
at y = 155 (9 gaps; claims A uses 8 of them, 6 dinner + 2 salad). The gaps are 31.8 mm
(were 36): a 3.9 mm tine leaves 27.9 mm clear for a plate. The short middle rows are 67 mm
apart (were 80).

### Plate inset consequence

A 260 mm dinner plate centred on a bank cuts the rim wire, because the outer row pairs sit
139 mm from the rack centre in a 561 mm rack. Plate candidates therefore seat off the bank
centreline:

| Consumer | Inset |
|---|---|
| claims, front bank (`claims.PLATE_INSET_Y`) | 25 mm rearward |
| claims, rear bank | 15 mm forward |
| HOTEC plates, front bank (`frigidaire_hotec_load.py`) | 10 mm rearward |
| plate fixture site | bank centre + 15 mm (y = -124) |

## Basket

Own frame: origin at the bottom face centre, X across the width, Y along the length,
Z up; the lowest wire surface (the corner-post feet) is z = 0. Revision `basket_1x4_320x95_v4`
(2026-09-23, unchanged by the 2026-09-28 rack re-measurement; its usdc hash is the same as in
`history/v5`): the basket is 32 cm long (Y), 9.5 cm wide (X) and 13 cm tall (Z, body), with
the handle top at 22 cm. The v3 photo-fitted DESIGN is unchanged: tapered lattice body, double
bottom edge and top lip, corner posts, three tapered cross partitions with a top edge, elongated
loop handle with an open aperture over the +X long wall on two straps. Values verified against
the lower-rack `measurements.json` (basket block), `data/build/frigidaire_collection/usd/parameters.json`
and `data/build/frigidaire_collection/validation/lower_rack_clearance.json`.

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Top rim envelope W x L (outer wire surfaces) | 9.5 x 32 | 95 x 320 (centreline +-44.5 / +-157 at z 127, r 3) | measured |
| Body height (top rim outer surface above the lowest wire) | 13 | 130 | measured |
| Handle top surface above the lowest wire | 22 | 220 | measured |
| Floor envelope W x L | | 80.6 x 304.6 (bottom rim centreline +-37.3 / +-149.3 at z 3.5) | v3 taper ratios 0.848 / 0.952 |
| Bottom rim / reinforcement | | r 3 at z 3.5; r 2.3 at z 12.5, 0.5 mm outside | v3 design |
| Top rim / lip | | r 3 at z 127; r 2.4 at z 122, 0.5 mm inside | v3 design |
| Floor lattice | | 43 cross x 11 long ribs, 7.0 mm pitch, wire 2.2 (r 1.1); apertures about 4.8 mm | v3 pitch; counts derived |
| Long / end walls | | 43 / 10 tapered uprights, 11 courses (z 21 to 115), wire 2.2 | v3 pitch; counts derived |
| Corner posts | | r 2.5, floor corner to rim corner | v3 design |
| Compartments | 4 in a row | cross partitions at y = -80, 0, +80 (10 tapered uprights, 12 courses from z 14 to 118, top edge r 2.1 at z 125) | measured count; equal quarters (top-down photo) |
| Handle | on one long side | elongated loop with an open aperture in the plane x = +40.5 (4 mm inside the top rim), over the +X (outer) wall; legs at y = +-109 from the rim (z 127) to z 163.5, then the v3 bends to a flat top at z 213.5 (top surface 220) spanning y = +-75 (r 6.5); lower rail r 5.5 at z 171.5 (middle) / 176 (ends); aperture about 30 mm; two straps at y = +-80 (the partition lines) | v3 loop profile lifted to the tape height |
| Sites | | handle_center [40.5, 0, 213.5], handle_aperture [40.5, 0, 192], utensil [0, -120, 96], compartments at y = -120 / -40 / 40 / 120 | derived |
| Wire paths | | 289 | derived |

Seat in the lower rack: `[184, 96, 9]` mm (assembly origin `[184, 104, 224]`), unchanged. The
reserved bay is x [130.5, 237.5], y [-70, 262] (footprint + 6 mm), which omits 8 tines
(columns 11-12 of rows 3-6) and ends the base rails of rows 3-6 at column 10. The wall-hit
probe in the tests needs an 18 mm rightward shift to go negative.

Clearance audit (`frigidaire_lower_rack_clearance.py`, `[RESULT] PASS`, static source
geometry only, 2026-09-28):

| Check | Value (mm) |
|---|---:|
| tines and base rails | 33.79 (TineBank4_Tooth09 to LongWall-1_Course08; 14.63 before, when column 11 stood 34 mm closer) |
| right wall | 8.0 |
| floor | 3.4 |
| remaining rack | 7.68 (rear floor bend) |
| visible wheel-bracket z gap | 1.92 |
| rails under the basket | none |
| removed tines | 8 |
| tine x-envelope gap | 29.85 |

A positive initial floor gap does not demonstrate settled support; that is the assembly
evidence run's job (`data/build/frigidaire_collection/images/assembly`, physics and render PASS on
2026-09-28). The loop's +Y end sits about 20 mm in front of the retracted upper rack when
the lower rack is extended, as before (the seat and the loop are unchanged).

## Consequences for the loads (2026-09-28)

| Check | Before (`v4` / `v3` racks) | Now |
|---|---|---|
| claims A FCL dry run | PASS, 49 objects, upper bowls 3 | PASS, 48 objects, upper bowls 2 (the wide centre slot shrank 111 to 99 mm) |
| HOTEC first-fit layout (`--layout-only`, v2 assets) | plates 8/8, bowls 6/8, cups 8/8 | plates 8/8, bowls 5/8, cups 8/8, counter 3 (`data/results/hotec/frigidaire/v14/layout.json`) |
| benchmark capacity gate (`frigidaire_bench.py --capacity`) | PASS | PASS (7/7, 15/15, 23/23; family caches regenerated by digest) |

Every Isaac result recorded on the previous racks is stale (listed in
`history/v5/archive_manifest.json`): HOTEC runs v11 to v13, the exposure search results,
and the whole HOTEC benchmark (`data/results/benchmark/frigidaire_hotec/`: baseline snapshot,
instances, goals, plans, episodes). Regenerate them before quoting numbers.

## Regenerate

Host-side previews (NumPy and Matplotlib only; they write `measurements.json` beside
the images):

```bash
python3 code/frigidaire/scripts/evaluation/frigidaire_upper_rack_preview.py \
    --out-dir data/build/frigidaire_collection/images/upper_rack      # 48 tines
python3 code/frigidaire/scripts/evaluation/frigidaire_lower_rack_preview.py \
    --out-dir data/build/frigidaire_collection/images/lower_rack      # 64 tines + basket_front/top/oblique.png
python3 code/frigidaire/scripts/evaluation/frigidaire_lower_rack_clearance.py   # -> validation/lower_rack_clearance.json
python3 code/frigidaire/scripts/evaluation/frigidaire_rack_dimensions_cm.py     # -> data/media/frigidaire_rack_dimensions/*.png (curated copies in docs/figures)
python3 code/frigidaire/scripts/setup/stage_frigidaire_collection.py --archive-current v6   # BEFORE a rebuild; never reuse a version
code/scripts/run_py.sh code/frigidaire/scripts/setup/build_frigidaire.py             # USD rebuild, [RESULT] PASS
```

`data/build/frigidaire_collection/usd/parameters.json` and `usd/geometry_validation.json`
record the generated dimensions and the revision ids.

## Previous revisions

The v3 build is archived under `data/build/frigidaire_collection/history/v3`, the first tape
build under `history/v4` and the 2026-09-23 outside-height build under `history/v5` (see
each `archive_manifest.json` for the stale-results list). The v3 geometry and its
photo-fitted derivations are described in
[docs/history/development_notes.md](history/development_notes.md); the 2026-09-21 tape
numbers (36 x 80 and 37 / 90 mm uniform pitches, 563 mm depth, 115 mm lower height) are
kept as the superseded `source` entries in `geometry.py`.
