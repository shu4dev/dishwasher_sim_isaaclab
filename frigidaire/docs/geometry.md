# FDPC4221AS current geometry (tape-measured racks and basket, heights fixed 2026-09-23)

Coordinates are metres in USD: X left/right, -Y front, +Y rear, Z up. Rack layout
images and this page report millimetres in the rack-local frame, with tine base
centres measured to the outer wire-rim edge. Wheel and grip projections are outside
the rim footprint.

The user tape-measured the real racks and basket on 2026-09-21. The source
(`frigidaire/src/dishsim_frigidaire/geometry.py`, `PARAMETERS`) was rewritten to those
numbers and the USD collection was rebuilt on 2026-09-22. The basket is the part that
matters; the rest of the appliance may relax in realism. Trusted inputs are the outer
rim size, the tine counts and the pitches. Edge margins are DERIVED from them and are
reported as derived. Wire diameters, tine leans, floor depths and every basket lattice
count remain estimates.

## Revisions

| Component | Revision | Basis |
|---|---|---|
| UpperRack | `upper_tines_4x13_v4` | tape: rim and 125 mm outside height, 4 x 13 grid, 37 mm pitch, 90 mm column pitch; v3 five-channel floor re-fitted inside the rim |
| LowerRack | `lower_tines_6x12_v3` | tape: rim and 115 mm outside height, 12 x 6 grid, 36 x 80 mm pitch, 95 / 45 mm tines |
| SilverwareBasket | `basket_1x4_320x95_v4` | tape, axes corrected from the front view 2026-09-22: 320 long x 95 wide top rim, 130 body and 220 mm handle top measured from the lowest wire, four compartments in a row; the v3 photo-fitted design (taper, double rims, loop handle) unchanged at that size, re-seated at x 184 |
| Cabinet, Door | `fdpc4221as_photo_v1` | unchanged photo fit |

The photo-fitted build (v3: photo-fitted racks and basket, the `v1` tine-grid revisions) is
archived as `build/frigidaire_collection/history/v3` (165 files, 301.6 MB; its
`archive_manifest.json` lists the old revision ids, usdc sha256, body positions and a
`stale_results` list). The first tape build (`upper_tines_4x13_v3`, `lower_tines_6x12_v2`,
`basket_1x4_320x95_v3`) is archived as `history/v4` (204 files, 344.5 MB, 2026-09-23).

## Outside heights (2026-09-23)

The user measured every height outside, bottom to top. The v4 build had put the rack rims
at the tape height as wire centrelines above an internal datum, so the racks measured taller
outside: upper 145.9 mm (its floor dips 16.8 mm below the datum), lower 119.4 mm, and the
basket 128.5 / 218.5 mm from its lowest wire. The current build keeps the floors, tines,
rollers and the basket's resting height, and fixes only the heights:

| Part | Change | Outside now |
|---|---|---:|
| Upper rack | rim wire centre 125 -> 104.1 mm above z = 0 | 125.0 (lowest floor wire to rim top) |
| Lower rack | rim wire centre 115 -> 110.6 mm above the floor-wire centre | 115.0 (floor wire underside to rim top) |
| Basket | bottom rim, reinforcement, floor lattice, corner-post feet, partition and strap feet 1.5 mm lower; seat z 7.5 -> 9 mm | 130 / 220 from the lowest wire (z = 0) |

`test_outer_heights_match_the_tape` pins all three (the preview measurements.json files do not
record heights; `usd/parameters.json` holds `rim_height` and `outer_height_tape`). The lower rack's
under-floor rails and wheel axles hang below the floor wires and are not part of the measured height.
Side effects, all small: the upper side wall above its z 61 kink is steeper (about 0.5 mm further
out at z 87, where HOTEC cups lean), the collider simplifier now merges that wall into one chord
capsule per side (1433 -> 1391 upper-rack colliders), the upper-rack centre of mass drops
54.6 -> 44.2 mm, the lower walls shift by at most 0.14 mm, and the basket's tine clearance rises
14.41 -> 14.63 mm. The basket's resting clearances (right wall 8.0, floor 3.4, bracket 1.92) are unchanged.

## Upper rack

Values verified against `build/frigidaire_collection/images/upper_rack/measurements.json`
(regenerated 2026-09-23 for `upper_tines_4x13_v4`).

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Outer rim W x D, height outside | 48 x 51.5 x 12.5 | 480 x 515 x 125 (rim wire centre z 104.1, lowest floor wire z -18.7) | measured |
| Rim centreline half-widths | | 237.8 / 255.3 | derived (4.4 mm rim wire) |
| Tine columns, pitch | 4 at 9 | x = -135, -45, +45, +135 | measured (symmetric) |
| Positions per column, pitch | 13 at 3.7 | y = -215.426 + 37 i, i = 0..12 | measured |
| Tines per column | 13 / 11 / 11 / 13 | 48 | measured |
| Side margins | 12 / 12 | 105 / 105 | derived |
| Front margin | 8 | 42.074 | derived |
| Rear margin | 5.5 | 28.926 | derived |
| Rear tip-centre margin | | 20.926 | derived |
| Tine diameter | | 3.6 | estimate |
| Tine rise / rearward lean | | 91 / 8 | estimate |

Derivation: the side margin is the symmetric leftover, (480 - 3 x 90) / 2 = 105. The
depth leftover, 515 - 12 x 37 = 71, is split by the tape ratio 80 : 55. The tape margins
over-determine the field by 30 mm in X and 64 mm in Y.

Absent tines: the two middle columns (x = +-45) lack positions 6 and 7 of 13 from the
front (y = -30.426 and 6.574). The surviving neighbours at y = -67.426 and 43.574 leave
one wide centre slot of 111 mm, centred at y = -11.9.

### Floor section: the photo-fitted five channels inside the tape rim

Half-section from the centre to the rim (wire centreline, mm; nominal corners before the
9 mm fillets). `PARAMETERS["upper_rack"]["channel_profile"]` in `geometry.py` holds the
valley and ridge offsets from the outer tine column, the ridge height, and the trough and
wall-foot insets from the rim centreline (the v3 offsets).

| x | z | Feature | Anchor |
|---:|---:|---|---|
| 0 to 53 | -10 | central ridge, carries the inner tine columns (x = +-45) | |
| 62 | +5 | ridge shoulder crest | |
| 72 | -5 | shoulder foot; the cup channel slopes down from here | |
| 135 | -16 | outer tine column, on the cup-channel slope | tape column |
| 145.5 | -18 | mug-cradle valley | column + 10.5 |
| 155.5 | +4 (+0.3 after the fillet) | low ridge between the mug cradle and the glass slope | column + 20.5 |
| 220 | -18 | glass trough, against the wall | rim - 17.8 |
| 231 | +2 | wall foot | rim - 6.8 |
| 235 | 61 | wall at the mid rim | rim - 2.8 |
| 237.8 | 104.1 | top rim centreline (125 before 2026-09-23) | |

Sites: cup channel at x = +-108.75 (mid cup slope), glass channel at x = +-187.75 (mid glass
slope, the tumbler mouth centre).

History of this section (2026-09-22). The first tape rebuild (`upper_tines_4x13_v2`, one day)
replaced the five channels with one V-shelf per side so an 80 mm tumbler could stand upright:
in the 480 mm rim the photo-fitted +18 mm crest at 155.5 left 79.5 mm to the wall. The user
found that outer wire structure further from the real rack than v3, so `upper_tines_4x13_v3`
restores the v3 shape inside the tape rim: mug valley and ridge anchored to the outer column,
the glass slope descending outward to a deep trough against the wall (v3 insets from the rim),
and the wall and rim as measured (the rim was lowered to 104.1 mm on 2026-09-23 so the rack is 125 mm outside). The one deliberate departure from v3 is the ridge
height, +4 mm instead of +18 (about +0.3 mm after the fillet), so an inverted 80 mm tumbler
stands near-upright over it with the ridge wire inside its open mouth, its outer rim point on
the wall foot and about 13 mm to the outer tine column. The claims dry run places all ten
tumblers (upright, 12 mm hover, mouth centre x 182.75).

## Lower rack

Values verified against `build/frigidaire_collection/images/lower_rack/measurements.json`.

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Outer rim W x D, height outside | 52.5 x 56.3 x 11.5 | 525 x 563 x 115 (rim wire centre z 110.6) | measured |
| Rim centreline half-widths | | 260.1 / 279.1 | derived (4.8 mm rim wire) |
| Columns, pitch | 12 at 3.6 | x = -207.923 + 36 i, i = 0..11 | measured |
| Rows, pitch | 6 at 8 | y = -205.433 + 80 j, j = 0..5 | measured |
| Left / right margins | 7.7 / 10.5 | 54.577 / 74.423 | derived |
| Front / rear margins | 10.5 / 12 | 76.067 / 86.933 | derived |
| Tine height, rows 1, 2, 5, 6 | 9.5 | 95 | measured |
| Tine height, rows 3, 4 | 4.5 | 45 | measured |
| Tine base height | | z = 6 | estimate |
| Tine diameter / rightward lean | | 3.9 / 8 | estimate |
| Tines present | | 64 (12, 12, 10, 10, 10, 10) | derived from the basket bay |

Derivation: the width leftover, 525 - 11 x 36 = 129, is split by the tape ratio
77 : 105; the depth leftover, 563 - 5 x 80 = 163, by 105 : 120. The tape margins
over-determine the field by 53 mm in X and 62 mm in Y.

Removed tines: the basket footprint x [136.5, 231.5], y [-64, 256] (the 95 x 320 top rim
around the seat), grown by 6 mm to the reserved bay x [130.5, 237.5], y [-70, 262], removes
columns 11 and 12 (1-based; x = 152.077 and 188.077) of rows 3 to 6: 8 tines. The base
rails of rows 3 to 6 end at the last remaining tine (column 10, x = 116.077), so the basket
rests on the floor wires.

Plate banks: front, between rows 1 and 2 at y = -165.433 (11 gaps, of which 9 lie left of
the bowl zone beside the basket bay); rear, between rows 5 and 6 at y = 154.567 (9 gaps;
claims A uses 8 of them, 6 dinner + 2 salad).

### Plate inset consequence

A 260 mm dinner plate centred on a bank cuts the rim wire, because the rows are 80 mm
apart in a 563 mm rack. Plate candidates therefore seat off the bank centreline:

| Consumer | Inset |
|---|---|
| claims, front bank (`claims.PLATE_INSET_Y`) | 25 mm rearward |
| claims, rear bank | 15 mm forward |
| HOTEC plates, front bank (`frigidaire_hotec_load.py`) | 10 mm rearward |
| plate fixture site | y = -150 |

## Basket

Own frame: origin at the bottom face centre, X across the width, Y along the length,
Z up; the lowest wire surface (the corner-post feet) is z = 0. Revision `basket_1x4_320x95_v4` (2026-09-23; `_v3` of 2026-09-22 evening sat 1.5 mm higher in its own frame): the user corrected the basket
axes from the FRONT view. The basket is 32 cm long (Y), 9.5 cm wide (X) and 13 cm tall
(Z, body), with the handle top at 22 cm. The earlier same-day readings (130 wide x 95 tall)
had the width and the height swapped. Superseded: `basket_1x4_320x130_v1` (box), `basket_1x4_320x130_v2`
(v3 design); both builds were overwritten in place, no archive. The v3 photo-fitted
DESIGN is unchanged: tapered lattice body, double bottom edge and top lip, corner posts,
three tapered cross partitions with a top edge, elongated loop handle with an open aperture
over the +X long wall on two straps. Only the size and the seat changed. Values verified
against the lower-rack `measurements.json` (basket block),
`build/frigidaire_collection/usd/parameters.json` and
`build/frigidaire_collection/validation/lower_rack_clearance.json`.

| Quantity | Tape (cm) | Model (mm) | Status |
|---|---:|---:|---|
| Top rim envelope W x L (outer wire surfaces) | 9.5 x 32 | 95 x 320 (centreline +-44.5 / +-157 at z 127, r 3) | measured |
| Body height (top rim outer surface above the lowest wire) | 13 | 130 | measured |
| Handle top surface above the lowest wire | 22 | 220 | measured |
| Floor envelope W x L | | 80.6 x 304.6 (bottom rim centreline +-37.3 / +-149.3 at z 3.5) | v3 taper ratios 0.848 / 0.952 |
| Bottom rim / reinforcement | | r 3 at z 3.5; r 2.3 at z 12.5, 0.5 mm outside | v3 design |
| Top rim / lip | | r 3 at z 127; r 2.4 at z 122, 0.5 mm inside | v3 design |
| Floor lattice | | 43 cross x 11 long ribs, 7.0 mm pitch, wire 2.2 (r 1.1); apertures about 4.8 mm | v3 pitch; counts derived |
| Long / end walls | | 43 / 10 tapered uprights, 11 courses (z 21 to 115; count from the 9.2 mm design pitch, 9.4 mm as placed), wire 2.2 | v3 pitch; counts derived |
| Corner posts | | r 2.5, floor corner to rim corner | v3 design |
| Compartments | 4 in a row | cross partitions at y = -80, 0, +80 (10 tapered uprights, 12 courses from z 14 to 118, top edge r 2.1 at z 125) | measured count; equal quarters (top-down photo) |
| Handle | on one long side | elongated loop with an open aperture in the plane x = +40.5 (4 mm inside the top rim), over the +X (outer) wall; legs at y = +-109 from the rim (z 127) to z 163.5, then the v3 bends to a flat top at z 213.5 (top surface 220) spanning y = +-75 (r 6.5); lower rail r 5.5 at z 171.5 (middle) / 176 (ends); aperture about 30 mm; two straps at y = +-80 (the partition lines) from the floor up the outer wall to the lower rail | v3 loop profile lifted to the tape height |
| Sites | | handle_center [40.5, 0, 213.5], handle_aperture [40.5, 0, 192], utensil [0, -120, 96], compartments at y = -120 / -40 / 40 / 120 | derived |
| Wire paths | | 289 | derived |

Seat in the lower rack: `[184, 96, 9]` mm (assembly origin `[184, 104, 224]`; the basket rests where it did at 7.5 mm, its bottom now at its own z = 0), re-fitted
for the 95 mm wide basket (the 130 mm wide basket sat at x 169): a seat sweep along X put the
tapered bottom rim about 8 mm from the sloping right wall fillet at x 184, and it keeps about
8 mm to the rear floor bend. The reserved bay is x [130.5, 237.5], y [-70, 262] (footprint +
6 mm), which omits 8 tines (columns 11-12 of rows 3-6) and ends the base rails of rows 3-6 at
column 10. The wall-hit probe in the tests needs an 18 mm rightward shift to go negative.

Clearance audit (`frigidaire_lower_rack_clearance.py`, `[RESULT] PASS`, static source
geometry only):

| Check | Value (mm) |
|---|---:|
| tines and base rails | 14.63 (TineBank5_Tooth09 to LongWall-1_Upright35; 14.41 before the basket grew 1.5 mm) |
| right wall | 8.0 |
| floor | 3.4 |
| remaining rack | 8.48 |
| visible wheel-bracket z gap | 1.92 |
| rails under the basket | none |
| removed tines | 8 |

A positive initial floor gap does not demonstrate settled support; that is the assembly
evidence run's job. The loop's +Y end sits about 20 mm in front of the retracted upper rack when
the lower rack is extended: 20.3 mm, loop end at world y -270.5 mm (basket origin y 104 + leg
109 + r 6.5, minus the 490 mm slide) against the upper rack's front grip at -250.2 mm (rim
centreline -255.3 - r 2.9, plus the rack origin 8). The seat's Y and the loop's Y extent are
unchanged from the 130 mm build, so the basket-lift gate has more room than with the old arch.

## Regenerate

Host-side previews (NumPy and Matplotlib only; they write `measurements.json` beside
the images):

```bash
python3 frigidaire/scripts/evaluation/frigidaire_upper_rack_preview.py \
    --out-dir build/frigidaire_collection/images/upper_rack      # 48 tines
python3 frigidaire/scripts/evaluation/frigidaire_lower_rack_preview.py \
    --out-dir build/frigidaire_collection/images/lower_rack      # 64 tines + basket_front/top/oblique.png
python3 frigidaire/scripts/evaluation/frigidaire_lower_rack_clearance.py   # -> validation/lower_rack_clearance.json
scripts/run_py.sh frigidaire/scripts/setup/build_frigidaire.py             # USD rebuild, [RESULT] PASS
```

`build/frigidaire_collection/usd/parameters.json` and `usd/geometry_validation.json`
record the generated dimensions and the revision ids.

## Previous revisions

The v3 build is archived under `build/frigidaire_collection/history/v3` (see
`archive_manifest.json` there for the stale-results list). The v3 geometry and its
photo-fitted derivations are described in
[docs/history/development_notes.md](history/development_notes.md).
