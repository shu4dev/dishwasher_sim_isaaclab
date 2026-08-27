# Bosch 800 Series source data (Stage A0)

Compiled 2026-08-10 from published sources (parallel web research, four independent
sweeps: manuals/spec sheets, replacement-rack parts, retailer cross-check, real-dish
shortlist). Target machine: **SHP78CM5N** (standard tall-tub, 16 place settings) — the
digital-twin reference. The ADA variant (SGX78C55UC) is documented in §6 but NOT modeled.

Every value carries a provenance tag:

- `published` — printed in an official spec sheet, manual, or listing (URL given).
- `derived-from-drawing` — read off a dimensioned drawing or computed from published
  values (derivation in notes).
- `estimated` — not published anywhere found within the timebox; engineering estimate
  with its basis in notes. **The `estimated` rows are the Stage-B caliper checklist.**

Primary documents:

- Spec sheet SHP78CM5N: https://media3.bosch-home.com/Documents/20595211_SHP78CM5N%20Spec%20Sheet.pdf
  (mirror: https://static.pcrichard.com/docs/SHP78CM5N_Specifications-Sheet_01.pdf)
- Installation instructions: https://media3.bosch-home.com/Documents/9001559533_B.pdf
- Spec sheet SGX78C55UC (ADA): https://media3.bosch-home.com/Documents/21860881_SGX78C55UC_Spec_Sheet.pdf
- Exterior cross-check: https://www.dimensions.com/element/bosch-800-series-recessed-handle-dishwasher-24

## 1 Exterior and installation [mm]

| item | value | provenance | source | notes |
|---|---|---|---|---|
| exterior_width | 598 | published | spec sheet | 23 9/16 in; all sources agree |
| exterior_height | 860–890 | published | spec sheet | 33 7/8–35 in; legs down → up |
| leveling_leg_range | 30 | derived-from-drawing | spec sheet | 890 − 860 |
| exterior_depth_closed | 602 | published | spec sheet | 23 3/4 in incl. door; pocket handle recessed, no projection |
| open_door_total_depth | 1321 | published | dimensions.com | 52 in back plane → door tip at 90° |
| door_projection_open_90 | 719 | derived-from-drawing | dimensions.com | 1321 − 602 |
| toe_kick_height | 90–120 | published | spec sheet | 3 1/2–4 3/4 in, tracks leg range |
| toe_kick_setback | 89 | derived-from-drawing | spec sheet | 3 1/2 in side-view callout; label ambiguous, medium confidence |
| niche_cutout | W 610 × H ≥860 × D 610 | published | spec sheet | table value; drawing prints W 600–616 (conflict, §7) |
| countertop_gap_above_top | ~13 | published | install PDF | appliance top → counter underside after leveling |
| countertop_underside_std | 864–876 | estimated | install PDF | standard 36 in counter; consistent with 860 min niche |
| net_weight_kg | 43.5 | published | spec sheet | 96 lb; dimensions.com's 100 lb is shipping weight (§7) |
| rear_service_passthroughs | elec 120×60, water 100×50, drain dia 32 | published | install PDF | rear connections; geometry-irrelevant to the twin |

## 2 Door and hinge [mm]

| item | value | provenance | source | notes |
|---|---|---|---|---|
| door_panel_width | 595 | derived-from-drawing | spec sheet | 23 7/16 in drawing callout; 3 mm narrower than body |
| door_panel_height | 770 | derived-from-drawing | spec sheet | 30 3/8 in; 117 + 770 = 887 ≈ max height — consistent |
| door_panel_thickness | 53 | derived-from-drawing | spec sheet | 2 1/8 in side-view callout |
| door_bottom_above_floor | 117 | derived-from-drawing | spec sheet | 4 5/8 in side view |
| hinge_axis_height | 150 | estimated | install PDF | **constraint (published-derived): hinge_z + setback = door_top(887) − projection(719) = 168**; enforce the constraint, not the split |
| hinge_axis_setback | 18 | estimated | install PDF | companion to the row above |
| open_door_inner_face_z | ~132 | estimated | — | hinge geometry: axis z − setback ≈ 150 − 18; the lower rack rolls onto this surface |
| dispenser_bump | center of inner door face, ~240×160 env., ~15 proud, center ~400 above door bottom | estimated | product photos | collision feature when the door is open (bump faces up) |

## 3 Tub interior [mm] — all estimated; primary Stage-B caliper targets

Nothing interior is published. Two independent sweeps estimated (W 545 × D 520 × H 560)
and (W 545 × D 550 × H 530); the modeling set below reconciles them against the published
anchors (plate-capacity table §4, door geometry §2, exterior §1) into one self-consistent
parameterization. Confidence: medium on W, low-medium on D/H and all z-positions.

| item | value | provenance | source | notes |
|---|---|---|---|---|
| tub_interior_width | 545 | estimated | both sweeps agree | 598 − ~26/side (SS wall + insulation) |
| tub_interior_depth | 544 | estimated | reconciled 520/550 | 602 − 53 door − ~5 rear; floor of the range is impossible — the PUBLISHED 549 lower-rack depth must stow inside (racks sit flush at the rear wall, nosing ~5 over the sill) |
| tub_interior_height | 545 | estimated | reconciled 560/530 | typical Bosch tall tub 21–22 in |
| tub_floor_z | 185 | estimated | constraint chain | floor_z + interior_H + top structure = 860; sits just above the ~170 sill |
| tub_front_sill_z | ~172 | estimated | manuals sweep | just above open-door inner face (~132) so the rack rolls out over a short ramp |
| tub_ceiling_z | 730 | estimated | = floor_z + interior_H | leaves 130 for top frame/controls/CrystalDry duct — thick, verify Stage B |
| sump_recess | dia ~200 × ~50 deep, center-front | estimated | manuals sweep | bowl-shaped floor; model floor as sloped slab + sump box |
| tub_walls | convex slabs | derived-from-drawing | — | modeling decision: explicit convex slabs (analytic decomposition path) |

## 4 Racks [mm]

### 4.1 Lower rack (OEM part 20007189, supersedes 20000533)

| item | value | provenance | source | notes |
|---|---|---|---|---|
| outer_W×D×H | 527 × 549 × 196 | published | Amazon OEM listings | 20 3/4 × 21 5/8 × 7.7 in; H incl. wheels + tines; second listing agrees within 5 mm (§7) |
| wheels | 8 × dia 35 | published | hnkparts 00611475 | 4/side, front pair ball-bearing; run on tub-floor ribs, no raised rail |
| wire_plane_above_tub_floor | ~43 | estimated | wheel geometry | wheel dia 35 + wire offset |
| tine_pitch | 50 (main rows), ~25 doubled | estimated | capacity derivation | 16 settings across 2 main rows of ~500 mm span; medium confidence |
| tine_height | 95 | estimated | listing height split | 196 − 35 wheels − ~65 basket wall; typical Bosch 90–100 |
| tine_features | rear fold-flat rows + adjustable front tines (FlexSpace) | published | Amazon listing + Bosch marketing | every-other-tine folds |
| travel | ~560 (fully out onto open door) | estimated | open-depth geometry | rack depth 549 + ~10; door provides the runway (1321 open depth) |

### 4.2 Middle rack (OEM part 20007068; RackMatic)

| item | value | provenance | source | notes |
|---|---|---|---|---|
| outer_W×D | 511 × 556 | published | Amazon OEM listing | 20 1/8 × 21 7/8 in; narrower — rides side rails |
| overall_H | ~190 | estimated | photos | ~85 wire wall + ~105 tines |
| tine_pitch | 45 | estimated | photos | low-medium confidence |
| tine_height | 60 | estimated | typical upper-rack | glasses lean on short tines |
| rackmatic | 3 heights / 9 positions, 50 total travel | published | Bosch marketing + Best Buy Q&A | "up to 2 in"; ~25 per step |
| rail_z_above_tub_floor | 330 mid (305/355 low/high) | estimated | plate-capacity constraint | see constraint below |
| travel | ~510 | estimated | rails stop short of full extension | roller EasyGlide, 500–530 range |

**Published plate-capacity constraint (anchors the vertical stack):** largest lower-rack
plate by RackMatic position — **320 mm (top), 300 mm (mid), 270 mm (low)** [published,
Bosch use-and-care via Best Buy Q&A]. Constraint: `middle_rack_underside_z ≈
tub_floor_z + wire_plane(43) + plate_dia + clearance` per position. With floor_z 185:
plate tops ≈ 185+43+320 = 548 → raised rail ≈ z 540–550, consistent with rail 355 above
tub floor at the top position.

### 4.3 Third rack (OEM 20007198 / kit SMZCD200UC, "Flexible 3rd Rack")

| item | value | provenance | source | notes |
|---|---|---|---|---|
| outer_W×D×H | ~508 × ~546 × ~90 | derived-from-drawing | Abt boxed dims 20×22×4 in | tray marginally smaller than box; H = raised side wings |
| tray_usable_depth | 50 (wings) / 80 (center channel) | estimated | product photos | shallow flat wings for cutlery lying flat |
| vgroove_drop | 28 | estimated | product photos | center channel drops below wing plane; low-medium |
| rail_z_above_tub_floor | 450 | estimated | ceiling constraint | wings top (rail + 90) must clear ceiling (730): 185+450+90 = 725 ✓; middle-rack glass clearance below ≈ 450 − 355 − wall ≈ tight — Stage-B caliper priority |
| travel | ~510 | estimated | same rail family as middle | |
| loading_area_bonus | +30 % | published | spec sheet | vs 2-rack Bosch (marketing, sanity only) |

### 4.4 Glides

| item | value | provenance | source | notes |
|---|---|---|---|---|
| glide_type | EasyGlide rollers on all 3 racks | published | prudentreviews + parts listings | NOT telescopic ball-bearing (that's Benchmark's EasyGlide Plus). Lower rack: wheels onto the open door. Middle/third: rollers in tub-side rails |

## 5 Spray arms [mm] — count published, dims estimated

| item | value | provenance | source | notes |
|---|---|---|---|---|
| spray_levels | 3 physical: lower arm, mid arm under middle rack, ceiling spray head | published | parts listings (PartSelect) | "Five-level wash" on the sheet counts zones, not arms |
| lower_arm_span | ~490 | estimated | 24-in replacement arms ~19–19.5 in | rotates above tub floor, under lower rack |
| mid_arm_span | ~410 | estimated | visibly shorter in parts photos | mounted under the middle rack (moves with it) |
| top_spray_head_dia | ~90 | estimated | ceiling disc sprayer | above third rack |

These bound under-rack clearance (absent from the v1 machine) — model as thin cylinders.

## 6 Variants (documented, not modeled)

| item | value | provenance | source | notes |
|---|---|---|---|---|
| ADA SGX78C55UC | H 814 × D 573, 15 settings, 37.6 kg, V-shaped std 3rd rack | published | Bosch ADA spec sheet | shorter AND shallower — not just a height change; do not mix with SHP geometry |
| SHX78 (bar handle) | same body; handle adds front projection | published | Lowe's listing | Lowe's 653 mm depth includes the handle (§7) |

## 7 Conflicts found (resolution chosen)

1. **Niche width**: table says 610 flat; drawing says 600–616. → Model uses the machine
   body (598), niche irrelevant to the twin; recorded for Stage B.
2. **Depth 25.70 in (Lowe's SHXM78Z55N)** vs 23 3/4 in — Lowe's includes the bar-handle
   projection. → 602 body depth (SHP pocket handle) stands.
3. **Weight 96 lb (Bosch) vs 100 lb (dimensions.com)** → 43.5 kg net; 100 lb is shipping.
4. **Lower-rack depth 21 5/8 vs 21.8 in** across two OEM listings (~5 mm) → 549.
5. **Corner-install clearance second value 817 (EN) vs 867 (FR)** in the same install
   PDF → unused by the model; recorded.
6. **Tub interior estimates** differ between sweeps (D 520/550, H 560/530) → reconciled
   modeling set in §3, all tagged estimated.
7. **Min water pressure 14 vs 7 psi** between the two official sheets → irrelevant to
   geometry; recorded.

## 8 Modeling set for `MACHINE_GEN["bosch800"]` (machine frame: X width, Y depth, Z up from floor)

The single self-consistent parameter set A1 implements. Provenance per row above; the
constraint equations (not the split values) are what the parametric model enforces:

- Body: 598 W × 860 H × 602 D; toe-kick void 90 h × 89 deep at the front bottom.
- Door: 595 × 770 × 53 panel; hinge axis z 150, setback 18 (**hinge_z + setback = 168**);
  locked open at 90°; inner face (open) at z ≈ 132, top surface becomes the rack runway;
  dispenser bump ~240×160×15 centered ~400 from the door bottom (faces up when open).
- Tub: interior 545 W × 530 D × 545 H; floor z 185 (sloped, sump dia 200 × 50 center-
  front); sill z 172; ceiling z 730; walls/roof/sump as explicit convex slabs.
- Lower rack: 527 × 549, wire plane at floor + 43, tine rows pitch 50, tine h 95,
  travel 0 → −560 (onto the door), 8 wheels dia 35.
- Middle rack: 511 × 556, rail z 515 (= floor + 330, RackMatic mid; param ±25),
  travel −510, tine pitch 45, tine h 60.
- Third rack: 508 × 546 × 90, rail z 635 (= floor + 450), travel −510, shallow tray
  (wings 50 deep, center channel −28), cutlery flat-lay.
- Spray arms: cylinders — lower span 490 above floor under lower rack; mid span 410
  under middle rack; ceiling head dia 90.
- RackMatic: ONE position modeled in Stage A (mid); height as a parameter.
- Counter: real 900 mm-deep-run worktop beside the machine per base placement (A1).

## 9 Real-dish shortlist (14 classes)

All 14 classes sourced, purchasable, and dishwasher-safe → include.

| class | product | key dims [mm] | mass [g] | notes |
|---|---|---|---|---|
| mug | IKEA 365+ 36 cl (802.783.67) | body dia 90, h 80 | 380 published | |
| plate | Corelle Winter Frost 10.25 in | dia 260, h ~25 est | 340 published | Vitrelle ≈ half ceramic mass |
| saucer | IKEA VÄRDERA (402.774.59) | dia 180 | ~230 estimated | one SKU with the cup |
| bowl | IKEA 365+ 13 cm (502.589.50) | dia 130, h 60 | ~290 estimated | thin porcelain wall at rim |
| cup | IKEA VÄRDERA teacup | h 80, body ~95 est | ~280 estimated | |
| fork | IKEA DRAGON (005.155.27) | L 190 | ~50 estimated | |
| spoon | IKEA DRAGON (705.155.24) | L 190 | ~55 estimated | |
| knife | IKEA DRAGON (405.155.30) | L 210 | ~80 estimated | table knife |
| spatula | IKEA GUBBRÖRA (705.781.30) | 250 × 50 × 10 | 40 published | |
| tumbler | Duralex Picardie **160 ml** | body dia 75, h 78 | ~150 estimated | chosen over the 250 ml (body dia 85) |
| wine_glass | IKEA SVALKA 30 cl (300.151.23) | h 180, bowl ~80 est, stem ~8 | 172 derived | |
| serving_spoon | IKEA FINMAT 13 in (705.783.85) | L 330 | ~110 estimated | |
| container | IKEA 365+ 750 ml square (905.779.31) | 150 × 150 × 70 | ~120 estimated | PP wall |
| lid | IKEA 365+ square lid (705.779.51) | 152 × 152 × ~13 | 113 published | pairs with container |

Source URLs (dish rows): IKEA/Corelle/Duralex product pages — recorded in the research
archive (`results/a0_research/`, this session) and inline in the rows' listings above.

## 10 Stage-B caliper checklist

Every `estimated` row above, in priority order:

1. Tub interior W/D/H, floor z, sill z, ceiling z (§3) — anchors everything.
2. Rail heights: middle (3 RackMatic positions) + third; middle-rack glass clearance.
3. Hinge axis height/setback split (constraint 168 is published-derived; split is not).
4. Tine pitch/height, lower + middle; third-rack tray depths + V-groove.
5. Rack travels (lower onto door; middle/third rail stop).
6. Spray arm spans/positions; dispenser bump envelope.
7. Dish masses currently estimated (saucer, cup, bowl, fork, spoon, knife, tumbler,
   serving_spoon, container).
