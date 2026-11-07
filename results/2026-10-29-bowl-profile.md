# Bowl body from the measured profile (HOTEC v4): run note, 2026-10-29
Plan: plans/2026-10-29-bowl-profile.md. **Works-when: met.** The scratch bowl.usda passed every check, the dataset now carries v4, and the hub matches it (HF commit `da381dee`). Mac only, Kit-free, no box.
- **Build.** `uv run --no-project --python 3.10 --with numpy==1.26.0 --with scipy==1.14.1 --with usd-core==24.5 --with matplotlib==3.9.2 python data/assets/hotec_wheatstraw/source/hotec_gen.py --out <scratch> --no-figures --set plate.mass_kg=0.094625 --set bowl.mass_kg=0.067375 --set cup.mass_kg=0.0595` gave `[RESULT] PASS`, and a second build was byte-identical. pytest in the same env with pytest added: 19 passed, 1 skipped (the USD round trip needs the container).
- **Bowl.** The inner surface is a PCHIP z(r) through the 7 points and the rim edge, mirrored at the axis. The outer is the uniform 2.8 mm normal offset. The rim top is flat, 2.8 mm wide; a 0.945 mm straight step reaches the corner (72.265, 73.12), within 0.024 mm of the offset curve. The rib spans r 29.70-32.86 from z 0 to the shell, and its collision band is buried 1 mm. 7 shell bands plus the rib give 192 pieces (`side_rows` 4). The plate and cup USDs differ from v3 only in the 3 stamp lines.

| check (mm unless noted) | target | v4 bowl.usda (visual and colliders, read back) |
|---|---|---|
| rim at the top, inner / outer | 69.465 / 72.265 | 69.465 / 72.265 (both meshes) |
| wall, normal: centre, z 11, 20, 35, 50, 70 | 2.80 +- 0.05 | 2.800, 2.800, 2.800, 2.799, 2.800, 2.800 |
| underside at the centre | 5.77 +- 0.1 | 5.770 |
| ring faces r 29.70 / 32.86 | z 0 up to the shell | visual z 0-9.595 / 0-10.817; colliders the same plus the rib top 10.595 / 11.817; every vertex below 5.77 lies within r 29.70-32.86 |
| collider union / open space | solid / open | 600/600 samples solid on 3 verticals through the ring; 6/6 open probes (recess, beside the ring, under the centre, axis, cavity) open |
| height | 73.12 | 73.120 |
| inner curve vs the plan's 8 points | within 0.1 | 7 nodes < 2e-6; rim edge 0.005 (the plan's 69.47 is rounded) |
| brimful | about 657 mL | 649.0 mL (the curve's exact integral 649.2) |
| cavity intrusion | under 1 | 0.61 (generator check), 0.94 (sampled on every hull face) |
| convex pieces | 150-220 | 192 |

- **Brimful.** 649 mL, from the plan's mirrored z(r) PCHIP. A PCHIP of r(z) through the same points gives 658 mL, which is where the plan's "about 657" came from. Neither is tuned.
- **Swap.** Hold folder: data/_trash_20261029/hotec_bowl_v3_ring/, all 13 v3 files plus MANIFEST.txt. Replaced the 9 hotec files and README.md (now v4), and bowl.json via its writer (plate.json and cup.json came out byte-identical). In-place pytest: 19 passed, 1 skipped. A rebuild from the in-place generator reproduced every shipped file.
- **Upload.** `uvx --from huggingface_hub hf upload shu4dev/dishsim-assets data/assets/hotec_wheatstraw hotec_wheatstraw --repo-type dataset --exclude plate.json --exclude cup.json --exclude bowl.json --commit-message "Bowl body from the user's depth profile"` made commit `da381dee` with 10 files. Hub check: all 10 git blob ids match, and the 10 files fetched at `da381dee` match the local sha256 (10/10). The hub folder holds exactly these 10 files, and machine.json was not touched.
- **Deviations.**
  1. The rim lip is dropped for the bowl (coordinator decision A). A w/2 lip on the wall, which leans 13.3 deg at the rim, would have made the bowl 144.02 mm wide.
  2. Brimful is 649 mL, not 657.
  3. hf 2.4.0 takes one `--exclude` per pattern.
  4. Finding, not fixed: the generator's intrusion check underestimates on the concave inner side, because each hull spans its whole 15 deg sector. Sampled on the hull faces: bowl v4 0.94 mm, v3 1.37 mm, plate 1.17 mm (over 1 mm since v1), cup 0.28 mm. Fixing it would touch the plate and cup.
  5. The stamp string and usd/README.md still call every kind's dimensions "nominal listing targets plus documented estimates".
- **Missing.** The container's USD round-trip test (next box window). Re-settling the sampler goals on v4. The recess-depth and brimful measurements. The model's 89.5 mL of material and the measured 67.4 g give 0.75 g/mL, below polypropylene's 0.9, so the real wall is probably thinner below the rim.
- **Files.** data/assets/hotec_wheatstraw/{README.md, bowl.json, catalog.json, parameters.json, source/{README.md, hotec_gen.py, test_hotec_gen.py}, usd/{README.md, bowl.usda, plate.usda, cup.usda}} (dataset copy, gitignored); docs/hotec_wheatstraw_asset.md (new section "Measured bowl body (v4, 2026-10-29)"); this note.
- **Suggested commit:** `Document the HOTEC v4 bowl built from the depth profile`
