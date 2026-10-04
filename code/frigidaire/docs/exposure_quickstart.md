# Exposure scorer: quickstart

> **Status 2026-09-29.** The scorer is live (the HOTEC search, top-5 and benchmark code use it),
> but the demo scene, pair demo, seven-pair summary and pool search this page used to run were
> retired on 2026-09-29 (in git history at HEAD 4455813), and their result folders are in the hold
> folder `/media/corallab-s1/2tbhdd/brianshu/dishsim/_trash_20260929/` until 2026-10-29. The
> numbers below come from the v3 racks (2026-09-20) and are history; the HOTEC figure comes from
> racks built before 2026-09-28. Reference: [exposure.md](exposure.md).

Run this first (Kit-free; rays run on the GPU when Warp sees one):

```
code/scripts/run_py.sh -m pytest code/frigidaire/tests/test_exposure.py
```

Then open `data/results/exposure/frigidaire/hotec/heatmap.png`, the one method figure still in the
tree: the HOTEC set (8 plates, 8 bowls, 8 cups) before and after the exposure search.

## What you are looking at

The HOTEC figure (made 2026-09-22, on the pre-2026-09-28 racks):

- Left, two rows: every food-contact sample coloured by exposure, top-down, one panel per rack
  (LowerRack, UpperRack). Bright = water from the spray arm can reach it. Dark = something is in
  the way. Top row: the start load (run v8, S 0.159); bottom row: the best load of the search
  (S 0.212).
- Right: mean exposure per piece for each load (bowls orange, cups green, plates blue); the panel
  title gives the worst piece.
- `score` = area-weighted mean exposure over all food-contact surfaces. `worst` = the
  single worst object. Higher is better. Compare only arrangements of the same objects.
- The retired demo-scene figure (2026-09-20, pictures in the hold folder) also drew the spray-arm
  discs and one ray fan per added kind (green rays reach the disc, red rays are blocked by the
  load), and put a red marker over the bar of a vessel or plate that holds water; one of those
  makes the whole arrangement infeasible.

## The commands

1. Scorer tests (22 tests): the command above.
2. The HOTEC exposure search (Kit-free, CUDA; 34 min for three sweeps on 2026-09-22):
   `code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_exposure_search.py --sweeps 3 --seed 0 --out <new folder>`
   → `search.json` and `best_layout.json` in the new folder. Always pass a new `--out`: the
   default is `data/results/exposure/frigidaire/hotec/`, which holds the 2026-09-22 records.
3. The method figure of two loads (Kit-free):
   `code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_exposure_figure.py --start <start layout.json> --best <best_layout.json> --out <heatmap.png>`
4. Retired 2026-09-29, see git history at HEAD 4455813: the demo scene with its Isaac still and
   12 s orbit video, the one-pair demo, the seven-pair chart and the pool search of random_06
   (`frigidaire_exposure_{scene,scene_render,demo,summary,search}.py`), and the Isaac settle of a
   search proposal that went with them.

## Numbers to remember (v3 racks, 2026-09-20; history)

Computed on the v3 racks with arm radii 0.245 / 0.205, before the 2026-09-22 and 2026-09-28
rebuilds; the folders behind them are in the hold folder. Do not compare them with a score on the
current racks.

| | score |
|---|---|
| random packing (random_06) | 0.075, 8 of 18 vessels hold water |
| hand-organized | 0.242 |
| best found by search, settled and reproduced | 0.274 (settling moved it by 0.00003) |
| demo scene: organized + plate + fork, knife, tablespoon | 0.233, feasible |

Organized beats random in all 7 pairs by 0.12 to 0.22, and still in all 7 if the ceiling
nozzle is given up to half of the upper rack's water.

## What the number assumes

Water travels in straight lines from a disc under each rack (where the arm sweeps: radius
0.233 m under the lower rack, 0.191 m under the upper rack), hits harder when it hits squarely,
and nothing else. No splash. The ceiling nozzle is off by default because nobody has inspected
it (the `ceiling_weight` argument of `score_arrangement` turns it on); the ranking above holds
for any weight up to 0.5 and only flips at 1.0, which switches the middle arm off.
Food contact is the inside of vessels, the top of plates, the spoon bowl, the fork tines and
the knife blade; handles never count. A flat plate holds water and is infeasible; on edge it
drains.

## Next

The scorer is closed at revision 5 and is the objective of the HOTEC search, top-5 and
benchmark code. If the chart reads right, the next big step is a real wash test of the two
random_06 arrangements (two afternoons; the 2026-09-20 plan: the tree keeps those two states as
test fixtures, the full runs are in the hold folder).
