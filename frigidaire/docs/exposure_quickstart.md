# Exposure scorer: quickstart

Run this first (10 seconds, Kit-free; rays run on the GPU when Warp sees one):

```
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_scene.py --pair random_06
```

Then open `results/exposure/frigidaire/scene/random_06_plus_method.png` next to the Isaac
still `random_06_plus_initial.png` and the orbit video `random_06_plus_orbit.mp4` (made once
with the Kit command below).

## What you are looking at

- The Isaac still: the settled organized random_06 load (9 bowls, 9 mugs) plus one dinner
  plate standing on edge in the lower rack and a fork, a knife and a tablespoon head-down in
  the basket. The plate and cutlery are posed, not settled.
- The method figure, left: every food-contact sample coloured by exposure. Bright = water
  from the spray arm can reach it. Dark = something is in the way. The black discs are the
  sources (where each arm sweeps). One ray fan per added kind: green rays reach the disc,
  red rays are blocked by the load.
- The method figure, right: mean exposure per object; orange = the added kinds. A red
  marker over a bar = a vessel or plate that holds water; one of those makes the whole
  arrangement infeasible.
- `score` = area-weighted mean exposure over all food-contact surfaces. `worst` = the
  single worst object. Higher is better. Compare only arrangements of the same objects.

## The commands

1. Demo scene (10 s): the command above. Isaac still plus a 12 s orbit video of the same scene (4 min, GPU):
   `scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_exposure_scene_render.py --headless --enable_cameras --state results/exposure/frigidaire/scene/random_06_plus.json --out-dir results/exposure/frigidaire/scene --orbit-seconds 12`
   → `random_06_plus_initial.png`, `random_06_plus_orbit.mp4` (refuses to overwrite; remove the old
   PNG, MP4 and evidence JSON via `docker exec dishsim-isaac rm` first).
2. One pair, all pictures (3 min, GIFs dominate):
   `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_demo.py --pair random_06`
   → `results/exposure/frigidaire/demo_random_06/evidence.png`, `rays.gif` (the spray arm
   rotating under one bowl with green open and red blocked rays).
3. All seven pairs, one chart (under a minute):
   `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_summary.py --ceiling-sweep --convergence random_06`
   → `results/exposure/frigidaire/summary/summary.png`, `ceiling_sweep.md`, `convergence.md`.
4. Find a better arrangement (under a minute):
   `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_search.py --pair random_06`
   → `results/exposure/frigidaire/search/hist.png` and `best.png`. Proposals that hold
   water are dropped before ranking. Proposals are not settled; settling the best takes
   4 min in Isaac (steps in `docs/exposure.md`).

## Numbers to remember

| | score |
|---|---|
| random packing (random_06) | 0.075, 8 of 18 vessels hold water |
| hand-organized | 0.242 |
| best found by search, settled and reproduced | 0.274 (settling moved it by 0.00003) |
| demo scene: organized + plate + fork, knife, tablespoon | 0.233, feasible |

Organized beats random in all 7 pairs by 0.12 to 0.22, and still in all 7 if the ceiling
nozzle is given up to half of the upper rack's water.

## What the number assumes

Water travels in straight lines from a disc under each rack (where the arm sweeps), hits
harder when it hits squarely, and nothing else. No splash. The ceiling nozzle is off by
default because nobody has inspected it (`--ceiling-weight` turns it on); the ranking above
holds for any weight up to 0.5 and only flips at 1.0, which switches the middle arm off.
Food contact is the inside of vessels, the top of plates, the spoon bowl, the fork tines and
the knife blade; handles never count. A flat plate holds water and is infeasible; on edge it
drains.

## Next

The scorer is closed at revision 5 and is the objective the arrangement planner uses. If
the chart reads right, the next big step is a real wash test of the two random_06
arrangements (two afternoons).
