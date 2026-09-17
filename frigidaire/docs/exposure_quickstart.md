# Exposure scorer: quickstart

Run this first (3 minutes, no GPU):

```
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_demo.py --pair random_06 --device cpu
```

Then open `results/exposure/frigidaire/demo_random_06/evidence.png`.

## What you are looking at

- Top row: Isaac renders of the two arrangements, same 18 objects.
- Bottom row: every bowl and mug interior coloured by exposure. Bright = water from the
  spray arm can reach it. Dark = something is in the way.
- Red circle = a vessel that holds water (cannot drain). One red circle makes the whole
  arrangement infeasible.
- Title numbers: `score` = average exposure over all food-contact surfaces. `worst` = the
  single worst object. Higher is better. Compare only arrangements of the same objects.

## The three commands

1. One pair, all pictures (3 min): the command above. Also writes `rays.gif`, the spray
   arm rotating under one bowl with green (open) and red (blocked) rays.
2. All seven pairs, one chart (6 min):
   `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_summary.py --device cpu --ceiling-sweep --convergence random_06`
   → `results/exposure/frigidaire/summary/summary.png`, `ceiling_sweep.md`, `convergence.md`.
3. Find a better arrangement (5 min):
   `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_search.py --pair random_06 --device cpu`
   → `results/exposure/frigidaire/search/hist.png` and `best.png`. Proposals are not
   settled; settling the best takes 4 min in Isaac (steps in `docs/exposure.md`).

## Numbers to remember

| | score |
|---|---|
| random packing (random_06) | 0.075, 8 of 18 vessels hold water |
| hand-organized | 0.242 |
| best found by search, settled and reproduced | 0.274 |

Organized beats random in all 7 pairs by 0.12 to 0.22.

## What the number assumes

Water travels in straight lines from a disc under each rack (where the arm sweeps), hits
harder when it hits squarely, and nothing else. No splash. The ceiling nozzle counts only
if you set `--ceiling-weight`; at weight 1.0 the ranking flips, so check the tub ceiling
before quoting anything with it.

## Next

Open `results/exposure/frigidaire/summary/summary.png`. If the chart reads right, the next
big step is a real wash test of the two random_06 arrangements (two afternoons).
