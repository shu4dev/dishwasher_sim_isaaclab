# Labelled multi-angle load views, the gate's overlap rule in the sampler, r07_00012 again
Slug: 2026-10-16-labelled-views-overlap-rule | From: results/2026-10-16-physical-cause-graph.md (follow-up, no lit review; one-hour budget)

## Goal
Each valid sample's load image shows the load from four angles with every dish labelled by its id, the sampler applies the joint gate's 1 mm resting-overlap rule so a load it accepts cannot fail the gate's preflight, and r07_00012 is rerun through the whole chain.

## Decisions (user request, 2026-10-16; no grilling, one hour)
- Load image = one PNG with four labelled views: the oblique view as now, a top view of each rack (vertical drops are along the rack's up axis, so a top view shows whether descent paths cross), and a front view; each dish carries its id (p01, b03, c05...) at its centroid in the view, white text with a dark outline. Same Warp renderer, same tints.
- Sampler: after each settle the placed dishes are checked pairwise with the gate's preflight rule (collision pieces overlapping more than 1 mm); a drop that leaves such a pair is undone and re-posed like a failed landing. Everything else unchanged.
- r07_00012: sampler with the rule, gate, fresh-session rebuild, edges, confirmation drops, images, video; valid if all pass. r05_00027: only its load image is re-rendered. No other packings this round.
- Budget: 60 min in all from the start; the box window at most 30 min; whatever is unfinished at the budget is reported as is.

## Change
- `frigidaire_bench_kit.py` `build_sequence`: the pairwise overlap check after each settle (the checker's preflight on the settled poses), undo + re-pose on a hit.
- `frigidaire_dependency_clips.py` `load.png`: 2 x 2 views (oblique, top lower rack, top upper rack, front) with id labels.
- `frigidaire_dependency_page.py`: unchanged apart from the image.

## Run
On the box: the sampler command of the parent on r07_00012 only (`--max-packings 1`, `--out-dir data/results/dot_rules/dependency/samples_v6`), then clips for r07_00012 and r05_00027, page, publish to the same URL.

## Works when
The page shows labelled four-view load images for every valid sample; `samples_v6/r07_00012.json` is valid (gate accepted, certified, edges confirmed) or the run note names the first failing item; the run note gives the budget actually used.

## Assumed / skipped
- Assumed: the overlap check uses the same convex pieces and 1 mm threshold as `frigidaire_initial_state_validate`'s preflight; labels use the short ids; the top views hide the other rack.
- Skipped: new packings, the descent-path overlay, anything beyond the hour.
