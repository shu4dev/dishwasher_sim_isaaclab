# Sampler success rate on the easy and medium presets, per stage
Slug: 2026-10-22-sampler-rate-easy-medium | From: results/2026-10-22-window-sampling-v7-and-section2.md (follow-up, no lit review)

## Goal
Measure, per difficulty preset, how often the goal sampler succeeds at each stage when the load is smaller than the
24-dish hard case: easy (7 bowls) and medium (8 plates + 7 bowls), 20 seeds each, through the new `sample.py` goal
path. Compared against today's hard figure (0 valid of 40 packings, 39 stopped in the sampler at one dish's landing),
this says whether the landing failures come from crowding or from the move itself.

## Decisions (grilled 2026-10-22 08:25 ET)
- Pipeline: `code/util/run_py.sh code/src/sampler/sample.py --difficulty <easy|medium> --from build --to certify --seed <s>
  --n-samples 1 --max-attempts 1 --out data/results/sampler/rate_<difficulty>` (the preset counts; each seed packs its
  own restricted load, then the kit's build, the joint gate with the lower-first retry, and the fresh-process witness).
  The `initial` stage is skipped: the rate is about the goal build. Rejected: the old kit loop with a cut roster.
- Seeds 0 to 19 for both difficulties (the same seeds). Hard is NOT rerun: the comparison uses today's v7 figure.
- Success is reported per stage, k of 20 per difficulty: packer found a packing; the sampler placed every dish (build
  PASS); joint gate accepted; witness rebuilt (certify PASS = valid). For every failed seed: the stage, the dish and the
  failure text (landing / moved / no free pose / gate outcome / rebuild count).
- Box: 4 processes at once (one per core pair `j,j+4`, j = 0..3, nice 10, `OMP_NUM_THREADS=2`), a shell scheduler that
  hands seeds to free slots; easy first, then medium. Time cap 2 h of box; stop it as soon as the last seed ends.
- Known trap (results/2026-10-22-repo-refactor-sampler.md, item 3): the kit's verdict strings say "24 of 24"; the
  counts must come from the record, not the text. If the kit or the certify step hardcodes 24 anywhere that changes a
  verdict, fix that one spot and say so in the note.

## Change
- No repo code unless the 24 trap bites. A scheduler script under `data/outputs/sampler_rate/` (gitignored) and a
  Kit-free tally script that reads every pair file's `stages` ledger and the kit records into one table.

## Run
On `dishsim-l4` (CLAUDE.md "Research workflow (repo)": `brev start`, `git pull --ff-only` on `plan`, the commands above
through the container, `brev stop` right after; confirm STOPPED with `brev ls`). Judge every stage from the pair file
and the kit's `[RESULT]` lines, never exit codes. Outputs `data/results/sampler/rate_{easy,medium}/`, logs
`data/logs/sampler_rate/`.

## Works when
`results/2026-10-22-sampler-rate-easy-medium.md` holds the per-stage table (easy, medium, and hard's 0 of 40 from v7),
the failed-seed list with reasons, the box window times, and one line on what the rates say about crowding versus
the move; the box is STOPPED at the end.

## Assumed / skipped
- Assumed: the four manifests exist on the box (written 10:36 UTC today; `--to load` passed) and the top-5 level-0 masks
  cache is present (it was for today's goal-path seeds). Skipped: the initial stage, hard, clips, pages, any sampler change.
