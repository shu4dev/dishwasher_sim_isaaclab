# Reorganize code/ into the repo map's six blocks
Slug: 2026-11-07-code-blocks-refactor | From: docs/repo_map.txt (blocks 0, 1, 1a, 1b, 2a, 2b, 3a, 3b)

## Goal
Every file under code/ sits in the block folder that owns it, as six packages named like the folders plus util/, with every import, sys.path line, path string and root derivation following it; behaviour unchanged.

## Decisions (from grilling)
- Six folders under code/: asset, sampler, motion_planning, arrangement_optimization, paths, optimal_arrangement, plus util/ (launchers, docker, tools, the Kit core). Rejected a tools/ folder at the repo root: the CLAUDE.md commands name code/util.
- Each folder is the package of its name (code/sampler/__init__.py, `from sampler.hwgate import ...`); entry scripts in `<block>/scripts/`, tests in `<block>/tests/`, package docs in `<block>/docs/`; the scorer (1a) and the robot (1b) are the sub-packages sampler.scorer and sampler.robot with the same layout. Rejected keeping dishsim_frigidaire whole (the robot would sit in asset/) and a dishsim_ prefix (nothing on Kit's path is named like a block).
- The Kit core keeps its name and its importers: code/util/dishsim (app, media, quats, config, geometry, transforms, checks, collision_world). The twin's runtime (twin, geometry, paths, loading, tableware, load_validation, organization) is asset/. planner.py (the FCL mirror, support graph, greedy sequencer), rearrange.py and rrt.py are motion_planning/.
- paths/ owns run (episodes.py), video.py and report.py; optimal_arrangement/certify.py takes track B's own_gate, own_load and final_score out of episodes.py, which imports them back. Rejected a shared output/ folder (a seventh folder) and duplicated copies.
- bench.py and frigidaire_bench_kit.py stay whole in sampler/scripts; the split by mode is a later round.
- The user commits the working tree first; the refactorer works on a worktree branch and commits there; the box test is a later round (the user's call: refactor first, test later).
- The old and new path of every file is plans/2026-11-07-code-blocks-refactor-map.tsv (180 rows; "retire" rows fold into a block README). The plan is the rule, the map the list.

## Change
- Move every file as the map says (git mv). New files: an __init__.py per package and sub-package (asset, sampler, sampler.scorer, sampler.robot, motion_planning, arrangement_optimization, paths, optimal_arrangement; dishsim_frigidaire's lazy `__all__` re-exports move to the package that now holds each module), optimal_arrangement/certify.py, a README.md per block folder (what the block is, its scripts, which row of docs/repo_map.txt it serves; the old stage READMEs fold in), code/README.md rewritten as one table row per block.
- Rewrite every import of dishsim.rearrange, dishsim.rrt, bench.*, dishsim_frigidaire.* and the script-to-script imports to the new packages; every sys.path line to `[ROOT / "code", ROOT / "code/util"]` plus the script's own folder where scripts import each other; every `code/...` path string in python, shell, compose, Dockerfile comments, pyproject, README.md, docs/architecture.md and docs/environment.md; every `parents[n]` and `".."` root derivation to the file's new depth (65 sites); pyproject `packages.find` where = ["code", "code/util"], package-data per new package (asset: *.json; sampler: schema/*.json), testpaths = the seven tests folders.
- Untouched: CLAUDE.md, docs/repo_map.txt, docs/maps/ (this session hands over their new lines), data/, results/, plans/, research/, the docker image.

## Run
Before moving anything, in the worktree: `ln -s /Users/brianshu/workspace/dishwasher_sim_isaaclab/data data` (data/ is gitignored) and record `PYTHONPATH=code/src:code/frigidaire/src /Users/brianshu/workspace/dishwasher_sim_isaaclab/.venv-kitfree/bin/python -m pytest --collect-only -q code/tests code/frigidaire/tests | tail -1`. After: `P=/Users/brianshu/workspace/dishwasher_sim_isaaclab/.venv-kitfree/bin/python; $P -m compileall -q code && PYTHONPATH=code:code/util $P -c "import asset, sampler, sampler.scorer, sampler.robot, motion_planning, arrangement_optimization, paths, optimal_arrangement, dishsim" && PYTHONPATH=code:code/util $P -m pytest --collect-only -q code | tail -1`

## Works when
Nothing is left under code/src, code/planner, code/initialization, code/execution, code/frigidaire or code/util/frigidaire; `grep -rn "code/src\|code/planner\|code/execution\|code/initialization\|code/frigidaire\|dishsim_frigidaire\|from bench\|import bench" code docs/architecture.md docs/environment.md README.md pyproject.toml` finds nothing; compileall and the nine imports pass; pytest collects the same number of tests as before the move.

## Assumed / skipped
- Assumed: the Mac's Kit-free venv collects the tests (any collection error present before the move is not the refactor's); Kit scripts change only in import and path lines, boot order untouched; the docstrings and comments that name old paths are rewritten with the paths.
- Skipped this round: running the tests and the box smoke, the split of the two multi-mode scripts, CLAUDE.md and the repo map.
