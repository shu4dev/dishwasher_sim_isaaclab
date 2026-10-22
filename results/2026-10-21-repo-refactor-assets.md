# Run note: repo refactor section 1, assets (2026-10-21, plan `plans/2026-10-21-repo-refactor.md` section 1)

- **Branch:** `worktree-agent-aebc7b64c2dd358c0`, off `plan` at `1290c6c`. Commits, one per step: `e6c8bdd` dishes
  (hotec_wheatstraw, bosch_props, frigidaire_prototypes), `93a592c` Frigidaire twin split, `17929e8` Bosch 800 +
  ArtVIP and the archive tooling, `7e3447d` docs, `d9130eb` hf CLI wording (coordinator's change: no git-lfs on the
  Mac), `6c76fa1` doc fix (where the v3-v5 builds and HOTEC v1 really are), then this note.
- **Works:** on the box, `data/assets` holds exactly the six flat folders plus `README.md`. Each folder has `source/`
  (the moved generator, plus a README with the exact command and the git SHA of its last run) and the files
  experiments load. The runtime reads only those:
  - per-state machine USDs (`dishsim.machine` no longer authors at import),
  - machine-keyed caches (`config.scenario_cache_dir` = `data/assets/<machine>/cache/<state>/<object>/`),
  - the Frigidaire `parameters.json` and `components.json` (`dishsim_frigidaire.geometry`, `asset`),
  - the prototypes and HOTEC folders.

  Kit-free checks: the Frigidaire runtime equals the moved generator bit for bit (parameters, tine grid, every
  component mesh, wire and solid, the scorer's triangles). `rack_layout` equals `rack_gen`. Every shipped USD's
  references resolve, and every machine USD's stamp matches the config. Both suites pass.

  **Not done, because the box would not start for window 3:** the three Kit checks, the moved tests, and the copy
  back to the Mac. From 00:29:46 to 01:47:30 UTC, more than 70 `brev start` calls each answered
  `Internal Server Error`.
- **Files (code repo):** 109 changed: 45 deleted, 63 modified, 1 added (`code/src/dishsim/rack_layout.py`), +928 / -16,113
  lines.
  - Moved out into `data/assets/<folder>/source/` (gone from this repo):
    - modules: `dishsim/{hotec_gen,prop_gen,rack_gen,usd_prep}.py`; `dishsim_frigidaire/{claims,usd_bootstrap,lower_rack_asset}.py`.
    - generator halves: of `dishsim/geometry.py` (`dump_cache`, now `cache_dump.py`); of `dishsim_frigidaire/geometry.py`, `asset.py` and `random_pose_assets.py`; `write_tableware` of `tableware.py`.
    - scripts: 18 Frigidaire initialization scripts, `extract_geometry.py`, `decompose_meshes.py`, `bosch800_asset_evidence.py`, `polish_lower_rack.py`, `lower_rack_polish_evidence.py`.
    - 2 Frigidaire docs and 10 test files (3 of them split).
  - Deleted: `code/util/tools/{archive_assets,restore_assets}.py`, `code/tests/test_archive_assets.py`, and bootstrap's restore step.
  - New in the dataset: `export_usd.py` (bosch800), `export_components.py` (frigidaire), `fetch_artvip.py`,
    `write_tableware.py`, and 13 READMEs.
  - Docs: 22 files, rewritten to "data/assets is a copy of the HF dataset: hf download to restore, hf upload to
    publish". CLAUDE.md is among them (named in the task); please review its asset lines before merging.
- **Data (box):** 115 moves, all listed in `data/outputs/refactor_20261021/restructure.json`. Folder sizes: bosch800
  273 MB, artvip_dishwasher_2 117 MB, bosch_props 58 MB, frigidaire_fdpc4221as 8.2 MB, hotec_wheatstraw 4.2 MB,
  frigidaire_prototypes 0.5 MB.
  - Exports (Kit-free, no rebake): 4 bosch800 state USDs authored (both_in, both_out, middle_out, third_out; the
    placement one was already there); the 4 ArtVIP ones were reused (stamps match); `components.json` is 6,513,015
    bytes.
  - Held for deletion in `data/_trash_20261021/` (MANIFEST.txt lists everything; undo = mv back; delete after
    2026-11-21):
    - the `frigidaire_fdpc4221as{,_v2}` twins and their delivery JSON,
    - the 4 stale bosch800 `mug` caches,
    - 16 unreferenced CoACD dirs,
    - the ArtVIP variants other than `dishwasher_2`, plus the HF download cache,
    - `model_dishwasher_2_rl.usda`.
  - Evidence moved to `data/media/bosch800_asset/evidence_20260910/`.
  - Not yet done: copying the tree back to the Mac (`assets_back.sh` also moves the Mac's old `data/assets/models`
    and `data/build/frigidaire_collection` to the Mac's `data/_trash_20261021/`).
- **Box windows (UTC):**
  1. `brev start` 22:14:25. SSH up 22:23 after a host change (`brev refresh`). `brev stop` 22:28:39. Work:
     inventory, archive restore with the old tooling, old-code baselines.
  2. `brev start` 23:24:31 hung on capacity. Retried 23:47:48; STARTING 23:48:11, SSH 23:49:43. Work, done by
     23:52:40: old-code cache baseline, sync, restructure, exports, checks, both suites. I then went quiet, and the
     box's own self-stop halted it after 30 idle minutes (STOPPING seen 00:28:36). That cost about 35 idle billed
     minutes.
  3. Never started (`Internal Server Error`, see above). The box is STOPPED.

  Billed: about 55 of the 120-minute cap.
- **Checks, in order:**
  1. `code/util/run_py.sh -m pytest code/tests/ -q`: `37 passed in 40.35s`. Before the change: `1 failed, 64 passed in
     70.07s`; the failure was `test_rack_gen_frozen.py::test_v1_geometry_byte_identical`, a digest pinned to corallab's
     numerics, now in `bosch800/source/`.
  2. `code/util/run_py.sh -m pytest code/frigidaire/tests/ -q`: `222 passed, 24 subtests passed in 13.97s`. Before:
     `307 passed, 41 subtests passed in 48.04s`; 85 tests moved with their generators.
  3. Bosch `placement` instance from the loaded rack USDs: **not run.** The baseline is there: in window 1, the old
     code regenerated `perturbed_s0` byte-identical to the pinned file (`cmp` clean, `[RESULT] PASS`). It used the
     same `model_bosch800_v0_placement.usda` that now ships; that file's stamp was checked.
  4. HOTEC bench episode: **not run.** The box holds no bench instance; `w3_jobs.sh` generates `easy_s0`, then runs
     `greedy_offline`.
  5. `frigidaire_dependency_kit.py --sample --edges physical --max-packings 1`: **not run.**
  - Moved generators' tests (run once, in `source/`): **not run.**
  - Kit-free equivalence (`data/outputs/refactor_20261021/w2_checks.json` on the box):
    - PASS: (a) Frigidaire, (b) rack layout, (d) USD references across all shipped USDs, (e) the 9 machine stamps.
    - PASS: (f) imports: 8 generator modules are not importable, every runtime module imports, and the bench's HOTEC
      registration and tub ceiling are unchanged.
    - FAIL: (c) caches. Expected for the ArtVIP ones: the restructure found exactly one missing piece dir, ArtVIP's
      `E_door_4_e03c5413b02bbf19`, and the old-code baseline (`oldcache.json`) failed all 10 ArtVIP caches on it while
      loading all 15 bosch800 ones. The new per-cache lines are in `w2_checks.json` on the box, not fetched before it
      stopped, so "identical to the baseline" is unconfirmed; the bosch800 placement caches load (check 1's
      `test_compat` known-optima test uses them).
- **To finish (Mac, one window):** `WT=<checkout with this branch> bash data/outputs/refactor_20261021/finish.sh`. The
  scripts and the staged `source/` trees are kept on the Mac in `data/outputs/refactor_20261021/`. `finish.sh`:
  1. retries `brev start` and syncs the code;
  2. runs `w3_main.sh`: the Bosch instance plus `cmp`, the moved tests, and a listing. It also schedules a halt 58
     minutes out, which is the cap;
  3. starts `w3_jobs.sh` (bench `easy_s0` plus one episode, and the dependency packing). It halts the box 10 minutes
     after both finish;
  4. copies `data/assets` back and fetches the logs into `data/outputs/refactor_20261021/box/`;
  5. ends with `brev stop`.
- **Publish (Mac, code repo root; not run):** do this after the copy-back, with `hf auth login` holding write access to
  `shu4dev/dishsim-assets` (`hf` = huggingface_hub >= 0.34):

  ```bash
  hf upload shu4dev/dishsim-assets data/assets . --repo-type dataset --commit-message "Six flat asset folders with their generators (code repo refactor, section 1)"
  hf repo-files delete shu4dev/dishsim-assets "dishsim_*.tar.gz" latest.json --repo-type dataset
  ```

  To restore anywhere: `hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets`. The hub keeps
  the 12 tarballs and `latest.json` in its history.
- **What the plan missed:**
  1. The box held only HOTEC v2 and the Frigidaire USD build. The Bosch caches, props, ArtVIP, the standalone Bosch 800
     and its evidence existed only in the HF archive. Window 1 restored tag `20260910_b3584ae` with the old
     `restore_assets.py`, before deleting it: assets 137 MB, models 3.5 MB, evidence 74 MB, plus the ArtVIP originals
     at 85 MB.
  2. No HOTEC bench instance exists on the box, so the bench check has to generate one into the bench's own folder.
  3. Three "generator-only" modules have runtime users:
     - `claims.py`: the bench's `slabs_disjoint`, and the HOTEC planner's bowl and floor helpers. Moved verbatim into
       `loading.py`.
     - `random_pose_assets.py`: `_pose` in the bench checker, and `LiveCollisionWorld`. Both moved into
       `random_pose_runtime.py`; only `validate_inputs` left the repo.
     - `usd_bootstrap.py`: `ensure_usd()` ran at import of `frigidaire_hotec_exposure_search.py`. The call is dropped:
       `run_py.sh` supplies `pxr`, and nothing Kit-free needs `PhysxSchema`.
  4. More mixed modules than the plan names:
     - `dishsim/geometry.py`: `dump_cache` went.
     - `tableware.py`: only the writer went. `CATALOG` and `tableware_geometry` stay, because the exposure scorer
       and `organization.py` read them.
     - `collision_world.py`: its rack piece-count check called `rack_gen.build`, so it is dropped.
  5. The runtime called the Frigidaire builders: `build_components()` for the scorer's rack triangles and wires, the
     sampling domains and the bench's tub ceiling, plus `_upper_rack()` at import of `frigidaire_hotec_load.py`.
     Their output was exported once as `frigidaire_fdpc4221as/components.json` (float-exact) and is read by
     `geometry.components()`.
  6. `lower_rack_asset.py` and its two scripts polish the standalone Bosch 800, not the Frigidaire, so they live in
     `bosch800/source/`. The standalone asset itself, absent from the plan's layout, is in `bosch800/usd/standalone/`.
  7. `usd_prep.py` authors both machines from `rack_gen`, so it stays whole in `bosch800/source/`;
     `artvip_dishwasher_2/source/` holds `fetch_artvip.py` and points there. The derived ArtVIP USDs reference the
     originals, so the originals ship too (Apache-2.0).
  8. Every ArtVIP cache lacks its `E_door_4` pieces under the current `COACD` params. This was true before the move.
  9. `bosch_props` holds 15 physics USDs, not 6, and the 10 YCB-derived ones are CC BY 4.0, not self-authored.
  10. The robot mirror has no folder among the six. It is `data/assets/robots/`, on corallab only, absent on the box.
      `ur5e.py` and `mirror_robot_usd.sh` still point there.
  11. Records written before the move:
      - `dishsim.legacy_data_path` maps their old asset paths.
      - Their `appliance_sha256` maps still list `tableware/*.usdc`, so a `frigidaire_hotec_load.py` replay of a
        pre-move layout fails its hash gate.
  12. `frigidaire_hotec_load.py` defaulted to HOTEC v1, which was dropped; its default is now the v2 folder.
  13. git-lfs can't be installed on the Mac, so the dataset is a plain folder synced with `hf`.
  14. The Frigidaire references, images, validation and build history (v3-v5) were only ever on corallab, never in
      the HF archive. "Git history only" holds for their code, not for the builds.
  15. A quiet stretch of mine cost about 35 billed idle minutes in window 2. The window-3 scripts schedule their own
      halt.
- Suggested commit message: `Move generated assets into six flat asset folders`
