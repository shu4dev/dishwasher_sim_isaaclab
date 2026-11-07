# Can this repo run on any dishwasher and any dish asset?
Slug: 2026-11-01-general-assets | Lit review: none (quick check only, no file)

## Question
Which steps of the pipelines can take a new dishwasher or dish set as data alone, and which cannot.

## What's known (max 5)
- Two read-only audits on 2026-11-01 (dishsim core: 69 rows; Frigidaire side: 68 rows). Classes: A = read from an asset
  file, B = a python literal a manifest field can replace, C = a method tied to a rack feature or dish kind, D = a number
  someone measures per asset, E = a frozen cache key, pin or sample.
- Bosch core: no manifest is read anywhere. Every machine and dish fact is a python table in `code/src/dishsim/config.py`;
  `machine.py`, `export_usd.py` and `gen_instances.py` branch on the string `bosch800`. About half the rows are B; 9 are C
  (rack_gen's two builders, rack_layout arithmetic, plate slots from one uniform tine row, basket and tray slots, the
  flat-wire-floor grid, four unsupported modes, USD authoring per machine family); 12 are D.
- Frigidaire side: 36 B, 31 C, 19 D, 8 E, 12 A. A manifest layer exists (`machine.json`, `<kind>.json`, schema in
  `code/src/sampler/schema/`) and one sampler stage of eight reads it. `machine.json` is generated from the python
  constants by `code/src/sampler/writers/`, so editing it changes nothing downstream.
- Already machine-agnostic: the sampler stage runner and loader, the physical graph, the FCL checker, the insertion
  descents, the greedy sequencer, the ray engines, the joint gate loop, the MCTS core, the compat solver and the
  rearrange driver.
- Stale items the audits found: the Frigidaire README's rack travel (-0.49/-0.44; the twin ships -0.5425/-0.5195),
  `robot/grasp.py` bowl numbers (pre-v4), two counter widths (1.8 and 1.2 m), the bench docstring's slacks (n+3/n+1/n+0
  against TIERS 3/2/1), bench masks cached by family digest only, the `config.py:1278` comment on DEFAULT_BASE_PLACEMENT.

## Gap
"Any dishwasher or dish" is impossible as data alone. Five steps stop it:
1. A bare USD carries no semantics. Which link is a rack, which joint slides, where the arms, tub and counter are: no
   code can infer these, so the floor is USD plus a manifest that names them.
2. Slot derivation is per rack feature. Implemented: tine rows with explicit columns (`between_tines`, `over_tines`,
   `centre_gaps`), the glass channel, the Bosch floor grid, plate slot, basket bay and tray zones. Not implemented:
   baskets as containers (the schema admits them, the slot finder skips them), cutlery, fold-down tines, cup shelves,
   bottle holders, mesh racks without a tine grid. Each is a new mode in code.
3. The score reads a dish as an axisymmetric open vessel: food contact = faces toward the lathe axis or facing up,
   pooling = interior more than 2 mm below one rim, ceilings per kind (plate, bowl, cup) per arm. Cutlery has its own
   contact code; handles, lids, square plates and stems break the rule. A new dish family is a new score revision.
4. Every number is measured. A new asset needs its tape-measure campaign (racks, tines, arms, travel, door, tub, counter;
   dish dimensions, masses, friction) and its PhysX re-measurements (settle reliability per state and kind, plate-slot
   tolerances, fault knobs, counter-hull hover, collision margin, CoACD params). The PhysX ones can become onboarding
   probes; the tape ones cannot.
5. S ranks arrangements of the same dishes in the same machine under its own arms. Comparing S across machines or dish
   sets means nothing by construction. Instances are per machine and per PhysX version. The robot block stays closed:
   its three grasps are kind-specific by design (decision, round 1).

## Candidate ideas (max 3)
1. Declared modes: the manifest names the mode each container and dish kind uses; a new feature is a new mode, one
   round each. Might work because the modes already exist as code. Might fail if the schema grows faster than the
   readers. Cheapest test: Kit-free byte-identical outputs on stored records after the readers switch.
2. Sampling replaces slots: poses sampled, FCL-checked, settle-gated for any rack. Any geometry works, but the capacity
   plan, the compat optimum and the insertion move need new definitions on sampled poses. Cheapest test: one easy pair
   built without slot semantics, certified by the joint gate.
3. Stop at the flags: no generality round.

## Picked
1, because the modes exist and the plumbing is half the rows. Decisions (2026-11-01): declared modes; two cache-key
schemes (the Bosch keeps its frozen anchors and 14 caches, manifest-defined assets hash their manifest plus CoACD
params); declared kinds now, kind-free methods as a later score revision; one schema for both stacks, Frigidaire side
first (plans/2026-11-01-frigidaire-manifests.md), the Bosch core the round after; the upgrade round runs before either.
Plan grill (same day): tiers live in a pair file under the machine folder; the writers move to each asset's `source/` and
read only the asset's own files; Kit-free byte identity on stored records is the proof; the Bosch core adopts
`machine.json` in its round, with the frozen anchors kept in a `frame: robot_mount` block so `config_hash` stays.

## Revised the same day: one benchmark, one machine, one set
The user merged the two benchmarks instead (plans/2026-11-01-merged-benchmark.md): the Frigidaire twin with the
HOTEC set is the only pair, the Bosch and ArtVIP twins and the YCB props leave the repo (unused, and every merge
step above was their cost), so the declared-modes programme and the manifests round are not needed; the sampler's
score-greedy load is the goal and the ground truth, tiers are three labels, the bench's fault rule and four
algorithms stay, everything runs on 6.2. The five flags still hold for anyone who adds a machine or a dish set.

## To grill (moot after the revision)
- The v2 field names, once the rewritten writers show what the measured files can fill without typing a number.
- Whether the Bosch anchors block fits the schema's `frame` flag or needs its own section.
