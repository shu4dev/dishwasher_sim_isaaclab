# Run note: a pre-build plate pose, the foot pivot to first rack contact, against the settled plates (box dishsim-l4, 2026-10-22)
From: plans/2026-10-22-plate-pose-model.md. Ran on the box (16 s on 8 workers): `code/util/run_py.sh code/src/sampler/order_check.py --records data/results/dot_rules/dependency/samples_v5/{r05_00027,r00_00038,r07_00012}.json data/results/dot_rules/dependency/samples_v7/r02_00017.json --models F,A,S --out data/results/sampler/plate_pose_model` (records written out in full), giving `[RESULT] PASS 12 checks, 0 errors, 16 s`. The full tables, with dx dy dz, the lowest point, and the stop lift and pivot per plate, are in `data/results/sampler/plate_pose_model/table.md`.

- **Records with settled plates:** 4 in samples_v5, v7 and v6b, on the Mac and the box. They are r05_00027 (valid), r00_00038 and r07_00012 (v5, built but not certified) and r02_00017 (v7, invalid). On the box, v6b is empty; on the Mac, its only record failed to re-settle. Together they hold 32 plates: 27 front-bank plates that settled in the gap, and 5 rear-bank plates that settled on the tines (three on r05, two on r00).
- **Hits on r05 (9 plate hits, a>b = a before b):** F passes: 9 of 9 hits, 0 spurious, no plate cycle, and all 11 hits on the full graph, which is acyclic. A gets all 9 hits but adds 3 spurious rear-bank edges: plate_01>plate_05, plate_01>plate_07 and plate_07>plate_05 (no cycle). S passes.
- **What F does:** every front plate stops at a 90 mm lift and pivots 24° before it touches the LowerRack (tilt 114°). Every rear plate stops at 85 mm and pivots only 2° (tilt 92°). So F and A leave all 32 plates on the tines, with their lowest point 87-92 mm above the plate floor.
- **Where the plates actually settle:** the front plates sit down in the gap (-4.7 to 1.4 mm, tilt 117-122°). The rear plates sit at 72 mm and 83.4°, leaning 6.6° away from their lean side, which a pivot toward the lean side cannot produce.
- **Where F's error comes from:** front errors are 92-99 mm of height, plus about 10 mm in y from FRONT_PLATE_INSET_Y. Rear errors are 18 mm in x and 21 mm in z.
- **Verdict:**
  - **F: FAIL.** The hits pass, but 0 of 32 plates are within 3° and 10 mm. Worst tilt 9.4° (r07_00012 plate_01), worst position 100.1 mm (r05_00027 plate_04).
  - **A: FAIL.** 3 spurious edges, and 0 of 32 plates within the limits (12 of the 27 front plates are within 3° of tilt). Worst 34.6° and 100.3 mm.
  - **S: PASS,** 0.0° and 0.0 mm by construction.
- **What the sampler should use for plates:** the settled pose (S) wherever a pose is needed, since neither pre-build model lands a plate where it rests. F can stand in for the plate-pair access edges only: exact on r05, but that is one load and 9 edges.
- **Observation, not a model run:** each bank's settled plates share one pose relative to their gap.
  - Rear: 5 of 5 match to 0.1 mm and 0.1° (gap mid -31.2 mm, z 187.3 mm, 83.4°).
  - Front: 22 of 27 are within 3° and 10 mm of the bank median (gap mid +23.4 mm, the row-mid y, z 119.0 mm, 122.0°). The 5 outside are all on r07_00012: four lean only 117-119°, and plate_01 slid 29 mm in y.
  - A template like this would be fitted to these same 32 plates, which this round ruled out.
- **Per plate (rack-local; mm = distance between pose origins):** F and A put every plate on the tines. S is 0.0° and 0.0 mm on all 32.

| load | plate | slot | lean | settled tilt, mode | F tilt, err, mm | A tilt, err, mm |
|---|---|---|---|---|---|---|
| r05_00027 | 01 | rear_08 | -28 | 83.4, tines | 92, 8.6, 28.1 | 118, 34.6, 76.5 |
| r05_00027 | 02 | front_01 | -32 | 120.8, gap | 114, 6.8, 99.5 | 122, 1.2, 99.8 |
| r05_00027 | 03 | front_04 | -28 | 122.2, gap | 114, 8.2, 100.0 | 118, 4.2, 100.1 |
| r05_00027 | 04 | front_03 | -28 | 122.2, gap | 114, 8.2, 100.1 | 118, 4.2, 100.1 |
| r05_00027 | 05 | rear_04 | -24 | 83.5, tines | 92, 8.5, 28.0 | 114, 30.5, 68.9 |
| r05_00027 | 06 | front_00 | -28 | 119.7, gap | 114, 5.7, 99.1 | 118, 1.7, 99.2 |
| r05_00027 | 07 | rear_06 | -24 | 83.4, tines | 92, 8.6, 28.1 | 114, 30.6, 69.0 |
| r05_00027 | 08 | front_05 | -28 | 122.2, gap | 114, 8.2, 100.0 | 118, 4.2, 100.1 |
| r00_00038 | 01 | front_04 | -24 | 122.2, gap | 114, 8.2, 100.1 | 114, 8.2, 100.1 |
| r00_00038 | 02 | rear_08 | -28 | 83.5, tines | 92, 8.5, 28.0 | 118, 34.5, 76.4 |
| r00_00038 | 03 | front_06 | -32 | 122.2, gap | 114, 8.2, 100.1 | 122, 0.2, 100.3 |
| r00_00038 | 04 | front_02 | -28 | 122.0, gap | 114, 8.0, 100.0 | 118, 4.0, 100.0 |
| r00_00038 | 05 | front_00 | -28 | 119.6, gap | 114, 5.6, 99.1 | 118, 1.6, 99.2 |
| r00_00038 | 06 | rear_04 | -24 | 83.4, tines | 92, 8.6, 28.0 | 114, 30.6, 69.0 |
| r00_00038 | 07 | front_05 | -24 | 122.2, gap | 114, 8.2, 100.0 | 114, 8.2, 100.0 |
| r00_00038 | 08 | front_01 | -28 | 120.9, gap | 114, 6.9, 99.5 | 118, 2.9, 99.6 |
| r07_00012 | 01 | front_01 | -24 | 120.6, gap | 114, 9.4, 94.6 | 114, 9.4, 94.6 |
| r07_00012 | 02 | front_02 | -28 | 122.1, gap | 114, 8.1, 100.0 | 118, 4.1, 100.1 |
| r07_00012 | 03 | front_05 | -32 | 119.0, gap | 114, 5.0, 98.9 | 122, 3.0, 99.2 |
| r07_00012 | 04 | front_07 | -32 | 117.2, gap | 114, 3.2, 98.4 | 122, 4.8, 98.8 |
| r07_00012 | 05 | front_00 | -24 | 118.2, gap | 114, 4.3, 98.7 | 114, 4.3, 98.7 |
| r07_00012 | 06 | front_08 | -32 | 118.0, gap | 114, 6.7, 97.2 | 122, 6.6, 98.0 |
| r07_00012 | 07 | front_04 | -28 | 119.8, gap | 114, 5.8, 99.3 | 118, 1.9, 99.4 |
| r07_00012 | 08 | front_06 | -32 | 118.7, gap | 114, 4.8, 99.0 | 122, 3.4, 99.7 |
| r02_00017 | 01 | front_06 | -32 | 122.1, gap | 114, 8.1, 100.0 | 122, 0.1, 100.2 |
| r02_00017 | 02 | front_07 | -28 | 122.2, gap | 114, 8.2, 100.0 | 118, 4.2, 100.1 |
| r02_00017 | 03 | front_00 | -32 | 120.1, gap | 114, 6.1, 98.4 | 122, 1.9, 98.6 |
| r02_00017 | 04 | front_03 | -32 | 122.1, gap | 114, 8.1, 99.9 | 122, 0.1, 100.2 |
| r02_00017 | 05 | front_02 | -32 | 122.0, gap | 114, 8.0, 100.0 | 122, 0.0, 100.2 |
| r02_00017 | 06 | front_05 | -24 | 122.2, gap | 114, 8.2, 100.0 | 114, 8.2, 100.0 |
| r02_00017 | 07 | front_04 | -28 | 122.1, gap | 114, 8.1, 100.0 | 118, 4.1, 100.1 |
| r02_00017 | 08 | front_01 | -32 | 121.1, gap | 114, 7.1, 99.1 | 122, 0.9, 99.3 |

- **Deviations from the plan:**
  1. F's "first contact" is the last 1° step that stays within the 1 mm allowance against all five appliance bodies. The contact was always the LowerRack, and no plate reached the 60° cap.
  2. Position error is the distance between pose origins, the measure behind the plan's own numbers. The rest mode is my threshold: tines when the lowest visual point is more than 30 mm above the plate floor.
  3. Hits are compared on the valid record only (r05). "Plate cycle" is checked on the plate-pair access and support edges.
  4. I copied r00_00038 and r07_00012 to the box from the Mac. The box's r02_00017 and r05_00027 match the Mac's (sha256 012bb99b… and fca9c3b2…).
- **Files changed:** `code/src/sampler/order_check.py` (152 to 303 lines: models F and A, `plate_errors`, `cyclic`, the plate tables; hits judged on valid records only). The gitignored outputs are `data/results/sampler/plate_pose_model/` (12 JSON + table.md) and `data/logs/sampler/plate_pose_model.log`.
- **Box window (UTC):**
  - Start: the first `brev start` (17:33:18) stayed STOPPED, so I killed it. The second, at 17:35:48, reached STARTING at 17:36:55 and RUNNING at 17:37:27.
  - Run: ssh worked at 17:37:50 after `brev refresh` (the self-stop was already running). I copied the files at 17:37:59, ran 17:38:15-17:38:31 and copied back at 17:38:53.
  - Stop: I restored the clone (deleted the 2 untracked copies; clean at 6f39f0c) at 17:38:57 and sent `brev stop` at 17:39:06. STOPPED at 17:46:25 (`brev ls`: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`).
  - That is 2.2 min from STARTING to the stop command, and 9.5 min to STOPPED.
- **Missing:** bowls and cups (settled in every model), the chain (a plate leaning on its neighbour) and any sampler run, all skipped this round per the plan.

Suggested commit message: `Add plate pose models F and A to order_check`
