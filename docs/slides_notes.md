# v0 slides notes (paste-ready)

- v0 delivered end-to-end: object-in-gripper -> OMPL RRT-Connect -> collision-free
  execution in Isaac Sim -> release -> stable placement in the dishwasher's lower rack,
  validated in BOTH robot-facing rack states (both racks out / both racks in).
- **24/24 trials succeed (100 %)** across 12 scenario-slots x 2 RNG seeds; failures are dominated by goal-infeasible deep-interior slots (reach limit), not planner or execution errors.
- Plan times: mean 2.67, median 1.47, p95 7.31, max 7.61 s against a 5 s budget — RRT-Connect over a custom FCL collision world (both_out: 100.0 % @ 0.83 ms/query; both_in: 100.0 % @ 0.73 ms/query).
- Both racks are PROCEDURAL realistic wire racks (rack_gen v2, styled after Whirlpool
  WDTA50SAKZ / Bosch 800 / Frigidaire FDPC4314AS): 3-gauge wire hierarchy, 30 mm
  Whirlpool-pattern tine rows with candy-cane ends, dark fold-down insert row, roller
  wheels, dipped front rail + grab handle, cup shelves + RackMatic blocks up top. The
  FCL pieces are the generator's exact convex parts — zero decomposition slop.
- The derived scene raises the upper rack 120 mm: the ArtVIP asset gives real-scale
  dishware only 154 mm of inter-rack clearance where real machines give 250-300 mm —
  the raise restores a real loading geometry (274 mm) and is what makes the retracted
  (both_in) state loadable at all.
- Analytic UR5e IK (8 branches, hand-rolled; ur-analytic-ik has no py3.12 wheel)
  matches Pinocchio to 1e-9 and live Isaac FK to 0.2 mm — goal sets are thousands of
  configs per feasible slot, with per-slot rejection funnels as reach-feasibility maps.
- Found and fixed via the parity gate: attached-object-vs-arm collisions must be
  world-checks (PhysX simulates them), and teleported ground-truth sampling needs
  mimic-consistent finger states — both would have silently corrupted the benchmark.
- The mug rides in a calibrated contact pinch: the jaws visibly close onto the rim
  band at trial start (stop-at-contact aperture from a measured force-vs-angle curve,
  ~5 N per pad) and visibly open before the hidden weld releases, followed by a
  collision-validated tool-axis retract (see docs/grasp_calibration.md).
- Known simplifications: the mug stands in the lower rack's open zone (plate loading
  between the tines is future work); grasp *acquisition* is out of scope (the mug
  starts already pinched, with a wrist weld carrying the load during planned motion).

## Best figures/clips (by path)

- docs/figures/success_by_slot.png — headline result
- docs/figures/outcome_breakdown.png — failure taxonomy
- media/F/trial_02_00_0.mp4 — full plan-execute-release clip (first end-to-end success)
- media/E/accepted_slot1_sheet.png — goal-config diversity at one slot
- media/D/overlay_E_shelf_1_04.png — rack decomposition, wire gaps preserved
- media/C/rigidity_iso.mp4 — pinched-object rigidity proof (pad forces monitored)
- media/C2/close_open.mp4 — calibrated pinch: close to contact, hold, open, forces vanish
- media/C2/force_vs_theta.png — the measured pinch force-vs-aperture curve
- docs/figures/best_trial_02_00_0.png — curated final still (trial_02_00_0)
- docs/figures/best_trial_02_01_0.png — curated final still (trial_02_01_0)
