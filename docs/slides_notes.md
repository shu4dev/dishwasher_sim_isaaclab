# v0 slides notes (paste-ready)

- v0 delivered end-to-end: object-in-gripper -> OMPL RRT-Connect -> collision-free
  execution in Isaac Sim -> release -> stable placement in the dishwasher's lower rack.
- **2/2 trials succeed (100 %)** across 1 slots x 1 RNG seeds; failures are dominated by goal-infeasible deep-interior slots (reach limit), not planner or execution errors.
- Plan times: mean 0.33, median 0.33, p95 0.47, max 0.38 s against a 5 s budget — RRT-Connect over a custom FCL collision world at ~0.2 ms per collision query.
- The collision world is a standalone, simulator-free module (needed for the future
  MCTS rearrangement planner): CoACD-decomposed rack (128 pieces keeps the wire gaps
  open), validated at 98 % agreement vs Isaac contact ground truth with zero
  non-conservative mismatches.
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
- Known simplifications: mug stands on the wire basket floor (the ArtVIP rack has no
  plate tines); grasp *acquisition* is out of scope (the mug starts already pinched,
  with a wrist weld carrying the load for rigidity during planned motion).

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
- docs/figures/best_trial_02_00_1.png — curated final still (trial_02_00_1)
