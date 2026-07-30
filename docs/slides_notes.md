# v0 slides notes (paste-ready)

- v0 delivered end-to-end: object-in-gripper -> OMPL RRT-Connect -> collision-free
  execution in Isaac Sim -> release -> stable placement in the dishwasher's lower rack.
- **23/31 trials succeed (74 %)** across 6 slots x 5 RNG seeds; failures are dominated by goal-infeasible deep-interior slots (reach limit), not planner or execution errors.
- Plan times: mean 1.41, median 0.81, p95 5.03, max 5.04 s against a 5 s budget — RRT-Connect over a custom FCL collision world at ~0.2 ms per collision query.
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
- Known simplifications: mug stands on the wire basket floor (the ArtVIP rack has no
  plate tines); clearance carry instead of a pinch grasp (grasping is out of scope for
  v0; two pads-on-rim attempts measurably interfere with the Robotiq jaw sweep).

## Best figures/clips (by path)

- docs/figures/success_by_slot.png — headline result
- docs/figures/outcome_breakdown.png — failure taxonomy
- media/F/trial_02_00_0.mp4 — full plan-execute-release clip (first end-to-end success)
- media/E/accepted_slot1_sheet.png — goal-config diversity at one slot
- media/D/overlay_E_shelf_1_04.png — rack decomposition, wire gaps preserved
- media/C/rigidity_iso.mp4 — welded-object rigidity proof
- docs/figures/best_trial_01_00_0.png — curated final still (trial_01_00_0)
- docs/figures/best_trial_01_01_0.png — curated final still (trial_01_01_0)
