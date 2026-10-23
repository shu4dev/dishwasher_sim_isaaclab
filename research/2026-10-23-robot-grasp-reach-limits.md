# Hardware limits of a UR5e + Robotiq 2F-85 loading mouth-up dishware: what is graspable and what is reachable, per object and per rack position
Slug: 2026-10-23-robot-grasp-reach-limits | Lit review: research/2026-10-23-robot-grasp-reach-limits-litreview.md

## Question
With every plate, bowl and cup lying mouth-up on the counter, which object types can the UR5e + 2F-85 grasp, and which rack positions can it place each into, measured as a per-object, per-slot feasibility table with videos of the primary cases?

## What's known (max 5)
- [srinivas2023], CASE 2023: UR5 + 2F-85, top-down rim pinch on upright bowls (r 8.5 cm) and cups, 0 bowl failures in 36 removals, 2 cup failures in 36. Our exact gripper: bowl and cup rim pinch is feasible when a rim point is clear.
- [voysey2021], Engineering Reports: plates lying on a tray with a one-shot pinch 12 of 24; with a force-guided surface slide (lower, rise 2 mm, slide 4 cm, close on the edge) 23 of 24. A flat plate needs a support-contact primitive, not reach.
- [otsuji2024], JRSJ: rim-grasped wet bowls need >= 39 N; straight jaws jammed on a small bowl; jaw-divider interference at release had to be designed out. Release into tines is its own failure mode.
- [WangHauser2019], ICRA: plans gated on IK and collision checks along the whole insertion path executed 79 %, overlap-only plans 17 %. The path, not the endpoint, decides placement.
- [Bartunek2026] ICRA 2026 and [Saini2024]: per-cell repeated-trial sweeps (256 x 5; 12 x 5) show success near 0 at the reach boundary and next to the base for kinematic or collision reasons. The fixed-mount feasibility map has a known shape and n = 5 per cell is coarse.

## Gap
No source measures per-slot rack feasibility for a fixed 6-DoF arm with tines, a lower rack below the mount and an upper rack overhead; none uses a 2F-85 on a ceramic plate lying flat; none tests a mouth-down grasp at all. The reachability maps are collision-blind and RM4D excludes poses below the base.

## Candidate ideas (max 3)
1. Per-object, per-slot gate table from the existing stack (analytic IK over all branches, FCL arm model, RRT-Connect; then Isaac lift-hold and settle), one object type at a time, mouth-up, n = 10 per cell with Clopper-Pearson intervals, plus a counter grasp gate per object first. Might work because the stack already did 3/3 upright bowls and the mount search. Might fail on plates (the literature says a rigid pinch on a flat plate fails) and on the box's robot bring-up (assets and the gripper-collider fix). Cheapest test: the Kit-free gates G0-G2 over every rack site for the bowl.
2. Voxel reachability map (RM4D-style) for the mount, then rack slots looked up. Might fail because every map is collision-blind and RM4D records only poses above the base. Cheapest test: none worth running.
3. Learned grasp sampler per object. Fails on bowls (0.00-0.15) per [alliegro2022]; not pursued.

## Picked
1, because it reuses the measured stack, answers the two questions the user asked (graspable; reachable everywhere) per object and per slot, and runs the three object types in parallel. The plate gets its grasp gate on the counter first, as the literature says a flat plate is a gripper limit, not a reach limit.

## To grill
- Whether a top-down rim pinch on a mouth-up HOTEC plate is physically possible with stock 2F-85 pads (fingertips hit the counter), or needs the surface-slide primitive; decided for this round: test the pinch with the plate's rim as the target and report the gate honestly, slide primitive deferred.
- Whether the reach table at the one measured mount (0.45, -0.55, 0.90) is the question, or the mount is a free variable; decided: fixed mount this round, the mount search is a later round.
- Whether n = 10 per slot fits the window; decided: G0-G2 are deterministic and cheap for every slot, G3-G4 in Isaac at n = 3 on the reachable slots this round, n = 10 later.
