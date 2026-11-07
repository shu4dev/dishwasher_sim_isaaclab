# Mount search and reach map for the foot-ring hold (Kit-free): run note, 2026-10-29
Plan: plans/2026-10-29-ring-mount.md. Mac `.venv-kitfree`, no box. Code and outputs: `data/results/robot/kitfree_v1/ring_mount/` (README lines in `kitfree_v1/README.md`). **Works-when: met.**
- **Conclusion: a mount change is not worth a box round.** The rule's best mount gains one bowl (bowl_06: 4/7 vs 3/7), but bowl_06 cannot go down its last 50 mm at any mount. On that path the bowl itself passes at least 9.5 mm through lower-rack tines, at all 503 of its grid poses where the hand clears. Its v4 goal is already 6.9 mm into a tine tip, Kit-free.
- Counting the bowl's own path, every one of the 825 mounts reaches at most 3/7 (07, 04, 03).
- The best mount keeps the 8 cup drops, the 4 front plate seats and the three picks. It costs the cups their episode start: no elbow-up home IK is free with the cup held, at z 0.45 and at every z 0.50 mount.
- Commands: `.venv-kitfree/bin/python data/results/robot/kitfree_v1/ring_mount/{smoke,cupcheck,search,reach_map,extra_b05}.py`. Times: 7, 6, 195 (7 workers), 232 (7 workers) and 19 s, about 8 min of compute in a 70-min session.

| mount (825 candidates; 173 pass every constraint) | constraints | bowls reached (hand rule) | with the bowl's own path clear | cup start free |
|---|---|---|---|---|
| current posture_v1 (0.45, -0.70, 0.60) yaw 2.56, rank 66 | pass | 3: 07, 04, 03 | 3 | yes |
| best by the rule (0.45, -0.70, 0.45) yaw 2.56 | pass | 4: + 06 (spin 330 only) | 3 | no |
| best with the cup start free (0.25, -0.80, 0.60) yaw 2.30, rank 34 | pass | 4: + 06 | 3 | yes |

| bowl | smallest deviation with a placement (centroid / axis), current / best / start-free mount | feasible poses of 4860 (with the bowl's path clear), current / best / start-free | blocker |
|---|---|---|---|
| 06 | hand rule: current 0 mm, 5°; best and start-free 0 mm, 0° (spin 330). None with its path clear | 66 (0) / 176 (0) / 254 (0) | its own path: lower-rack tines |
| 07, 04, 03 | 0 / 0 at the goal (spins 150 or 120, 30, 30), in slot | 07: 525 (19) / 646 (25) / 986 (36); 04: 228 (3) / 228 (3) / 216 (3); 03: 454 (2) at all three | none |
| 02 | none | 0 | the gripper vs the cabinet or upper rack at all 4860 poses |
| 05 (flagged goal) | none | 0 | no IK, or the forearm vs the upper rack. Its 6 tool- and path-clear poses (21-28 mm, +5°) have no free arm IK at any of the 825 mounts |
| 01 | none | 0 | the gripper vs the lower rack, bowl_05, bowl_07 at all 4860 poses |

- **Deviations:**
  - The plate seats were recomputed with the round-12 window-2 hold (mirror, 6 mm). mount_seats used the coded base-face hold, which passes through the plate. The gripper is excluded from this check, since the hand stops on the front neighbour by the accepted rule.
  - The plate pick was added: the face-up rim pinch at the counter spot, with mount_seats' pick rule. None existed before; it removes 173 candidates.
  - The cup drops and the h45 cup pick are reused unchanged. These are not the Part B 11 mm pick.
  - Added beyond the plan: the bowl-path column (a note), a third reach-map mount and `extra_b05.py`.
  - Ties after the pedestal were broken by passing spins, then index. The base yaw changes no bowl result.
- **Missing:** the carry (pick to descent start), the bowl and plate episode starts, Isaac, and a rerun on easy_bowls_v4.
- **Surprise:** bowl_06's goals_v4 pose has two tines through its wall (6.9 and 3.6 mm, Kit-free; goals_v4 already lists `release_collides_kitfree`). Like bowl_05's, it may need a re-settle.

**Suggested commit:** `Re-score the mount search for the bowl foot-ring hold`
