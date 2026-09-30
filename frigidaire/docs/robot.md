# UR5e + Robotiq 2F-85 in the Frigidaire twin (2026-09-29, work in progress)

Goal (user): execute an instance's moves with a real arm instead of teleports; test case easy_s0 (greedy plan).
Decisions: pure friction grasp (no weld); pedestal mount chosen by a reachability search; upper rack pushed in
while the lower rack is loaded (the extended upper rack covers the lower rack: no gap); one re-grasp per move.

## Code

| File | What |
|---|---|
| `frigidaire/src/dishsim_frigidaire/robot/ur5e.py` | articulation cfg (ported from on-corrallab), `deinstance_gripper`, `pad_material`, `set_gripper` (mimic-consistent inner fingers) |
| `robot/kin.py` | analytic UR5e IK/FK (ported unchanged) |
| `robot/grasp.py` | top-down grasps of a HOTEC bowl: rim pinch (upright), foot-ring clamp (mouth down), rim side point (lying); tilted variants that clear the counter slab |
| `robot/rig.py` | Kit controller: joint moves with velocity feed-forward, IK lines, gripper, blocked-motion detection |
| `robot/collide.py` | FCL arm model (convex hull per link from the USD collision meshes, calibrated link offsets) vs appliance, counter, dishes, pedestal, floor; OMPL RRT-Connect |
| `frigidaire/scripts/setup/mirror_robot_usd.sh` | Isaac 6.0 UR5e + 2F-85 asset mirror (11 MB, assets/robots) |
| `frigidaire/scripts/setup/frigidaire_grip_probe.py` | friction gate G0 and the collider diagnostics |
| `frigidaire/scripts/setup/frigidaire_grasp_test.py` | generated grasps on one bowl, per pose class |
| `frigidaire/scripts/setup/frigidaire_arm_model.py` | exports the arm collision model to results/robot/arm_model.json |
| `frigidaire/scripts/setup/frigidaire_robot_mount.py` | collision-checked mount search (floor / counter grids) |
| `frigidaire/scripts/experiment/frigidaire_robot_episode.py` | the episode: benchmark start state + robot, two phases, per-move report, video |

## Measured findings (landmines)

1. **Instanced gripper colliders make no contacts on Isaac 4.5** (the 6.0 asset): PhysX creates the shapes but
   the fingers close through a 30 mm cube, the ground and the bowl (0 N). `SetInstanceable(False)` on the gripper
   prims before `sim.reset` fixes it (cube held at 26-60 N). This was the 2026-08 "pads give 0 N" regression.
2. The TCP (`T_WRIST3_TCP`) is at the fingertip ENDS; the pads span ~6 mm beyond it to ~32 mm behind it; open
   pad faces are 87 mm apart, closed 2 mm; closing axis = TCP y.
3. A sleeping bowl floats when its support is teleported away (use `sleep_threshold = 0`).
4. Rim pinch of an upright HOTEC bowl: the flared rim lets it pivot ~30 mm on the lip, then it hangs stably
   (0.3 mm drift / 3 s at 40 N per pad; stiffness 400, effort 8 N m). Pad friction 1.0 changed nothing.
5. Mouth-down bowl: the 4 mm foot ring is the only top-down feature; a centred clamp is blocked by the widening
   body, an off-centre clamp held once when the bowl was placed between the open pads, but approached from
   above the inside pad lands on the bowl floor (4 mm below the ring top). Not reliable yet.
6. The implicit PD drives lag by damping x velocity / stiffness (0.16 rad on the wrists at 1.2 rad/s): joint
   paths need velocity feed-forward. The UR5e elbow limit is +-pi (planner bounds).
7. Geometry of the scene: the extended lower rack's rear ~12 cm is under the counter's front edge (y -0.30);
   the extended upper rack covers the lower rack. With the collision model no single floor mount reaches every
   easy_s0 start and goal: best (0.45, -0.55, 0.90) reaches 7/7 starts and 6/7 goal proxies (all IK branches).

## Status

**2026-09-29: the easier case PASSES** (user's scope call after easy_s0 proved hard): `--test-case upright3`
(3 upright bowls on the counter -> 2 lower-rack spots with the upper rack pushed in, then 1 upper-rack spot;
release = lowered until contact, as the bowl hangs): 3/3 moves on the first attempt, all racked, none disturbed,
jaw 0.60 rad on the wall, 143 s simulated / 7.9 min wall. Video `media/robot/robot_upright3/episode.mp4`, record
`results/robot/episodes/robot_upright3.json`, page https://claude.ai/artifact/Lkm8mUMhRBP8iddoLjKSMu.
Mount (0.45, -0.55, 0.90), yaw 2.60 rad (`results/robot/mount/easy_s0_any.json`, all IK branches).

What made it work, in order of discovery (each measured in this session): de-instanced gripper colliders; the
stable-tilted rim hold measured in the hand after each lift; velocity feed-forward (PD lag); real joint limits
(elbow +-pi); a per-link FCL arm model with the gripper placed by LIVE poses and one hull per gripper collider (a
single hull fills the jaw gap); contact exemptions for the gripper only; wrist-up IK preference (a flipped wrist
hangs its housing at the dish); grasp candidates paired with a hover posture whose approach line is continuous and
free; carrying: rise to Z_SAFE first (a joint move dipped the dish into the counter), 0.25 rad/s, smoothstep timing,
the carried dish checked against the arm's own links, a looser settle tolerance while carrying.

Open for easy_s0: mouth-down bowls (foot-ring clamp unreliable), the lying bowl under the counter edge, one goal no
mount reaches; placements into the benchmark's tilted mouth-down goal poses need a regrasp or a wrist flip.
