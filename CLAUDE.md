# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Isaac Lab RL environments where a UR5e + Robotiq 2F-85 opens the door of an articulated ArtVIP
dishwasher (`Isaac-Open-Dishwasher-UR5e-v0` / `-Play-v0`). Runs on Isaac Sim **6.0.1-rc.7** +
Isaac Lab **3.0.0** at `/workspace/isaaclab` (this repo is nested inside that tree, but is an
independent git repo). Never upgrade or downgrade Isaac Sim / Isaac Lab. Everything runs
`--headless` on a single NVIDIA L4: default `--num_envs 512`, hard cap 1024, state-based
observations only.

The parent tree's `CLAUDE.md`/`AGENTS.md` (IsaacLab contributor rules) mostly targets the Isaac
Lab source itself — its changelog-fragment and `./isaaclab.sh -f` pre-commit workflow do **not**
apply here. What does carry over: commit-message conventions and **no AI attribution/co-author
lines in commits**.

## Commands

All Python goes through the Isaac Lab wrapper, run from this project root so `logs/` resolves:

```bash
# one-time setup
/workspace/isaaclab/isaaclab.sh -p -m pip install -e source/dishwasher_tasks
# assets (~82 MB, gitignored) — see README.md for the snapshot_download one-liner

# scene inspection: regenerates docs/joint_report.md and the derived RL USD,
# runs the 500-step stability check and the three-part passive-door test
/workspace/isaaclab/isaaclab.sh -p scripts/00_inspect_scene.py --headless --test_door

# smoke test the env (300 zero-action steps)
/workspace/isaaclab/isaaclab.sh -p scripts/zero_agent.py --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 4 --headless

# train (~3 s/iteration at 512 envs; 200 iterations ≈ 12 min)
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 512 --headless

# play latest checkpoint / record a clip
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16 --headless --enable_cameras --video --video_length 400
```

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Always verify success from
log content (iteration lines, `[RESULT] PASS`, absence of tracebacks / `free(): invalid
pointer`), never from the exit code.

## Launcher landmines (why the scripts look the way they do)

Full write-up in `docs/environment.md`; violating any of these produces native crashes or
silent import shadowing, not clean errors:

1. **Boot-first**: every entry script launches `AppLauncher` *before* importing
   `dishwasher_tasks` or resolving any env cfg. The stock Isaac Lab template flow
   (hydra/`resolve_task_config` + `launch_simulation(env_cfg, ...)` pre-boot) crashes Kit with
   `free(): invalid pointer` for this project's config. Do not "simplify" the scripts back to
   the template pattern, and keep module-scope `pxr`/`omni` imports out of the package
   (`utils/usd_prep.py` imports pxr lazily inside the function for this reason).
2. **`env_cfg.sim` must be a `PresetCfg` wrapper** (`DishwasherSimCfg`), not a plain
   `SimulationCfg`. Standalone scripts collapse it with
   `isaaclab_tasks.utils.hydra.resolve_presets(env_cfg)` after boot.
3. **The package is `dishwasher_tasks`, deliberately not matching the repo directory name** —
   Kit's extension scan turns a same-named directory into a namespace package that shadows the
   real one. Don't rename either side to match the other.

## Architecture

`source/dishwasher_tasks/dishwasher_tasks/` (pip-installed editable):

- `robots/ur5e_robotiq_2f85.py` — `UR5E_ROBOTIQ_2F_85_CFG`. The UR5e USD carries the gripper as
  a variant (`Gripper=Robotiq_2f_85`); a local mirror under `assets/robots/` is preferred over
  the live Nucleus S3 fetch. Two things here are load-bearing:
  `articulation_root_prim_path="/root_joint"` (the asset has a second, disabled
  ArticulationRootAPI on the gripper subtree), and the gripper actuator gains + armature 0.001 +
  passive damping 0.05 — without them the near-massless mimic-joint finger cluster resonates and
  explodes when the arm moves fast.
- `robots/dishwasher.py` — `DISHWASHER_CFG` with the passive door: stiffness 0, damping 5,
  friction 0.6. The friction **is the door latch**: the bottom-hinged door is gravity-unstable
  at 0°, so lowering it makes every episode start with the door falling open. Points at the
  derived `model_dishwasher_2_rl.usda`.
- `utils/usd_prep.py` — derives that RL copy: removes ArtVIP's world-weld `FixedJoint` (body1
  set, no body0 — it pins the machine at its authored world pose and blows up any relocated,
  cloned spawn) and zeroes the authored door drive (a per-degree-units spring toward 90° that
  kicks the door during the physics-reset step). The downloaded originals are never modified.
- `tasks/manager_based/dishwasher/` — cabinet-task-shaped manager-based env:
  `open_dishwasher_env_cfg.py` (robot-agnostic scene/rewards/terminations),
  `config/ur5e/joint_pos_env_cfg.py` (robot, actions, ee_frame, reward params, `_PLAY`),
  `config/ur5e/agents/rsl_rl_ppo_cfg.py` (new-style rsl-rl 5.0 `actor=`/`critic=` cfg),
  `mdp/` (door-adapted ports of the cabinet reward/observation terms plus the
  `door_open_sustained` termination).

Numeric provenance: every prim path, joint name, frame offset, and placement number in the env
configs is a *measured* value recorded in `docs/joint_report.md` (generated by
`scripts/00_inspect_scene.py`) and `docs/asset_survey.md`. If you change the dishwasher variant,
robot home pose, or scene layout, re-run the inspection script and take the new numbers from the
report — don't eyeball them. Two traps encoded there: spawn poses place the articulation **root
link** frame (for the dishwasher that's `E_body_5` at the machine's rear corner, not the asset
origin), and this Isaac Lab is XYZW-quaternion / `.torch`-ProxyArray throughout (2.x tutorial
snippets are wrong on both counts).

Only `finger_joint` is actuated on the gripper (0 = open, ~0.8 rad = closed — inverted vs. the
Franka convention, which is why `mdp/rewards.py:grasp_handle` differs in sign from the cabinet
original). The five mimic-driven finger joints must stay out of the action space.

## Ground rules

- `assets/` and `logs/` are gitignored; never commit them (asset sources + licenses are in
  README.md). The preserved first-run artifacts live in `docs/first_run/`.
- The dishwasher base stays fixed (`fix_root_link=True`) at all times.
- Ask the user before: downloads over 2 GB, training runs expected to exceed 30 minutes, or
  opening/exposing ports. GUI verification happens only via the streaming client — pause and ask
  the user to connect.
- Commit style: imperative ~50-char subject, wrapped body explaining what/why, one commit per
  logical milestone, no AI attribution lines.
