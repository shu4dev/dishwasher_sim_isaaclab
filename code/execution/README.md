# execution

A UR5e + Robotiq 2F-85 carries out an instance's moves in the Frigidaire twin: no teleport, no weld. Also here:
the arm bring-up probes, the trial logs and the robot report. Work in progress; reference
`code/frigidaire/docs/robot.md`.

- Input: a HOTEC instance (`data/results/benchmark/frigidaire_hotec/instances/<tier>/<id>.json`), the robot asset
  mirror under `data/assets/robots/`, the Frigidaire build `data/build/frigidaire_collection/usd/fdpc4221as.usdc`.
- Output: run records `data/results/robot/runs/<run-id>/<instance>.json`, trial logs and media
  `data/artifacts/<run-id>/<trial-id>/`, probe results under `data/results/robot/`, the report.

Folder: [frigidaire/](frigidaire/README.md).

## Commands, in order

```bash
USD_CORE_VERSION=<x.y.z> code/execution/frigidaire/mirror_robot_usd.sh
code/util/run_py.sh code/execution/frigidaire/robot_asset_manifest.py --check
code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_smoke.py --headless
code/util/run_kit.sh code/execution/frigidaire/frigidaire_arm_model.py --headless
python3 code/execution/frigidaire/frigidaire_robot_mount.py --instance data/results/benchmark/frigidaire_hotec/instances/easy/easy_s0.json
code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --enable_cameras --instance data/results/benchmark/frigidaire_hotec/instances/easy/easy_s0.json --profile headline --run-id <id>
code/util/run_py.sh code/execution/frigidaire/frigidaire_robot_report.py --runs <id> --conventions p0_conventions --out REPORT.md
```

## Package modules

First needed here: `dishsim_frigidaire.robot` (`flags`, `harness`, `triallog`, `rig`, `ur5e`, `grasp`,
`collide`, `kin`).
