# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Run rearrangement algorithms against saved instances, physics-validated per move.

One persistent Kit session serves a batch of instances that share one (machine, placement,
rack state). Per episode: park the object pool, teleport the roster to the instance's
recorded settled initials, settle, verify the reproduction, then drive the closed-loop
episode (:func:`dishsim.rearrange.run_episode`) with the Isaac oracle — every move teleports,
settles, and aborts on the first fault. Episode records are the primary artifact; video is
on-demand (``--video``, requires ``--enable_cameras``).

Run: scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless \
         --instances "results/instances/bosch800/placement/*.json" --algorithms greedy
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import copy
import glob
import hashlib
import math
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Rearrangement benchmark runner.")
parser.add_argument("--instances", type=str, default=None,
                    help='Instance JSON glob, e.g. "results/instances/bosch800/placement/*.json".')
parser.add_argument("--cells", type=str, default=None,
                    help="Comma list of dishsim.tiers.CELLS names; expands to per-cell "
                         "instance globs under --instances_root.")
parser.add_argument("--instances_root", type=str,
                    default="results/instances/bosch800/placement")
parser.add_argument("--algorithms", type=str, default="greedy", help="Comma list of registry names.")
parser.add_argument("--budget_mult", type=float, default=3.0, help="Move budget = ceil(mult * n_items).")
parser.add_argument("--video", action="store_true", help="Record one MP4 per episode (needs --enable_cameras).")
parser.add_argument("--seed", type=int, default=0,
                    help="Base seed; each (instance, algorithm) gets a derived seed, recorded.")
parser.add_argument("--time_budget_s", type=float, default=60.0,
                    help="Per-episode PLANNING-time budget [s] (algorithm thinking only); "
                         "<= 0 = unlimited.")
parser.add_argument("--out", type=str, default=os.path.join(PROJECT_ROOT, "results", "rearrange"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

_enable_cameras = args_cli.enable_cameras  # 2.1's AppLauncher pops this off the namespace
app_launcher = AppLauncher(args_cli)
args_cli.enable_cameras = _enable_cameras
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

if bool(args_cli.instances) == bool(args_cli.cells):
    raise SystemExit("[FAIL] give exactly one of --instances or --cells")
if args_cli.cells:
    _cells = [c.strip() for c in args_cli.cells.split(",") if c.strip()]
    _paths = sorted(p for c in _cells for p in glob.glob(
        os.path.join(PROJECT_ROOT, args_cli.instances_root, c, "*.json")))
    if not _paths:
        raise SystemExit(f"[FAIL] no instances for cells {_cells} under "
                         f"{args_cli.instances_root}")
else:
    _pattern = args_cli.instances if os.path.isabs(args_cli.instances) \
        else os.path.join(PROJECT_ROOT, args_cli.instances)
    _paths = sorted(glob.glob(_pattern))
    if not _paths:
        raise SystemExit(f"[FAIL] no instances match {_pattern}")
_docs = [json.load(open(p)) for p in _paths]
_ctx = {(d["machine"], d["base_placement"], d["state"]) for d in _docs}
if len(_ctx) != 1:
    listing = "\n".join(f"  {p}: {d['machine']}/{d['base_placement']}/{d['state']}"
                        for p, d in zip(_paths, _docs))
    raise SystemExit(f"[FAIL] one Kit session serves one (machine, placement, state); "
                     f"the glob mixes {sorted(_ctx)}:\n{listing}\n"
                     f"Run one invocation per state.")
MACHINE, BASE_PLACEMENT, STATE = next(iter(_ctx))

# context order contract: machine -> scenario -> placement, all BEFORE scene imports
if MACHINE != config.MACHINE_BASELINE_NAME:
    config.apply_machine(MACHINE)
config.apply_scenario(STATE)
config.apply_base_placement(BASE_PLACEMENT)

from dishsim import rearrange  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, VideoWriter, release_sim_for_close  # noqa: E402
from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402

ALGORITHMS = {"greedy": rearrange.Greedy}
try:  # sampling planners register when their module imports cleanly (never sink a batch)
    from dishsim import rrt as _rrt

    ALGORITHMS.update({"rrt": _rrt.RRT, "rrt_connect": _rrt.RRTConnect,
                       "rrt_star": _rrt.RRTStar})
except Exception as _exc:  # noqa: BLE001
    print(f"[WARN] rrt planners unavailable: {_exc!r}")


def _episode_seed(base: int, instance_name: str, algo_name: str) -> int:
    """Deterministic per-(instance, algorithm) seed.

    ``hash()`` on str is salted per process (PYTHONHASHSEED), so it cannot be used here: a
    replay would draw a different stream and the recorded seed would be a lie.
    """
    key = f"{base}|{instance_name}|{algo_name}".encode()
    return int(hashlib.sha256(key).hexdigest()[:8], 16)


class IsaacOracle:
    """The physics side of the episode seam: teleport, settle, judge, observe."""

    def __init__(self, scene, sim, roster, step_fn):
        self.scene, self.sim, self.roster, self.step = scene, sim, roster, step_fn
        self.T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
        self.T_base_w = T_inv(self.T_w_base)
        self.device = scene["dishwasher"].data.joint_pos.device

    def teleport(self, item_id, T_base):
        pos_w, quat = T_to_pos_quat(self.T_w_base @ np.asarray(T_base))
        # project poses are pos + XYZW; isaaclab 2.1 wants pos + WXYZ
        self.scene[item_id].write_root_pose_to_sim(root_pose=torch.tensor(
            np.concatenate([pos_w, xyzw_to_wxyz(quat)])[None], dtype=torch.float32, device=self.device))
        self.scene[item_id].write_root_velocity_to_sim(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=self.device))

    def park(self, item_id, pos_w):
        self.teleport(item_id, self.T_base_w @ make_T(pos_w, (0.0, 0.0, 0.0, 1.0)))

    def measured(self, item_id):
        return self.T_base_w @ make_T(
            self.scene[item_id].data.root_pos_w[0].cpu().numpy(),
            wxyz_to_xyzw(self.scene[item_id].data.root_quat_w[0].cpu().numpy()))

    def poses(self):
        return {it["item_id"]: self.measured(it["item_id"]) for it in self.roster}

    def at_goal(self, item, T):
        return rearrange.at_goal(item, T)

    def _disturbed(self, pre, poses, moved_id):
        return [
            it["item_id"] for it in self.roster if it["item_id"] != moved_id and (
                float(np.linalg.norm(poses[it["item_id"]][:3, 3] - pre[it["item_id"]][:3, 3]))
                > rearrange.DISTURB_POS_M
                or rearrange.rot_angle_deg(pre[it["item_id"]], poses[it["item_id"]])
                > rearrange.DISTURB_ROT_DEG)]

    def execute(self, move):
        pre = self.poses()
        self.teleport(move.item_id, move.T_base_obj)
        hist = []
        for s in range(rearrange.SETTLE_STEPS_MOVE):
            self.step(1)
            if s >= rearrange.SETTLE_STEPS_MOVE - rearrange.DRIFT_WINDOW:
                hist.append(self.measured(move.item_id))
        poses = self.poses()
        settled = poses[move.item_id]
        drift_p = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3]))
        drift_deg = rearrange.rot_angle_deg(hist[0], hist[-1])
        dev_p = float(np.linalg.norm(settled[:3, 3] - np.asarray(move.T_base_obj)[:3, 3]))
        disturbed = self._disturbed(pre, poses, move.item_id)
        info = {"settle_dev_mm": round(dev_p * 1e3, 1), "drift_mm": round(drift_p * 1e3, 1),
                "disturbed": disturbed}
        if disturbed:
            # fatal FIRST: a put-back cannot restore a knocked neighbour
            return poses, "disturbed", info
        if drift_p > rearrange.STABLE_POS_M or drift_deg > rearrange.STABLE_ROT_DEG \
                or dev_p > rearrange.MOVE_DEV_MAX_M:
            # NON-fatal failed settle: return the item to its pre-move settled pose,
            # re-settle, and hand the episode back exactly the pre-move state. The put-back
            # is a teleport-into-contact at a pose that was a settled equilibrium moments
            # ago, so the INIT_MATCH_* reproduction gates are the right judge of it.
            self.teleport(move.item_id, pre[move.item_id])
            for _ in range(rearrange.SETTLE_STEPS_MOVE):
                self.step(1)
            poses = self.poses()
            back_dp = float(np.linalg.norm(
                poses[move.item_id][:3, 3] - pre[move.item_id][:3, 3]))
            back_dr = rearrange.rot_angle_deg(pre[move.item_id], poses[move.item_id])
            info["teleport_back_mm"] = round(back_dp * 1e3, 1)
            re_disturbed = self._disturbed(pre, poses, move.item_id)
            if re_disturbed:
                info["disturbed"] = re_disturbed
                return poses, "disturbed", info      # the put-back knocked a neighbour
            if back_dp > rearrange.INIT_MATCH_POS_M or back_dr > rearrange.INIT_MATCH_ROT_DEG:
                return poses, "unstable-settle", info  # the put-back itself failed
            return poses, "failed-settle", info
        return poses, None, info


def main() -> int:
    instances = [rearrange.Instance.load(p) for p in _paths]
    algo_names = [a.strip() for a in args_cli.algorithms.split(",") if a.strip()]
    unknown = [a for a in algo_names if a not in ALGORITHMS]
    if unknown:
        print(f"[FAIL] unknown algorithm(s) {unknown} (have: {sorted(ALGORITHMS)})")
        return 1

    # object pool: per-class max over the batch (rosters share the deterministic plan, so in
    # practice the pool IS the shared roster); parked at the reveal grid
    pool_items = {it["item_id"]: it["object_class"] for inst in instances for it in inst.items}
    classes = sorted(set(pool_items.values()))
    obj_specs = [{"name": item_id, "usd_path": config.OBJECTS[cls].usd_path,
                  "pos": (-2.0 - 0.3 * (i % 6), -1.5 + 0.3 * (i // 6), 0.10),
                  "quat": (0.0, 0.0, 0.0, 1.0), "color": config.item_color(item_id)}
                 for i, (item_id, cls) in enumerate(sorted(pool_items.items()))]
    park_w = {s["name"]: s["pos"] for s in obj_specs}

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device))
    rig = None
    if args_cli.video:
        assert args_cli.enable_cameras, "--video needs --enable_cameras"
        ep_cam = config.EPISODE_CAMERA
        rig = CameraRig({"episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                                     dict(ep_cam["lens"]))}, hw=config.CAMERA_HW)
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()

    video = {"writer": None, "frame": 0}

    def step(n=1):
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            video["frame"] += 1
            if video["writer"] is not None and video["frame"] % 2 == 0:
                rig.update(dt)
                video["writer"].add(rig.grab_one("episode"))

    step(300)  # racks settle at the scenario extensions
    world = rearrange.ArrangementWorld(STATE, classes)
    out_dir = os.path.join(args_cli.out, MACHINE, STATE)
    os.makedirs(out_dir, exist_ok=True)
    media_dir = os.path.join(PROJECT_ROOT, "media", "rearrange", MACHINE, STATE)

    n_solved = n_episodes = 0
    for inst_src in instances:
        for algo_name in algo_names:
            # Every algorithm must see the artifact's inputs, not its predecessor's. The
            # episode reset below rewrites T_base_init with the MEASURED reproduction, so
            # sharing one Instance across algorithms silently re-baselines each run on the
            # previous one's settle drift — which would invalidate any comparison.
            inst = copy.deepcopy(inst_src)
            roster = inst.items
            oracle = IsaacOracle(scene, sim, roster, step)
            cell = inst.meta.get("cell")
            rec_dir = os.path.join(out_dir, cell) if cell else out_dir
            os.makedirs(rec_dir, exist_ok=True)
            record_path = os.path.join(rec_dir, f"{inst.name}__{algo_name}.json")
            video_path = None
            if rig is not None:
                vid_dir = os.path.join(media_dir, cell) if cell else media_dir
                os.makedirs(vid_dir, exist_ok=True)
                video_path = os.path.join(vid_dir, f"{inst.name}__{algo_name}.mp4")
                video["writer"] = VideoWriter(video_path, fps=config.CAMERA_FPS)

            # episode reset: park the whole pool, place the roster, settle, verify
            for item_id in pool_items:
                oracle.park(item_id, park_w[item_id])
            step(30)
            for it in roster:
                oracle.teleport(it["item_id"], it["T_base_init"])
            step(rearrange.SETTLE_STEPS_INIT)
            mismatch = []
            for it in roster:
                T = oracle.measured(it["item_id"])
                dp = float(np.linalg.norm(T[:3, 3] - np.asarray(it["T_base_init"])[:3, 3]))
                dr = rearrange.rot_angle_deg(it["T_base_init"], T)
                if dp > rearrange.INIT_MATCH_POS_M or dr > rearrange.INIT_MATCH_ROT_DEG:
                    mismatch.append({"item_id": it["item_id"], "pos_mm": round(dp * 1e3, 1),
                                     "rot_deg": round(dr, 1)})
            if mismatch:
                record = {"instance": inst.name, "algorithm": algo_name, "machine": MACHINE,
                          "state": STATE, "solved": False, "abort": "init-mismatch",
                          "init_mismatch": mismatch,
                          "instance_meta": dict(inst.meta)}  # keeps cell attribution intact
                print(f"[WARN] {inst.name}/{algo_name}: init-mismatch {mismatch}")
            else:
                world.clear()
                world.n_queries = 0  # per-episode counter (else it accumulates over the batch)
                # seed the mirror + episode from the MEASURED reproduced initials
                for it in roster:
                    it["T_base_init"] = oracle.measured(it["item_id"])
                n_items = len(roster)
                budget = math.ceil(args_cli.budget_mult * n_items)
                # derived per (instance, algorithm) so a stochastic planner is replayable and
                # two algorithms on one instance do not share a stream
                seed = _episode_seed(args_cli.seed, inst.name, algo_name)
                try:
                    algo = ALGORITHMS[algo_name](seed=seed)  # stochastic planners consume it
                except TypeError:
                    algo = ALGORITHMS[algo_name]()           # deterministic ones don't
                time_budget = (None if args_cli.time_budget_s is None
                               or args_cli.time_budget_s <= 0 else args_cli.time_budget_s)
                record = rearrange.run_episode(inst, algo, world, oracle,
                                               budget=budget, algorithm_name=algo_name,
                                               time_budget_s=time_budget,
                                               counter_cap=inst.meta.get("counter_cap"))
                record["seed"] = seed
                n_solved += record["solved"]
            n_episodes += 1
            if video["writer"] is not None:
                video["writer"].close()
                record["video"] = os.path.relpath(video_path, PROJECT_ROOT)
                video["writer"] = None
            with open(record_path, "w") as f:
                json.dump(record, f, indent=1)
            print(f"[INFO] {inst.name}/{algo_name}: solved={record['solved']} "
                  f"abort={record.get('abort')} at_goal={record.get('at_goal_final', '-')}"
                  f"/{record.get('n_items', '-')} moves={record.get('moves_used', '-')} "
                  f"-> {os.path.relpath(record_path, PROJECT_ROOT)}")

    print(f"[INFO] {n_solved}/{n_episodes} episodes solved")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    code = main()
    release_sim_for_close()
    simulation_app.close()
    raise SystemExit(code)
