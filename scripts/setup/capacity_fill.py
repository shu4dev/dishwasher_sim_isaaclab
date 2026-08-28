# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Settle certification of a capacity plan: the planned load, arriving one item at a time.

:mod:`dishsim.capacity` certifies a full load with FCL — every item's release-hover pose is
collision-free against the machine and against every earlier item at its own goal. That is a
geometry statement, not a physics one: a disc can be collision-free in a gap it cannot balance
in, and a bowl can be collision-free over a wire lattice that tips it. This script closes that
gap. In plan order it teleports each planned item to its planned pose raised by
:data:`ARRIVAL_HOVER_M` [m], zeroes its velocities, settles it for
``rearrange.SETTLE_STEPS_MOVE`` physics steps, and gates the result on the settled pose
holding still over the last ``rearrange.DRIFT_WINDOW`` steps.

Two artifacts, both load-bearing:

- the RECORD (``--out``): per item ``seated`` (the settle gate) and ``at_goal``
  (:func:`dishsim.placement.evaluate_placement` re-run on the SETTLED pose). Those are
  different questions — an item that comes to rest one slot over, or leaning past its mode's
  tilt tolerance, is perfectly stable and still wrong — so an honest capacity claim needs both
  numbers and neither may be reported as the other;
- the fill TIMELAPSE (``--video``): the staggered arrivals ARE the video, and the video is this
  work's acceptance evidence. That is the whole reason items arrive one at a time instead of
  all at once; the all-at-once tableau is ``scripts/evaluation/reveal_render.py``, whose scene
  and media idiom this script otherwise mirrors (it also owns the beauty orbit, so this run
  writes only the timelapse plus final stills).

Honest capacity: an item that fails the settle gate is parked off-stage again and recorded as
not-seated — the run NEVER aborts on it. That is the deliberate opposite of a rearrangement
episode (:func:`dishsim.rearrange.run_episode` aborts on the first fault): the number this
script reports is "how much of the certified plan physically holds", which an early abort
would leave unmeasured. Re-parking also keeps a tipped item from poisoning its neighbours'
verdicts.

What this script does NOT do: the v1 stow/closability half (drive the loaded racks back in and
check nothing is displaced). The Bosch lower rack cannot ride back over the door sill under
load — measured 176.9 mm short of stowed with a single bowl aboard, see
``docs/known_limitations.md`` — so the certified load fills top-down and ends with the machine
open. This pass therefore verifies one state's phase, and the default ``placement`` phase is
the lower rack.

Traps encoded here: boot-first (AppLauncher before any ``dishsim``/``isaaclab`` scene import),
the context order contract (machine -> scenario -> placement, all before scene imports), and
gates that live in :mod:`dishsim.rearrange` rather than :mod:`dishsim.config` — ``config`` feeds
``geometry.config_hash``, so a tuning knob added there would silently invalidate every baked
collision cache.

Run: scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras --video \
         --plan results/capacity/bosch800/side_winner/full_load_plan.json --state placement
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Physically settle a certified capacity plan.")
parser.add_argument("--plan", type=str, required=True,
                    help="Capacity-plan JSON (see scripts/setup/plan_full_load.py).")
parser.add_argument("--state", type=str, default="placement",
                    help="Plan phase to verify — the machine state whose rack is extended.")
parser.add_argument("--video", action="store_true",
                    help="Write the fill timelapse + final stills (needs --enable_cameras).")
parser.add_argument("--out", type=str, default=None,
                    help="Record JSON (default: results/capacity/<machine>/<placement>/"
                         "settled_verification.json).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import json
import sys

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

_plan_path = args_cli.plan if os.path.isabs(args_cli.plan) else os.path.join(PROJECT_ROOT, args_cli.plan)
PLAN = json.load(open(_plan_path))
# context order contract: machine -> scenario -> placement, all BEFORE scene imports
if PLAN["machine"] != config.MACHINE_BASELINE_NAME:
    config.apply_machine(PLAN["machine"])
config.apply_scenario(args_cli.state)
config.apply_base_placement(PLAN["base_placement"])

from dishsim import placement, rearrange  # noqa: E402
from dishsim import scene as dscene  # noqa: E402
from dishsim.media import CameraRig, VideoWriter  # noqa: E402
from dishsim.transforms import T_inv, T_to_pos_quat, make_T  # noqa: E402

#: Arrival hover above the planned pose [m]. The plan's ``T_base_obj`` is already a nominal
#: RELEASE-HOVER pose (``capacity._nominal_release_pose``); this is the extra drop the v1 fill
#: used so an item arrives through the air onto its support instead of teleporting into
#: contact with it — the same short fall a released item takes, and what makes the timelapse
#: read as loading rather than as popping into existence.
ARRIVAL_HOVER_M = 0.020
#: Physics steps letting the racks settle at the state's scenario extensions before any item
#: arrives (the run_rearrange warm-up).
WARMUP_STEPS = 300
#: Renders burned before the first captured frame — CameraRig returns black frames until the
#: pipeline has warmed up.
CAMERA_WARMUP_FRAMES = 8
#: Physics steps after re-parking an unstable item (let it come to rest off-stage).
PARK_STEPS = 10
#: The v1 acceptance bar: this fraction of the planned items must seat.
SEATED_FRACTION_BAR = 0.8


def park_pos_w(index: int) -> tuple[float, float, float]:
    """Off-stage parking spot [m, world] for the ``index``-th item (the reveal grid)."""
    return (-2.0 - 0.3 * (index % 6), -1.5 + 0.3 * (index // 6), 0.10)


def slot_frames(state: str, classes: list[str]) -> dict[str, dict[int, placement.SlotFrame]]:
    """Per class, ``{slot_id: SlotFrame}`` for ``state`` — the frames the at-goal verdict needs.

    The plan artifact records ``slot_id`` but not the slot frame, so the frames are re-derived
    from the same collision caches the plan was certified against (deterministic given the
    cache — :func:`dishsim.placement.derive_slots`). Each class is read under its own active
    object: the placement mode, and therefore the derivation, is per class.

    Raises:
        FileNotFoundError: A class's cache for this state has not been baked.
    """
    out: dict[str, dict[int, placement.SlotFrame]] = {}
    for cls in classes:
        cdir = config.scenario_cache_dir(state, object_name=cls)
        if not os.path.exists(os.path.join(cdir, "scene_state.json")):
            raise FileNotFoundError(
                f"missing cache {cdir} — bake it with: scripts/setup/build_state.py "
                f"--machine {config.MACHINE} --placement {config.BASE_PLACEMENT} "
                f"--state {state} --classes {cls}")
        with config.active_object(cls):
            out[cls] = {s.slot_id: s for s in placement.derive_slots(cdir)}
    return out


def main() -> int:
    phase = next((p for p in PLAN["phases"] if p["state"] == args_cli.state), None)
    items = list(phase.get("items", [])) if phase else []
    if not items:
        print(f"[RESULT] FAIL (plan places nothing in state {args_cli.state!r})")
        return 1
    classes = sorted({it["object_class"] for it in items})
    print(f"[INFO] {len(items)} planned items in {args_cli.state} "
          f"({PLAN['machine']} @ {PLAN['base_placement']}, classes {', '.join(classes)})")

    try:
        slots = slot_frames(args_cli.state, classes)
    except FileNotFoundError as exc:
        print(f"[RESULT] FAIL ({exc})")
        return 1

    # The at-goal datum must be the slot the plan was CERTIFIED against, not merely one derived
    # from today's config: PLACEMENT_MODES is outside geometry.config_hash, so the manifest's
    # staleness check cannot see a retuned slot datum and the run would silently score items
    # against a slot the plan never used.
    for it in items:
        drift_m = float(np.linalg.norm(
            slots[it["object_class"]][it["slot_id"]].T_base_slot[:3, 3]
            - np.asarray(it["T_base_slot"], dtype=float)[:3, 3]))
        if drift_m > 5e-4:
            print(f"[RESULT] FAIL (plan slot table drifted: {it['item_id']} slot "
                  f"{it['slot_id']} moved {drift_m * 1e3:.2f} mm since the plan was written — "
                  f"re-run scripts/setup/plan_full_load.py)")
            return 1

    # ---- Kit scene: every planned item spawned parked off-stage -----------------------------
    obj_specs = [{"name": it["item_id"], "usd_path": config.OBJECTS[it["object_class"]].usd_path,
                  "pos": park_pos_w(i), "quat": (0.0, 0.0, 0.0, 1.0),
                  "color": config.item_color(it["item_id"])}
                 for i, it in enumerate(items)]
    park_w = {s["name"]: s["pos"] for s in obj_specs}

    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg()))
    rig = None
    if args_cli.video:
        assert args_cli.enable_cameras, "--video needs --enable_cameras"
        ep_cam = config.EPISODE_CAMERA
        rig = CameraRig({**{k: v for k, v in config.CAMERAS.items()},
                         "episode": (tuple(ep_cam["eye"]), tuple(ep_cam["target"]),
                                     dict(ep_cam["lens"]))}, hw=config.CAMERA_HW)
    scene = InteractiveScene(dscene.make_scene_cfg(objects=obj_specs))
    sim.reset()
    if rig is not None:
        rig.apply_poses(sim.device)
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    device = scene["dishwasher"].data.joint_pos.torch.device
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_base_w = T_inv(T_w_base)

    video = {"writer": None, "frame": 0}

    def step(n: int = 1) -> None:
        for _ in range(n):
            dscene.hold_targets(scene)
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
            video["frame"] += 1
            if video["writer"] is not None and video["frame"] % 2 == 0:
                # only the timelapse camera: rig.update() would render all four every frame,
                # which is exactly the waste CameraRig.grab_one's docstring warns about.
                rig.cams["episode"].update(dt)
                video["writer"].add(rig.grab_one("episode"))

    def teleport(item_id: str, T_base=None, pos_w=None, hover_m: float = 0.0) -> None:
        """Put an item at a base-frame pose (or a world position), velocities zeroed.

        ``hover_m`` lifts along world z, which is base z: every base placement is a pure yaw
        about world z (``config.BASE_PLACEMENTS``), the v1 fill's idiom.
        """
        if pos_w is None:
            pos_w, quat = T_to_pos_quat(T_w_base @ np.asarray(T_base, dtype=float))
            pos_w = np.asarray(pos_w, dtype=float).copy()
            pos_w[2] += hover_m
        else:
            quat = (0.0, 0.0, 0.0, 1.0)
        scene[item_id].write_root_pose_to_sim_index(root_pose=torch.tensor(
            np.concatenate([np.asarray(pos_w, dtype=float), np.asarray(quat, dtype=float)])[None],
            dtype=torch.float32, device=device))
        scene[item_id].write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=device))

    def measured(item_id: str) -> np.ndarray:
        """The item's live pose in the base frame, shape [4, 4] (physics-backed buffers)."""
        return T_base_w @ make_T(scene[item_id].data.root_pos_w.torch[0].cpu().numpy(),
                                 scene[item_id].data.root_quat_w.torch[0].cpu().numpy())

    step(WARMUP_STEPS)  # racks settle at the scenario extensions; the machine is empty

    media_dir = os.path.join(PROJECT_ROOT, "media", "capacity", config.MACHINE, args_cli.state)
    if rig is not None:
        for _ in range(CAMERA_WARMUP_FRAMES):  # burn black frames before the timelapse opens
            step(1)
            rig.update(dt)
        video["writer"] = VideoWriter(os.path.join(media_dir, "fill_timelapse.mp4"),
                                      fps=config.CAMERA_FPS)

    def judge(item_id: str, cls: str, slot_id: int, hist: list) -> tuple[bool, dict, dict]:
        """Settle verdict for one item from its drift history: (seated, verdict, metrics)."""
        settled = hist[-1]
        T_planned = planned[item_id]
        drift_p = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3]))
        drift_deg = rearrange.rot_angle_deg(hist[0], hist[-1])
        dev_p = float(np.linalg.norm(settled[:3, 3] - T_planned[:3, 3]))
        # v1's gate, in this branch's constants: the item must have stopped moving AND still be
        # near the pose it was commanded to. The deviation term is what separates "resting" from
        # "resting on the floor next to the machine" — both hold still.
        seated = (drift_p < rearrange.STABLE_POS_M and drift_deg < rearrange.STABLE_ROT_DEG
                  and dev_p < rearrange.MOVE_DEV_MAX_M)
        with config.active_object(cls):
            verdict = placement.evaluate_placement(slots[cls][slot_id], settled)
        return seated, verdict, {"drift_mm": round(drift_p * 1e3, 1),
                                 "drift_deg": round(drift_deg, 2),
                                 "dev_mm": round(dev_p * 1e3, 1)}

    planned = {it["item_id"]: np.asarray(it["T_base_obj"], dtype=float) for it in items}

    # ---- the fill: one item at a time, each gated on its own settle ---------------------------
    records, in_machine = [], []
    for i, it in enumerate(items):
        item_id, cls = it["item_id"], it["object_class"]
        before = {m: measured(m) for m in in_machine}  # to catch neighbours knocked by this drop
        teleport(item_id, T_base=planned[item_id], hover_m=ARRIVAL_HOVER_M)
        hist = []
        for s in range(rearrange.SETTLE_STEPS_MOVE):
            step(1)
            if s >= rearrange.SETTLE_STEPS_MOVE - rearrange.DRIFT_WINDOW:
                hist.append(measured(item_id))

        seated, verdict, metrics = judge(item_id, cls, it["slot_id"], hist)
        # An arrival that shoves a settled neighbour is a real capacity failure, invisible to
        # the arriving item's own gate — the rearrangement oracle's disturbance test, reused.
        disturbed = [m for m in in_machine
                     if np.linalg.norm(measured(m)[:3, 3] - before[m][:3, 3]) > rearrange.DISTURB_POS_M
                     or rearrange.rot_angle_deg(before[m], measured(m)) > rearrange.DISTURB_ROT_DEG]
        slot = slots[cls][it["slot_id"]]
        records.append({
            "item_id": item_id, "object_class": cls, "slot_id": it["slot_id"],
            "slot_name": it.get("slot_name"), "mode": slot.mode, "rack": slot.rack,
            "seated": bool(seated), **metrics,
            # at_goal is conjoined with seated so it can never exceed it: a still-moving item
            # can pass through its tolerance disc at the sampling instant, and a parked item is
            # 2 m away from the machine entirely.
            "at_goal": bool(seated and verdict["ok"]),
            "lateral_mm": round(verdict["lateral_m"] * 1e3, 1),
            "tilt_deg": round(verdict["tilt_deg"], 2),
            "disturbed_on_arrival": disturbed,
        })
        print(f"[INFO] {i + 1}/{len(items)} {item_id} ({cls} -> slot {it['slot_id']} "
              f"{it.get('slot_name', '?')}): {'seated' if seated else 'UNSEATED'}, "
              f"drift {metrics['drift_mm']} mm / {metrics['drift_deg']} deg, "
              f"dev {metrics['dev_mm']} mm, at_goal={bool(seated and verdict['ok'])}"
              + (f", DISTURBED {disturbed}" if disturbed else ""))
        if not seated:
            # honest capacity: park it again and keep filling — an unstable item must neither
            # end the run nor sit in the tableau disturbing the items that arrive after it
            print(f"[WARN] {item_id}: unstable settle — parked, not counted")
            teleport(item_id, pos_w=park_w[item_id])
            step(PARK_STEPS)
        else:
            in_machine.append(item_id)

    # ---- final pass: the verdict is the FINISHED tableau, not per-arrival snapshots -----------
    # Per-arrival verdicts freeze at the moment each item lands, so a later drop that knocks an
    # earlier item out of its slot would otherwise never be seen. Re-settle once and re-judge
    # everything still in the machine; these are the numbers the acceptance bar reads.
    by_id = {r["item_id"]: r for r in records}
    hist_final: dict[str, list] = {m: [] for m in in_machine}
    for _ in range(rearrange.DRIFT_WINDOW):
        step(1)
        for m in in_machine:
            hist_final[m].append(measured(m))
    for m in in_machine:
        rec = by_id[m]
        seated_f, verdict_f, metrics_f = judge(m, rec["object_class"], rec["slot_id"], hist_final[m])
        rec["seated_final"] = bool(seated_f)
        rec["at_goal_final"] = bool(seated_f and verdict_f["ok"])
        rec["drift_final_mm"], rec["dev_final_mm"] = metrics_f["drift_mm"], metrics_f["dev_mm"]
        if not rec["at_goal_final"] and rec["at_goal"]:
            print(f"[WARN] {m}: was at goal on arrival, not at goal in the finished load "
                  f"(dev {metrics_f['dev_mm']} mm)")
    for r in records:  # parked items are not in the machine at the end, by construction
        r.setdefault("seated_final", False)
        r.setdefault("at_goal_final", False)

    n_planned = len(records)
    n_seated = sum(r["seated_final"] for r in records)
    n_at_goal = sum(r["at_goal_final"] for r in records)
    fraction = n_seated / n_planned

    # ---- media: close the timelapse on the settled load, then the final stills ---------------
    media = {}
    if rig is not None:
        step(60)  # a beat of the finished load before the clip ends
        video["writer"].close()
        media["timelapse"] = os.path.relpath(video["writer"].path, PROJECT_ROOT)
        video["writer"] = None
        rig.update(dt)
        media["stills"] = [os.path.relpath(p, PROJECT_ROOT)
                           for p in rig.save_stills(media_dir, "fill")]
        print(f"[INFO] fill media -> {os.path.relpath(media_dir, PROJECT_ROOT)}")

    # The state belongs in the name: a machine has several loadable states, and verifying a
    # second one must not overwrite the first one's record.
    out_path = os.path.join(PROJECT_ROOT, "results", "capacity", config.MACHINE,
                            config.BASE_PLACEMENT, f"settled_verification_{args_cli.state}.json")
    if args_cli.out:
        out_path = (args_cli.out if os.path.isabs(args_cli.out)
                    else os.path.join(PROJECT_ROOT, args_cli.out))
    doc = {
        "schema_version": 1,
        "plan": os.path.relpath(_plan_path, PROJECT_ROOT),
        "machine": config.MACHINE, "base_placement": config.BASE_PLACEMENT,
        "state": args_cli.state,
        "gates": {"arrival_hover_m": ARRIVAL_HOVER_M,
                  "settle_steps": rearrange.SETTLE_STEPS_MOVE,
                  "drift_window": rearrange.DRIFT_WINDOW,
                  "stable_pos_m": rearrange.STABLE_POS_M,
                  "stable_rot_deg": rearrange.STABLE_ROT_DEG,
                  "dev_max_m": rearrange.MOVE_DEV_MAX_M,
                  "seated_fraction_bar": SEATED_FRACTION_BAR},
        "n_planned": n_planned, "n_seated": n_seated, "n_at_goal": n_at_goal,
        "fraction": round(fraction, 4),
        "counts_planned": {cls: sum(r["object_class"] == cls for r in records) for cls in classes},
        "counts_seated": {cls: sum(r["seated_final"] for r in records if r["object_class"] == cls)
                          for cls in classes},
        "items": records,
        "media": media,
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    tmp = out_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(tmp, out_path)
    print(f"[INFO] wrote {os.path.relpath(out_path, PROJECT_ROOT)}")

    print(f"[INFO] settled capacity: {n_seated}/{n_planned} seated ({fraction:.0%}), "
          f"{n_at_goal}/{n_planned} at goal (finished load)")
    # Both arms, because "seated" only means stopped-moving-near-the-target: a load where every
    # item rests but half of them rest outside their slot's tolerances is not a verified load.
    if n_seated < SEATED_FRACTION_BAR * n_planned:
        print(f"[RESULT] FAIL (seated {n_seated}/{n_planned} = {fraction:.0%} "
              f"< {SEATED_FRACTION_BAR:.0%} bar)")
        return 1
    if n_at_goal < SEATED_FRACTION_BAR * n_planned:
        print(f"[RESULT] FAIL (at goal {n_at_goal}/{n_planned} = {n_at_goal / n_planned:.0%} "
              f"< {SEATED_FRACTION_BAR:.0%} bar)")
        return 1
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception as exc:  # a crash must still print the line the repo judges runs by
        import traceback

        traceback.print_exc()
        print(f"[RESULT] FAIL ({type(exc).__name__}: {exc})")
        code = 1
    simulation_app.close()
    raise SystemExit(code)
