# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The rearrangement benchmark: instances, the closed-loop episode driver, the greedy baseline.

A problem instance fixes a machine rack state and gives every object a settled INITIAL pose
and an exact TARGET pose (the capacity plan's certified release-hover pose, carrying its
SlotFrame so the at-goal verdict reuses :func:`placement.evaluate_placement`). A move
teleports one object to any commanded pose — in-machine or on the counter buffer band, which
is just physical space. Episodes ABORT on the first fault (colliding command, unstable
settle, disturbed neighbor) or at the move budget.

Kit-free by construction: :func:`run_episode` is parameterized by an ``oracle`` (executes a
move, returns settled poses + fault — Isaac in ``scripts/experiment/run_rearrange.py``, a toy
in ``tests/test_rearrange.py``) and a ``world`` (FCL feasibility, :class:`ArrangementWorld`).

Context discipline: the CALLER applies machine -> object -> scenario -> placement before
using anything here (the instance records which context it was generated under).
"""

import json
import os
import time
from dataclasses import dataclass

import numpy as np

from . import config, placement
from .collision_world import CollisionWorld, load_object_pieces
from .transforms import T_inv, make_T

# Fault/verdict knobs. Defaults are the measured settle numbers from the retired
# capacity_fill/reveal campaigns (git history), not eyeballed. Module constants on purpose:
# nothing here may touch config.py (geometry.config_hash must stay byte-identical).
SETTLE_STEPS_MOVE = 75      # physics steps settled after every move
SETTLE_STEPS_INIT = 150     # physics steps settled at episode reset
DRIFT_WINDOW = 30           # stability window: pose drift measured over the last N steps
STABLE_POS_M = 0.005        # moved item must stop drifting (measured per-item gate)
STABLE_ROT_DEG = 3.0
MOVE_DEV_MAX_M = 0.06       # settled vs commanded pose (absorbs the hover drop + roll-to-tine)
DISTURB_POS_M = 0.010       # any NON-moved item beyond this from its pre-move pose = fault
DISTURB_ROT_DEG = 10.0
INIT_MATCH_POS_M = 0.010    # episode reset must reproduce the instance's settled initials
INIT_MATCH_ROT_DEG = 10.0
COUNTER_GRID_PITCH_M = 0.11  # buffer candidate spacing; FCL gates actual use


def rot_angle_deg(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """Rotation angle [deg] between two homogeneous transforms."""
    R = np.asarray(T_a)[:3, :3].T @ np.asarray(T_b)[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


@dataclass(frozen=True)
class Move:
    """One teleport: put ``item_id`` at ``T_base_obj`` (commanded pose, base frame)."""

    item_id: str
    T_base_obj: np.ndarray


@dataclass
class Instance:
    """One rearrangement problem, loaded from / dumped to a JSON artifact.

    ``items`` entries: ``{"item_id", "object_class", "T_base_init" (np [4,4], measured
    settled), "target": {"T_base_obj" (np [4,4], certified release-hover pose),
    "slot": SlotFrame.to_json()}}``.
    """

    name: str
    machine: str
    base_placement: str
    state: str
    items: list
    meta: dict

    @staticmethod
    def load(path: str) -> "Instance":
        with open(path) as f:
            doc = json.load(f)
        for it in doc["items"]:
            it["T_base_init"] = np.array(it["T_base_init"])
            it["target"]["T_base_obj"] = np.array(it["target"]["T_base_obj"])
        return Instance(name=doc["name"], machine=doc["machine"],
                        base_placement=doc["base_placement"], state=doc["state"],
                        items=doc["items"], meta=doc.get("meta", {}))

    def dump(self, path: str) -> str:
        items = []
        for it in self.items:
            items.append({**it, "T_base_init": np.asarray(it["T_base_init"]).tolist(),
                          "target": {**it["target"],
                                     "T_base_obj": np.asarray(it["target"]["T_base_obj"]).tolist()}})
        doc = {"schema_version": 1, "name": self.name, "machine": self.machine,
               "base_placement": self.base_placement, "state": self.state,
               "meta": self.meta, "items": items}
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(doc, f, indent=1)
        os.replace(tmp, path)
        return path

    def item(self, item_id: str) -> dict:
        return next(it for it in self.items if it["item_id"] == item_id)


def at_goal(item: dict, T_base_obj: np.ndarray) -> bool:
    """Is this item's SETTLED pose inside its placement mode's tolerances at its target slot?"""
    slot = placement.SlotFrame.from_json(item["target"]["slot"])
    with config.active_object(item["object_class"]):
        return bool(placement.evaluate_placement(slot, np.asarray(T_base_obj))["ok"])


class ArrangementWorld:
    """The shared FCL mirror of the current arrangement (harness pre-check + algorithms).

    One :class:`CollisionWorld` per session: the statics of every per-class cache of one
    state are identical, so the first class's cache serves them all; each class's convex
    pieces load once.
    """

    def __init__(self, state: str, classes: list):
        classes = sorted(set(classes))
        with config.active_object(classes[0]):
            self.world = CollisionWorld(
                cache_dir=config.scenario_cache_dir(state, object_name=classes[0]))
        self.pieces = {}
        for cls in classes:
            with config.active_object(cls):
                self.pieces[cls] = load_object_pieces(
                    config.scenario_cache_dir(state, object_name=cls))
        self._poses: dict = {}
        self._cls: dict = {}

    def clear(self) -> None:
        """Drop every mirrored item (a generator starts each instance from the empty machine)."""
        for item_id in list(self._poses):
            self.world.remove_object(item_id)
        self._poses.clear()
        self._cls.clear()

    def sync(self, poses: dict, classes: dict) -> None:
        """Mirror measured settled poses (``{item_id: T_base_obj}``) into the FCL world."""
        for item_id, T in poses.items():
            T = np.asarray(T)
            self._cls[item_id] = classes[item_id]
            if self.world.has_object(item_id):
                self.world.set_object_pose(item_id, T)
            else:
                self.world.add_object(item_id, self.pieces[classes[item_id]], T)
            self._poses[item_id] = T

    def move_collides(self, item_id: str, T: np.ndarray, object_class: str = None) -> bool:
        """Would ``item_id`` at commanded ``T`` interpenetrate statics or any OTHER item?"""
        return bool(self.blockers(item_id, T, object_class))

    def blockers(self, item_id: str, T: np.ndarray, object_class: str = None) -> list:
        """Names blocking ``item_id`` at ``T``: item ids and/or static body names.

        ``object_class`` is only needed for an item the world has never seen (a generator
        probing candidates before committing them).
        """
        cls = self._cls.get(item_id, object_class)
        had = self.world.has_object(item_id)
        if had:
            self.world.remove_object(item_id)
        try:
            _, pairs = self.world.object_in_collision(
                self.pieces[cls], np.asarray(T), return_pairs=True)
        finally:
            if had:
                self.world.add_object(item_id, self.pieces[cls], self._poses[item_id])
        return sorted({partner for _, partner in pairs})

    def buffer_poses(self, object_class: str) -> list:
        """Candidate release poses on the counter buffer band (base frame), grid order.

        World-identity orientation (Bosch classes stage as authored), bottom at the counter
        top + release hover; FCL decides which cells are actually free.
        """
        rect = config.TASK["spawn_rect_w"]
        spec = config.OBJECTS[object_class]
        top_z = config.COUNTERTOP_CENTER_W[2] + config.COUNTERTOP_SIZE[2] / 2.0
        z = top_z + spec.bbox_half[2] + config.RELEASE_HOVER_M
        T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
        out = []
        for x in np.arange(rect["x_min"] + 0.05, rect["x_max"] - 0.05 + 1e-9, COUNTER_GRID_PITCH_M):
            for y in np.arange(rect["y_min"] + 0.05, rect["y_max"] - 0.05 + 1e-9, COUNTER_GRID_PITCH_M):
                T_w = np.eye(4)
                T_w[:3, 3] = (x, y, z)
                out.append(T_base_w @ T_w)
        return out


def run_episode(instance: Instance, algo, world, oracle, budget: int,
                algorithm_name: str = "") -> dict:
    """Drive one closed-loop episode; abort on the first fault; return the episode record.

    ``oracle.execute(move) -> (poses, fault, move_info)`` — settled base-frame poses for the
    whole roster, ``fault in (None, "unstable-settle", "disturbed")``, and a per-move info
    dict (e.g. settle_dev_mm, disturbed ids). ``oracle.at_goal(item, T) -> bool``.
    """
    classes = {it["item_id"]: it["object_class"] for it in instance.items}
    poses = {it["item_id"]: np.asarray(it["T_base_init"]) for it in instance.items}
    world.sync(poses, classes)

    def observe(moves_used):
        return {"items": {it["item_id"]: {"object_class": it["object_class"],
                                          "T_base_obj": poses[it["item_id"]],
                                          "at_goal": oracle.at_goal(it, poses[it["item_id"]])}
                          for it in instance.items},
                "moves_used": moves_used, "budget": budget}

    obs = observe(0)
    at_goal_initial = sum(v["at_goal"] for v in obs["items"].values())
    algo.reset(instance, world)

    solved, abort = False, None
    planning_time_s, move_log = [], []
    moves_used = 0
    while True:
        if all(v["at_goal"] for v in obs["items"].values()):
            solved = True
            break
        t0 = time.perf_counter()
        move = algo.next_move(obs)
        planning_time_s.append(round(time.perf_counter() - t0, 6))
        if move is None:
            abort = "give-up"
            break
        if moves_used >= budget:
            abort = "budget"
            break
        if world.move_collides(move.item_id, move.T_base_obj):
            abort = "collision-command"
            move_log.append({"item_id": move.item_id,
                             "T_base_obj": np.asarray(move.T_base_obj).tolist(),
                             "blockers": world.blockers(move.item_id, move.T_base_obj)})
            break
        poses, fault, info = oracle.execute(move)
        poses = {k: np.asarray(v) for k, v in poses.items()}
        moves_used += 1
        move_log.append({"item_id": move.item_id,
                         "T_base_obj": np.asarray(move.T_base_obj).tolist(), **info})
        world.sync(poses, classes)
        if fault is not None:
            abort = fault
            break
        obs = observe(moves_used)

    final = {it["item_id"]: oracle.at_goal(it, poses[it["item_id"]]) for it in instance.items}
    n = len(instance.items)
    return {
        "instance": instance.name, "algorithm": algorithm_name,
        "machine": instance.machine, "state": instance.state,
        "solved": bool(solved), "abort": abort,
        "n_items": n, "at_goal_initial": int(at_goal_initial),
        "at_goal_final": int(sum(final.values())),
        "fraction_at_goal": round(sum(final.values()) / n, 4) if n else 1.0,
        "moves_used": moves_used, "budget": budget,
        "planning_time_s": planning_time_s,
        "moves": move_log,
        "final_poses": {k: np.asarray(v).tolist() for k, v in poses.items()},
    }


class Greedy:
    """Baseline: send home whatever fits; relocate one blocker to the buffer when stuck.

    # ponytail: one-blocker lookahead; swap-cycles beyond one relocation need a real planner
    """

    def reset(self, instance: Instance, world) -> None:
        self.instance, self.world = instance, world
        self._relocated: set = set()

    def next_move(self, obs):
        items = obs["items"]
        # pass 1: any misplaced item whose target release pose is free -> send it home
        for it in self.instance.items:
            item_id = it["item_id"]
            if items[item_id]["at_goal"]:
                self._relocated.discard(item_id)
                continue
            T_target = it["target"]["T_base_obj"]
            if not self.world.move_collides(item_id, T_target):
                return Move(item_id, T_target)
        # pass 2: relocate the first eligible blocker of a misplaced item to a buffer spot
        for it in self.instance.items:
            item_id = it["item_id"]
            if items[item_id]["at_goal"]:
                continue
            blockers = self.world.blockers(item_id, it["target"]["T_base_obj"])
            if any(b not in items for b in blockers):
                continue  # static-blocked target — nothing a relocation can fix
            for b in blockers:
                if items[b]["at_goal"] or b in self._relocated:
                    continue
                for T_buf in self.world.buffer_poses(items[b]["object_class"]):
                    if not self.world.move_collides(b, T_buf):
                        self._relocated.add(b)
                        return Move(b, T_buf)
        return None  # give up: static-blocked, blockers spent their buffer trip, or no space
