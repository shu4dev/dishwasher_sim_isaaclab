# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Trial trajectory recording + playback format (the Phase 2 -> Phase 3 handoff).

An experiment records the *measured* scene state every physics step; evaluation replays that
recording kinematically to render video, and never re-runs the planner or the physics.

Why measured state is enough: a PhysX articulation is reduced-coordinate, so every link pose
is a pure function of ``(root_pose, joint_pos)`` and the robot here is a tree with no loop
closure. Writing all 12 robot DOFs (including the 5 mimic-driven finger joints) and calling
``sim.forward()`` reproduces every link transform exactly by forward kinematics — the mimic
constraints only govern how the DOFs *evolve* under the solver, which playback never runs.

Joint and body **names** are stored with the arrays, so a recording never couples to the USD
joint ordering of the machine that produced it; :meth:`TrajectoryRecording.robot_joint_order`
remaps on load.

Storage is a single ``.npz`` per trial with the provenance ``meta`` embedded as a member, so
the data can never be separated from the configuration that produced it (~0.3 MB compressed
for a 4000-step trial). This module is Kit-free: ``numpy`` only, plus an optional ``torch``
buffer on the recording side.
"""

import json
import os
from dataclasses import dataclass, field

import numpy as np

from . import config

#: Bumped when the array set or meta contract changes incompatibly.
SCHEMA_VERSION = 1

#: Per-step stage codes. Stored as uint8; the legend travels inside every recording's meta so
#: an old file stays readable after this table grows.
PHASE_LEGEND: dict[int, str] = {
    0: "idle",
    1: "rack-approach",
    2: "rack-engage",
    3: "rack-grip-close",
    4: "rack-slide",
    5: "rack-settle",
    6: "rack-grip-open",
    7: "rack-disengage",
    8: "pick-approach",
    9: "pick-descend",
    10: "pick-pregrasp-settle",
    11: "pick-close",
    12: "pick-hold",
    13: "pick-weld-verify",
    14: "pick-lift",
    15: "welded-close",
    16: "place-execute",
    17: "place-hold",
    18: "place-open",
    19: "post-open-hold",
    20: "release-settle",
    21: "retract",
    22: "final-settle",
    23: "no-goal-idle",
    255: "unknown",
}
PHASE_CODE: dict[str, int] = {v: k for k, v in PHASE_LEGEND.items()}

#: Physics steps between kinematic checksum samples (``verify_replay.py`` compares these).
CHECK_STRIDE = 60

_FLOAT_KEYS = ("robot_joint_pos", "dishwasher_joint_pos", "object_root_pose_w")


def phase_code(name: str) -> int:
    """Stage code for a stage name (unknown names map to 255, never raising mid-trial)."""
    return PHASE_CODE.get(name, 255)


@dataclass
class TrajectoryRecorder:
    """Accumulates per-step measured state during a trial (Kit side).

    Sampling is deliberately allocation-free: one preallocated device tensor is filled with
    device-to-device copies and moved to host once, at :meth:`end`. The runner drives it from
    a single place — the function that owns ``sim.step()`` — so a step can never go
    unrecorded.

    Usage::

        rec = TrajectoryRecorder(max_steps=40_000)
        rec.begin(scene, sim, meta)       # after the trial reset settles
        ...                               # rec.sample() inside step_sim(); rec.mark_captured()
        rec.end(path)                     # writes the .npz

    Args:
        max_steps: Buffer capacity [physics steps]; sampling past it raises.
        check_stride: Physics steps between kinematic checksum samples.
    """

    max_steps: int = 40_000
    check_stride: int = CHECK_STRIDE

    active: bool = field(default=False, init=False)
    phase: int = field(default=0, init=False)
    weld: bool = field(default=False, init=False)
    _n: int = field(default=0, init=False)

    def __len__(self) -> int:
        return self._n

    def begin(self, scene, sim, meta: dict, object_keys: list | None = None) -> None:
        """Start recording from the current scene state (call after the trial reset settles).

        Args:
            scene: Live interactive scene.
            sim: Simulation context.
            meta: Provenance embedded in the recording.
            object_keys: Scene keys of the manipulable objects to track. Defaults to the
                single ``"carried_object"``, which keeps every existing recording and every
                Phase 3 consumer byte-identical. A multi-object episode passes one key per
                item; ``object_root_pose_w`` then widens to ``[N, 7 * n_objects]`` and the
                names land in ``meta["object_keys"]`` so a reader can slice it without
                guessing.
        """
        import torch  # noqa: PLC0415

        self._scene, self._sim, self._meta = scene, sim, dict(meta)
        robot, dw = scene["robot"], scene["dishwasher"]
        self._object_keys = list(object_keys) if object_keys else ["carried_object"]
        self._objs = [scene[k] for k in self._object_keys]
        self._obj = self._objs[0]
        self._robot, self._dw = robot, dw
        device = robot.data.joint_pos.torch.device
        self._n_rq = int(robot.data.joint_pos.torch.shape[1])
        self._n_dq = int(dw.data.joint_pos.torch.shape[1])
        self._n_body = int(robot.data.body_link_pos_w.torch.shape[1])

        width = self._n_rq + self._n_dq + 7 * len(self._objs)
        self._buf = torch.empty((self.max_steps, width), dtype=torch.float32, device=device)
        self._check = torch.empty(
            (self.max_steps // self.check_stride + 2, self._n_body, 7), dtype=torch.float32, device=device
        )
        self._check_idx: list[int] = []
        self._phase = np.zeros(self.max_steps, dtype=np.uint8)
        self._weld = np.zeros(self.max_steps, dtype=bool)
        self._captured = np.zeros(self.max_steps, dtype=bool)
        self._n = 0
        self._n_check = 0
        self.active = True
        self._meta.update(
            {
                "schema_version": SCHEMA_VERSION,
                "sim_dt": float(config.SIM_DT),
                "robot_joint_names": list(robot.joint_names),
                "dishwasher_joint_names": list(dw.joint_names),
                "robot_body_names": list(robot.body_names),
                "quat_convention": "xyzw",
                "frame": "world",
                "phase_legend": {str(k): v for k, v in PHASE_LEGEND.items()},
                "check_stride": int(self.check_stride),
                "object_keys": list(self._object_keys),
            }
        )

    def set_phase(self, name: str) -> None:
        """Tag subsequent samples with a stage name (see :data:`PHASE_LEGEND`)."""
        self.phase = phase_code(name)

    def set_weld(self, enabled: bool) -> None:
        """Record the weld constraint's state for subsequent samples."""
        self.weld = bool(enabled)

    def sample(self) -> None:
        """Capture the post-step scene state (no-op unless :attr:`active`)."""
        if not self.active:
            return
        if self._n >= self.max_steps:
            raise RuntimeError(f"trajectory buffer full ({self.max_steps} steps)")
        k = self._n
        a, b = self._n_rq, self._n_rq + self._n_dq
        self._buf[k, :a] = self._robot.data.joint_pos.torch[0]
        self._buf[k, a:b] = self._dw.data.joint_pos.torch[0]
        for i, obj in enumerate(self._objs):
            o = b + 7 * i
            self._buf[k, o : o + 3] = obj.data.root_pos_w.torch[0]
            self._buf[k, o + 3 : o + 7] = obj.data.root_quat_w.torch[0]
        self._phase[k] = self.phase
        self._weld[k] = self.weld
        if k % self.check_stride == 0:
            self._check[self._n_check, :, :3] = self._robot.data.body_link_pos_w.torch[0]
            self._check[self._n_check, :, 3:] = self._robot.data.body_link_quat_w.torch[0]
            self._check_idx.append(k)
            self._n_check += 1
        self._n = k + 1

    def mark_captured(self) -> None:
        """Flag the most recent sample as one that produced a video frame."""
        if self.active and self._n > 0:
            self._captured[self._n - 1] = True

    def end(self, path: str, extra_meta: dict | None = None) -> dict:
        """Flush the buffer to ``path`` (``.npz``); returns the meta that was embedded."""
        self.active = False
        n = self._n
        flat = self._buf[:n].cpu().numpy()
        a, b = self._n_rq, self._n_rq + self._n_dq
        meta = dict(self._meta)
        meta.update(extra_meta or {})
        meta.update({"n_steps": int(n), "n_captured": int(self._captured[:n].sum())})
        arrays = {
            "robot_joint_pos": flat[:, :a],
            "dishwasher_joint_pos": flat[:, a:b],
            "object_root_pose_w": flat[:, b : b + 7 * len(self._object_keys)],
            "phase": self._phase[:n],
            "weld": self._weld[:n],
            "captured": self._captured[:n],
            "check_step_idx": np.asarray(self._check_idx, dtype=np.int32),
            "check_body_pose_w": self._check[: self._n_check].cpu().numpy(),
            "meta": np.array(json.dumps(meta)),
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        np.savez_compressed(path, **arrays)
        return meta


@dataclass
class TrajectoryRecording:
    """A trial recording loaded from disk (Kit-free reader)."""

    robot_joint_pos: np.ndarray  # [N, n_robot_joints]
    dishwasher_joint_pos: np.ndarray  # [N, n_dw_joints]
    object_root_pose_w: np.ndarray  # [N, 7] world pos + XYZW quat
    phase: np.ndarray  # [N] uint8
    weld: np.ndarray  # [N] bool
    captured: np.ndarray  # [N] bool
    check_step_idx: np.ndarray  # [M] int32
    check_body_pose_w: np.ndarray  # [M, n_bodies, 7]
    meta: dict

    @staticmethod
    def load(path: str) -> "TrajectoryRecording":
        """Load a recording; validates the schema version."""
        with np.load(path, allow_pickle=False) as z:
            meta = json.loads(str(z["meta"]))
            if meta.get("schema_version") != SCHEMA_VERSION:
                raise ValueError(
                    f"{path}: schema_version {meta.get('schema_version')} != {SCHEMA_VERSION} "
                    "— re-record the trial with the current experiment runner"
                )
            return TrajectoryRecording(
                robot_joint_pos=z["robot_joint_pos"],
                dishwasher_joint_pos=z["dishwasher_joint_pos"],
                object_root_pose_w=z["object_root_pose_w"],
                phase=z["phase"],
                weld=z["weld"],
                captured=z["captured"],
                check_step_idx=z["check_step_idx"],
                check_body_pose_w=z["check_body_pose_w"],
                meta=meta,
            )

    def __len__(self) -> int:
        return int(len(self.robot_joint_pos))

    @property
    def captured_steps(self) -> np.ndarray:
        """Indices of the steps that produced a video frame."""
        return np.flatnonzero(self.captured)

    def phase_name(self, step: int) -> str:
        """Stage name at a step, from the recording's own legend."""
        return self.meta["phase_legend"].get(str(int(self.phase[step])), "unknown")

    def joint_order(self, live_names: list[str], kind: str = "robot") -> np.ndarray:
        """Column permutation mapping this recording's joints onto ``live_names``.

        Args:
            live_names: Joint names of the scene being replayed, in its own order.
            kind: ``"robot"`` or ``"dishwasher"``.

        Returns:
            Index array ``cols`` such that ``recorded[:, cols]`` matches ``live_names``.
        """
        recorded = self.meta[f"{kind}_joint_names"]
        missing = set(live_names) ^ set(recorded)
        if missing:
            raise ValueError(f"{kind} joint sets differ between recording and scene: {sorted(missing)}")
        index = {name: i for i, name in enumerate(recorded)}
        return np.array([index[name] for name in live_names], dtype=int)


def time_parameterize(
    path_q: np.ndarray,
    speed_rad_s: float = config.EXEC_JOINT_SPEED_RAD_S,
    dt: float = config.SIM_DT,
) -> np.ndarray:
    """Resample a waypoint path at a constant joint-speed cap.

    Args:
        path_q: Waypoints, shape [N, 6].
        speed_rad_s: Max per-joint speed [rad/s].
        dt: Control period [s].

    Returns:
        Dense joint targets, shape [T, 6] (one row per physics step, ends exactly at the goal).
    """
    path_q = np.asarray(path_q, dtype=float)
    out = [path_q[0]]
    step = speed_rad_s * dt
    for a, b in zip(path_q[:-1], path_q[1:]):
        span = float(np.max(np.abs(b - a)))
        n = max(1, int(np.ceil(span / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)
