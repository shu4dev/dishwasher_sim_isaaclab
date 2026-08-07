# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kinematic playback of a recorded trial (Kit side, no physics).

Every body in the scene is written from the recording each frame, so nothing is simulated:
no gravity, no contacts, no drive, no weld. That is what makes evaluation independent of the
experiment — the geometry on screen is exactly the geometry that was measured, not a
re-execution that could diverge.

Two rules this module exists to enforce:

- **Write the whole scene, then** ``sim.step()``. ``sim.forward()`` updates the physics
  buffers — a read-back of the link poses is then exactly right — but it does **not** push the
  new transforms to the renderer, so the camera keeps returning the pre-write image (measured:
  11 dB vs 23 dB against the live capture). Only a step propagates transforms to RTX.
  Stepping is safe here precisely because playback rewrites every body every frame and the
  simulation runs with gravity disabled and zero velocities: one step cannot accumulate.
- **Root poses come from** :func:`dishsim.scene.write_default_states`. Joint positions are
  written per frame, but the articulation *roots* are not — without that call the dishwasher
  sits ~25 cm off its spawn pose and the replay renders a subtly wrong scene.

Joint columns are remapped by name, so a recording replays correctly even if the scene's USD
joint ordering differs from the machine that produced it.
"""

import numpy as np

from . import config


def scene_objects_for(recording) -> list:
    """``objects=`` spec for :func:`dishsim.scene.make_scene_cfg` reproducing a recording's items.

    A single-object recording returns ``[]`` — the caller keeps ``with_object=True`` and the
    legacy ``carried_object`` prim, so every existing recording replays exactly as before. A
    multi-object episode returns one spec per item, named after its ``item_id`` so the scene
    keys match what :class:`TrajectoryPlayer` expects to write.

    Spawn poses come from the recording's FIRST frame, so nothing has to be re-derived from a
    layout seed and playback starts where the episode did.

    Args:
        recording: A loaded :class:`dishsim.trajectory.TrajectoryRecording`.

    Returns:
        Object specs, empty for single-object recordings.
    """
    from . import config  # noqa: PLC0415

    keys = list(recording.meta.get("object_keys") or [])
    if not keys or keys == ["carried_object"]:
        return []
    classes = recording.meta.get("object_classes") or {}
    missing = [k for k in keys if k not in classes]
    if missing:
        raise RuntimeError(
            f"recording lists object keys {missing} with no class mapping in "
            f"meta['object_classes'] — re-record with the current runner"
        )
    poses = np.asarray(recording.object_root_pose_w)
    out = []
    for i, key in enumerate(keys):
        block = poses[0, 7 * i : 7 * (i + 1)]
        out.append({
            "name": key,
            "usd_path": config.OBJECTS[classes[key]].usd_path,
            "pos": tuple(float(v) for v in block[:3]),
            "quat": tuple(float(v) for v in block[3:]),
        })
    return out


class TrajectoryPlayer:
    """Drives a live scene from a :class:`~dishsim.trajectory.TrajectoryRecording`."""

    def __init__(self, scene, sim, recording, warmup_renders: int = 10) -> None:
        """
        Args:
            scene: The rebuilt ``InteractiveScene`` (same object + scenario as the recording).
            sim: The ``SimulationContext``.
            recording: The loaded recording.
            warmup_renders: Discarded renders before the first kept frame — the camera
                returns black for the first several pumps (see :class:`dishsim.media.CameraRig`).
        """
        import torch  # noqa: PLC0415

        self.scene, self.sim, self.rec = scene, sim, recording
        self.robot, self.dw = scene["robot"], scene["dishwasher"]
        # One recording may hold several manipulable objects (a multi-object episode). The key
        # list lives in the recording's meta; single-object recordings omit it and fall back to
        # the legacy name, so every existing recording replays byte-identically.
        self._obj_keys = list(recording.meta.get("object_keys") or ["carried_object"])
        self.objs = [scene[k] for k in self._obj_keys]
        self.obj = self.objs[0]
        self.device = self.robot.data.joint_pos.torch.device
        self._torch = torch

        origins = scene.env_origins
        assert float(np.abs(origins.cpu().numpy()).max()) < 1e-9, (
            "replay assumes a single env at the world origin; recorded poses are absolute world poses"
        )

        # remap recorded columns onto this scene's joint order (names, never positions)
        self._robot_cols = recording.joint_order(list(self.robot.joint_names), kind="robot")
        self._dw_cols = recording.joint_order(list(self.dw.joint_names), kind="dishwasher")
        self._robot_q = self._to_device(recording.robot_joint_pos[:, self._robot_cols])
        self._dw_q = self._to_device(recording.dishwasher_joint_pos[:, self._dw_cols])
        poses = np.asarray(recording.object_root_pose_w)
        assert poses.shape[1] == 7 * len(self.objs), (
            f"recording holds {poses.shape[1] // 7} object pose block(s) but the scene was given "
            f"{len(self.objs)} object key(s): {self._obj_keys}"
        )
        self._obj_pose = [self._to_device(poses[:, 7 * i : 7 * (i + 1)]) for i in range(len(self.objs))]
        self._zero_rq = torch.zeros((1, self._robot_q.shape[1]), dtype=torch.float32, device=self.device)
        self._zero_dq = torch.zeros((1, self._dw_q.shape[1]), dtype=torch.float32, device=self.device)
        self._zero_v6 = torch.zeros((1, 6), dtype=torch.float32, device=self.device)

        # body-name remap for the checksum comparison
        rec_bodies = recording.meta["robot_body_names"]
        live_bodies = list(self.robot.body_names)
        assert set(rec_bodies) == set(live_bodies), "robot body sets differ between recording and scene"
        index = {name: i for i, name in enumerate(rec_bodies)}
        self._body_cols = np.array([index[name] for name in live_bodies], dtype=int)
        self.warmup_renders = warmup_renders
        self._warmed = False

    def _to_device(self, arr: np.ndarray):
        return self._torch.as_tensor(np.ascontiguousarray(arr), dtype=self._torch.float32, device=self.device)

    def write_step(self, step: int) -> None:
        """Teleport the whole scene to the recorded state at ``step``.

        Drive *targets* are set to the same values as the states. Without that the position
        drives still act during the propagation step and pull the arm toward whatever target
        was last commanded — measured as up to 21 mm of link drift per frame.
        """
        self.robot.write_joint_position_to_sim_index(position=self._robot_q[step : step + 1])
        self.robot.write_joint_velocity_to_sim_index(velocity=self._zero_rq)
        self.robot.set_joint_position_target(self._robot_q[step : step + 1])
        self.dw.write_joint_position_to_sim_index(position=self._dw_q[step : step + 1])
        self.dw.write_joint_velocity_to_sim_index(velocity=self._zero_dq)
        self.dw.set_joint_position_target(self._dw_q[step : step + 1])
        for obj, pose in zip(self.objs, self._obj_pose):
            obj.write_root_pose_to_sim_index(root_pose=pose[step : step + 1])
            obj.write_root_velocity_to_sim_index(root_velocity=self._zero_v6)

    def read_body_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """Read-back robot link poses, ordered like the recording's ``robot_body_names``."""
        pos = self.robot.data.body_link_pos_w.torch[0].cpu().numpy()
        quat = self.robot.data.body_link_quat_w.torch[0].cpu().numpy()
        inv = np.argsort(self._body_cols)
        return pos[inv], quat[inv]

    def advance(self, rig=None, dt: float | None = None) -> None:
        """Propagate the written state to physics *and* the renderer (see module docs)."""
        self.scene.write_data_to_sim()  # push the drive targets set by write_step
        self.sim.step()
        if rig is not None:
            rig.update(dt if dt is not None else config.SIM_DT)

    def warmup(self, rig, dt: float | None = None) -> None:
        """Discard the first renders — early camera frames come back black."""
        if self._warmed or rig is None:
            return
        for _ in range(self.warmup_renders):
            self.advance(rig, dt)
            rig.grab()
        self._warmed = True


def assert_frames_vary(frames: list[np.ndarray], min_changed_frac: float = 0.10) -> tuple[bool, str]:
    """Guard against the frozen-render failure mode (identical frame every step).

    Args:
        frames: Captured frames in order.
        min_changed_frac: Minimum fraction of consecutive pairs that must differ.

    Returns:
        (ok, detail).
    """
    if len(frames) < 2:
        return True, "too few frames to compare"
    changed = sum(
        1 for a, b in zip(frames[:-1], frames[1:])
        if int(np.abs(a.astype(np.int16) - b.astype(np.int16)).max()) > 0
    )
    frac = changed / (len(frames) - 1)
    ok = frac >= min_changed_frac
    return ok, f"{frac * 100:.1f}% of consecutive frames differ ({changed}/{len(frames) - 1})"
