# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Visual-evidence capture: fixed camera rig, PNG stills, MP4 clips.

The cameras (poses in :data:`dishsim.config.CAMERAS`) are FIXED across all phases so shots stay
comparable. :class:`CameraRig` is Kit-only (imports ``isaaclab.sensors`` lazily); requires the
app to run with ``--headless --enable_cameras`` (dishsim.app) — without the flag camera sensors
return nothing. :class:`VideoWriter` is Kit-free (imageio/numpy).
"""

import os

import numpy as np


def release_sim_for_close(sim=None) -> None:
    """Flush python's stdio before ``simulation_app.close()``; otherwise a no-op on Isaac Lab 3.0.

    Isaac Lab 2.1 (Isaac Sim 4.5, corallab's isaac45-final commit) needed this to disarm a
    timeline on-stop callback that spun ``close()`` forever at full CPU, and its fast shutdown
    discarded block-buffered stdout. 3.0's ``SimulationContext`` has no such handle and 6.x
    flushes on close; the flush stays as cheap insurance for the ``[RESULT]`` line the logs are
    judged by. Kept so every standalone script's shutdown reads the same on both stacks.
    """
    import sys  # noqa: PLC0415

    sys.stdout.flush()
    sys.stderr.flush()


class CameraRig:
    """Named fixed cameras rendering RGB at a common resolution.

    Construct BEFORE ``sim.reset()``; call :meth:`apply_poses` after it; call :meth:`update`
    after each ``sim.step()`` you intend to capture; warm up >= 5 renders before trusting
    frames (early frames can come back black).
    """

    def __init__(self, cam_specs: dict[str, tuple], hw: tuple[int, int] = (720, 1280)):
        """Args:
            cam_specs: ``{name: (eye, target)}`` or ``{name: (eye, target, lens)}``. A carried
                lens wins over :data:`dishsim.config.CAMERA_LENS_DEFAULT`, which is what lets a
                replay re-render through the same optics the experiment was shot with instead of
                whatever the config happens to say now.
            hw: (height, width) in pixels.
        """
        import isaaclab.sim as sim_utils  # noqa: PLC0415  (Kit-only)
        from isaaclab.sensors import Camera, CameraCfg  # noqa: PLC0415
        from isaaclab_physx.renderers import IsaacRtxRendererCfg  # noqa: PLC0415

        # COPY, never alias: callers pass config.CAMERAS itself, and holding references
        # would let any future in-place re-aim silently overwrite the module-level poses
        # (measured, git history — the retired runtime re-aim feature corrupted recordings).
        from . import config  # noqa: PLC0415

        self._specs = {k: (tuple(v[0]), tuple(v[1])) for k, v in cam_specs.items()}
        #: Lens actually used per camera — a spec may carry its own (replay), otherwise config.
        self.lenses = {
            k: dict(v[2]) if len(v) > 2 and v[2] else dict(config.CAMERA_LENS_DEFAULT)
            for k, v in cam_specs.items()
        }
        self.cams: dict[str, object] = {}
        for name in cam_specs:
            lens = self.lenses[name]
            self.cams[name] = Camera(
                CameraCfg(
                    prim_path=f"/World/Cam_{name}",
                    update_period=0.0,
                    height=hw[0],
                    width=hw[1],
                    data_types=["rgb"],
                    # no per-env scene partitions (3.0's default): these world cameras carry no
                    # partition token, so they would see only the ground and lights, not env_0
                    renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False),
                    spawn=sim_utils.PinholeCameraCfg(
                        focal_length=float(lens["focal_length"]),
                        focus_distance=400.0,
                        horizontal_aperture=float(lens["horizontal_aperture"]),
                        clipping_range=(0.05, 100.0),
                    ),
                )
            )

    def apply_poses(self, device: str) -> None:
        import torch  # noqa: PLC0415

        for name, cam in self.cams.items():
            eye, target = self._specs[name]
            cam.set_world_poses_from_view(
                eyes=torch.tensor([eye], dtype=torch.float32, device=device),
                targets=torch.tensor([target], dtype=torch.float32, device=device),
            )


    def update(self, dt: float) -> None:
        for cam in self.cams.values():
            cam.update(dt)

    def grab_one(self, name: str) -> np.ndarray:
        """Latest RGB frame from ONE camera (call :meth:`update` first).

        :meth:`grab` copies every camera off the device; a caller writing a single-camera video
        then discards all but one. At 720x1280 that is ~3.7 MB of wasted transfer per captured
        frame, per extra camera — which is what makes adding a wide episode view look expensive
        when it is not.
        """
        frame = self.cams[name].data.output["rgb"].torch[0].detach().cpu().numpy()
        return frame[..., :3].astype(np.uint8)

    def grab(self) -> dict[str, np.ndarray]:
        """Latest RGB frame per camera as HxWx3 uint8 (call :meth:`update` first)."""
        out = {}
        for name, cam in self.cams.items():
            frame = cam.data.output["rgb"].torch[0].detach().cpu().numpy()
            out[name] = frame[..., :3].astype(np.uint8)
        return out

    def save_stills(self, out_dir: str, tag: str) -> list[str]:
        """Write ``<out_dir>/<tag>_<cam>.png`` per camera; returns the paths."""
        from PIL import Image  # noqa: PLC0415

        os.makedirs(out_dir, exist_ok=True)
        paths = []
        for name, frame in self.grab().items():
            path = os.path.join(out_dir, f"{tag}_{name}.png")
            Image.fromarray(frame).save(path)
            paths.append(path)
        return paths


class VideoWriter:
    """Thin imageio/ffmpeg MP4 writer (H.264, keeps files small)."""

    def __init__(self, path: str, fps: int = 30):
        import imageio  # noqa: PLC0415

        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self._writer = imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=None)
        self.frames = 0

    def add(self, frame: np.ndarray) -> None:
        self._writer.append_data(frame)
        self.frames += 1

    def close(self) -> None:
        self._writer.close()
