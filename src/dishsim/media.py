# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Visual-evidence capture: fixed camera rig, PNG stills, MP4 clips, contact sheets.

The cameras (poses in :data:`dishsim.config.CAMERAS`) are FIXED across all phases so shots stay
comparable. :class:`CameraRig` is Kit-only (imports ``isaaclab.sensors`` lazily); requires the
app to run with ``--headless --enable_cameras`` — without the flag camera sensors return
nothing. :func:`contact_sheet` and :class:`VideoWriter` are Kit-free (PIL/imageio/numpy).
"""

import os

import numpy as np


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

        # COPY, never alias. Callers pass config.CAMERAS itself, and set_view writes into
        # _specs — so aliasing let a temporary close-up permanently overwrite the module-level
        # pose. run_trials.py re-aims "iso" for a final still and then "restores" it from
        # config.CAMERAS["iso"], which by then held the close-up: the restore was a no-op from
        # the second trial onward, and every later recording stored the wrong camera.
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

    def set_view(self, name: str, eye, target, device: str) -> None:
        """Re-aim one camera at runtime (e.g. a close-up framed on a measured pose)."""
        import torch  # noqa: PLC0415

        self._specs[name] = (tuple(eye), tuple(target))
        self.cams[name].set_world_poses_from_view(
            eyes=torch.tensor([tuple(eye)], dtype=torch.float32, device=device),
            targets=torch.tensor([tuple(target)], dtype=torch.float32, device=device),
        )

    def restore_view(self, name: str, device: str) -> None:
        """Re-aim a camera back to its configured pose after a temporary close-up."""
        from . import config  # noqa: PLC0415

        eye, target = config.CAMERAS[name]
        self.set_view(name, eye, target, device)

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


def write_orbit(rig: CameraRig, scene, sim, dt: float, path: str, *, center,
                radius: float = 1.25, height: float = 0.55, n_frames: int = 240,
                camera: str = "iso", fps: int | None = None) -> str:
    """A 360° orbit clip around ``center`` [m, world], stepping physics per frame.

    The shared final-evidence shot (capacity_fill's loaded-machine orbit, the full-load
    episode's closing orbit). Steals the named camera for the sweep and restores its
    configured pose after.

    Returns:
        The clip path.
    """
    from . import config  # noqa: PLC0415

    writer = VideoWriter(path, fps=fps or config.CAMERA_FPS)
    c = np.asarray(center, dtype=float)
    for k in range(n_frames):
        a = 2.0 * np.pi * k / n_frames
        eye = c + np.array([radius * np.cos(a), radius * np.sin(a), height])
        rig.set_view(camera, eye, c, sim.device)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        rig.update(dt)
        writer.add(rig.grab_one(camera))
    writer.close()
    rig.restore_view(camera, sim.device)
    return path


def contact_sheet(images: list[np.ndarray], labels: list[str], out_png: str, cols: int = 4) -> str:
    """Tile images into a labeled grid PNG (labels drawn in a strip above each tile)."""
    from PIL import Image, ImageDraw  # noqa: PLC0415

    assert len(images) == len(labels)
    if not images:
        raise ValueError("no images for contact sheet")
    h, w = images[0].shape[:2]
    strip = 28
    cols = max(1, min(cols, len(images)))
    rows = (len(images) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * w, rows * (h + strip)), (20, 20, 20))
    draw = ImageDraw.Draw(sheet)
    for i, (img, label) in enumerate(zip(images, labels)):
        r, c = divmod(i, cols)
        x, y = c * w, r * (h + strip)
        draw.text((x + 8, y + 6), label, fill=(240, 240, 240))
        sheet.paste(Image.fromarray(img), (x, y + strip))
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    sheet.save(out_png)
    return out_png
