"""Frame capture and compact GIF / WebP encoding with Pillow."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image


def figure_to_image(fig) -> Image.Image:
    """Rasterizes a matplotlib figure (Agg) into an RGB Pillow image."""
    fig.canvas.draw()
    buffer = np.asarray(fig.canvas.buffer_rgba())
    return Image.fromarray(buffer[:, :, :3].copy())


class FrameRecorder:
    """Collects rasterized frames; ``hold`` repeats the last frame (pauses without re-rendering)."""

    def __init__(self) -> None:
        self.frames: list[Image.Image] = []
        self.durations: list[int] = []

    def capture(self, fig, duration_ms: int = 40) -> None:
        self.frames.append(figure_to_image(fig))
        self.durations.append(duration_ms)

    def hold(self, duration_ms: int) -> None:
        if self.durations:
            self.durations[-1] += duration_ms

    def save(self, path: str | Path, colors: int = 128) -> Path:
        return save_gif(self.frames, self.durations, path, colors=colors)


def _global_palette(frames: list[Image.Image], colors: int) -> Image.Image:
    """One palette for the whole animation (no per-frame flicker): quantize a mosaic of samples."""
    sample = frames[:: max(1, len(frames) // 12)][:12]
    width, height = sample[0].size
    mosaic = Image.new("RGB", (width, height * len(sample)))
    for i, frame in enumerate(sample):
        mosaic.paste(frame, (0, i * height))
    return mosaic.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)


def save_gif(frames: list[Image.Image], durations: list[int], path: str | Path, colors: int = 128) -> Path:
    """Encodes an optimized looping GIF (or animated WebP if the suffix is .webp)."""
    if not frames:
        raise ValueError("no frames to save")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".webp":
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=durations, loop=0, quality=80, method=6)
        return path
    palette = _global_palette(frames, colors)
    quantized = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    quantized[0].save(
        path,
        save_all=True,
        append_images=quantized[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=1,
    )
    return path
