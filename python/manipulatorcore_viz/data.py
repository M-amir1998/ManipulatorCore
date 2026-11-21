"""Loading of the JSON documents written by ``mc_demos``."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def load_demo(directory: str | Path, name: str) -> dict[str, Any]:
    """Loads ``<directory>/<name>.json``."""
    path = Path(directory) / f"{name}.json"
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def pose_matrix(flat: list[float]) -> np.ndarray:
    """4x4 homogeneous matrix from the row-major list used in the JSON files."""
    matrix = np.asarray(flat, dtype=float).reshape(4, 4)
    return matrix


def joint_points(frame: dict[str, Any]) -> np.ndarray:
    """Base origin, joint origins and end-effector position of a robot frame (N x 3)."""
    return np.asarray(frame["joints"], dtype=float)


def link_segments(frame: dict[str, Any]) -> np.ndarray:
    """Link capsules of a robot frame as rows [ax, ay, az, bx, by, bz, radius, body]."""
    links = frame.get("links", [])
    return np.asarray(links, dtype=float).reshape(-1, 8)


def ee_position(frame: dict[str, Any]) -> np.ndarray:
    return pose_matrix(frame["ee"])[:3, 3]


def subsample(items: list[Any], count: int) -> list[Any]:
    """At most ``count`` evenly spaced items, always keeping the first and the last."""
    if len(items) <= count:
        return list(items)
    indices = np.linspace(0, len(items) - 1, count).round().astype(int)
    return [items[i] for i in indices]
