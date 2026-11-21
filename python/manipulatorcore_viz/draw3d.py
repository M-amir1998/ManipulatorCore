"""Drawing helpers for spatial arms with matplotlib's mplot3d (CPU rendering, no OpenGL)."""

from __future__ import annotations

from typing import Any

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from manipulatorcore_viz.data import joint_points, link_segments, pose_matrix
from manipulatorcore_viz.style import PALETTE

_BOX_FACES = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]


def setup_axes_3d(
    ax,
    center: tuple[float, float, float],
    half_size: float,
    elev: float = 22.0,
    azim: float = -58.0,
    floor: bool = True,
) -> None:
    """Equal-aspect, uncluttered 3-D axes with an optional light floor grid at z = 0."""
    cx, cy, cz = center
    ax.set_xlim(cx - half_size, cx + half_size)
    ax.set_ylim(cy - half_size, cy + half_size)
    ax.set_zlim(cz - half_size, cz + half_size)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_proj_type("persp", focal_length=0.9)
    if floor:
        ticks = np.linspace(-half_size, half_size, 9)
        for t in ticks:
            ax.plot([cx + t, cx + t], [cy - half_size, cy + half_size], [0, 0], color=PALETTE["grid"], lw=0.6, zorder=0)
            ax.plot([cx - half_size, cx + half_size], [cy + t, cy + t], [0, 0], color=PALETTE["grid"], lw=0.6, zorder=0)


def points_per_meter_3d(ax) -> float:
    """Approximate points per metre at the scene centre (for capsule line widths)."""
    fig = ax.figure
    bbox = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    x0, x1 = ax.get_xlim3d()
    # mplot3d shrinks the cube to fit the view; 0.5 is a stable empirical factor for these layouts.
    return 0.5 * bbox.width * 72.0 / abs(x1 - x0)


def draw_robot_3d(
    ax,
    frame: dict[str, Any],
    color: str | None = None,
    alpha: float = 1.0,
    joints: bool = True,
    highlight: dict[int, str] | None = None,
) -> None:
    """Link capsules as round-capped thick lines (true radius), plus joint markers."""
    color = color or PALETTE["link"]
    scale = points_per_meter_3d(ax)
    highlight = highlight or {}
    for seg in link_segments(frame):
        body = int(seg[7])
        c = highlight.get(body, PALETTE["base"] if body < 0 else (color if body % 2 == 0 else PALETTE["link_alt"]))
        if alpha < 1.0 and body not in highlight:
            c = color
        ax.plot(
            [seg[0], seg[3]],
            [seg[1], seg[4]],
            [seg[2], seg[5]],
            color=c,
            lw=max(1.0, 2.0 * seg[6] * scale),
            alpha=alpha,
            solid_capstyle="round",
        )
    if joints:
        pts = joint_points(frame)[1:-1]
        ax.scatter(
            pts[:, 0],
            pts[:, 1],
            pts[:, 2],
            s=10,
            color=PALETTE["joint"],
            edgecolor=color,
            lw=0.8,
            alpha=alpha,
            depthshade=False,
        )


def draw_triad(ax, flat_pose: list[float], length: float = 0.1, alpha: float = 1.0, lw: float = 1.8) -> None:
    """Coordinate frame (x red, y green, z blue)."""
    T = pose_matrix(flat_pose)
    o = T[:3, 3]
    for axis, color in ((0, PALETTE["axis_x"]), (1, PALETTE["axis_y"]), (2, PALETTE["axis_z"])):
        tip = o + length * T[:3, axis]
        ax.plot([o[0], tip[0]], [o[1], tip[1]], [o[2], tip[2]], color=color, lw=lw, alpha=alpha)


def _box_vertices(T: np.ndarray, half: np.ndarray) -> np.ndarray:
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float) * half
    return corners @ T[:3, :3].T + T[:3, 3]


def draw_obstacles_3d(
    ax, obstacles: list[dict[str, Any]], alpha: float = 0.18, highlight: set[str] | None = None
) -> None:
    """Boxes as translucent faces, spheres as low-resolution surfaces; half-spaces are omitted
    (the floor grid stands in for the ground plane)."""
    highlight = highlight or set()
    for obstacle in obstacles:
        face = PALETTE["collision"] if obstacle.get("name") in highlight else PALETTE["obstacle"]
        T = pose_matrix(obstacle["pose"])
        if obstacle["type"] == "box":
            v = _box_vertices(T, np.asarray(obstacle["half_extents"], dtype=float))
            faces = [[v[i] for i in f] for f in _BOX_FACES]
            ax.add_collection3d(
                Poly3DCollection(faces, facecolor=face, edgecolor=PALETTE["obstacle_edge"], linewidths=0.5, alpha=alpha)
            )
        elif obstacle["type"] == "sphere":
            u, v = np.mgrid[0 : 2 * np.pi : 16j, 0 : np.pi : 10j]
            r = obstacle["radius"]
            c = T[:3, 3]
            ax.plot_surface(
                c[0] + r * np.cos(u) * np.sin(v),
                c[1] + r * np.sin(u) * np.sin(v),
                c[2] + r * np.cos(v),
                color=face,
                alpha=alpha + 0.15,
                linewidth=0,
                shade=True,
            )
        elif obstacle["type"] == "capsule":
            a = np.asarray(obstacle["a"])
            b = np.asarray(obstacle["b"])
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=face, lw=6, alpha=alpha + 0.2)


def draw_path_3d(ax, points: np.ndarray, color: str, lw: float = 1.6, ls: str = "-", alpha: float = 1.0) -> None:
    points = np.asarray(points, dtype=float)
    if len(points) > 1:
        ax.plot(points[:, 0], points[:, 1], points[:, 2], color=color, lw=lw, ls=ls, alpha=alpha)


def scene_bounds(frames: list[dict[str, Any]], margin: float = 0.15) -> tuple[tuple[float, float, float], float]:
    """Cube enclosing all joint positions of the given frames (centre, half size)."""
    pts = np.vstack([joint_points(f) for f in frames])
    lo = pts.min(axis=0) - margin
    hi = pts.max(axis=0) + margin
    lo[2] = min(lo[2], 0.0)
    center = 0.5 * (lo + hi)
    half = 0.5 * float(np.max(hi - lo))
    return (float(center[0]), float(center[1]), float(center[2])), half
