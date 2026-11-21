"""Drawing helpers for planar arms (x-y plane)."""

from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib.patches import Circle, Ellipse, Rectangle

from manipulatorcore_viz.data import joint_points, link_segments, pose_matrix
from manipulatorcore_viz.style import PALETTE


def points_per_meter(ax) -> float:
    """Conversion from data units to typographic points along x for line widths."""
    fig = ax.figure
    bbox = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    x0, x1 = ax.get_xlim()
    return bbox.width * 72.0 / abs(x1 - x0)


def draw_planar_arm(
    ax, frame: dict[str, Any], color: str | None = None, alpha: float = 1.0, zorder: int = 5, joints: bool = True
) -> None:
    """Draws link capsules with their true radius, joint markers and the base."""
    color = color or PALETTE["link"]
    scale = points_per_meter(ax)
    for seg in link_segments(frame):
        ax.plot(
            [seg[0], seg[3]],
            [seg[1], seg[4]],
            color=color,
            lw=max(1.0, 2.0 * seg[6] * scale),
            alpha=alpha,
            solid_capstyle="round",
            zorder=zorder,
        )
    pts = joint_points(frame)
    if joints:
        ax.scatter(
            pts[:-1, 0],
            pts[:-1, 1],
            s=18,
            facecolor=PALETTE["joint"],
            edgecolor=color,
            linewidths=1.2,
            alpha=alpha,
            zorder=zorder + 1,
        )
    ax.add_patch(
        Rectangle(
            (-0.12, -0.08), 0.24, 0.08, facecolor=PALETTE["base"], edgecolor="none", alpha=alpha, zorder=zorder - 1
        )
    )


def draw_frame_2d(ax, flat_pose: list[float], length: float = 0.15, alpha: float = 1.0, zorder: int = 8) -> None:
    """End-effector (or target) frame: x and y axes in the plane."""
    T = pose_matrix(flat_pose)
    origin = T[:2, 3]
    for axis, color in ((0, PALETTE["axis_x"]), (1, PALETTE["axis_y"])):
        tip = origin + length * T[:2, axis]
        ax.plot([origin[0], tip[0]], [origin[1], tip[1]], color=color, lw=1.6, alpha=alpha, zorder=zorder)


def draw_obstacles_2d(ax, obstacles: list[dict[str, Any]], zorder: int = 2) -> None:
    """Spheres become discs; boxes become rectangles (projected onto the x-y plane)."""
    for obstacle in obstacles:
        T = pose_matrix(obstacle["pose"])
        if obstacle["type"] == "sphere":
            ax.add_patch(
                Circle(
                    T[:2, 3],
                    obstacle["radius"],
                    facecolor=PALETTE["obstacle"],
                    edgecolor=PALETTE["obstacle_edge"],
                    lw=1.0,
                    zorder=zorder,
                )
            )
        elif obstacle["type"] == "box":
            half = np.asarray(obstacle["half_extents"])
            corner = T[:2, 3] - half[:2]
            ax.add_patch(
                Rectangle(
                    corner,
                    2 * half[0],
                    2 * half[1],
                    facecolor=PALETTE["obstacle"],
                    edgecolor=PALETTE["obstacle_edge"],
                    lw=1.0,
                    zorder=zorder,
                )
            )


def draw_velocity_ellipse(ax, center: np.ndarray, ellipse: list[float], scale: float, zorder: int = 7) -> None:
    """Velocity ellipse (semi-axes from the singular values of the planar Jacobian)."""
    a, b, angle = ellipse
    ax.add_patch(
        Ellipse(
            center,
            2 * a * scale,
            2 * b * scale,
            angle=np.degrees(angle),
            facecolor=PALETTE["plan"],
            alpha=0.15,
            edgecolor=PALETTE["plan"],
            lw=1.0,
            zorder=zorder,
        )
    )


def setup_planar_axes(ax, reach: float) -> None:
    ax.set_aspect("equal")
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_visible(False)
