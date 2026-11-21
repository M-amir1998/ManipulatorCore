"""Interaction control against a compliant wall (impedance and admittance), simulated.

The desired end-effector motion continues 6 cm behind the wall; the controller's stiffness and the
wall's stiffness act in series, so the contact force settles at a predictable value."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position, pose_matrix, subsample
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption


def render(args, name: str, title: str) -> None:
    d = load_demo(args.data, name)
    frames = d["frames"]
    t = np.asarray([f["t"] for f in frames])
    x = np.asarray([ee_position(f)[0] for f in frames])
    xd = np.asarray([pose_matrix(f["desired"])[0, 3] for f in frames])
    force = np.asarray([np.linalg.norm(f["force"]) for f in frames])
    trace = np.asarray([ee_position(f) for f in frames])
    wall_x = d["wall_x"]
    rec = FrameRecorder()
    center = (0.35, 0.0, 0.55)
    half = 0.6
    y0, z0 = trace[0, 1], trace[0, 2]
    for f in subsample(frames, 85):
        k = int(np.searchsorted(t, f["t"], side="right"))
        fig = plt.figure(figsize=(9.6, 5.0), dpi=100)
        caption(
            fig,
            title,
            f"Redundant7R in simulation · desired motion ends 6 cm behind a compliant wall · "
            f"K = {d['stiffness']:.0f} N/m",
        )
        ax = fig.add_axes([0.0, 0.0, 0.55, 0.9], projection="3d")
        setup_axes_3d(ax, center, half, elev=16, azim=-75)
        draw_obstacles_3d(ax, d["base"])
        wall = [
            [
                (wall_x, y0 - 0.3, z0 - 0.35),
                (wall_x, y0 + 0.3, z0 - 0.35),
                (wall_x, y0 + 0.3, z0 + 0.35),
                (wall_x, y0 - 0.3, z0 + 0.35),
            ]
        ]
        ax.add_collection3d(
            Poly3DCollection(wall, facecolor=PALETTE["obstacle"], edgecolor=PALETTE["obstacle_edge"], alpha=0.35)
        )
        draw_path_3d(ax, trace[:k], PALETTE["trace"], lw=2.0)
        draw_robot_3d(ax, f)
        p = ee_position(f)
        des = pose_matrix(f["desired"])[:3, 3]
        ax.scatter(*des, s=30, color=PALETTE["plan"], depthshade=False)
        fvec = np.asarray(f["force"])
        if np.linalg.norm(fvec) > 0.5:
            tip = p + fvec * 0.004
            ax.plot([p[0], tip[0]], [p[1], tip[1]], [p[2], tip[2]], color=PALETTE["collision"], lw=2.5)

        ax1 = fig.add_axes([0.62, 0.55, 0.35, 0.3])
        ax1.plot(t[:k], 100 * (xd[:k] - wall_x), color=PALETTE["plan"], lw=1.4, ls="--", label="desired")
        ax1.plot(t[:k], 100 * (x[:k] - wall_x), color=PALETTE["trace"], lw=1.6, label="end-effector")
        ax1.axhline(-100 * d["contact_radius"], color=PALETTE["obstacle_edge"], lw=1, ls=":")
        ax1.set_xlim(0, t[-1])
        ax1.set_ylim(100 * (x.min() - wall_x) - 2, 100 * (xd.max() - wall_x) + 2)
        ax1.set_ylabel("x - wall [cm]")
        ax1.legend(loc="upper left")
        ax2 = fig.add_axes([0.62, 0.12, 0.35, 0.3])
        ax2.plot(t[:k], force[:k], color=PALETTE["collision"], lw=1.6)
        ax2.set_xlim(0, t[-1])
        ax2.set_ylim(0, force.max() * 1.15 + 1)
        ax2.set_ylabel("contact force [N]")
        ax2.set_xlabel("time [s]")
        rec.capture(fig, 45)
        plt.close(fig)
    rec.hold(1500)
    report(rec.save(args.out / f"{name}.gif", colors=96))


def main() -> None:
    args = parse_args(__doc__)
    render(args, "impedance", "Cartesian impedance control")
    render(args, "admittance", "Admittance control")


if __name__ == "__main__":
    main()
