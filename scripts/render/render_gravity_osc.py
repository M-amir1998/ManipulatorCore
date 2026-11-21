"""Gravity compensation (PD sags, PD + g(q) holds) and operational-space circle tracking, simulated."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position, subsample
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption


def gravity(args) -> None:
    d = load_demo(args.data, "gravity_hold")
    runs = d["runs"]
    rec = FrameRecorder()
    n = len(runs[0]["frames"])
    for k in [int(i) for i in np.linspace(0, n - 1, 60).round()]:
        fig = plt.figure(figsize=(9.6, 4.6), dpi=100)
        caption(fig, "Gravity compensation", "Redundant7R in simulation · same PD gains · target pose shown as ghost")
        for i, run in enumerate(runs):
            ax = fig.add_axes([i * 0.5, 0.0, 0.5, 0.88], projection="3d")
            setup_axes_3d(ax, (0.25, 0.0, 0.55), 0.6, elev=12, azim=-90)
            draw_obstacles_3d(ax, d["base"])
            draw_robot_3d(ax, d["target"], color=PALETTE["ghost"], alpha=0.3, joints=False)
            f = run["frames"][k]
            draw_robot_3d(ax, f)
            sag = 1e3 * (ee_position(d["target"]) - ee_position(f))[2]
            ax.set_title(f"{run['controller']}: end-effector {sag:.0f} mm below target", fontsize=9.5, y=0.95)
        rec.capture(fig, 60)
        plt.close(fig)
    rec.hold(1200)
    report(rec.save(args.out / "gravity_hold.gif", colors=64))


def osc(args) -> None:
    d = load_demo(args.data, "operational_space")
    frames = d["frames"]
    t = np.asarray([f["t"] for f in frames])
    desired = np.asarray([f["desired"] for f in frames])
    actual = np.asarray([ee_position(f) for f in frames])
    err = np.linalg.norm(desired - actual, axis=1) * 1e3
    rec = FrameRecorder()
    for f in subsample(frames, 80):
        k = int(np.searchsorted(t, f["t"], side="right"))
        fig = plt.figure(figsize=(9.6, 4.8), dpi=100)
        caption(
            fig,
            "Operational-space control",
            "Redundant7R · Khatib OSC with dynamically consistent null-space posture task · 0.15 m circle in 4 s",
        )
        ax = fig.add_axes([0.0, 0.0, 0.58, 0.9], projection="3d")
        setup_axes_3d(ax, (0.3, 0.1, 0.55), 0.6, elev=28, azim=-50)
        draw_obstacles_3d(ax, d["base"])
        draw_path_3d(ax, desired, PALETTE["plan"], lw=1.2, ls="--")
        draw_path_3d(ax, actual[:k], PALETTE["trace"], lw=2.2)
        draw_robot_3d(ax, f)
        ax2 = fig.add_axes([0.64, 0.16, 0.33, 0.6])
        ax2.plot(t[:k], err[:k], color=PALETTE["trace"], lw=1.6)
        ax2.set_xlim(0, t[-1])
        ax2.set_ylim(0, err.max() * 1.2)
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("tracking error [mm]")
        rec.capture(fig, 55)
        plt.close(fig)
    report(rec.save(args.out / "operational_space.gif", colors=96))


def main() -> None:
    args = parse_args(__doc__)
    gravity(args)
    osc(args)


if __name__ == "__main__":
    main()
