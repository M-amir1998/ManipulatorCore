"""Redundancy resolution: operational-space control holds the end-effector pose while a null-space
posture task (with smooth joint-limit repulsion) swings the elbow (simulated)."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import joint_points, subsample
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, draw_triad, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "nullspace")
    frames = d["frames"]
    t = np.asarray([f["t"] for f in frames])
    err = np.asarray([f["ee_error"] for f in frames]) * 1e3
    q = np.asarray([f["q"] for f in frames])
    elbow = np.asarray([joint_points(f)[4] for f in frames])
    rec = FrameRecorder()
    center = (0.25, 0.0, 0.6)
    half = 0.6
    for f in subsample(frames, 100):
        k = int(np.searchsorted(t, f["t"], side="right"))
        fig = plt.figure(figsize=(9.6, 5.0), dpi=100)
        caption(
            fig,
            "Null-space redundancy resolution",
            "Redundant7R · operational-space control holds the tool pose · null-space posture task moves the elbow",
        )
        ax = fig.add_axes([0.0, 0.0, 0.55, 0.9], projection="3d")
        setup_axes_3d(ax, center, half, elev=20, azim=-35)
        draw_obstacles_3d(ax, d["base"])
        draw_triad(ax, d["hold"], length=0.14, lw=2.2, alpha=0.5)
        draw_path_3d(ax, elbow[:k], PALETTE["plan"], lw=1.4, alpha=0.8)
        draw_robot_3d(ax, f)
        ax1 = fig.add_axes([0.62, 0.55, 0.35, 0.3])
        ax1.plot(t[:k], q[:k, 2], color=PALETTE["plan"], lw=1.6, label="q3 (upper-arm roll)")
        ax1.plot(t[:k], q[:k, 0], color=PALETTE["series"][3], lw=1.6, label="q1")
        ax1.set_xlim(0, t[-1])
        ax1.set_ylim(q[:, [0, 2]].min() - 0.2, q[:, [0, 2]].max() + 0.2)
        ax1.set_ylabel("joint angle [rad]")
        ax1.legend(loc="upper right")
        ax2 = fig.add_axes([0.62, 0.12, 0.35, 0.3])
        ax2.plot(t[:k], err[:k], color=PALETTE["trace"], lw=1.6)
        ax2.set_xlim(0, t[-1])
        ax2.set_ylim(0, max(0.2, err.max() * 1.2))
        ax2.set_ylabel("tool position error [mm]")
        ax2.set_xlabel("time [s]")
        rec.capture(fig, 60)
        plt.close(fig)
    report(rec.save(args.out / "nullspace.gif", colors=96))


if __name__ == "__main__":
    main()
