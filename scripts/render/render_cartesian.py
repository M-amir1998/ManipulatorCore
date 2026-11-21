"""6R end-effector following a straight line, a horizontal circle and a vertical circle with
geodesic (SLERP-equivalent) orientation; each sample is solved by warm-started DLS IK."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position, pose_matrix, subsample
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, draw_triad, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption

LABELS = {"line": "straight line", "horizontal_circle": "horizontal circle", "vertical_circle": "vertical circle"}


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "cartesian_paths")
    rec = FrameRecorder()
    center = (0.55, 0.15, 0.85)
    half = 0.72
    for path in d["paths"]:
        frames = path["frames"]
        targets = np.asarray([pose_matrix(f["target"])[:3, 3] for f in frames])
        trace = np.asarray([ee_position(f) for f in frames])
        q = np.asarray([f["q"] for f in frames])
        t = np.asarray([f["t"] for f in frames])
        for f in subsample(frames, 42):
            k = int(np.searchsorted(t, f["t"], side="right"))
            fig = plt.figure(figsize=(9.6, 5.0), dpi=100)
            caption(
                fig,
                "Cartesian trajectories",
                f"Industrial6R · {LABELS[path['name']]} · quintic time scaling · "
                "orientation by SO(3) geodesic · DLS IK per sample",
            )
            ax = fig.add_axes([0.0, 0.0, 0.62, 0.9], projection="3d")
            setup_axes_3d(ax, center, half, elev=20, azim=-40)
            draw_obstacles_3d(ax, d["base"])
            draw_path_3d(ax, targets, PALETTE["plan"], lw=1.2, ls="--", alpha=0.8)
            draw_path_3d(ax, trace[:k], PALETTE["trace"], lw=2.4)
            draw_robot_3d(ax, f)
            draw_triad(ax, f["ee"], length=0.1)
            axq = fig.add_axes([0.66, 0.16, 0.31, 0.62])
            for j in range(q.shape[1]):
                axq.plot(
                    t[:k], q[:k, j], color=PALETTE["series"][j % len(PALETTE["series"])], lw=1.4, label=f"q{j + 1}"
                )
            axq.set_xlim(0, t[-1])
            axq.set_ylim(q.min() - 0.2, q.max() + 0.2)
            axq.set_xlabel("time [s]")
            axq.set_ylabel("joint angle [rad]")
            axq.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.22))
            rec.capture(fig, 60)
            plt.close(fig)
        rec.hold(700)
    report(rec.save(args.out / "cartesian_paths.gif", colors=96))


if __name__ == "__main__":
    main()
