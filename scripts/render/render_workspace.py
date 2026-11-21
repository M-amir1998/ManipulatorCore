"""Sampled workspaces coloured by Yoshikawa manipulability (planar 2R and 7-DOF arm)."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import load_demo
from manipulatorcore_viz.draw3d import draw_robot_3d, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "workspace")
    fig = plt.figure(figsize=(11, 4.8), dpi=110)
    caption(
        fig,
        "Workspace and manipulability",
        "deterministic joint-space sampling mapped through forward "
        "kinematics · colour = Yoshikawa index of the translational Jacobian",
    )
    p2 = np.asarray(d["planar2r"]["samples"])
    ax = fig.add_axes([0.04, 0.08, 0.40, 0.78])
    sc = ax.scatter(p2[:, 0], p2[:, 1], c=p2[:, 2], s=1.5, cmap="viridis", linewidths=0)
    ax.set_aspect("equal")
    ax.set_title("Planar2R: w = l1 l2 |sin q2| (zero on both boundaries)", fontsize=9)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02, label="manipulability")
    p7 = np.asarray(d["redundant7r"]["samples"])
    ax3 = fig.add_axes([0.48, 0.0, 0.48, 0.9], projection="3d")
    setup_axes_3d(ax3, (0.0, 0.0, 0.5), 1.25, elev=18, azim=-60, floor=False)
    keep = p7[:, 1] < 0.0  # cut away half of the cloud to expose the interior
    ax3.scatter(
        p7[keep, 0], p7[keep, 1], p7[keep, 2], c=p7[keep, 3], s=1.0, cmap="viridis", alpha=0.6, depthshade=False
    )
    draw_robot_3d(ax3, d["redundant7r"]["home"])
    ax3.set_title("Redundant7R (half section, y < 0)", fontsize=9, color=PALETTE["text"])
    path = args.out / "workspace.png"
    fig.savefig(path)
    plt.close(fig)
    report(path)


if __name__ == "__main__":
    main()
