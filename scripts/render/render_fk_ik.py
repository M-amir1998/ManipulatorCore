"""Planar 2R arm: forward kinematics with the velocity ellipse, and both closed-form IK branches."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position
from manipulatorcore_viz.draw2d import draw_frame_2d, draw_planar_arm, draw_velocity_ellipse, setup_planar_axes
from manipulatorcore_viz.style import PALETTE, caption


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "fk_ik_2r")
    reach = sum(d["model"]["lengths"]) + 0.15
    fig, (ax_fk, ax_ik) = plt.subplots(1, 2, figsize=(9.6, 4.6), dpi=100)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.02, wspace=0.04)
    fk = d["fk"]
    ik = d["ik"]
    fk_trace = np.asarray([ee_position(f)[:2] for f in fk])
    targets = np.asarray([step["target"] for step in ik])
    rec = FrameRecorder()
    n = max(len(fk), len(ik))
    for k in range(n):
        fig.clear()
        ax_fk, ax_ik = fig.subplots(1, 2)
        fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.02, wspace=0.04)
        caption(
            fig,
            "Forward and inverse kinematics",
            "planar 2R arm · velocity ellipse from the Jacobian SVD · both closed-form IK branches",
        )
        for ax, title in ((ax_fk, "forward kinematics  q -> T(q)"), (ax_ik, "inverse kinematics  T -> q (2 branches)")):
            setup_planar_axes(ax, reach)
            ax.set_title(title, color=PALETTE["muted"], fontsize=9, fontweight="normal")
            ax.add_patch(
                plt.Circle((0, 0), sum(d["model"]["lengths"]), fill=False, ls=":", color=PALETTE["grid"], lw=1)
            )

        f = fk[k % len(fk)]
        ax_fk.plot(fk_trace[:, 0], fk_trace[:, 1], color=PALETTE["trace"], lw=1.0, alpha=0.35)
        draw_velocity_ellipse(ax_fk, ee_position(f)[:2], f["ellipse"], scale=0.25)
        draw_planar_arm(ax_fk, f)
        draw_frame_2d(ax_fk, f["ee"], length=0.22)
        q = f["q"]
        ax_fk.text(
            -reach + 0.08, -reach + 0.12, f"q = ({q[0]:+.2f}, {q[1]:+.2f}) rad", fontsize=9, color=PALETTE["text"]
        )

        step = ik[k % len(ik)]
        ax_ik.plot(targets[:, 0], targets[:, 1], color=PALETTE["target"], lw=1.0, ls="--", alpha=0.6)
        sols = step["solutions"]
        if len(sols) > 1:
            draw_planar_arm(ax_ik, sols[1], color=PALETTE["ghost"], alpha=0.55)
        if sols:
            draw_planar_arm(ax_ik, sols[0])
        ax_ik.scatter(*step["target"], s=60, marker="x", color=PALETTE["target"], zorder=10, lw=2)
        ax_ik.text(-reach + 0.08, -reach + 0.12, "elbow up (q2 > 0) and elbow down", fontsize=9, color=PALETTE["text"])
        rec.capture(fig, 50)
    report(rec.save(args.out / "fk_ik_2r.gif", colors=64))


if __name__ == "__main__":
    main()
