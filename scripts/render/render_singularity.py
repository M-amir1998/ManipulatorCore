"""Resolved-rate motion into a singularity: Moore-Penrose pseudoinverse versus damped least squares."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position
from manipulatorcore_viz.draw2d import draw_planar_arm, draw_velocity_ellipse, setup_planar_axes
from manipulatorcore_viz.style import PALETTE, caption


def series(frames, key):
    return np.asarray([f[key] for f in frames], dtype=float)


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "singularity")
    pinv = d["runs"]["pinv"]["frames"]
    dls = d["runs"]["dls"]["frames"]
    reach = sum(d["model"]["lengths"])
    colors = {"pinv": PALETTE["trace"], "dls": PALETTE["plan"]}
    t_p, t_d = series(pinv, "t"), series(dls, "t")
    t_end = 1.6  # after this the damped solution rests at the boundary; nothing new happens
    n = int(np.searchsorted(t_d, t_end))
    rec = FrameRecorder()
    for k in [*range(0, n, 2), n - 1]:
        fig = plt.figure(figsize=(9.6, 4.8), dpi=100)
        caption(
            fig,
            "Singularity robustness",
            "constant Cartesian velocity towards the workspace boundary · "
            "pseudoinverse vs damped least squares (lambda = 0.08)",
        )
        ax = fig.add_axes([0.0, 0.0, 0.46, 0.88])
        setup_planar_axes(ax, reach + 0.1)
        ax.set_xlim(-0.3, reach + 0.15)
        ax.set_ylim(-1.25, 0.85)
        ax.add_patch(plt.Circle((0, 0), reach, fill=False, ls=":", color=PALETTE["muted"], lw=1))
        kp = min(k, len(pinv) - 1)
        for name, frames, idx in (("pinv", pinv, kp), ("dls", dls, k)):
            trace = np.asarray([ee_position(f)[:2] for f in frames[: idx + 1]])
            ax.plot(trace[:, 0], trace[:, 1], color=colors[name], lw=1.6)
        draw_planar_arm(ax, pinv[kp], color=colors["pinv"], alpha=0.55)
        draw_velocity_ellipse(ax, ee_position(dls[k])[:2], dls[k]["ellipse"], scale=0.35)
        draw_planar_arm(ax, dls[k])
        ax.text(
            -0.25,
            -1.18,
            "outer workspace boundary (dotted): the arm is fully stretched there",
            fontsize=8,
            color=PALETTE["muted"],
        )

        ax1 = fig.add_axes([0.55, 0.53, 0.42, 0.32])
        ax2 = fig.add_axes([0.55, 0.10, 0.42, 0.32])
        ax1.semilogy(
            t_p[: kp + 1], series(pinv, "qd_norm")[: kp + 1], color=colors["pinv"], lw=1.6, label="pseudoinverse"
        )
        ax1.semilogy(t_d[: k + 1], series(dls, "qd_norm")[: k + 1], color=colors["dls"], lw=1.6, label="DLS")
        ax1.set_xlim(0, t_end)
        ax1.set_ylim(0.1, 100)
        ax1.set_ylabel("|qd| [rad/s]")
        ax1.legend(loc="upper left")
        if d["runs"]["pinv"]["diverged"] and k >= len(pinv) - 1:
            ax1.annotate(
                "joint speed explodes",
                (t_p[-1], series(pinv, "qd_norm")[-1]),
                xytext=(-110, -14),
                textcoords="offset points",
                fontsize=8,
                color=colors["pinv"],
            )
        ax2.semilogy(t_d[: k + 1], series(dls, "sigma_min")[: k + 1], color=PALETTE["text"], lw=1.4, label="sigma_min")
        ax2.semilogy(
            t_d[: k + 1],
            series(dls, "velocity_error")[: k + 1] + 1e-6,
            color=colors["dls"],
            lw=1.2,
            ls="--",
            label="DLS task-velocity error [m/s]",
        )
        ax2.set_xlim(0, t_end)
        ax2.set_ylim(1e-4, 2)
        ax2.set_xlabel("time [s]")
        ax2.legend(loc="lower left")
        rec.capture(fig, 50)
        plt.close(fig)
    rec.hold(1800)
    report(rec.save(args.out / "singularity.gif", colors=64))


if __name__ == "__main__":
    main()
