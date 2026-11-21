"""PD vs PD + gravity compensation vs computed torque on the same joint trajectory (simulated).

Each panel shows the reference configuration (ghost) and the simulated arm; the bottom plot shows the
end-effector position error of the three controllers."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption

COLORS = [PALETTE["trace"], PALETTE["series"][3], PALETTE["plan"]]


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "computed_torque")
    runs = d["runs"]
    n = len(runs[0]["frames"])
    t = np.asarray([f["t"] for f in runs[0]["frames"]])
    errors = [np.asarray([f["position_error"] for f in r["frames"]]) * 1e3 for r in runs]
    traces = [np.asarray([ee_position(f) for f in r["frames"]]) for r in runs]
    rec = FrameRecorder()
    center = (0.25, 0.1, 0.55)
    half = 0.65
    for k in [int(i) for i in np.linspace(0, n - 1, 70).round()]:
        fig = plt.figure(figsize=(9.6, 5.4), dpi=100)
        caption(
            fig,
            "Joint-space tracking control",
            "Redundant7R in simulation · same quintic reference · ghost = reference, solid = simulated arm",
        )
        for i, run in enumerate(runs):
            ax = fig.add_axes([0.005 + i * 0.33, 0.33, 0.33, 0.58], projection="3d")
            setup_axes_3d(ax, center, half, elev=18, azim=-60)
            draw_obstacles_3d(ax, d["base"])
            f = run["frames"][k]
            draw_robot_3d(ax, f["reference"], color=PALETTE["ghost"], alpha=0.3, joints=False)
            draw_path_3d(ax, traces[i][: k + 1], COLORS[i], lw=2.0)
            draw_robot_3d(ax, f)
            m = run["metrics"]
            ax.set_title(
                f"{run['controller']}\nEE RMSE {1e3 * m['position_rmse']:.1f} mm", fontsize=9.5, color=COLORS[i], y=0.93
            )
        axe = fig.add_axes([0.08, 0.08, 0.88, 0.2])
        for i, run in enumerate(runs):
            axe.plot(t[: k + 1], errors[i][: k + 1], color=COLORS[i], lw=1.6, label=run["controller"])
        axe.set_xlim(0, t[-1])
        axe.set_ylim(0, max(e.max() for e in errors) * 1.1)
        axe.set_ylabel("EE error [mm]")
        axe.set_xlabel("time [s]")
        axe.legend(ncol=3, loc="upper right")
        rec.capture(fig, 55)
        plt.close(fig)
    rec.hold(1500)
    report(rec.save(args.out / "computed_torque.gif", colors=96))


if __name__ == "__main__":
    main()
