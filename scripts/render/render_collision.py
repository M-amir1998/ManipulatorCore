"""Per-link clearance while the 6R arm sweeps past obstacles: links coloured by distance (green safe,
amber within 10 cm, red within 2 cm or penetrating) and the closest-point segments of near pairs."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_robot_3d, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption


def color_for(distance: float) -> str:
    if distance is None or distance > 0.10:
        return PALETTE["safe"]
    if distance > 0.02:
        return PALETTE["warning"]
    return PALETTE["collision"]


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "collision")
    frames = d["frames"]
    t = np.asarray([f["t"] for f in frames])
    clearance = np.asarray([[c if c is not None else np.nan for c in f["clearance"]] for f in frames])
    rec = FrameRecorder()
    for k in range(0, len(frames), 2):
        f = frames[k]
        fig = plt.figure(figsize=(9.6, 5.0), dpi=100)
        caption(
            fig,
            "Collision checking and clearance",
            "Industrial6R · capsule links vs box, sphere and floor · "
            "signed distances, closest points, AABB broad phase",
        )
        ax = fig.add_axes([0.0, 0.0, 0.58, 0.9], projection="3d")
        setup_axes_3d(ax, (0.35, 0.1, 0.6), 0.7, elev=22, azim=-55)
        draw_obstacles_3d(ax, d["obstacles"], alpha=0.25)
        highlight = {i: color_for(c) for i, c in enumerate(f["clearance"])}
        draw_robot_3d(ax, f, highlight=highlight)
        for pair in f["pairs"]:
            if pair["kind"] == "env" and pair["distance"] < 0.15:
                a, b = np.asarray(pair["a"]), np.asarray(pair["b"])
                ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=color_for(pair["distance"]), lw=1.2, ls=":")
        ax2 = fig.add_axes([0.64, 0.14, 0.33, 0.68])
        for j in range(clearance.shape[1]):
            ax2.plot(t[: k + 1], clearance[: k + 1, j], lw=1.3, color=PALETTE["series"][j % 7], label=f"link {j + 1}")
        ax2.axhspan(-0.05, 0.02, color=PALETTE["collision"], alpha=0.08)
        ax2.axhspan(0.02, 0.10, color=PALETTE["warning"], alpha=0.08)
        ax2.set_xlim(0, t[-1])
        ax2.set_ylim(-0.05, 0.6)
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("minimum clearance [m]")
        ax2.legend(ncol=2, loc="upper right")
        rec.capture(fig, 60)
        plt.close(fig)
    report(rec.save(args.out / "collision.gif", colors=96))


if __name__ == "__main__":
    main()
