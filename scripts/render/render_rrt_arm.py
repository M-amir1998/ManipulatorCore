"""Sampling-based planning in the exact 2-D configuration space of a planar arm.

Left: workspace with obstacles. Right: configuration space (grey = collision, computed by the C++
collision checker on a grid), RRT-Connect trees growing in insertion order, the found path, the
shortcut path and the executed, time-parameterized motion. Also writes a static comparison of the
four planners' trees (planners_2r.png).
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position
from manipulatorcore_viz.draw2d import draw_obstacles_2d, draw_planar_arm, setup_planar_axes
from manipulatorcore_viz.style import PALETTE, caption

TREE_COLORS = [PALETTE["plan"], PALETTE["series"][3]]


def cspace_axes(ax, cspace) -> None:
    lo, hi = cspace["lower"], cspace["upper"]
    grid = np.asarray(cspace["occupied"], dtype=float)
    cmap = plt.matplotlib.colors.ListedColormap([PALETTE["background"], PALETTE["obstacle"]])
    ax.imshow(grid, origin="lower", extent=[lo[0], hi[0], lo[1], hi[1]], cmap=cmap, interpolation="nearest", zorder=0)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.set_xlabel("q1 [rad]")
    ax.set_ylabel("q2 [rad]")
    ax.grid(False)


def tree_segments(tree, count: int | None = None):
    nodes = np.asarray(tree["nodes"], dtype=float).reshape(-1, 4)
    if count is not None:
        nodes = nodes[:count]
    segs, colors = [], []
    for q1, q2, parent, index in nodes:
        if parent >= 0 and parent < len(nodes):
            p = nodes[int(parent)]
            segs.append([(p[0], p[1]), (q1, q2)])
            colors.append(TREE_COLORS[int(index) % 2])
    for a, b in tree.get("edges", []):
        if count is None or (a < len(nodes) and b < len(nodes)):
            segs.append([tuple(nodes[a][:2]), tuple(nodes[b][:2])])
            colors.append(TREE_COLORS[0])
    return segs, colors


def draw_path(ax, path, color, lw, ls="-"):
    path = np.asarray(path, dtype=float)
    if len(path) > 1:
        ax.plot(path[:, 0], path[:, 1], color=color, lw=lw, ls=ls, zorder=5)


def animation(d, args) -> None:
    run = d["runs"]["RRT-Connect"]
    nodes = len(run["tree"]["nodes"])
    reach = 1.95
    rec = FrameRecorder()

    def frame(count, show_path=0, exec_index=None):
        fig = plt.figure(figsize=(9.6, 4.8), dpi=100)
        caption(
            fig,
            "Motion planning in configuration space",
            "RRT-Connect on a planar 2R arm · exact C-space obstacles from the collision checker · "
            "shortcutting · time-optimal execution",
        )
        ax_w = fig.add_axes([0.0, 0.02, 0.46, 0.84])
        ax_c = fig.add_axes([0.56, 0.12, 0.40, 0.74])
        setup_planar_axes(ax_w, reach)
        draw_obstacles_2d(ax_w, d["obstacles"])
        cspace_axes(ax_c, d["cspace"])
        segs, colors = tree_segments(run["tree"], count)
        ax_c.add_collection(LineCollection(segs, colors=colors, linewidths=0.8, zorder=2))
        draw_planar_arm(ax_w, d["start"], color=PALETTE["ghost"], alpha=0.45, joints=False)
        draw_planar_arm(ax_w, d["goal"], color=PALETTE["target"], alpha=0.45, joints=False)
        for q, color in ((d["start"]["q"], PALETTE["ghost"]), (d["goal"]["q"], PALETTE["target"])):
            ax_c.scatter(*q, s=50, color=color, edgecolor=PALETTE["text"], zorder=6)
        if show_path >= 1:
            draw_path(ax_c, run["path"], PALETTE["link"], 1.4, ls="--")
        if show_path >= 2:
            draw_path(ax_c, run["shortcut"], PALETTE["trace"], 2.4)
        if exec_index is not None:
            ex = run["execution"]
            f = ex[exec_index]
            trace = np.asarray([ee_position(g)[:2] for g in ex[: exec_index + 1]])
            ax_w.plot(trace[:, 0], trace[:, 1], color=PALETTE["trace"], lw=1.8, zorder=4)
            draw_planar_arm(ax_w, f)
            ax_c.scatter(*f["q"], s=40, color=PALETTE["trace"], zorder=7)
        else:
            draw_planar_arm(ax_w, d["start"])
        label = "exploring" if show_path == 0 else ("path found" if show_path == 1 else "shortcut + execution")
        ax_c.set_title(f"{label} · {min(count, nodes)} nodes", fontsize=9, color=PALETTE["muted"], fontweight="normal")
        return fig

    for count in np.unique(np.linspace(2, nodes, 30).round().astype(int)):
        fig = frame(int(count))
        rec.capture(fig, 80)
        plt.close(fig)
    for show in (1, 2):
        fig = frame(nodes, show_path=show)
        rec.capture(fig, 1000)
        plt.close(fig)
    ex = run["execution"]
    for i in range(len(ex)):
        fig = frame(nodes, show_path=2, exec_index=i)
        rec.capture(fig, 50)
        plt.close(fig)
    rec.hold(1500)
    report(rec.save(args.out / "rrt_2r.gif", colors=64))


def comparison(d, args) -> None:
    fig, axes = plt.subplots(1, 4, figsize=(12, 3.6), dpi=110)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.80, bottom=0.14, wspace=0.25)
    caption(
        fig,
        "Four planners, one configuration space",
        "planar 2R arm · same start/goal · tree or roadmap (blue), raw path (dashed), shortcut path (orange)",
    )
    for ax, name in zip(axes, ["RRT", "RRT-Connect", "RRT*", "PRM"], strict=True):
        run = d["runs"][name]
        cspace_axes(ax, d["cspace"])
        segs, colors = tree_segments(run["tree"])
        ax.add_collection(LineCollection(segs, colors=colors, linewidths=0.5, alpha=0.8, zorder=2))
        draw_path(ax, run["path"], PALETTE["link"], 1.2, ls="--")
        draw_path(ax, run["shortcut"], PALETTE["trace"], 2.0)
        ax.set_title(
            f"{name}\n{run['nodes']} nodes · {run['path_length']:.2f} -> {run['shortcut_length']:.2f} rad", fontsize=8.5
        )
    path = args.out / "planners_2r.png"
    fig.savefig(path)
    plt.close(fig)
    report(path)


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "rrt_2r")
    comparison(d, args)
    animation(d, args)


if __name__ == "__main__":
    main()
