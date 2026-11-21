"""Honest failure cases:

* narrow_passage.png   - RRT vs RRT-Connect through a narrow C-space passage (success rates over 20 seeds)
* ik_local_minimum.png - gradient-based IK stalls at joint limits from a bad seed; multi-start succeeds
* cartesian_vs_cspace.gif - straight-line Cartesian interpolation collides; C-space planning does not
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position, subsample
from manipulatorcore_viz.draw2d import draw_obstacles_2d, draw_planar_arm, setup_planar_axes
from manipulatorcore_viz.draw3d import draw_obstacles_3d, draw_path_3d, draw_robot_3d, draw_triad, setup_axes_3d
from manipulatorcore_viz.style import PALETTE, caption
from render_rrt_arm import cspace_axes, draw_path, tree_segments


def narrow_passage(args) -> None:
    d = load_demo(args.data, "narrow_passage")
    fig = plt.figure(figsize=(11, 4.2), dpi=110)
    caption(
        fig,
        "Narrow passage",
        "planar 2R reaching through a 0.25 m slit · same budget (3000 samples) · success over 20 seeds",
    )
    ax_w = fig.add_axes([0.0, 0.02, 0.30, 0.82])
    setup_planar_axes(ax_w, 1.95)
    draw_obstacles_2d(ax_w, d["obstacles"])
    draw_planar_arm(ax_w, d["start"], color=PALETTE["ghost"], alpha=0.6)
    draw_planar_arm(ax_w, d["goal"], color=PALETTE["target"])
    for i, name in enumerate(["RRT", "RRT-Connect"]):
        ax = fig.add_axes([0.34 + i * 0.33, 0.12, 0.28, 0.70])
        cspace_axes(ax, d["cspace"])
        run = d["runs"][name]
        segs, colors = tree_segments(run["tree"])
        ax.add_collection(LineCollection(segs, colors=colors, linewidths=0.5, zorder=2))
        draw_path(ax, run["path"], PALETTE["trace"], 2.0)
        rate = d["stats"][name]["success_rate"]
        ax.set_title(f"{name}: {100 * rate:.0f}% solved", fontsize=9)
    path = args.out / "narrow_passage.png"
    fig.savefig(path)
    plt.close(fig)
    report(path)


def ik_local_minimum(args) -> None:
    d = load_demo(args.data, "ik_local_minimum")
    reach = sum(d["model"]["lengths"]) + 0.1
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.4), dpi=110)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.82, bottom=0.02, wspace=0.05)
    caption(
        fig,
        "IK is local",
        "Planar3R with joint limits of +-1.9 rad · same target · DLS from one seed vs seeded multi-start",
    )
    single = d["single_start"]
    for ax in axes:
        setup_planar_axes(ax, reach)
        ax.scatter(*d["target"], marker="x", s=80, color=PALETTE["target"], lw=2.5, zorder=10)
    for i, f in enumerate(single["frames"][::6]):
        draw_planar_arm(axes[0], f, color=PALETTE["ghost"], alpha=0.15 + 0.5 * i / 10, joints=False)
    draw_planar_arm(axes[0], single["frames"][-1], color=PALETTE["collision"])
    axes[0].set_title(f"single start: {single['status']} ({single['position_error']:.2f} m away)", fontsize=9)
    if d["multi_start"] is not None:
        draw_planar_arm(axes[1], d["multi_start"])
        axes[1].set_title("multi-start: converged", fontsize=9)
    path = args.out / "ik_local_minimum.png"
    fig.savefig(path)
    plt.close(fig)
    report(path)


def cartesian_vs_cspace(args) -> None:
    d = load_demo(args.data, "cartesian_vs_cspace")
    cart = d["cartesian"]
    plan = d["planned"]
    rec = FrameRecorder()
    n = 60
    cart_s, plan_s = subsample(cart, n), subsample(plan, n)
    cart_trace = np.asarray([ee_position(f) for f in cart])
    plan_trace = np.asarray([ee_position(f) for f in plan])
    for k in range(n):
        fig = plt.figure(figsize=(9.6, 4.6), dpi=100)
        caption(
            fig,
            "Cartesian interpolation vs configuration-space planning",
            "Redundant7R · same start and goal grasps · red links = collision",
        )
        for i, (frames, sampled, trace, title) in enumerate(
            (
                (cart, cart_s, cart_trace, "straight line in Cartesian space + IK"),
                (plan, plan_s, plan_trace, "RRT-Connect + shortcut + time-optimal timing"),
            )
        ):
            ax = fig.add_axes([i * 0.5, 0.0, 0.5, 0.86], projection="3d")
            setup_axes_3d(ax, (0.35, 0.0, 0.45), 0.6, elev=24, azim=-35)
            f = sampled[k]
            colliding = bool(f.get("collision"))
            draw_obstacles_3d(ax, d["obstacles"], alpha=0.3, highlight={"pillar"} if colliding else set())
            draw_obstacles_3d(ax, d["base"])
            draw_triad(ax, d["start_pose"], length=0.08, alpha=0.6)
            draw_triad(ax, d["goal_pose"], length=0.08, alpha=0.6)
            idx = frames.index(f)
            draw_path_3d(ax, trace[: idx + 1], PALETTE["collision"] if i == 0 else PALETTE["trace"], lw=2.0)
            hl = dict.fromkeys(range(7), PALETTE["collision"]) if colliding else None
            draw_robot_3d(ax, f, highlight=hl)
            hits = sum(1 for g in frames[: idx + 1] if g.get("collision"))
            ax.set_title(f"{title}\n{hits} colliding samples so far", fontsize=9, y=0.95)
        rec.capture(fig, 70)
        plt.close(fig)
    rec.hold(1500)
    report(rec.save(args.out / "cartesian_vs_cspace.gif", colors=96))


def main() -> None:
    args = parse_args(__doc__)
    narrow_passage(args)
    ik_local_minimum(args)
    cartesian_vs_cspace(args)


if __name__ == "__main__":
    main()
