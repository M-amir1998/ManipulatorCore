"""Hero animation: Cartesian goal -> IK -> RRT-Connect -> time-optimal timing -> computed torque.

Everything drawn comes from the C++ pipeline (demo ``hero``): IK candidates, planner paths, the timed
reference and the executed motion from the simulator.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from _common import parse_args, report
from manipulatorcore_viz import FrameRecorder, load_demo
from manipulatorcore_viz.data import ee_position, subsample
from manipulatorcore_viz.draw3d import (
    draw_obstacles_3d,
    draw_path_3d,
    draw_robot_3d,
    draw_triad,
    setup_axes_3d,
)
from manipulatorcore_viz.style import PALETTE

STEPS = [
    "Cartesian target pose",
    "Multi-start IK + collision filter",
    "RRT-Connect + shortcutting",
    "Spline + time-optimal timing",
    "Computed-torque execution",
]


def main() -> None:
    args = parse_args(__doc__)
    d = load_demo(args.data, "hero")
    s = d["summary"]
    fig = plt.figure(figsize=(9.6, 5.4), dpi=100)
    ax = fig.add_axes([0.0, 0.0, 0.70, 1.0], projection="3d")
    panel = fig.add_axes([0.70, 0.0, 0.30, 1.0])
    panel.set_axis_off()

    center = (0.30, 0.05, 0.55)
    half = 0.62
    executed = d["executed"]
    candidates = d["ik_candidates"]
    chosen = d["chosen_candidate"]
    raw = np.asarray(d["raw_path_ee"])
    short = np.asarray(d["shortcut_path_ee"])
    planned = np.asarray(d["planned_ee"])

    notes = [
        "tool frame inside the middle compartment",
        f"{s['ik_candidates']} collision-free solutions",
        f"path {s['raw_length']:.2f} -> {s['shortcut_length']:.2f} rad",
        f"{s['trajectory_duration']:.2f} s, joint v/a limits",
        f"tracking error <= {1e3 * s['max_tracking_error']:.1f} mm",
    ]

    def draw(step: int, robot_frame, ghosts=(), raw_n=0, short_n=0, trace=None, done=False):
        ax.cla()
        setup_axes_3d(ax, center, half, elev=24, azim=-52)
        draw_obstacles_3d(ax, d["obstacles"])
        draw_triad(ax, d["target"], length=0.12, lw=2.4)
        for ghost in ghosts:
            draw_robot_3d(ax, ghost, color=PALETTE["ghost"], alpha=0.22, joints=False)
        if raw_n > 1:
            draw_path_3d(ax, raw[:raw_n], PALETTE["plan"], lw=1.0, alpha=0.45)
        if short_n > 1:
            draw_path_3d(ax, short[:short_n], PALETTE["plan"], lw=2.0, ls="--")
        if step >= 3:
            draw_path_3d(ax, planned, PALETTE["plan"], lw=1.6, ls="--", alpha=0.7)
        if trace is not None and len(trace) > 1:
            draw_path_3d(ax, trace, PALETTE["trace"], lw=2.4)
        draw_robot_3d(ax, robot_frame)
        draw_triad(ax, robot_frame["ee"], length=0.08, lw=1.4)

        panel.cla()
        panel.set_axis_off()
        panel.set_xlim(0, 1)
        panel.set_ylim(0, 1)
        panel.text(0.02, 0.93, "ManipulatorCore", fontsize=13, fontweight="bold", color=PALETTE["text"])
        panel.text(0.02, 0.88, "7-DOF arm · shelf insertion", fontsize=9, color=PALETTE["muted"])
        for i, label in enumerate(STEPS):
            y = 0.76 - 0.12 * i
            active = i == step and not done
            finished = i < step or done
            color = PALETTE["text"] if active else (PALETTE["safe"] if finished else PALETTE["muted"])
            marker = ">" if active else ("+" if finished else "-")
            panel.text(
                0.02,
                y,
                f"{marker} {i + 1}. {label}",
                fontsize=9.5,
                fontweight="bold" if active else "normal",
                color=color,
            )
            if finished or active:
                panel.text(0.09, y - 0.045, notes[i], fontsize=8, color=PALETTE["muted"])
        if done:
            panel.text(0.02, 0.12, "target reached", fontsize=11, fontweight="bold", color=PALETTE["safe"])
            panel.text(
                0.02,
                0.07,
                f"final error {1e3 * s['final_position_error']:.2f} mm, {1e3 * s['final_orientation_error']:.2f} mrad",
                fontsize=8.5,
                color=PALETTE["muted"],
            )

    rec = FrameRecorder()
    start = d["start"]
    # 1. Target pose.
    draw(0, start)
    rec.capture(fig, 900)
    # 2. IK candidates appear one by one.
    for k in range(1, len(candidates) + 1):
        draw(1, start, ghosts=candidates[:k])
        rec.capture(fig, 260)
    rec.hold(500)
    # 3. Planner path, then the shortcut.
    for n in np.linspace(2, len(raw), 10).round().astype(int):
        draw(2, start, ghosts=[candidates[chosen]], raw_n=int(n))
        rec.capture(fig, 70)
    for n in np.linspace(2, len(short), 6).round().astype(int):
        draw(2, start, ghosts=[candidates[chosen]], raw_n=len(raw), short_n=int(n))
        rec.capture(fig, 90)
    rec.hold(500)
    # 4. Timed reference shown; 5. execution in the simulator.
    draw(3, start, ghosts=[candidates[chosen]], short_n=len(short))
    rec.capture(fig, 900)
    frames = subsample(executed, 70)
    trace = np.asarray([ee_position(f) for f in executed])
    times = np.asarray([f["t"] for f in executed])
    for f in frames:
        k = int(np.searchsorted(times, f["t"], side="right"))
        draw(4, f, trace=trace[:k])
        rec.capture(fig, 40)
    draw(4, executed[-1], trace=trace, done=True)
    rec.capture(fig, 2600)

    if args.preview:
        args.preview.mkdir(parents=True, exist_ok=True)
        for i in (0, len(rec.frames) // 3, len(rec.frames) // 2, len(rec.frames) - 1):
            rec.frames[i].save(args.preview / f"hero_{i:03d}.png")
    report(rec.save(args.out / "hero.gif", colors=96))


if __name__ == "__main__":
    main()
