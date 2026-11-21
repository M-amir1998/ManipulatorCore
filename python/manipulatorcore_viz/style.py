"""Shared visual style: one palette and one set of matplotlib defaults for every asset."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

PALETTE = {
    "background": "#ffffff",
    "panel": "#f7f8fa",
    "grid": "#e3e6eb",
    "text": "#1f2933",
    "muted": "#7b8794",
    "link": "#2f3b52",
    "link_alt": "#52627a",
    "joint": "#ffffff",
    "base": "#8a94a6",
    "ghost": "#9aa5b8",
    "obstacle": "#c9ced6",
    "obstacle_edge": "#8a94a6",
    "trace": "#e4572e",
    "plan": "#3a86ff",
    "target": "#2a9d8f",
    "warning": "#f4a261",
    "collision": "#d62828",
    "safe": "#2a9d8f",
    "axis_x": "#d62728",
    "axis_y": "#2ca02c",
    "axis_z": "#1f77b4",
    "series": ["#3a86ff", "#e4572e", "#2a9d8f", "#8338ec", "#f4a261", "#6c757d", "#ff006e"],
}


def apply_style() -> None:
    """Applies the common matplotlib style (white background, light grids, compact fonts)."""
    plt.rcParams.update(
        {
            "figure.facecolor": PALETTE["background"],
            "axes.facecolor": PALETTE["background"],
            "axes.edgecolor": PALETTE["grid"],
            "axes.labelcolor": PALETTE["text"],
            "axes.titlesize": 10,
            "axes.titleweight": "bold",
            "axes.labelsize": 9,
            "axes.grid": True,
            "grid.color": PALETTE["grid"],
            "grid.linewidth": 0.6,
            "xtick.color": PALETTE["muted"],
            "ytick.color": PALETTE["muted"],
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 8,
            "legend.frameon": False,
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "text.color": PALETTE["text"],
            "lines.solid_capstyle": "round",
            "savefig.facecolor": PALETTE["background"],
        }
    )


def caption(fig, title: str, subtitle: str = "") -> None:
    """Small title in the top-left corner of a figure (kept minimal on purpose)."""
    fig.text(0.012, 0.975, title, ha="left", va="top", fontsize=11, fontweight="bold", color=PALETTE["text"])
    if subtitle:
        fig.text(0.012, 0.935, subtitle, ha="left", va="top", fontsize=8.5, color=PALETTE["muted"])
