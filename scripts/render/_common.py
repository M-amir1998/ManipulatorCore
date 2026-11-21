"""Shared command-line handling for the render scripts.

Every script reads the JSON written by ``mc_demos`` (default: build/demo_data) and writes its asset
to docs/assets (default). Run ``python scripts/render/render_all.py --demos <path to mc_demos>`` to
regenerate everything.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

from manipulatorcore_viz.style import apply_style  # noqa: E402


def parse_args(description: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--data", type=Path, default=REPO / "build" / "demo_data", help="mc_demos output directory")
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "assets", help="asset output directory")
    parser.add_argument("--preview", type=Path, default=None, help="also write a few PNG frames here")
    args = parser.parse_args()
    apply_style()
    return args


def report(path: Path) -> None:
    size = path.stat().st_size / 1e6
    print(f"wrote {path.relative_to(REPO) if path.is_relative_to(REPO) else path} ({size:.2f} MB)")
