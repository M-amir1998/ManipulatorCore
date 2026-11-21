"""Regenerates every documentation asset from source.

1. Runs the C++ demo application (``mc_demos``) to compute the data (JSON) into --data.
2. Runs every render script, writing GIF/PNG assets into --out (default docs/assets).

Example:
    python scripts/render/render_all.py --demos build/release/bin/mc_demos
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = [
    "render_hero_demo.py",
    "render_fk_ik.py",
    "render_singularity.py",
    "render_rrt_arm.py",
    "render_cartesian.py",
    "render_computed_torque.py",
    "render_impedance.py",
    "render_nullspace.py",
    "render_collision.py",
    "render_workspace.py",
    "render_failures.py",
    "render_gravity_osc.py",
]


def find_demos() -> Path | None:
    for pattern in ("build/*/bin/mc_demos*", "build/bin/mc_demos*"):
        for candidate in sorted(REPO.glob(pattern)):
            if candidate.suffix in ("", ".exe"):
                return candidate
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demos", type=Path, default=None, help="path to the mc_demos executable")
    parser.add_argument("--data", type=Path, default=REPO / "build" / "demo_data")
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "assets")
    parser.add_argument("--skip-demos", action="store_true", help="reuse existing JSON in --data")
    args = parser.parse_args()

    if not args.skip_demos:
        demos = args.demos or find_demos()
        if demos is None or not demos.exists():
            print("mc_demos not found; build the project first or pass --demos", file=sys.stderr)
            return 2
        subprocess.run([str(demos), str(args.data)], check=True)

    here = Path(__file__).resolve().parent
    for script in SCRIPTS:
        subprocess.run(
            [sys.executable, str(here / script), "--data", str(args.data), "--out", str(args.out)], check=True
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
