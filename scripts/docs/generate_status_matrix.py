"""Generates the algorithm status matrix in README.md from configs/algorithms.yaml.

The table is written between the markers ``<!-- status-matrix:start -->`` and
``<!-- status-matrix:end -->``. ``--check`` exits with status 1 if README.md is out of date.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
START = "<!-- status-matrix:start -->"
END = "<!-- status-matrix:end -->"
STATUS = {"implemented": "✅", "experimental": "🧪 experimental", "planned": "🗺️ planned"}


def render(registry: dict) -> str:
    lines = [
        "| Area | Algorithm | Implemented | Tested | Demo | Benchmark |",
        "| --- | --- | :---: | :---: | :---: | :---: |",
    ]
    for entry in registry["algorithms"]:
        status = entry["status"]
        if status not in STATUS:
            raise ValueError(f"unknown status '{status}' for {entry['name']}")
        tests = entry.get("tests", [])
        for test in tests:
            if not (REPO / test).exists():
                raise FileNotFoundError(f"{entry['name']}: test file {test} does not exist")
        demo = entry.get("demo")
        if demo and not (REPO / "docs" / "assets" / demo).exists():
            raise FileNotFoundError(f"{entry['name']}: demo asset {demo} does not exist")
        demo_cell = f"[{demo.split('.')[0]}](docs/assets/{demo})" if demo else "—"
        tested_cell = "✅" if tests else "—"
        bench = entry.get("benchmark")
        bench_cell = "✅" if bench else "—"
        lines.append(
            f"| {entry['area']} | {entry['name']} | {STATUS[status]} | {tested_cell} | {demo_cell} | {bench_cell} |"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if README.md is out of date")
    parser.add_argument("--readme", type=Path, default=REPO / "README.md")
    args = parser.parse_args()
    registry = yaml.safe_load((REPO / "configs" / "algorithms.yaml").read_text(encoding="utf-8"))
    table = render(registry)
    text = args.readme.read_text(encoding="utf-8")
    if START not in text or END not in text:
        print(f"markers not found in {args.readme}", file=sys.stderr)
        return 1
    head, rest = text.split(START, 1)
    _, tail = rest.split(END, 1)
    updated = f"{head}{START}\n{table}\n{END}{tail}"
    if args.check:
        if updated != text:
            print("README.md status matrix is out of date; run scripts/docs/generate_status_matrix.py", file=sys.stderr)
            return 1
        print("status matrix up to date")
        return 0
    args.readme.write_text(updated, encoding="utf-8", newline="\n")
    print(f"updated {args.readme}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
