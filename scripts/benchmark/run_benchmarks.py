"""Runs the kernel benchmarks (Google Benchmark) and the metric benchmarks, and writes
docs/benchmarks/README.md.

Only machine-neutral metadata is recorded (CPU model, core count, compiler and build type): no host
names, user names or paths.

Example:
    python scripts/benchmark/run_benchmarks.py --build build/release
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def executable(build: Path, name: str) -> Path:
    for candidate in (build / "bin" / f"{name}.exe", build / "bin" / name):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"{name} not found under {build}/bin; build the Release configuration first")


def cmake_cache(build: Path, key: str) -> str:
    cache = build / "CMakeCache.txt"
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith(f"{key}:"):
            return line.split("=", 1)[1]
    return "unknown"


def compiler_description(build: Path) -> str:
    """Compiler id and version from CMake's compiler-detection file (not stored in the cache)."""
    for detected in sorted(build.glob("CMakeFiles/*/CMakeCXXCompiler.cmake")):
        values = {}
        for line in detected.read_text(encoding="utf-8", errors="replace").splitlines():
            for key in ("CMAKE_CXX_COMPILER_ID", "CMAKE_CXX_COMPILER_VERSION"):
                if line.startswith(f"set({key} "):
                    values[key] = line.split(" ", 1)[1].rstrip(")").strip('"')
        if values:
            return f"{values.get('CMAKE_CXX_COMPILER_ID', '?')} {values.get('CMAKE_CXX_COMPILER_VERSION', '')}".strip()
    return Path(cmake_cache(build, "CMAKE_CXX_COMPILER")).name


def cpu_name() -> str:
    if sys.platform == "win32":
        try:
            out = subprocess.run(
                ["powershell", "-NoProfile", "-Command", "(Get-CimInstance Win32_Processor).Name"],
                capture_output=True,
                text=True,
                check=True,
            )
            return out.stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            pass
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def format_time(ns: float) -> str:
    if ns < 1e3:
        return f"{ns:.0f} ns"
    if ns < 1e6:
        return f"{ns / 1e3:.1f} us"
    return f"{ns / 1e6:.1f} ms"


def kernel_tables(results: dict) -> str:
    to_ns = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
    rows = []
    for b in results["benchmarks"]:
        if b.get("run_type") == "aggregate":
            continue
        ns = b["real_time"] * to_ns[b.get("time_unit", "ns")]
        rows.append((b["name"], ns, b["iterations"]))
    lines = [
        "## Kernel timings (Google Benchmark)\n",
        "Wall-clock time per call. `/6` and `/7` denote the Industrial6R and Redundant7R models.\n",
        "| Kernel | Time per call | Iterations |",
        "| --- | ---: | ---: |",
    ]
    lines += [f"| `{name}` | {format_time(ns)} | {it} |" for name, ns, it in rows]
    return "\n".join(lines) + "\n\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", type=Path, required=True, help="Release build directory")
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "benchmarks" / "README.md")
    parser.add_argument("--min-time", default="0.2s", help="Google Benchmark minimum time per kernel")
    args = parser.parse_args()

    build_type = cmake_cache(args.build, "CMAKE_BUILD_TYPE")
    if build_type not in ("Release", "RelWithDebInfo"):
        print(f"warning: build type is {build_type}; timings are only meaningful for Release", file=sys.stderr)

    with tempfile.TemporaryDirectory() as tmp:
        kernel_json = Path(tmp) / "kernels.json"
        subprocess.run(
            [
                str(executable(args.build, "mc_benchmarks")),
                f"--benchmark_min_time={args.min_time}",
                f"--benchmark_out={kernel_json}",
                "--benchmark_out_format=json",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        kernels = json.loads(kernel_json.read_text(encoding="utf-8"))
        metrics_md = Path(tmp) / "metrics.md"
        subprocess.run(
            [str(executable(args.build, "mc_metric_benchmarks")), str(metrics_md)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        metrics = metrics_md.read_text(encoding="utf-8")

    compiler = compiler_description(args.build)
    context = kernels.get("context", {})
    header = [
        "# Benchmarks\n",
        "All numbers on this page are measured by `scripts/benchmark/run_benchmarks.py` from the "
        "`mc_benchmarks` (Google Benchmark kernels) and `mc_metric_benchmarks` (algorithm quality) "
        "executables of a Release build. Results depend on the machine; rerun the script to reproduce "
        "them locally.\n",
        "| Item | Value |",
        "| --- | --- |",
        f"| CPU | {cpu_name()} ({context.get('num_cpus', '?')} logical cores) |",
        f"| OS | {platform.system()} {platform.release()} |",
        f"| Compiler | {compiler} |",
        f"| Build type | {build_type} |",
        "| Threads | single-threaded |\n",
    ]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(header) + "\n" + kernel_tables(kernels) + metrics, encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
