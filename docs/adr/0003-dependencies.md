# ADR 0003 — Dependencies

## Status

Accepted.

## Context

The library must build on a CPU-only laptop on Linux and Windows without ROS, GPU toolkits or large
frameworks, and an installed package must be consumable with `find_package`.

## Decision

* **Core:** C++20 standard library and Eigen 3.4 only. An installed Eigen is preferred; otherwise the
  pinned release tarball (SHA-256 checked) is fetched headers-only — Eigen's own CMake project is not
  configured — and installed under `include/manipulatorcore/third_party/eigen3` with its MPL2
  licence. `EIGEN_MPL2_ONLY` is defined.
* **Tests and benchmarks:** GoogleTest 1.15 and Google Benchmark 1.9, fetched when not installed,
  never part of the installed package.
* **Python:** numpy, matplotlib and Pillow for rendering; pybind11 only for the optional bindings.
* **URDF:** a small built-in XML reader instead of an XML library (URDF import/export are optional
  conveniences, not requirements of the core).
* **ROS 2:** only in the optional `ros2/` colcon package, which consumes the installed library.

## Consequences

* A plain `cmake -S . -B build && cmake --build build` works offline once the dependencies are cached.
* The installed package is self-contained when Eigen was fetched; when a system Eigen is used the
  package config calls `find_dependency(Eigen3)`.
* The XML reader supports the URDF subset needed for serial chains (no DTDs, namespaces or external
  entities), which is documented.
