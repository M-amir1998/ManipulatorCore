# Project status

Snapshot of what is implemented and how it was verified. Every number below was measured locally;
nothing is extrapolated. Last updated for version 0.1.0.

## Summary

| Item | Status |
| --- | --- |
| Repository | <https://github.com/M-amir1998/ManipulatorCore> |
| Version | 0.1.0 |
| C++ standard | C++20 |
| Build system | CMake ≥ 3.21, presets for Linux and Windows |
| Compilers verified | MSVC 19.50 (Visual Studio 2026), clang-cl 20.1, GCC 13.3 (Ubuntu 24.04 container) |
| Core dependency | Eigen 3.4.0 (system package or fetched headers, SHA-256 pinned) |
| Test/benchmark dependencies | GoogleTest 1.15.2, Google Benchmark 1.9.1 (fetched, SHA-256 pinned) |
| Optional | pybind11 2.13.6 (Python bindings), ROS 2 Jazzy (`ros2/` adapter), Python 3.11+ with NumPy, Matplotlib, Pillow (rendering) |
| CI | Jenkins only (`Jenkinsfile`); no GitHub Actions |
| Licence | MIT |

## Implemented

**Robot models.** `Planar2R`, `Planar3R`, `Industrial6R` (standard DH, spherical wrist) and
`Redundant7R` (S-R-S), all generic with cylinder inertias, armature, friction, limits and capsule
collision geometry. Models can also be built from standard/modified DH tables or URDF (serial
subset), and exported to URDF.

**Geometry.** SO(3)/SE(3) exp/log with small-angle series and near-π handling, adjoints, left
Jacobian and inverse, Lie bracket; quaternion (Hamilton, scalar-first), axis-angle and intrinsic
Euler ZYX/XYZ/ZYZ conversions; SLERP, geodesic and screw interpolation; sphere, capsule, box and
half-space primitives.

**Kinematics.** Chain FK, direct DH product, PoE (space and body); space, body and geometric
Jacobians, point Jacobians and $\dot J \dot q$; SVD rank, condition number and Yoshikawa
manipulability; workspace sampling.

**IK methods.** Closed-form planar 2R/3R (all branches); Jacobian transpose; SVD pseudoinverse;
damped least squares (fixed and adaptive damping); Levenberg–Marquardt; task weights; active-set
joint-limit clamping; null-space objectives (centering, limit barrier, smooth activation,
manipulability); seeded multi-start.

**Dynamics algorithms.** RNEA, CRBA, ABA with armature; gravity, bias and Coriolis torques;
Christoffel Coriolis matrix; kinetic/potential energy; explicit Euler, semi-implicit Euler and RK4.

**Trajectory methods.** Cubic and quintic polynomials; trapezoidal and double-S (jerk-limited)
profiles; synchronized joint lines; natural and clamped cubic splines; Cartesian line and arc with
SO(3) geodesic orientation; time-optimal parameterization by reachability analysis under joint
velocity and acceleration limits, with breakpoint grid refinement and post-solve verification.

**Collision model.** Exact sphere/capsule distances (segment–segment); sphere/capsule–box by
minimizing the convex box signed-distance function along the core (golden-section search); box–box by
GJK when separated and a 15-axis separating-axis test for penetration depth; half-space distances;
AABB broad phase; robot self- and environment checking with an allowed-pair matrix and separate
safety margins.

**Planning algorithms.** RRT, RRT-Connect, RRT\* (Karaman–Frazzoli radius, rewiring with cost
propagation), PRM with A\*; random shortcutting; spline smoothing with re-validation;
`planToPose` (multi-start IK → planning → smoothing → timing → verification); six deterministic
7-DOF scenarios.

**Controllers.** Gravity compensation, PD, PID with anti-windup, computed torque, Cartesian PD,
Cartesian impedance, admittance, operational-space control with dynamically consistent null-space
projection and posture/limit tasks.

**Simulation.** Deterministic fixed-step simulator (RK4 or semi-implicit Euler, zero-order-hold
control), torque saturation, viscous friction, joint-limit stops, penalty contact between a tool
sphere and planar surfaces; bit-identical repeated runs.

**Benchmarks.** Google Benchmark kernels (`mc_benchmarks`) and metric benchmarks (`mc_metric_benchmarks`:
IK success and accuracy, planner success and path quality, controller tracking). Results:
[docs/benchmarks/README.md](docs/benchmarks/README.md).

**Visualization.** `mc_demos` exports JSON from the C++ library; `python/manipulatorcore_viz` renders
GIF/PNG with Matplotlib on the CPU; `scripts/render/render_all.py` reproduces every figure.

**README animations.** Hero (7-DOF pose-goal pipeline) plus eight gallery animations: FK/IK, singularity,
RRT-Connect in C-space, Cartesian paths, computed torque, impedance, null-space, Cartesian vs C-space
failure. A further four animations and four figures are linked from the README and live in
`docs/assets`.

**Adapters.** pybind11 bindings (`manipulatorcore` module: models, URDF, Jacobians, IK, dynamics) and
the ROS 2 package `manipulatorcore_ros` (URDF export, planner node, simulator node, launch file,
RViz configuration).

## Verification

The C++ sources, tests and build files are identical at every result below (no changes since the
clang-cl UBSan fix); later commits touched only documentation, scripts and the ROS 2 package.

| Check | Toolchain / scope | Result |
| --- | --- | --- |
| Release build + full test suite | MSVC 19.50, Windows 11, `/W4 /WX` | 327 tests: 327 passed, 0 failed, 0 skipped (297 unit, 13 numerical, 14 integration, 3 smoke) |
| Debug build + full test suite | GCC 13.3, Ubuntu 24.04 container, `-Wall -Wextra -Wpedantic` and more, as errors | 327/327 passed, 0 compiler warnings |
| AddressSanitizer + UBSan | GCC 13.3, `-fno-sanitize-recover=all`, leak detection on | 327/327 passed, no reports |
| AddressSanitizer | MSVC 19.50, `/fsanitize=address` | 327/327 passed, no reports |
| UndefinedBehaviorSanitizer | clang-cl 20.1, trap mode | 327/327 passed |
| clang-tidy | 20.1 with the repository `.clang-tidy`, all 45 library sources (MSVC compile database) | no findings |
| clang-format | 20.1, all 143 tracked C++ files | clean |
| Python tests | pytest: visualization, bindings, status matrix | 11 passed, 0 skipped (the bindings test skips when the module is not built) |
| Python Ruff | `ruff check` on `python/`, `scripts/`, `ros2/`; `ruff format --check` on `python/` and `scripts/` | clean (the ROS launch file keeps the ROS single-quote style) |
| Install / `find_package` | MSVC install tree + `examples/external_project` | consumer configures, builds and runs |
| ROS 2 package | Jazzy container (GCC 13): colcon build + test, launch smoke test | 20/20 tests; one goal planned in 0.04 s (goal error 2.8e-5 m) and executed once in the simulator |
| Benchmarks | MSVC Release, pinned to one performance core | [docs/benchmarks/README.md](docs/benchmarks/README.md) |
| Jenkins | `Jenkinsfile` (declarative, `disableConcurrentBuilds`, `RENDER_DOCS=false` by default) | not executed on a Jenkins server; the commands of its stages were run locally as listed above, on Windows and in an Ubuntu container rather than on a Jenkins agent |

## Git

* Authors: every commit is authored by the repository owner (`git log --format='%an <%ae>'`).
* Repository size: about 3.8 MB (compressed) of reachable history and 244 tracked files; the largest
  file is a 0.5 MB GIF. Rendered frames, videos, build trees and benchmark scratch
  data are excluded by `.gitignore`.
* Secret scan: `gitleaks git` over the full history reports no leaks.

## Known limitations

* Serial chains only; no branched or closed kinematic trees.
* Collision geometry is limited to spheres, capsules, boxes and half-spaces. URDF meshes are ignored and
  cylinders are imported as capsules.
* Collision checking along edges is discrete (fixed resolution); there is no continuous (swept-volume)
  check, so thin obstacles require a fine resolution or a safety margin.
* Analytical IK covers only the planar 2R/3R arms. The 6R and 7R arms use local numerical IK, whose
  cold-start success depends on the seed (see the IK benchmark); multi-start mitigates this.
* Nearest-neighbour queries in the planners are linear scans, which dominates RRT\* and PRM run time
  for large roadmaps.
* Time parameterization handles joint velocity and acceleration limits only; torque and jerk limits are
  not part of it.
* The simulator is a validation tool, not a physics engine: one-sided penalty contact between a tool
  sphere and planes, no friction cone, no multi-body contact.
* The Jenkins pipeline has been written and the commands of its stages were run locally, but it has
  not been executed on a Jenkins server.
* The ROS 2 adapter was verified on Jazzy in a container only; RViz was not exercised there, and it is
  not a `ros2_control` hardware interface or a MoveIt plugin.
* Benchmarks come from a single laptop CPU and should be read as relative, not absolute, numbers.

## Future algorithms

* Torque- and jerk-constrained time parameterization.
* Continuous collision checking and convex mesh geometry.
* kd-tree or ball-tree nearest-neighbour search for the planners; informed RRT\* and BIT\*.
* Closed-form IK for spherical-wrist 6R arms and for S-R-S 7R arms with an arm-angle parameter.
* Hybrid force/position control and impedance with inertia shaping.
* Trajectory optimization (CHOMP/TrajOpt-style) as a post-processing stage.
