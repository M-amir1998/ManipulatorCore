# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-11-21

Initial release.

### Added

- **Geometry:** SO(3)/SE(3) exponential and logarithm maps with robust small-angle and near-π
  handling, adjoints, quaternion/axis-angle/Euler conversions, SLERP and screw interpolation,
  collision primitives.
- **Model:** serial-chain robot model with inertia and armature, standard and modified DH tables,
  URDF import/export, built-in generic Planar2R, Planar3R, Industrial6R and Redundant7R arms.
- **Kinematics:** chain, DH and product-of-exponentials forward kinematics; space, body, geometric
  and point Jacobians and Jdot*qd; SVD singularity and manipulability analysis; workspace sampling.
- **Inverse kinematics:** closed-form planar 2R/3R; Jacobian transpose, pseudoinverse, damped least
  squares (fixed/adaptive) and Levenberg-Marquardt; active-set joint-limit clamping; null-space
  objectives with refinement; multi-start IK.
- **Dynamics:** RNEA, CRBA, ABA, Christoffel Coriolis matrix, energies, closed-form 2R reference,
  explicit/semi-implicit Euler and RK4 integrators.
- **Trajectories:** polynomial, trapezoidal and double-S profiles, splines, Cartesian line/arc paths,
  and reachability-based time-optimal parameterization with verification.
- **Collision:** signed distances for spheres, capsules, boxes and half-spaces, GJK, SAT depth, AABB
  broad phase and a robot checker with allowed-collision matrix and margins.
- **Planning:** RRT, RRT-Connect, RRT*, PRM, shortcutting, re-validated spline smoothing, a pose-goal
  pipeline with clearance and verification, and deterministic scenarios.
- **Control:** gravity compensation, PID/PD (+ gravity), computed torque, Cartesian PD, impedance,
  admittance and operational-space control with null-space posture and joint-limit avoidance.
- **Simulation:** deterministic manipulator simulator with saturation, damping, joint-limit stops
  and compliant contact.
- **Tooling:** demos and metric benchmarks, Google Benchmark kernels, reproducible animations,
  Python rendering package and pybind11 bindings, optional ROS 2 package, Jenkins pipeline,
  clang-format/clang-tidy/Ruff configuration, CMake presets and install/package support.
