# ManipulatorCore documentation

## Start here

* [Conventions](mathematics/conventions.md) — frames, rotations, quaternions, Euler angles, twists,
  wrenches, Jacobians, dynamics, units, tolerances, error handling and randomness. Read this first.
* [Architecture](architecture/overview.md) — modules, dependencies, data flow and design policies.
* [Benchmarks](benchmarks/README.md) — measured kernel timings and algorithm-quality metrics.

## Algorithms

| Area | Page |
| --- | --- |
| Rigid-body geometry | [SO(3), SE(3), rotation conversions, interpolation](algorithms/rigid-body-transforms.md) |
| Robot model | [Joints, inertia, DH tables, built-in robots, URDF](algorithms/robot-model.md) |
| Kinematics | [Forward kinematics: chain, DH, product of exponentials](algorithms/forward-kinematics.md) |
| Kinematics | [Jacobians, singularities, manipulability, workspace](algorithms/jacobians-and-singularities.md) |
| Kinematics | [Inverse kinematics: analytical, numerical, redundancy](algorithms/inverse-kinematics.md) |
| Dynamics | [RNEA, CRBA, ABA, forward dynamics, energy](algorithms/dynamics.md) |
| Trajectories | [Polynomials, trapezoid, double-S, splines, Cartesian paths](algorithms/trajectory-generation.md) |
| Trajectories | [Time-optimal path parameterization](algorithms/time-parameterization.md) |
| Collision | [Primitive distances, GJK, broad phase, robot checker](algorithms/collision-detection.md) |
| Planning | [RRT, RRT-Connect, RRT*, PRM, smoothing, pose-goal pipeline](algorithms/motion-planning.md) |
| Control | [Gravity compensation, PD/PID, computed torque](algorithms/joint-space-control.md) |
| Control | [Cartesian PD, impedance, admittance, operational space](algorithms/task-space-control.md) |
| Simulation | [Deterministic simulator, integrators, contact model](algorithms/simulation.md) |

## Tutorials

* [Getting started](tutorials/getting-started.md) — build, test, first program.
* [Regenerating the figures](tutorials/rendering.md) — demos, renderers and assets.
* [ROS 2 integration](tutorials/ros2.md) — optional adapter package.

## Design decisions

* [ADR 0001 — Twist and Jacobian conventions](adr/0001-twist-and-jacobian-conventions.md)
* [ADR 0002 — Error handling](adr/0002-error-handling.md)
* [ADR 0003 — Dependencies](adr/0003-dependencies.md)
* [ADR 0004 — Visualization pipeline](adr/0004-visualization-pipeline.md)

## Algorithm registry

[configs/algorithms.yaml](../configs/algorithms.yaml) lists every algorithm with its status, tests,
demo, benchmark and reference. The status matrix in the top-level README is generated from it by
`scripts/docs/generate_status_matrix.py`.
