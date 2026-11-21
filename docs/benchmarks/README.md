# Benchmarks

All numbers on this page are measured by `scripts/benchmark/run_benchmarks.py` from the `mc_benchmarks` (Google Benchmark kernels) and `mc_metric_benchmarks` (algorithm quality) executables of a Release build. Results depend on the machine; rerun the script to reproduce them locally.

| Item | Value |
| --- | --- |
| CPU | 13th Gen Intel(R) Core(TM) i5-1335U (12 logical cores) |
| OS | Windows 11 |
| Compiler | MSVC 19.50.35729.0 |
| Build type | Release |
| Threads | single-threaded, pinned to logical CPU 2 |

## Kernel timings (Google Benchmark)

Wall-clock time per call. `/6` and `/7` denote the Industrial6R and Redundant7R models.

| Kernel | Time per call | Iterations |
| --- | ---: | ---: |
| `BM_ExpSO3` | 23 ns | 10000000 |
| `BM_LogSO3` | 16 ns | 17920000 |
| `BM_ExpSE3` | 52 ns | 5973333 |
| `BM_LogSE3` | 54 ns | 4480000 |
| `BM_SE3Compose` | 12 ns | 25600000 |
| `BM_QuaternionFromRotation` | 25 ns | 8960000 |
| `BM_ForwardKinematics/6` | 338 ns | 896000 |
| `BM_ForwardKinematics/7` | 403 ns | 896000 |
| `BM_GeometricJacobian/6` | 759 ns | 448000 |
| `BM_GeometricJacobian/7` | 796 ns | 358400 |
| `BM_InverseKinematicsDLS/6` | 33.4 us | 8960 |
| `BM_InverseKinematicsDLS/7` | 43.2 us | 8960 |
| `BM_InverseDynamicsRNEA/6` | 1.3 us | 224000 |
| `BM_InverseDynamicsRNEA/7` | 1.5 us | 256000 |
| `BM_MassMatrixCRBA/6` | 1.9 us | 243277 |
| `BM_MassMatrixCRBA/7` | 2.1 us | 100000 |
| `BM_ForwardDynamicsABA/6` | 3.5 us | 100000 |
| `BM_ForwardDynamicsABA/7` | 3.7 us | 89600 |
| `BM_ForwardDynamicsCRBA_LLT/6` | 3.9 us | 89600 |
| `BM_ForwardDynamicsCRBA_LLT/7` | 4.4 us | 89600 |
| `BM_CollisionFreeQuery` | 4.6 us | 59733 |
| `BM_MinimumDistanceQuery` | 6.9 us | 44800 |
| `BM_Planner/rrt_connect_obstacle_avoidance` | 2.0 ms | 149 |
| `BM_Planner/rrt_connect_shelf` | 480.6 ms | 1 |
| `BM_Planner/rrt_star_1500_tabletop` | 160.0 ms | 2 |
| `BM_ControllerCall/pd_gravity` | 1.7 us | 162909 |
| `BM_ControllerCall/computed_torque` | 1.6 us | 199111 |
| `BM_ControllerCall/cartesian_impedance` | 7.6 us | 44800 |
| `BM_ControllerCall/operational_space` | 10.5 us | 29867 |

## Inverse kinematics

Protocol: 300 reachable targets per robot, generated as the forward kinematics of uniformly random joint configurations inside the limits (seed 2024). Every solve starts from an independent uniformly random configuration (cold start); the warm-start column instead seeds each solve within +-0.3 rad of a known solution and measures local convergence. Tolerances: 0.1 mm and 1 mrad on the full 6-D pose. Iterations are averaged over converged solves; errors are medians over all solves. A limit violation is a returned configuration outside the joint limits.

### Industrial6R

| Method | Success (cold start) | Success (warm start) | Mean iterations | Median position error | Median orientation error | Mean time per solve | Limit violations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Jacobian transpose | 22.7 % | 65.7 % | 381.0 | 1.08e-01 m | 1.93e-02 rad | 564 us | 0 |
| Pseudoinverse (Gauss-Newton) | 32.0 % | 100.0 % | 11.5 | 1.26e-01 m | 4.40e-02 rad | 431 us | 0 |
| DLS (lambda = 0.05) | 32.0 % | 99.3 % | 14.7 | 1.26e-01 m | 3.91e-02 rad | 122 us | 0 |
| DLS (adaptive) | 31.3 % | 96.0 % | 18.6 | 1.26e-01 m | 3.71e-02 rad | 577 us | 0 |
| Levenberg-Marquardt | 32.7 % | 99.3 % | 15.3 | 1.18e-01 m | 2.37e-02 rad | 133 us | 0 |
| DLS + 8 random restarts | 91.3 % | 99.3 % | 14.7 | 4.55e-05 m | 9.92e-06 rad | 398 us | 0 |

### Redundant7R

| Method | Success (cold start) | Success (warm start) | Mean iterations | Median position error | Median orientation error | Mean time per solve | Limit violations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Jacobian transpose | 41.0 % | 65.0 % | 423.9 | 1.16e-03 m | 2.43e-04 rad | 946 us | 0 |
| Pseudoinverse (Gauss-Newton) | 67.0 % | 100.0 % | 12.3 | 2.26e-05 m | 2.79e-05 rad | 497 us | 0 |
| DLS (lambda = 0.05) | 66.3 % | 98.7 % | 15.6 | 7.80e-05 m | 1.55e-05 rad | 101 us | 0 |
| DLS (adaptive) | 64.0 % | 95.7 % | 17.8 | 6.96e-05 m | 3.03e-05 rad | 565 us | 0 |
| Levenberg-Marquardt | 70.7 % | 99.3 % | 16.4 | 6.15e-05 m | 1.30e-05 rad | 82 us | 0 |
| DLS + 8 random restarts | 99.7 % | 98.7 % | 18.6 | 5.07e-05 m | 9.87e-06 rad | 188 us | 0 |

Cold-start success is limited for every single-start method: from a random configuration the iterates often end in local minima or against joint limits. Warm starts show the local behaviour (the Jacobian transpose converges linearly and needs many iterations; damping costs a few iterations but tolerates singular configurations), and random restarts address the local nature of all gradient-based methods. Mean time per solve includes failed solves, which run to their iteration limit.

## Motion planning

Protocol: the six deterministic 7-DOF scenarios of `redundant7RScenarios()` (open reach, obstacle avoidance, narrow passage, shelf reach, tabletop reach, around obstacle), 5 seeds each. Budgets: 20 000 samples for RRT and RRT-Connect, 3 000 for RRT* (which always uses its whole budget to improve the path), 6 000 roadmap samples for PRM (grown in batches until the query succeeds). Steering step 0.4 rad, edge resolution 0.02 rad. Path length is the joint-space length in radians before and after 200 random shortcut iterations. Collision checks count state-validity queries.

### Summary over all scenarios

| Planner | Success | Median time | Path length | After shortcut | Nodes | Collision checks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RRT | 67 % | 7.9 ms | 8.46 | 4.59 | 5875 | 87403 |
| RRT-Connect | 100 % | 2.0 ms | 8.82 | 4.94 | 934 | 19464 |
| RRT* | 63 % | 426.4 ms | 5.79 | 4.50 | 2845 | 89817 |
| PRM | 90 % | 621.1 ms | 6.24 | 4.34 | 702 | 521217 |

### Per scenario

| Scenario | Planner | Success | Median time | Path length | After shortcut | Nodes | Collision checks |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| open_reach | RRT | 100 % | 2.2 ms | 3.79 | 2.39 | 76 | 1013 |
| open_reach | RRT-Connect | 100 % | 0.3 ms | 2.80 | 2.39 | 9 | 126 |
| open_reach | RRT* | 100 % | 282.8 ms | 2.78 | 2.39 | 2992 | 93795 |
| open_reach | PRM | 100 % | 0.2 ms | 2.39 | 2.39 | 2 | 110 |
| obstacle_avoidance | RRT | 100 % | 6.5 ms | 8.68 | 4.87 | 123 | 1689 |
| obstacle_avoidance | RRT-Connect | 100 % | 2.0 ms | 7.28 | 4.81 | 30 | 556 |
| obstacle_avoidance | RRT* | 100 % | 397.0 ms | 6.31 | 4.84 | 2937 | 90848 |
| obstacle_avoidance | PRM | 100 % | 600.1 ms | 7.72 | 5.36 | 167 | 179744 |
| narrow_passage | RRT | 0 % | 2950.4 ms | - | - | 16949 | 252746 |
| narrow_passage | RRT-Connect | 100 % | 289.8 ms | 11.44 | 4.96 | 2617 | 56179 |
| narrow_passage | RRT* | 0 % | 437.9 ms | - | - | 2643 | 82566 |
| narrow_passage | PRM | 40 % | 12342.0 ms | 4.83 | 3.08 | 3215 | 2176850 |
| shelf_reach | RRT | 0 % | 3239.1 ms | - | - | 17080 | 254456 |
| shelf_reach | RRT-Connect | 100 % | 183.9 ms | 17.12 | 6.39 | 2898 | 59085 |
| shelf_reach | RRT* | 0 % | 494.2 ms | - | - | 2659 | 89090 |
| shelf_reach | PRM | 100 % | 747.4 ms | 6.00 | 3.07 | 499 | 414639 |
| tabletop_reach | RRT | 100 % | 6.8 ms | 8.66 | 4.91 | 135 | 1793 |
| tabletop_reach | RRT-Connect | 100 % | 1.3 ms | 5.47 | 4.91 | 18 | 311 |
| tabletop_reach | RRT* | 100 % | 446.5 ms | 6.26 | 4.91 | 2969 | 92784 |
| tabletop_reach | PRM | 100 % | 677.5 ms | 7.21 | 4.95 | 164 | 184300 |
| around_obstacle | RRT | 100 % | 7.9 ms | 12.71 | 6.19 | 889 | 12718 |
| around_obstacle | RRT-Connect | 100 % | 1.9 ms | 8.80 | 6.19 | 31 | 525 |
| around_obstacle | RRT* | 80 % | 411.5 ms | 8.29 | 6.20 | 2867 | 89820 |
| around_obstacle | PRM | 100 % | 571.0 ms | 8.44 | 6.41 | 164 | 171658 |

## Control

Protocol: the generic 7-DOF arm in the simulator (RK4, 1 ms, joint damping and torque saturation enabled; the controllers' models do not include the joint damping). All controllers of a table track the same reference; metrics are computed from the simulated motion. The settling time is measured on the end-effector position error with a 1 mm band (0 s: the error never left the band; the full record length: it never settled). Time per control call is measured separately on 200 states.

### Joint-space tracking (quintic, 1.6 s, then 0.6 s hold)

Gains: Kp = 400 N m/rad, Kd = 40 N m s/rad (PD family; PID adds Ki = 200); computed torque uses critically damped error dynamics with omega = 25 rad/s.

| Controller | Joint RMSE | Cartesian RMSE | Orientation RMSE | RMS torque | Peak torque | Control effort | Settling time (1 mm) | Time per control call |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| PD | 3.36e-02 rad | 7.16e-02 m | 1.19e-01 rad | 13.9 N m | 48.7 N m | 2975 | 2.20 s | 0.5 us |
| PD + gravity | 2.49e-03 rad | 5.01e-03 m | 8.62e-03 rad | 13.0 N m | 38.8 N m | 2609 | 1.70 s | 1.9 us |
| PID + gravity | 2.54e-03 rad | 5.14e-03 m | 8.86e-03 rad | 13.0 N m | 38.8 N m | 2605 | 1.71 s | 2.0 us |
| Computed torque | 1.49e-03 rad | 2.24e-04 m | 1.71e-03 rad | 13.0 N m | 38.5 N m | 2599 | 0.00 s | 2.1 us |

### Task-space tracking (straight line 0.35 m with 0.3 rad rotation, 2 s, then 0.6 s hold)

Task-space PD and impedance use K = diag(150 N m/rad, 1500 N/m), D = diag(15, 120); operational-space control uses Kp = 400 1/s^2, Kd = 40 1/s on all six task coordinates. All use the same null-space posture task.

| Controller | Cartesian RMSE | Orientation RMSE | RMS torque | Peak torque | Control effort | Settling time (1 mm) | Time per control call |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Task-space PD | 1.19e-03 m | 2.28e-03 rad | 15.1 N m | 37.7 N m | 4148 | 1.99 s | 12.3 us |
| Cartesian impedance | 1.34e-03 m | 2.56e-03 rad | 15.1 N m | 37.7 N m | 4144 | 2.00 s | 11.6 us |
| Operational space | 1.95e-04 m | 2.17e-03 rad | 15.1 N m | 37.4 N m | 4145 | 0.00 s | 11.5 us |

