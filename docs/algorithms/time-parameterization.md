# Time-optimal path parameterization

## Purpose

Given a geometric joint path $q(s)$, $s \in [0, L]$ (for example a smoothed planner output), find the
fastest rest-to-rest timing $s(t)$ that respects joint velocity and acceleration limits.

## Mathematical formulation

Along the path $\dot q = q'(s)\dot s$ and $\ddot q = q'(s)\ddot s + q''(s)\dot s^2$. With
$x = \dot s^2$ and $u = \ddot s$ (so $dx/ds = 2u$) the box limits become **linear** in $(u, x)$:

$$
\left|q_i'(s)\,u + q_i''(s)\,x\right| \le a_{max,i}, \qquad q_i'(s)^2\, x \le v_{max,i}^2, \qquad x \ge 0 .
$$

The path is discretized on a grid $s_0 = 0 < \dots < s_N = L$ with constant $u_i$ on each segment,
$x_{i+1} = x_i + 2\Delta_i u_i$. Following the reachability analysis of Pham & Pham (TOPP-RA):

1. **Backward pass** — controllable sets $K_i = [0, \bar x_i]$: $K_N = \{0\}$ (stop at the end) and
   $\bar x_i$ is the largest $x$ for which some $u$ satisfies the segment constraints and
   $x + 2\Delta_i u \in K_{i+1}$.
2. **Forward pass** — from $x_0 = 0$, choose the largest admissible $u_i$ that keeps
   $x_{i+1} \in K_{i+1}$.
3. **Timing** — $\Delta t_i = 2\Delta_i / (\sqrt{x_i} + \sqrt{x_{i+1}})$ (exact for constant $\ddot s$).

For box joint limits each step is a two-variable linear program. ManipulatorCore solves it in closed
form: every constraint $a\,u + b\,x \le c$ with $a \ne 0$ bounds $u$ from one side by an affine function
of $x$, and a feasible $u$ exists iff every lower bound lies below every upper bound — a set of
one-dimensional inequalities in $x$ whose intersection is an interval.

## Assumptions

* The path is $C^1$ with bounded second derivative between grid points (e.g. a cubic spline). A
  piecewise-linear path has unbounded $q''$ at corners and must stop there; use
  `stopAtWaypointsTrajectory` for polylines.
* Only kinematic box constraints (joint velocity and acceleration). Torque limits are not modelled.

## Conventions

Path parameter $s$ in the units of the path (chord length in joint space for `SplineJointPath`),
joint limits per joint as in `RobotModel::velocityLimits()/accelerationLimits()`.

## Algorithm

* Constraints are imposed at **both ends** of every grid segment with the same $u$, so a segment's
  accelerations at $s_i$ and $s_{i+1}$ are both admissible.
* The grid is uniform but **refined at path breakpoints** (spline knots): every knot is a grid point
  and every knot interval holds at least `segments_per_breakpoint_interval` segments. Second
  derivatives can vary quickly near knots that follow sharp polyline corners.
* Limits are tightened by `safety_margin` (1 % by default) to absorb discretization error.
* **Verification**: the solution is sampled inside every segment against the true limits. If a peak
  still exceeds a limit (ratio $r_v$ for velocity, $r_a$ for acceleration), the whole trajectory is
  slowed uniformly by $k = \max(1, r_v, \sqrt{r_a})$, since a time scaling $t \to t/k$ divides
  velocities by $k$ and accelerations by $k^2$. The returned trajectory therefore satisfies the
  sampled limits by construction, and `timeScale()` reports any slow-down applied.

## Implementation

[time_parameterization.hpp](../../include/manipulatorcore/trajectory/time_parameterization.hpp) /
[time_parameterization.cpp](../../src/trajectory/time_parameterization.cpp):
`JointPath` (with `SplineJointPath`, `StraightJointPath`), `timeOptimalParameterization(path, vmax, amax, options)`
returning a `TimeOptimalTrajectory` (a `JointTrajectory`).

## Numerical considerations

* The backward pass fails with `InvalidArgument` only if the discretized problem is infeasible
  from rest, which cannot happen for consistent positive limits.
* Rounding can make the forward-pass interval marginally empty; the lower bound is then used, which
  keeps $x_{i+1}$ inside the controllable set.
* Complexity is $O(N n^2)$ for $N$ grid segments and $n$ joints (pairwise bound comparison).

## Configuration

`TimeParameterizationOptions`: `grid_segments` (1000), `segments_per_breakpoint_interval` (6),
`max_grid_segments` (20000), `velocity_scale` and `acceleration_scale` (1.0), `safety_margin`
(0.01), `verification_samples_per_segment` (4).

## Example

```cpp
#include "manipulatorcore/trajectory/time_parameterization.hpp"

using namespace mc;
std::vector<VecX> waypoints = /* collision-free path from a planner */;
auto path = std::make_shared<SplineJointPath>(waypoints);
const TimeOptimalTrajectory traj =
    timeOptimalParameterization(path, model.velocityLimits(), model.accelerationLimits());
const LimitUsage usage = evaluateLimitUsage(traj, model.velocityLimits(), model.accelerationLimits(), 1e-3);
```

## Benchmark

See [benchmarks](../benchmarks/README.md).

## Limitations

* Box joint limits only (no torque, jerk or Cartesian constraints).
* Rest-to-rest; non-zero boundary velocities are not supported.
* A spline through planner waypoints may leave the collision-free tube between waypoints; the
  planning module therefore re-validates the spline and refines it before timing
  (see [motion planning](motion-planning.md)).

Validated in [test_trajectory.cpp](../../tests/unit/test_trajectory.cpp): straight paths reproduce
the minimum-time trapezoid within 1 %, and spline paths respect the limits under dense sampling while
using more than 90 % of at least one limit.

## References

* H. Pham and Q.-C. Pham, "A new approach to time-optimal path parameterization based on reachability analysis", *IEEE Transactions on Robotics*, 34(3), 2018.
* J. E. Bobrow, S. Dubowsky and J. S. Gibson, "Time-optimal control of robotic manipulators along specified paths", *International Journal of Robotics Research*, 4(3), 1985.
* K. G. Shin and N. D. McKay, "Minimum-time control of robotic manipulators with geometric path constraints", *IEEE Transactions on Automatic Control*, 30(6), 1985.
