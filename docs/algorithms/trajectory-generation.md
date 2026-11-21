# Trajectory generation

## Purpose

Point-to-point and multi-point joint trajectories, and Cartesian end-effector trajectories, with
analytic velocities and accelerations. These are the references tracked by the controllers and the
inputs of the [time parameterization](time-parameterization.md) of planned paths.

## Mathematical formulation

### Polynomials (per joint)

Cubic with boundary positions and velocities on $[0, T]$, $h = q_f - q_0$:

$$
q(t) = q_0 + v_0 t + \frac{3h - (2v_0 + v_f)T}{T^2}t^2 + \frac{-2h + (v_0 + v_f)T}{T^3}t^3 .
$$

Quintic with boundary positions, velocities and accelerations (Biagiotti & Melchiorri, eq. 2.5):

$$
\begin{aligned}
a_3 &= \tfrac{20h - (8v_f + 12v_0)T - (3a_0 - a_f)T^2}{2T^3}, \quad
a_4 = \tfrac{-30h + (14v_f + 16v_0)T + (3a_0 - 2a_f)T^2}{2T^4}, \\
a_5 &= \tfrac{12h - 6(v_f + v_0)T + (a_f - a_0)T^2}{2T^5}, \qquad a_0' = q_0,\ a_1 = v_0,\ a_2 = a_0/2 .
\end{aligned}
$$

### Scalar time scalings $s(t)$, rest to rest

* Cubic $s = D(3\tau^2 - 2\tau^3)$ and quintic $s = D(10\tau^3 - 15\tau^4 + 6\tau^5)$, $\tau = t/T$.
* Trapezoidal velocity: with $t_a = v_{max}/a_{max}$, the cruise phase exists iff
  $D \ge v_{max}^2/a_{max}$ and $T = D/v_{max} + t_a$; otherwise the profile is triangular with
  $t_a = \sqrt{D/a_{max}}$. For a prescribed duration the cruise velocity is
  $v = \tfrac12\left(a T - \sqrt{a^2T^2 - 4aD}\right)$.
* Double-S (jerk-limited, 7 segments): constant-jerk phases $(+j, 0, -j, 0, -j, 0, +j)$. With
  $T_j$ the jerk-phase and $T_a$ the acceleration-phase duration, the four cases are closed form:
  1. $v_{max} j_{max} \ge a_{max}^2$: $T_j = a_{max}/j_{max}$, $T_a = T_j + v_{max}/a_{max}$; else
     $T_j = \sqrt{v_{max}/j_{max}}$, $T_a = 2T_j$.
  2. If $T_v = D/v_{max} - T_a < 0$ the cruise velocity is not reached: for
     $D \ge 2a_{max}^3/j_{max}^2$, $T_a = \tfrac{T_j}{2} + \sqrt{T_j^2/4 + D/a_{max}}$; otherwise
     $T_j = \sqrt[3]{D/(2j_{max})}$, $T_a = 2T_j$.

### Synchronized straight-line joint motion

$q(t) = q_0 + s(t)\,(q_1 - q_0)$ with $s \in [0, 1]$ keeps the path on the joint-space line and
starts/stops all joints together. The scalar limits are the tightest joint limits along the line:
$\dot s_{max} = \min_i v_{max,i}/|\Delta_i|$, and similarly for acceleration and jerk.

### Cubic splines

Interpolating $C^2$ cubic splines through $(u_k, x_k)$ solve a tridiagonal system for the knot
second derivatives $M_k$ (natural: $M_0 = M_N = 0$; clamped: prescribed end derivatives), solved with
the Thomas algorithm. Evaluation on $[u_i, u_{i+1}]$, $A = (u_{i+1}-u)/h$, $B = (u-u_i)/h$:

$$
x = A x_i + B x_{i+1} + \big((A^3 - A)M_i + (B^3 - B)M_{i+1}\big)\tfrac{h^2}{6}.
$$

### Cartesian paths

Paths are parameterized by $s \in [0, 1]$; orientation follows the SO(3) geodesic
$R(s) = R_0 \exp(s\,\varphi)$ with $\varphi = \log(R_0^\top R_1)^\vee$, which is equivalent to quaternion
SLERP and has constant world angular rate $R_0\varphi$.

* Line: $p(s) = p_0 + s(p_1 - p_0)$.
* Circular arc of angle $\theta$ about unit axis $a$ through centre $c$:
  $p(s) = c + a_\parallel + \exp(s\theta[a])\, r_\perp$, with tangent $\theta\, a \times r(s)$ and
  centripetal second derivative $-\theta^2 r(s)$.

A `CartesianTrajectory` combines a path with a scalar time scaling:
$V = \tfrac{dP}{ds}\dot s$ and $\dot V = \tfrac{d^2P}{ds^2}\dot s^2 + \tfrac{dP}{ds}\ddot s$ (world-aligned
$[\omega; \dot p]$, matching the geometric Jacobian).

## Assumptions

* Profiles are rest to rest (zero velocity and acceleration at both ends) unless the polynomial API
  is given non-zero boundary conditions.
* Cartesian paths are evaluated independently of the robot; reachability is the caller's concern
  (e.g. IK per sample, or a task-space controller).

## Conventions

Joint-space quantities in rad (m for prismatic joints) and SI time. Cartesian twists and
accelerations are world-aligned and angular-first, see [conventions](../mathematics/conventions.md).

## Algorithm

`stopAtWaypointsTrajectory` chains rest-to-rest synchronized trapezoids between consecutive waypoints
(no blending): the motion stays exactly on the polyline and within limits but stops at each waypoint.
For smooth motion through waypoints use a spline path with
[time-optimal parameterization](time-parameterization.md).

## Implementation

* [profiles.hpp](../../include/manipulatorcore/trajectory/profiles.hpp): `PolynomialProfile`,
  `TrapezoidalProfile`, `DoubleSProfile`.
* [polynomial.hpp](../../include/manipulatorcore/trajectory/polynomial.hpp): `PolynomialTrajectory`,
  `LinearInterpolationTrajectory`, `LineTrajectory`, `CompositeTrajectory`, `stopAtWaypointsTrajectory`.
* [spline.hpp](../../include/manipulatorcore/trajectory/spline.hpp): `CubicSpline`, `CubicSplineTrajectory`.
* [cartesian.hpp](../../include/manipulatorcore/trajectory/cartesian.hpp): `LinePath`, `CircularArcPath`,
  `CartesianTrajectory`.
* [joint_trajectory.hpp](../../include/manipulatorcore/trajectory/joint_trajectory.hpp): the
  `JointTrajectory` interface, `sampleUniform`, `evaluateLimitUsage`.

## Numerical considerations

* Polynomials are evaluated with Horner's scheme for value and both derivatives.
* The double-S profile integrates its constant-jerk phases analytically; the end state matches the
  target distance to round-off and samples are clamped to $[0, D]$.
* Spline systems are strictly diagonally dominant, so the Thomas algorithm needs no pivoting.

## Configuration

Limits are passed per joint (`VecX`) to `LineTrajectory::trapezoidal/doubleS` and per path to
`CartesianTrajectory::trapezoidal(path, v_lin, a_lin, v_ang, a_ang)`.

## Example

```cpp
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"

using namespace mc;
const VecX q0 = VecX::Zero(7), q1 = VecX::Constant(7, 0.5);
const LineTrajectory joint = LineTrajectory::doubleS(q0, q1, VecX::Constant(7, 1.5), VecX::Constant(7, 5.0),
                                                      VecX::Constant(7, 50.0));
const TrajectoryPoint p = joint.sample(0.4);  // p.q, p.qd, p.qdd

const SE3 A = SE3::fromTranslation(Vec3(0.5, 0.0, 0.4));
const SE3 B(Eigen::AngleAxisd(0.5, Vec3::UnitZ()).toRotationMatrix(), Vec3(0.5, 0.3, 0.5));
const auto line = std::make_shared<LinePath>(A, B);
const CartesianTrajectory cart = CartesianTrajectory::trapezoidal(line, 0.5, 1.0, 1.0, 2.0);
const CartesianSample s = cart.sample(0.5 * cart.duration());  // s.pose, s.twist, s.acceleration
```

## Benchmark

See [benchmarks](../benchmarks/README.md).

## Limitations

* No blending (parabolic or circular) between linear segments; use splines plus time-optimal
  parameterization instead.
* Double-S profiles are rest to rest; non-zero boundary velocities are not supported.
* Cartesian paths are limited to lines and circular arcs.

Validated in [test_trajectory.cpp](../../tests/unit/test_trajectory.cpp) (boundary conditions,
finite-difference derivatives, dense limit sampling, all four double-S cases).

## References

* L. Biagiotti and C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer, 2008 (ch. 2-3).
* B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009 (ch. 4).
* K. Shoemake, "Animating rotation with quaternion curves", SIGGRAPH 1985 (SLERP).
* K. M. Lynch and F. C. Park, *Modern Robotics*, Cambridge University Press, 2017 (ch. 9).
