# Jacobians and singularities

## Purpose

This page covers the velocity kinematics of serial chains: the space, body and geometric Jacobians,
Jacobians of points on intermediate bodies, the acceleration bias $\dot J_g \dot q$, SVD-based rank
and conditioning analysis, Yoshikawa's manipulability, differential inverse kinematics, and
workspace sampling. Inverse kinematics, task-space control, the operational-space formulation and
the simulator's contact model all use these functions.

## Mathematical formulation

### Three Jacobians

Let $\hat a_i$ be the world-frame axis of joint $i$, $p_i$ its world-frame origin (on the axis for
revolute joints) and $p$ the end-effector origin. Rows are ordered $(\omega, v)$.

| Jacobian | Definition | Column $i$, revolute | Column $i$, prismatic |
| --- | --- | --- | --- |
| Space $J_s$ | $\mathcal{V}_s = J_s\dot q$, $[\mathcal{V}_s] = \dot T T^{-1}$ | $(\hat a_i,\ -\hat a_i\times p_i)$ | $(0,\ \hat a_i)$ |
| Body $J_b$ | $\mathcal{V}_b = J_b\dot q$, $[\mathcal{V}_b] = T^{-1}\dot T$ | $J_b = \mathrm{Ad}_{T_{se}^{-1}}J_s$ | same |
| Geometric $J_g$ | $(\omega_s, \dot p) = J_g\dot q$ | $(\hat a_i,\ \hat a_i\times(p - p_i))$ | $(0,\ \hat a_i)$ |

The space-twist linear part is the velocity of the body-fixed point that is momentarily at the base
origin. The geometric Jacobian instead uses the velocity of the end-effector origin, written in the
base frame. The two are related by

```math
J_g = \begin{bmatrix} I & 0 \\ -[p] & I \end{bmatrix} J_s .
```

$J_g$ is the classical geometric Jacobian of Siciliano et al. and Spong et al. with the angular rows
first. Those texts stack the linear rows first.

The PoE Jacobians are evaluated from the screw axes (see [forward kinematics](forward-kinematics.md)):

```math
J_{s,i} = \mathrm{Ad}_{e^{[S_1]q_1}\cdots e^{[S_{i-1}]q_{i-1}}} S_i, \qquad
J_{b,i} = \mathrm{Ad}_{e^{-[B_n]q_n}\cdots e^{-[B_{i+1}]q_{i+1}}} B_i .
```

### Point Jacobian

For a point $r$ fixed to body $k$ (world position $r_w = T_{s,k}\,r$), `pointJacobian` returns
$(\omega_k, \dot r_w) = J\dot q$. Its columns $i \le k$ have the geometric form with $p$ replaced by
$r_w$. Columns $i > k$ are zero.

### Acceleration bias $\dot J_g\dot q$

`geometricJacobianDotTimesVelocity` computes the bias analytically, by differentiating the columns
of $J_g$ along $\dot q$. With $\omega_{i-1} = \sum_{k<i,\ \text{revolute}} \hat a_k\dot q_k$ the angular
velocity of the body that carries joint $i$'s axis, $\dot{\hat a}_i = \omega_{i-1}\times\hat a_i$, and
$\dot p_i$, $\dot p$ the velocities of the joint origin and of the end-effector origin:

```math
\dot J_g\dot q = \sum_{i\ \text{revolute}} \begin{bmatrix} \dot{\hat a}_i \\ \dot{\hat a}_i\times(p - p_i) + \hat a_i\times(\dot p - \dot p_i) \end{bmatrix}\dot q_i
 + \sum_{i\ \text{prismatic}} \begin{bmatrix} 0 \\ \dot{\hat a}_i \end{bmatrix}\dot q_i .
```

This is the end-effector acceleration $(\dot\omega, \ddot p)$ produced by $\dot q$ when $\ddot q = 0$, as
needed for task-space inverse dynamics: $(\dot\omega, \ddot p) = J_g\ddot q + \dot J_g\dot q$.

### SVD metrics

With singular values $\sigma_1 \ge \dots \ge \sigma_r$ ($r = \min(m, n)$) of an $m\times n$ Jacobian:

- default rank tolerance $\tau = \sigma_1\max(m, n)\,\varepsilon_\text{mach}$, overridable per call;
  rank $= \#\{\sigma_i > \tau\}$; `isSingular()` is true when rank $< r$;
- condition number $\kappa = \sigma_1/\sigma_r$ ($+\infty$ if $\sigma_r = 0$); inverse condition
  number $\sigma_r/\sigma_1 \in [0, 1]$ (0 if $\sigma_1 = 0$);
- Yoshikawa manipulability $w = \prod_{i=1}^{r}\sigma_i$. For $m \le n$ this equals $\sqrt{\det(JJ^\top)}$,
  the volume of the velocity ellipsoid $\{J\dot q : \lVert\dot q\rVert \le 1\}$ up to a constant.

`analyzeSingularity(model, q)` analyzes the full $J_g$ and its rotational (rows 0–2) and
translational (rows 3–5) blocks separately.

### Differential inverse kinematics

For a task velocity $\dot x$:

| Solver | Joint velocity | Notes |
| --- | --- | --- |
| `Transpose` | $J^\top\dot x$ | direction only, not an inverse |
| `PseudoInverse` | $J^+\dot x$ from the SVD, $\sigma_i \le \tau$ treated as zero | `damping` is used as $\tau$ (negative: default) |
| `DampedLeastSquares` | $J^\top(JJ^\top + \lambda^2 I)^{-1}\dot x$ | minimizes $\lVert J\dot q - \dot x\rVert^2 + \lambda^2\lVert\dot q\rVert^2$ |

The adaptive damping of Nakamura & Hanafusa and Chiaverini is

```math
\lambda = \begin{cases} \lambda_\text{max}\sqrt{1 - (\sigma_\text{min}/\epsilon)^2} & \sigma_\text{min} < \epsilon \\ 0 & \text{otherwise.} \end{cases}
```

`jointVelocityWithNullSpace` returns $J^\#\dot x + (I - J^\# J)\dot q_0$, where $J^\#$ is the damped
pseudoinverse with gains $\sigma_i/(\sigma_i^2 + \lambda^2)$ (Moore–Penrose for $\lambda = 0$).

## Assumptions

- Joint axes are unit vectors (enforced by the model). Jacobians are evaluated at the configuration
  stored in the `KinematicState` passed in, or at the given $q$.
- Manipulability values are compared only between configurations of the same robot, with the same
  row selection and units.

## Conventions

See [conventions](../mathematics/conventions.md). All Jacobians are $6\times n$ with angular rows
first. $J_s$ and $J_b$ map to Modern Robotics space and body twists. $J_g$ maps to
$(\omega_s, \dot p_{ee})$ and is the Jacobian paired with the world-aligned pose error
$e_w = (\log(R_dR^\top)^\vee,\ p_d - p)$. `planarTaskJacobian` returns rows $(v_x, v_y, \omega_z)$ of $J_g$
in that order (rows 3, 4, 2), the usual task ordering for planar arms. An analytic (Euler-rate)
Jacobian is intentionally not provided.

## Algorithm

```text
spaceJacobian(model, state):          # O(n) after forward kinematics
  for i in 0..n-1:
     a <- R_base_body[i] * axis_i;  p_i <- origin of body i
     J[:, i] <- revolute ? (a, p_i x a) : (0, a)

geometricJacobianDotTimesVelocity(q, qd):
  forward kinematics; precompute world axes a_k and origins p_k
  v_ee <- velocity of the tool origin;  omega <- 0
  for i in 0..n-1:
     a_dot <- omega x a_i
     if revolute: pdot_i <- velocity of p_i due to joints 0..i-1
                  bias.ang += a_dot qd_i
                  bias.lin += (a_dot x (p_ee - p_i) + a_i x (v_ee - pdot_i)) qd_i
                  omega += a_i qd_i
     else:        bias.lin += a_dot qd_i
```

The bias routine computes point velocities by direct summation, so its cost is $O(n^2)$, which is
negligible for $n \le 7$.

## Implementation

Headers: [jacobian.hpp](../../include/manipulatorcore/kinematics/jacobian.hpp),
[poe.hpp](../../include/manipulatorcore/kinematics/poe.hpp),
[singularity.hpp](../../include/manipulatorcore/kinematics/singularity.hpp),
[differential_kinematics.hpp](../../include/manipulatorcore/kinematics/differential_kinematics.hpp),
[workspace.hpp](../../include/manipulatorcore/kinematics/workspace.hpp),
[linalg.hpp](../../include/manipulatorcore/math/linalg.hpp).
Sources: [jacobian.cpp](../../src/kinematics/jacobian.cpp), [poe.cpp](../../src/kinematics/poe.cpp),
[singularity.cpp](../../src/kinematics/singularity.cpp),
[differential_kinematics.cpp](../../src/kinematics/differential_kinematics.cpp),
[workspace.cpp](../../src/kinematics/workspace.cpp), [linalg.cpp](../../src/math/linalg.cpp).

```cpp
Mat6X spaceJacobian(const RobotModel& model, const KinematicState& state);      // also (model, q)
Mat6X bodyJacobian(const RobotModel& model, const KinematicState& state);       // also (model, q)
Mat6X geometricJacobian(const RobotModel& model, const KinematicState& state);  // also (model, q)
Mat6X pointJacobian(const RobotModel& model, const KinematicState& state, int body, const Vec3& point_in_body);
Vec6  geometricJacobianDotTimesVelocity(const RobotModel& model, const VecX& q, const VecX& qd);
MatX  planarTaskJacobian(const Mat6X& geometric_jacobian);
Mat6X poeSpaceJacobian(const ProductOfExponentials& poe, const VecX& q);
Mat6X poeBodyJacobian(const ProductOfExponentials& poe, const VecX& q);

JacobianAnalysis analyzeJacobian(const MatX& J, double tolerance = -1.0);
double yoshikawaManipulability(const MatX& J);
SingularityReport analyzeSingularity(const RobotModel& model, const VecX& q);

VecX jointVelocityForTask(const MatX& J, const VecX& task_velocity, DifferentialSolver solver,
                          double damping = 0.05);
double adaptiveDamping(double sigma_min, double epsilon, double lambda_max);
VecX jointVelocityWithNullSpace(const MatX& J, const VecX& task_velocity, const VecX& qd_null,
                                double damping = 0.0);

std::vector<WorkspaceSample> sampleWorkspace(const RobotModel& model, const WorkspaceOptions& options = {});
WorkspaceBounds workspaceBounds(const std::vector<WorkspaceSample>& samples);
```

### Validation

- [test_kinematics.cpp](../../tests/unit/test_kinematics.cpp), parameterized over the four
  built-in robots and a mixed revolute/prismatic chain (`JacobianFiniteDifference/*`), 10 random
  configurations each, central differences with $h = 10^{-6}$ and tolerance $10^{-7}$:
  - `SpaceJacobian` against $(\partial T/\partial q_i\,T^{-1})^\vee$; `BodyJacobian` against
    $(T^{-1}\partial T/\partial q_i)^\vee$; `GeometricJacobian` against $((\partial R/\partial q_i) R^\top)^\vee$
    and $\partial p/\partial q_i$;
  - `PoEJacobiansMatchChain` (within $10^{-12}$), `GeometricFromSpaceJacobian` (the $[I\ 0; -[p]\ I]$
    relation within $10^{-13}$), `PointJacobian` (against `numericalJacobian` of the point position);
  - `JacobianDotTimesVelocity`: against $\big(J_g(q + h\dot q) - J_g(q - h\dot q)\big)\dot q/(2h)$, tolerance $10^{-6}$.
- `Jacobian.PlanarTaskJacobianRows`; `Singularity.Planar2RManipulabilityClosedForm`
  ($w = \ell_1\ell_2\lvert\sin q_2\rvert$ for the $xy$ rows); `Singularity.Planar2RStretchedIsSingular`
  (rank 1, $w = 0$); `Singularity.Industrial6RWristSingularity` (rank drops from 6 to 5 with
  $\sigma_\text{min} < 10^{-12}$ when $q_5 = 0$ aligns joints 4 and 6); `Singularity.Redundant7RVerticalIsSingular`.
- `Workspace.Planar2RSamplesLieInAnnulus` (all samples in $0.2 \le r \le 1.8$ m),
  `Workspace.DeterministicForSeed`.
- [test_linalg.cpp](../../tests/unit/test_linalg.cpp): `Linalg.PseudoInverseSatisfiesPenroseConditions`,
  `Linalg.PseudoInverseOfRankDeficientMatrix`, `Linalg.MinimumNormSolutionMatchesPseudoInverse`,
  `Linalg.DampedLeastSquaresMatchesNormalEquations`, `Linalg.DampedLeastSquaresIsBoundedAtSingularity`,
  `Linalg.NullSpaceProjector`, `Linalg.SvdMetricsConditionNumber`, `Linalg.NumericalJacobianOfLinearMap`,
  `Linalg.RejectsNonFiniteInput`.

## Numerical considerations

- Rank decisions, pseudoinverses and manipulability use a Jacobi SVD (`Eigen::JacobiSVD`). Singular
  values are returned in descending order.
- The default tolerance $\sigma_1\max(m,n)\varepsilon$ detects only numerically exact rank loss. It
  is the right test for "is this matrix rank-deficient", but not for "is this configuration close
  to a singularity". For the latter, threshold $\sigma_\text{min}$ or the inverse condition number
  with an application-specific value, or pass an explicit tolerance.
- `solveDampedLeastSquares` forms $AA^\top + \lambda^2 I$ (or $A^\top A + \lambda^2 I$ when $A$ has more
  rows than columns) and solves it by Cholesky (LLT). If LLT fails, which is possible only for
  extremely small $\lambda$ on a singular matrix, it falls back to the SVD form
  $\sum_i \sigma_i/(\sigma_i^2 + \lambda^2)\,v_i u_i^\top b$. $\lambda = 0$ selects the minimum-norm SVD
  solution. The DLS step is bounded near singularities: $\lVert\dot q\rVert \le \lVert\dot x\rVert/(2\lambda)$.
- With $\lambda > 0$, $I - J^\#J$ is not an exact projector, and the secondary velocity leaks into
  the task. The header states the exact-projection property only for full row rank and $\lambda = 0$.
- `numericalJacobian` uses central differences (default step $10^{-6}$). The truncation error is
  $O(h^2)$ and the round-off error $O(\varepsilon/h)$.

## Configuration

`WorkspaceOptions`:

| Field | Default | Meaning |
| --- | --- | --- |
| `samples` | 20000 | number of uniform samples of the joint-limit box |
| `seed` | 7 | seed of `mc::Random` (bit-reproducible) |
| `manipulability_rows` | `{3, 4, 5}` | rows of $J_g$ used for the manipulability index (translational block); planar arms should use `{3, 4}` |

Function parameters: `analyzeJacobian(J, tolerance = -1)` (negative: default tolerance);
`jointVelocityForTask(..., damping = 0.05)` ($\lambda$ for DLS, $\tau$ for the pseudoinverse);
`jointVelocityWithNullSpace(..., damping = 0.0)`.

## Example

```cpp
#include "manipulatorcore/kinematics/differential_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

int main() {
  using namespace mc;
  const RobotModel arm = makeIndustrial6R();
  VecX q = industrial6RHome();
  q(4) = 1e-3;  // close to the wrist singularity (joints 4 and 6 nearly aligned)

  const KinematicState state = forwardKinematics(arm, q);
  const Mat6X Jg = geometricJacobian(arm, state);
  const JacobianAnalysis analysis = analyzeJacobian(Jg);

  // Desired end-effector motion: 5 cm/s along base x, no rotation ([omega; pdot]).
  Vec6 xdot = Vec6::Zero();
  xdot(3) = 0.05;
  const double lambda = adaptiveDamping(analysis.sigma_min, 0.05, 0.1);
  const VecX qd = jointVelocityForTask(Jg, xdot, DifferentialSolver::DampedLeastSquares, lambda);

  const double w_translational = analyzeSingularity(arm, q).translational.manipulability;
  return (qd.allFinite() && analysis.inverseConditionNumber() < 0.1 && w_translational > 0.0) ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- Yoshikawa's index depends on units and scale. Mixing angular (rad/s) and linear (m/s) rows makes
  its value depend on the length unit. It scales with link lengths and changes with the number of
  rows. Values are not comparable across robots or row selections, and only the zero crossing has
  an invariant meaning. For a Jacobian with more rows than columns (for example the full $6\times 2$
  $J_g$ of a planar arm), the product of singular values is $\sqrt{\det(J^\top J)}$, not
  $\sqrt{\det(JJ^\top)} = 0$. Select the task rows first.
- The condition number is scale-invariant only for a fixed Jacobian. With mixed rows it still
  depends on the length unit.
- No analytic Jacobian, Jacobian derivatives other than $\dot J_g\dot q$, or task-priority
  (multi-level) inverse kinematics are provided.
- `sampleWorkspace` samples the joint box uniformly. The resulting Cartesian density is not uniform,
  so the bounds are an inner estimate of the reachable set. Joint limits are respected, collisions
  are not.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 5 (space and body Jacobians, singularities, manipulability).
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 3 (geometric Jacobian, kinematic singularities, redundancy).
- M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006 (2nd ed. 2020). Velocity kinematics.
- T. Yoshikawa, "Manipulability of robotic mechanisms," *The International Journal of Robotics Research*, 4(2), 1985.
- Y. Nakamura and H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control," *ASME Journal of Dynamic Systems, Measurement, and Control*, 108(3), 1986.
- C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods," *IEEE Transactions on Systems, Man, and Cybernetics*, 16(1), 1986.
- S. Chiaverini, "Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators," *IEEE Transactions on Robotics and Automation*, 13(3), 1997.
