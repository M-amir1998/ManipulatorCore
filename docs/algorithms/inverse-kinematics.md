# Inverse kinematics

## Purpose

Inverse kinematics (IK) finds joint configurations whose end-effector pose equals a target pose.
ManipulatorCore provides closed-form solutions for planar 2R and 3R arms and an iterative solver for
arbitrary serial chains. The iterative solver offers four update rules, task weighting, hard
joint-limit enforcement, null-space secondary objectives for redundant arms, and seeded multi-start.
The pose-goal planning pipeline uses multi-start IK to generate goal configurations (see
[motion planning](motion-planning.md)).

## Mathematical formulation

### Analytical planar IK

For a 2R arm with link lengths $\ell_1, \ell_2$ and target $(x, y)$, $r^2 = x^2 + y^2$:

```math
c_2 = \frac{r^2 - \ell_1^2 - \ell_2^2}{2\ell_1\ell_2}, \qquad
s_2 = \pm\sqrt{1 - c_2^2}, \qquad
q_2 = \mathrm{atan2}(s_2, c_2), \qquad
q_1 = \mathrm{atan2}(y, x) - \mathrm{atan2}(\ell_2 s_2,\ \ell_1 + \ell_2 c_2).
```

There are two solutions inside the annulus $\lvert\ell_1 - \ell_2\rvert < r < \ell_1 + \ell_2$, returned
with $q_2 > 0$ first. On the boundary there is one solution, and outside the annulus none. Angles are
wrapped to $(-\pi, \pi]$. A 3R arm with tool orientation $\phi = q_1 + q_2 + q_3$ is reduced to a 2R
problem for the wrist point $w = (x, y) - \ell_3(\cos\phi, \sin\phi)$, and then
$q_3 = \mathrm{wrap}(\phi - q_1 - q_2)$.

### Pose error and task weighting

The numerical solver drives the world-aligned error to zero:

$$
e_w(q) = \big(\log(R_d R(q)^\top)^\vee,\ p_d - p(q)\big) \in \mathbb{R}^6,
$$

paired with the geometric Jacobian, $\dot e_w \approx -J_g\dot q$ near the solution. Task weights
$w_r \ge 0$ select and scale rows. Rows with $w_r = 0$ are removed, and the remaining rows of $e_w$
and $J_g$ are multiplied by $\sqrt{w_r}$, giving $\tilde e$ and $\tilde J$. The body-frame error
$e_b = \log(T^{-1}T_d)^\vee$ (paired with $J_b$) is available as `poseErrorBody` but is not used by the
solver.

### Update rules

| Method | Step $\Delta q$ | Source |
| --- | --- | --- |
| `JacobianTranspose` | $\alpha\tilde J^\top\tilde e$, $\alpha = \dfrac{\langle\tilde e, \tilde J\tilde J^\top\tilde e\rangle}{\lVert\tilde J\tilde J^\top\tilde e\rVert^2}$ | Buss 2004 (step length minimizing $\lVert\tilde e - \alpha\tilde J\tilde J^\top\tilde e\rVert$) |
| `PseudoInverse` | $\tilde J^+\tilde e$ (Gauss–Newton, SVD with tolerance `pinv_tolerance`) | |
| `DampedLeastSquares` | $\tilde J^\top(\tilde J\tilde J^\top + \lambda^2 I)^{-1}\tilde e$ | Wampler 1986; Nakamura & Hanafusa 1986 |
| `LevenbergMarquardt` | $(\tilde J^\top\tilde J + \mu I)^{-1}\tilde J^\top\tilde e$, $\mu = \tfrac{1}{2}\lVert\tilde e\rVert^2 + w$ | Sugihara 2011 |

For DLS, $\lambda$ = `damping`, or with `adaptive_damping`
$\lambda = \lambda_\text{max}\sqrt{1 - (\sigma_\text{min}/\epsilon)^2}$ for $\sigma_\text{min} < \epsilon$
and 0 otherwise ($\lambda_\text{max}$ = `damping`, $\epsilon$ = `singular_region`). When the
resulting $\lambda$ is 0, the pseudoinverse step is used. In Levenberg–Marquardt the damping shrinks
as the error decreases, so the method moves from gradient-like steps far from the solution to
Gauss–Newton steps close to it. $w$ = `lm_bias` keeps the system positive definite at a singular
solution, and $\mu$ is bounded below by $10^{-12}$.

### Null-space objectives

For a secondary cost $H(q)$ the solver adds $P\,z$ with $z = -k\nabla H(q)$ and
$P = I - \tilde J_\text{free}^+\tilde J_\text{free}$, the Moore–Penrose null-space projector of the
current (weighted, free-column) task Jacobian. To first order this term does not change the task
error and does not increase $H$.

| Objective | $H(q)$ | Gradient |
| --- | --- | --- |
| `JointCentering` | $\tfrac{1}{2}\sum_i \big((q_i - c_i)/\rho_i\big)^2$, $\rho_i = u_i - l_i$, $c$ = `null_space_center` or mid-range | $(q_i - c_i)/\rho_i^2$ |
| `JointLimitBarrier` (Zghal et al.) | $\tfrac{1}{n}\sum_i \rho_i^2/\big(4(u_i - q_i)(q_i - l_i)\big)$, minimum 1 at mid-range | $\tfrac{1}{n}\rho_i^2(2q_i - u_i - l_i)/\big(4(u_i - q_i)^2(q_i - l_i)^2\big)$ |
| `JointLimitActivation` | $\tfrac{1}{2}\sum_i (\delta_i/d_i)^2$, $\delta_i$ = penetration into the band $d_i = 0.1\rho_i$ at either limit, 0 inside | $\delta_i/d_i^2$ (C¹) |
| `Manipulability` | $-w(q)$, Yoshikawa index of the full $J_g$ | central differences, $h = 10^{-6}$ |

## Assumptions

- The target pose is finite. Reachability is not checked in advance. An unreachable target ends
  with `MaxIterations` or `Stalled` and the closest configuration found.
- The seed is a valid configuration. With `enforce_joint_limits`, it is first clamped into the limits.
- The iterative methods are local. They converge to a solution near the seed if one exists, and
  global coverage requires multi-start.

## Conventions

See [conventions](../mathematics/conventions.md). The error is $e_w$, paired with $J_g$. Task
weights are ordered $(\omega_x, \omega_y, \omega_z, v_x, v_y, v_z)$: `{0,0,0,1,1,1}` solves for position
only, and planar arms use `{0,0,1,1,1,0}` (planar orientation and position). Euler angles are never
subtracted. The orientation tolerance applies to the norm of the selected rotational rows of $e_w$
(the geodesic angle when all three are selected), and the position tolerance to the norm of the
selected translational rows. Neither norm is weighted.

## Algorithm

```text
solveIK(model, T_d, seed, options):
  q <- clamp(seed) if enforce_joint_limits else seed
  e <- weighted error at q;  refinement_left <- null_space_refinement_iterations (if an objective is set)
  for it in 0..max_iterations-1:
     if converged(e):                                 # |e_pos| <= tol_p and |e_rot| <= tol_r
        if refinement_left == 0: return Converged
        refinement_left -= 1                          # keep iterating: task held, objective minimized
     J <- weighted rows of J_g(q);  z <- -k grad H(q)  (if an objective is set)
     locked <- {};  dq_locked <- 0
     repeat up to n+1 passes:                         # active-set joint-limit clamping
        J_free <- J with locked columns zeroed
        dq <- step(J_free, e - J dq_locked)          # one of the four update rules
        dq += (I - J_free^+ J_free) z_free            # z with locked entries zeroed
        dq *= step_size;  scale dq so that max|dq_i| <= max_step
        dq[locked] <- dq_locked[locked]
        for each free joint whose q_i + dq_i leaves [l_i, u_i]:
           lock it;  dq_locked_i <- clamp(q_i + dq_i) - q_i
        stop when no new joint was locked (or limits are not enforced)
     q_next <- clamp(q + dq);  moved <- max|q_next - q|;  q <- q_next;  e <- error at q
     if moved < stall_tolerance: return Converged if converged(e) else Stalled
  return Converged if converged(e) else MaxIterations

solveIKMultiStart(model, T_d, seed, options):
  for attempt in 0..attempts-1:
     start <- seed if attempt == 0 and use_initial_guess, else uniform sample of the joint box
     r <- solveIK(model, T_d, start, options.ik)
     keep r if converged, within limits, accepted by validator, and
        max|r.q - s.q| >= distinct_threshold for every kept s;  stop after max_solutions
  return kept solutions, stable-sorted by |q - seed|
```

Locked joints keep their clamping motion, and the remaining joints compensate for the error that
motion leaves. A locked joint stays locked for the rest of the iteration and is released at the
next iteration, when the active set is rebuilt from scratch.

## Implementation

Headers: [inverse_kinematics.hpp](../../include/manipulatorcore/kinematics/inverse_kinematics.hpp),
[analytical_ik.hpp](../../include/manipulatorcore/kinematics/analytical_ik.hpp),
[objectives.hpp](../../include/manipulatorcore/optimization/objectives.hpp).
Sources: [inverse_kinematics.cpp](../../src/kinematics/inverse_kinematics.cpp),
[analytical_ik.cpp](../../src/kinematics/analytical_ik.cpp),
[objectives.cpp](../../src/optimization/objectives.cpp), [linalg.cpp](../../src/math/linalg.cpp).

```cpp
std::vector<Vec2> planar2RInverseKinematics(double l1, double l2, const Vec2& target,
                                            double boundary_tolerance = 1e-12);
std::vector<Vec3> planar3RInverseKinematics(double l1, double l2, double l3, const Vec2& target,
                                            double phi, double boundary_tolerance = 1e-12);
std::vector<double> planarLinkLengths(const RobotModel& model);
std::vector<VecX> analyticalIKPlanar2R(const RobotModel& model, const Vec2& target, bool respect_limits = true);
std::vector<VecX> analyticalIKPlanar3R(const RobotModel& model, const Vec2& target, double phi,
                                       bool respect_limits = true);

Vec6 poseErrorWorld(const SE3& current, const SE3& desired);
Vec6 poseErrorBody(const SE3& current, const SE3& desired);
IKResult solveIK(const RobotModel& model, const SE3& target, const VecX& seed, const IKOptions& options = {});
std::vector<IKResult> solveIKMultiStart(const RobotModel& model, const SE3& target, const VecX& seed,
                                        const MultiStartIKOptions& options = {});

ObjectiveValue evaluateObjective(NullSpaceObjective objective, const RobotModel& model, const VecX& q);
```

`IKResult` carries `q`, `status` (`Converged`, `MaxIterations`, `Stalled`), `iterations`, the final
unweighted `position_error` and `orientation_error` over the selected rows, and `within_limits`.
Failure to converge is reported through the status, not by an exception. Invalid options, a seed
of the wrong size or a non-finite target throw `InvalidArgument`.

### Validation

[test_inverse_kinematics.cpp](../../tests/unit/test_inverse_kinematics.cpp):

- `AnalyticalIK.Planar2RTwoSolutionsReachTarget` (200 targets from FK; every solution reproduces the
  target within $10^{-12}$; ordering $q_2 > 0$ first), `AnalyticalIK.Planar2RUnreachableAndBoundary`,
  `AnalyticalIK.Planar3RReachesPositionAndOrientation`, `AnalyticalIK.RejectsNonPlanarModels`.
- `Methods/NumericalIK.ConvergesOnReachableTargetsFromNearbySeeds`, instantiated for all four
  methods (JT with 3000 iterations, the others with 200), on Industrial6R and Redundant7R with seeds
  perturbed by up to 0.25 rad. At least 80 % of 40 trials converge, every converged result meets
  both tolerances measured independently with `poseDistance`, and every result stays within limits.
- `NumericalIK.UnreachableTargetReportsFailureWithoutNaN`, `NumericalIK.DampedLeastSquaresHandlesSingularStart`
  (adaptive DLS from the singular stretched configuration of the 7R arm),
  `NumericalIK.RespectsJointLimitsNearTheBoundary`, `NumericalIK.PositionOnlyTaskForPlanarArm`,
  `NumericalIK.InvalidOptionsThrow`.
- `NumericalIK.NullSpaceRefinementKeepsTaskAndImprovesPosture`: starting from an exact solution,
  refinement lowers the joint-centering cost while keeping both tolerances, and the projected
  gradient on the 7R self-motion drops below 5 % of its initial value.
  `NumericalIK.NullSpaceStepIsFirstOrderTaskPreserving`: $J P z = 0$ within $10^{-12}$ and $z^\top Pz \ge 0$.
- `MultiStartIK.FindsDistinctValidSolutions` (at least two distinct branches of the 6R arm, sorted
  by distance to the seed), `MultiStartIK.ValidatorFiltersSolutions`.
- `ObjectiveGradients.MatchFiniteDifferences` (all objectives), `Objectives.ActivationPenaltyIsZeroInInterior`,
  `Objectives.BarrierIsFiniteAtTheLimits`; `DifferentialKinematics.AdaptiveDampingIsContinuous`.

## Numerical considerations

- No inverse is formed. The pseudoinverse step uses the SVD-based minimum-norm solve, DLS a
  Cholesky solve of $\tilde J\tilde J^\top + \lambda^2 I$ (with SVD fallback), and LM a Cholesky solve
  of $\tilde J^\top\tilde J + \mu I$ (see [Jacobians and singularities](jacobians-and-singularities.md)).
- The Jacobian-transpose step length is set to zero when $\lVert\tilde J\tilde J^\top\tilde e\rVert^2 \le 10^{-300}$.
- The step limit scales the whole step uniformly (`max_step` on the infinity norm), so the step
  direction is preserved.
- The pure pseudoinverse step is undamped. Near singularities its steps are limited only by
  `max_step`. Use DLS (preferably adaptive) or LM when the path to the solution may cross singular
  configurations.
- The barrier objective floors the limit distances at $10^{-9}\rho_i$, so it stays finite at the
  boundary. Its gradient is still very large there.
- Convergence is declared on the unweighted norms of the selected rows, not on the weighted error
  the step minimizes.

## Configuration

`IKOptions`:

| Field | Default | Meaning |
| --- | --- | --- |
| `method` | `DampedLeastSquares` | update rule |
| `max_iterations` | 200 | iteration budget |
| `position_tolerance` | `1e-4` | [m] |
| `orientation_tolerance` | `1e-3` | [rad] |
| `step_size` | 1.0 | scale applied to each step |
| `max_step` | 0.5 | max $\lvert\Delta q_i\rvert$ per iteration [rad or m] |
| `damping` | 0.05 | DLS $\lambda$, or $\lambda_\text{max}$ when adaptive |
| `adaptive_damping` | `false` | DLS: $\lambda$ from $\sigma_\text{min}$ |
| `singular_region` | 0.05 | adaptive DLS threshold $\epsilon$ on $\sigma_\text{min}$ |
| `lm_bias` | `1e-3` | LM constant $w$ |
| `pinv_tolerance` | -1 | pseudoinverse singular-value tolerance (negative: default) |
| `stall_tolerance` | `1e-12` | $\lVert\Delta q\rVert_\infty$ below which the solve stops |
| `task_weights` | ones | weights of $(\omega_x, \omega_y, \omega_z, v_x, v_y, v_z)$ |
| `enforce_joint_limits` | `true` | active-set clamping on every iterate |
| `null_space_objective` | `None` | secondary objective |
| `null_space_gain` | 0.1 | $k$ |
| `null_space_center` | empty | centre for `JointCentering` (empty: mid-range) |
| `null_space_refinement_iterations` | 0 | iterations after convergence spent on the objective |

`MultiStartIKOptions`:

| Field | Default | Meaning |
| --- | --- | --- |
| `ik` | `IKOptions{}` | options of each solve |
| `attempts` | 32 | seeds tried, including the initial guess |
| `seed` | 1 | RNG seed of the random restarts |
| `max_solutions` | 8 | stop after this many accepted solutions |
| `distinct_threshold` | `1e-2` | infinity-norm distance below which solutions are duplicates |
| `use_initial_guess` | `true` | first attempt starts from the given seed |
| `validator` | empty | optional acceptance test, for example collision-free |

## Example

```cpp
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

#include <vector>

int main() {
  using namespace mc;
  const RobotModel arm = makeRedundant7R();
  const SE3 target = endEffectorPose(arm, redundant7RHome());

  IKOptions options;
  options.method = IKMethod::LevenbergMarquardt;
  options.null_space_objective = NullSpaceObjective::JointCentering;  // use the extra DOF
  options.null_space_refinement_iterations = 50;
  const IKResult single = solveIK(arm, target, arm.midConfiguration(), options);

  MultiStartIKOptions multi;
  multi.ik.method = IKMethod::DampedLeastSquares;
  multi.ik.adaptive_damping = true;
  multi.attempts = 16;
  multi.seed = 42;  // reproducible restarts
  const std::vector<IKResult> solutions = solveIKMultiStart(arm, target, redundant7RHome(), multi);

  return (single.converged() && single.within_limits && !solutions.empty()) ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- Closed-form IK exists only for planar 2R and 3R chains built like `makePlanar2R`/`makePlanar3R`
  (`planarLinkLengths` throws otherwise). There is no closed-form solver for spherical-wrist 6R arms.
- The iterative solver is local. Success depends on the seed, and multi-start trades time for
  coverage without any completeness guarantee.
- `boundary_tolerance` acts on $c_2$ (dimensionless), not on a distance. Targets are treated as on
  the boundary only when $\lvert s_2\rvert$ is at most that tolerance.
- Only one secondary objective at a time. There is no strict task-priority hierarchy beyond
  "primary task, then null space".
- With DLS or LM the task is solved in a damped least-squares sense. Close to singular solutions,
  convergence slows, and the tolerances may not be reached within the budget.
- Joint limits are hard box constraints. Collision avoidance is not part of the solve; the
  multi-start `validator` can only reject solutions afterwards.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 6 (numerical inverse kinematics).
- M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006 (2nd ed. 2020). Inverse kinematics of the planar elbow arm.
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 3 (inverse kinematics algorithms, redundancy resolution).
- S. R. Buss, "Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods," technical report, University of California, San Diego, 2004.
- C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods," *IEEE Transactions on Systems, Man, and Cybernetics*, 16(1), 1986.
- Y. Nakamura and H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control," *ASME Journal of Dynamic Systems, Measurement, and Control*, 108(3), 1986.
- S. Chiaverini, "Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators," *IEEE Transactions on Robotics and Automation*, 13(3), 1997.
- T. Sugihara, "Solvability-unconcerned inverse kinematics by the Levenberg-Marquardt method," *IEEE Transactions on Robotics*, 27(5), 2011.
- H. Zghal, R. V. Dubey and J. A. Euler, "Efficient gradient projection optimization for manipulators with multiple degrees of redundancy," *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, 1990.
- T. Yoshikawa, "Manipulability of robotic mechanisms," *The International Journal of Robotics Research*, 4(2), 1985.
