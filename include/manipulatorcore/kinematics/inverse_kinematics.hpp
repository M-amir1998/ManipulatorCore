#pragma once

/// @file inverse_kinematics.hpp
/// @brief Iterative numerical inverse kinematics for arbitrary serial chains.
///
/// The pose error is the world-aligned error e = [log(R_d R^T)^v ; p_d - p], paired with the
/// geometric Jacobian. Euler angles are never subtracted. Each method computes a joint step
/// dq from the (weighted) error and Jacobian:
///
///  - JacobianTranspose:   dq = alpha J^T e, alpha = <e, J J^T e> / |J J^T e|^2 (Buss 2004)
///  - PseudoInverse:       dq = J^+ e (Gauss-Newton), SVD with a rank tolerance
///  - DampedLeastSquares:  dq = J^T (J J^T + lambda^2 I)^{-1} e (Wampler 1986; Nakamura 1986)
///  - LevenbergMarquardt:  dq = (J^T J + (E + w) I)^{-1} J^T e, E = 1/2 |e|^2 (Sugihara 2011)
///
/// Joint limits are enforced on every iterate with an active-set clamping loop: joints that
/// would cross a limit are fixed at the limit and the step is recomputed for the remaining joints.
/// Results always lie within the limits when enforce_joint_limits is true.

#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/robot_model.hpp"
#include "manipulatorcore/optimization/objectives.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace mc {

enum class IKMethod { JacobianTranspose, PseudoInverse, DampedLeastSquares, LevenbergMarquardt };

enum class IKStatus {
  Converged,      ///< both tolerances met
  MaxIterations,  ///< iteration budget exhausted
  Stalled         ///< step became negligible before converging (local minimum or limits)
};

[[nodiscard]] const char* toString(IKMethod method);
[[nodiscard]] const char* toString(IKStatus status);

struct IKOptions {
  IKMethod method = IKMethod::DampedLeastSquares;
  int max_iterations = 200;
  double position_tolerance = 1e-4;     ///< [m]
  double orientation_tolerance = 1e-3;  ///< [rad]
  double step_size = 1.0;               ///< scale applied to each step
  double max_step = 0.5;                ///< max |dq_i| per iteration [rad or m]
  double damping = 0.05;                ///< DLS lambda (or lambda_max when adaptive)
  bool adaptive_damping = false;        ///< DLS: lambda from adaptiveDamping(sigma_min, ...)
  double singular_region = 0.05;        ///< DLS adaptive: epsilon threshold on sigma_min
  double lm_bias = 1e-3;                ///< LM: constant w added to the error-dependent damping
  double pinv_tolerance = -1.0;         ///< pseudoinverse singular-value tolerance (< 0: default)
  double stall_tolerance = 1e-12;       ///< |dq|_inf below which the solve is considered stalled
  /// Task weights for [omega_x, omega_y, omega_z, v_x, v_y, v_z]. Zero weights remove a component
  /// from the task (e.g. {0,0,0,1,1,1} solves position only; planar arms use {0,0,1,1,1,0}).
  Vec6 task_weights = Vec6::Ones();
  bool enforce_joint_limits = true;
  /// Secondary objective minimized in the null space of the task (redundant arms).
  NullSpaceObjective null_space_objective = NullSpaceObjective::None;
  double null_space_gain = 0.1;
  VecX null_space_center;  ///< optional centre for JointCentering (defaults to mid-range)
  /// Extra iterations after the task has converged, during which the secondary objective keeps
  /// being minimized while the primary task is held (redundancy resolution). 0 stops at the first
  /// converged iterate.
  int null_space_refinement_iterations = 0;
};

struct IKResult {
  VecX q;
  IKStatus status = IKStatus::MaxIterations;
  int iterations = 0;
  double position_error = 0.0;     ///< |p_d - p| over the weighted components [m]
  double orientation_error = 0.0;  ///< |log(R_d R^T)| over the weighted components [rad]
  bool within_limits = true;
  [[nodiscard]] bool converged() const { return status == IKStatus::Converged; }
};

/// World-aligned pose error [log(R_d R^T)^v ; p_d - p] between the current and desired poses.
[[nodiscard]] Vec6 poseErrorWorld(const SE3& current, const SE3& desired);

/// Body-frame pose error log(T^{-1} T_d)^v (pairs with the body Jacobian).
[[nodiscard]] Vec6 poseErrorBody(const SE3& current, const SE3& desired);

/// Solves IK from @p seed. Throws InvalidArgument for invalid options or seed.
[[nodiscard]] IKResult solveIK(const RobotModel& model, const SE3& target, const VecX& seed,
                               const IKOptions& options = {});

struct MultiStartIKOptions {
  IKOptions ik;
  int attempts = 32;                 ///< total number of seeds tried (including the initial guess)
  std::uint64_t seed = 1;            ///< RNG seed for the random restarts
  int max_solutions = 8;             ///< stop after this many distinct accepted solutions
  double distinct_threshold = 1e-2;  ///< solutions closer than this (inf-norm) are duplicates
  bool use_initial_guess = true;     ///< first attempt starts from the given seed
  /// Optional acceptance test (e.g. collision-free). Rejected solutions are discarded.
  std::function<bool(const VecX&)> validator;
};

/// Multi-start IK: runs solveIK from the initial guess and from uniformly random configurations
/// inside the joint limits. Returns converged, accepted, mutually distinct solutions sorted by
/// distance to the initial guess.
[[nodiscard]] std::vector<IKResult> solveIKMultiStart(const RobotModel& model, const SE3& target, const VecX& seed,
                                                      const MultiStartIKOptions& options = {});

}  // namespace mc
