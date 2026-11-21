#pragma once

/// @file differential_kinematics.hpp
/// @brief Mapping between joint velocities and task twists, including near singularities.

#include "manipulatorcore/math/types.hpp"

namespace mc {

enum class DifferentialSolver {
  Transpose,          ///< qd = J^T x (direction only; not an inverse)
  PseudoInverse,      ///< qd = J^+ x via SVD
  DampedLeastSquares  ///< qd = J^T (J J^T + lambda^2 I)^{-1} x
};

/// Task twist (or other task velocity) produced by joint velocities: x = J qd.
[[nodiscard]] VecX taskVelocity(const MatX& J, const VecX& qd);

/// Joint velocity for a desired task velocity. @p damping is lambda for DampedLeastSquares and the
/// singular-value tolerance for PseudoInverse (negative = default tolerance).
[[nodiscard]] VecX jointVelocityForTask(const MatX& J, const VecX& task_velocity, DifferentialSolver solver,
                                        double damping = 0.05);

/// Adaptive damping (Nakamura & Hanafusa 1986; Chiaverini 1997):
///   lambda^2 = (1 - (sigma_min / epsilon)^2) lambda_max^2  if sigma_min < epsilon, else 0.
/// Returns lambda. Smoothly increases damping only inside the singular region.
[[nodiscard]] double adaptiveDamping(double sigma_min, double epsilon, double lambda_max);

/// Redundancy resolution qd = J# x + (I - J# J) qd_null with J# the damped pseudoinverse
/// (lambda = 0 gives the Moore-Penrose pseudoinverse). The secondary velocity qd_null does not
/// change the task velocity when J has full row rank and lambda = 0.
[[nodiscard]] VecX jointVelocityWithNullSpace(const MatX& J, const VecX& task_velocity, const VecX& qd_null,
                                              double damping = 0.0);

}  // namespace mc
