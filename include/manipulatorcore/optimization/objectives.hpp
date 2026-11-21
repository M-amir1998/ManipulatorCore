#pragma once

/// @file objectives.hpp
/// @brief Differentiable secondary objectives for redundancy resolution (null-space tasks).
///
/// Each objective is a scalar cost H(q) to be minimized, with its gradient. A null-space task
/// typically uses qd_null = -k grad H(q).

#include "manipulatorcore/model/robot_model.hpp"

namespace mc {

enum class NullSpaceObjective {
  None,
  JointCentering,       ///< H = 1/2 sum(((q_i - c_i) / range_i)^2)
  JointLimitBarrier,    ///< Zghal et al.: H = sum(range_i^2 / (4 (u_i - q_i)(q_i - l_i))) / n
  JointLimitActivation, ///< C1 quadratic penalty active only near the limits
  Manipulability        ///< H = -w(q), Yoshikawa manipulability of the geometric Jacobian
};

[[nodiscard]] const char* toString(NullSpaceObjective objective);

struct ObjectiveValue {
  double value = 0.0;
  VecX gradient;
};

/// Joint-centering cost around @p center (defaults to the limit mid-points when empty).
[[nodiscard]] ObjectiveValue jointCenteringObjective(const RobotModel& model, const VecX& q,
                                                     const VecX& center = VecX());

/// Barrier cost of H. Zghal, R. V. Dubey and J. A. Euler (1990): minimum 1 at the mid-range,
/// unbounded at the limits. Requires strictly interior q.
[[nodiscard]] ObjectiveValue jointLimitBarrierObjective(const RobotModel& model, const VecX& q);

/// Quadratic penalty that is exactly zero in the interior and grows within @p activation_fraction
/// of the range from each limit: phi = 1/2 ((q - (l + d)) / d)^2 for q < l + d, with
/// d = activation_fraction * range (symmetric at the upper limit). C1 continuous, so the
/// resulting null-space velocity has no jumps.
[[nodiscard]] ObjectiveValue jointLimitActivationObjective(const RobotModel& model, const VecX& q,
                                                           double activation_fraction = 0.1);

/// Negative Yoshikawa manipulability of the geometric Jacobian rows selected by
/// @p translational_only, with a central-difference gradient.
[[nodiscard]] ObjectiveValue manipulabilityObjective(const RobotModel& model, const VecX& q,
                                                     bool translational_only = false);

/// Dispatches to the objective selected by @p objective. NullSpaceObjective::None returns zero.
[[nodiscard]] ObjectiveValue evaluateObjective(NullSpaceObjective objective, const RobotModel& model,
                                               const VecX& q);

}  // namespace mc
