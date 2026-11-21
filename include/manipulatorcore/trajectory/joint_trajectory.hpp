#pragma once

/// @file joint_trajectory.hpp
/// @brief Common interface of joint-space trajectories and limit checking.

#include "manipulatorcore/math/types.hpp"

#include <vector>

namespace mc {

/// Joint position, velocity and acceleration at one instant.
struct TrajectoryPoint {
  double time = 0.0;
  VecX q;
  VecX qd;
  VecX qdd;
};

/// A time-parameterized joint-space trajectory defined on [0, duration()]. Sampling outside that
/// interval clamps the time (the trajectory holds its end states with zero velocity only if the
/// trajectory itself ends at rest).
class JointTrajectory {
 public:
  JointTrajectory() = default;
  JointTrajectory(const JointTrajectory&) = default;
  JointTrajectory(JointTrajectory&&) = default;
  JointTrajectory& operator=(const JointTrajectory&) = default;
  JointTrajectory& operator=(JointTrajectory&&) = default;
  virtual ~JointTrajectory() = default;

  [[nodiscard]] virtual int dof() const = 0;
  [[nodiscard]] virtual double duration() const = 0;
  [[nodiscard]] virtual TrajectoryPoint sample(double t) const = 0;
};

/// Samples a trajectory every @p dt from 0 to duration (the final point is always included).
[[nodiscard]] std::vector<TrajectoryPoint> sampleUniform(const JointTrajectory& trajectory, double dt);

/// Worst-case usage of velocity and acceleration limits over a dense sampling.
struct LimitUsage {
  double max_velocity_ratio = 0.0;      ///< max_i,t |qd_i(t)| / vmax_i
  double max_acceleration_ratio = 0.0;  ///< max_i,t |qdd_i(t)| / amax_i
  [[nodiscard]] bool within(double tolerance = 1e-9) const {
    return max_velocity_ratio <= 1.0 + tolerance && max_acceleration_ratio <= 1.0 + tolerance;
  }
};

[[nodiscard]] LimitUsage evaluateLimitUsage(const JointTrajectory& trajectory, const VecX& velocity_limits,
                                            const VecX& acceleration_limits, double dt = 1e-3);

}  // namespace mc
