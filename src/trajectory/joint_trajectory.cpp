#include "manipulatorcore/trajectory/joint_trajectory.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

std::vector<TrajectoryPoint> sampleUniform(const JointTrajectory& trajectory, double dt) {
  require(std::isfinite(dt) && dt > 0.0, "sampleUniform: dt must be positive");
  const double T = trajectory.duration();
  const auto steps = static_cast<std::size_t>(std::floor(T / dt));
  std::vector<TrajectoryPoint> points;
  points.reserve(steps + 2);
  for (std::size_t k = 0; k <= steps; ++k) {
    points.push_back(trajectory.sample(static_cast<double>(k) * dt));
  }
  if (points.back().time < T) {
    points.push_back(trajectory.sample(T));
  }
  return points;
}

LimitUsage evaluateLimitUsage(const JointTrajectory& trajectory, const VecX& velocity_limits,
                              const VecX& acceleration_limits, double dt) {
  requireVector(velocity_limits, trajectory.dof(), "evaluateLimitUsage: velocity limits");
  requireVector(acceleration_limits, trajectory.dof(), "evaluateLimitUsage: acceleration limits");
  LimitUsage usage;
  for (const TrajectoryPoint& p : sampleUniform(trajectory, dt)) {
    usage.max_velocity_ratio =
        std::max(usage.max_velocity_ratio, p.qd.cwiseAbs().cwiseQuotient(velocity_limits).maxCoeff());
    usage.max_acceleration_ratio =
        std::max(usage.max_acceleration_ratio, p.qdd.cwiseAbs().cwiseQuotient(acceleration_limits).maxCoeff());
  }
  return usage;
}

}  // namespace mc
