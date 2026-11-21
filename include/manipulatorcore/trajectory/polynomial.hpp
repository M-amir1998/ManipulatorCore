#pragma once

/// @file polynomial.hpp
/// @brief Point-to-point joint trajectories: polynomial boundary-value problems, linear
/// interpolation and synchronized straight-line motions.

#include "manipulatorcore/trajectory/joint_trajectory.hpp"
#include "manipulatorcore/trajectory/profiles.hpp"

#include <memory>
#include <vector>

namespace mc {

/// Per-joint polynomial q_i(t) = sum_k c_ik t^k on [0, T].
class PolynomialTrajectory final : public JointTrajectory {
 public:
  /// Cubic with prescribed boundary positions and velocities.
  [[nodiscard]] static PolynomialTrajectory cubic(const VecX& q0, const VecX& qf, double duration,
                                                  const VecX& v0 = VecX(), const VecX& vf = VecX());

  /// Quintic with prescribed boundary positions, velocities and accelerations.
  [[nodiscard]] static PolynomialTrajectory quintic(const VecX& q0, const VecX& qf, double duration,
                                                    const VecX& v0 = VecX(), const VecX& vf = VecX(),
                                                    const VecX& a0 = VecX(), const VecX& af = VecX());

  [[nodiscard]] int dof() const override { return static_cast<int>(coefficients_.rows()); }
  [[nodiscard]] double duration() const override { return duration_; }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;
  /// Coefficients: row i = joint i, column k = coefficient of t^k.
  [[nodiscard]] const MatX& coefficients() const { return coefficients_; }

 private:
  PolynomialTrajectory(MatX coefficients, double duration);
  MatX coefficients_;
  double duration_;
};

/// Piecewise-linear interpolation through waypoints at given times (velocity is piecewise
/// constant and acceleration zero; the velocity jumps at waypoints).
class LinearInterpolationTrajectory final : public JointTrajectory {
 public:
  LinearInterpolationTrajectory(std::vector<double> times, std::vector<VecX> waypoints);
  [[nodiscard]] int dof() const override;
  [[nodiscard]] double duration() const override { return times_.back(); }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;

 private:
  std::vector<double> times_;
  std::vector<VecX> waypoints_;
};

/// Straight line q0 -> q1 in joint space traversed with a scalar profile s(t) in [0, 1]. All
/// joints start and stop together and the path stays on the line.
class LineTrajectory final : public JointTrajectory {
 public:
  LineTrajectory(VecX q0, VecX q1, std::shared_ptr<const ScalarProfile> profile);

  /// Minimum-time synchronized trapezoidal motion under per-joint velocity/acceleration limits.
  [[nodiscard]] static LineTrajectory trapezoidal(const VecX& q0, const VecX& q1, const VecX& velocity_limits,
                                                  const VecX& acceleration_limits);

  /// Minimum-time synchronized jerk-limited (double-S) motion.
  [[nodiscard]] static LineTrajectory doubleS(const VecX& q0, const VecX& q1, const VecX& velocity_limits,
                                              const VecX& acceleration_limits, const VecX& jerk_limits);

  /// Rest-to-rest quintic of the given duration.
  [[nodiscard]] static LineTrajectory quintic(const VecX& q0, const VecX& q1, double duration);

  [[nodiscard]] int dof() const override { return static_cast<int>(q0_.size()); }
  [[nodiscard]] double duration() const override { return profile_->duration(); }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;

 private:
  VecX q0_;
  VecX delta_;
  std::shared_ptr<const ScalarProfile> profile_;
};

/// Sequence of joint trajectories played back to back (each must start where the previous ends).
class CompositeTrajectory final : public JointTrajectory {
 public:
  explicit CompositeTrajectory(std::vector<std::shared_ptr<const JointTrajectory>> segments);
  [[nodiscard]] int dof() const override;
  [[nodiscard]] double duration() const override { return start_times_.back(); }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;

 private:
  std::vector<std::shared_ptr<const JointTrajectory>> segments_;
  std::vector<double> start_times_;  // size segments + 1; last = total duration
};

/// Rest-to-rest synchronized trapezoidal motion through every waypoint (stopping at each one).
/// Always respects the limits and never leaves the polyline, at the cost of stopping.
[[nodiscard]] CompositeTrajectory stopAtWaypointsTrajectory(const std::vector<VecX>& waypoints,
                                                            const VecX& velocity_limits,
                                                            const VecX& acceleration_limits);

}  // namespace mc
