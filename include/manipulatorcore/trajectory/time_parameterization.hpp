#pragma once

/// @file time_parameterization.hpp
/// @brief Time-optimal time parameterization of joint-space paths under velocity and
/// acceleration limits.
///
/// Given a geometric path q(s), s in [0, L], the problem is to find s(t) minimizing traversal time
/// subject to |qd_i| <= vmax_i and |qdd_i| <= amax_i, with qd = q'(s) sd and
/// qdd = q'(s) sdd + q''(s) sd^2. In the variables x = sd^2 and u = sdd (dx/ds = 2u) these
/// constraints are linear. The path is discretized on a grid; a backward pass computes the
/// controllable sets K_i = [0, x_i^max] and a forward pass greedily picks the largest admissible
/// u. This is the reachability-analysis formulation of H. Pham and Q.-C. Pham, "A New Approach to
/// Time-Optimal Path Parameterization Based on Reachability Analysis", IEEE T-RO 34(3), 2018,
/// restricted to box joint limits, where each step is a 2-variable linear program solved in
/// closed form.
///
/// Constraints are imposed at both ends of every grid segment, and the limits are tightened by a
/// small safety margin to cover the discretization error between grid points.

#include "manipulatorcore/trajectory/joint_trajectory.hpp"
#include "manipulatorcore/trajectory/spline.hpp"

#include <memory>
#include <vector>

namespace mc {

/// Geometric joint-space path q(s), s in [0, length()].
class JointPath {
 public:
  JointPath() = default;
  JointPath(const JointPath&) = default;
  JointPath(JointPath&&) = default;
  JointPath& operator=(const JointPath&) = default;
  JointPath& operator=(JointPath&&) = default;
  virtual ~JointPath() = default;

  [[nodiscard]] virtual int dof() const = 0;
  [[nodiscard]] virtual double length() const = 0;
  [[nodiscard]] virtual VecX position(double s) const = 0;
  [[nodiscard]] virtual VecX firstDerivative(double s) const = 0;
  [[nodiscard]] virtual VecX secondDerivative(double s) const = 0;
};

/// Natural cubic spline through waypoints, parameterized by cumulative chord length. C2 smooth,
/// passes through every waypoint, but may deviate from the straight segments between them (so
/// collision validity must be re-checked by the caller).
class SplineJointPath final : public JointPath {
 public:
  explicit SplineJointPath(const std::vector<VecX>& waypoints);
  [[nodiscard]] int dof() const override { return spline_.dimension(); }
  [[nodiscard]] double length() const override { return spline_.end(); }
  [[nodiscard]] VecX position(double s) const override { return spline_.evaluate(s).value; }
  [[nodiscard]] VecX firstDerivative(double s) const override { return spline_.evaluate(s).first; }
  [[nodiscard]] VecX secondDerivative(double s) const override { return spline_.evaluate(s).second; }

 private:
  static CubicSpline build(const std::vector<VecX>& waypoints);
  CubicSpline spline_;
};

/// Straight segment q0 + s (q1 - q0)/|q1 - q0|, s in [0, |q1 - q0|].
class StraightJointPath final : public JointPath {
 public:
  StraightJointPath(const VecX& q0, const VecX& q1);
  [[nodiscard]] int dof() const override { return static_cast<int>(q0_.size()); }
  [[nodiscard]] double length() const override { return length_; }
  [[nodiscard]] VecX position(double s) const override { return q0_ + s * direction_; }
  [[nodiscard]] VecX firstDerivative(double) const override { return direction_; }
  [[nodiscard]] VecX secondDerivative(double) const override { return VecX::Zero(q0_.size()); }

 private:
  VecX q0_;
  VecX direction_;
  double length_;
};

struct TimeParameterizationOptions {
  int grid_segments = 1000;       ///< number of path segments of the discretization
  double velocity_scale = 1.0;    ///< fraction of the velocity limits to use, (0, 1]
  double acceleration_scale = 1.0;
  double safety_margin = 0.01;    ///< limits are tightened by this fraction (discretization error)
};

/// The time-parameterized path produced by timeOptimalParameterization().
class TimeOptimalTrajectory final : public JointTrajectory {
 public:
  [[nodiscard]] int dof() const override { return path_->dof(); }
  [[nodiscard]] double duration() const override { return times_.back(); }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;

  /// Grid of the solution: path parameter, sd^2 and sdd per grid point.
  [[nodiscard]] const std::vector<double>& gridS() const { return s_; }
  [[nodiscard]] const std::vector<double>& gridSdSquared() const { return x_; }

 private:
  friend TimeOptimalTrajectory timeOptimalParameterization(std::shared_ptr<const JointPath>, const VecX&,
                                                          const VecX&, const TimeParameterizationOptions&);
  TimeOptimalTrajectory() = default;
  std::shared_ptr<const JointPath> path_;
  std::vector<double> s_;
  std::vector<double> x_;
  std::vector<double> u_;
  std::vector<double> times_;
};

/// Time-optimal rest-to-rest traversal of @p path. Throws InvalidArgument for invalid limits or
/// a path whose discretization admits no solution.
[[nodiscard]] TimeOptimalTrajectory timeOptimalParameterization(std::shared_ptr<const JointPath> path,
                                                                const VecX& velocity_limits,
                                                                const VecX& acceleration_limits,
                                                                const TimeParameterizationOptions& options = {});

}  // namespace mc
