#pragma once

/// @file spline.hpp
/// @brief Vector-valued C2 cubic splines and spline trajectories through waypoints.

#include "manipulatorcore/trajectory/joint_trajectory.hpp"

#include <vector>

namespace mc {

/// Interpolating cubic spline x(u) through (u_k, x_k), solved with the tridiagonal (Thomas)
/// algorithm on the knot second derivatives.
class CubicSpline {
 public:
  enum class Boundary {
    Natural,  ///< x''(u_0) = x''(u_N) = 0
    Clamped   ///< prescribed first derivatives at both ends
  };

  CubicSpline(std::vector<double> knots, std::vector<VecX> values, Boundary boundary = Boundary::Natural,
              const VecX& start_derivative = VecX(), const VecX& end_derivative = VecX());

  struct Sample {
    VecX value;
    VecX first;
    VecX second;
  };

  /// Value and derivatives at u (clamped to the knot range).
  [[nodiscard]] Sample evaluate(double u) const;
  [[nodiscard]] double start() const { return knots_.front(); }
  [[nodiscard]] double end() const { return knots_.back(); }
  [[nodiscard]] int dimension() const { return static_cast<int>(values_.front().size()); }
  [[nodiscard]] const std::vector<double>& knots() const { return knots_; }

 private:
  std::vector<double> knots_;
  std::vector<VecX> values_;
  std::vector<VecX> second_;  // second derivatives at the knots
};

/// Joint trajectory through waypoints at given times, C2 continuous, with prescribed (default
/// zero) start and end velocities.
class CubicSplineTrajectory final : public JointTrajectory {
 public:
  CubicSplineTrajectory(std::vector<double> times, std::vector<VecX> waypoints, const VecX& start_velocity = VecX(),
                        const VecX& end_velocity = VecX());
  [[nodiscard]] int dof() const override { return spline_.dimension(); }
  [[nodiscard]] double duration() const override { return spline_.end(); }
  [[nodiscard]] TrajectoryPoint sample(double t) const override;

 private:
  CubicSpline spline_;
};

}  // namespace mc
