#pragma once

/// @file cartesian.hpp
/// @brief Cartesian end-effector paths and trajectories.
///
/// Paths are parameterized by s in [0, 1]. Orientation follows the SO(3) geodesic
/// R(s) = R_0 exp(s log(R_0^T R_1)), which is equivalent to quaternion SLERP. Derivatives are
/// world-aligned [omega; v] per unit s, matching the geometric Jacobian.

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/trajectory/profiles.hpp"

#include <memory>

namespace mc {

class CartesianPath {
 public:
  CartesianPath() = default;
  CartesianPath(const CartesianPath&) = default;
  CartesianPath(CartesianPath&&) = default;
  CartesianPath& operator=(const CartesianPath&) = default;
  CartesianPath& operator=(CartesianPath&&) = default;
  virtual ~CartesianPath() = default;

  [[nodiscard]] virtual SE3 pose(double s) const = 0;
  /// d/ds of the pose as [omega; pdot] per unit s (base frame).
  [[nodiscard]] virtual Vec6 tangent(double s) const = 0;
  /// d^2/ds^2 as [omegadot; pddot] per unit s^2 (base frame).
  [[nodiscard]] virtual Vec6 curvature(double s) const = 0;
  /// Path length of the origin [m].
  [[nodiscard]] virtual double translationLength() const = 0;
  /// Total rotation angle [rad].
  [[nodiscard]] virtual double rotationAngle() const = 0;
};

/// Straight line between two poses with geodesic orientation interpolation.
class LinePath final : public CartesianPath {
 public:
  LinePath(const SE3& start, const SE3& end);
  [[nodiscard]] SE3 pose(double s) const override;
  [[nodiscard]] Vec6 tangent(double s) const override;
  [[nodiscard]] Vec6 curvature(double s) const override;
  [[nodiscard]] double translationLength() const override { return (end_.translation() - start_.translation()).norm(); }
  [[nodiscard]] double rotationAngle() const override { return phi_.norm(); }

 private:
  SE3 start_;
  SE3 end_;
  Vec3 phi_;  // log(R0^T R1), body-frame rotation vector
};

/// Circular arc about @p axis through @p center, starting at the start pose's position and
/// sweeping @p angle radians (right-hand rule; 2 pi gives a full circle). Orientation is
/// interpolated geodesically from the start rotation to @p end_rotation.
class CircularArcPath final : public CartesianPath {
 public:
  CircularArcPath(const SE3& start, const Vec3& center, const Vec3& axis, double angle, const Mat3& end_rotation);
  [[nodiscard]] SE3 pose(double s) const override;
  [[nodiscard]] Vec6 tangent(double s) const override;
  [[nodiscard]] Vec6 curvature(double s) const override;
  [[nodiscard]] double translationLength() const override;
  [[nodiscard]] double rotationAngle() const override { return phi_.norm(); }
  [[nodiscard]] double radius() const { return radius_vector_.norm(); }

 private:
  Mat3 R0_;
  Vec3 center_;
  Vec3 axis_;
  double angle_;
  Vec3 radius_vector_;  // p0 - center, projected onto the plane normal to the axis
  Vec3 axial_offset_;   // component of p0 - center along the axis
  Vec3 phi_;
};

/// Pose, twist and acceleration of a Cartesian trajectory at one instant (base frame).
struct CartesianSample {
  double time = 0.0;
  SE3 pose;
  Vec6 twist = Vec6::Zero();         ///< [omega; pdot]
  Vec6 acceleration = Vec6::Zero();  ///< [omegadot; pddot]
};

/// A Cartesian path traversed with a time scaling s(t) in [0, 1].
class CartesianTrajectory {
 public:
  CartesianTrajectory(std::shared_ptr<const CartesianPath> path, std::shared_ptr<const ScalarProfile> scaling);

  /// Minimum-time trapezoidal scaling under linear and angular speed/acceleration limits.
  [[nodiscard]] static CartesianTrajectory trapezoidal(std::shared_ptr<const CartesianPath> path,
                                                       double max_linear_velocity, double max_linear_acceleration,
                                                       double max_angular_velocity,
                                                       double max_angular_acceleration);

  /// Rest-to-rest quintic scaling of a given duration.
  [[nodiscard]] static CartesianTrajectory quintic(std::shared_ptr<const CartesianPath> path, double duration);

  [[nodiscard]] double duration() const { return scaling_->duration(); }
  [[nodiscard]] CartesianSample sample(double t) const;
  [[nodiscard]] const CartesianPath& path() const { return *path_; }

 private:
  std::shared_ptr<const CartesianPath> path_;
  std::shared_ptr<const ScalarProfile> scaling_;
};

}  // namespace mc
