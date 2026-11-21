#include "manipulatorcore/trajectory/cartesian.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mc {

LinePath::LinePath(const SE3& start, const SE3& end)
    : start_(start), end_(end), phi_(logSO3(start.rotation().transpose() * end.rotation())) {}

SE3 LinePath::pose(double s) const {
  s = std::clamp(s, 0.0, 1.0);
  return {start_.rotation() * expSO3(s * phi_), start_.translation() + s * (end_.translation() - start_.translation())};
}

Vec6 LinePath::tangent(double /*s*/) const {
  Vec6 d;
  d << start_.rotation() * phi_, end_.translation() - start_.translation();
  return d;
}

Vec6 LinePath::curvature(double /*s*/) const { return Vec6::Zero(); }

CircularArcPath::CircularArcPath(const SE3& start, const Vec3& center, const Vec3& axis, double angle,
                                 const Mat3& end_rotation)
    : R0_(start.rotation()), center_(center), angle_(angle) {
  requireFinite(center, "CircularArcPath: center");
  requireFinite(axis, "CircularArcPath: axis");
  requireFinite(angle, "CircularArcPath: angle");
  require(axis.norm() > 1e-12, "CircularArcPath: axis must be non-zero");
  axis_ = axis.normalized();
  const Vec3 d = start.translation() - center;
  axial_offset_ = axis_.dot(d) * axis_;
  radius_vector_ = d - axial_offset_;
  require(radius_vector_.norm() > 1e-9, "CircularArcPath: start point lies on the axis");
  phi_ = logSO3(R0_.transpose() * end_rotation);
}

SE3 CircularArcPath::pose(double s) const {
  s = std::clamp(s, 0.0, 1.0);
  const Mat3 Rot = expSO3(axis_ * (s * angle_));
  return {R0_ * expSO3(s * phi_), center_ + axial_offset_ + Rot * radius_vector_};
}

Vec6 CircularArcPath::tangent(double s) const {
  s = std::clamp(s, 0.0, 1.0);
  const Vec3 r = expSO3(axis_ * (s * angle_)) * radius_vector_;
  Vec6 d;
  d << R0_ * phi_, angle_ * axis_.cross(r);
  return d;
}

Vec6 CircularArcPath::curvature(double s) const {
  s = std::clamp(s, 0.0, 1.0);
  const Vec3 r = expSO3(axis_ * (s * angle_)) * radius_vector_;
  Vec6 d;
  d << Vec3::Zero(), -angle_ * angle_ * r;  // centripetal: a x (a x r) = -r for r normal to a
  return d;
}

double CircularArcPath::translationLength() const { return std::abs(angle_) * radius_vector_.norm(); }

CartesianTrajectory::CartesianTrajectory(std::shared_ptr<const CartesianPath> path,
                                         std::shared_ptr<const ScalarProfile> scaling)
    : path_(std::move(path)), scaling_(std::move(scaling)) {
  require(path_ != nullptr && scaling_ != nullptr, "CartesianTrajectory: path and scaling must not be null");
  require(scaling_->distance() == 0.0 || std::abs(scaling_->distance() - 1.0) < 1e-12,
          "CartesianTrajectory: the scaling must run from s = 0 to s = 1");
}

CartesianTrajectory CartesianTrajectory::trapezoidal(std::shared_ptr<const CartesianPath> path,
                                                     double max_linear_velocity, double max_linear_acceleration,
                                                     double max_angular_velocity, double max_angular_acceleration) {
  require(path != nullptr, "CartesianTrajectory::trapezoidal: null path");
  require(max_linear_velocity > 0.0 && max_linear_acceleration > 0.0 && max_angular_velocity > 0.0 &&
              max_angular_acceleration > 0.0,
          "CartesianTrajectory::trapezoidal: limits must be positive");
  const double L = path->translationLength();
  const double theta = path->rotationAngle();
  double v = std::numeric_limits<double>::infinity();
  double a = std::numeric_limits<double>::infinity();
  if (L > 0.0) {
    v = std::min(v, max_linear_velocity / L);
    a = std::min(a, max_linear_acceleration / L);
  }
  if (theta > 0.0) {
    v = std::min(v, max_angular_velocity / theta);
    a = std::min(a, max_angular_acceleration / theta);
  }
  if (!std::isfinite(v)) {
    return {std::move(path), std::make_shared<TrapezoidalProfile>(TrapezoidalProfile::minimumTime(0.0, 1.0, 1.0))};
  }
  return {std::move(path), std::make_shared<TrapezoidalProfile>(TrapezoidalProfile::minimumTime(1.0, v, a))};
}

CartesianTrajectory CartesianTrajectory::quintic(std::shared_ptr<const CartesianPath> path, double duration) {
  return {std::move(path), std::make_shared<PolynomialProfile>(PolynomialProfile::quintic(1.0, duration))};
}

CartesianSample CartesianTrajectory::sample(double t) const {
  const ProfileSample p = scaling_->sample(t);
  const double s = scaling_->distance() > 0.0 ? p.s : 0.0;
  CartesianSample out;
  out.time = std::clamp(t, 0.0, duration());
  out.pose = path_->pose(s);
  const Vec6 d1 = path_->tangent(s);
  out.twist = d1 * p.sd;
  out.acceleration = path_->curvature(s) * (p.sd * p.sd) + d1 * p.sdd;
  return out;
}

}  // namespace mc
