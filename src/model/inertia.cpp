#include "manipulatorcore/model/inertia.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>

namespace mc {

namespace {

/// Minimal rotation taking e_z to the unit vector u.
Mat3 rotationZTo(const Vec3& u) {
  const Vec3 axis = Vec3::UnitZ().cross(u);
  const double s = axis.norm();
  const double c = u.z();
  if (s < 1e-12) {
    return c > 0.0 ? Mat3::Identity() : rotX(kPi);
  }
  return expSO3(axis / s * std::atan2(s, c));
}

}  // namespace

Inertia Inertia::pointMass(double mass, const Vec3& com) { return {mass, com, Mat3::Zero()}; }

Inertia Inertia::solidCylinder(double mass, double radius, const Vec3& a, const Vec3& b) {
  require(mass >= 0.0 && radius >= 0.0, "Inertia::solidCylinder: mass and radius must be non-negative");
  const Vec3 d = b - a;
  const double length = d.norm();
  const double axial = 0.5 * mass * radius * radius;
  const double transverse = mass * (3.0 * radius * radius + length * length) / 12.0;
  const Mat3 local = Vec3(transverse, transverse, axial).asDiagonal();
  const Mat3 R = length > 1e-12 ? rotationZTo(d / length) : Mat3::Identity();
  return {mass, 0.5 * (a + b), R * local * R.transpose()};
}

Inertia Inertia::solidBox(double mass, const Vec3& size, const Vec3& center) {
  require(mass >= 0.0 && (size.array() >= 0.0).all(), "Inertia::solidBox: mass and size must be non-negative");
  const Vec3 sq = size.cwiseProduct(size);
  const Mat3 I = (mass / 12.0 * Vec3(sq.y() + sq.z(), sq.x() + sq.z(), sq.x() + sq.y())).asDiagonal();
  return {mass, center, I};
}

Inertia Inertia::solidSphere(double mass, double radius, const Vec3& center) {
  require(mass >= 0.0 && radius >= 0.0, "Inertia::solidSphere: mass and radius must be non-negative");
  return {mass, center, Mat3::Identity() * (0.4 * mass * radius * radius)};
}

Inertia Inertia::combine(const Inertia& a, const Inertia& b) {
  const double mass = a.mass + b.mass;
  if (mass <= 0.0) {
    return {};
  }
  const Vec3 com = (a.mass * a.com + b.mass * b.com) / mass;
  // Parallel-axis theorem: I_about_com = I_c + m (|r|^2 I - r r^T) = I_c - m [r]^2.
  const Vec3 ra = a.com - com;
  const Vec3 rb = b.com - com;
  const Mat3 I = a.inertia_com - a.mass * hat(ra) * hat(ra) + b.inertia_com - b.mass * hat(rb) * hat(rb);
  return {mass, com, 0.5 * (I + I.transpose())};
}

Inertia Inertia::transformed(const SE3& T_a_body) const {
  const Mat3& R = T_a_body.rotation();
  return {mass, T_a_body * com, R * inertia_com * R.transpose()};
}

Mat3 Inertia::inertiaAboutOrigin() const { return inertia_com - mass * hat(com) * hat(com); }

Mat6 Inertia::spatialMatrix() const {
  const Mat3 C = hat(com);
  Mat6 G;
  G.topLeftCorner<3, 3>() = inertia_com - mass * C * C;
  G.topRightCorner<3, 3>() = mass * C;
  G.bottomLeftCorner<3, 3>() = -mass * C;
  G.bottomRightCorner<3, 3>() = mass * Mat3::Identity();
  return G;
}

void Inertia::validate() const {
  require(std::isfinite(mass) && mass >= 0.0, "Inertia: mass must be finite and non-negative");
  require(com.allFinite(), "Inertia: centre of mass must be finite");
  require(inertia_com.allFinite(), "Inertia: inertia tensor must be finite");
  const double scale = std::max(1.0, inertia_com.cwiseAbs().maxCoeff());
  require((inertia_com - inertia_com.transpose()).cwiseAbs().maxCoeff() <= 1e-9 * scale,
          "Inertia: inertia tensor must be symmetric");
  const Eigen::SelfAdjointEigenSolver<Mat3> eig(inertia_com, Eigen::EigenvaluesOnly);
  const Vec3& principal = eig.eigenvalues();
  const double tol = 1e-9 * scale;
  require((principal.array() >= -tol).all(), "Inertia: principal moments must be non-negative");
  require(principal(0) + principal(1) >= principal(2) - tol,
          "Inertia: principal moments violate the triangle inequality");
}

}  // namespace mc
