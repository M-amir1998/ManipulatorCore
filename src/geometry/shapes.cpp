#include "manipulatorcore/geometry/shapes.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <cmath>
#include <utility>

namespace mc {

namespace {

template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

/// Rotation whose z axis is the unit vector u (minimal rotation from e_z).
Mat3 rotationAligningZ(const Vec3& u) {
  const Vec3 z = Vec3::UnitZ();
  const Vec3 axis = z.cross(u);
  const double s = axis.norm();
  const double c = z.dot(u);
  if (s < 1e-12) {
    return c > 0.0 ? Mat3::Identity() : rotX(kPi);
  }
  return expSO3(axis / s * std::atan2(s, c));
}

}  // namespace

void validateShape(const Shape& shape) {
  requireFinite(shape.pose.rotation(), "Shape pose rotation");
  requireFinite(shape.pose.translation(), "Shape pose translation");
  require(isRotationMatrix(shape.pose.rotation(), 1e-6), "Shape pose rotation is not in SO(3)");
  std::visit(
      Overloaded{
          [](const Sphere& s) { require(std::isfinite(s.radius) && s.radius > 0.0, "Sphere radius must be positive"); },
          [](const Capsule& c) {
            require(std::isfinite(c.radius) && c.radius > 0.0, "Capsule radius must be positive");
            require(std::isfinite(c.half_length) && c.half_length >= 0.0, "Capsule half_length must be non-negative");
          },
          [](const Box& b) {
            require(b.half_extents.allFinite() && (b.half_extents.array() > 0.0).all(),
                    "Box half extents must be positive");
          },
          [](const HalfSpace& h) {
            require(h.normal.allFinite() && std::abs(h.normal.norm() - 1.0) < 1e-9,
                    "HalfSpace normal must be a unit vector");
            require(std::isfinite(h.offset), "HalfSpace offset must be finite");
          },
      },
      shape.geometry);
}

Shape makeSphere(const Vec3& center, double radius, std::string name) {
  Shape shape{Sphere{radius}, SE3::fromTranslation(center), std::move(name)};
  validateShape(shape);
  return shape;
}

Shape makeCapsule(const Vec3& a, const Vec3& b, double radius, std::string name) {
  requireFinite(a, "makeCapsule: a");
  requireFinite(b, "makeCapsule: b");
  const Vec3 d = b - a;
  const double length = d.norm();
  const Mat3 R = length > 1e-12 ? rotationAligningZ(d / length) : Mat3::Identity();
  Shape shape{Capsule{radius, 0.5 * length}, SE3(R, 0.5 * (a + b)), std::move(name)};
  validateShape(shape);
  return shape;
}

Shape makeBox(const SE3& pose, const Vec3& half_extents, std::string name) {
  Shape shape{Box{half_extents}, pose, std::move(name)};
  validateShape(shape);
  return shape;
}

Shape makeHalfSpace(const Vec3& normal, double offset, std::string name) {
  requireFinite(normal, "makeHalfSpace: normal");
  const double n = normal.norm();
  require(n > 1e-12, "makeHalfSpace: normal must be non-zero");
  Shape shape{HalfSpace{normal / n, offset / n}, SE3::Identity(), std::move(name)};
  validateShape(shape);
  return shape;
}

Shape transformShape(const SE3& T_new_parent, const Shape& shape) {
  Shape out = shape;
  out.pose = T_new_parent * shape.pose;
  return out;
}

}  // namespace mc
