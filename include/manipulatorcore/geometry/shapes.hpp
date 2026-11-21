#pragma once

/// @file shapes.hpp
/// @brief Geometric primitives used for collision checking and visualization.
///
/// Every primitive is defined in its own local frame; a Shape places it in a parent frame (a robot
/// body frame or the world) through T_parent_shape.

#include "manipulatorcore/geometry/se3.hpp"

#include <string>
#include <variant>

namespace mc {

/// Sphere centred at the local origin.
struct Sphere {
  double radius = 0.0;
};

/// Capsule: all points within @p radius of the segment from (0,0,-half_length) to (0,0,+half_length).
struct Capsule {
  double radius = 0.0;
  double half_length = 0.0;
};

/// Axis-aligned box in its local frame (an oriented box once posed), centred at the origin.
struct Box {
  Vec3 half_extents = Vec3::Zero();
};

/// Half-space {x : normal . x <= offset}. The plane is its boundary; the solid side is opposite the
/// normal. Example: a floor at z = 0 is HalfSpace{(0,0,1), 0}.
struct HalfSpace {
  Vec3 normal = Vec3::UnitZ();
  double offset = 0.0;
};

using ShapeGeometry = std::variant<Sphere, Capsule, Box, HalfSpace>;

/// A primitive placed in a parent frame.
struct Shape {
  ShapeGeometry geometry;
  SE3 pose;          ///< T_parent_shape
  std::string name;  ///< Optional label for diagnostics and visualization.
};

/// Throws InvalidArgument if the shape has non-positive or non-finite dimensions, or a non-unit
/// half-space normal.
void validateShape(const Shape& shape);

/// Sphere of @p radius centred at @p center (parent frame).
[[nodiscard]] Shape makeSphere(const Vec3& center, double radius, std::string name = {});

/// Capsule whose core segment runs from @p a to @p b (parent frame). If a == b this is a sphere.
[[nodiscard]] Shape makeCapsule(const Vec3& a, const Vec3& b, double radius, std::string name = {});

/// Box with the given pose (T_parent_box) and half extents.
[[nodiscard]] Shape makeBox(const SE3& pose, const Vec3& half_extents, std::string name = {});

/// Half-space {x : normal . x <= offset} (parent frame). The normal is normalized.
[[nodiscard]] Shape makeHalfSpace(const Vec3& normal, double offset, std::string name = {});

/// Shape expressed in a new parent: returns a copy with pose T_new_parent * pose.
[[nodiscard]] Shape transformShape(const SE3& T_new_parent, const Shape& shape);

}  // namespace mc
