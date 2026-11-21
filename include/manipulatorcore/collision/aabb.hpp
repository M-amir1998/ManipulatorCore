#pragma once

/// @file aabb.hpp
/// @brief Axis-aligned bounding boxes for broad-phase culling.

#include "manipulatorcore/geometry/shapes.hpp"

#include <limits>

namespace mc {

struct AABB {
  Vec3 min = Vec3::Constant(std::numeric_limits<double>::infinity());
  Vec3 max = Vec3::Constant(-std::numeric_limits<double>::infinity());

  /// Tight bounds of a posed primitive. Half-spaces are unbounded.
  [[nodiscard]] static AABB of(const ShapeGeometry& geometry, const SE3& pose);

  [[nodiscard]] bool isUnbounded() const { return !min.allFinite() || !max.allFinite(); }

  /// True if the boxes, each inflated by margin / 2, overlap.
  [[nodiscard]] bool overlaps(const AABB& other, double margin = 0.0) const;

  /// Lower bound on the distance between any contents of the two boxes (0 if they overlap).
  [[nodiscard]] double distanceLowerBound(const AABB& other) const;

  void expand(const AABB& other);
};

}  // namespace mc
