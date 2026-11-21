#include "manipulatorcore/collision/aabb.hpp"

#include <cmath>
#include <limits>

namespace mc {

AABB AABB::of(const ShapeGeometry& geometry, const SE3& pose) {
  AABB box;
  if (const auto* s = std::get_if<Sphere>(&geometry)) {
    box.min = pose.translation() - Vec3::Constant(s->radius);
    box.max = pose.translation() + Vec3::Constant(s->radius);
  } else if (const auto* c = std::get_if<Capsule>(&geometry)) {
    const Vec3 a = pose * Vec3(0.0, 0.0, -c->half_length);
    const Vec3 b = pose * Vec3(0.0, 0.0, c->half_length);
    box.min = a.cwiseMin(b) - Vec3::Constant(c->radius);
    box.max = a.cwiseMax(b) + Vec3::Constant(c->radius);
  } else if (const auto* bx = std::get_if<Box>(&geometry)) {
    const Vec3 extent = pose.rotation().cwiseAbs() * bx->half_extents;
    box.min = pose.translation() - extent;
    box.max = pose.translation() + extent;
  } else {
    box.min = Vec3::Constant(-std::numeric_limits<double>::infinity());
    box.max = Vec3::Constant(std::numeric_limits<double>::infinity());
  }
  return box;
}

bool AABB::overlaps(const AABB& other, double margin) const {
  if (isUnbounded() || other.isUnbounded()) {
    return true;
  }
  return (max.array() + margin >= other.min.array()).all() && (other.max.array() + margin >= min.array()).all();
}

double AABB::distanceLowerBound(const AABB& other) const {
  if (isUnbounded() || other.isUnbounded()) {
    return 0.0;
  }
  const Vec3 gap = (other.min - max).cwiseMax(min - other.max).cwiseMax(Vec3::Zero());
  return gap.norm();
}

void AABB::expand(const AABB& other) {
  min = min.cwiseMin(other.min);
  max = max.cwiseMax(other.max);
}

}  // namespace mc
