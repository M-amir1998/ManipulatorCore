#pragma once

/// @file distance.hpp
/// @brief Signed distance and closest points between posed geometric primitives.
///
/// | pair                          | method                                         | exact?          |
/// | ----------------------------- | ---------------------------------------------- | --------------- |
/// | sphere/capsule - sphere/capsule | closest points between core segments          | yes (signed)    |
/// | sphere/capsule - box          | convex 1-D minimization of the box SDF on the core | separation: yes; penetration: core depth |
/// | box - box                     | GJK (separated), separating-axis depth (overlap) | separation: yes; penetration: SAT depth |
/// | anything - half-space         | support point against the plane                | yes (signed)    |
///
/// Negative distances indicate penetration. For penetrating capsule-box and box-box pairs the
/// magnitude is a penetration measure (not necessarily the exact translational depth), which is
/// sufficient for collision tests, safety margins and visualization.

#include "manipulatorcore/geometry/shapes.hpp"

#include <functional>

namespace mc {

struct DistanceResult {
  double distance = 0.0;       ///< signed distance; < 0 means penetration
  Vec3 point_a = Vec3::Zero(); ///< closest (or deepest) point on A, world frame
  Vec3 point_b = Vec3::Zero(); ///< closest (or deepest) point on B, world frame
  Vec3 normal = Vec3::UnitZ(); ///< unit direction from A towards B
};

/// Signed distance between geometry @p a at world pose @p T_a and @p b at @p T_b. Throws
/// InvalidArgument for half-space vs half-space.
[[nodiscard]] DistanceResult computeDistance(const ShapeGeometry& a, const SE3& T_a, const ShapeGeometry& b,
                                             const SE3& T_b);

/// Convenience overload for shapes whose pose is relative to a parent frame placed at T_parent.
[[nodiscard]] DistanceResult computeDistance(const Shape& a, const SE3& T_world_parent_a, const Shape& b,
                                             const SE3& T_world_parent_b);

/// True if the shapes overlap or are closer than @p margin.
[[nodiscard]] bool shapesCollide(const ShapeGeometry& a, const SE3& T_a, const ShapeGeometry& b, const SE3& T_b,
                                 double margin = 0.0);

/// Closest points between segments [p1, q1] and [p2, q2] (C. Ericson, Real-Time Collision
/// Detection, 2005, section 5.1.9). Returns the squared distance.
double closestPointsSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Vec3& c1,
                                   Vec3& c2);

/// Signed distance from a world point to a box with half extents @p h at pose @p T_box, and the
/// closest point on the box surface.
double signedDistancePointBox(const Vec3& point, const SE3& T_box, const Vec3& h, Vec3* closest = nullptr);

/// Support mapping of a convex set: the point of the set furthest along a direction.
using SupportFunction = std::function<Vec3(const Vec3& direction)>;

struct GjkResult {
  bool intersecting = false;
  double distance = 0.0;  ///< 0 when intersecting
  Vec3 point_a = Vec3::Zero();
  Vec3 point_b = Vec3::Zero();
  int iterations = 0;
};

/// Gilbert-Johnson-Keerthi distance between two convex sets given by support functions.
/// E. G. Gilbert, D. W. Johnson and S. S. Keerthi, IEEE J. Robotics and Automation 4(2), 1988.
[[nodiscard]] GjkResult gjkDistance(const SupportFunction& support_a, const SupportFunction& support_b,
                                    const Vec3& initial_direction = Vec3::UnitX(), int max_iterations = 64);

}  // namespace mc
