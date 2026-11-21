#pragma once

/// @file analytical_ik.hpp
/// @brief Closed-form inverse kinematics for planar 2R and 3R arms.
///
/// Planar arms move in the x-y plane with joint angles measured counter-clockwise about +z.
/// Reference: M. W. Spong, S. Hutchinson and M. Vidyasagar, Robot Modeling and Control, 2006,
/// section 3.3.

#include "manipulatorcore/model/robot_model.hpp"

#include <vector>

namespace mc {

/// All solutions (q1, q2) placing the tip of a 2R arm with link lengths l1, l2 at @p target.
/// Two solutions (q2 > 0 first, then q2 < 0) inside the workspace annulus, one on its boundary,
/// none outside. Angles are wrapped to (-pi, pi]. Targets within @p boundary_tolerance of the
/// boundary are snapped onto it.
[[nodiscard]] std::vector<Vec2> planar2RInverseKinematics(double l1, double l2, const Vec2& target,
                                                          double boundary_tolerance = 1e-12);

/// All solutions (q1, q2, q3) placing the tip of a 3R arm at @p target with tool orientation
/// @p phi = q1 + q2 + q3 (the wrist point is solved as a 2R problem).
[[nodiscard]] std::vector<Vec3> planar3RInverseKinematics(double l1, double l2, double l3, const Vec2& target,
                                                          double phi, double boundary_tolerance = 1e-12);

/// Link lengths of a planar chain built like makePlanar2R()/makePlanar3R(). Throws if the model is
/// not a planar revolute chain about +z with links along x.
[[nodiscard]] std::vector<double> planarLinkLengths(const RobotModel& model);

/// Model-based 2R IK. When @p respect_limits is true, solutions outside the joint limits are dropped.
[[nodiscard]] std::vector<VecX> analyticalIKPlanar2R(const RobotModel& model, const Vec2& target,
                                                     bool respect_limits = true);

/// Model-based 3R IK with tool orientation @p phi.
[[nodiscard]] std::vector<VecX> analyticalIKPlanar3R(const RobotModel& model, const Vec2& target, double phi,
                                                     bool respect_limits = true);

}  // namespace mc
