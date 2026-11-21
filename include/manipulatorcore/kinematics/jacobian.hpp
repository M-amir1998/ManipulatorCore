#pragma once

/// @file jacobian.hpp
/// @brief Space, body and geometric Jacobians of serial chains.
///
/// All Jacobians are 6 x n with rows ordered [omega; v]:
///  - space Jacobian J_s:     V_s = J_s qd       (Modern Robotics space twist)
///  - body Jacobian J_b:      V_b = J_b qd       (end-effector frame)
///  - geometric Jacobian J_g: [omega_s; pdot_ee] = J_g qd (EE-origin velocity in the base frame)
/// See docs/mathematics/conventions.md.

#include "manipulatorcore/kinematics/forward_kinematics.hpp"

namespace mc {

[[nodiscard]] Mat6X spaceJacobian(const RobotModel& model, const KinematicState& state);
[[nodiscard]] Mat6X spaceJacobian(const RobotModel& model, const VecX& q);

[[nodiscard]] Mat6X bodyJacobian(const RobotModel& model, const KinematicState& state);
[[nodiscard]] Mat6X bodyJacobian(const RobotModel& model, const VecX& q);

[[nodiscard]] Mat6X geometricJacobian(const RobotModel& model, const KinematicState& state);
[[nodiscard]] Mat6X geometricJacobian(const RobotModel& model, const VecX& q);

/// Geometric Jacobian of a point fixed to body @p body (point given in the body frame):
/// [omega_body; pdot_point] = J qd, base frame. Columns of joints after @p body are zero.
[[nodiscard]] Mat6X pointJacobian(const RobotModel& model, const KinematicState& state, int body,
                                  const Vec3& point_in_body);

/// Bias term Jdot_g(q, qd) qd of the geometric Jacobian: the end-effector [angular; linear]
/// acceleration produced by qd when qdd = 0. Needed for task-space inverse dynamics.
[[nodiscard]] Vec6 geometricJacobianDotTimesVelocity(const RobotModel& model, const VecX& q, const VecX& qd);

/// Rows (vx, vy, omega_z) of a geometric Jacobian: the planar task Jacobian for arms moving in
/// the base x-y plane.
[[nodiscard]] MatX planarTaskJacobian(const Mat6X& geometric_jacobian);

}  // namespace mc
