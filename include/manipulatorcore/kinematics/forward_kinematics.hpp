#pragma once

/// @file forward_kinematics.hpp
/// @brief Forward kinematics of the general serial chain.

#include "manipulatorcore/model/robot_model.hpp"

#include <vector>

namespace mc {

/// Poses of every body frame and of the end-effector at a configuration.
struct KinematicState {
  VecX q;                         ///< configuration the state was computed at
  std::vector<SE3> T_base_body;   ///< T_base_body[i]: pose of body i in the base frame
  SE3 T_base_ee;                  ///< end-effector (tool) pose in the base frame

  /// World-frame unit axis of joint i.
  [[nodiscard]] Vec3 jointAxis(const RobotModel& model, int i) const;

  /// World-frame origin of joint i (equal to the origin of body i).
  [[nodiscard]] Vec3 jointOrigin(int i) const { return T_base_body[static_cast<std::size_t>(i)].translation(); }
};

/// Computes forward kinematics into @p state, reusing its storage.
void computeForwardKinematics(const RobotModel& model, const VecX& q, KinematicState& state);

/// Forward kinematics for all bodies and the end-effector.
[[nodiscard]] KinematicState forwardKinematics(const RobotModel& model, const VecX& q);

/// End-effector pose T_base_ee(q).
[[nodiscard]] SE3 endEffectorPose(const RobotModel& model, const VecX& q);

}  // namespace mc
