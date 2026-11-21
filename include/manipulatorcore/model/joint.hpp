#pragma once

/// @file joint.hpp
/// @brief Single-degree-of-freedom joints of a serial manipulator.

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/math/types.hpp"

#include <string>

namespace mc {

enum class JointType {
  Revolute,  ///< rotation about the joint axis [rad]
  Prismatic  ///< translation along the joint axis [m]
};

/// Position, velocity, acceleration and effort limits of one joint. Units are rad / rad/s / rad/s^2
/// / N·m for revolute joints and m / m/s / m/s^2 / N for prismatic joints.
struct JointLimits {
  double lower = -kPi;
  double upper = kPi;
  double velocity = 2.0;
  double acceleration = 10.0;
  double effort = 100.0;
};

/// A joint connecting a parent frame (the base or the previous body) to its child body.
///
/// T_parent_body(q) = origin * motion(q), where origin is the joint frame at q = 0 written in the
/// parent frame and motion(q) is a rotation about (or translation along) the unit axis.
struct Joint {
  std::string name;
  JointType type = JointType::Revolute;
  SE3 origin;                 ///< T_parent_joint at q = 0
  Vec3 axis = Vec3::UnitZ();  ///< unit axis in the joint frame
  JointLimits limits;
  double damping = 0.0;   ///< viscous friction [N·m·s/rad or N·s/m]; used by the simulator only
  double armature = 0.0;  ///< reflected rotor inertia [kg·m² or kg], added to the mass-matrix diagonal

  /// Joint motion T_joint_body(q).
  [[nodiscard]] SE3 motion(double q) const;

  /// Pose of the child body in the parent frame, origin * motion(q).
  [[nodiscard]] SE3 transform(double q) const;

  /// Motion subspace S (6-vector, [omega; v], child frame): [axis; 0] or [0; axis].
  [[nodiscard]] Vec6 motionSubspace() const;

  /// Throws InvalidArgument for non-unit axes, inverted or non-finite limits, or negative
  /// velocity/acceleration/effort/damping/armature.
  void validate() const;
};

}  // namespace mc
