#pragma once

/// @file robot_model.hpp
/// @brief Serial-manipulator model: joints, inertial properties, collision geometry, tool frame.

#include "manipulatorcore/geometry/shapes.hpp"
#include "manipulatorcore/model/inertia.hpp"
#include "manipulatorcore/model/joint.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace mc {

/// One moving body of the chain and the joint that attaches it to its parent.
struct Body {
  std::string name;
  Joint joint;                   ///< connects the parent (base or previous body) to this body
  Inertia inertia;               ///< body frame
  std::vector<Shape> collision;  ///< collision primitives in the body frame
};

/// An n-joint serial manipulator.
///
/// Body i is attached to body i-1 (or the base for i = 0) by joint i. The frame of body i is the
/// joint frame after the joint motion. The end-effector frame is T_flange_ee relative to the last
/// body. All conventions are documented in docs/mathematics/conventions.md.
///
/// The model is an immutable-by-default value type; mutators re-validate their input.
class RobotModel {
 public:
  RobotModel(std::string name, std::vector<Body> bodies, const SE3& T_flange_ee = SE3::Identity(),
             const Vec3& gravity = Vec3(0.0, 0.0, -kStandardGravity));

  [[nodiscard]] const std::string& name() const { return name_; }

  /// Number of joints (degrees of freedom).
  [[nodiscard]] int dof() const { return static_cast<int>(bodies_.size()); }

  [[nodiscard]] const std::vector<Body>& bodies() const { return bodies_; }
  [[nodiscard]] const Body& body(int i) const;
  [[nodiscard]] const Joint& joint(int i) const { return body(i).joint; }

  /// Index of the body with the given name, or -1.
  [[nodiscard]] int bodyIndex(std::string_view name) const;

  /// Fixed transform from the last body frame to the end-effector (tool) frame.
  [[nodiscard]] const SE3& toolTransform() const { return T_flange_ee_; }
  void setToolTransform(const SE3& T_flange_ee);

  /// Gravitational acceleration in the base frame [m/s^2].
  [[nodiscard]] const Vec3& gravity() const { return gravity_; }
  void setGravity(const Vec3& gravity);

  /// Collision primitives fixed to the base frame (pedestal, mounting plate).
  [[nodiscard]] const std::vector<Shape>& baseCollision() const { return base_collision_; }
  void setBaseCollision(std::vector<Shape> shapes);

  void setBodyInertia(int i, const Inertia& inertia);
  void setBodyCollision(int i, std::vector<Shape> shapes);
  void setJointLimits(int i, const JointLimits& limits);

  [[nodiscard]] const VecX& lowerLimits() const { return lower_; }
  [[nodiscard]] const VecX& upperLimits() const { return upper_; }
  [[nodiscard]] const VecX& velocityLimits() const { return velocity_; }
  [[nodiscard]] const VecX& accelerationLimits() const { return acceleration_; }
  [[nodiscard]] const VecX& effortLimits() const { return effort_; }

  /// True if lower - tolerance <= q <= upper + tolerance component-wise.
  [[nodiscard]] bool withinLimits(const VecX& q, double tolerance = 0.0) const;

  /// Projection of q onto the joint-limit box.
  [[nodiscard]] VecX clampToLimits(const VecX& q) const;

  /// (lower + upper) / 2.
  [[nodiscard]] VecX midConfiguration() const;

  /// The zero vector of size dof() (not necessarily within limits).
  [[nodiscard]] VecX zeroConfiguration() const { return VecX::Zero(dof()); }

  /// Sum of body masses [kg].
  [[nodiscard]] double totalMass() const;

  /// Throws InvalidArgument unless q has dof() finite entries.
  void checkConfiguration(const VecX& q, const char* what = "configuration") const;

 private:
  void validate() const;
  void refreshLimitCache();

  std::string name_;
  std::vector<Body> bodies_;
  SE3 T_flange_ee_;
  Vec3 gravity_;
  std::vector<Shape> base_collision_;
  VecX lower_;
  VecX upper_;
  VecX velocity_;
  VecX acceleration_;
  VecX effort_;
};

}  // namespace mc
