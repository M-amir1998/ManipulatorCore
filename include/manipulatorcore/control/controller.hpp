#pragma once

/// @file controller.hpp
/// @brief Controller interface, references and measured inputs.
///
/// Controllers map the measured state (and, for interaction controllers, the measured external
/// wrench) to joint torques. They are deliberately free of actuator effects: torque saturation,
/// joint friction and sensor models belong to the plant (see the simulation module).

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/math/types.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/joint_trajectory.hpp"

#include <functional>
#include <memory>
#include <string>

namespace mc {

/// Measured quantities available to a controller at one control instant.
struct ControllerInput {
  double time = 0.0;
  VecX q;
  VecX qd;
  /// External wrench [moment; force] acting on the end-effector, expressed in the base frame and
  /// referred to the end-effector origin (e.g. from a wrist force/torque sensor). Zero if unused.
  Vec6 external_wrench = Vec6::Zero();
};

struct JointReference {
  VecX q;
  VecX qd;
  VecX qdd;
};

/// Desired end-effector motion; twist and acceleration are world-aligned [omega; pdot].
struct CartesianReference {
  SE3 pose;
  Vec6 twist = Vec6::Zero();
  Vec6 acceleration = Vec6::Zero();
};

using JointReferenceFn = std::function<JointReference(double time)>;
using CartesianReferenceFn = std::function<CartesianReference(double time)>;

/// Constant joint setpoint (zero velocity and acceleration).
[[nodiscard]] JointReferenceFn constantJointReference(const VecX& q);

/// Samples a joint trajectory (holding the final point after it ends).
[[nodiscard]] JointReferenceFn trajectoryReference(std::shared_ptr<const JointTrajectory> trajectory);

/// Constant end-effector pose.
[[nodiscard]] CartesianReferenceFn constantPoseReference(const SE3& pose);

/// Samples a Cartesian trajectory.
[[nodiscard]] CartesianReferenceFn cartesianTrajectoryReference(std::shared_ptr<const CartesianTrajectory> trajectory);

class Controller {
 public:
  Controller() = default;
  Controller(const Controller&) = default;
  Controller(Controller&&) = default;
  Controller& operator=(const Controller&) = default;
  Controller& operator=(Controller&&) = default;
  virtual ~Controller() = default;

  [[nodiscard]] virtual std::string name() const = 0;
  /// Joint torque command. Stateful controllers (integrators, admittance filters) update their
  /// internal state here, so calls must be made in increasing time order.
  [[nodiscard]] virtual VecX computeTorque(const ControllerInput& input) = 0;
  /// Clears internal state (integrators, filters).
  virtual void reset() {}
};

}  // namespace mc
