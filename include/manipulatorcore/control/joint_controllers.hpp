#pragma once

/// @file joint_controllers.hpp
/// @brief Joint-space controllers: gravity compensation, PID / PD (+ gravity), computed torque.

#include "manipulatorcore/control/controller.hpp"
#include "manipulatorcore/model/robot_model.hpp"

namespace mc {

/// tau = g(q): holds any configuration against gravity when the model is exact (no damping, so
/// any initial velocity is preserved).
class GravityCompensationController final : public Controller {
 public:
  explicit GravityCompensationController(RobotModel model) : model_(std::move(model)) {}
  [[nodiscard]] std::string name() const override { return "gravity compensation"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;

 private:
  RobotModel model_;
};

struct JointPIDGains {
  VecX kp;  ///< [N·m/rad]
  VecX ki;  ///< [N·m/(rad·s)]; empty or zero for PD
  VecX kd;  ///< [N·m·s/rad]
};

/// Independent-joint PID: tau = Kp e + Ki integral(e) + Kd edot [+ g(q)], e = q_d - q.
///
/// With gravity_compensation = false and Ki = 0 this is the classical PD law, which leaves a
/// steady-state error e = Kp^{-1} g(q) under gravity. Adding g(q) (PD + gravity compensation)
/// makes the setpoint globally asymptotically stable for an exact model (Takegaki & Arimoto 1981).
/// The integrator is clamped to +-integral_limit to avoid wind-up.
class JointPIDController final : public Controller {
 public:
  JointPIDController(RobotModel model, JointPIDGains gains, JointReferenceFn reference,
                     bool gravity_compensation = true, VecX integral_limit = VecX());

  [[nodiscard]] std::string name() const override;
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;
  void reset() override;

 private:
  RobotModel model_;
  JointPIDGains gains_;
  JointReferenceFn reference_;
  bool gravity_compensation_;
  VecX integral_limit_;
  VecX integral_;
  double last_time_ = 0.0;
  bool initialized_ = false;
};

/// Computed-torque (inverse-dynamics) control:
///   tau = M(q) (qdd_d + Kd edot + Kp e) + C(q, qd) qd + g(q),
/// evaluated with one RNEA call. With an exact model the closed loop is the decoupled linear
/// error dynamics edd + Kd edot + Kp e = 0. A different (e.g. perturbed) model can be supplied to
/// study robustness.
class ComputedTorqueController final : public Controller {
 public:
  ComputedTorqueController(RobotModel model, VecX kp, VecX kd, JointReferenceFn reference);
  [[nodiscard]] std::string name() const override { return "computed torque"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;

  /// Gains for critically damped error dynamics with natural frequency omega [rad/s] per joint.
  static void criticallyDampedGains(double omega, int dof, VecX& kp, VecX& kd);

 private:
  RobotModel model_;
  VecX kp_;
  VecX kd_;
  JointReferenceFn reference_;
};

}  // namespace mc
