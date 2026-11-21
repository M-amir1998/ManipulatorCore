#include "manipulatorcore/control/joint_controllers.hpp"

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/math/error.hpp"

#include <limits>
#include <utility>

namespace mc {

VecX GravityCompensationController::computeTorque(const ControllerInput& input) {
  return gravityTorques(model_, input.q);
}

JointPIDController::JointPIDController(RobotModel model, JointPIDGains gains, JointReferenceFn reference,
                                       bool gravity_compensation, VecX integral_limit)
    : model_(std::move(model)),
      gains_(std::move(gains)),
      reference_(std::move(reference)),
      gravity_compensation_(gravity_compensation),
      integral_limit_(std::move(integral_limit)) {
  const int n = model_.dof();
  requireVector(gains_.kp, n, "JointPIDController: kp");
  requireVector(gains_.kd, n, "JointPIDController: kd");
  if (gains_.ki.size() == 0) {
    gains_.ki = VecX::Zero(n);
  }
  requireVector(gains_.ki, n, "JointPIDController: ki");
  if (integral_limit_.size() == 0) {
    integral_limit_ = VecX::Constant(n, std::numeric_limits<double>::infinity());
  }
  require(integral_limit_.size() == n && (integral_limit_.array() >= 0.0).all(),
          "JointPIDController: integral limits must be non-negative");
  require(static_cast<bool>(reference_), "JointPIDController: reference must be set");
  integral_ = VecX::Zero(n);
}

std::string JointPIDController::name() const {
  std::string n = gains_.ki.cwiseAbs().maxCoeff() > 0.0 ? "PID" : "PD";
  return gravity_compensation_ ? n + " + gravity" : n;
}

void JointPIDController::reset() {
  integral_.setZero();
  initialized_ = false;
}

VecX JointPIDController::computeTorque(const ControllerInput& input) {
  model_.checkConfiguration(input.q, "JointPIDController: q");
  const JointReference ref = reference_(input.time);
  const VecX e = ref.q - input.q;
  const VecX e_dot = ref.qd - input.qd;
  const double dt = initialized_ ? input.time - last_time_ : 0.0;
  require(dt >= 0.0, "JointPIDController: time must not decrease");
  last_time_ = input.time;
  initialized_ = true;
  integral_ = (integral_ + dt * e).cwiseMax(-integral_limit_).cwiseMin(integral_limit_);
  VecX tau = gains_.kp.cwiseProduct(e) + gains_.ki.cwiseProduct(integral_) + gains_.kd.cwiseProduct(e_dot);
  if (gravity_compensation_) {
    tau += gravityTorques(model_, input.q);
  }
  return tau;
}

ComputedTorqueController::ComputedTorqueController(RobotModel model, VecX kp, VecX kd, JointReferenceFn reference)
    : model_(std::move(model)), kp_(std::move(kp)), kd_(std::move(kd)), reference_(std::move(reference)) {
  requireVector(kp_, model_.dof(), "ComputedTorqueController: kp");
  requireVector(kd_, model_.dof(), "ComputedTorqueController: kd");
  require(static_cast<bool>(reference_), "ComputedTorqueController: reference must be set");
}

VecX ComputedTorqueController::computeTorque(const ControllerInput& input) {
  const JointReference ref = reference_(input.time);
  const VecX a = ref.qdd + kd_.cwiseProduct(ref.qd - input.qd) + kp_.cwiseProduct(ref.q - input.q);
  return inverseDynamics(model_, input.q, input.qd, a);  // M a + C qd + g in one RNEA pass
}

void ComputedTorqueController::criticallyDampedGains(double omega, int dof, VecX& kp, VecX& kd) {
  require(omega > 0.0 && dof > 0, "criticallyDampedGains: omega and dof must be positive");
  kp = VecX::Constant(dof, omega * omega);
  kd = VecX::Constant(dof, 2.0 * omega);
}

}  // namespace mc
