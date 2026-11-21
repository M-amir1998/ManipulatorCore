#include "manipulatorcore/control/task_space_controllers.hpp"

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <utility>

namespace mc {

Mat6 diagonalGain(double rotational, double translational) {
  Vec6 d;
  d << Vec3::Constant(rotational), Vec3::Constant(translational);
  return d.asDiagonal();
}

namespace {

void validateGain(const Mat6& K, const char* what) {
  require(K.allFinite() && isSymmetric(K, 1e-12), std::string(what) + " must be finite and symmetric");
}

void validateNullSpace(const RobotModel& model, const NullSpaceTask& task) {
  if (task.posture.size() > 0) {
    requireVector(task.posture, model.dof(), "NullSpaceTask: posture");
  }
  require(task.stiffness >= 0.0 && task.damping >= 0.0 && task.joint_limit_gain >= 0.0,
          "NullSpaceTask: gains must be non-negative");
}

/// Secondary torque tau_0 of a posture task (before projection).
VecX postureTorque(const RobotModel& model, const VecX& q, const VecX& qd, const NullSpaceTask& task) {
  VecX tau0 = -task.damping * qd;
  if (task.posture.size() > 0) {
    tau0 += task.stiffness * (task.posture - q);
  }
  if (task.joint_limit_gain > 0.0) {
    tau0 -= task.joint_limit_gain * jointLimitActivationObjective(model, q, task.joint_limit_activation).gradient;
  }
  return tau0;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Task-space PD and impedance
// ------------------------------------------------------------------------------------------------

TaskSpacePDController::TaskSpacePDController(RobotModel model, Mat6 kp, Mat6 kd, CartesianReferenceFn reference,
                                             NullSpaceTask null_space)
    : model_(std::move(model)), kp_(kp), kd_(kd), reference_(std::move(reference)), null_space_(std::move(null_space)) {
  validateGain(kp_, "TaskSpacePDController: kp");
  validateGain(kd_, "TaskSpacePDController: kd");
  validateNullSpace(model_, null_space_);
  require(static_cast<bool>(reference_), "TaskSpacePDController: reference must be set");
}

VecX TaskSpacePDController::computeTorque(const ControllerInput& input) {
  const KinematicState state = forwardKinematics(model_, input.q);
  const Mat6X J = geometricJacobian(model_, state);
  const CartesianReference ref = reference_(input.time);
  const Vec6 e = poseErrorWorld(state.T_base_ee, ref.pose);
  const Vec6 wrench = kp_ * e + kd_ * (ref.twist - J * input.qd);
  return J.transpose() * wrench + gravityTorques(model_, input.q) +
         nullSpaceProjector(J) * postureTorque(model_, input.q, input.qd, null_space_);
}

CartesianImpedanceController::CartesianImpedanceController(RobotModel model, Mat6 stiffness, Mat6 damping,
                                                           CartesianReferenceFn reference, NullSpaceTask null_space)
    : model_(std::move(model)),
      stiffness_(stiffness),
      damping_(damping),
      reference_(std::move(reference)),
      null_space_(std::move(null_space)) {
  validateGain(stiffness_, "CartesianImpedanceController: stiffness");
  validateGain(damping_, "CartesianImpedanceController: damping");
  validateNullSpace(model_, null_space_);
  require(static_cast<bool>(reference_), "CartesianImpedanceController: reference must be set");
}

VecX CartesianImpedanceController::computeTorque(const ControllerInput& input) {
  const KinematicState state = forwardKinematics(model_, input.q);
  const Mat6X J = geometricJacobian(model_, state);
  const CartesianReference ref = reference_(input.time);
  const Vec6 e = poseErrorWorld(state.T_base_ee, ref.pose);
  const Vec6 wrench = stiffness_ * e + damping_ * (ref.twist - J * input.qd);
  return J.transpose() * wrench + biasTorques(model_, input.q, input.qd) +
         nullSpaceProjector(J) * postureTorque(model_, input.q, input.qd, null_space_);
}

// ------------------------------------------------------------------------------------------------
// Admittance
// ------------------------------------------------------------------------------------------------

AdmittanceController::AdmittanceController(RobotModel model, AdmittanceParameters admittance, Mat6 inner_kp,
                                           Mat6 inner_kd, CartesianReferenceFn reference, double null_space_damping)
    : model_(std::move(model)),
      params_(admittance),
      kp_(inner_kp),
      kd_(inner_kd),
      reference_(std::move(reference)),
      null_damping_(null_space_damping) {
  require((params_.mass.array() > 0.0).all() && (params_.damping.array() >= 0.0).all() &&
              (params_.stiffness.array() >= 0.0).all(),
          "AdmittanceController: mass must be positive, damping and stiffness non-negative");
  validateGain(kp_, "AdmittanceController: inner kp");
  validateGain(kd_, "AdmittanceController: inner kd");
  require(null_damping_ >= 0.0, "AdmittanceController: null-space damping must be non-negative");
  require(static_cast<bool>(reference_), "AdmittanceController: reference must be set");
}

void AdmittanceController::reset() {
  xe_.setZero();
  xe_dot_.setZero();
  initialized_ = false;
}

VecX AdmittanceController::computeTorque(const ControllerInput& input) {
  const double dt = initialized_ ? input.time - last_time_ : 0.0;
  require(dt >= 0.0, "AdmittanceController: time must not decrease");
  last_time_ = input.time;
  initialized_ = true;

  // Admittance filter M xedd + D xedot + K xe = F_ext (semi-implicit Euler).
  const Vec3 force = input.external_wrench.tail<3>();
  const Vec3 xe_ddot =
      (force - params_.damping.cwiseProduct(xe_dot_) - params_.stiffness.cwiseProduct(xe_)).cwiseQuotient(params_.mass);
  if (dt > 0.0) {
    xe_dot_ += dt * xe_ddot;
    xe_ += dt * xe_dot_;
  }

  const CartesianReference ref = reference_(input.time);
  SE3 commanded = ref.pose;
  commanded.translation() += xe_;
  Vec6 V_c = ref.twist;
  V_c.tail<3>() += xe_dot_;
  Vec6 a_c = ref.acceleration;
  a_c.tail<3>() += xe_ddot;

  // Inner loop: resolved-acceleration (task-space inverse dynamics) tracking of the commanded pose.
  const KinematicState state = forwardKinematics(model_, input.q);
  const Mat6X J = geometricJacobian(model_, state);
  const Vec6 e = poseErrorWorld(state.T_base_ee, commanded);
  const Vec6 a_task =
      a_c + kd_ * (V_c - J * input.qd) + kp_ * e - geometricJacobianDotTimesVelocity(model_, input.q, input.qd);
  VecX qdd = solveDampedLeastSquares(J, a_task, 1e-3);
  qdd += nullSpaceProjector(J) * (-null_damping_ * input.qd);
  // Cancelling the measured wrench makes the inner loop an (ideal) motion source, so the
  // interaction compliance is defined by the admittance parameters alone.
  return inverseDynamics(model_, input.q, input.qd, qdd) - J.transpose() * input.external_wrench;
}

// ------------------------------------------------------------------------------------------------
// Operational space
// ------------------------------------------------------------------------------------------------

OperationalSpaceTerms operationalSpaceTerms(const RobotModel& model, const VecX& q, const VecX& qd,
                                            double singularity_threshold) {
  require(singularity_threshold > 0.0, "operationalSpaceTerms: singularity threshold must be positive");
  OperationalSpaceTerms t;
  t.J = geometricJacobian(model, q);
  t.M = massMatrix(model, q);
  const Eigen::LLT<MatX> llt(t.M);
  require(llt.info() == Eigen::Success, "operationalSpaceTerms: mass matrix is not positive definite");
  const MatX Minv_JT = llt.solve(MatX(t.J.transpose()));
  const Mat6 Lambda_inv = t.J * Minv_JT;
  // Lambda = (J M^-1 J^T)^-1 with eigenvalues clamped from below near singularities.
  const Eigen::SelfAdjointEigenSolver<Mat6> eig(0.5 * (Lambda_inv + Lambda_inv.transpose()));
  const Vec6 inv_eigen = eig.eigenvalues().cwiseMax(singularity_threshold).cwiseInverse();
  t.Lambda = eig.eigenvectors() * inv_eigen.asDiagonal() * eig.eigenvectors().transpose();
  t.Jbar = Minv_JT * t.Lambda;
  t.null_space = MatX::Identity(model.dof(), model.dof()) - t.J.transpose() * t.Jbar.transpose();
  t.Jdot_qd = geometricJacobianDotTimesVelocity(model, q, qd);
  return t;
}

OperationalSpaceController::OperationalSpaceController(RobotModel model, Mat6 kp, Mat6 kd,
                                                       CartesianReferenceFn reference, NullSpaceTask null_space,
                                                       double singularity_threshold)
    : model_(std::move(model)),
      kp_(kp),
      kd_(kd),
      reference_(std::move(reference)),
      null_space_(std::move(null_space)),
      threshold_(singularity_threshold) {
  validateGain(kp_, "OperationalSpaceController: kp");
  validateGain(kd_, "OperationalSpaceController: kd");
  validateNullSpace(model_, null_space_);
  require(threshold_ > 0.0, "OperationalSpaceController: singularity threshold must be positive");
  require(static_cast<bool>(reference_), "OperationalSpaceController: reference must be set");
}

VecX OperationalSpaceController::computeTorque(const ControllerInput& input) {
  const OperationalSpaceTerms t = operationalSpaceTerms(model_, input.q, input.qd, threshold_);
  const CartesianReference ref = reference_(input.time);
  const SE3 T = endEffectorPose(model_, input.q);
  const Vec6 a_star = ref.acceleration + kd_ * (ref.twist - t.J * input.qd) + kp_ * poseErrorWorld(T, ref.pose);
  const VecX h = biasTorques(model_, input.q, input.qd);
  const Vec6 F = t.Lambda * (a_star - t.Jdot_qd) + t.Jbar.transpose() * h;
  return t.J.transpose() * F + t.null_space * postureTorque(model_, input.q, input.qd, null_space_);
}

}  // namespace mc
