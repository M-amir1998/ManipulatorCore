#pragma once

/// @file task_space_controllers.hpp
/// @brief Task-space controllers: Cartesian PD, impedance, admittance and operational-space control.
///
/// All controllers use the world-aligned pose error e = [log(R_d R^T); p_d - p] and the geometric
/// Jacobian J_g, so gains are 6x6 matrices ordered [rotation; translation]. Redundant arms receive
/// a secondary posture task in the null space of the primary task.

#include "manipulatorcore/control/controller.hpp"
#include "manipulatorcore/model/robot_model.hpp"
#include "manipulatorcore/optimization/objectives.hpp"

namespace mc {

/// Secondary joint-space task executed in the null space of the Cartesian task.
struct NullSpaceTask {
  VecX posture;  ///< desired posture (empty: no posture stiffness)
  double stiffness = 0.0;  ///< [N·m/rad]
  double damping = 0.0;    ///< [N·m·s/rad]
  /// Additional smooth joint-limit repulsion: tau += -limit_gain * grad(activation penalty).
  double joint_limit_gain = 0.0;
  double joint_limit_activation = 0.1;  ///< fraction of the range near each limit
};

/// Diagonal 6x6 gain from rotational and translational scalars.
[[nodiscard]] Mat6 diagonalGain(double rotational, double translational);

/// Cartesian PD with gravity compensation (Jacobian-transpose control):
///   tau = J^T (Kp e + Kd (V_d - V)) + g(q) + N^T tau_null.
/// Simple and robust near singularities (no inversion), but the closed-loop task dynamics depend on
/// the configuration-dependent task inertia.
class TaskSpacePDController final : public Controller {
 public:
  TaskSpacePDController(RobotModel model, Mat6 kp, Mat6 kd, CartesianReferenceFn reference, NullSpaceTask null_space = {});
  [[nodiscard]] std::string name() const override { return "task-space PD"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;

 private:
  RobotModel model_;
  Mat6 kp_;
  Mat6 kd_;
  CartesianReferenceFn reference_;
  NullSpaceTask null_space_;
};

/// Cartesian impedance control without inertia shaping (no force sensing required):
///   tau = J^T (K e + D (V_d - V)) + C(q, qd) qd + g(q) + N^T tau_null.
/// The end-effector then behaves like a spring K and damper D around the desired pose; when the
/// environment pushes back, the steady-state interaction wrench is K e. The apparent inertia is
/// the robot's own task-space inertia.
class CartesianImpedanceController final : public Controller {
 public:
  CartesianImpedanceController(RobotModel model, Mat6 stiffness, Mat6 damping, CartesianReferenceFn reference,
                               NullSpaceTask null_space = {});
  [[nodiscard]] std::string name() const override { return "Cartesian impedance"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;

 private:
  RobotModel model_;
  Mat6 stiffness_;
  Mat6 damping_;
  CartesianReferenceFn reference_;
  NullSpaceTask null_space_;
};

/// Parameters of the admittance model M_d xedd + D_d xedot + K_d xe = F_ext acting on the
/// translational deviation xe = x_c - x_d of the commanded position from the desired one.
struct AdmittanceParameters {
  Vec3 mass = Vec3::Constant(2.0);        ///< [kg]
  Vec3 damping = Vec3::Constant(60.0);    ///< [N·s/m]
  Vec3 stiffness = Vec3::Constant(300.0); ///< [N/m]
};

/// Admittance control: the measured external force drives a virtual mass-spring-damper that
/// shifts the commanded position x_c away from the desired x_d; a resolved-acceleration inner loop
/// tracks x_c and cancels the measured wrench:
///   qdd_ref = J^# (a_c + Kd (V_c - V) + Kp e_c - Jdot qd) + null-space damping,
///   tau = RNEA(q, qd, qdd_ref) - J^T F_ext.
/// The orientation tracks the desired orientation. Requires ControllerInput::external_wrench.
/// Like any admittance scheme, stability degrades for very stiff environments combined with
/// low virtual mass; see docs/algorithms/control.md.
class AdmittanceController final : public Controller {
 public:
  AdmittanceController(RobotModel model, AdmittanceParameters admittance, Mat6 inner_kp, Mat6 inner_kd,
                       CartesianReferenceFn reference, double null_space_damping = 2.0);
  [[nodiscard]] std::string name() const override { return "admittance"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;
  void reset() override;
  /// Current admittance deviation x_c - x_d [m].
  [[nodiscard]] const Vec3& deviation() const { return xe_; }

 private:
  RobotModel model_;
  AdmittanceParameters params_;
  Mat6 kp_;
  Mat6 kd_;
  CartesianReferenceFn reference_;
  double null_damping_;
  Vec3 xe_ = Vec3::Zero();
  Vec3 xe_dot_ = Vec3::Zero();
  double last_time_ = 0.0;
  bool initialized_ = false;
};

/// Operational-space control (O. Khatib, "A unified approach for motion and force control of robot
/// manipulators: the operational space formulation", IEEE J. Robotics and Automation, 1987):
///   Lambda = (J M^{-1} J^T)^{-1},  Jbar = M^{-1} J^T Lambda
///   F = Lambda (a* - Jdot qd) + Jbar^T h,   a* = a_d + Kd (V_d - V) + Kp e
///   tau = J^T F + (I - J^T Jbar^T) tau_0
/// With an exact model the task acceleration equals a*, and tau_0 produces no task acceleration
/// (dynamically consistent null space). Near singularities Lambda is regularized by clamping the
/// eigenvalues of J M^{-1} J^T from below (singularity_threshold), which bounds the task force.
class OperationalSpaceController final : public Controller {
 public:
  OperationalSpaceController(RobotModel model, Mat6 kp, Mat6 kd, CartesianReferenceFn reference,
                             NullSpaceTask null_space = {}, double singularity_threshold = 1e-4);
  [[nodiscard]] std::string name() const override { return "operational space"; }
  [[nodiscard]] VecX computeTorque(const ControllerInput& input) override;

 private:
  RobotModel model_;
  Mat6 kp_;
  Mat6 kd_;
  CartesianReferenceFn reference_;
  NullSpaceTask null_space_;
  double threshold_;
};

/// Quantities of the operational-space formulation at a state (exposed for analysis and tests).
struct OperationalSpaceTerms {
  Mat6X J;          ///< geometric Jacobian
  MatX M;           ///< joint-space mass matrix
  Mat6 Lambda;      ///< task-space inertia
  MatX Jbar;        ///< dynamically consistent generalized inverse M^{-1} J^T Lambda (n x 6)
  MatX null_space;  ///< I - J^T Jbar^T (n x n), maps tau_0 to null-space torques
  Vec6 Jdot_qd;     ///< Jdot qd
};

[[nodiscard]] OperationalSpaceTerms operationalSpaceTerms(const RobotModel& model, const VecX& q, const VecX& qd,
                                                          double singularity_threshold = 1e-4);

}  // namespace mc
