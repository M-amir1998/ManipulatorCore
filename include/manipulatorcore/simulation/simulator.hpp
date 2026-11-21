#pragma once

/// @file simulator.hpp
/// @brief Lightweight, deterministic, CPU-only manipulator simulator.
///
/// The plant is the rigid-body model of the robot plus:
///  - viscous joint damping (Joint::damping),
///  - actuator torque saturation at the effort limits,
///  - stiff spring-damper joint-limit stops,
///  - compliant contact between one robot point (a small sphere) and planar surfaces:
///      F_n = max(0, k delta - c delta_dot) along the surface normal, optional viscous tangential
///      friction. This is a penalty model for qualitative interaction studies, not a substitute for
///      a full contact solver (no Coulomb cone, no multi-point contact).
///
/// The controller is sampled once per step (zero-order hold); contact, damping and limit forces are
/// re-evaluated at every integrator stage. Given the same model, options, initial state and
/// controller, two runs produce bit-identical logs. Explicit integration is conditionally stable:
/// stiff contacts and high gains require small time steps (see docs/algorithms/simulation.md).

#include "manipulatorcore/control/controller.hpp"
#include "manipulatorcore/dynamics/integrators.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <string>
#include <vector>

namespace mc {

/// Planar compliant surface; the solid occupies {x : normal . x <= offset}.
struct ContactSurface {
  Vec3 normal = Vec3::UnitZ();
  double offset = 0.0;
  double stiffness = 5000.0;        ///< [N/m]
  double damping = 50.0;            ///< [N·s/m]
  double tangential_damping = 0.0;  ///< viscous friction [N·s/m]
  std::string name;
};

/// The robot point that can touch the surfaces: a sphere attached to a body.
struct ContactPoint {
  int body = -1;              ///< body index; -1 selects the last body
  Vec3 point = Vec3::Zero();  ///< in the body frame; for body -1 this is relative to the tool frame
  double radius = 0.0;        ///< [m]
};

struct SimulatorOptions {
  double dt = 1e-3;
  Integrator integrator = Integrator::RK4;
  bool joint_damping = true;
  bool saturate_torques = true;
  bool joint_limit_stops = true;
  double joint_limit_stiffness = 5000.0;  ///< [N·m/rad]
  double joint_limit_damping = 50.0;      ///< [N·m·s/rad]
  std::vector<ContactSurface> contacts;
  ContactPoint contact_point;
  int record_every = 1;  ///< log every k-th step
};

struct SimulationLog {
  std::vector<double> time;
  std::vector<VecX> q;
  std::vector<VecX> qd;
  std::vector<VecX> tau;  ///< applied (saturated) actuator torque
  std::vector<SE3> ee_pose;
  std::vector<Vec3> contact_force;  ///< total contact force on the contact point (base frame)
  void clear();
};

class Simulator {
 public:
  explicit Simulator(RobotModel model, SimulatorOptions options = {});

  void reset(const VecX& q0, const VecX& qd0 = VecX());
  [[nodiscard]] double time() const { return time_; }
  [[nodiscard]] const VecX& q() const { return q_; }
  [[nodiscard]] const VecX& qd() const { return qd_; }
  [[nodiscard]] const RobotModel& model() const { return model_; }
  [[nodiscard]] const SimulatorOptions& options() const { return options_; }

  /// Contact force acting on the robot's contact point at a state (base frame).
  [[nodiscard]] Vec3 contactForce(const VecX& q, const VecX& qd) const;

  /// Wrench [moment; force] of the contact about the end-effector origin (what a wrist F/T sensor
  /// would report, expressed in the base frame).
  [[nodiscard]] Vec6 endEffectorWrench(const VecX& q, const VecX& qd) const;

  /// Joint acceleration of the plant for an applied actuator torque.
  [[nodiscard]] VecX plantAcceleration(const VecX& q, const VecX& qd, const VecX& tau) const;

  /// Advances one time step under @p controller.
  void step(Controller& controller);

  /// Runs for @p duration seconds (rounded to whole steps) and returns the accumulated log.
  const SimulationLog& run(Controller& controller, double duration);

  [[nodiscard]] const SimulationLog& log() const { return log_; }

 private:
  void record(const VecX& tau);
  [[nodiscard]] VecX externalTorque(const VecX& q, const VecX& qd) const;

  RobotModel model_;
  SimulatorOptions options_;
  int contact_body_ = 0;
  Vec3 contact_point_in_body_ = Vec3::Zero();
  double time_ = 0.0;
  VecX q_;
  VecX qd_;
  long long steps_ = 0;
  SimulationLog log_;
};

}  // namespace mc
