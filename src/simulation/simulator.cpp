#include "manipulatorcore/simulation/simulator.hpp"

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"

#include <cmath>
#include <utility>

namespace mc {

void SimulationLog::clear() {
  time.clear();
  q.clear();
  qd.clear();
  tau.clear();
  ee_pose.clear();
  contact_force.clear();
}

Simulator::Simulator(RobotModel model, SimulatorOptions options)
    : model_(std::move(model)), options_(std::move(options)) {
  require(std::isfinite(options_.dt) && options_.dt > 0.0, "SimulatorOptions: dt must be positive");
  require(options_.record_every >= 1, "SimulatorOptions: record_every must be >= 1");
  require(options_.joint_limit_stiffness >= 0.0 && options_.joint_limit_damping >= 0.0,
          "SimulatorOptions: joint-limit gains must be non-negative");
  for (ContactSurface& c : options_.contacts) {
    require(c.normal.norm() > 1e-12, "ContactSurface: normal must be non-zero");
    const double n = c.normal.norm();
    c.normal /= n;
    c.offset /= n;
    require(c.stiffness >= 0.0 && c.damping >= 0.0 && c.tangential_damping >= 0.0,
            "ContactSurface: gains must be non-negative");
  }
  const ContactPoint& cp = options_.contact_point;
  require(cp.body >= -1 && cp.body < model_.dof(), "ContactPoint: body index out of range");
  require(cp.radius >= 0.0, "ContactPoint: radius must be non-negative");
  if (cp.body < 0) {
    contact_body_ = model_.dof() - 1;
    contact_point_in_body_ = model_.toolTransform() * cp.point;
  } else {
    contact_body_ = cp.body;
    contact_point_in_body_ = cp.point;
  }
  reset(model_.zeroConfiguration());
}

void Simulator::reset(const VecX& q0, const VecX& qd0) {
  model_.checkConfiguration(q0, "Simulator::reset: q0");
  q_ = q0;
  qd_ = qd0.size() == 0 ? VecX::Zero(model_.dof()) : qd0;
  model_.checkConfiguration(qd_, "Simulator::reset: qd0");
  time_ = 0.0;
  steps_ = 0;
  log_.clear();
}

Vec3 Simulator::contactForce(const VecX& q, const VecX& qd) const {
  if (options_.contacts.empty()) {
    return Vec3::Zero();
  }
  const KinematicState state = forwardKinematics(model_, q);
  const Vec3 p = state.T_base_body[static_cast<std::size_t>(contact_body_)] * contact_point_in_body_;
  const Vec3 v = pointJacobian(model_, state, contact_body_, contact_point_in_body_).bottomRows<3>() * qd;
  Vec3 total = Vec3::Zero();
  for (const ContactSurface& c : options_.contacts) {
    const double gap = c.normal.dot(p) - c.offset - options_.contact_point.radius;
    if (gap >= 0.0) {
      continue;
    }
    const double vn = c.normal.dot(v);
    const double fn = std::max(0.0, c.stiffness * (-gap) - c.damping * vn);  // no adhesion
    const Vec3 vt = v - vn * c.normal;
    total += fn * c.normal - c.tangential_damping * vt;
  }
  return total;
}

Vec6 Simulator::endEffectorWrench(const VecX& q, const VecX& qd) const {
  const Vec3 f = contactForce(q, qd);
  Vec6 w = Vec6::Zero();
  if (f.squaredNorm() == 0.0) {
    return w;
  }
  const KinematicState state = forwardKinematics(model_, q);
  const Vec3 p = state.T_base_body[static_cast<std::size_t>(contact_body_)] * contact_point_in_body_;
  w.head<3>() = (p - state.T_base_ee.translation()).cross(f);
  w.tail<3>() = f;
  return w;
}

VecX Simulator::externalTorque(const VecX& q, const VecX& qd) const {
  VecX tau = VecX::Zero(model_.dof());
  if (options_.joint_damping) {
    for (int i = 0; i < model_.dof(); ++i) {
      tau(i) -= model_.joint(i).damping * qd(i);
    }
  }
  if (options_.joint_limit_stops) {
    for (int i = 0; i < model_.dof(); ++i) {
      const double lo = model_.lowerLimits()(i);
      const double hi = model_.upperLimits()(i);
      if (q(i) < lo) {
        tau(i) += options_.joint_limit_stiffness * (lo - q(i)) - options_.joint_limit_damping * std::min(qd(i), 0.0);
      } else if (q(i) > hi) {
        tau(i) += options_.joint_limit_stiffness * (hi - q(i)) - options_.joint_limit_damping * std::max(qd(i), 0.0);
      }
    }
  }
  if (!options_.contacts.empty()) {
    const Vec3 f = contactForce(q, qd);
    if (f.squaredNorm() > 0.0) {
      const KinematicState state = forwardKinematics(model_, q);
      const Mat6X J = pointJacobian(model_, state, contact_body_, contact_point_in_body_);
      tau += J.bottomRows<3>().transpose() * f;
    }
  }
  return tau;
}

VecX Simulator::plantAcceleration(const VecX& q, const VecX& qd, const VecX& tau) const {
  return forwardDynamics(model_, q, qd, tau + externalTorque(q, qd));
}

void Simulator::record(const VecX& tau) {
  if (steps_ % options_.record_every != 0) {
    return;
  }
  log_.time.push_back(time_);
  log_.q.push_back(q_);
  log_.qd.push_back(qd_);
  log_.tau.push_back(tau);
  log_.ee_pose.push_back(endEffectorPose(model_, q_));
  log_.contact_force.push_back(contactForce(q_, qd_));
}

void Simulator::step(Controller& controller) {
  ControllerInput input;
  input.time = time_;
  input.q = q_;
  input.qd = qd_;
  input.external_wrench = endEffectorWrench(q_, qd_);
  VecX tau = controller.computeTorque(input);
  model_.checkConfiguration(tau, "Simulator: controller torque");
  if (options_.saturate_torques) {
    tau = tau.cwiseMax(-model_.effortLimits()).cwiseMin(model_.effortLimits());
  }
  record(tau);
  const AccelerationFunction f = [&](double, const VecX& q, const VecX& qd) { return plantAcceleration(q, qd, tau); };
  JointState next = integrateStep(options_.integrator, f, time_, {q_, qd_}, options_.dt);
  require(next.q.allFinite() && next.qd.allFinite(),
          "Simulator: state became non-finite (time step too large for the gains or contact stiffness)");
  q_ = std::move(next.q);
  qd_ = std::move(next.qd);
  ++steps_;
  time_ = static_cast<double>(steps_) * options_.dt;  // no accumulated round-off in time
}

const SimulationLog& Simulator::run(Controller& controller, double duration) {
  require(std::isfinite(duration) && duration >= 0.0, "Simulator::run: duration must be non-negative");
  const auto steps = static_cast<long long>(std::llround(duration / options_.dt));
  for (long long k = 0; k < steps; ++k) {
    step(controller);
  }
  return log_;
}

}  // namespace mc
