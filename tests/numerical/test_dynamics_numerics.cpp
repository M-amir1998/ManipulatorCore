// Integration accuracy and energy behaviour of the dynamics integrators on conservative systems.

#include "manipulatorcore/dynamics/integrators.hpp"
#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/math/error.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mc {
namespace {

struct EnergyTrace {
  double initial = 0.0;
  double max_deviation = 0.0;
  double final_deviation = 0.0;
};

EnergyTrace simulatePassive(const RobotModel& model, Integrator integrator, const VecX& q0, double dt,
                            double duration) {
  const VecX zero = VecX::Zero(model.dof());
  const AccelerationFunction f = [&](double, const VecX& q, const VecX& qd) {
    return forwardDynamics(model, q, qd, zero);
  };
  JointState s{q0, zero};
  const auto energy = [&](const JointState& x) {
    return kineticEnergy(model, x.q, x.qd) + potentialEnergy(model, x.q);
  };
  EnergyTrace trace;
  trace.initial = energy(s);
  const int steps = static_cast<int>(std::lround(duration / dt));
  for (int k = 0; k < steps; ++k) {
    s = integrateStep(integrator, f, k * dt, s, dt);
    trace.max_deviation = std::max(trace.max_deviation, std::abs(energy(s) - trace.initial));
  }
  trace.final_deviation = energy(s) - trace.initial;
  return trace;
}

TEST(DynamicsNumerics, Rk4ConservesEnergyOfFreeSwingingArm) {
  // Planar 2R swinging under gravity from a horizontal pose (no damping, no torque).
  const RobotModel model = makePlanar2R();
  const EnergyTrace rk4 = simulatePassive(model, Integrator::RK4, Vec2(0.0, 0.0), 1e-3, 3.0);
  // Total energy swing is ~ m g l ~ 40 J; RK4 at 1 ms keeps it to well below 1 mJ.
  EXPECT_LT(rk4.max_deviation, 1e-4);
}

TEST(DynamicsNumerics, SemiImplicitEulerEnergyErrorIsBounded) {
  const RobotModel model = makePlanar2R();
  const EnergyTrace semi = simulatePassive(model, Integrator::SemiImplicitEuler, Vec2(0.0, 0.0), 1e-3, 3.0);
  const EnergyTrace semi_long = simulatePassive(model, Integrator::SemiImplicitEuler, Vec2(0.0, 0.0), 1e-3, 6.0);
  // Symplectic: the error oscillates instead of accumulating.
  EXPECT_LT(semi.max_deviation, 0.5);
  EXPECT_LT(semi_long.max_deviation, 2.0 * semi.max_deviation + 0.05);
}

TEST(DynamicsNumerics, ExplicitEulerGainsEnergy) {
  // Documented failure mode: explicit Euler injects energy into oscillatory systems.
  const RobotModel model = makePlanar2R();
  const EnergyTrace euler = simulatePassive(model, Integrator::ExplicitEuler, Vec2(0.0, 0.0), 1e-3, 3.0);
  const EnergyTrace rk4 = simulatePassive(model, Integrator::RK4, Vec2(0.0, 0.0), 1e-3, 3.0);
  EXPECT_GT(euler.final_deviation, 0.0);
  EXPECT_GT(std::abs(euler.final_deviation), 100.0 * rk4.max_deviation);
}

TEST(DynamicsNumerics, Rk4ConservesEnergyOfSevenDofArm) {
  RobotModel model = makeRedundant7R();
  const EnergyTrace rk4 = simulatePassive(model, Integrator::RK4, redundant7RHome(), 1e-3, 1.0);
  EXPECT_LT(rk4.max_deviation, 1e-4);
}

TEST(DynamicsNumerics, Rk4IsFourthOrder) {
  // Halving dt should reduce the final-state error by ~16x for a smooth trajectory.
  const RobotModel model = makePlanar2R();
  const VecX zero = VecX::Zero(2);
  const AccelerationFunction f = [&](double, const VecX& q, const VecX& qd) {
    return forwardDynamics(model, q, qd, zero);
  };
  const auto run = [&](double dt) {
    JointState s{Vec2(0.3, -0.2), zero};
    const int steps = static_cast<int>(std::lround(0.5 / dt));
    for (int k = 0; k < steps; ++k) {
      s = integrateStep(Integrator::RK4, f, k * dt, s, dt);
    }
    return s.q;
  };
  const VecX reference = run(1.25e-4);
  const double e1 = (run(4e-3) - reference).norm();
  const double e2 = (run(2e-3) - reference).norm();
  EXPECT_GT(e1 / e2, 12.0);
  EXPECT_LT(e1 / e2, 20.0);
}

TEST(DynamicsNumerics, IntegratorRejectsInvalidStep) {
  const AccelerationFunction f = [](double, const VecX& q, const VecX&) { return VecX::Zero(q.size()); };
  EXPECT_THROW((void)integrateStep(Integrator::RK4, f, 0.0, {VecX::Zero(1), VecX::Zero(1)}, 0.0), InvalidArgument);
}

}  // namespace
}  // namespace mc
