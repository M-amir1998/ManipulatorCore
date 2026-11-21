#include "manipulatorcore/dynamics/planar_2r.hpp"
#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

std::vector<RobotModel> dynamicsRobots() {
  std::vector<RobotModel> robots = test::allBuiltinRobots();
  robots.push_back(test::makeMixedChain());
  return robots;
}

// ------------------------------------------------------------------------------------------------
// Closed-form Lagrangian reference (planar 2R)
// ------------------------------------------------------------------------------------------------

TEST(Dynamics, Planar2RMatchesLagrangianReference) {
  const Planar2RParameters params;
  const RobotModel model = makePlanar2R(params);
  Random rng(71);
  for (int k = 0; k < 100; ++k) {
    const Vec2 q = test::randomConfiguration(model, rng);
    const Vec2 qd = test::randomVector(2, rng, 2.0);
    const Vec2 qdd = test::randomVector(2, rng, 5.0);
    const Planar2RDynamicsTerms ref = planar2RLagrangianDynamics(params, q, qd);
    EXPECT_MATRIX_NEAR(massMatrix(model, q), ref.M, 1e-12);
    EXPECT_MATRIX_NEAR(gravityTorques(model, q), ref.g, 1e-12);
    EXPECT_MATRIX_NEAR(coriolisTorques(model, q, qd), (ref.C * qd).eval(), 1e-12);
    EXPECT_MATRIX_NEAR(inverseDynamics(model, q, qd, qdd), (ref.M * qdd + ref.C * qd + ref.g).eval(), 1e-11);
    // Christoffel-symbol C from CRBA finite differences matches the analytical C.
    EXPECT_MATRIX_NEAR(coriolisMatrix(model, q, qd), ref.C, 1e-7);
  }
}

// ------------------------------------------------------------------------------------------------
// Properties that must hold for every robot
// ------------------------------------------------------------------------------------------------

class DynamicsProperties : public ::testing::TestWithParam<int> {
 protected:
  RobotModel model() const { return dynamicsRobots().at(static_cast<std::size_t>(GetParam())); }
};

TEST_P(DynamicsProperties, MassMatrixIsSymmetricPositiveDefinite) {
  const RobotModel m = model();
  Random rng(73);
  for (int k = 0; k < 30; ++k) {
    const MatX M = massMatrix(m, test::randomConfiguration(m, rng));
    EXPECT_TRUE(isSymmetric(M, 1e-14)) << m.name();
    EXPECT_TRUE(isPositiveDefinite(M)) << m.name();
  }
}

TEST_P(DynamicsProperties, CrbaMatchesRneaColumns) {
  // Column j of M is tau(q, 0, e_j) - g(q).
  const RobotModel m = model();
  Random rng(79);
  const VecX q = test::randomConfiguration(m, rng);
  const VecX g = gravityTorques(m, q);
  const VecX zero = VecX::Zero(m.dof());
  MatX M_rnea(m.dof(), m.dof());
  for (int j = 0; j < m.dof(); ++j) {
    M_rnea.col(j) = inverseDynamics(m, q, zero, VecX::Unit(m.dof(), j)) - g;
  }
  EXPECT_MATRIX_NEAR(massMatrix(m, q), M_rnea, 1e-12) << m.name();
}

TEST_P(DynamicsProperties, EquationOfMotionDecomposition) {
  // tau = M qdd + C qd + g, with each term computed independently.
  const RobotModel m = model();
  Random rng(83);
  for (int k = 0; k < 20; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const VecX qd = test::randomVector(m.dof(), rng, 2.0);
    const VecX qdd = test::randomVector(m.dof(), rng, 5.0);
    const VecX expected = massMatrix(m, q) * qdd + coriolisTorques(m, q, qd) + gravityTorques(m, q);
    EXPECT_MATRIX_NEAR(inverseDynamics(m, q, qd, qdd), expected, 1e-10) << m.name();
  }
}

TEST_P(DynamicsProperties, ForwardDynamicsInvertsInverseDynamics) {
  const RobotModel m = model();
  Random rng(89);
  for (int k = 0; k < 20; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const VecX qd = test::randomVector(m.dof(), rng, 2.0);
    const VecX tau = test::randomVector(m.dof(), rng, 20.0);
    const VecX qdd_aba = forwardDynamics(m, q, qd, tau, ForwardDynamicsMethod::ArticulatedBody);
    const VecX qdd_crba = forwardDynamics(m, q, qd, tau, ForwardDynamicsMethod::CompositeRigidBody);
    EXPECT_MATRIX_NEAR(qdd_aba, qdd_crba, 1e-9) << m.name();
    EXPECT_MATRIX_NEAR(inverseDynamics(m, q, qd, qdd_aba), tau, 1e-9) << m.name();
  }
}

TEST_P(DynamicsProperties, GravityIsGradientOfPotentialEnergy) {
  const RobotModel m = model();
  Random rng(97);
  const VecX q = test::randomConfiguration(m, rng);
  const auto potential = [&](const VecX& x) -> VecX { return VecX::Constant(1, potentialEnergy(m, x)); };
  const MatX dV = numericalJacobian(potential, q, 1e-6);
  EXPECT_MATRIX_NEAR(gravityTorques(m, q).transpose().eval(), dV, 1e-6) << m.name();
}

TEST_P(DynamicsProperties, ChristoffelCoriolisIsConsistent) {
  const RobotModel m = model();
  Random rng(101);
  const VecX q = test::randomConfiguration(m, rng);
  const VecX qd = test::randomVector(m.dof(), rng, 1.5);
  const MatX C = coriolisMatrix(m, q, qd);
  EXPECT_MATRIX_NEAR((C * qd).eval(), coriolisTorques(m, q, qd), 1e-6) << m.name();
  // Passivity property: N = Mdot - 2C is skew-symmetric.
  const MatX N = massMatrixDerivative(m, q, qd) - 2.0 * C;
  EXPECT_MATRIX_NEAR(N, (-N.transpose()).eval(), 1e-6) << m.name();
}

TEST_P(DynamicsProperties, PowerBalance) {
  // d/dt (T + V) = qd^T tau along the motion produced by tau.
  const RobotModel m = model();
  Random rng(103);
  const VecX q = test::randomConfiguration(m, rng);
  const VecX qd = test::randomVector(m.dof(), rng, 1.0);
  const VecX tau = test::randomVector(m.dof(), rng, 10.0);
  const VecX qdd = forwardDynamics(m, q, qd, tau);
  const double h = 1e-6;
  const auto energy = [&](const VecX& x, const VecX& v) { return kineticEnergy(m, x, v) + potentialEnergy(m, x); };
  const double dE = (energy(q + h * qd, qd + h * qdd) - energy(q - h * qd, qd - h * qdd)) / (2.0 * h);
  EXPECT_NEAR(dE, qd.dot(tau), 1e-5 * (1.0 + std::abs(qd.dot(tau)))) << m.name();
}

TEST_P(DynamicsProperties, ZeroVelocityAndAcceleration) {
  const RobotModel m = model();
  Random rng(107);
  const VecX q = test::randomConfiguration(m, rng);
  const VecX zero = VecX::Zero(m.dof());
  EXPECT_MATRIX_NEAR(coriolisTorques(m, q, zero), zero, 1e-15);
  EXPECT_MATRIX_NEAR(inverseDynamics(m, q, zero, zero), gravityTorques(m, q), 0.0);
  // Holding torque = gravity torque gives zero acceleration (gravity compensation).
  EXPECT_MATRIX_NEAR(forwardDynamics(m, q, zero, gravityTorques(m, q)), zero, 1e-10);
}

INSTANTIATE_TEST_SUITE_P(AllRobots, DynamicsProperties, ::testing::Range(0, 5));

TEST(Dynamics, ZeroGravityRemovesGravityTerms) {
  RobotModel model = makeRedundant7R();
  model.setGravity(Vec3::Zero());
  EXPECT_MATRIX_NEAR(gravityTorques(model, redundant7RHome()), VecX::Zero(7), 0.0);
  EXPECT_NEAR(potentialEnergy(model, redundant7RHome()), 0.0, 0.0);
}

TEST(Dynamics, ArmatureAddsToMassMatrixDiagonal) {
  RobotModel model = makePlanar2R();
  const Vec2 q(0.3, 0.7);
  const MatX M0 = massMatrix(model, q);
  std::vector<Body> bodies = model.bodies();
  bodies[1].joint.armature = 0.25;
  const RobotModel with_armature("2R+armature", bodies, model.toolTransform(), model.gravity());
  const MatX M1 = massMatrix(with_armature, q);
  EXPECT_NEAR(M1(1, 1) - M0(1, 1), 0.25, 1e-15);
  EXPECT_NEAR(M1(0, 0), M0(0, 0), 1e-15);
}

TEST(Dynamics, MasslessDistalBodyIsRejectedByForwardDynamics) {
  RobotModel model = makePlanar2R();
  std::vector<Body> bodies = model.bodies();
  bodies[1].inertia = Inertia{};
  const RobotModel bad("massless", bodies, model.toolTransform(), model.gravity());
  EXPECT_THROW((void)forwardDynamics(bad, Vec2(0.1, 0.2), Vec2::Zero(), Vec2::Zero()), InvalidArgument);
  EXPECT_THROW((void)forwardDynamics(bad, Vec2(0.1, 0.2), Vec2::Zero(), Vec2::Zero(),
                                     ForwardDynamicsMethod::CompositeRigidBody),
               InvalidArgument);
}

TEST(Dynamics, InputValidation) {
  const RobotModel model = makePlanar2R();
  EXPECT_THROW((void)inverseDynamics(model, Vec2::Zero(), Vec2::Zero(), VecX::Zero(3)), InvalidArgument);
  EXPECT_THROW((void)massMatrix(model, Vec2(std::nan(""), 0.0)), InvalidArgument);
}

TEST(Dynamics, CenterOfMassOfStretchedPlanarArm) {
  const RobotModel model = makePlanar2R();  // m1 = 2 at x = 0.5, m2 = 1.5 at x = 1.4
  const Vec3 com = centerOfMass(model, Vec2::Zero());
  EXPECT_NEAR(com.x(), (2.0 * 0.5 + 1.5 * 1.4) / 3.5, 1e-15);
}

}  // namespace
}  // namespace mc
