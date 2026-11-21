#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/poe.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/kinematics/workspace.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mc {
namespace {

// ------------------------------------------------------------------------------------------------
// Forward kinematics
// ------------------------------------------------------------------------------------------------

TEST(ForwardKinematics, Planar2RMatchesClosedForm) {
  const RobotModel model = makePlanar2R();  // l1 = 1.0, l2 = 0.8
  Random rng(21);
  for (int k = 0; k < 100; ++k) {
    const VecX q = test::randomConfiguration(model, rng);
    const Vec3 p = endEffectorPose(model, q).translation();
    EXPECT_NEAR(p.x(), 1.0 * std::cos(q(0)) + 0.8 * std::cos(q(0) + q(1)), 1e-14);
    EXPECT_NEAR(p.y(), 1.0 * std::sin(q(0)) + 0.8 * std::sin(q(0) + q(1)), 1e-14);
    EXPECT_NEAR(p.z(), 0.0, 0.0);
    EXPECT_MATRIX_NEAR(endEffectorPose(model, q).rotation(), rotZ(q(0) + q(1)), 1e-14);
  }
}

TEST(ForwardKinematics, Planar3RKnownConfigurations) {
  const RobotModel model = makePlanar3R();  // 1.0, 0.8, 0.4
  EXPECT_MATRIX_NEAR(endEffectorPose(model, Vec3(0, 0, 0)).translation(), Vec3(2.2, 0, 0), 1e-15);
  EXPECT_MATRIX_NEAR(endEffectorPose(model, Vec3(kHalfPi, 0, 0)).translation(), Vec3(0, 2.2, 0), 1e-14);
  EXPECT_MATRIX_NEAR(endEffectorPose(model, Vec3(0, kHalfPi, -kHalfPi)).translation(), Vec3(1.4, 0.8, 0), 1e-14);
  EXPECT_MATRIX_NEAR(endEffectorPose(model, Vec3(0, kPi, 0)).translation(), Vec3(-0.2, 0, 0), 1e-14);
}

TEST(ForwardKinematics, HomeConfigurationOfRedundant7RIsVertical) {
  const RobotModel model = makeRedundant7R();
  const SE3 T = endEffectorPose(model, model.zeroConfiguration());
  EXPECT_MATRIX_NEAR(T.rotation(), Mat3::Identity(), 1e-15);
  EXPECT_MATRIX_NEAR(T.translation(), Vec3(0.0, 0.0, 1.36), 1e-14);
}

TEST(ForwardKinematics, StateIsConsistent) {
  const RobotModel model = makeIndustrial6R();
  Random rng(22);
  const VecX q = test::randomConfiguration(model, rng);
  const KinematicState s = forwardKinematics(model, q);
  ASSERT_EQ(s.T_base_body.size(), 6U);
  EXPECT_POSE_NEAR(s.T_base_ee, s.T_base_body.back() * model.toolTransform(), 1e-15);
  EXPECT_POSE_NEAR(s.T_base_ee, endEffectorPose(model, q), 1e-15);
  // FK inverse consistency: T * T^-1 = I.
  EXPECT_POSE_NEAR(s.T_base_ee * s.T_base_ee.inverse(), SE3::Identity(), 1e-14);
}

TEST(ForwardKinematics, PoEMatchesChainForAllRobots) {
  std::vector<RobotModel> robots = test::allBuiltinRobots();
  robots.push_back(test::makeMixedChain());
  Random rng(23);
  for (const RobotModel& model : robots) {
    const ProductOfExponentials poe = productOfExponentials(model);
    for (int k = 0; k < 30; ++k) {
      const VecX q = test::randomConfiguration(model, rng);
      const SE3 T = endEffectorPose(model, q);
      EXPECT_POSE_NEAR(poeForwardSpace(poe, q), T, 1e-12) << model.name();
      EXPECT_POSE_NEAR(poeForwardBody(poe, q), T, 1e-12) << model.name();
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Jacobians against finite differences of forward kinematics
// ------------------------------------------------------------------------------------------------

// dT/dq_i by central differences.
Mat4 dTdq(const RobotModel& model, const VecX& q, int i, double h) {
  VecX qp = q;
  VecX qm = q;
  qp(i) += h;
  qm(i) -= h;
  return (endEffectorPose(model, qp).matrix() - endEffectorPose(model, qm).matrix()) / (2.0 * h);
}

// Central differences have O(h^2) truncation error and O(eps/h) round-off; with h = 1e-6 and
// O(1) second derivatives the combined error is ~1e-9, so 1e-7 is a safe tolerance.
constexpr double kFdStep = 1e-6;
constexpr double kFdTol = 1e-7;

class JacobianFiniteDifference : public ::testing::TestWithParam<int> {
 protected:
  RobotModel model() const {
    std::vector<RobotModel> robots = test::allBuiltinRobots();
    robots.push_back(test::makeMixedChain());
    return robots.at(static_cast<std::size_t>(GetParam()));
  }
};

TEST_P(JacobianFiniteDifference, SpaceJacobian) {
  const RobotModel m = model();
  Random rng(31);
  for (int k = 0; k < 10; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const Mat4 T_inv = endEffectorPose(m, q).inverse().matrix();
    Mat6X numeric(6, m.dof());
    for (int i = 0; i < m.dof(); ++i) {
      numeric.col(i) = veeSE3(dTdq(m, q, i, kFdStep) * T_inv);  // [V_s] = Tdot T^-1
    }
    EXPECT_MATRIX_NEAR(spaceJacobian(m, q), numeric, kFdTol) << m.name();
  }
}

TEST_P(JacobianFiniteDifference, BodyJacobian) {
  const RobotModel m = model();
  Random rng(32);
  for (int k = 0; k < 10; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const Mat4 T_inv = endEffectorPose(m, q).inverse().matrix();
    Mat6X numeric(6, m.dof());
    for (int i = 0; i < m.dof(); ++i) {
      numeric.col(i) = veeSE3(T_inv * dTdq(m, q, i, kFdStep));  // [V_b] = T^-1 Tdot
    }
    EXPECT_MATRIX_NEAR(bodyJacobian(m, q), numeric, kFdTol) << m.name();
  }
}

TEST_P(JacobianFiniteDifference, GeometricJacobian) {
  const RobotModel m = model();
  Random rng(33);
  for (int k = 0; k < 10; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const Mat3 R_T = endEffectorPose(m, q).rotation().transpose();
    Mat6X numeric(6, m.dof());
    for (int i = 0; i < m.dof(); ++i) {
      const Mat4 dT = dTdq(m, q, i, kFdStep);
      numeric.col(i).head<3>() = vee(dT.topLeftCorner<3, 3>() * R_T);  // omega_s = vee(Rdot R^T)
      numeric.col(i).tail<3>() = dT.topRightCorner<3, 1>();            // pdot
    }
    EXPECT_MATRIX_NEAR(geometricJacobian(m, q), numeric, kFdTol) << m.name();
  }
}

TEST_P(JacobianFiniteDifference, PoEJacobiansMatchChain) {
  const RobotModel m = model();
  const ProductOfExponentials poe = productOfExponentials(m);
  Random rng(34);
  for (int k = 0; k < 10; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    EXPECT_MATRIX_NEAR(poeSpaceJacobian(poe, q), spaceJacobian(m, q), 1e-12) << m.name();
    EXPECT_MATRIX_NEAR(poeBodyJacobian(poe, q), bodyJacobian(m, q), 1e-12) << m.name();
  }
}

TEST_P(JacobianFiniteDifference, GeometricFromSpaceJacobian) {
  // J_g = [I 0; -[p] I] J_s
  const RobotModel m = model();
  Random rng(35);
  const VecX q = test::randomConfiguration(m, rng);
  const KinematicState s = forwardKinematics(m, q);
  Mat6 X = Mat6::Identity();
  X.bottomLeftCorner<3, 3>() = -hat(s.T_base_ee.translation());
  EXPECT_MATRIX_NEAR(geometricJacobian(m, s), (X * spaceJacobian(m, s)).eval(), 1e-13);
}

TEST_P(JacobianFiniteDifference, PointJacobian) {
  const RobotModel m = model();
  Random rng(36);
  const VecX q = test::randomConfiguration(m, rng);
  const int body = m.dof() / 2;
  const Vec3 point(0.05, -0.02, 0.1);
  const auto position = [&](const VecX& x) -> VecX {
    return forwardKinematics(m, x).T_base_body[static_cast<std::size_t>(body)] * point;
  };
  const MatX numeric = numericalJacobian(position, q, kFdStep);
  const Mat6X J = pointJacobian(m, forwardKinematics(m, q), body, point);
  EXPECT_MATRIX_NEAR(J.bottomRows<3>().eval(), numeric, kFdTol) << m.name();
}

TEST_P(JacobianFiniteDifference, JacobianDotTimesVelocity) {
  // Jdot qd = d/dt (J(q(t)) qd) along q(t) = q + t qd.
  const RobotModel m = model();
  Random rng(37);
  for (int k = 0; k < 5; ++k) {
    const VecX q = test::randomConfiguration(m, rng);
    const VecX qd = test::randomVector(m.dof(), rng);
    const double h = 1e-6;
    const VecX numeric =
        (geometricJacobian(m, VecX(q + h * qd)) * qd - geometricJacobian(m, VecX(q - h * qd)) * qd) / (2.0 * h);
    EXPECT_MATRIX_NEAR(geometricJacobianDotTimesVelocity(m, q, qd), numeric, 1e-6) << m.name();
  }
}

INSTANTIATE_TEST_SUITE_P(AllRobots, JacobianFiniteDifference, ::testing::Range(0, 5));

TEST(Jacobian, PlanarTaskJacobianRows) {
  const RobotModel model = makePlanar2R();
  const Mat6X J = geometricJacobian(model, Vec2(0.3, 0.9));
  const MatX Jp = planarTaskJacobian(J);
  EXPECT_MATRIX_NEAR(Jp.row(0).eval(), J.row(3).eval(), 0.0);
  EXPECT_MATRIX_NEAR(Jp.row(2).eval(), J.row(2).eval(), 0.0);
}

// ------------------------------------------------------------------------------------------------
// Singularity analysis
// ------------------------------------------------------------------------------------------------

TEST(Singularity, Planar2RManipulabilityClosedForm) {
  // w = l1 l2 |sin q2| for the translational planar Jacobian (Yoshikawa 1985).
  const RobotModel model = makePlanar2R();
  for (const double q2 : {-2.5, -1.0, 0.3, 1.2, 2.9}) {
    const Mat6X J = geometricJacobian(model, Vec2(0.4, q2));
    const MatX Jxy = J.middleRows<2>(3);
    EXPECT_NEAR(yoshikawaManipulability(Jxy), 1.0 * 0.8 * std::abs(std::sin(q2)), 1e-13);
  }
}

TEST(Singularity, Planar2RStretchedIsSingular) {
  const RobotModel model = makePlanar2R();
  const MatX Jxy = geometricJacobian(model, Vec2(0.4, 0.0)).middleRows<2>(3);
  const JacobianAnalysis a = analyzeJacobian(Jxy);
  EXPECT_EQ(a.rank, 1);
  EXPECT_TRUE(a.isSingular());
  EXPECT_NEAR(a.manipulability, 0.0, 1e-15);
  EXPECT_NEAR(a.inverseConditionNumber(), 0.0, 1e-15);
}

TEST(Singularity, Industrial6RWristSingularity) {
  const RobotModel model = makeIndustrial6R();
  VecX q = industrial6RHome();
  const SingularityReport regular = analyzeSingularity(model, q);
  EXPECT_EQ(regular.full.rank, 6);
  q(4) = 0.0;  // wrist singularity: joints 4 and 6 aligned
  const SingularityReport wrist = analyzeSingularity(model, q);
  EXPECT_EQ(wrist.full.rank, 5);
  EXPECT_LT(wrist.full.sigma_min, 1e-12);
}

TEST(Singularity, Redundant7RVerticalIsSingular) {
  const RobotModel model = makeRedundant7R();
  EXPECT_LT(analyzeSingularity(model, model.zeroConfiguration()).full.rank, 6);
  EXPECT_EQ(analyzeSingularity(model, redundant7RHome()).full.rank, 6);
}

// ------------------------------------------------------------------------------------------------
// Workspace
// ------------------------------------------------------------------------------------------------

TEST(Workspace, Planar2RSamplesLieInAnnulus) {
  const RobotModel model = makePlanar2R();
  const auto samples = sampleWorkspace(model, {.samples = 2000, .seed = 3, .manipulability_rows = {3, 4}});
  ASSERT_EQ(samples.size(), 2000U);
  for (const WorkspaceSample& s : samples) {
    const double r = s.position.norm();
    EXPECT_GE(r, 0.2 - 1e-12);
    EXPECT_LE(r, 1.8 + 1e-12);
    EXPECT_GE(s.manipulability, 0.0);
  }
  const WorkspaceBounds b = workspaceBounds(samples);
  EXPECT_GT(b.max_reach, 1.7);
  EXPECT_LE(b.max_reach, 1.8 + 1e-12);
}

TEST(Workspace, DeterministicForSeed) {
  const RobotModel model = makeRedundant7R();
  const auto a = sampleWorkspace(model, {.samples = 50, .seed = 99});
  const auto b = sampleWorkspace(model, {.samples = 50, .seed = 99});
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].position, b[i].position);
  }
}

}  // namespace
}  // namespace mc
