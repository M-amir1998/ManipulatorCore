#include "manipulatorcore/kinematics/analytical_ik.hpp"
#include "manipulatorcore/kinematics/differential_kinematics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "manipulatorcore/optimization/objectives.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mc {
namespace {

// ------------------------------------------------------------------------------------------------
// Analytical IK
// ------------------------------------------------------------------------------------------------

TEST(AnalyticalIK, Planar2RTwoSolutionsReachTarget) {
  const RobotModel model = makePlanar2R();
  Random rng(41);
  for (int k = 0; k < 200; ++k) {
    // Sample reachable targets from FK of random configurations.
    const VecX q_true = test::randomConfiguration(model, rng);
    const Vec2 target = endEffectorPose(model, q_true).translation().head<2>();
    const auto solutions = analyticalIKPlanar2R(model, target);
    ASSERT_FALSE(solutions.empty());
    for (const VecX& q : solutions) {
      EXPECT_MATRIX_NEAR(endEffectorPose(model, q).translation().head<2>().eval(), target, 1e-12);
    }
    if (solutions.size() == 2) {
      EXPECT_GT(solutions[0](1), 0.0);  // q2 > 0 first
      EXPECT_LT(solutions[1](1), 0.0);
    }
  }
}

TEST(AnalyticalIK, Planar2RUnreachableAndBoundary) {
  EXPECT_TRUE(planar2RInverseKinematics(1.0, 0.8, Vec2(2.0, 0.0)).empty());    // too far
  EXPECT_TRUE(planar2RInverseKinematics(1.0, 0.8, Vec2(0.1, 0.0)).empty());    // inside the hole
  const auto stretched = planar2RInverseKinematics(1.0, 0.8, Vec2(0.0, 1.8));  // boundary
  ASSERT_EQ(stretched.size(), 1U);
  EXPECT_NEAR(stretched[0](0), kHalfPi, 1e-12);
  EXPECT_NEAR(stretched[0](1), 0.0, 1e-12);
  EXPECT_THROW((void)planar2RInverseKinematics(-1.0, 0.8, Vec2(1.0, 0.0)), InvalidArgument);
}

TEST(AnalyticalIK, Planar3RReachesPositionAndOrientation) {
  const RobotModel model = makePlanar3R();
  Random rng(43);
  for (int k = 0; k < 200; ++k) {
    const VecX q_true = test::randomConfiguration(model, rng);
    const SE3 T = endEffectorPose(model, q_true);
    const double phi = q_true.sum();
    const auto solutions = analyticalIKPlanar3R(model, T.translation().head<2>(), phi, false);
    ASSERT_FALSE(solutions.empty());
    for (const VecX& q : solutions) {
      EXPECT_POSE_NEAR(endEffectorPose(model, q), T, 1e-11);
    }
  }
}

TEST(AnalyticalIK, RejectsNonPlanarModels) {
  EXPECT_THROW((void)planarLinkLengths(makeIndustrial6R()), InvalidArgument);
}

// ------------------------------------------------------------------------------------------------
// Numerical IK
// ------------------------------------------------------------------------------------------------

struct IKCase {
  IKMethod method;
  int max_iterations;
};

class NumericalIK : public ::testing::TestWithParam<IKCase> {};

TEST_P(NumericalIK, ConvergesOnReachableTargetsFromNearbySeeds) {
  for (const RobotModel& model : {makeIndustrial6R(), makeRedundant7R()}) {
    Random rng(47);
    int converged = 0;
    const int trials = 40;
    for (int k = 0; k < trials; ++k) {
      const VecX q_true = test::randomConfiguration(model, rng);
      const SE3 target = endEffectorPose(model, q_true);
      const VecX seed = model.clampToLimits(q_true + test::randomVector(model.dof(), rng, 0.25));
      IKOptions options;
      options.method = GetParam().method;
      options.max_iterations = GetParam().max_iterations;
      const IKResult r = solveIK(model, target, seed, options);
      EXPECT_TRUE(r.q.allFinite());
      EXPECT_TRUE(r.within_limits);
      if (r.converged()) {
        ++converged;
        const PoseDistance d = poseDistance(endEffectorPose(model, r.q), target);
        EXPECT_LE(d.translation, options.position_tolerance * (1.0 + 1e-9));
        EXPECT_LE(d.rotation, options.orientation_tolerance * (1.0 + 1e-9));
      }
    }
    // Every method must solve the large majority of nearby-seed problems.
    EXPECT_GE(converged, trials * 8 / 10) << model.name() << " / " << toString(GetParam().method);
  }
}

INSTANTIATE_TEST_SUITE_P(Methods, NumericalIK,
                         ::testing::Values(IKCase{IKMethod::JacobianTranspose, 3000},
                                           IKCase{IKMethod::PseudoInverse, 200},
                                           IKCase{IKMethod::DampedLeastSquares, 200},
                                           IKCase{IKMethod::LevenbergMarquardt, 200}),
                         [](const ::testing::TestParamInfo<IKCase>& info) {
                           std::string name = toString(info.param.method);
                           std::erase(name, '-');
                           return name;
                         });

TEST(NumericalIK, UnreachableTargetReportsFailureWithoutNaN) {
  const RobotModel model = makeRedundant7R();
  const SE3 target = SE3::fromTranslation(Vec3(3.0, 0.0, 0.5));  // far beyond reach
  for (const IKMethod method : {IKMethod::PseudoInverse, IKMethod::DampedLeastSquares, IKMethod::LevenbergMarquardt}) {
    IKOptions options;
    options.method = method;
    options.max_iterations = 100;
    const IKResult r = solveIK(model, target, redundant7RHome(), options);
    EXPECT_FALSE(r.converged()) << toString(method);
    EXPECT_TRUE(r.q.allFinite()) << toString(method);
    EXPECT_TRUE(r.within_limits) << toString(method);
    EXPECT_GT(r.position_error, 1.0);
  }
}

TEST(NumericalIK, DampedLeastSquaresHandlesSingularStart) {
  // q = 0 is a singular configuration of the 7R arm (fully stretched).
  const RobotModel model = makeRedundant7R();
  const SE3 target = endEffectorPose(model, redundant7RHome());
  IKOptions options;
  options.method = IKMethod::DampedLeastSquares;
  options.adaptive_damping = true;
  options.damping = 0.1;
  options.max_iterations = 500;
  const IKResult r = solveIK(model, target, model.zeroConfiguration(), options);
  EXPECT_TRUE(r.converged()) << toString(r.status);
  EXPECT_TRUE(r.q.allFinite());
}

TEST(NumericalIK, RespectsJointLimitsNearTheBoundary) {
  const RobotModel model = makeIndustrial6R();
  VecX q_true = industrial6RHome();
  q_true(1) = model.upperLimits()(1) - 1e-3;  // target generated close to a limit
  const SE3 target = endEffectorPose(model, q_true);
  VecX seed = industrial6RHome();
  IKOptions options;
  options.max_iterations = 500;
  const IKResult r = solveIK(model, target, seed, options);
  EXPECT_TRUE(r.within_limits);
  EXPECT_TRUE(model.withinLimits(r.q));
  EXPECT_TRUE(r.converged()) << toString(r.status) << " pos " << r.position_error;
}

TEST(NumericalIK, PositionOnlyTaskForPlanarArm) {
  const RobotModel model = makePlanar2R();
  IKOptions options;
  options.task_weights << 0, 0, 0, 1, 1, 0;  // x, y only
  const SE3 target = SE3::fromTranslation(Vec3(0.9, 0.9, 0.0));
  const IKResult r = solveIK(model, target, Vec2(0.1, 0.5), options);
  ASSERT_TRUE(r.converged());
  EXPECT_MATRIX_NEAR(endEffectorPose(model, r.q).translation().head<2>().eval(), Vec2(0.9, 0.9), 1e-4);
}

TEST(NumericalIK, NullSpaceRefinementKeepsTaskAndImprovesPosture) {
  const RobotModel model = makeRedundant7R();
  // The seed is already an exact solution with a poorly centred upper-arm roll, so all motion
  // during refinement comes from the secondary objective and must stay in the task null space.
  VecX seed = redundant7RHome();
  seed(2) = 1.2;
  const SE3 target = endEffectorPose(model, seed);
  IKOptions options;
  options.method = IKMethod::PseudoInverse;
  options.null_space_objective = NullSpaceObjective::JointCentering;
  options.null_space_gain = 2.0;
  options.null_space_refinement_iterations = 300;
  options.max_iterations = 400;
  const IKResult r = solveIK(model, target, seed, options);
  ASSERT_TRUE(r.converged());
  const double before = jointCenteringObjective(model, seed).value;
  const double after = jointCenteringObjective(model, r.q).value;
  EXPECT_LT(after, before);
  // A 6-D task on a 7-DOF arm leaves a one-dimensional self-motion (elbow swivel). At the refined
  // solution the cost gradient projected onto that null space must (nearly) vanish.
  const auto projectedGradient = [&](const VecX& q) {
    return (nullSpaceProjector(geometricJacobian(model, q)) * jointCenteringObjective(model, q).gradient).norm();
  };
  EXPECT_LT(projectedGradient(r.q), 0.05 * projectedGradient(seed));
  const PoseDistance d = poseDistance(endEffectorPose(model, r.q), target);
  EXPECT_LE(d.translation, options.position_tolerance);
  EXPECT_LE(d.rotation, options.orientation_tolerance);

  // Without refinement the solver returns the (already converged) seed unchanged.
  options.null_space_refinement_iterations = 0;
  EXPECT_MATRIX_NEAR(solveIK(model, target, seed, options).q, seed, 0.0);
}

TEST(NumericalIK, NullSpaceStepIsFirstOrderTaskPreserving) {
  const RobotModel model = makeRedundant7R();
  const VecX q = redundant7RHome();
  const Mat6X J = geometricJacobian(model, q);
  const VecX z = -jointCenteringObjective(model, q).gradient;
  const VecX dq = nullSpaceProjector(J) * z;
  EXPECT_MATRIX_NEAR((J * dq).eval(), VecX::Zero(6), 1e-12);
  EXPECT_LE(z.dot(dq), z.squaredNorm() + 1e-15);
  EXPECT_GE(z.dot(dq), 0.0);  // projected descent direction never increases the cost to first order
}

TEST(NumericalIK, InvalidOptionsThrow) {
  const RobotModel model = makePlanar2R();
  IKOptions options;
  options.position_tolerance = 0.0;
  EXPECT_THROW((void)solveIK(model, SE3::Identity(), Vec2(0.1, 0.1), options), InvalidArgument);
  options = {};
  options.task_weights.setZero();
  EXPECT_THROW((void)solveIK(model, SE3::Identity(), Vec2(0.1, 0.1), options), InvalidArgument);
  EXPECT_THROW((void)solveIK(model, SE3::Identity(), VecX::Zero(3), {}), InvalidArgument);
}

TEST(MultiStartIK, FindsDistinctValidSolutions) {
  const RobotModel model = makeIndustrial6R();
  const SE3 target = endEffectorPose(model, industrial6RHome());
  MultiStartIKOptions options;
  options.attempts = 40;
  options.seed = 5;
  const auto solutions = solveIKMultiStart(model, target, industrial6RHome(), options);
  ASSERT_GE(solutions.size(), 2U);  // a 6R elbow arm has several IK branches
  for (std::size_t i = 0; i < solutions.size(); ++i) {
    EXPECT_TRUE(solutions[i].converged());
    EXPECT_TRUE(model.withinLimits(solutions[i].q));
    for (std::size_t j = 0; j < i; ++j) {
      EXPECT_GT((solutions[i].q - solutions[j].q).cwiseAbs().maxCoeff(), options.distinct_threshold);
    }
  }
  // Sorted by distance to the seed; the seed itself is a solution.
  EXPECT_LT((solutions.front().q - industrial6RHome()).norm(), 1e-3);
}

TEST(MultiStartIK, ValidatorFiltersSolutions) {
  const RobotModel model = makeIndustrial6R();
  const SE3 target = endEffectorPose(model, industrial6RHome());
  MultiStartIKOptions options;
  options.validator = [](const VecX& q) { return q(0) > 10.0; };  // rejects everything
  EXPECT_TRUE(solveIKMultiStart(model, target, industrial6RHome(), options).empty());
}

// ------------------------------------------------------------------------------------------------
// Differential kinematics and objectives
// ------------------------------------------------------------------------------------------------

TEST(DifferentialKinematics, PseudoInverseReproducesTwist) {
  const RobotModel model = makeRedundant7R();
  const Mat6X J = geometricJacobian(model, redundant7RHome());
  Vec6 twist;
  twist << 0.1, -0.2, 0.05, 0.3, 0.1, -0.1;
  const VecX qd = jointVelocityForTask(J, twist, DifferentialSolver::PseudoInverse, -1.0);
  EXPECT_MATRIX_NEAR(taskVelocity(J, qd), VecX(twist), 1e-12);
}

TEST(DifferentialKinematics, NullSpaceVelocityDoesNotChangeTwist) {
  const RobotModel model = makeRedundant7R();
  const Mat6X J = geometricJacobian(model, redundant7RHome());
  Vec6 twist = Vec6::Zero();
  Random rng(53);
  const VecX qd = jointVelocityWithNullSpace(J, twist, test::randomVector(7, rng));
  EXPECT_MATRIX_NEAR((J * qd).eval(), VecX::Zero(6), 1e-12);
}

TEST(DifferentialKinematics, AdaptiveDampingIsContinuous) {
  EXPECT_EQ(adaptiveDamping(0.2, 0.1, 0.05), 0.0);
  EXPECT_NEAR(adaptiveDamping(0.0, 0.1, 0.05), 0.05, 1e-15);
  EXPECT_NEAR(adaptiveDamping(0.1 - 1e-12, 0.1, 0.05), 0.0, 1e-6);
}

class ObjectiveGradients : public ::testing::TestWithParam<NullSpaceObjective> {};

TEST_P(ObjectiveGradients, MatchFiniteDifferences) {
  const RobotModel model = makeRedundant7R();
  Random rng(59);
  for (int k = 0; k < 5; ++k) {
    // Stay inside the limits so the barrier is smooth.
    const VecX q = 0.8 * test::randomConfiguration(model, rng);
    const ObjectiveValue v = evaluateObjective(GetParam(), model, q);
    const auto f = [&](const VecX& x) -> VecX {
      return VecX::Constant(1, evaluateObjective(GetParam(), model, x).value);
    };
    const MatX numeric = numericalJacobian(f, q, 1e-5);
    EXPECT_MATRIX_NEAR(v.gradient.transpose().eval(), numeric, 1e-5) << toString(GetParam());
  }
}

INSTANTIATE_TEST_SUITE_P(Objectives, ObjectiveGradients,
                         ::testing::Values(NullSpaceObjective::JointCentering, NullSpaceObjective::JointLimitBarrier,
                                           NullSpaceObjective::JointLimitActivation,
                                           NullSpaceObjective::Manipulability));

TEST(Objectives, ActivationPenaltyIsZeroInInterior) {
  const RobotModel model = makeRedundant7R();
  const ObjectiveValue v = jointLimitActivationObjective(model, model.midConfiguration());
  EXPECT_EQ(v.value, 0.0);
  EXPECT_EQ(v.gradient.norm(), 0.0);
  VecX q = model.upperLimits();
  const ObjectiveValue at_limit = jointLimitActivationObjective(model, q);
  EXPECT_GT(at_limit.value, 0.0);
  EXPECT_TRUE((at_limit.gradient.array() > 0.0).all());  // descent pushes away from the upper limits
}

TEST(Objectives, BarrierIsFiniteAtTheLimits) {
  const RobotModel model = makeRedundant7R();
  const ObjectiveValue v = jointLimitBarrierObjective(model, model.upperLimits());
  EXPECT_TRUE(std::isfinite(v.value));
  EXPECT_TRUE(v.gradient.allFinite());
}

}  // namespace
}  // namespace mc
