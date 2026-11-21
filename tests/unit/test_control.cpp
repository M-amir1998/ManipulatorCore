#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/control/metrics.hpp"
#include "manipulatorcore/control/task_space_controllers.hpp"
#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

ControllerInput inputAt(const VecX& q, const VecX& qd, double t = 0.0) {
  ControllerInput in;
  in.time = t;
  in.q = q;
  in.qd = qd;
  return in;
}

TEST(Control, ComputedTorqueProducesCommandedErrorDynamics) {
  const RobotModel model = makeRedundant7R();
  Random rng(401);
  const VecX q = test::randomConfiguration(model, rng);
  const VecX qd = test::randomVector(7, rng);
  const JointReference ref{test::randomConfiguration(model, rng), test::randomVector(7, rng),
                           test::randomVector(7, rng, 3.0)};
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(20.0, 7, kp, kd);
  ComputedTorqueController ctc(model, kp, kd, [&](double) { return ref; });
  const VecX tau = ctc.computeTorque(inputAt(q, qd));
  const VecX qdd = forwardDynamics(model, q, qd, tau);
  // edd + Kd edot + Kp e = 0  <=>  qdd = qdd_d + Kd (qd_d - qd) + Kp (q_d - q)
  const VecX expected = ref.qdd + kd.cwiseProduct(ref.qd - qd) + kp.cwiseProduct(ref.q - q);
  EXPECT_MATRIX_NEAR(qdd, expected, 1e-8);
}

TEST(Control, OperationalSpaceTaskAccelerationEqualsCommand) {
  const RobotModel model = makeRedundant7R();
  Random rng(403);
  const VecX q = redundant7RHome() + test::randomVector(7, rng, 0.2);
  const VecX qd = test::randomVector(7, rng, 0.5);
  const SE3 target = endEffectorPose(model, q) * SE3(Eigen::AngleAxisd(0.1, Vec3::UnitX()).toRotationMatrix(),
                                                     Vec3(0.02, -0.01, 0.03));
  const Mat6 kp = diagonalGain(100.0, 200.0);
  const Mat6 kd = diagonalGain(20.0, 28.0);
  NullSpaceTask posture;
  posture.posture = model.midConfiguration();
  posture.stiffness = 5.0;
  posture.damping = 1.0;
  OperationalSpaceController osc(model, kp, kd, constantPoseReference(target), posture);
  const VecX tau = osc.computeTorque(inputAt(q, qd));
  const VecX qdd = forwardDynamics(model, q, qd, tau);
  const Mat6X J = geometricJacobian(model, q);
  const Vec6 task_acc = J * qdd + geometricJacobianDotTimesVelocity(model, q, qd);
  const Vec6 a_star = kd * (-J * qd) + kp * poseErrorWorld(endEffectorPose(model, q), target);
  EXPECT_MATRIX_NEAR(task_acc, a_star, 1e-8);
}

TEST(Control, OperationalSpaceNullSpaceIsDynamicallyConsistent) {
  const RobotModel model = makeRedundant7R();
  Random rng(405);
  const VecX q = redundant7RHome() + test::randomVector(7, rng, 0.2);
  const VecX qd = VecX::Zero(7);
  const OperationalSpaceTerms t = operationalSpaceTerms(model, q, qd);
  EXPECT_TRUE(isSymmetric(t.Lambda, 1e-9));
  EXPECT_TRUE(isPositiveDefinite(t.Lambda));
  // Any torque projected by N^T causes no task acceleration: J M^-1 N^T tau0 = 0.
  const VecX tau0 = test::randomVector(7, rng, 10.0);
  const VecX qdd = t.M.llt().solve(t.null_space * tau0);
  EXPECT_MATRIX_NEAR((t.J * qdd).eval(), VecX::Zero(6), 1e-9);
}

TEST(Control, OperationalSpaceStaysBoundedAtSingularity) {
  const RobotModel model = makeRedundant7R();
  const VecX q = model.zeroConfiguration();  // fully stretched: singular
  OperationalSpaceController osc(model, diagonalGain(50, 100), diagonalGain(10, 20),
                                 constantPoseReference(SE3::fromTranslation(Vec3(0.3, 0.0, 1.0))));
  const VecX tau = osc.computeTorque(inputAt(q, VecX::Zero(7)));
  EXPECT_TRUE(tau.allFinite());
  EXPECT_LT(tau.cwiseAbs().maxCoeff(), 1e5);
}

TEST(Control, ImpedanceAtEquilibriumOnlyCompensatesGravity) {
  const RobotModel model = makeRedundant7R();
  const VecX q = redundant7RHome();
  CartesianImpedanceController imp(model, diagonalGain(50, 500), diagonalGain(5, 50),
                                   constantPoseReference(endEffectorPose(model, q)));
  EXPECT_MATRIX_NEAR(imp.computeTorque(inputAt(q, VecX::Zero(7))), gravityTorques(model, q), 1e-9);
}

TEST(Control, ImpedanceWrenchIsStiffnessTimesError) {
  const RobotModel model = makeRedundant7R();
  const VecX q = redundant7RHome();
  const SE3 T = endEffectorPose(model, q);
  const Vec3 offset(0.0, 0.0, 0.01);
  const Mat6 K = diagonalGain(50, 800);
  CartesianImpedanceController imp(model, K, diagonalGain(5, 50),
                                   constantPoseReference(SE3(T.rotation(), T.translation() + offset)));
  const VecX tau = imp.computeTorque(inputAt(q, VecX::Zero(7))) - gravityTorques(model, q);
  // tau = J^T F  ->  recover F by least squares (J has full row rank here).
  const Mat6X J = geometricJacobian(model, q);
  const VecX F = solveMinimumNorm(J.transpose(), tau);
  EXPECT_MATRIX_NEAR(F.tail<3>().eval(), (800.0 * offset).eval(), 1e-9);
}

TEST(Control, PidIntegratorIsClampedAndTimeMustIncrease) {
  const RobotModel model = makePlanar2R();
  JointPIDGains gains{Vec2(10, 10), Vec2(100, 100), Vec2(1, 1)};
  JointPIDController pid(model, gains, constantJointReference(Vec2(1.0, 0.0)), false, Vec2(0.01, 0.01));
  (void)pid.computeTorque(inputAt(Vec2::Zero(), Vec2::Zero(), 0.0));
  const VecX tau = pid.computeTorque(inputAt(Vec2::Zero(), Vec2::Zero(), 10.0));
  // Integral term saturates at Ki * limit = 1.0 for joint 0.
  EXPECT_NEAR(tau(0), 10.0 * 1.0 + 100.0 * 0.01, 1e-12);
  EXPECT_THROW((void)pid.computeTorque(inputAt(Vec2::Zero(), Vec2::Zero(), 5.0)), InvalidArgument);
  EXPECT_EQ(pid.name(), "PID");
  pid.reset();
  EXPECT_NO_THROW((void)pid.computeTorque(inputAt(Vec2::Zero(), Vec2::Zero(), 5.0)));
}

TEST(Control, ControllerNames) {
  const RobotModel model = makePlanar2R();
  JointPIDGains pd{Vec2(10, 10), VecX(), Vec2(1, 1)};
  EXPECT_EQ(JointPIDController(model, pd, constantJointReference(Vec2::Zero()), false).name(), "PD");
  EXPECT_EQ(JointPIDController(model, pd, constantJointReference(Vec2::Zero()), true).name(), "PD + gravity");
}

TEST(Control, InvalidConfigurationThrows) {
  const RobotModel model = makePlanar2R();
  EXPECT_THROW(JointPIDController(model, {Vec3(1, 1, 1), VecX(), Vec2(1, 1)}, constantJointReference(Vec2::Zero())),
               InvalidArgument);
  Mat6 asymmetric = Mat6::Identity();
  asymmetric(0, 1) = 1.0;
  EXPECT_THROW(TaskSpacePDController(model, asymmetric, Mat6::Identity(), constantPoseReference(SE3())),
               InvalidArgument);
}

TEST(Metrics, KnownSignals) {
  TrackingSignals s;
  for (int k = 0; k <= 10; ++k) {
    s.time.push_back(0.1 * k);
    s.q_desired.push_back(Vec2(1.0, 1.0));
    s.q.push_back(Vec2(1.0 - (k < 5 ? 0.1 : 0.0), 1.0));
    s.tau.push_back(Vec2(2.0, -3.0));
    s.position_error.push_back(k < 5 ? 0.01 : 0.0);
  }
  const TrackingMetrics m = computeTrackingMetrics(s, 1e-3);
  EXPECT_NEAR(m.peak_torque, 3.0, 0.0);
  EXPECT_NEAR(m.control_effort, 13.0 * 1.0, 1e-12);
  EXPECT_NEAR(m.max_joint_error, 0.1, 1e-15);
  EXPECT_NEAR(m.settling_time, 0.5, 1e-12);
  EXPECT_NEAR(m.joint_rmse, std::sqrt(5 * 0.01 / 2.0 / 11.0), 1e-12);
}

}  // namespace
}  // namespace mc
