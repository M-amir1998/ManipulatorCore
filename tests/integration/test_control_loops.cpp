// Closed-loop behaviour of the controllers in the simulator.

#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/control/metrics.hpp"
#include "manipulatorcore/control/task_space_controllers.hpp"
#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/simulation/simulator.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace mc {
namespace {

double jointRmse(const SimulationLog& log, const JointReferenceFn& ref) {
  double sum = 0.0;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    sum += (ref(log.time[k]).q - log.q[k]).squaredNorm() / static_cast<double>(log.q[k].size());
  }
  return std::sqrt(sum / static_cast<double>(log.time.size()));
}

TEST(ControlLoops, GravityCompensationHoldsPose) {
  const RobotModel model = makeRedundant7R();
  Simulator sim(model);
  sim.reset(redundant7RHome());
  GravityCompensationController g(model);
  sim.run(g, 1.0);
  EXPECT_MATRIX_NEAR(sim.q(), redundant7RHome(), 1e-9);
  EXPECT_LT(sim.qd().norm(), 1e-9);
}

TEST(ControlLoops, PDLeavesGravityErrorAndGravityCompensationRemovesIt) {
  const RobotModel model = makeIndustrial6R();
  const VecX target = industrial6RHome();
  JointPIDGains gains{VecX::Constant(6, 400.0), VecX(), VecX::Constant(6, 60.0)};
  SimulatorOptions opt;
  opt.saturate_torques = false;

  Simulator plain(model, opt);
  plain.reset(target);
  JointPIDController pd(model, gains, constantJointReference(target), false);
  plain.run(pd, 4.0);
  const double pd_error = (plain.q() - target).cwiseAbs().maxCoeff();
  // Classical PD: steady-state error ~ Kp^-1 g(q), clearly non-zero for the shoulder.
  EXPECT_GT(pd_error, 0.05);

  Simulator compensated(model, opt);
  compensated.reset(target);
  JointPIDController pdg(model, gains, constantJointReference(target), true);
  compensated.run(pdg, 4.0);
  EXPECT_LT((compensated.q() - target).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(ControlLoops, IntegralActionRemovesSteadyStateError) {
  const RobotModel model = makePlanar2R();
  const VecX target = Vec2(0.4, 0.6);
  SimulatorOptions opt;
  opt.saturate_torques = false;
  Simulator sim(model, opt);
  sim.reset(target);
  JointPIDController pid(model, {Vec2(300, 300), Vec2(400, 400), Vec2(40, 40)}, constantJointReference(target), false);
  sim.run(pid, 8.0);
  EXPECT_LT((sim.q() - target).cwiseAbs().maxCoeff(), 1e-3);
}

TEST(ControlLoops, ComputedTorqueOutperformsPDOnTheSameTrajectory) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  VecX q1 = q0;
  q1 << 0.8, 0.2, -0.5, 1.0, 0.6, 0.4, -0.8;
  const auto traj = std::make_shared<PolynomialTrajectory>(PolynomialTrajectory::quintic(q0, q1, 2.0));
  const JointReferenceFn ref = trajectoryReference(traj);
  SimulatorOptions opt;
  opt.saturate_torques = false;

  const auto run = [&](Controller& c) {
    Simulator sim(model, opt);
    sim.reset(q0);
    sim.run(c, 2.5);
    return jointRmse(sim.log(), ref);
  };
  JointPIDGains gains{VecX::Constant(7, 400.0), VecX(), VecX::Constant(7, 40.0)};
  JointPIDController pd(model, gains, ref, false);
  JointPIDController pdg(model, gains, ref, true);
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(25.0, 7, kp, kd);
  ComputedTorqueController ctc(model, kp, kd, ref);
  const double e_pd = run(pd);
  const double e_pdg = run(pdg);
  const double e_ctc = run(ctc);
  EXPECT_LT(e_ctc, e_pdg);
  EXPECT_LT(e_pdg, e_pd);
  // Exact model: residual error comes only from the zero-order hold over 1 ms steps (joint damping
  // is part of the plant but not of the model).
  EXPECT_LT(e_ctc, 2e-3);
}

TEST(ControlLoops, TaskSpaceControllersReachTargetPose) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 start = endEffectorPose(model, q0);
  const SE3 target(start.rotation() * Eigen::AngleAxisd(0.3, Vec3::UnitY()).toRotationMatrix(),
                   start.translation() + Vec3(-0.1, 0.15, 0.1));
  NullSpaceTask posture;
  posture.posture = q0;
  posture.stiffness = 10.0;
  posture.damping = 2.0;

  TaskSpacePDController pd(model, diagonalGain(150, 1500), diagonalGain(15, 120), constantPoseReference(target),
                           posture);
  OperationalSpaceController osc(model, diagonalGain(100, 100), diagonalGain(20, 20), constantPoseReference(target),
                                 posture);
  for (Controller* c : std::initializer_list<Controller*>{&pd, &osc}) {
    Simulator sim(model);
    sim.reset(q0);
    sim.run(*c, 3.0);
    const PoseDistance d = poseDistance(endEffectorPose(model, sim.q()), target);
    EXPECT_LT(d.translation, 2e-3) << c->name();
    EXPECT_LT(d.rotation, 1e-2) << c->name();
  }
}

TEST(ControlLoops, OperationalSpaceTracksCartesianLine) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 start = endEffectorPose(model, q0);
  const auto path = std::make_shared<LinePath>(start, SE3(start.rotation(), start.translation() + Vec3(0.0, 0.3, 0.1)));
  const auto traj = std::make_shared<CartesianTrajectory>(CartesianTrajectory::quintic(path, 2.0));
  OperationalSpaceController osc(model, diagonalGain(400, 400), diagonalGain(40, 40),
                                 cartesianTrajectoryReference(traj), {.posture = q0, .stiffness = 5, .damping = 2});
  Simulator sim(model);
  sim.reset(q0);
  sim.run(osc, 2.0);
  double worst = 0.0;
  for (std::size_t k = 0; k < sim.log().time.size(); ++k) {
    worst = std::max(worst,
                     (traj->sample(sim.log().time[k]).pose.translation() - sim.log().ee_pose[k].translation()).norm());
  }
  EXPECT_LT(worst, 2e-3);
}

/// Simulator with a vertical wall at x = wall_x in front of the end-effector.
SimulatorOptions wallOptions(double wall_x, double stiffness) {
  SimulatorOptions opt;
  ContactSurface wall;
  wall.normal = -Vec3::UnitX();  // solid for x >= wall_x
  wall.offset = -wall_x;
  wall.stiffness = stiffness;
  wall.damping = 200.0;
  opt.contacts = {wall};
  return opt;
}

TEST(ControlLoops, ImpedanceContactForceMatchesSeriesSprings) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 T0 = endEffectorPose(model, q0);
  const double wall_x = T0.translation().x() + 0.01;
  const double k_wall = 10000.0;
  const double K = 500.0;
  const double push = 0.05;  // desired position 5 cm behind the wall
  Mat6 stiffness = diagonalGain(50.0, K);
  Mat6 damping = diagonalGain(5.0, 2.0 * std::sqrt(K * 5.0));
  const SE3 desired(T0.rotation(), Vec3(wall_x + push, T0.translation().y(), T0.translation().z()));
  CartesianImpedanceController imp(model, stiffness, damping, constantPoseReference(desired),
                                   {.posture = q0, .stiffness = 5.0, .damping = 2.0});
  Simulator sim(model, wallOptions(wall_x, k_wall));
  sim.reset(q0);
  sim.run(imp, 3.0);
  const double force = sim.log().contact_force.back().norm();
  // Spring of the controller and of the wall in series.
  const double expected = K * k_wall / (K + k_wall) * push;
  EXPECT_NEAR(force, expected, 0.05 * expected);
  EXPECT_LT(sim.qd().norm(), 1e-2);  // settled, stable contact
}

TEST(ControlLoops, AdmittanceYieldsToContact) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 T0 = endEffectorPose(model, q0);
  const double wall_x = T0.translation().x() + 0.01;
  const double k_wall = 20000.0;
  const double push = 0.05;
  AdmittanceParameters adm;
  adm.mass = Vec3::Constant(2.0);
  adm.stiffness = Vec3::Constant(400.0);
  adm.damping = Vec3::Constant(2.0 * std::sqrt(400.0 * 2.0) * 1.5);
  const SE3 desired(T0.rotation(), Vec3(wall_x + push, T0.translation().y(), T0.translation().z()));
  AdmittanceController ac(model, adm, diagonalGain(400, 900), diagonalGain(40, 60), constantPoseReference(desired));
  SimulatorOptions opt = wallOptions(wall_x, k_wall);
  opt.dt = 5e-4;
  Simulator sim(model, opt);
  sim.reset(q0);
  sim.run(ac, 3.0);
  const double force = sim.log().contact_force.back().norm();
  const double expected = 400.0 * k_wall / (400.0 + k_wall) * push;
  EXPECT_NEAR(force, expected, 0.1 * expected);
  EXPECT_LT(ac.deviation().x(), -0.04);  // the commanded position yielded by ~ F / K_d
}

TEST(ControlLoops, SimulationIsDeterministic) {
  const RobotModel model = makeRedundant7R();
  const auto run = [&] {
    Simulator sim(model, wallOptions(0.5, 5000.0));
    sim.reset(redundant7RHome());
    CartesianImpedanceController imp(model, diagonalGain(30, 300), diagonalGain(3, 30),
                                     constantPoseReference(SE3::fromTranslation(Vec3(0.7, 0.0, 0.6))));
    return sim.run(imp, 0.5);
  };
  const SimulationLog a = run();
  const SimulationLog b = run();
  ASSERT_EQ(a.q.size(), b.q.size());
  for (std::size_t k = 0; k < a.q.size(); ++k) {
    ASSERT_EQ(a.q[k], b.q[k]);
    ASSERT_EQ(a.tau[k], b.tau[k]);
  }
}

TEST(ControlLoops, TorqueSaturationAndJointLimitStops) {
  const RobotModel model = makePlanar2R();
  // A huge PD gain towards a target beyond the joint limit: torques saturate and the joint-limit
  // stop keeps the joint near its limit.
  JointPIDController pd(model, {Vec2(1e5, 1e5), VecX(), Vec2(10, 10)}, constantJointReference(Vec2(4.0, 0.0)), true);
  Simulator sim(model);
  sim.reset(Vec2(2.9, 0.0));
  sim.run(pd, 2.0);
  for (const VecX& tau : sim.log().tau) {
    EXPECT_LE(tau.cwiseAbs().maxCoeff(), model.effortLimits().maxCoeff() + 1e-12);
  }
  EXPECT_LT(sim.q()(0), model.upperLimits()(0) + 0.05);
}

}  // namespace
}  // namespace mc
