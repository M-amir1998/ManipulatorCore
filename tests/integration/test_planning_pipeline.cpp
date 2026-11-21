// End-to-end planning: scenarios solved with each planner, and the pose-goal pipeline from a
// Cartesian target to a timed, collision-free trajectory.

#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/planning/pose_goal_planning.hpp"
#include "manipulatorcore/planning/scenarios.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

std::shared_ptr<const MotionValidator> scenarioValidator(const RobotCollisionChecker& checker) {
  return std::make_shared<const MotionValidator>(
      JointSpace::fromModel(checker.model()), [&checker](const VecX& q) { return checker.isCollisionFree(q); }, 0.02);
}

TEST(PlanningIntegration, RRTConnectSolvesAllScenarios) {
  for (const PlanningScenario& s : redundant7RScenarios()) {
    const RobotCollisionChecker checker(s.robot, s.obstacles);
    const auto validator = scenarioValidator(checker);
    PlannerSettings settings;
    settings.max_iterations = 20000;
    RRTConnectPlanner planner(validator, settings);
    const PlannerResult r = planner.solve({s.start, {s.goal}});
    ASSERT_TRUE(r.success()) << s.name << ": " << toString(r.status);
    // Collision-free planned path remains collision free when re-checked at a finer resolution.
    const MotionValidator fine(
        JointSpace::fromModel(s.robot), [&checker](const VecX& q) { return checker.isCollisionFree(q); }, 0.005);
    EXPECT_TRUE(fine.isPathValid(r.path)) << s.name;
  }
}

TEST(PlanningIntegration, AllPlannersSolvePlanarScenario) {
  const PlanningScenario s = planar2RScenario();
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto validator = scenarioValidator(checker);
  for (const std::string name : {"RRT", "RRT-Connect", "RRT*", "PRM"}) {
    PlannerSettings settings;
    settings.max_iterations = 5000;
    settings.max_step = 0.3;
    const PlannerResult r = makePlanner(name, validator, settings)->solve({s.start, {s.goal}});
    ASSERT_TRUE(r.success()) << name;
    EXPECT_TRUE(validator->isPathValid(r.path)) << name;
  }
}

TEST(PlanningIntegration, PoseGoalPipelineReachesTargetWithoutCollision) {
  const auto scenarios = redundant7RScenarios();
  const PlanningScenario& s = scenarios[4];  // tabletop_reach
  ASSERT_EQ(s.name, "tabletop_reach");
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const SE3 target = endEffectorPose(s.robot, s.goal);

  PoseGoalPlanningOptions options;
  options.ik.attempts = 40;
  options.ik.seed = 3;
  options.max_goal_candidates = 3;
  const PoseGoalPlanningResult result = planToPose(checker, s.start, target, options);
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_NE(result.trajectory, nullptr);
  EXPECT_LE(result.path_length, result.raw_length + 1e-9);

  // The timed trajectory starts at the start, ends at an IK solution of the target, respects the
  // joint limits and stays collision free when densely sampled.
  const JointTrajectory& traj = *result.trajectory;
  EXPECT_MATRIX_NEAR(traj.sample(0.0).q, s.start, 1e-9);
  const PoseDistance d = poseDistance(endEffectorPose(s.robot, traj.sample(traj.duration()).q), target);
  EXPECT_LT(d.translation, 1e-3);
  EXPECT_LT(d.rotation, 1e-2);
  const LimitUsage usage = evaluateLimitUsage(traj, s.robot.velocityLimits(), s.robot.accelerationLimits(), 2e-3);
  EXPECT_TRUE(usage.within(1e-9)) << "velocity ratio " << usage.max_velocity_ratio << ", acceleration ratio "
                                  << usage.max_acceleration_ratio << ", smoothed " << result.smoothed;
  for (const TrajectoryPoint& p : sampleUniform(traj, 0.01)) {
    ASSERT_TRUE(checker.isCollisionFree(p.q)) << "collision at t = " << p.time;
  }
}

TEST(PlanningIntegration, UnreachablePoseIsReported) {
  const RobotCollisionChecker checker(makeRedundant7R());
  PoseGoalPlanningOptions options;
  options.ik.attempts = 8;
  const PoseGoalPlanningResult r =
      planToPose(checker, redundant7RHome(), SE3::fromTranslation(Vec3(3.0, 0.0, 0.5)), options);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.message, "no collision-free IK solution found");
}

}  // namespace
}  // namespace mc
