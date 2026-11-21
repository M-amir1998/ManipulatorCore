// Plan a collision-free motion to a Cartesian goal and execute it with computed-torque control in
// the simulator.

#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/planning/pose_goal_planning.hpp"
#include "manipulatorcore/planning/scenarios.hpp"
#include "manipulatorcore/simulation/simulator.hpp"

#include <cstdio>

int main() {
  using namespace mc;
  const PlanningScenario scene = redundant7RScenarios().at(4);  // tabletop_reach
  const RobotCollisionChecker checker(scene.robot, scene.obstacles);
  const SE3 goal = endEffectorPose(scene.robot, scene.goal);

  // Multi-start IK -> RRT-Connect to each candidate -> shortcut -> spline + time-optimal timing.
  const PoseGoalPlanningResult plan = planToPose(checker, scene.start, goal);
  if (!plan.success) {
    std::printf("planning failed: %s\n", plan.message.c_str());
    return 1;
  }
  std::printf("%zu IK candidates, path %.2f -> %.2f rad after shortcutting, %.2f s trajectory\n",
              plan.ik_candidates.size(), plan.raw_length, plan.path_length, plan.trajectory->duration());

  // Track the trajectory with computed torque in the simulator.
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(30.0, scene.robot.dof(), kp, kd);
  ComputedTorqueController controller(scene.robot, kp, kd, trajectoryReference(plan.trajectory));
  Simulator sim(scene.robot);
  sim.reset(scene.start);
  sim.run(controller, plan.trajectory->duration() + 0.5);

  const PoseDistance error = poseDistance(endEffectorPose(scene.robot, sim.q()), goal);
  std::printf("final end-effector error: %.2f mm, %.2f mrad\n", 1e3 * error.translation, 1e3 * error.rotation);
  return error.translation < 2e-3 ? 0 : 1;
}
