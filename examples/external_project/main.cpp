// Downstream usage of an installed ManipulatorCore through find_package().

#include <manipulatorcore/collision/robot_collision.hpp>
#include <manipulatorcore/control/joint_controllers.hpp>
#include <manipulatorcore/planning/planner.hpp>
#include <manipulatorcore/planning/scenarios.hpp>

#include <cstdio>
#include <memory>

int main() {
  const mc::PlanningScenario scene = mc::planar2RScenario();
  const mc::RobotCollisionChecker checker(scene.robot, scene.obstacles);
  const auto validator = std::make_shared<const mc::MotionValidator>(
      mc::JointSpace::fromModel(scene.robot), [&checker](const mc::VecX& q) { return checker.isCollisionFree(q); });
  mc::RRTConnectPlanner planner(validator, mc::PlannerSettings{});
  const mc::PlannerResult result = planner.solve({scene.start, {scene.goal}});
  std::printf("RRT-Connect: %s, %zu waypoints, %zu collision checks\n", mc::toString(result.status), result.path.size(),
              static_cast<std::size_t>(result.stats.collision_checks));

  mc::GravityCompensationController hold(scene.robot);
  mc::ControllerInput input;
  input.q = scene.start;
  input.qd = mc::VecX::Zero(scene.robot.dof());
  const mc::VecX tau = hold.computeTorque(input);
  std::printf("gravity torque at the start: [%.2f %.2f] N m\n", tau(0), tau(1));
  return result.success() ? 0 : 1;
}
