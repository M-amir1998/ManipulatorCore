// Kernel timings for collision queries, planners and controllers.

#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/control/task_space_controllers.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/planning/planner.hpp"
#include "manipulatorcore/planning/scenarios.hpp"

#include <benchmark/benchmark.h>

#include <memory>
#include <vector>

namespace {

using namespace mc;

const PlanningScenario& scenario(int index) {
  static const std::vector<PlanningScenario> all = redundant7RScenarios();
  return all.at(static_cast<std::size_t>(index));
}

std::vector<VecX> configurations(const RobotModel& model) {
  Random rng(4);
  std::vector<VecX> out;
  for (int i = 0; i < 128; ++i) {
    out.push_back(rng.uniformVector(model.lowerLimits(), model.upperLimits()));
  }
  return out;
}

void BM_CollisionFreeQuery(benchmark::State& state) {
  const PlanningScenario& s = scenario(3);  // shelf: 7 obstacles
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto qs = configurations(s.robot);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(checker.isCollisionFree(qs[i++ % qs.size()]));
  }
}
BENCHMARK(BM_CollisionFreeQuery);

void BM_MinimumDistanceQuery(benchmark::State& state) {
  const PlanningScenario& s = scenario(3);
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto qs = configurations(s.robot);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(checker.minimumDistance(qs[i++ % qs.size()]));
  }
}
BENCHMARK(BM_MinimumDistanceQuery);

void BM_Planner(benchmark::State& state, const char* name, int scenario_index) {
  const PlanningScenario& s = scenario(scenario_index);
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto validator = std::make_shared<const MotionValidator>(
      JointSpace::fromModel(s.robot), [&checker](const VecX& q) { return checker.isCollisionFree(q); }, 0.02);
  std::uint64_t seed = 1;
  for (auto _ : state) {
    PlannerSettings settings;
    settings.seed = seed++;
    settings.max_iterations = std::string(name) == "RRT*" ? 1500 : 20000;
    auto planner = makePlanner(name, validator, settings);
    benchmark::DoNotOptimize(planner->solve({s.start, {s.goal}}));
  }
}
BENCHMARK_CAPTURE(BM_Planner, rrt_connect_obstacle_avoidance, "RRT-Connect", 1)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Planner, rrt_connect_shelf, "RRT-Connect", 3)->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(BM_Planner, rrt_star_1500_tabletop, "RRT*", 4)->Unit(benchmark::kMillisecond);

void BM_ControllerCall(benchmark::State& state, int which) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  std::unique_ptr<Controller> c;
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(25.0, 7, kp, kd);
  const SE3 target = endEffectorPose(model, q0);
  switch (which) {
    case 0:
      c = std::make_unique<JointPIDController>(model, JointPIDGains{kp, VecX(), kd}, constantJointReference(q0), true);
      break;
    case 1:
      c = std::make_unique<ComputedTorqueController>(model, kp, kd, constantJointReference(q0));
      break;
    case 2:
      c = std::make_unique<CartesianImpedanceController>(model, diagonalGain(50, 500), diagonalGain(5, 50),
                                                         constantPoseReference(target));
      break;
    default:
      c = std::make_unique<OperationalSpaceController>(model, diagonalGain(100, 100), diagonalGain(20, 20),
                                                       constantPoseReference(target));
  }
  ControllerInput in;
  in.q = q0;
  in.qd = VecX::Constant(7, 0.1);
  for (auto _ : state) {
    in.time += 0.001;
    benchmark::DoNotOptimize(c->computeTorque(in));
  }
}
BENCHMARK_CAPTURE(BM_ControllerCall, pd_gravity, 0)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_ControllerCall, computed_torque, 1)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_ControllerCall, cartesian_impedance, 2)->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_ControllerCall, operational_space, 3)->Unit(benchmark::kMicrosecond);

}  // namespace
