// Integrated pipeline on the 7-DOF arm:
//   target pose -> multi-start IK -> collision filtering -> RRT-Connect to each candidate
//   -> shortcutting -> spline + time-optimal parameterization -> computed-torque execution.

#include "../common/export.hpp"
#include "demos.hpp"
#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/planning/pose_goal_planning.hpp"
#include "manipulatorcore/planning/scenarios.hpp"
#include "manipulatorcore/simulation/simulator.hpp"

#include <cmath>
#include <cstdio>

namespace mc::demos {

using apps::Json;

namespace {

Json eeTrace(const RobotModel& model, const std::vector<VecX>& path) {
  Json pts = Json::array();
  for (const VecX& q : path) {
    pts.push(apps::vec3(endEffectorPose(model, q).translation()));
  }
  return pts;
}

void hero(const std::filesystem::path& out) {
  const auto scenarios = redundant7RScenarios();
  const PlanningScenario& s = scenarios[3];  // shelf_reach
  const RobotModel& model = s.robot;
  const RobotCollisionChecker checker(model, s.obstacles);
  const SE3 target = endEffectorPose(model, s.goal);

  PoseGoalPlanningOptions options;
  options.ik.attempts = 48;
  options.ik.seed = 11;
  options.ik.max_solutions = 6;
  options.max_goal_candidates = 4;
  options.planner_settings.seed = 3;
  // Run at half the velocity and about a third of the acceleration limits, as a cautious cell would.
  options.timing.velocity_scale = 0.5;
  options.timing.acceleration_scale = 0.35;
  const PoseGoalPlanningResult plan = planToPose(checker, s.start, target, options);
  if (!plan.success) {
    std::printf("  hero: planning failed: %s\n", plan.message.c_str());
    return;
  }

  // Execute the timed trajectory with computed-torque control in the simulator.
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(30.0, model.dof(), kp, kd);
  ComputedTorqueController controller(model, kp, kd, trajectoryReference(plan.trajectory));
  SimulatorOptions sim_options;
  sim_options.record_every = 20;  // 50 frames per simulated second
  Simulator sim(model, sim_options);
  sim.reset(s.start);
  const double settle = 0.6;
  const SimulationLog& log = sim.run(controller, plan.trajectory->duration() + settle);

  Json executed = Json::array();
  double max_tracking = 0.0;
  bool collision = false;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    const Vec3 ref = endEffectorPose(model, plan.trajectory->sample(log.time[k]).q).translation();
    max_tracking = std::max(max_tracking, (ref - log.ee_pose[k].translation()).norm());
    collision = collision || !checker.isCollisionFree(log.q[k]);
    executed.push(apps::robotFrame(model, log.q[k], log.time[k]).set("reference_ee", apps::vec3(ref)));
  }
  const PoseDistance final_error = poseDistance(log.ee_pose.back(), target);

  Json candidates = Json::array();
  for (const VecX& q : plan.ik_candidates) {
    candidates.push(apps::robotFrame(model, q));
  }
  Json planned_trace = Json::array();
  for (const TrajectoryPoint& p : sampleUniform(*plan.trajectory, 0.02)) {
    planned_trace.push(apps::vec3(endEffectorPose(model, p.q).translation()));
  }
  Json doc = Json::object();
  doc.set("scenario", s.name)
      .set("obstacles", apps::shapes(s.obstacles))
      .set("target", apps::pose(target))
      .set("start", apps::robotFrame(model, s.start))
      .set("ik_candidates", candidates)
      .set("chosen_candidate", plan.chosen_candidate)
      .set("raw_path_ee", eeTrace(model, plan.raw_path))
      .set("shortcut_path_ee", eeTrace(model, plan.path))
      .set("planned_ee", planned_trace)
      .set("executed", executed)
      .set("summary", Json::object()
                          .set("ik_candidates", plan.ik_candidates.size())
                          .set("raw_length", plan.raw_length)
                          .set("shortcut_length", plan.path_length)
                          .set("smoothed", plan.smoothed)
                          .set("trajectory_duration", plan.trajectory->duration())
                          .set("planner_nodes", plan.planner_stats.nodes)
                          .set("collision_checks", static_cast<double>(plan.planner_stats.collision_checks))
                          .set("max_tracking_error", max_tracking)
                          .set("final_position_error", final_error.translation)
                          .set("final_orientation_error", final_error.rotation)
                          .set("collision_during_execution", collision));
  apps::writeJson(out, "hero", doc);
  std::printf(
      "  hero: %zu IK candidates, path %.2f -> %.2f rad, %.2f s trajectory, tracking <= %.1f mm, final error "
      "%.2f mm, collision-free execution: %s\n",
      plan.ik_candidates.size(), plan.raw_length, plan.path_length, plan.trajectory->duration(), max_tracking * 1e3,
      final_error.translation * 1e3, collision ? "no" : "yes");
}

}  // namespace

std::vector<Demo> heroDemos() {
  return {{"hero", "Integrated IK + RRT-Connect + time parameterization + computed-torque execution", hero}};
}

}  // namespace mc::demos
