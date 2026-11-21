#include "manipulatorcore/planning/pose_goal_planning.hpp"

#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"

#include <algorithm>
#include <limits>
#include <string>

namespace mc {

std::shared_ptr<const JointTrajectory> timeParameterizePath(const MotionValidator& validator, const RobotModel& model,
                                                            const std::vector<VecX>& path, bool smooth,
                                                            const SplineSmoothingOptions& smoothing,
                                                            const TimeParameterizationOptions& timing,
                                                            bool* used_spline) {
  require(!path.empty(), "timeParameterizePath: empty path");
  if (used_spline != nullptr) {
    *used_spline = false;
  }
  if (smooth) {
    if (const auto spline = smoothPathWithSpline(validator, path, smoothing)) {
      if (used_spline != nullptr) {
        *used_spline = true;
      }
      return std::make_shared<TimeOptimalTrajectory>(
          timeOptimalParameterization(*spline, model.velocityLimits(), model.accelerationLimits(), timing));
    }
  }
  return std::make_shared<CompositeTrajectory>(
      stopAtWaypointsTrajectory(path, model.velocityLimits(), model.accelerationLimits()));
}

PoseGoalPlanningResult planToPose(const RobotCollisionChecker& checker, const VecX& start, const SE3& target,
                                  const PoseGoalPlanningOptions& options) {
  const RobotModel& model = checker.model();
  model.checkConfiguration(start, "planToPose: start");
  require(options.max_goal_candidates > 0, "PoseGoalPlanningOptions: max_goal_candidates must be positive");
  require(options.clearance >= 0.0 && options.verification_dt > 0.0,
          "PoseGoalPlanningOptions: clearance must be >= 0 and verification_dt > 0");
  PoseGoalPlanningResult result;
  // Plan against a copy of the checker with the environment clearance applied.
  RobotCollisionChecker planning_checker = checker;
  CollisionCheckerOptions inflated = checker.options();
  inflated.safety_margin = std::max(inflated.safety_margin, options.clearance);
  planning_checker.setOptions(inflated);
  const auto collision_free = [&planning_checker](const VecX& q) { return planning_checker.isCollisionFree(q); };
  const auto validator =
      std::make_shared<const MotionValidator>(JointSpace::fromModel(model), collision_free, options.edge_resolution);
  if (!validator->isValid(start)) {
    result.message = "start configuration violates the planning clearance or the joint limits";
    return result;
  }

  // 1. Collision-free IK candidates.
  MultiStartIKOptions ik = options.ik;
  ik.validator = collision_free;
  for (const IKResult& r : solveIKMultiStart(model, target, start, ik)) {
    result.ik_candidates.push_back(r.q);
  }
  if (result.ik_candidates.empty()) {
    result.message = "no collision-free IK solution found";
    return result;
  }

  // 2. Plan to each candidate, shortcut, keep the shortest.
  double best = std::numeric_limits<double>::infinity();
  const int candidates = std::min<int>(options.max_goal_candidates, static_cast<int>(result.ik_candidates.size()));
  for (int c = 0; c < candidates; ++c) {
    PlannerSettings settings = options.planner_settings;
    settings.seed += static_cast<std::uint64_t>(c);
    auto planner = makePlanner(options.planner, validator, settings);
    const PlannerResult plan = planner->solve({start, {result.ik_candidates[static_cast<std::size_t>(c)]}});
    result.planner_stats.iterations += plan.stats.iterations;
    result.planner_stats.nodes += plan.stats.nodes;
    result.planner_stats.collision_checks += plan.stats.collision_checks;
    result.planner_stats.edge_checks += plan.stats.edge_checks;
    result.planner_stats.planning_time += plan.stats.planning_time;
    if (!plan.success()) {
      continue;
    }
    ShortcutResult shortcut = shortcutPath(*validator, plan.path, options.shortcut);
    if (shortcut.final_length < best) {
      best = shortcut.final_length;
      result.chosen_candidate = c;
      result.raw_path = plan.path;
      result.raw_length = plan.path_length;
      result.path = std::move(shortcut.path);
      result.path_length = shortcut.final_length;
    }
  }
  if (result.chosen_candidate < 0) {
    result.message = "the planner did not reach any IK candidate within its budget";
    return result;
  }

  // 3. Time parameterization (spline smoothing is revalidated before use).
  result.trajectory = timeParameterizePath(*validator, model, result.path, options.smooth, options.smoothing,
                                           options.timing, &result.smoothed);

  // 4. Independent verification of the timed trajectory against the caller's checker.
  for (const TrajectoryPoint& p : sampleUniform(*result.trajectory, options.verification_dt)) {
    if (!checker.isCollisionFree(p.q)) {
      result.message = "the timed trajectory failed collision verification at t = " + std::to_string(p.time);
      return result;
    }
  }
  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace mc
