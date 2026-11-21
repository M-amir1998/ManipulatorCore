// Planning demos: configuration-space exploration of the planar 2R arm, narrow-passage statistics
// and Cartesian interpolation versus configuration-space planning on the 7-DOF arm.

#include "../common/export.hpp"
#include "demos.hpp"
#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/planning/path_processing.hpp"
#include "manipulatorcore/planning/planner.hpp"
#include "manipulatorcore/planning/pose_goal_planning.hpp"
#include "manipulatorcore/planning/scenarios.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"

#include <cstdio>
#include <memory>

namespace mc::demos {

using apps::Json;

namespace {

/// Exact occupancy of the 2-D configuration space on a regular grid (row = q2, column = q1).
Json cspaceGrid(const RobotCollisionChecker& checker, int n) {
  const VecX lo = checker.model().lowerLimits();
  const VecX hi = checker.model().upperLimits();
  Json rows = Json::array();
  for (int j = 0; j < n; ++j) {
    Json row = Json::array();
    for (int i = 0; i < n; ++i) {
      const Vec2 q(lo(0) + (hi(0) - lo(0)) * (i + 0.5) / n, lo(1) + (hi(1) - lo(1)) * (j + 0.5) / n);
      row.push(checker.isCollisionFree(q) ? 0 : 1);
    }
    rows.push(row);
  }
  return Json::object().set("n", n).set("lower", apps::vec(lo)).set("upper", apps::vec(hi)).set("occupied", rows);
}

Json treeJson(const PlannerTree& tree) {
  Json nodes = Json::array();
  for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
    nodes.push(
        Json::array().push(tree.nodes[i](0)).push(tree.nodes[i](1)).push(tree.parents[i]).push(tree.tree_index[i]));
  }
  Json edges = Json::array();
  for (const auto& [a, b] : tree.edges) {
    edges.push(Json::array().push(a).push(b));
  }
  return Json::object().set("nodes", nodes).set("edges", edges);
}

Json pathJson(const std::vector<VecX>& path) {
  Json a = Json::array();
  for (const VecX& q : path) {
    a.push(apps::vec(q));
  }
  return a;
}

std::shared_ptr<const MotionValidator> validatorFor(const RobotCollisionChecker& checker, double resolution) {
  return std::make_shared<const MotionValidator>(
      JointSpace::fromModel(checker.model()), [&checker](const VecX& q) { return checker.isCollisionFree(q); },
      resolution);
}

void rrtPlanar(const std::filesystem::path& out) {
  const PlanningScenario s = planar2RScenario();
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto validator = validatorFor(checker, 0.01);
  Json runs = Json::object();
  for (const std::string name : {"RRT", "RRT-Connect", "RRT*", "PRM"}) {
    PlannerSettings settings;
    settings.seed = 4;
    settings.max_step = 0.25;
    settings.max_iterations = name == "RRT*" ? 2500 : 6000;
    settings.max_rewire_radius = 0.8;
    settings.prm_batch = 150;
    settings.record_tree = true;
    const PlannerResult r = makePlanner(name, validator, settings)->solve({s.start, {s.goal}});
    const ShortcutResult sc = shortcutPath(*validator, r.path, {.iterations = 200, .seed = 1});
    Json execution = Json::array();
    if (r.success()) {
      bool spline = false;
      const auto traj = timeParameterizePath(*validator, s.robot, sc.path, true, {}, {}, &spline);
      for (const TrajectoryPoint& p : sampleUniform(*traj, traj->duration() / 60.0)) {
        execution.push(apps::robotFrame(s.robot, p.q, p.time));
      }
    }
    runs.set(name, Json::object()
                       .set("success", r.success())
                       .set("tree", treeJson(r.tree))
                       .set("path", pathJson(r.path))
                       .set("shortcut", pathJson(sc.path))
                       .set("path_length", r.path_length)
                       .set("shortcut_length", sc.final_length)
                       .set("nodes", r.stats.nodes)
                       .set("collision_checks", static_cast<double>(r.stats.collision_checks))
                       .set("execution", execution));
    std::printf("  rrt_2r/%s: %s, %zu nodes, length %.2f -> %.2f\n", name.c_str(), toString(r.status), r.stats.nodes,
                r.path_length, sc.final_length);
  }
  Json doc = Json::object();
  doc.set("obstacles", apps::shapes(s.obstacles))
      .set("start", apps::robotFrame(s.robot, s.start))
      .set("goal", apps::robotFrame(s.robot, s.goal))
      .set("cspace", cspaceGrid(checker, 180))
      .set("runs", runs);
  apps::writeJson(out, "rrt_2r", doc);
}

void narrowPassage(const std::filesystem::path& out) {
  const PlanningScenario s = planar2RNarrowPassageScenario();
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const auto validator = validatorFor(checker, 0.01);
  Json stats = Json::object();
  Json trees = Json::object();
  for (const std::string name : {"RRT", "RRT-Connect"}) {
    int successes = 0;
    double iterations = 0.0;
    const int trials = 20;
    for (int seed = 1; seed <= trials; ++seed) {
      PlannerSettings settings;
      settings.seed = static_cast<std::uint64_t>(seed);
      settings.max_step = 0.25;
      settings.max_iterations = 3000;
      settings.record_tree = seed == 1;
      const PlannerResult r = makePlanner(name, validator, settings)->solve({s.start, {s.goal}});
      successes += r.success() ? 1 : 0;
      iterations += r.stats.iterations;
      if (seed == 1) {
        trees.set(
            name,
            Json::object().set("success", r.success()).set("tree", treeJson(r.tree)).set("path", pathJson(r.path)));
      }
    }
    stats.set(name, Json::object()
                        .set("success_rate", successes / static_cast<double>(trials))
                        .set("mean_iterations", iterations / trials));
    std::printf("  narrow_passage/%s: %d/%d solved within 3000 iterations\n", name.c_str(), successes, trials);
  }
  Json doc = Json::object();
  doc.set("obstacles", apps::shapes(s.obstacles))
      .set("start", apps::robotFrame(s.robot, s.start))
      .set("goal", apps::robotFrame(s.robot, s.goal))
      .set("cspace", cspaceGrid(checker, 180))
      .set("stats", stats)
      .set("runs", trees);
  apps::writeJson(out, "narrow_passage", doc);
}

/// Straight-line Cartesian interpolation between two grasps collides with the pillar; planning in
/// configuration space does not.
void cartesianVsCspace(const std::filesystem::path& out) {
  const auto scenarios = redundant7RScenarios();
  const PlanningScenario& s = scenarios[1];  // obstacle_avoidance
  const RobotCollisionChecker checker(s.robot, s.obstacles);
  const SE3 A = endEffectorPose(s.robot, s.start);
  const SE3 B = endEffectorPose(s.robot, s.goal);
  const auto line = std::make_shared<LinePath>(A, B);
  Json cartesian = Json::array();
  VecX q = s.start;
  int colliding = 0;
  for (int k = 0; k <= 80; ++k) {
    const SE3 target = line->pose(k / 80.0);
    q = solveIK(s.robot, target, q, {}).q;
    const bool free = checker.isCollisionFree(q);
    colliding += free ? 0 : 1;
    cartesian.push(apps::robotFrame(s.robot, q, k / 80.0).set("collision", !free));
  }
  PoseGoalPlanningOptions options;
  options.ik.seed = 2;
  const PoseGoalPlanningResult plan = planToPose(checker, s.start, B, options);
  Json planned = Json::array();
  if (plan.success) {
    for (const TrajectoryPoint& p : sampleUniform(*plan.trajectory, plan.trajectory->duration() / 80.0)) {
      planned.push(apps::robotFrame(s.robot, p.q, p.time).set("collision", !checker.isCollisionFree(p.q)));
    }
  }
  std::printf("  cartesian_vs_cspace: %d/81 Cartesian samples collide; planned path %s\n", colliding,
              plan.success ? "collision free" : plan.message.c_str());
  Json doc = Json::object();
  doc.set("obstacles", apps::shapes(s.obstacles))
      .set("base", apps::shapes(s.robot.baseCollision()))
      .set("start_pose", apps::pose(A))
      .set("goal_pose", apps::pose(B))
      .set("cartesian", cartesian)
      .set("planned", planned);
  apps::writeJson(out, "cartesian_vs_cspace", doc);
}

}  // namespace

std::vector<Demo> planningDemos() {
  return {{"rrt_2r", "RRT, RRT-Connect, RRT* and PRM in the exact 2-D configuration space", rrtPlanar},
          {"narrow_passage", "RRT versus RRT-Connect through a narrow passage", narrowPassage},
          {"cartesian_vs_cspace", "Cartesian straight-line interpolation versus configuration-space planning",
           cartesianVsCspace}};
}

}  // namespace mc::demos
