#include "benchmarks.hpp"
#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/planning/path_processing.hpp"
#include "manipulatorcore/planning/planner.hpp"
#include "manipulatorcore/planning/scenarios.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <vector>

namespace mc::bench {

namespace {

struct Aggregate {
  int runs = 0;
  int success = 0;
  std::vector<double> time_ms;
  double raw_length = 0.0;
  double short_length = 0.0;
  double nodes = 0.0;
  double checks = 0.0;
};

double median(std::vector<double> v) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

PlannerSettings settingsFor(const std::string& planner, std::uint64_t seed) {
  PlannerSettings s;
  s.seed = seed;
  s.max_step = 0.4;
  s.max_iterations = planner == "RRT*" ? 3000 : (planner == "PRM" ? 6000 : 20000);
  s.max_rewire_radius = 1.2;
  return s;
}

std::string row(const std::string& label, const Aggregate& a) {
  std::ostringstream os;
  const double n = std::max(1, a.success);
  os << "| " << label << " | " << fmt("%.0f %%", 100.0 * a.success / std::max(1, a.runs)) << " | "
     << fmt("%.1f ms", median(a.time_ms)) << " | " << (a.success > 0 ? fmt("%.2f", a.raw_length / n) : std::string("-"))
     << " | " << (a.success > 0 ? fmt("%.2f", a.short_length / n) : std::string("-")) << " | "
     << fmt("%.0f", a.nodes / std::max(1, a.runs)) << " | " << fmt("%.0f", a.checks / std::max(1, a.runs)) << " |\n";
  return os.str();
}

}  // namespace

std::string runPlanningBenchmark(int seeds) {
  const std::vector<std::string> planners = {"RRT", "RRT-Connect", "RRT*", "PRM"};
  std::map<std::string, Aggregate> per_planner;
  std::ostringstream detail;
  detail << "### Per scenario\n\n"
         << "| Scenario | Planner | Success | Median time | Path length | After shortcut | Nodes | Collision checks |\n"
         << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
  for (const PlanningScenario& s : redundant7RScenarios()) {
    const RobotCollisionChecker checker(s.robot, s.obstacles);
    const auto validator = std::make_shared<const MotionValidator>(
        JointSpace::fromModel(s.robot), [&checker](const VecX& q) { return checker.isCollisionFree(q); }, 0.02);
    for (const std::string& name : planners) {
      Aggregate a;
      for (int seed = 1; seed <= seeds; ++seed) {
        auto planner = makePlanner(name, validator, settingsFor(name, static_cast<std::uint64_t>(seed)));
        const auto t0 = std::chrono::steady_clock::now();
        const PlannerResult r = planner->solve({s.start, {s.goal}});
        a.time_ms.push_back(1e3 * secondsSince(t0));
        ++a.runs;
        a.nodes += static_cast<double>(r.stats.nodes);
        a.checks += static_cast<double>(r.stats.collision_checks);
        if (r.success()) {
          ++a.success;
          a.raw_length += r.path_length;
          a.short_length += shortcutPath(*validator, r.path, {.iterations = 200, .seed = 1}).final_length;
        }
      }
      detail << "| " << s.name << " " << row(name, a);
      Aggregate& total = per_planner[name];
      total.runs += a.runs;
      total.success += a.success;
      total.time_ms.insert(total.time_ms.end(), a.time_ms.begin(), a.time_ms.end());
      total.raw_length += a.raw_length;
      total.short_length += a.short_length;
      total.nodes += a.nodes;
      total.checks += a.checks;
      std::printf("  planning/%s/%s: %d/%d\n", s.name.c_str(), name.c_str(), a.success, a.runs);
    }
  }
  std::ostringstream md;
  md << "## Motion planning\n\n"
     << "Protocol: the six deterministic 7-DOF scenarios of `redundant7RScenarios()` (open reach, obstacle "
        "avoidance, narrow passage, shelf reach, tabletop reach, around obstacle), "
     << seeds
     << " seeds each. Budgets: 20 000 samples for RRT and RRT-Connect, 3 000 for RRT* (which always uses its "
        "whole budget to improve the path), 6 000 roadmap samples for PRM (grown in batches until the query "
        "succeeds). Steering step 0.4 rad, edge resolution 0.02 rad. Path length is the joint-space length in "
        "radians before and after 200 random shortcut iterations. Collision checks count state-validity queries.\n\n"
     << "### Summary over all scenarios\n\n"
     << "| Planner | Success | Median time | Path length | After shortcut | Nodes | Collision checks |\n"
     << "| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
  for (const std::string& name : planners) {
    md << row(name, per_planner[name]);
  }
  md << "\n" << detail.str() << "\n";
  return md.str();
}

}  // namespace mc::bench
