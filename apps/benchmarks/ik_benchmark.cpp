#include "benchmarks.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

#include <algorithm>
#include <sstream>
#include <vector>

namespace mc::bench {

namespace {

struct Method {
  std::string label;
  IKOptions options;
  int restarts = 0;  // > 0: multi-start
};

double median(std::vector<double> v) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

}  // namespace

std::string runIkBenchmark(int targets) {
  std::vector<Method> methods;
  {
    IKOptions o;
    o.method = IKMethod::JacobianTranspose;
    o.max_iterations = 1000;
    methods.push_back({"Jacobian transpose", o});
    o.method = IKMethod::PseudoInverse;
    o.max_iterations = 200;
    methods.push_back({"Pseudoinverse (Gauss-Newton)", o});
    o.method = IKMethod::DampedLeastSquares;
    o.damping = 0.05;
    methods.push_back({"DLS (lambda = 0.05)", o});
    o.adaptive_damping = true;
    o.damping = 0.1;
    methods.push_back({"DLS (adaptive)", o});
    o.adaptive_damping = false;
    o.method = IKMethod::LevenbergMarquardt;
    methods.push_back({"Levenberg-Marquardt", o});
    o.method = IKMethod::DampedLeastSquares;
    o.damping = 0.05;
    methods.push_back({"DLS + 8 random restarts", o, 8});
  }

  std::ostringstream md;
  md << "## Inverse kinematics\n\n"
     << "Protocol: " << targets
     << " reachable targets per robot, generated as the forward kinematics of uniformly random joint "
        "configurations inside the limits (seed 2024). Every solve starts from an independent uniformly random "
        "configuration (cold start). Tolerances: 0.1 mm and 1 mrad on the full 6-D pose. Iterations are averaged "
        "over converged solves; errors are medians over all solves. A limit violation is a returned configuration "
        "outside the joint limits.\n\n";

  for (const RobotModel& model : {makeIndustrial6R(), makeRedundant7R()}) {
    md << "### " << model.name() << "\n\n"
       << "| Method | Success | Mean iterations | Median position error | Median orientation error | Mean time per "
          "solve | Limit violations |\n"
       << "| --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";
    Random target_rng(2024);
    std::vector<SE3> goals;
    for (int k = 0; k < targets; ++k) {
      goals.push_back(endEffectorPose(model, target_rng.uniformVector(model.lowerLimits(), model.upperLimits())));
    }
    for (const Method& m : methods) {
      Random seed_rng(77);
      int success = 0;
      int violations = 0;
      double iterations = 0.0;
      std::vector<double> pos;
      std::vector<double> ori;
      const auto t0 = std::chrono::steady_clock::now();
      for (const SE3& goal : goals) {
        const VecX seed = seed_rng.uniformVector(model.lowerLimits(), model.upperLimits());
        IKResult r;
        if (m.restarts > 0) {
          MultiStartIKOptions ms;
          ms.ik = m.options;
          ms.attempts = m.restarts;
          ms.max_solutions = 1;
          ms.seed = seed_rng.nextU64();
          const auto sols = solveIKMultiStart(model, goal, seed, ms);
          r = sols.empty() ? solveIK(model, goal, seed, m.options) : sols.front();
        } else {
          r = solveIK(model, goal, seed, m.options);
        }
        success += r.converged() ? 1 : 0;
        violations += model.withinLimits(r.q) ? 0 : 1;
        if (r.converged()) {
          iterations += r.iterations;
        }
        pos.push_back(r.position_error);
        ori.push_back(r.orientation_error);
      }
      const double elapsed = secondsSince(t0);
      md << "| " << m.label << " | " << fmt("%.1f %%", 100.0 * success / targets) << " | "
         << (success > 0 ? fmt("%.1f", iterations / success) : std::string("-")) << " | " << fmt("%.2e m", median(pos))
         << " | " << fmt("%.2e rad", median(ori)) << " | " << fmt("%.0f us", 1e6 * elapsed / targets) << " | "
         << violations << " |\n";
      std::printf("  ik/%s/%s: %d/%d\n", model.name().c_str(), m.label.c_str(), success, targets);
    }
    md << "\n";
  }
  md << "The Jacobian transpose converges linearly and is slow on full 6-D targets; the pseudoinverse is fast "
        "but less robust from cold starts near singular configurations; damping trades a few iterations for "
        "robustness; restarts address the local nature of all gradient-based methods.\n\n";
  return md.str();
}

}  // namespace mc::bench
