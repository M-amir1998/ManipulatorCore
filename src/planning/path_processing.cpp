#include "manipulatorcore/planning/path_processing.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

double pathLength(const JointSpace& space, const std::vector<VecX>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += space.distance(path[i - 1], path[i]);
  }
  return length;
}

std::vector<VecX> densifyPath(const JointSpace& space, const std::vector<VecX>& path, double max_step) {
  require(max_step > 0.0, "densifyPath: max_step must be positive");
  std::vector<VecX> out;
  if (path.empty()) {
    return out;
  }
  out.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto pieces = std::max(1, static_cast<int>(std::ceil(space.distance(path[i - 1], path[i]) / max_step)));
    for (int k = 1; k <= pieces; ++k) {
      out.push_back(space.interpolate(path[i - 1], path[i], static_cast<double>(k) / pieces));
    }
  }
  return out;
}

ShortcutResult shortcutPath(const MotionValidator& validator, const std::vector<VecX>& path,
                            const ShortcutOptions& options) {
  require(options.iterations >= 0, "ShortcutOptions: iterations must be non-negative");
  const JointSpace& space = validator.space();
  ShortcutResult result;
  result.path = path;
  result.initial_length = pathLength(space, path);
  Random rng(options.seed);
  for (int it = 0; it < options.iterations && result.path.size() >= 3; ++it) {
    std::vector<double> cumulative{0.0};
    for (std::size_t i = 1; i < result.path.size(); ++i) {
      cumulative.push_back(cumulative.back() + space.distance(result.path[i - 1], result.path[i]));
    }
    const double total = cumulative.back();
    if (total <= 0.0) {
      break;
    }
    double s1 = rng.uniform(0.0, total);
    double s2 = rng.uniform(0.0, total);
    if (s1 > s2) {
      std::swap(s1, s2);
    }
    const auto locate = [&](double s) {
      const auto it_seg = std::upper_bound(cumulative.begin(), cumulative.end(), s);
      const auto seg = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(it_seg - cumulative.begin(), 1,
                                                                           static_cast<std::ptrdiff_t>(cumulative.size()) - 1)) -
                       1;
      const double len = cumulative[seg + 1] - cumulative[seg];
      const double t = len > 0.0 ? (s - cumulative[seg]) / len : 0.0;
      return std::make_pair(seg, space.interpolate(result.path[seg], result.path[seg + 1], t));
    };
    const auto [i1, q1] = locate(s1);
    const auto [i2, q2] = locate(s2);
    if (i1 == i2) {
      continue;  // already a straight segment
    }
    // The detour q1 -> path[i1+1] -> ... -> path[i2] -> q2 is replaced by the edge q1 -> q2.
    if (!validator.isEdgeValid(q1, q2)) {
      continue;
    }
    std::vector<VecX> next(result.path.begin(), result.path.begin() + static_cast<std::ptrdiff_t>(i1) + 1);
    if (space.distance(next.back(), q1) > 1e-12) {
      next.push_back(q1);
    }
    if (space.distance(next.back(), q2) > 1e-12) {
      next.push_back(q2);
    }
    for (std::size_t k = i2 + 1; k < result.path.size(); ++k) {
      if (space.distance(next.back(), result.path[k]) > 1e-12) {
        next.push_back(result.path[k]);
      }
    }
    if (pathLength(space, next) < pathLength(space, result.path) - 1e-12) {
      result.path = std::move(next);
      ++result.accepted;
    }
  }
  result.final_length = pathLength(space, result.path);
  return result;
}

std::optional<std::shared_ptr<const SplineJointPath>> smoothPathWithSpline(const MotionValidator& validator,
                                                                           const std::vector<VecX>& path,
                                                                           const SplineSmoothingOptions& options) {
  require(options.check_resolution > 0.0 && options.max_refinements >= 0, "SplineSmoothingOptions: invalid options");
  if (path.size() < 2 || pathLength(validator.space(), path) <= 1e-12) {
    return std::nullopt;
  }
  std::vector<VecX> waypoints = path;
  for (int attempt = 0; attempt <= options.max_refinements; ++attempt) {
    auto spline = std::make_shared<const SplineJointPath>(waypoints);
    const double L = spline->length();
    const auto steps = std::max(1, static_cast<int>(std::ceil(L / options.check_resolution)));
    bool valid = true;
    for (int k = 0; k <= steps && valid; ++k) {
      valid = validator.isValid(spline->position(L * static_cast<double>(k) / steps));
    }
    if (valid) {
      return spline;
    }
    // Subdivide every segment so the interpolating spline hugs the (valid) polyline more tightly.
    std::vector<VecX> refined{waypoints.front()};
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      refined.push_back(0.5 * (waypoints[i - 1] + waypoints[i]));
      refined.push_back(waypoints[i]);
    }
    waypoints = std::move(refined);
  }
  return std::nullopt;
}

}  // namespace mc
