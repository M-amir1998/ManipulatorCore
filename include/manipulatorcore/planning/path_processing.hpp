#pragma once

/// @file path_processing.hpp
/// @brief Path length, shortcutting, densification and collision-checked spline smoothing.

#include "manipulatorcore/planning/joint_space.hpp"
#include "manipulatorcore/trajectory/time_parameterization.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace mc {

/// Sum of metric distances between consecutive waypoints.
[[nodiscard]] double pathLength(const JointSpace& space, const std::vector<VecX>& path);

/// Inserts interpolated waypoints so that consecutive waypoints are at most @p max_step apart.
[[nodiscard]] std::vector<VecX> densifyPath(const JointSpace& space, const std::vector<VecX>& path, double max_step);

struct ShortcutOptions {
  int iterations = 300;
  std::uint64_t seed = 7;
};

struct ShortcutResult {
  std::vector<VecX> path;
  double initial_length = 0.0;
  double final_length = 0.0;
  int accepted = 0;
};

/// Random shortcutting: repeatedly picks two random points along the path (anywhere on its
/// segments) and replaces the portion between them by a straight edge when that edge is valid.
/// Every accepted edge is validated with @p validator, so the result stays collision free.
[[nodiscard]] ShortcutResult shortcutPath(const MotionValidator& validator, const std::vector<VecX>& path,
                                          const ShortcutOptions& options = {});

struct SplineSmoothingOptions {
  double check_resolution = 0.01;  ///< arc-length step of the validity checks along the spline
  int max_refinements = 6;         ///< times the waypoint list may be subdivided
};

/// Fits a C2 spline through the waypoints and validates it densely. If the spline leaves the
/// valid set, the polyline is subdivided (pulling the spline towards it) and the fit retried.
/// Returns nullopt if no valid spline was found; callers then keep the polyline.
[[nodiscard]] std::optional<std::shared_ptr<const SplineJointPath>> smoothPathWithSpline(
    const MotionValidator& validator, const std::vector<VecX>& path, const SplineSmoothingOptions& options = {});

}  // namespace mc
