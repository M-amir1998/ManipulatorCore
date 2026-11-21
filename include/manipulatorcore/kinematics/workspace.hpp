#pragma once

/// @file workspace.hpp
/// @brief Deterministic sampling of the reachable workspace and manipulability maps.

#include "manipulatorcore/model/robot_model.hpp"

#include <cstdint>
#include <vector>

namespace mc {

struct WorkspaceSample {
  VecX q;
  Vec3 position;          ///< end-effector origin in the base frame
  double manipulability;  ///< Yoshikawa index of the selected geometric-Jacobian rows
};

struct WorkspaceOptions {
  int samples = 20000;
  std::uint64_t seed = 7;
  /// Geometric-Jacobian rows used for the manipulability index. The default {3,4,5} is the
  /// translational block; planar arms should use {3,4}.
  std::vector<int> manipulability_rows = {3, 4, 5};
};

/// Uniform random sampling of the joint-limit box mapped through forward kinematics.
[[nodiscard]] std::vector<WorkspaceSample> sampleWorkspace(const RobotModel& model,
                                                           const WorkspaceOptions& options = {});

/// Axis-aligned bounds and maximum distance from the base origin of a sample set.
struct WorkspaceBounds {
  Vec3 min = Vec3::Zero();
  Vec3 max = Vec3::Zero();
  double max_reach = 0.0;
};

[[nodiscard]] WorkspaceBounds workspaceBounds(const std::vector<WorkspaceSample>& samples);

}  // namespace mc
