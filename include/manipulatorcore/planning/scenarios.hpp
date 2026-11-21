#pragma once

/// @file scenarios.hpp
/// @brief Deterministic planning scenarios used by tests, benchmarks and demos.

#include "manipulatorcore/geometry/shapes.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <string>
#include <vector>

namespace mc {

struct PlanningScenario {
  std::string name;
  std::string description;
  RobotModel robot;
  std::vector<Shape> obstacles;
  VecX start;
  VecX goal;
};

/// Scenarios for the generic 7-DOF arm: open reach, obstacle avoidance, narrow passage, shelf
/// reach, tabletop reach and around obstacle. Every start and goal is collision free.
[[nodiscard]] std::vector<PlanningScenario> redundant7RScenarios();

/// A planar 2R arm among circular obstacles; its 2-D configuration space can be drawn exactly.
[[nodiscard]] PlanningScenario planar2RScenario();

/// A planar 2R narrow-passage variant (two obstacles leave a thin free channel in C-space).
[[nodiscard]] PlanningScenario planar2RNarrowPassageScenario();

}  // namespace mc
