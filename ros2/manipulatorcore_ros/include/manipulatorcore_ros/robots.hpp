#pragma once

/// @file robots.hpp
/// @brief Name-based access to the ManipulatorCore built-in robots and planning scenarios, shared
/// by the command-line tools and nodes of this package.

#include "manipulatorcore/math/types.hpp"
#include "manipulatorcore/model/robot_model.hpp"
#include "manipulatorcore/planning/scenarios.hpp"

#include <optional>
#include <string>
#include <vector>

namespace manipulatorcore_ros {

/// Robot names accepted by makeRobot(): redundant7r, industrial6r, planar2r, planar3r.
[[nodiscard]] const std::vector<std::string>& builtinRobotNames();

/// The built-in robot with the given name. Throws std::invalid_argument for unknown names.
[[nodiscard]] mc::RobotModel makeRobot(const std::string& name);

/// Default configuration of a built-in robot: the library's home configuration for redundant7r
/// and industrial6r, a slightly bent configuration for the planar arms.
[[nodiscard]] mc::VecX homeConfiguration(const std::string& name);

/// Joint names of a model in joint order.
[[nodiscard]] std::vector<std::string> jointNames(const mc::RobotModel& model);

/// Names of the planning scenarios available for @p robot (empty for robots without scenarios).
[[nodiscard]] std::vector<std::string> scenarioNames(const std::string& robot);

/// The scenario with the given name for @p robot, or std::nullopt if there is none.
[[nodiscard]] std::optional<mc::PlanningScenario> findScenario(const std::string& robot, const std::string& name);

/// Comma-separated list, used in error messages.
[[nodiscard]] std::string joinNames(const std::vector<std::string>& names);

}  // namespace manipulatorcore_ros
