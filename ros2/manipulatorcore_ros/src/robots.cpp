#include "manipulatorcore_ros/robots.hpp"

#include "manipulatorcore/model/builtin_robots.hpp"

#include <stdexcept>
#include <utility>

namespace manipulatorcore_ros {

namespace {

constexpr const char* kRedundant7R = "redundant7r";

}  // namespace

const std::vector<std::string>& builtinRobotNames() {
  static const std::vector<std::string> names = {"redundant7r", "industrial6r", "planar2r", "planar3r"};
  return names;
}

mc::RobotModel makeRobot(const std::string& name) {
  if (name == "redundant7r") {
    return mc::makeRedundant7R();
  }
  if (name == "industrial6r") {
    return mc::makeIndustrial6R();
  }
  if (name == "planar2r") {
    return mc::makePlanar2R();
  }
  if (name == "planar3r") {
    return mc::makePlanar3R();
  }
  throw std::invalid_argument("unknown robot '" + name + "' (expected one of: " + joinNames(builtinRobotNames()) + ")");
}

mc::VecX homeConfiguration(const std::string& name) {
  if (name == "redundant7r") {
    return mc::redundant7RHome();
  }
  if (name == "industrial6r") {
    return mc::industrial6RHome();
  }
  if (name == "planar2r") {
    mc::VecX q(2);
    q << 0.5, 0.8;
    return q;
  }
  if (name == "planar3r") {
    mc::VecX q(3);
    q << 0.5, 0.8, 0.4;
    return q;
  }
  throw std::invalid_argument("unknown robot '" + name + "' (expected one of: " + joinNames(builtinRobotNames()) + ")");
}

std::vector<std::string> jointNames(const mc::RobotModel& model) {
  std::vector<std::string> names;
  names.reserve(model.bodies().size());
  for (const mc::Body& body : model.bodies()) {
    names.push_back(body.joint.name);
  }
  return names;
}

std::vector<std::string> scenarioNames(const std::string& robot) {
  std::vector<std::string> names;
  if (robot == kRedundant7R) {
    for (const mc::PlanningScenario& scenario : mc::redundant7RScenarios()) {
      names.push_back(scenario.name);
    }
  }
  return names;
}

std::optional<mc::PlanningScenario> findScenario(const std::string& robot, const std::string& name) {
  if (robot != kRedundant7R) {
    return std::nullopt;
  }
  for (mc::PlanningScenario& scenario : mc::redundant7RScenarios()) {
    if (scenario.name == name) {
      return std::move(scenario);
    }
  }
  return std::nullopt;
}

std::string joinNames(const std::vector<std::string>& names) {
  std::string out;
  for (const std::string& name : names) {
    if (!out.empty()) {
      out += ", ";
    }
    out += name;
  }
  return out;
}

}  // namespace manipulatorcore_ros
