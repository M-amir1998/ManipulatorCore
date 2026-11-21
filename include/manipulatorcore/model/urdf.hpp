#pragma once

/// @file urdf.hpp
/// @brief Lightweight URDF export and import for serial chains (no external XML dependency).
///
/// Export writes a self-contained URDF usable by ROS tooling (robot_state_publisher, RViz).
/// Import supports the subset of URDF needed for serial manipulators: revolute, continuous,
/// prismatic and fixed joints; inertial blocks; sphere, box, cylinder and capsule collision
/// geometry. Meshes are ignored. Cylinders are imported as capsules of the same radius and
/// length, which slightly over-approximates the volume (conservative for collision checking).
/// Fixed joints are merged into the neighbouring bodies.

#include "manipulatorcore/model/robot_model.hpp"

#include <string>
#include <string_view>

namespace mc {

struct UrdfExportOptions {
  std::string base_link = "base_link";
  std::string tool_link = "tool0";
  /// Emit <visual> elements mirroring the collision primitives (capsules become a cylinder plus
  /// two spheres, because URDF has no portable capsule primitive).
  bool include_visuals = true;
};

struct UrdfImportOptions {
  std::string base_link{};  ///< empty: the root link of the tree
  std::string tip_link{};   ///< empty: follow the chain until it ends (must not branch)
  /// Position range used for continuous joints [rad].
  double continuous_joint_range = kTwoPi;
  /// Defaults for limits missing from the URDF.
  double default_velocity_limit = 2.0;
  double default_acceleration_limit = 10.0;
  double default_effort_limit = 100.0;
  Vec3 gravity = Vec3(0.0, 0.0, -kStandardGravity);
};

/// Serializes a model as URDF. Body links are named after the bodies, joints after the joints.
[[nodiscard]] std::string toURDF(const RobotModel& model, const UrdfExportOptions& options = {});

/// Parses a URDF document into a serial RobotModel. Throws InvalidArgument on malformed XML,
/// unsupported joint types, or a tree that does not reduce to a serial chain between the base and
/// tip links.
[[nodiscard]] RobotModel fromURDF(std::string_view urdf_xml, const UrdfImportOptions& options = {});

/// Reads and parses a URDF file.
[[nodiscard]] RobotModel loadURDF(const std::string& path, const UrdfImportOptions& options = {});

}  // namespace mc
