#pragma once

/// @file conversions.hpp
/// @brief Conversions between ManipulatorCore types and ROS 2 messages.
///
/// All functions are free of rclcpp; they only depend on message types. Invalid input throws
/// std::invalid_argument with a descriptive message.

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/geometry/shapes.hpp"
#include "manipulatorcore/math/types.hpp"
#include "manipulatorcore/trajectory/joint_trajectory.hpp"
#include "manipulatorcore/trajectory/spline.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>

namespace manipulatorcore_ros {

// ---------------------------------------------------------------------------------------------
// Rigid-body transforms
// ---------------------------------------------------------------------------------------------

[[nodiscard]] geometry_msgs::msg::Point toPointMsg(const mc::Vec3& point);
[[nodiscard]] mc::Vec3 fromPointMsg(const geometry_msgs::msg::Point& msg);

/// Pose message of T (position = translation, orientation = canonical unit quaternion, w >= 0).
[[nodiscard]] geometry_msgs::msg::Pose toPoseMsg(const mc::SE3& T);

/// SE3 from a pose message. The quaternion is normalized; a zero or non-finite quaternion or a
/// non-finite position throws std::invalid_argument.
[[nodiscard]] mc::SE3 fromPoseMsg(const geometry_msgs::msg::Pose& msg);

[[nodiscard]] geometry_msgs::msg::Transform toTransformMsg(const mc::SE3& T);

/// SE3 from a transform message (same validation as fromPoseMsg()).
[[nodiscard]] mc::SE3 fromTransformMsg(const geometry_msgs::msg::Transform& msg);

// ---------------------------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------------------------

/// Non-negative duration in seconds, rounded to the nearest nanosecond.
[[nodiscard]] builtin_interfaces::msg::Duration toDurationMsg(double seconds);
[[nodiscard]] double toSeconds(const builtin_interfaces::msg::Duration& duration);

// ---------------------------------------------------------------------------------------------
// Joint states and trajectories
// ---------------------------------------------------------------------------------------------

/// JointState with names and positions, plus velocities and efforts when given (empty vectors are
/// left out). The header is left default-initialized; the caller sets the stamp.
[[nodiscard]] sensor_msgs::msg::JointState makeJointState(const std::vector<std::string>& joint_names,
                                                          const mc::VecX& positions,
                                                          const mc::VecX& velocities = mc::VecX(),
                                                          const mc::VecX& efforts = mc::VecX());

/// Positions of @p joint_names (in that order) taken from a JointState whose joints may be listed in
/// any order. Throws if a joint is missing or a position is not finite.
[[nodiscard]] mc::VecX jointPositions(const sensor_msgs::msg::JointState& msg,
                                      const std::vector<std::string>& joint_names);

/// Samples @p trajectory every @p dt seconds (the final point is always included) into a
/// trajectory message with positions, velocities and accelerations. A final sample closer than
/// 1 microsecond to the previous one replaces it, so time_from_start is strictly increasing.
[[nodiscard]] trajectory_msgs::msg::JointTrajectory toJointTrajectoryMsg(const mc::JointTrajectory& trajectory,
                                                                         const std::vector<std::string>& joint_names,
                                                                         double dt);

/// C2 cubic spline through the positions of a trajectory message, re-ordered into
/// @p joint_names order (the message may list the joints in any order, extra joints are ignored).
///
/// The spline uses the velocities of the first and last point as boundary conditions when they
/// are present (zero otherwise). If the first point has time_from_start > 0 or the message has a
/// single point, @p start is inserted at t = 0 (an empty @p start then throws). Points closer than
/// 1 microsecond to their predecessor are dropped. Throws on missing joints, inconsistent sizes,
/// non-finite values or decreasing times.
[[nodiscard]] mc::CubicSplineTrajectory fromJointTrajectoryMsg(const trajectory_msgs::msg::JointTrajectory& msg,
                                                               const std::vector<std::string>& joint_names,
                                                               const mc::VecX& start = mc::VecX());

// ---------------------------------------------------------------------------------------------
// Visualization
// ---------------------------------------------------------------------------------------------

[[nodiscard]] std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0F);

struct MarkerStyle {
  std::string frame_id = "base_link";
  std::string ns = "manipulatorcore";
  int id = 0;  ///< id of the first marker; further markers use consecutive ids
  std_msgs::msg::ColorRGBA color = makeColor(0.55F, 0.6F, 0.7F, 0.8F);
  /// A half-space is drawn as a thin square slab whose top face is the boundary plane.
  double half_space_extent = 2.0;      ///< edge length of the slab [m]
  double half_space_thickness = 0.01;  ///< [m]
};

/// RViz markers of a shape whose pose is expressed in style.frame_id:
///   Sphere    -> SPHERE
///   Box       -> CUBE
///   Capsule   -> CYLINDER (the core segment) + two SPHERE caps; RViz has no capsule primitive
///   HalfSpace -> thin, large CUBE below the boundary plane (on the solid side)
/// Markers get consecutive ids starting at style.id.
[[nodiscard]] std::vector<visualization_msgs::msg::Marker> toMarkers(const mc::Shape& shape, const MarkerStyle& style);

/// LINE_STRIP marker through @p points with line @p width [m] (style.id is the marker id).
[[nodiscard]] visualization_msgs::msg::Marker makeLineStrip(const std::vector<mc::Vec3>& points,
                                                            const MarkerStyle& style, double width);

}  // namespace manipulatorcore_ros
