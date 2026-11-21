#include "manipulatorcore_ros/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace manipulatorcore_ros {

namespace {

/// Two trajectory samples closer than this are treated as the same instant [s].
constexpr double kMinTimeSpacing = 1e-6;

void check(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

mc::SE3 fromPositionAndQuaternion(double px, double py, double pz, double qx, double qy, double qz, double qw,
                                  const char* what) {
  const mc::Vec3 p(px, py, pz);
  const mc::Quat q(qw, qx, qy, qz);
  check(p.allFinite(), std::string(what) + ": position is not finite");
  check(q.coeffs().allFinite(), std::string(what) + ": orientation is not finite");
  check(q.norm() > 1e-9, std::string(what) + ": orientation quaternion has zero norm");
  return mc::SE3::fromQuaternion(q.normalized(), p);
}

std::vector<double> toStdVector(const mc::VecX& v) { return {v.data(), v.data() + v.size()}; }

/// Index of every name of @p wanted inside @p available.
std::vector<std::size_t> nameIndices(const std::vector<std::string>& available, const std::vector<std::string>& wanted,
                                     const char* what) {
  std::vector<std::size_t> indices;
  indices.reserve(wanted.size());
  for (const std::string& name : wanted) {
    const auto it = std::find(available.begin(), available.end(), name);
    check(it != available.end(), std::string(what) + ": joint '" + name + "' is missing");
    indices.push_back(static_cast<std::size_t>(it - available.begin()));
  }
  return indices;
}

visualization_msgs::msg::Marker baseMarker(const MarkerStyle& style, int id, int32_t type) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = style.frame_id;
  marker.ns = style.ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color = style.color;
  return marker;
}

void setScale(visualization_msgs::msg::Marker& marker, double x, double y, double z) {
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Rigid-body transforms
// ---------------------------------------------------------------------------------------------

geometry_msgs::msg::Point toPointMsg(const mc::Vec3& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

mc::Vec3 fromPointMsg(const geometry_msgs::msg::Point& msg) { return {msg.x, msg.y, msg.z}; }

geometry_msgs::msg::Pose toPoseMsg(const mc::SE3& T) {
  geometry_msgs::msg::Pose msg;
  msg.position = toPointMsg(T.translation());
  const mc::Quat q = T.quaternion();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

mc::SE3 fromPoseMsg(const geometry_msgs::msg::Pose& msg) {
  return fromPositionAndQuaternion(msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y,
                                   msg.orientation.z, msg.orientation.w, "Pose");
}

geometry_msgs::msg::Transform toTransformMsg(const mc::SE3& T) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = T.translation().x();
  msg.translation.y = T.translation().y();
  msg.translation.z = T.translation().z();
  const mc::Quat q = T.quaternion();
  msg.rotation.x = q.x();
  msg.rotation.y = q.y();
  msg.rotation.z = q.z();
  msg.rotation.w = q.w();
  return msg;
}

mc::SE3 fromTransformMsg(const geometry_msgs::msg::Transform& msg) {
  return fromPositionAndQuaternion(msg.translation.x, msg.translation.y, msg.translation.z, msg.rotation.x,
                                   msg.rotation.y, msg.rotation.z, msg.rotation.w, "Transform");
}

// ---------------------------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------------------------

builtin_interfaces::msg::Duration toDurationMsg(double seconds) {
  check(std::isfinite(seconds) && seconds >= 0.0, "toDurationMsg: duration must be finite and non-negative");
  const auto total_ns = static_cast<std::int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<std::int32_t>(total_ns / 1000000000);
  msg.nanosec = static_cast<std::uint32_t>(total_ns % 1000000000);
  return msg;
}

double toSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

// ---------------------------------------------------------------------------------------------
// Joint states and trajectories
// ---------------------------------------------------------------------------------------------

sensor_msgs::msg::JointState makeJointState(const std::vector<std::string>& joint_names, const mc::VecX& positions,
                                            const mc::VecX& velocities, const mc::VecX& efforts) {
  const auto n = static_cast<mc::Index>(joint_names.size());
  check(positions.size() == n, "makeJointState: positions do not match the joint names");
  check(velocities.size() == 0 || velocities.size() == n, "makeJointState: velocities do not match the joint names");
  check(efforts.size() == 0 || efforts.size() == n, "makeJointState: efforts do not match the joint names");
  sensor_msgs::msg::JointState msg;
  msg.name = joint_names;
  msg.position = toStdVector(positions);
  msg.velocity = toStdVector(velocities);
  msg.effort = toStdVector(efforts);
  return msg;
}

mc::VecX jointPositions(const sensor_msgs::msg::JointState& msg, const std::vector<std::string>& joint_names) {
  const std::vector<std::size_t> indices = nameIndices(msg.name, joint_names, "JointState");
  mc::VecX q(static_cast<mc::Index>(joint_names.size()));
  for (std::size_t i = 0; i < indices.size(); ++i) {
    check(indices[i] < msg.position.size(), "JointState: no position for joint '" + joint_names[i] + "'");
    q(static_cast<mc::Index>(i)) = msg.position[indices[i]];
  }
  check(q.allFinite(), "JointState: positions are not finite");
  return q;
}

trajectory_msgs::msg::JointTrajectory toJointTrajectoryMsg(const mc::JointTrajectory& trajectory,
                                                           const std::vector<std::string>& joint_names, double dt) {
  check(std::isfinite(dt) && dt > 0.0, "toJointTrajectoryMsg: dt must be positive");
  check(static_cast<int>(joint_names.size()) == trajectory.dof(),
        "toJointTrajectoryMsg: " + std::to_string(joint_names.size()) + " joint names for a trajectory with " +
            std::to_string(trajectory.dof()) + " joints");
  const double duration = trajectory.duration();
  check(std::isfinite(duration) && duration >= 0.0, "toJointTrajectoryMsg: invalid trajectory duration");

  std::vector<double> times;
  for (std::size_t k = 0;; ++k) {
    const double t = static_cast<double>(k) * dt;
    if (t >= duration - kMinTimeSpacing) {
      break;
    }
    times.push_back(t);
  }
  times.push_back(duration);

  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = joint_names;
  msg.points.reserve(times.size());
  for (const double t : times) {
    const mc::TrajectoryPoint p = trajectory.sample(t);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = toStdVector(p.q);
    point.velocities = toStdVector(p.qd);
    point.accelerations = toStdVector(p.qdd);
    point.time_from_start = toDurationMsg(t);
    msg.points.push_back(std::move(point));
  }
  return msg;
}

mc::CubicSplineTrajectory fromJointTrajectoryMsg(const trajectory_msgs::msg::JointTrajectory& msg,
                                                 const std::vector<std::string>& joint_names, const mc::VecX& start) {
  check(!joint_names.empty(), "JointTrajectory: no joint names requested");
  check(!msg.points.empty(), "JointTrajectory: message has no points");
  const auto n = static_cast<mc::Index>(joint_names.size());
  const std::size_t m = msg.joint_names.size();
  const std::vector<std::size_t> indices = nameIndices(msg.joint_names, joint_names, "JointTrajectory");

  const auto reorder = [&](const std::vector<double>& values, const char* what) {
    check(values.size() == m, std::string("JointTrajectory: ") + what + " have " + std::to_string(values.size()) +
                                  " entries for " + std::to_string(m) + " joints");
    mc::VecX out(n);
    for (mc::Index i = 0; i < n; ++i) {
      out(i) = values[indices[static_cast<std::size_t>(i)]];
    }
    check(out.allFinite(), std::string("JointTrajectory: ") + what + " are not finite");
    return out;
  };

  std::vector<double> times;
  std::vector<mc::VecX> waypoints;
  mc::VecX start_velocity = mc::VecX::Zero(n);
  mc::VecX end_velocity = mc::VecX::Zero(n);

  const double first_time = toSeconds(msg.points.front().time_from_start);
  const bool insert_start = first_time > kMinTimeSpacing || msg.points.size() == 1;
  if (insert_start) {
    check(start.size() == n,
          "JointTrajectory: the first point is not at t = 0 and no start configuration of matching size is given");
    check(start.allFinite(), "JointTrajectory: start configuration is not finite");
    times.push_back(0.0);
    waypoints.push_back(start);
  }

  for (std::size_t k = 0; k < msg.points.size(); ++k) {
    const trajectory_msgs::msg::JointTrajectoryPoint& point = msg.points[k];
    double t = toSeconds(point.time_from_start);
    check(t >= 0.0, "JointTrajectory: negative time_from_start");
    mc::VecX q = reorder(point.positions, "positions");
    if (k == 0 && !insert_start) {
      t = 0.0;
      if (!point.velocities.empty()) {
        start_velocity = reorder(point.velocities, "velocities");
      }
    }
    if (k + 1 == msg.points.size() && !point.velocities.empty()) {
      end_velocity = reorder(point.velocities, "velocities");
    }
    if (!times.empty()) {
      check(t >= times.back(), "JointTrajectory: time_from_start must be non-decreasing");
      if (t - times.back() < kMinTimeSpacing) {
        waypoints.back() = std::move(q);  // same instant: the later point wins, the earlier time is kept
        continue;
      }
    }
    times.push_back(t);
    waypoints.push_back(std::move(q));
  }
  check(times.size() >= 2, "JointTrajectory: the trajectory has zero duration");
  return {std::move(times), std::move(waypoints), start_velocity, end_velocity};
}

// ---------------------------------------------------------------------------------------------
// Visualization
// ---------------------------------------------------------------------------------------------

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std::vector<visualization_msgs::msg::Marker> toMarkers(const mc::Shape& shape, const MarkerStyle& style) {
  using visualization_msgs::msg::Marker;
  std::vector<Marker> markers;
  int id = style.id;

  if (const auto* sphere = std::get_if<mc::Sphere>(&shape.geometry)) {
    Marker marker = baseMarker(style, id, Marker::SPHERE);
    marker.pose = toPoseMsg(shape.pose);
    const double d = 2.0 * sphere->radius;
    setScale(marker, d, d, d);
    markers.push_back(std::move(marker));
  } else if (const auto* box = std::get_if<mc::Box>(&shape.geometry)) {
    Marker marker = baseMarker(style, id, Marker::CUBE);
    marker.pose = toPoseMsg(shape.pose);
    setScale(marker, 2.0 * box->half_extents.x(), 2.0 * box->half_extents.y(), 2.0 * box->half_extents.z());
    markers.push_back(std::move(marker));
  } else if (const auto* capsule = std::get_if<mc::Capsule>(&shape.geometry)) {
    const double d = 2.0 * capsule->radius;
    if (capsule->half_length > 0.0) {
      Marker cylinder = baseMarker(style, id++, Marker::CYLINDER);
      cylinder.pose = toPoseMsg(shape.pose);
      setScale(cylinder, d, d, 2.0 * capsule->half_length);
      markers.push_back(std::move(cylinder));
    }
    for (const double sign : {-1.0, 1.0}) {
      Marker cap = baseMarker(style, id++, Marker::SPHERE);
      cap.pose = toPoseMsg(shape.pose * mc::SE3::fromTranslation(mc::Vec3(0.0, 0.0, sign * capsule->half_length)));
      setScale(cap, d, d, d);
      markers.push_back(std::move(cap));
      if (capsule->half_length <= 0.0) {
        break;
      }
    }
  } else if (const auto* half_space = std::get_if<mc::HalfSpace>(&shape.geometry)) {
    check(half_space->normal.norm() > 1e-12, "toMarkers: half-space normal has zero norm");
    // Slab frame: z along the normal, x as close as possible to the shape's x (or y) axis.
    const mc::Vec3 n = half_space->normal.normalized();
    const mc::Vec3 helper = std::abs(n.x()) < 0.9 ? mc::Vec3::UnitX() : mc::Vec3::UnitY();
    const mc::Vec3 x = (helper - n * n.dot(helper)).normalized();
    mc::Mat3 R_shape_slab;
    R_shape_slab.col(0) = x;
    R_shape_slab.col(1) = n.cross(x);
    R_shape_slab.col(2) = n;
    const mc::Vec3 center = n * (half_space->offset - 0.5 * style.half_space_thickness);
    Marker marker = baseMarker(style, id, Marker::CUBE);
    marker.pose = toPoseMsg(shape.pose * mc::SE3(R_shape_slab, center));
    setScale(marker, style.half_space_extent, style.half_space_extent, style.half_space_thickness);
    markers.push_back(std::move(marker));
  }
  return markers;
}

visualization_msgs::msg::Marker makeLineStrip(const std::vector<mc::Vec3>& points, const MarkerStyle& style,
                                              double width) {
  check(std::isfinite(width) && width > 0.0, "makeLineStrip: width must be positive");
  visualization_msgs::msg::Marker marker = baseMarker(style, style.id, visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = width;
  marker.points.reserve(points.size());
  for (const mc::Vec3& p : points) {
    marker.points.push_back(toPointMsg(p));
  }
  return marker;
}

}  // namespace manipulatorcore_ros
