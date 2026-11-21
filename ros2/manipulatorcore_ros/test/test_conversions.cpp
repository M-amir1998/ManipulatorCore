#include "manipulatorcore/model/urdf.hpp"
#include "manipulatorcore_ros/conversions.hpp"
#include "manipulatorcore_ros/robots.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace manipulatorcore_ros {
namespace {

using visualization_msgs::msg::Marker;

mc::VecX vec(std::initializer_list<double> values) {
  mc::VecX v(static_cast<mc::Index>(values.size()));
  mc::Index i = 0;
  for (const double x : values) {
    v(i++) = x;
  }
  return v;
}

mc::SE3 samplePose(int k) {
  const mc::Vec3 axis = mc::Vec3(0.3 + k, -0.7 + 0.5 * k, 0.2 - 0.3 * k).normalized();
  const double angle = -3.0 + 0.9 * k;
  return {Eigen::AngleAxisd(angle, axis).toRotationMatrix(), mc::Vec3(0.1 * k, -0.4 + 0.2 * k, 0.75 - 0.1 * k)};
}

mc::CubicSplineTrajectory sampleTrajectory() {
  return {{0.0, 0.7, 1.5, 2.25},
          {vec({0.0, 0.5, -0.2}), vec({0.4, 0.1, 0.3}), vec({0.9, -0.3, 0.1}), vec({1.0, -0.2, -0.4})},
          vec({0.0, 0.0, 0.0}),
          vec({0.0, 0.0, 0.0})};
}

const std::vector<std::string> kNames = {"joint1", "joint2", "joint3"};

// ---------------------------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------------------------

TEST(PoseConversion, RoundTripPreservesTransform) {
  for (int k = 0; k < 8; ++k) {
    const mc::SE3 T = samplePose(k);
    const geometry_msgs::msg::Pose msg = toPoseMsg(T);
    EXPECT_GE(msg.orientation.w, 0.0);
    const double norm = std::sqrt(msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y +
                                  msg.orientation.z * msg.orientation.z + msg.orientation.w * msg.orientation.w);
    EXPECT_NEAR(norm, 1.0, 1e-12);
    EXPECT_TRUE(fromPoseMsg(msg).isApprox(T, 1e-12)) << "pose " << k;
  }
}

TEST(PoseConversion, MatchesHamiltonQuaternionConvention) {
  // 90 degrees about z: q = (w, x, y, z) = (cos 45, 0, 0, sin 45).
  const mc::SE3 T(Eigen::AngleAxisd(mc::kHalfPi, mc::Vec3::UnitZ()).toRotationMatrix(), mc::Vec3(1.0, 2.0, 3.0));
  const geometry_msgs::msg::Pose msg = toPoseMsg(T);
  EXPECT_NEAR(msg.orientation.w, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(msg.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(msg.orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(msg.orientation.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(msg.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.position.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.position.z, 3.0);
}

TEST(PoseConversion, NormalizesQuaternionAndRejectsInvalidInput) {
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = 2.0;  // 180 degrees about x, not normalized
  msg.orientation.w = 0.0;
  const mc::SE3 T = fromPoseMsg(msg);
  EXPECT_TRUE(T.rotation().isApprox(mc::Vec3(1.0, -1.0, -1.0).asDiagonal().toDenseMatrix(), 1e-12));

  geometry_msgs::msg::Pose zero;
  zero.orientation.w = 0.0;
  EXPECT_THROW((void)fromPoseMsg(zero), std::invalid_argument);

  geometry_msgs::msg::Pose nan;
  nan.position.x = std::nan("");
  EXPECT_THROW((void)fromPoseMsg(nan), std::invalid_argument);
}

TEST(TransformConversion, RoundTripPreservesTransform) {
  for (int k = 0; k < 8; ++k) {
    const mc::SE3 T = samplePose(k);
    const geometry_msgs::msg::Transform msg = toTransformMsg(T);
    EXPECT_DOUBLE_EQ(msg.translation.x, T.translation().x());
    EXPECT_TRUE(fromTransformMsg(msg).isApprox(T, 1e-12)) << "transform " << k;
  }
}

TEST(DurationConversion, SplitsSecondsAndNanoseconds) {
  const builtin_interfaces::msg::Duration d = toDurationMsg(2.25);
  EXPECT_EQ(d.sec, 2);
  EXPECT_EQ(d.nanosec, 250000000U);
  EXPECT_DOUBLE_EQ(toSeconds(d), 2.25);
  EXPECT_EQ(toDurationMsg(0.9999999999).sec, 1);
  EXPECT_EQ(toDurationMsg(0.9999999999).nanosec, 0U);
  EXPECT_THROW((void)toDurationMsg(-0.1), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------------
// Joint states and trajectories
// ---------------------------------------------------------------------------------------------

TEST(JointStateConversion, BuildsMessageAndReordersPositions) {
  const sensor_msgs::msg::JointState msg = makeJointState(kNames, vec({0.1, 0.2, 0.3}), vec({1.0, 2.0, 3.0}));
  EXPECT_EQ(msg.name, kNames);
  EXPECT_EQ(msg.position, (std::vector<double>{0.1, 0.2, 0.3}));
  EXPECT_EQ(msg.velocity, (std::vector<double>{1.0, 2.0, 3.0}));
  EXPECT_TRUE(msg.effort.empty());
  EXPECT_THROW((void)makeJointState(kNames, vec({0.1, 0.2})), std::invalid_argument);

  sensor_msgs::msg::JointState shuffled;
  shuffled.name = {"joint3", "gripper", "joint1", "joint2"};
  shuffled.position = {0.3, 9.0, 0.1, 0.2};
  EXPECT_TRUE(jointPositions(shuffled, kNames).isApprox(vec({0.1, 0.2, 0.3})));

  shuffled.name = {"joint3", "joint1"};
  shuffled.position = {0.3, 0.1};
  EXPECT_THROW((void)jointPositions(shuffled, kNames), std::invalid_argument);
}

TEST(TrajectoryConversion, SamplesUniformlyWithJointNames) {
  const mc::CubicSplineTrajectory trajectory = sampleTrajectory();
  const double dt = 0.1;
  const trajectory_msgs::msg::JointTrajectory msg = toJointTrajectoryMsg(trajectory, kNames, dt);

  EXPECT_EQ(msg.joint_names, kNames);
  ASSERT_EQ(msg.points.size(), 24U);  // 0.0, 0.1, ..., 2.2 and the end point 2.25
  EXPECT_DOUBLE_EQ(toSeconds(msg.points.front().time_from_start), 0.0);
  EXPECT_NEAR(toSeconds(msg.points.back().time_from_start), trajectory.duration(), 1e-9);
  double previous = -1.0;
  for (std::size_t k = 0; k < msg.points.size(); ++k) {
    const auto& point = msg.points[k];
    const double t = toSeconds(point.time_from_start);
    EXPECT_GT(t, previous);
    if (k + 1 < msg.points.size()) {
      EXPECT_NEAR(t, static_cast<double>(k) * dt, 1e-9);
    }
    previous = t;
    ASSERT_EQ(point.positions.size(), 3U);
    ASSERT_EQ(point.velocities.size(), 3U);
    ASSERT_EQ(point.accelerations.size(), 3U);
    const mc::TrajectoryPoint expected = trajectory.sample(t);
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(point.positions[j], expected.q(static_cast<mc::Index>(j)), 1e-9);
      EXPECT_NEAR(point.velocities[j], expected.qd(static_cast<mc::Index>(j)), 1e-6);
    }
  }
}

TEST(TrajectoryConversion, KeepsTimesStrictlyIncreasingNearTheEnd) {
  // duration = 2.25 is a multiple of dt up to round-off; no duplicate final point may appear.
  const trajectory_msgs::msg::JointTrajectory msg = toJointTrajectoryMsg(sampleTrajectory(), kNames, 0.0225);
  for (std::size_t k = 1; k < msg.points.size(); ++k) {
    EXPECT_GT(toSeconds(msg.points[k].time_from_start), toSeconds(msg.points[k - 1].time_from_start) + 1e-7);
  }
  EXPECT_NEAR(toSeconds(msg.points.back().time_from_start), 2.25, 1e-9);
}

TEST(TrajectoryConversion, RejectsMismatchedJointNames) {
  EXPECT_THROW((void)toJointTrajectoryMsg(sampleTrajectory(), {"joint1", "joint2"}, 0.01), std::invalid_argument);
  EXPECT_THROW((void)toJointTrajectoryMsg(sampleTrajectory(), kNames, 0.0), std::invalid_argument);
}

TEST(TrajectoryConversion, RoundTripReproducesTrajectory) {
  const mc::CubicSplineTrajectory original = sampleTrajectory();
  const trajectory_msgs::msg::JointTrajectory msg = toJointTrajectoryMsg(original, kNames, 0.01);
  const mc::CubicSplineTrajectory restored = fromJointTrajectoryMsg(msg, kNames);
  EXPECT_NEAR(restored.duration(), original.duration(), 1e-9);
  for (double t = 0.0; t <= original.duration(); t += 0.0137) {
    EXPECT_TRUE(restored.sample(t).q.isApprox(original.sample(t).q, 1e-6)) << "t = " << t;
    EXPECT_LT((restored.sample(t).qd - original.sample(t).qd).cwiseAbs().maxCoeff(), 1e-4) << "t = " << t;
  }
}

TEST(TrajectoryConversion, ReordersJointsAndIgnoresExtraJoints) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = {"joint3", "gripper", "joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint a;
  a.positions = {0.3, 5.0, 0.1, 0.2};
  a.time_from_start = toDurationMsg(0.0);
  trajectory_msgs::msg::JointTrajectoryPoint b;
  b.positions = {0.6, 5.0, 0.4, 0.5};
  b.time_from_start = toDurationMsg(1.0);
  msg.points = {a, b};

  const mc::CubicSplineTrajectory trajectory = fromJointTrajectoryMsg(msg, kNames);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 1.0);
  EXPECT_TRUE(trajectory.sample(0.0).q.isApprox(vec({0.1, 0.2, 0.3})));
  EXPECT_TRUE(trajectory.sample(1.0).q.isApprox(vec({0.4, 0.5, 0.6})));
  EXPECT_LT(trajectory.sample(1.0).qd.norm(), 1e-12);  // no velocities given: rest at both ends

  msg.joint_names = {"joint3", "gripper", "joint1", "joint4"};
  EXPECT_THROW((void)fromJointTrajectoryMsg(msg, kNames), std::invalid_argument);
}

TEST(TrajectoryConversion, InsertsStartWhenFirstPointIsInTheFuture) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = kNames;
  trajectory_msgs::msg::JointTrajectoryPoint target;
  target.positions = {1.0, 1.0, 1.0};
  target.time_from_start = toDurationMsg(2.0);
  msg.points = {target};

  EXPECT_THROW((void)fromJointTrajectoryMsg(msg, kNames), std::invalid_argument);
  const mc::CubicSplineTrajectory trajectory = fromJointTrajectoryMsg(msg, kNames, vec({0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(trajectory.duration(), 2.0);
  EXPECT_TRUE(trajectory.sample(0.0).q.isZero());
  EXPECT_TRUE(trajectory.sample(2.0).q.isApprox(vec({1.0, 1.0, 1.0})));
}

TEST(TrajectoryConversion, RejectsInvalidMessages) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = kNames;
  EXPECT_THROW((void)fromJointTrajectoryMsg(msg, kNames), std::invalid_argument);  // no points

  trajectory_msgs::msg::JointTrajectoryPoint a;
  a.positions = {0.0, 0.0, 0.0};
  trajectory_msgs::msg::JointTrajectoryPoint b;
  b.positions = {0.1, 0.1};  // wrong size
  b.time_from_start = toDurationMsg(1.0);
  msg.points = {a, b};
  EXPECT_THROW((void)fromJointTrajectoryMsg(msg, kNames), std::invalid_argument);

  b.positions = {0.1, 0.1, 0.1};
  a.time_from_start = toDurationMsg(0.0);
  trajectory_msgs::msg::JointTrajectoryPoint c = b;
  c.time_from_start = toDurationMsg(0.5);  // goes back in time
  msg.points = {a, b, c};
  EXPECT_THROW((void)fromJointTrajectoryMsg(msg, kNames), std::invalid_argument);
}

// ---------------------------------------------------------------------------------------------
// Markers
// ---------------------------------------------------------------------------------------------

TEST(MarkerConversion, SphereAndBox) {
  MarkerStyle style;
  style.frame_id = "world";
  style.ns = "test";
  style.id = 7;
  const auto sphere = toMarkers(mc::makeSphere(mc::Vec3(0.1, 0.2, 0.3), 0.25), style);
  ASSERT_EQ(sphere.size(), 1U);
  EXPECT_EQ(sphere[0].type, Marker::SPHERE);
  EXPECT_EQ(sphere[0].id, 7);
  EXPECT_EQ(sphere[0].ns, "test");
  EXPECT_EQ(sphere[0].header.frame_id, "world");
  EXPECT_DOUBLE_EQ(sphere[0].scale.x, 0.5);
  EXPECT_DOUBLE_EQ(sphere[0].pose.position.z, 0.3);
  EXPECT_GT(sphere[0].color.a, 0.0F);

  const mc::SE3 pose = samplePose(3);
  const auto box = toMarkers(mc::makeBox(pose, mc::Vec3(0.1, 0.2, 0.3)), style);
  ASSERT_EQ(box.size(), 1U);
  EXPECT_EQ(box[0].type, Marker::CUBE);
  EXPECT_DOUBLE_EQ(box[0].scale.x, 0.2);
  EXPECT_DOUBLE_EQ(box[0].scale.y, 0.4);
  EXPECT_DOUBLE_EQ(box[0].scale.z, 0.6);
  EXPECT_TRUE(fromPoseMsg(box[0].pose).isApprox(pose, 1e-12));
}

TEST(MarkerConversion, CapsuleIsCylinderWithSphericalCaps) {
  MarkerStyle style;
  style.id = 3;
  const mc::Vec3 a(0.0, 0.0, 0.1);
  const mc::Vec3 b(0.4, 0.0, 0.1);
  const auto markers = toMarkers(mc::makeCapsule(a, b, 0.05), style);
  ASSERT_EQ(markers.size(), 3U);
  EXPECT_EQ(markers[0].type, Marker::CYLINDER);
  EXPECT_EQ(markers[1].type, Marker::SPHERE);
  EXPECT_EQ(markers[2].type, Marker::SPHERE);
  EXPECT_EQ(markers[0].id, 3);
  EXPECT_EQ(markers[1].id, 4);
  EXPECT_EQ(markers[2].id, 5);
  EXPECT_NEAR(markers[0].scale.x, 0.1, 1e-12);
  EXPECT_NEAR(markers[0].scale.z, 0.4, 1e-12);
  // Cylinder axis (local z) along the segment, centred between the end points.
  const mc::SE3 cylinder = fromPoseMsg(markers[0].pose);
  EXPECT_TRUE(cylinder.translation().isApprox(0.5 * (a + b), 1e-12));
  EXPECT_NEAR(std::abs(cylinder.rotation().col(2).dot((b - a).normalized())), 1.0, 1e-12);
  // Caps at the segment ends.
  const mc::Vec3 cap1 = fromPointMsg(markers[1].pose.position);
  const mc::Vec3 cap2 = fromPointMsg(markers[2].pose.position);
  EXPECT_NEAR(std::min((cap1 - a).norm(), (cap1 - b).norm()), 0.0, 1e-12);
  EXPECT_NEAR(std::min((cap2 - a).norm(), (cap2 - b).norm()), 0.0, 1e-12);
  EXPECT_GT((cap1 - cap2).norm(), 0.39);
}

TEST(MarkerConversion, HalfSpaceIsSlabBelowBoundaryPlane) {
  MarkerStyle style;
  style.half_space_extent = 3.0;
  style.half_space_thickness = 0.02;
  const auto floor = toMarkers(mc::makeHalfSpace(mc::Vec3::UnitZ(), 0.0), style);
  ASSERT_EQ(floor.size(), 1U);
  EXPECT_EQ(floor[0].type, Marker::CUBE);
  EXPECT_DOUBLE_EQ(floor[0].scale.x, 3.0);
  EXPECT_DOUBLE_EQ(floor[0].scale.z, 0.02);
  EXPECT_NEAR(floor[0].pose.position.z, -0.01, 1e-12);

  // Tilted half-space: the slab's top face lies on the plane and its normal matches.
  const mc::Vec3 n = mc::Vec3(1.0, -1.0, 2.0).normalized();
  const auto tilted = toMarkers(mc::makeHalfSpace(n, 0.4), style);
  ASSERT_EQ(tilted.size(), 1U);
  const mc::SE3 slab = fromPoseMsg(tilted[0].pose);
  const mc::Vec3 top_center = slab * mc::Vec3(0.0, 0.0, 0.01);
  EXPECT_NEAR(n.dot(top_center), 0.4, 1e-12);
  EXPECT_TRUE(slab.rotation().col(2).isApprox(n, 1e-12));
}

TEST(MarkerConversion, LineStrip) {
  MarkerStyle style;
  style.ns = "path";
  const Marker marker = makeLineStrip({mc::Vec3(0, 0, 0), mc::Vec3(1, 0, 0), mc::Vec3(1, 1, 0)}, style, 0.01);
  EXPECT_EQ(marker.type, Marker::LINE_STRIP);
  EXPECT_EQ(marker.points.size(), 3U);
  EXPECT_DOUBLE_EQ(marker.scale.x, 0.01);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);
}

// ---------------------------------------------------------------------------------------------
// Built-in robots
// ---------------------------------------------------------------------------------------------

TEST(Robots, BuiltinRobotsHaveConsistentNamesAndHomes) {
  for (const std::string& name : builtinRobotNames()) {
    const mc::RobotModel model = makeRobot(name);
    const std::vector<std::string> joints = jointNames(model);
    ASSERT_EQ(static_cast<int>(joints.size()), model.dof()) << name;
    for (int i = 0; i < model.dof(); ++i) {
      EXPECT_EQ(joints[static_cast<std::size_t>(i)], "joint" + std::to_string(i + 1)) << name;
    }
    const mc::VecX home = homeConfiguration(name);
    EXPECT_EQ(home.size(), model.dof()) << name;
    EXPECT_TRUE(model.withinLimits(home)) << name;
  }
  EXPECT_THROW((void)makeRobot("unknown"), std::invalid_argument);
}

TEST(Robots, ScenariosAndUrdf) {
  const auto scenario = findScenario("redundant7r", "tabletop_reach");
  ASSERT_TRUE(scenario.has_value());
  EXPECT_FALSE(scenario->obstacles.empty());
  EXPECT_FALSE(findScenario("redundant7r", "no_such_scenario").has_value());
  EXPECT_FALSE(findScenario("industrial6r", "tabletop_reach").has_value());
  EXPECT_EQ(scenarioNames("redundant7r").size(), 6U);

  const std::string urdf = mc::toURDF(makeRobot("redundant7r"));
  EXPECT_NE(urdf.find("<link name=\"base_link\">"), std::string::npos);
  EXPECT_NE(urdf.find("<link name=\"tool0\"/>"), std::string::npos);
  EXPECT_NE(urdf.find("<joint name=\"joint7\""), std::string::npos);
}

}  // namespace
}  // namespace manipulatorcore_ros
