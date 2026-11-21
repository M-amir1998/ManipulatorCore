// Pose-goal planning with ManipulatorCore.
//
// Subscriptions:  goal_pose              (geometry_msgs/msg/PoseStamped, frame base_frame)
//                 joint_states           (sensor_msgs/msg/JointState)
// Publications:   joint_trajectory       (trajectory_msgs/msg/JointTrajectory)
//                 planning_scene_markers (visualization_msgs/msg/MarkerArray, transient local)
//
// Each goal is planned from the latest joint state against the obstacles of the selected
// scenario (multi-start IK, sampling-based planning, shortcutting, spline smoothing and
// time-optimal timing). Planning runs on a worker thread; goals that arrive while a plan is being
// computed are rejected. Failures are logged and never stop the node.

#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/planning/pose_goal_planning.hpp"
#include "manipulatorcore_ros/conversions.hpp"
#include "manipulatorcore_ros/robots.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace manipulatorcore_ros {

namespace {

constexpr const char* kNoScenario = "none";
constexpr double kGoalArrowLength = 0.15;  // [m]

}  // namespace

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("manipulatorcore_planner", options) {
    robot_name_ =
        declareParameter<std::string>("robot", "redundant7r", "Built-in robot: " + joinNames(builtinRobotNames()));
    scenario_name_ = declareParameter<std::string>(
        "scenario", "tabletop_reach", "Planning scenario whose obstacles are used, or 'none' for free space");
    base_frame_ = declareParameter<std::string>("base_frame", "base_link", "Frame of goals and markers");
    trajectory_dt_ =
        declareParameter<double>("trajectory_dt", 0.01, "Sampling interval of the published trajectory [s]");
    const std::string planner =
        declareParameter<std::string>("planner", "RRT-Connect", "Sampling-based planner: RRT, RRT-Connect, RRT*, PRM");
    const auto ik_attempts = declareParameter<int64_t>("ik_attempts", 32, "Multi-start IK seeds per goal");
    const auto max_candidates =
        declareParameter<int64_t>("max_goal_candidates", 4, "IK solutions the planner tries to reach");
    const auto max_iterations =
        declareParameter<int64_t>("max_iterations", 20000, "Planner sample budget per IK solution");
    const double time_limit =
        declareParameter<double>("planning_time_limit", 5.0, "Planner wall-clock limit per IK solution [s], 0 = none");
    const double clearance = declareParameter<double>("clearance", 0.02, "Robot-obstacle clearance while planning [m]");
    const auto seed = declareParameter<int64_t>("seed", 1, "Seed of IK restarts and planner sampling");

    if (!(std::isfinite(trajectory_dt_) && trajectory_dt_ > 0.0)) {
      throw std::invalid_argument("parameter 'trajectory_dt' must be positive");
    }
    if (ik_attempts < 1 || max_candidates < 1 || max_iterations < 1 || seed < 0) {
      throw std::invalid_argument(
          "parameters 'ik_attempts', 'max_goal_candidates' and 'max_iterations' must be positive and 'seed' "
          "non-negative");
    }
    if (!(std::isfinite(time_limit) && time_limit >= 0.0) || !(std::isfinite(clearance) && clearance >= 0.0)) {
      throw std::invalid_argument("parameters 'planning_time_limit' and 'clearance' must be non-negative");
    }
    if (planner != "RRT" && planner != "RRT-Connect" && planner != "RRT*" && planner != "PRM") {
      throw std::invalid_argument("unknown planner '" + planner + "' (expected RRT, RRT-Connect, RRT* or PRM)");
    }

    options_.planner = planner;
    options_.ik.attempts = static_cast<int>(ik_attempts);
    options_.ik.seed = static_cast<std::uint64_t>(seed);
    options_.max_goal_candidates = static_cast<int>(max_candidates);
    options_.planner_settings.seed = static_cast<std::uint64_t>(seed);
    options_.planner_settings.max_iterations = static_cast<int>(max_iterations);
    options_.planner_settings.time_limit = time_limit;
    options_.clearance = clearance;

    mc::RobotModel model = makeRobot(robot_name_);
    joint_names_ = jointNames(model);
    std::vector<mc::Shape> obstacles;
    if (scenario_name_ != kNoScenario && !scenario_name_.empty()) {
      if (auto scenario = findScenario(robot_name_, scenario_name_)) {
        obstacles = std::move(scenario->obstacles);
      } else if (scenarioNames(robot_name_).empty()) {
        RCLCPP_WARN(get_logger(), "Robot '%s' has no planning scenarios; planning without obstacles",
                    robot_name_.c_str());
        scenario_name_ = kNoScenario;
      } else {
        throw std::invalid_argument("unknown scenario '" + scenario_name_ +
                                    "' (expected one of: " + joinNames(scenarioNames(robot_name_)) + ", none)");
      }
    } else {
      scenario_name_ = kNoScenario;
    }
    checker_ = std::make_unique<mc::RobotCollisionChecker>(std::move(model), std::move(obstacles));

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("planning_scene_markers",
                                                                         rclcpp::QoS(1).transient_local());
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { onJointState(*msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { onGoal(*msg); });

    publishMarkers(std::nullopt, nullptr);
    RCLCPP_INFO(get_logger(), "Planning for %s in scenario '%s' (%zu obstacles) with %s; waiting for goals on '%s'",
                checker_->model().name().c_str(), scenario_name_.c_str(), checker_->obstacles().size(), planner.c_str(),
                goal_sub_->get_topic_name());
  }

  ~PlannerNode() override {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  PlannerNode(const PlannerNode&) = delete;
  PlannerNode& operator=(const PlannerNode&) = delete;
  PlannerNode(PlannerNode&&) = delete;
  PlannerNode& operator=(PlannerNode&&) = delete;

 private:
  template <typename T>
  T declareParameter(const std::string& name, const T& default_value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  void onJointState(const sensor_msgs::msg::JointState& msg) {
    try {
      current_q_ = jointPositions(msg, joint_names_);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring joint state: %s", e.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped& msg) {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
      RCLCPP_ERROR(get_logger(), "Rejected goal in frame '%s': goals must be expressed in '%s'",
                   msg.header.frame_id.c_str(), base_frame_.c_str());
      return;
    }
    if (!current_q_) {
      RCLCPP_ERROR(get_logger(), "Rejected goal: no joint state received on '%s' yet",
                   joint_state_sub_->get_topic_name());
      return;
    }
    mc::SE3 target;
    try {
      target = fromPoseMsg(msg.pose);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Rejected goal: %s", e.what());
      return;
    }
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejected goal: still planning the previous goal");
      return;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    const mc::VecX start = *current_q_;
    const mc::Vec3 p = target.translation();
    const mc::Quat q = target.quaternion();
    RCLCPP_INFO(get_logger(), "Planning to position (%.3f, %.3f, %.3f), orientation xyzw (%.3f, %.3f, %.3f, %.3f)",
                p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
    worker_ = std::thread([this, start, target] {
      plan(start, target);
      busy_ = false;
    });
  }

  void plan(const mc::VecX& start, const mc::SE3& target) {
    try {
      const auto t0 = std::chrono::steady_clock::now();
      const mc::PoseGoalPlanningResult result = mc::planToPose(*checker_, start, target, options_);
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (!result.success || !result.trajectory) {
        RCLCPP_ERROR(get_logger(), "Planning failed after %.2f s: %s (%zu collision-free IK solutions)", elapsed,
                     result.message.c_str(), result.ik_candidates.size());
        publishMarkers(target, nullptr);
        return;
      }
      if (result.trajectory->duration() <= 0.0) {
        // The current configuration already satisfies the goal: there is nothing to execute.
        publishMarkers(target, nullptr);
        RCLCPP_INFO(get_logger(), "Goal already reached (planned in %.2f s); no trajectory published", elapsed);
        return;
      }
      trajectory_msgs::msg::JointTrajectory msg =
          toJointTrajectoryMsg(*result.trajectory, joint_names_, trajectory_dt_);
      msg.header.stamp = get_clock()->now();
      trajectory_pub_->publish(msg);
      publishMarkers(target, result.trajectory.get());
      const mc::SE3 reached =
          mc::endEffectorPose(checker_->model(), result.trajectory->sample(result.trajectory->duration()).q);
      const mc::PoseDistance error = mc::poseDistance(reached, target);
      RCLCPP_INFO(get_logger(),
                  "Planning succeeded in %.2f s: %zu waypoints, joint path length %.3f rad, %s timing; published "
                  "trajectory with %zu points, duration %.3f s (goal error %.1e m, %.1e rad)",
                  elapsed, result.path.size(), result.path_length, result.smoothed ? "spline" : "waypoint",
                  msg.points.size(), result.trajectory->duration(), error.translation, error.rotation);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Planning failed with an error: %s", e.what());
    }
  }

  void publishMarkers(const std::optional<mc::SE3>& goal, const mc::JointTrajectory* trajectory) {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.header.frame_id = base_frame_;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    MarkerStyle style;
    style.frame_id = base_frame_;
    style.ns = "obstacles";
    for (const mc::Shape& obstacle : checker_->obstacles()) {
      const bool half_space = std::holds_alternative<mc::HalfSpace>(obstacle.geometry);
      style.color = half_space ? makeColor(0.75F, 0.75F, 0.75F, 0.35F) : makeColor(0.85F, 0.55F, 0.25F, 0.85F);
      for (Marker& marker : toMarkers(obstacle, style)) {
        array.markers.push_back(std::move(marker));
        ++style.id;
      }
    }

    if (goal) {
      Marker arrow;
      arrow.header.frame_id = base_frame_;
      arrow.ns = "goal";
      arrow.type = Marker::ARROW;
      arrow.action = Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      const mc::Vec3 tip = goal->translation();
      const mc::Vec3 approach = goal->rotation().col(2);
      arrow.points.push_back(toPointMsg(tip - kGoalArrowLength * approach));
      arrow.points.push_back(toPointMsg(tip));
      arrow.scale.x = 0.01;
      arrow.scale.y = 0.02;
      arrow.scale.z = 0.03;
      arrow.color = makeColor(0.2F, 0.4F, 1.0F, 1.0F);
      array.markers.push_back(std::move(arrow));
    }

    if (trajectory != nullptr) {
      std::vector<mc::Vec3> points;
      for (const mc::TrajectoryPoint& p : mc::sampleUniform(*trajectory, trajectory_dt_)) {
        points.push_back(mc::endEffectorPose(checker_->model(), p.q).translation());
      }
      MarkerStyle path_style;
      path_style.frame_id = base_frame_;
      path_style.ns = "planned_path";
      path_style.color = makeColor(0.1F, 0.8F, 0.3F, 1.0F);
      array.markers.push_back(makeLineStrip(points, path_style, 0.008));
    }
    marker_pub_->publish(array);
  }

  std::string robot_name_;
  std::string scenario_name_;
  std::string base_frame_;
  double trajectory_dt_ = 0.01;
  std::vector<std::string> joint_names_;
  std::unique_ptr<const mc::RobotCollisionChecker> checker_;
  mc::PoseGoalPlanningOptions options_;
  std::optional<mc::VecX> current_q_;
  std::atomic<bool> busy_{false};
  std::thread worker_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace manipulatorcore_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<manipulatorcore_ros::PlannerNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("manipulatorcore_planner"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
