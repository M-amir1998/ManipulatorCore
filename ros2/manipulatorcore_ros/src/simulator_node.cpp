// Real-time ManipulatorCore simulation of a built-in robot under computed-torque control.
//
// Subscriptions:  joint_trajectory (trajectory_msgs/msg/JointTrajectory)
// Publications:   joint_states     (sensor_msgs/msg/JointState)
//
// The simulator advances in fixed steps of sim_dt, paced by a wall-clock timer, and holds the
// current configuration until a trajectory arrives. A received trajectory replaces the active one
// and starts immediately (header.stamp is not used); an empty trajectory stops the robot where it
// is.

#include "manipulatorcore/control/controller.hpp"
#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/simulation/simulator.hpp"
#include "manipulatorcore_ros/conversions.hpp"
#include "manipulatorcore_ros/robots.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace manipulatorcore_ros {

namespace {

/// When the simulation falls further behind the wall clock than this, it skips ahead [s].
constexpr double kMaxRealTimeLag = 0.25;

/// Value of a double-array gain parameter: empty -> @p fallback, one entry -> broadcast.
mc::VecX gainVector(const std::vector<double>& values, const mc::VecX& fallback, const char* name) {
  if (values.empty()) {
    return fallback;
  }
  const mc::Index n = fallback.size();
  if (values.size() == 1) {
    return mc::VecX::Constant(n, values.front());
  }
  if (static_cast<mc::Index>(values.size()) != n) {
    throw std::invalid_argument(std::string("parameter '") + name + "' needs 1 or " + std::to_string(n) +
                                " entries, got " + std::to_string(values.size()));
  }
  mc::VecX v = Eigen::Map<const mc::VecX>(values.data(), n);
  if (!v.allFinite() || (v.array() < 0.0).any()) {
    throw std::invalid_argument(std::string("parameter '") + name + "' must be finite and non-negative");
  }
  return v;
}

std::string formatVector(const mc::VecX& v) {
  std::string out = "[";
  for (mc::Index i = 0; i < v.size(); ++i) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%s%.4g", i == 0 ? "" : ", ", v(i));
    out += buffer;
  }
  return out + "]";
}

}  // namespace

class SimulatorNode : public rclcpp::Node {
 public:
  explicit SimulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("manipulatorcore_simulator", options) {
    const std::string robot =
        declareParameter<std::string>("robot", "redundant7r", "Built-in robot: " + joinNames(builtinRobotNames()));
    const std::string scenario = declareParameter<std::string>(
        "scenario", "", "Start at the start configuration of this planning scenario (ignored when empty)");
    const auto initial = declareParameter<std::vector<double>>(
        "initial_configuration", {}, "Initial joint positions [rad]; overrides 'scenario' when not empty");
    const double dt = declareParameter<double>("sim_dt", 1e-3, "Simulator time step [s]");
    const double rate = declareParameter<double>("publish_rate", 100.0, "joint_states rate and pacing rate [Hz]");
    const double omega = declareParameter<double>(
        "natural_frequency", 20.0, "Closed-loop natural frequency for critically damped default gains [rad/s]");
    const auto kp_param = declareParameter<std::vector<double>>("kp", {}, "Position gains (1 or dof entries)");
    const auto kd_param = declareParameter<std::vector<double>>("kd", {}, "Velocity gains (1 or dof entries)");

    if (!(std::isfinite(dt) && dt > 0.0 && dt <= 0.01)) {
      throw std::invalid_argument("parameter 'sim_dt' must be in (0, 0.01] s");
    }
    if (!(std::isfinite(rate) && rate > 0.0 && rate <= 1000.0)) {
      throw std::invalid_argument("parameter 'publish_rate' must be in (0, 1000] Hz");
    }

    mc::RobotModel model = makeRobot(robot);
    joint_names_ = jointNames(model);
    const int dof = model.dof();

    mc::VecX q0 = homeConfiguration(robot);
    std::string q0_source = "home configuration";
    if (!initial.empty()) {
      if (static_cast<int>(initial.size()) != dof) {
        throw std::invalid_argument("parameter 'initial_configuration' needs " + std::to_string(dof) +
                                    " entries, got " + std::to_string(initial.size()));
      }
      q0 = Eigen::Map<const mc::VecX>(initial.data(), dof);
      q0_source = "initial_configuration";
    } else if (!scenario.empty() && scenario != "none") {
      if (const auto s = findScenario(robot, scenario)) {
        q0 = s->start;
        q0_source = "start of scenario '" + scenario + "'";
      } else if (scenarioNames(robot).empty()) {
        RCLCPP_WARN(get_logger(), "Robot '%s' has no planning scenarios; ignoring scenario '%s'", robot.c_str(),
                    scenario.c_str());
      } else {
        throw std::invalid_argument("unknown scenario '" + scenario +
                                    "' (expected one of: " + joinNames(scenarioNames(robot)) + ")");
      }
    }
    if (!q0.allFinite()) {
      throw std::invalid_argument("initial configuration is not finite");
    }
    if (!model.withinLimits(q0)) {
      RCLCPP_WARN(get_logger(), "Initial configuration %s is outside the joint limits", formatVector(q0).c_str());
    }

    mc::VecX kp_default;
    mc::VecX kd_default;
    mc::ComputedTorqueController::criticallyDampedGains(omega, dof, kp_default, kd_default);
    const mc::VecX kp = gainVector(kp_param, kp_default, "kp");
    const mc::VecX kd = gainVector(kd_param, kd_default, "kd");

    mc::SimulatorOptions sim_options;
    sim_options.dt = dt;
    // The node streams the state instead of keeping the in-memory log.
    sim_options.record_every = std::numeric_limits<int>::max();
    simulator_ = std::make_unique<mc::Simulator>(model, sim_options);
    simulator_->reset(q0);
    hold_q_ = q0;
    zero_ = mc::VecX::Zero(dof);
    controller_ =
        std::make_unique<mc::ComputedTorqueController>(model, kp, kd, [this](double t) { return reference(t); });

    const double period = 1.0 / rate;
    max_steps_per_tick_ = std::max(1, static_cast<int>(std::ceil(4.0 * period / dt)));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory", 10,
        [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { onTrajectory(*msg); });

    wall_start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { onTimer(); });

    RCLCPP_INFO(get_logger(), "Simulating %s (%d joints) from the %s %s", model.name().c_str(), dof, q0_source.c_str(),
                formatVector(q0).c_str());
    RCLCPP_INFO(get_logger(), "Computed-torque control, kp = %s, kd = %s; sim_dt = %g s, publishing at %g Hz",
                formatVector(kp).c_str(), formatVector(kd).c_str(), dt, rate);
  }

 private:
  template <typename T>
  T declareParameter(const std::string& name, const T& default_value, const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  [[nodiscard]] mc::JointReference reference(double t) const {
    if (trajectory_) {
      const double tau = t - trajectory_start_;
      if (tau < trajectory_->duration()) {
        mc::TrajectoryPoint p = trajectory_->sample(std::max(tau, 0.0));
        return {std::move(p.q), std::move(p.qd), std::move(p.qdd)};
      }
      return {trajectory_->sample(trajectory_->duration()).q, zero_, zero_};
    }
    return {hold_q_, zero_, zero_};
  }

  void onTrajectory(const trajectory_msgs::msg::JointTrajectory& msg) {
    if (msg.points.empty()) {
      hold_q_ = simulator_->q();
      trajectory_.reset();
      RCLCPP_INFO(get_logger(), "Received an empty trajectory: stopping at the current configuration");
      return;
    }
    try {
      auto trajectory =
          std::make_shared<const mc::CubicSplineTrajectory>(fromJointTrajectoryMsg(msg, joint_names_, simulator_->q()));
      const double jump = (trajectory->sample(0.0).q - simulator_->q()).cwiseAbs().maxCoeff();
      if (jump > 0.05) {
        RCLCPP_WARN(get_logger(), "Trajectory starts %.3f rad away from the current configuration", jump);
      }
      if (trajectory_) {
        RCLCPP_INFO(get_logger(), "Replacing the active trajectory");
      }
      trajectory_ = std::move(trajectory);
      trajectory_start_ = simulator_->time();
      RCLCPP_INFO(get_logger(), "Executing trajectory: %zu points, duration %.3f s", msg.points.size(),
                  trajectory_->duration());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Rejected trajectory: %s", e.what());
    }
  }

  void onTimer() {
    const auto now = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(now - wall_start_).count();
    const double dt = simulator_->options().dt;
    try {
      int steps = 0;
      while (simulator_->time() + 0.5 * dt < wall && steps < max_steps_per_tick_) {
        simulator_->step(*controller_);
        ++steps;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Simulation step failed (%s); holding the last valid configuration", e.what());
      hold_q_ = simulator_->q();
      trajectory_.reset();
      simulator_->reset(hold_q_);
      wall_start_ = now;
    }

    const double lag = wall - simulator_->time();
    if (lag > kMaxRealTimeLag) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Simulation is %.3f s behind real time; skipping ahead (reduce the load or increase sim_dt)",
                           lag);
      wall_start_ +=
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(lag));
    }

    if (trajectory_ && simulator_->time() - trajectory_start_ >= trajectory_->duration()) {
      hold_q_ = trajectory_->sample(trajectory_->duration()).q;
      const double error = (simulator_->q() - hold_q_).cwiseAbs().maxCoeff();
      RCLCPP_INFO(get_logger(), "Trajectory finished after %.3f s; max joint error at the end %.5f rad",
                  trajectory_->duration(), error);
      trajectory_.reset();
    }

    sensor_msgs::msg::JointState msg = makeJointState(joint_names_, simulator_->q(), simulator_->qd());
    msg.header.stamp = get_clock()->now();
    joint_state_pub_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::unique_ptr<mc::Simulator> simulator_;
  std::unique_ptr<mc::ComputedTorqueController> controller_;
  mc::VecX hold_q_;
  mc::VecX zero_;
  std::shared_ptr<const mc::JointTrajectory> trajectory_;
  double trajectory_start_ = 0.0;
  int max_steps_per_tick_ = 1;
  std::chrono::steady_clock::time_point wall_start_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace manipulatorcore_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<manipulatorcore_ros::SimulatorNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("manipulatorcore_simulator"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
