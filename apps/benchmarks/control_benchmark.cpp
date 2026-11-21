#include "benchmarks.hpp"
#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/control/metrics.hpp"
#include "manipulatorcore/control/task_space_controllers.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/simulation/simulator.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"

#include <memory>
#include <sstream>
#include <vector>

namespace mc::bench {

namespace {

/// Mean wall-clock time of computeTorque() over a fixed set of states (separate from simulation).
double controlCallMicros(Controller& c, const RobotModel& model) {
  Random rng(5);
  std::vector<ControllerInput> inputs(200);
  for (std::size_t k = 0; k < inputs.size(); ++k) {
    inputs[k].time = 0.001 * static_cast<double>(k);
    inputs[k].q = 0.5 * rng.uniformVector(model.lowerLimits(), model.upperLimits());
    inputs[k].qd = 0.2 * rng.uniformVector(-VecX::Ones(model.dof()), VecX::Ones(model.dof()));
  }
  double sink = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  const int repeats = 10;
  for (int r = 0; r < repeats; ++r) {
    for (const ControllerInput& in : inputs) {
      sink += c.computeTorque(in)(0);
    }
  }
  const double us = 1e6 * secondsSince(t0) / (repeats * static_cast<double>(inputs.size()));
  return sink == 12345.678 ? -1.0 : us;  // keep the result observable
}

struct Row {
  std::string label;
  TrackingMetrics metrics;
  double call_us;
};

TrackingMetrics simulateJoint(Controller& c, const RobotModel& model, const VecX& q0, const JointReferenceFn& ref,
                              double duration) {
  Simulator sim(model);
  sim.reset(q0);
  const SimulationLog& log = sim.run(c, duration);
  TrackingSignals s;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    const VecX q_ref = ref(log.time[k]).q;
    const SE3 T_ref = endEffectorPose(model, q_ref);
    s.time.push_back(log.time[k]);
    s.q.push_back(log.q[k]);
    s.q_desired.push_back(q_ref);
    s.tau.push_back(log.tau[k]);
    s.position_error.push_back((T_ref.translation() - log.ee_pose[k].translation()).norm());
    s.orientation_error.push_back(rotationDistance(T_ref.rotation(), log.ee_pose[k].rotation()));
  }
  return computeTrackingMetrics(s, 1e-3);
}

TrackingMetrics simulateCartesian(Controller& c, const RobotModel& model, const VecX& q0,
                                  const CartesianTrajectory& traj, double duration) {
  Simulator sim(model);
  sim.reset(q0);
  const SimulationLog& log = sim.run(c, duration);
  TrackingSignals s;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    const SE3 T_ref = traj.sample(log.time[k]).pose;
    s.time.push_back(log.time[k]);
    s.q.push_back(log.q[k]);
    s.q_desired.push_back(log.q[k]);  // joint error is not defined for task-space references
    s.tau.push_back(log.tau[k]);
    s.position_error.push_back((T_ref.translation() - log.ee_pose[k].translation()).norm());
    s.orientation_error.push_back(rotationDistance(T_ref.rotation(), log.ee_pose[k].rotation()));
  }
  return computeTrackingMetrics(s, 1e-3);
}

std::string table(const std::vector<Row>& rows, bool joint_space) {
  std::ostringstream md;
  md << "| Controller |" << (joint_space ? " Joint RMSE |" : "")
     << " Cartesian RMSE | Orientation RMSE | RMS torque | Peak torque | Control effort | Settling time (1 mm) | "
        "Time per control call |\n"
     << "| --- |" << (joint_space ? " ---: |" : "") << " ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n";
  for (const Row& r : rows) {
    const TrackingMetrics& m = r.metrics;
    md << "| " << r.label << " |" << (joint_space ? " " + fmt("%.2e rad", m.joint_rmse) + " |" : "") << " "
       << fmt("%.2e m", m.position_rmse) << " | " << fmt("%.2e rad", m.orientation_rmse) << " | "
       << fmt("%.1f N m", m.rms_torque) << " | " << fmt("%.1f N m", m.peak_torque) << " | "
       << fmt("%.0f", m.control_effort) << " | " << fmt("%.2f s", m.settling_time) << " | " << fmt("%.1f us", r.call_us)
       << " |\n";
  }
  return md.str();
}

}  // namespace

std::string runControlBenchmark() {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  std::ostringstream md;
  md << "## Control\n\n"
     << "Protocol: the generic 7-DOF arm in the simulator (RK4, 1 ms, joint damping and torque saturation "
        "enabled; the controllers' models do not include the joint damping). All controllers of a table track the "
        "same reference; metrics are computed from the simulated motion. The settling time is measured on the "
        "end-effector position error with a 1 mm band. Time per control call is measured separately on 200 "
        "states.\n\n";
  {
    VecX q1(7);
    q1 << 1.1, 0.15, -0.6, 1.25, 0.7, 0.5, -0.9;
    const auto traj = std::make_shared<PolynomialTrajectory>(PolynomialTrajectory::quintic(q0, q1, 1.6));
    const JointReferenceFn ref = trajectoryReference(traj);
    JointPIDGains pd{VecX::Constant(7, 400.0), VecX(), VecX::Constant(7, 40.0)};
    JointPIDGains pid{VecX::Constant(7, 400.0), VecX::Constant(7, 200.0), VecX::Constant(7, 40.0)};
    VecX kp;
    VecX kd;
    ComputedTorqueController::criticallyDampedGains(25.0, 7, kp, kd);
    std::vector<std::pair<std::string, std::unique_ptr<Controller>>> controllers;
    controllers.emplace_back("PD", std::make_unique<JointPIDController>(model, pd, ref, false));
    controllers.emplace_back("PD + gravity", std::make_unique<JointPIDController>(model, pd, ref, true));
    controllers.emplace_back("PID + gravity", std::make_unique<JointPIDController>(model, pid, ref, true));
    controllers.emplace_back("Computed torque", std::make_unique<ComputedTorqueController>(model, kp, kd, ref));
    std::vector<Row> rows;
    for (auto& [label, c] : controllers) {
      const TrackingMetrics m = simulateJoint(*c, model, q0, ref, 2.2);
      c->reset();
      rows.push_back({label, m, controlCallMicros(*c, model)});
      std::printf("  control/joint/%s: %.2e rad\n", label.c_str(), m.joint_rmse);
    }
    md << "### Joint-space tracking (quintic, 1.6 s, then 0.6 s hold)\n\n"
       << "Gains: Kp = 400 N m/rad, Kd = 40 N m s/rad (PD family; PID adds Ki = 200); computed torque uses "
          "critically damped error dynamics with omega = 25 rad/s.\n\n"
       << table(rows, true) << "\n";
  }
  {
    const SE3 T0 = endEffectorPose(model, q0);
    const SE3 T1(T0.rotation() * rotY(0.3), T0.translation() + Vec3(-0.1, 0.3, 0.1));
    const auto path = std::make_shared<LinePath>(T0, T1);
    const auto traj = std::make_shared<CartesianTrajectory>(CartesianTrajectory::quintic(path, 2.0));
    const CartesianReferenceFn ref = cartesianTrajectoryReference(traj);
    const NullSpaceTask posture{q0, 10.0, 3.0, 10.0, 0.1};
    std::vector<std::pair<std::string, std::unique_ptr<Controller>>> controllers;
    controllers.emplace_back("Task-space PD", std::make_unique<TaskSpacePDController>(
                                                  model, diagonalGain(150, 1500), diagonalGain(15, 120), ref, posture));
    controllers.emplace_back("Cartesian impedance",
                             std::make_unique<CartesianImpedanceController>(model, diagonalGain(150, 1500),
                                                                            diagonalGain(15, 120), ref, posture));
    controllers.emplace_back("Operational space",
                             std::make_unique<OperationalSpaceController>(model, diagonalGain(400, 400),
                                                                          diagonalGain(40, 40), ref, posture));
    std::vector<Row> rows;
    for (auto& [label, c] : controllers) {
      const TrackingMetrics m = simulateCartesian(*c, model, q0, *traj, 2.6);
      rows.push_back({label, m, controlCallMicros(*c, model)});
      std::printf("  control/task/%s: %.2e m\n", label.c_str(), m.position_rmse);
    }
    md << "### Task-space tracking (straight line 0.35 m with 0.3 rad rotation, 2 s, then 0.6 s hold)\n\n"
       << "Task-space PD and impedance use K = diag(150 N m/rad, 1500 N/m), D = diag(15, 120); "
          "operational-space control uses Kp = 400 1/s^2, Kd = 40 1/s on all six task coordinates. All use the "
          "same null-space posture task.\n\n"
       << table(rows, false) << "\n";
  }
  return md.str();
}

}  // namespace mc::bench
