// Control demos: controller comparison on a common trajectory, interaction control against a
// compliant wall, null-space redundancy resolution, gravity compensation and operational-space
// tracking. Every motion shown is the output of the simulator, not a replay of the reference.

#include "../common/export.hpp"
#include "demos.hpp"
#include "manipulatorcore/control/joint_controllers.hpp"
#include "manipulatorcore/control/metrics.hpp"
#include "manipulatorcore/control/task_space_controllers.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/simulation/simulator.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

namespace mc::demos {

using apps::Json;

namespace {

constexpr double kFrameDt = 0.02;  // 50 frames per second of simulated time

Json metricsJson(const TrackingMetrics& m) {
  return Json::object()
      .set("joint_rmse", m.joint_rmse)
      .set("max_joint_error", m.max_joint_error)
      .set("position_rmse", m.position_rmse)
      .set("rms_torque", m.rms_torque)
      .set("peak_torque", m.peak_torque);
}

void controllerComparison(const std::filesystem::path& out) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  VecX q1(7);
  q1 << 1.1, 0.15, -0.6, 1.25, 0.7, 0.5, -0.9;
  const auto traj = std::make_shared<PolynomialTrajectory>(PolynomialTrajectory::quintic(q0, q1, 1.6));
  const JointReferenceFn ref = trajectoryReference(traj);
  const double duration = 2.2;

  JointPIDGains gains{VecX::Constant(7, 400.0), VecX(), VecX::Constant(7, 40.0)};
  VecX kp;
  VecX kd;
  ComputedTorqueController::criticallyDampedGains(25.0, 7, kp, kd);
  std::vector<std::pair<std::string, std::unique_ptr<Controller>>> controllers;
  controllers.emplace_back("PD", std::make_unique<JointPIDController>(model, gains, ref, false));
  controllers.emplace_back("PD + gravity", std::make_unique<JointPIDController>(model, gains, ref, true));
  controllers.emplace_back("computed torque", std::make_unique<ComputedTorqueController>(model, kp, kd, ref));

  Json runs = Json::array();
  for (auto& [label, controller] : controllers) {
    SimulatorOptions opt;
    opt.record_every = static_cast<int>(std::lround(kFrameDt / opt.dt));
    Simulator sim(model, opt);
    sim.reset(q0);
    const SimulationLog& log = sim.run(*controller, duration);
    TrackingSignals signals;
    Json frames = Json::array();
    for (std::size_t k = 0; k < log.time.size(); ++k) {
      const VecX q_ref = ref(log.time[k]).q;
      signals.time.push_back(log.time[k]);
      signals.q.push_back(log.q[k]);
      signals.q_desired.push_back(q_ref);
      signals.tau.push_back(log.tau[k]);
      signals.position_error.push_back(
          (endEffectorPose(model, q_ref).translation() - log.ee_pose[k].translation()).norm());
      frames.push(apps::robotFrame(model, log.q[k], log.time[k])
                      .set("reference", apps::robotFrame(model, q_ref, log.time[k]))
                      .set("joint_error", (q_ref - log.q[k]).norm())
                      .set("position_error", signals.position_error.back())
                      .set("tau", apps::vec(log.tau[k])));
    }
    const TrackingMetrics m = computeTrackingMetrics(signals);
    runs.push(Json::object().set("controller", label).set("frames", frames).set("metrics", metricsJson(m)));
    std::printf("  computed_torque/%s: joint RMSE %.4f rad, EE RMSE %.4f m\n", label.c_str(), m.joint_rmse,
                m.position_rmse);
  }
  Json doc = Json::object();
  doc.set("base", apps::shapes(model.baseCollision())).set("runs", runs);
  apps::writeJson(out, "computed_torque", doc);
}

/// Wall in front of the end-effector: solid for x >= wall_x.
ContactSurface wallAt(double wall_x) {
  ContactSurface wall;
  wall.normal = -Vec3::UnitX();
  wall.offset = -wall_x;
  wall.stiffness = 8000.0;
  wall.damping = 150.0;
  wall.tangential_damping = 20.0;
  wall.name = "wall";
  return wall;
}

void interaction(const std::filesystem::path& out, const std::string& kind) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 T0 = endEffectorPose(model, q0);
  const double wall_x = T0.translation().x() + 0.12;
  // Desired motion: approach the wall and continue 6 cm "into" it, then hold.
  const SE3 T1(T0.rotation(), Vec3(wall_x + 0.06, T0.translation().y(), T0.translation().z()));
  const auto line = std::make_shared<LinePath>(T0, T1);
  const auto ctraj = std::make_shared<CartesianTrajectory>(CartesianTrajectory::quintic(line, 2.0));
  const CartesianReferenceFn ref = cartesianTrajectoryReference(ctraj);
  NullSpaceTask posture{q0, 8.0, 3.0, 0.0, 0.1};

  std::unique_ptr<Controller> controller;
  const double K = 600.0;
  if (kind == "impedance") {
    controller = std::make_unique<CartesianImpedanceController>(
        model, diagonalGain(60.0, K), diagonalGain(6.0, 2.0 * std::sqrt(K * 4.0)), ref, posture);
  } else {
    AdmittanceParameters adm;
    adm.stiffness = Vec3::Constant(K);
    adm.mass = Vec3::Constant(3.0);
    adm.damping = Vec3::Constant(2.0 * std::sqrt(K * 3.0) * 1.2);
    controller =
        std::make_unique<AdmittanceController>(model, adm, diagonalGain(400.0, 900.0), diagonalGain(40.0, 60.0), ref);
  }
  SimulatorOptions opt;
  opt.dt = 5e-4;
  opt.contacts = {wallAt(wall_x)};
  opt.contact_point.radius = 0.02;
  opt.record_every = static_cast<int>(std::lround(kFrameDt / opt.dt));
  Simulator sim(model, opt);
  sim.reset(q0);
  const SimulationLog& log = sim.run(*controller, 3.5);
  Json frames = Json::array();
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    frames.push(apps::robotFrame(model, log.q[k], log.time[k])
                    .set("desired", apps::pose(ctraj->sample(log.time[k]).pose))
                    .set("force", apps::vec3(log.contact_force[k])));
  }
  const double f_end = log.contact_force.back().norm();
  std::printf("  %s: steady contact force %.1f N (controller stiffness %.0f N/m, desired penetration 6 cm)\n",
              kind.c_str(), f_end, K);
  Json doc = Json::object();
  doc.set("controller", controller->name())
      .set("wall_x", wall_x)
      .set("contact_radius", opt.contact_point.radius)
      .set("stiffness", K)
      .set("base", apps::shapes(model.baseCollision()))
      .set("frames", frames);
  apps::writeJson(out, kind, doc);
}

void nullSpace(const std::filesystem::path& out) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 hold = endEffectorPose(model, q0);
  // The posture target swings the upper-arm roll; the end-effector must not move.
  auto posture_ref = [q0](double t) {
    VecX p = q0;
    p(2) = 1.1 * std::sin(kTwoPi * t / 4.0);
    p(0) = 0.35 * std::sin(kTwoPi * t / 4.0);
    return p;
  };
  // Operational-space control holds the end-effector; its null-space posture target follows
  // posture_ref(t), and a smooth joint-limit repulsion keeps the joints away from their stops.
  class PostureSweep final : public Controller {
   public:
    PostureSweep(RobotModel model, SE3 hold, std::function<VecX(double)> posture)
        : model_(std::move(model)), hold_(hold), posture_(std::move(posture)) {}
    [[nodiscard]] std::string name() const override { return "operational space + posture sweep"; }
    VecX computeTorque(const ControllerInput& input) override {
      OperationalSpaceController osc(model_, diagonalGain(400.0, 400.0), diagonalGain(40.0, 40.0),
                                     constantPoseReference(hold_), {posture_(input.time), 40.0, 6.0, 20.0, 0.12});
      return osc.computeTorque(input);
    }

   private:
    RobotModel model_;
    SE3 hold_;
    std::function<VecX(double)> posture_;
  };
  PostureSweep controller(model, hold, posture_ref);
  SimulatorOptions opt;
  opt.record_every = static_cast<int>(std::lround(kFrameDt / opt.dt));
  Simulator sim(model, opt);
  sim.reset(q0);
  const SimulationLog& log = sim.run(controller, 8.0);
  Json frames = Json::array();
  double worst = 0.0;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    const PoseDistance d = poseDistance(log.ee_pose[k], hold);
    worst = std::max(worst, d.translation);
    frames.push(apps::robotFrame(model, log.q[k], log.time[k])
                    .set("ee_error", d.translation)
                    .set("posture_target", apps::vec(posture_ref(log.time[k]))));
  }
  std::printf("  nullspace: elbow swings while the end-effector stays within %.2f mm\n", worst * 1e3);
  Json doc = Json::object();
  doc.set("hold", apps::pose(hold))
      .set("lower", apps::vec(model.lowerLimits()))
      .set("upper", apps::vec(model.upperLimits()))
      .set("base", apps::shapes(model.baseCollision()))
      .set("frames", frames);
  apps::writeJson(out, "nullspace", doc);
}

void gravityHold(const std::filesystem::path& out) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  JointPIDGains gains{VecX::Constant(7, 150.0), VecX(), VecX::Constant(7, 20.0)};
  Json runs = Json::array();
  for (const bool compensate : {false, true}) {
    JointPIDController pd(model, gains, constantJointReference(q0), compensate);
    SimulatorOptions opt;
    opt.record_every = static_cast<int>(std::lround(kFrameDt / opt.dt));
    Simulator sim(model, opt);
    sim.reset(q0);
    const SimulationLog& log = sim.run(pd, 3.0);
    Json frames = Json::array();
    for (std::size_t k = 0; k < log.time.size(); ++k) {
      frames.push(apps::robotFrame(model, log.q[k], log.time[k]));
    }
    runs.push(Json::object().set("controller", pd.name()).set("frames", frames));
    std::printf("  gravity_hold/%s: final joint error %.3f rad\n", pd.name().c_str(), (log.q.back() - q0).norm());
  }
  Json doc = Json::object();
  doc.set("target", apps::robotFrame(model, q0)).set("base", apps::shapes(model.baseCollision())).set("runs", runs);
  apps::writeJson(out, "gravity_hold", doc);
}

void operationalSpaceCircle(const std::filesystem::path& out) {
  const RobotModel model = makeRedundant7R();
  const VecX q0 = redundant7RHome();
  const SE3 T0 = endEffectorPose(model, q0);
  const auto circle = std::make_shared<CircularArcPath>(T0, T0.translation() + Vec3(0.0, 0.15, 0.0), Vec3::UnitZ(),
                                                        kTwoPi, T0.rotation());
  const auto ctraj = std::make_shared<CartesianTrajectory>(CartesianTrajectory::quintic(circle, 4.0));
  OperationalSpaceController osc(model, diagonalGain(400.0, 600.0), diagonalGain(40.0, 50.0),
                                 cartesianTrajectoryReference(ctraj), {q0, 10.0, 3.0, 10.0, 0.1});
  SimulatorOptions opt;
  opt.record_every = static_cast<int>(std::lround(kFrameDt / opt.dt));
  Simulator sim(model, opt);
  sim.reset(q0);
  const SimulationLog& log = sim.run(osc, 4.5);
  Json frames = Json::array();
  double worst = 0.0;
  for (std::size_t k = 0; k < log.time.size(); ++k) {
    const Vec3 desired = ctraj->sample(log.time[k]).pose.translation();
    worst = std::max(worst, (desired - log.ee_pose[k].translation()).norm());
    frames.push(apps::robotFrame(model, log.q[k], log.time[k]).set("desired", apps::vec3(desired)));
  }
  std::printf("  operational_space: max tracking error %.2f mm on a 0.15 m circle\n", worst * 1e3);
  Json doc = Json::object();
  doc.set("base", apps::shapes(model.baseCollision())).set("frames", frames);
  apps::writeJson(out, "operational_space", doc);
}

}  // namespace

std::vector<Demo> controlDemos() {
  return {{"computed_torque", "PD vs PD + gravity vs computed torque on the same trajectory", controllerComparison},
          {"impedance", "Cartesian impedance control pressing against a compliant wall",
           [](const std::filesystem::path& o) { interaction(o, "impedance"); }},
          {"admittance", "Admittance control pressing against a compliant wall",
           [](const std::filesystem::path& o) { interaction(o, "admittance"); }},
          {"nullspace", "Null-space posture motion with a fixed end-effector pose", nullSpace},
          {"gravity_hold", "PD without and with gravity compensation holding a pose", gravityHold},
          {"operational_space", "Operational-space control tracking a circle", operationalSpaceCircle}};
}

}  // namespace mc::demos
