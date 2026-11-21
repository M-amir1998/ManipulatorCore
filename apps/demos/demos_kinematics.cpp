// Kinematics demos: forward/inverse kinematics, singularities, workspace, Cartesian paths,
// collision clearances and an IK local-minimum failure case.

#include "../common/export.hpp"
#include "demos.hpp"
#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/analytical_ik.hpp"
#include "manipulatorcore/kinematics/differential_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/kinematics/workspace.hpp"
#include "manipulatorcore/math/linalg.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdio>
#include <memory>

namespace mc::demos {

using apps::Json;

namespace {

Json planarModelInfo(const RobotModel& model) {
  Json l = Json::array();
  for (const double v : planarLinkLengths(model)) {
    l.push(v);
  }
  return Json::object().set("name", model.name()).set("lengths", l);
}

/// Velocity ellipse of the planar translational Jacobian: semi-axes and orientation.
Json velocityEllipse(const RobotModel& model, const VecX& q) {
  const MatX J = geometricJacobian(model, q).middleRows<2>(3);
  const Eigen::SelfAdjointEigenSolver<Mat2> eig(J * J.transpose());
  const Vec2 axes = eig.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  const Vec2 major = eig.eigenvectors().col(1);
  return Json::array().push(axes(1)).push(axes(0)).push(std::atan2(major.y(), major.x()));
}

void fkIk2R(const std::filesystem::path& out) {
  const RobotModel model = makePlanar2R();
  Json fk = Json::array();
  const int frames = 90;
  for (int k = 0; k < frames; ++k) {
    const double t = 4.0 * k / frames;
    const double phase = kTwoPi * t / 4.0;
    const Vec2 q(0.35 + 0.9 * std::sin(phase), 1.1 * std::sin(phase + 1.0) + 0.4);
    fk.push(apps::robotFrame(model, q, t).set("ellipse", velocityEllipse(model, q)));
  }
  // IK: a circle in the workspace, solved in closed form; both elbow branches are kept.
  Json ik = Json::array();
  const Vec2 center(0.85, 0.45);
  const double radius = 0.42;
  for (int k = 0; k < 110; ++k) {
    const double a = kTwoPi * k / 110.0;
    const Vec2 target = center + radius * Vec2(std::cos(a), std::sin(a));
    // Solutions are ordered q2 > 0 first, so each branch keeps its identity along the circle.
    Json sols = Json::array();
    for (const VecX& q : analyticalIKPlanar2R(model, target)) {
      sols.push(apps::robotFrame(model, q, k));
    }
    ik.push(Json::object().set("target", apps::vec(target)).set("solutions", sols));
  }
  Json doc = Json::object();
  doc.set("model", planarModelInfo(model)).set("fk", fk).set("ik", ik);
  apps::writeJson(out, "fk_ik_2r", doc);
}

/// Resolved-rate control towards a singular (fully stretched) configuration with the
/// pseudoinverse and with damped least squares.
void singularity(const std::filesystem::path& out) {
  const RobotModel model = makePlanar2R();
  const double dt = 0.01;
  const Vec2 v_cmd(0.35, 0.0);  // straight towards the outer workspace boundary
  Json runs = Json::object();
  for (const char* method : {"pinv", "dls"}) {
    Vec2 q(-0.55, 1.25);
    Json frames = Json::array();
    bool diverged = false;
    for (int k = 0; k <= 400; ++k) {
      const MatX J = geometricJacobian(model, q).middleRows<2>(3);
      const JacobianAnalysis a = analyzeJacobian(J);
      VecX qd = std::string(method) == "pinv"
                    ? jointVelocityForTask(J, v_cmd, DifferentialSolver::PseudoInverse, 1e-12)
                    : jointVelocityForTask(J, v_cmd, DifferentialSolver::DampedLeastSquares, 0.08);
      const double speed = qd.norm();
      const Vec2 v_actual = J * qd;
      frames.push(apps::robotFrame(model, q, k * dt)
                      .set("qd_norm", speed)
                      .set("sigma_min", a.sigma_min)
                      .set("condition", a.condition_number)
                      .set("manipulability", a.manipulability)
                      .set("velocity_error", (v_actual - v_cmd).norm())
                      .set("ellipse", velocityEllipse(model, q)));
      if (speed > 60.0) {  // the pseudoinverse solution explodes near the singularity
        diverged = true;
        break;
      }
      q += dt * qd;
    }
    runs.set(method, Json::object().set("frames", frames).set("diverged", diverged));
    std::printf("  singularity/%s: %s\n", method, diverged ? "joint speed diverged" : "bounded");
  }
  Json doc = Json::object();
  doc.set("model", planarModelInfo(model)).set("command", apps::vec(v_cmd)).set("dt", dt).set("runs", runs);
  apps::writeJson(out, "singularity", doc);
}

void workspace(const std::filesystem::path& out) {
  Json doc = Json::object();
  {
    const RobotModel model = makePlanar2R();
    Json pts = Json::array();
    for (const WorkspaceSample& s :
         sampleWorkspace(model, {.samples = 20000, .seed = 3, .manipulability_rows = {3, 4}})) {
      pts.push(Json::array().push(s.position.x()).push(s.position.y()).push(s.manipulability));
    }
    doc.set("planar2r", Json::object().set("model", planarModelInfo(model)).set("samples", pts));
  }
  {
    const RobotModel model = makeRedundant7R();
    Json pts = Json::array();
    for (const WorkspaceSample& s : sampleWorkspace(model, {.samples = 12000, .seed = 5})) {
      pts.push(Json::array().push(s.position.x()).push(s.position.y()).push(s.position.z()).push(s.manipulability));
    }
    doc.set("redundant7r", Json::object().set("samples", pts).set("home", apps::robotFrame(model, redundant7RHome())));
  }
  apps::writeJson(out, "workspace", doc);
}

/// End-effector line and circle on the 6R arm, solved sample by sample with warm-started DLS IK.
void cartesianPaths(const std::filesystem::path& out) {
  const RobotModel model = makeIndustrial6R();
  const VecX home = industrial6RHome();
  const SE3 T0 = endEffectorPose(model, home);
  Json paths = Json::array();
  struct Spec {
    const char* name;
    std::shared_ptr<const CartesianPath> path;
    double duration;
  };
  const SE3 line_end(T0.rotation() * rotX(0.35), T0.translation() + Vec3(-0.15, 0.45, 0.15));
  // Paths are placed where the arm stays well conditioned (sigma_min >= 0.1 along the path).
  const std::vector<Spec> specs = {{"line", std::make_shared<LinePath>(T0, line_end), 3.0},
                                   {"horizontal_circle",
                                    std::make_shared<CircularArcPath>(T0, T0.translation() + Vec3(0.0, 0.15, 0.0),
                                                                      Vec3::UnitZ(), kTwoPi, T0.rotation()),
                                    4.0},
                                   {"vertical_circle",
                                    std::make_shared<CircularArcPath>(T0, T0.translation() + Vec3(0.0, 0.10, 0.0),
                                                                      Vec3::UnitX(), kTwoPi, T0.rotation()),
                                    4.0}};
  for (const Spec& spec : specs) {
    VecX q = home;  // every path starts at the home pose T0 (the line ends elsewhere)
    const CartesianTrajectory traj = CartesianTrajectory::quintic(spec.path, spec.duration);
    Json frames = Json::array();
    double worst = 0.0;
    for (int k = 0; k <= 75; ++k) {
      const double t = spec.duration * k / 75.0;
      const CartesianSample s = traj.sample(t);
      IKOptions o;
      o.position_tolerance = 1e-6;
      o.orientation_tolerance = 1e-5;
      const IKResult r = solveIK(model, s.pose, q, o);
      q = r.q;
      worst = std::max(worst, r.position_error);
      frames.push(apps::robotFrame(model, q, t).set("target", apps::pose(s.pose)));
    }
    paths.push(Json::object().set("name", spec.name).set("frames", frames));
    std::printf("  cartesian/%s: max IK position error %.2e m\n", spec.name, worst);
  }
  Json doc = Json::object();
  doc.set("robot", model.name()).set("base", apps::shapes(model.baseCollision())).set("paths", paths);
  apps::writeJson(out, "cartesian_paths", doc);
}

/// The 6R arm sweeps past obstacles; per-link clearances and closest points are exported.
void collisionClearance(const std::filesystem::path& out) {
  const RobotModel model = makeIndustrial6R();
  const std::vector<Shape> obstacles = {
      makeBox(SE3::fromTranslation(Vec3(0.62, -0.05, 0.25)), Vec3(0.12, 0.12, 0.25), "crate"),
      makeSphere(Vec3(0.45, 0.42, 0.85), 0.12, "ball"), makeHalfSpace(Vec3::UnitZ(), 0.0, "floor")};
  const RobotCollisionChecker checker(model, obstacles);
  const VecX q0 = industrial6RHome();
  Json frames = Json::array();
  for (int k = 0; k <= 120; ++k) {
    const double s = std::sin(kTwoPi * k / 120.0);
    VecX q = q0;
    q(0) = 0.9 * s;
    q(1) = 0.45 + 0.25 * std::sin(kTwoPi * k / 60.0);
    q(2) = -0.55 + 0.2 * s;
    q(4) = -1.0 - 0.3 * s;
    Json pairs = Json::array();
    for (const CollisionPairInfo& p : checker.pairDistances(q, 0.25)) {
      pairs.push(Json::object()
                     .set("kind", p.kind == CollisionPairInfo::Kind::Self ? "self" : "env")
                     .set("body_a", p.body_a)
                     .set("body_b", p.body_b)
                     .set("distance", p.result.distance)
                     .set("a", apps::vec3(p.result.point_a))
                     .set("b", apps::vec3(p.result.point_b)));
    }
    frames.push(apps::robotFrame(model, q, k / 30.0)
                    .set("clearance", apps::vec(checker.bodyClearances(q)))
                    .set("pairs", pairs));
  }
  Json doc = Json::object();
  doc.set("obstacles", apps::shapes(obstacles)).set("frames", frames);
  apps::writeJson(out, "collision", doc);
}

/// Gradient-based IK is local: from an unlucky seed with restrictive joint limits it stalls, while
/// seeded multi-start finds a solution.
void localIkFailure(const std::filesystem::path& out) {
  RobotModel model = makePlanar3R();
  for (int i = 0; i < 3; ++i) {
    model.setJointLimits(i, {-1.9, 1.9, 3.0, 15.0, 150.0});
  }
  const Vec2 target_xy(-1.35, 0.75);
  const SE3 target = SE3::fromTranslation(Vec3(target_xy.x(), target_xy.y(), 0.0));
  IKOptions o;
  o.task_weights << 0, 0, 0, 1, 1, 0;
  o.method = IKMethod::DampedLeastSquares;
  const VecX seed = Vec3(-1.2, 0.3, 0.2);
  // Record the iterates of the single-start solve one iteration at a time.
  Json single = Json::array();
  VecX q = seed;
  IKResult last;
  for (int it = 0; it < 60; ++it) {
    single.push(apps::robotFrame(model, q, it));
    IKOptions one = o;
    one.max_iterations = 1;
    last = solveIK(model, target, q, one);
    q = last.q;
  }
  const IKResult single_result = solveIK(model, target, seed, o);
  MultiStartIKOptions ms;
  ms.ik = o;
  ms.attempts = 20;
  ms.seed = 2;
  const auto multi = solveIKMultiStart(model, target, seed, ms);
  Json doc = Json::object();
  doc.set("model", planarModelInfo(model))
      .set("target", apps::vec(target_xy))
      .set("single_start", Json::object()
                               .set("frames", single)
                               .set("status", toString(single_result.status))
                               .set("position_error", single_result.position_error))
      .set("multi_start", multi.empty() ? Json() : apps::robotFrame(model, multi.front().q));
  apps::writeJson(out, "ik_local_minimum", doc);
  std::printf("  ik_local_minimum: single start %s (error %.3f m), multi-start %zu solutions\n",
              toString(single_result.status), single_result.position_error, multi.size());
}

}  // namespace

std::vector<Demo> kinematicsDemos() {
  return {
      {"fk_ik_2r", "Planar 2R forward kinematics and both closed-form IK branches", fkIk2R},
      {"singularity", "Resolved-rate motion into a singularity: pseudoinverse vs damped least squares", singularity},
      {"workspace", "Sampled workspaces coloured by manipulability", workspace},
      {"cartesian_paths", "6R end-effector line, circle and arc with geodesic orientation", cartesianPaths},
      {"collision", "Per-link clearances and closest points while sweeping past obstacles", collisionClearance},
      {"ik_local_minimum", "Local IK failure from a bad seed versus multi-start", localIkFailure}};
}

}  // namespace mc::demos
