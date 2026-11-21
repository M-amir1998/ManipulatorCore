#include "manipulatorcore/model/builtin_robots.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <array>
#include <utility>

namespace mc {

namespace {

constexpr double deg(double degrees) { return degrees * kPi / 180.0; }

Joint revolute(std::string name, const SE3& origin, const Vec3& axis, const JointLimits& limits, double damping,
               double armature) {
  Joint j;
  j.name = std::move(name);
  j.type = JointType::Revolute;
  j.origin = origin;
  j.axis = axis.normalized();
  j.limits = limits;
  j.damping = damping;
  j.armature = armature;
  return j;
}

/// Uniform link made of straight cylindrical segments through the given points (body frame);
/// mass is distributed proportionally to segment length. Returns inertia and capsule geometry.
std::pair<Inertia, std::vector<Shape>> segmentedLink(const std::string& name, double mass, double radius,
                                                     const std::vector<Vec3>& points) {
  double total_length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total_length += (points[i] - points[i - 1]).norm();
  }
  Inertia inertia;
  std::vector<Shape> shapes;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double length = (points[i] - points[i - 1]).norm();
    const double m = total_length > 0.0 ? mass * length / total_length : mass;
    inertia = Inertia::combine(inertia, Inertia::solidCylinder(m, radius, points[i - 1], points[i]));
    shapes.push_back(makeCapsule(points[i - 1], points[i], radius, name + "_" + std::to_string(i - 1)));
  }
  return {inertia, shapes};
}

RobotModel makePlanarChain(const std::string& name, const std::vector<double>& lengths,
                           const std::vector<double>& masses, double radius) {
  std::vector<Body> bodies;
  JointLimits limits;
  limits.lower = -kPi;
  limits.upper = kPi;
  limits.velocity = 3.0;
  limits.acceleration = 15.0;
  limits.effort = 150.0;
  for (std::size_t i = 0; i < lengths.size(); ++i) {
    require(lengths[i] > 0.0 && masses[i] > 0.0, name + ": link lengths and masses must be positive");
    Body b;
    b.name = "link" + std::to_string(i + 1);
    const SE3 origin = i == 0 ? SE3::Identity() : SE3::fromTranslation(Vec3(lengths[i - 1], 0.0, 0.0));
    b.joint = revolute("joint" + std::to_string(i + 1), origin, Vec3::UnitZ(), limits, 0.0, 0.0);
    auto [inertia, shapes] = segmentedLink(b.name, masses[i], radius, {Vec3::Zero(), Vec3(lengths[i], 0.0, 0.0)});
    b.inertia = inertia;
    b.collision = std::move(shapes);
    bodies.push_back(std::move(b));
  }
  return RobotModel(name, std::move(bodies), SE3::fromTranslation(Vec3(lengths.back(), 0.0, 0.0)),
                    Vec3(0.0, -kStandardGravity, 0.0));
}

}  // namespace

RobotModel makePlanar2R(const Planar2RParameters& p) {
  require(p.link_radius > 0.0, "makePlanar2R: link radius must be positive");
  return makePlanarChain("Planar2R", {p.l1, p.l2}, {p.m1, p.m2}, p.link_radius);
}

RobotModel makePlanar3R(const Planar3RParameters& p) {
  require(p.link_radius > 0.0, "makePlanar3R: link radius must be positive");
  return makePlanarChain("Planar3R", {p.l1, p.l2, p.l3}, {p.m1, p.m2, p.m3}, p.link_radius);
}

DHTable industrial6RDHTable() {
  // Generic elbow manipulator with a spherical wrist. Not modelled on any specific product.
  //            a      alpha      d      theta    limits [lower, upper, velocity, accel, effort]
  DHTable table;
  table.convention = DHConvention::Standard;
  table.rows = {
      {0.15, -kHalfPi, 0.45, 0.0, JointType::Revolute, {deg(-170), deg(170), 2.5, 8.0, 400.0}, "link1"},
      {0.60, 0.0, 0.0, -kHalfPi, JointType::Revolute, {deg(-110), deg(110), 2.5, 8.0, 400.0}, "link2"},
      {0.10, -kHalfPi, 0.0, 0.0, JointType::Revolute, {deg(-140), deg(80), 3.0, 10.0, 200.0}, "link3"},
      {0.0, kHalfPi, 0.62, 0.0, JointType::Revolute, {deg(-175), deg(175), 4.0, 15.0, 60.0}, "link4"},
      {0.0, -kHalfPi, 0.0, 0.0, JointType::Revolute, {deg(-120), deg(120), 4.0, 15.0, 60.0}, "link5"},
      {0.0, 0.0, 0.10, 0.0, JointType::Revolute, {deg(-175), deg(175), 5.0, 20.0, 30.0}, "link6"},
  };
  table.tool = SE3::fromTranslation(Vec3(0.0, 0.0, 0.05));
  return table;
}

RobotModel makeIndustrial6R() {
  const DHTable table = industrial6RDHTable();
  RobotModel model = robotFromDH(table, "Industrial6R");

  // Link geometry in RobotModel body frames. For standard DH, the next joint origin in body i is
  // Tx(a_i) Rx(alpha_i) * (0, 0, d_{i+1}).
  struct LinkSpec {
    double mass;
    double radius;
    std::vector<Vec3> points;
  };
  const std::array<LinkSpec, 6> specs = {{
      {20.0, 0.10, {Vec3(0.0, 0.0, -0.12), Vec3(0.0, 0.0, 0.0), Vec3(0.15, 0.0, 0.0)}},
      {15.0, 0.08, {Vec3(0.0, 0.0, 0.0), Vec3(0.60, 0.0, 0.0)}},
      {10.0, 0.065, {Vec3(0.0, 0.0, 0.0), Vec3(0.10, 0.0, 0.0), Vec3(0.10, 0.50, 0.0)}},
      {3.0, 0.055, {Vec3(0.0, 0.0, -0.12), Vec3(0.0, 0.0, 0.0)}},
      {1.5, 0.05, {Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.08, 0.0)}},
      {0.8, 0.035, {Vec3(0.0, 0.0, -0.02), Vec3(0.0, 0.0, 0.05)}},
  }};
  const std::array<double, 6> armature = {0.5, 0.5, 0.3, 0.1, 0.1, 0.05};
  const std::array<double, 6> damping = {1.0, 1.0, 0.8, 0.3, 0.3, 0.1};

  std::vector<Body> bodies = model.bodies();
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    auto [inertia, shapes] = segmentedLink(bodies[i].name, specs[i].mass, specs[i].radius, specs[i].points);
    bodies[i].inertia = inertia;
    bodies[i].collision = std::move(shapes);
    bodies[i].joint.armature = armature[i];
    bodies[i].joint.damping = damping[i];
  }
  RobotModel out("Industrial6R", std::move(bodies), model.toolTransform(), model.gravity());
  out.setBaseCollision({makeCapsule(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.30), 0.13, "pedestal")});
  return out;
}

VecX industrial6RHome() {
  VecX q(6);
  q << 0.0, 0.25, -0.45, 0.0, -1.2, 0.0;
  return q;
}

RobotModel makeRedundant7R() {
  // Anthropomorphic S-R-S arm. Shoulder axes (1-3) and wrist axes (5-7) intersect in a point.
  struct Spec {
    double offset;  // z offset of this joint from the previous one [m]
    Vec3 axis;
    JointLimits limits;
    double mass;
    double radius;
    double length;  // capsule length of the body along its z axis [m]
    double armature;
    double damping;
  };
  const std::array<Spec, 7> specs = {{
      {0.15, Vec3::UnitZ(), {deg(-170), deg(170), 2.0, 10.0, 180.0}, 4.0, 0.07, 0.21, 0.20, 0.5},
      {0.21, Vec3::UnitY(), {deg(-120), deg(120), 2.0, 10.0, 180.0}, 4.0, 0.07, 0.21, 0.20, 0.5},
      {0.21, Vec3::UnitZ(), {deg(-170), deg(170), 2.2, 12.0, 110.0}, 3.0, 0.06, 0.21, 0.15, 0.4},
      {0.21, Vec3::UnitY(), {deg(-120), deg(120), 2.2, 12.0, 110.0}, 2.7, 0.06, 0.20, 0.15, 0.4},
      {0.20, Vec3::UnitZ(), {deg(-170), deg(170), 2.6, 15.0, 110.0}, 1.7, 0.05, 0.20, 0.08, 0.2},
      {0.20, Vec3::UnitY(), {deg(-120), deg(120), 3.0, 18.0, 40.0}, 1.8, 0.05, 0.08, 0.08, 0.2},
      {0.08, Vec3::UnitZ(), {deg(-175), deg(175), 3.0, 20.0, 40.0}, 0.7, 0.04, 0.10, 0.04, 0.1},
  }};
  const std::array<const char*, 7> names = {"shoulder_yaw_link", "shoulder_pitch_link", "upper_arm_link", "elbow_link",
                                            "forearm_link",      "wrist_pitch_link",    "flange_link"};
  std::vector<Body> bodies;
  for (std::size_t i = 0; i < specs.size(); ++i) {
    const Spec& s = specs[i];
    Body b;
    b.name = names[i];
    b.joint = revolute("joint" + std::to_string(i + 1), SE3::fromTranslation(Vec3(0.0, 0.0, s.offset)), s.axis,
                       s.limits, s.damping, s.armature);
    auto [inertia, shapes] = segmentedLink(b.name, s.mass, s.radius, {Vec3::Zero(), Vec3(0.0, 0.0, s.length)});
    b.inertia = inertia;
    b.collision = std::move(shapes);
    bodies.push_back(std::move(b));
  }
  RobotModel model("Redundant7R", std::move(bodies), SE3::fromTranslation(Vec3(0.0, 0.0, 0.10)));
  model.setBaseCollision({makeCapsule(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.12), 0.10, "base")});
  return model;
}

VecX redundant7RHome() {
  VecX q(7);
  q << 0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0;
  return q;
}

}  // namespace mc
