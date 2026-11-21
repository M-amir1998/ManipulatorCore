#pragma once

#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <vector>

namespace mc::test {

/// All built-in robots, for property tests that must hold on every model.
inline std::vector<RobotModel> allBuiltinRobots() {
  return {makePlanar2R(), makePlanar3R(), makeIndustrial6R(), makeRedundant7R()};
}

/// A 3-joint chain mixing revolute and prismatic joints with non-trivial joint origins, used to
/// exercise code paths the built-in (all-revolute) robots do not cover.
inline RobotModel makeMixedChain() {
  std::vector<Body> bodies(3);
  bodies[0].name = "rot_base";
  bodies[0].joint.name = "j1";
  bodies[0].joint.origin = SE3::fromTranslation(Vec3(0.0, 0.0, 0.3));
  bodies[0].joint.axis = Vec3::UnitZ();
  bodies[0].inertia = Inertia::solidCylinder(3.0, 0.05, Vec3::Zero(), Vec3(0.0, 0.0, 0.2));
  bodies[1].name = "slider";
  bodies[1].joint.name = "j2";
  bodies[1].joint.type = JointType::Prismatic;
  bodies[1].joint.origin = SE3(Eigen::AngleAxisd(0.4, Vec3::UnitX()).toRotationMatrix(), Vec3(0.1, 0.0, 0.2));
  bodies[1].joint.axis = Vec3(1.0, 1.0, 0.0).normalized();
  bodies[1].joint.limits = {-0.3, 0.5, 1.0, 5.0, 200.0};
  bodies[1].inertia = Inertia::solidBox(2.0, Vec3(0.1, 0.2, 0.3), Vec3(0.05, 0.0, 0.0));
  bodies[2].name = "wrist";
  bodies[2].joint.name = "j3";
  bodies[2].joint.origin = SE3(Eigen::AngleAxisd(-0.7, Vec3(0.0, 1.0, 1.0).normalized()).toRotationMatrix(),
                               Vec3(0.0, 0.25, 0.1));
  bodies[2].joint.axis = Vec3(0.3, -0.2, 1.0).normalized();
  bodies[2].joint.armature = 0.02;
  bodies[2].inertia = Inertia::solidSphere(1.0, 0.05, Vec3(0.0, 0.0, 0.1));
  return RobotModel("MixedChain", std::move(bodies), SE3::fromTranslation(Vec3(0.0, 0.0, 0.15)));
}

/// Uniform random configuration inside the joint limits.
inline VecX randomConfiguration(const RobotModel& model, Random& rng) {
  return rng.uniformVector(model.lowerLimits(), model.upperLimits());
}

/// Uniform random vector with entries in [-scale, scale].
inline VecX randomVector(Index n, Random& rng, double scale = 1.0) {
  VecX v(n);
  for (Index i = 0; i < n; ++i) {
    v(i) = rng.uniform(-scale, scale);
  }
  return v;
}

}  // namespace mc::test
