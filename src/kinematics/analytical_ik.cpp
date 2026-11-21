#include "manipulatorcore/kinematics/analytical_ik.hpp"

#include "manipulatorcore/math/angles.hpp"
#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

std::vector<Vec2> planar2RInverseKinematics(double l1, double l2, const Vec2& target, double boundary_tolerance) {
  require(l1 > 0.0 && l2 > 0.0, "planar2RInverseKinematics: link lengths must be positive");
  requireFinite(target, "planar2RInverseKinematics: target");
  const double r_sq = target.squaredNorm();
  // Law of cosines for the elbow angle.
  double c2 = (r_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
  if (c2 > 1.0 + boundary_tolerance || c2 < -1.0 - boundary_tolerance) {
    return {};  // outside the annulus |l1 - l2| <= r <= l1 + l2
  }
  c2 = std::clamp(c2, -1.0, 1.0);
  const double s2_abs = std::sqrt(std::max(0.0, 1.0 - c2 * c2));
  auto solve = [&](double s2) {
    const double q2 = std::atan2(s2, c2);
    const double q1 = std::atan2(target.y(), target.x()) - std::atan2(l2 * s2, l1 + l2 * c2);
    return Vec2(wrapToPi(q1), wrapToPi(q2));
  };
  if (s2_abs <= boundary_tolerance) {
    return {solve(0.0)};  // fully stretched or folded: a single (double) solution
  }
  return {solve(s2_abs), solve(-s2_abs)};
}

std::vector<Vec3> planar3RInverseKinematics(double l1, double l2, double l3, const Vec2& target, double phi,
                                            double boundary_tolerance) {
  require(l3 > 0.0, "planar3RInverseKinematics: link lengths must be positive");
  requireFinite(phi, "planar3RInverseKinematics: phi");
  const Vec2 wrist = target - l3 * Vec2(std::cos(phi), std::sin(phi));
  std::vector<Vec3> out;
  for (const Vec2& q12 : planar2RInverseKinematics(l1, l2, wrist, boundary_tolerance)) {
    out.emplace_back(q12.x(), q12.y(), wrapToPi(phi - q12.x() - q12.y()));
  }
  return out;
}

std::vector<double> planarLinkLengths(const RobotModel& model) {
  const char* msg = "planarLinkLengths: model is not a planar revolute chain about +z with links along +x";
  std::vector<double> lengths;
  for (int i = 0; i < model.dof(); ++i) {
    const Joint& j = model.joint(i);
    require(j.type == JointType::Revolute && (j.axis - Vec3::UnitZ()).norm() < 1e-12, msg);
    require((j.origin.rotation() - Mat3::Identity()).cwiseAbs().maxCoeff() < 1e-12, msg);
    const Vec3& p = j.origin.translation();
    require(std::abs(p.y()) < 1e-12 && std::abs(p.z()) < 1e-12 && p.x() >= 0.0, msg);
    if (i == 0) {
      require(p.x() < 1e-12, msg);
    } else {
      lengths.push_back(p.x());
    }
  }
  const SE3& tool = model.toolTransform();
  require((tool.rotation() - Mat3::Identity()).cwiseAbs().maxCoeff() < 1e-12 &&
              std::abs(tool.translation().y()) < 1e-12 && std::abs(tool.translation().z()) < 1e-12,
          msg);
  lengths.push_back(tool.translation().x());
  return lengths;
}

std::vector<VecX> analyticalIKPlanar2R(const RobotModel& model, const Vec2& target, bool respect_limits) {
  require(model.dof() == 2, "analyticalIKPlanar2R: model must have 2 joints");
  const std::vector<double> l = planarLinkLengths(model);
  std::vector<VecX> out;
  for (const Vec2& s : planar2RInverseKinematics(l[0], l[1], target)) {
    const VecX q = s;
    if (!respect_limits || model.withinLimits(q, 1e-12)) {
      out.push_back(q);
    }
  }
  return out;
}

std::vector<VecX> analyticalIKPlanar3R(const RobotModel& model, const Vec2& target, double phi, bool respect_limits) {
  require(model.dof() == 3, "analyticalIKPlanar3R: model must have 3 joints");
  const std::vector<double> l = planarLinkLengths(model);
  std::vector<VecX> out;
  for (const Vec3& s : planar3RInverseKinematics(l[0], l[1], l[2], target, phi)) {
    const VecX q = s;
    if (!respect_limits || model.withinLimits(q, 1e-12)) {
      out.push_back(q);
    }
  }
  return out;
}

}  // namespace mc
