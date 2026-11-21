#include "manipulatorcore/model/robot_model.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <utility>

namespace mc {

RobotModel::RobotModel(std::string name, std::vector<Body> bodies, const SE3& T_flange_ee, const Vec3& gravity)
    : name_(std::move(name)), bodies_(std::move(bodies)), T_flange_ee_(T_flange_ee), gravity_(gravity) {
  validate();
  refreshLimitCache();
}

const Body& RobotModel::body(int i) const {
  require(i >= 0 && i < dof(), "RobotModel::body: index out of range");
  return bodies_[static_cast<std::size_t>(i)];
}

int RobotModel::bodyIndex(std::string_view name) const {
  for (int i = 0; i < dof(); ++i) {
    if (bodies_[static_cast<std::size_t>(i)].name == name) {
      return i;
    }
  }
  return -1;
}

void RobotModel::setToolTransform(const SE3& T_flange_ee) {
  require(T_flange_ee.translation().allFinite() && isRotationMatrix(T_flange_ee.rotation(), 1e-6),
          "RobotModel::setToolTransform: invalid transform");
  T_flange_ee_ = T_flange_ee;
}

void RobotModel::setGravity(const Vec3& gravity) {
  requireFinite(gravity, "RobotModel::setGravity");
  gravity_ = gravity;
}

void RobotModel::setBaseCollision(std::vector<Shape> shapes) {
  for (const Shape& s : shapes) {
    validateShape(s);
  }
  base_collision_ = std::move(shapes);
}

void RobotModel::setBodyInertia(int i, const Inertia& inertia) {
  require(i >= 0 && i < dof(), "RobotModel::setBodyInertia: index out of range");
  inertia.validate();
  bodies_[static_cast<std::size_t>(i)].inertia = inertia;
}

void RobotModel::setBodyCollision(int i, std::vector<Shape> shapes) {
  require(i >= 0 && i < dof(), "RobotModel::setBodyCollision: index out of range");
  for (const Shape& s : shapes) {
    validateShape(s);
  }
  bodies_[static_cast<std::size_t>(i)].collision = std::move(shapes);
}

void RobotModel::setJointLimits(int i, const JointLimits& limits) {
  require(i >= 0 && i < dof(), "RobotModel::setJointLimits: index out of range");
  Joint candidate = bodies_[static_cast<std::size_t>(i)].joint;
  candidate.limits = limits;
  candidate.validate();
  bodies_[static_cast<std::size_t>(i)].joint.limits = limits;
  refreshLimitCache();
}

bool RobotModel::withinLimits(const VecX& q, double tolerance) const {
  if (q.size() != dof() || !q.allFinite()) {
    return false;
  }
  return ((q.array() >= lower_.array() - tolerance) && (q.array() <= upper_.array() + tolerance)).all();
}

VecX RobotModel::clampToLimits(const VecX& q) const {
  checkConfiguration(q, "RobotModel::clampToLimits");
  return q.cwiseMax(lower_).cwiseMin(upper_);
}

VecX RobotModel::midConfiguration() const { return 0.5 * (lower_ + upper_); }

double RobotModel::totalMass() const {
  double m = 0.0;
  for (const Body& b : bodies_) {
    m += b.inertia.mass;
  }
  return m;
}

void RobotModel::checkConfiguration(const VecX& q, const char* what) const { requireVector(q, dof(), what); }

void RobotModel::validate() const {
  require(!bodies_.empty(), "RobotModel '" + name_ + "': a model needs at least one joint");
  requireFinite(gravity_, "RobotModel gravity");
  require(T_flange_ee_.translation().allFinite() && isRotationMatrix(T_flange_ee_.rotation(), 1e-6),
          "RobotModel '" + name_ + "': invalid tool transform");
  for (const Body& b : bodies_) {
    b.joint.validate();
    b.inertia.validate();
    for (const Shape& s : b.collision) {
      validateShape(s);
    }
  }
  for (const Shape& s : base_collision_) {
    validateShape(s);
  }
}

void RobotModel::refreshLimitCache() {
  const int n = dof();
  lower_.resize(n);
  upper_.resize(n);
  velocity_.resize(n);
  acceleration_.resize(n);
  effort_.resize(n);
  for (int i = 0; i < n; ++i) {
    const JointLimits& l = bodies_[static_cast<std::size_t>(i)].joint.limits;
    lower_(i) = l.lower;
    upper_(i) = l.upper;
    velocity_(i) = l.velocity;
    acceleration_(i) = l.acceleration;
    effort_(i) = l.effort;
  }
}

}  // namespace mc
