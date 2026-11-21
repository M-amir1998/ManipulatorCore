#include "manipulatorcore/planning/joint_space.hpp"

#include "manipulatorcore/math/error.hpp"

#include <cmath>
#include <utility>

namespace mc {

JointSpace::JointSpace(VecX lower, VecX upper, VecX weights)
    : lower_(std::move(lower)), upper_(std::move(upper)), weights_(std::move(weights)) {
  require(lower_.size() > 0, "JointSpace: dimension must be positive");
  requireVector(upper_, lower_.size(), "JointSpace: upper bounds");
  requireFinite(lower_, "JointSpace: lower bounds");
  require((lower_.array() <= upper_.array()).all(), "JointSpace: lower bounds must not exceed upper bounds");
  if (weights_.size() == 0) {
    weights_ = VecX::Ones(lower_.size());
  }
  requireVector(weights_, lower_.size(), "JointSpace: weights");
  require((weights_.array() > 0.0).all(), "JointSpace: weights must be positive");
}

JointSpace JointSpace::fromModel(const RobotModel& model, VecX weights) {
  return {model.lowerLimits(), model.upperLimits(), std::move(weights)};
}

double JointSpace::distance(const VecX& a, const VecX& b) const {
  return std::sqrt((weights_.array() * (a - b).array().square()).sum());
}

VecX JointSpace::steer(const VecX& from, const VecX& to, double max_distance) const {
  const double d = distance(from, to);
  if (d <= max_distance) {
    return to;
  }
  return from + (max_distance / d) * (to - from);
}

bool JointSpace::contains(const VecX& q, double tolerance) const {
  return q.size() == lower_.size() && q.allFinite() && (q.array() >= lower_.array() - tolerance).all() &&
         (q.array() <= upper_.array() + tolerance).all();
}

double JointSpace::measure() const { return (weights_.array().sqrt() * (upper_ - lower_).array()).prod(); }

MotionValidator::MotionValidator(JointSpace space, StateValidityFn is_valid, double resolution)
    : space_(std::move(space)), is_valid_(std::move(is_valid)), resolution_(resolution) {
  require(static_cast<bool>(is_valid_), "MotionValidator: validity function must be set");
  require(std::isfinite(resolution) && resolution > 0.0, "MotionValidator: resolution must be positive");
}

bool MotionValidator::isValid(const VecX& q) const {
  ++state_checks_;
  return space_.contains(q) && is_valid_(q);
}

bool MotionValidator::isEdgeValid(const VecX& a, const VecX& b) const {
  ++edge_checks_;
  const double span = (b - a).cwiseAbs().maxCoeff();
  const auto steps = static_cast<int>(std::ceil(span / resolution_));
  if (steps <= 0) {
    return isValid(b);
  }
  for (int k = 1; k <= steps; ++k) {
    if (!isValid(space_.interpolate(a, b, static_cast<double>(k) / steps))) {
      return false;
    }
  }
  return true;
}

bool MotionValidator::isPathValid(const std::vector<VecX>& path) const {
  if (path.empty() || !isValid(path.front())) {
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!isEdgeValid(path[i - 1], path[i])) {
      return false;
    }
  }
  return true;
}

void MotionValidator::resetCounters() const {
  state_checks_ = 0;
  edge_checks_ = 0;
}

}  // namespace mc
