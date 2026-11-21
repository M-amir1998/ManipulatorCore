#include "manipulatorcore/model/joint.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <cmath>

namespace mc {

SE3 Joint::motion(double q) const {
  if (type == JointType::Revolute) {
    return SE3::fromRotation(expSO3(axis * q));
  }
  return SE3::fromTranslation(axis * q);
}

SE3 Joint::transform(double q) const { return origin * motion(q); }

Vec6 Joint::motionSubspace() const {
  Vec6 S = Vec6::Zero();
  if (type == JointType::Revolute) {
    S.head<3>() = axis;
  } else {
    S.tail<3>() = axis;
  }
  return S;
}

void Joint::validate() const {
  const std::string label = "Joint '" + name + "'";
  require(axis.allFinite() && std::abs(axis.norm() - 1.0) < 1e-9, label + ": axis must be a unit vector");
  require(origin.translation().allFinite() && isRotationMatrix(origin.rotation(), 1e-6),
          label + ": origin must be a valid rigid transform");
  require(std::isfinite(limits.lower) && std::isfinite(limits.upper) && limits.lower <= limits.upper,
          label + ": position limits must be finite with lower <= upper");
  require(std::isfinite(limits.velocity) && limits.velocity > 0.0, label + ": velocity limit must be positive");
  require(std::isfinite(limits.acceleration) && limits.acceleration > 0.0,
          label + ": acceleration limit must be positive");
  require(std::isfinite(limits.effort) && limits.effort > 0.0, label + ": effort limit must be positive");
  require(std::isfinite(damping) && damping >= 0.0, label + ": damping must be non-negative");
  require(std::isfinite(armature) && armature >= 0.0, label + ": armature must be non-negative");
}

}  // namespace mc
