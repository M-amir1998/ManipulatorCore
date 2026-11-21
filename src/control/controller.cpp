#include "manipulatorcore/control/controller.hpp"

#include "manipulatorcore/math/error.hpp"

namespace mc {

JointReferenceFn constantJointReference(const VecX& q) {
  requireFinite(q, "constantJointReference: q");
  return [q](double) { return JointReference{q, VecX::Zero(q.size()), VecX::Zero(q.size())}; };
}

JointReferenceFn trajectoryReference(std::shared_ptr<const JointTrajectory> trajectory) {
  require(trajectory != nullptr, "trajectoryReference: null trajectory");
  return [trajectory](double t) {
    TrajectoryPoint p = trajectory->sample(t);
    return JointReference{std::move(p.q), std::move(p.qd), std::move(p.qdd)};
  };
}

CartesianReferenceFn constantPoseReference(const SE3& pose) {
  return [pose](double) { return CartesianReference{pose, Vec6::Zero(), Vec6::Zero()}; };
}

CartesianReferenceFn cartesianTrajectoryReference(std::shared_ptr<const CartesianTrajectory> trajectory) {
  require(trajectory != nullptr, "cartesianTrajectoryReference: null trajectory");
  return [trajectory](double t) {
    const CartesianSample s = trajectory->sample(t);
    return CartesianReference{s.pose, s.twist, s.acceleration};
  };
}

}  // namespace mc
