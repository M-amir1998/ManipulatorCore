#include "manipulatorcore/kinematics/forward_kinematics.hpp"

namespace mc {

Vec3 KinematicState::jointAxis(const RobotModel& model, int i) const {
  return T_base_body[static_cast<std::size_t>(i)].rotation() * model.joint(i).axis;
}

void computeForwardKinematics(const RobotModel& model, const VecX& q, KinematicState& state) {
  model.checkConfiguration(q, "forwardKinematics: q");
  const int n = model.dof();
  state.q = q;
  state.T_base_body.resize(static_cast<std::size_t>(n));
  SE3 T;
  for (int i = 0; i < n; ++i) {
    T = T * model.joint(i).transform(q(i));
    state.T_base_body[static_cast<std::size_t>(i)] = T;
  }
  state.T_base_ee = T * model.toolTransform();
}

KinematicState forwardKinematics(const RobotModel& model, const VecX& q) {
  KinematicState state;
  computeForwardKinematics(model, q, state);
  return state;
}

SE3 endEffectorPose(const RobotModel& model, const VecX& q) {
  model.checkConfiguration(q, "endEffectorPose: q");
  SE3 T;
  for (int i = 0; i < model.dof(); ++i) {
    T = T * model.joint(i).transform(q(i));
  }
  return T * model.toolTransform();
}

}  // namespace mc
