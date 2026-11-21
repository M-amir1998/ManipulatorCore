#include "manipulatorcore/kinematics/jacobian.hpp"

#include "manipulatorcore/math/error.hpp"

namespace mc {

namespace {

bool isRevolute(const RobotModel& model, int i) { return model.joint(i).type == JointType::Revolute; }

}  // namespace

Mat6X spaceJacobian(const RobotModel& model, const KinematicState& state) {
  const int n = model.dof();
  Mat6X J(6, n);
  for (int i = 0; i < n; ++i) {
    const Vec3 a = state.jointAxis(model, i);
    if (isRevolute(model, i)) {
      // S = [a; -a x p] for any point p on the joint axis.
      J.col(i) << a, state.jointOrigin(i).cross(a);
    } else {
      J.col(i) << Vec3::Zero(), a;
    }
  }
  return J;
}

Mat6X spaceJacobian(const RobotModel& model, const VecX& q) { return spaceJacobian(model, forwardKinematics(model, q)); }

Mat6X bodyJacobian(const RobotModel& model, const KinematicState& state) {
  return state.T_base_ee.inverse().adjoint() * spaceJacobian(model, state);
}

Mat6X bodyJacobian(const RobotModel& model, const VecX& q) { return bodyJacobian(model, forwardKinematics(model, q)); }

Mat6X geometricJacobian(const RobotModel& model, const KinematicState& state) {
  const int n = model.dof();
  const Vec3 p_ee = state.T_base_ee.translation();
  Mat6X J(6, n);
  for (int i = 0; i < n; ++i) {
    const Vec3 a = state.jointAxis(model, i);
    if (isRevolute(model, i)) {
      J.col(i) << a, a.cross(p_ee - state.jointOrigin(i));
    } else {
      J.col(i) << Vec3::Zero(), a;
    }
  }
  return J;
}

Mat6X geometricJacobian(const RobotModel& model, const VecX& q) {
  return geometricJacobian(model, forwardKinematics(model, q));
}

Mat6X pointJacobian(const RobotModel& model, const KinematicState& state, int body, const Vec3& point_in_body) {
  require(body >= 0 && body < model.dof(), "pointJacobian: body index out of range");
  const Vec3 p = state.T_base_body[static_cast<std::size_t>(body)] * point_in_body;
  Mat6X J = Mat6X::Zero(6, model.dof());
  for (int i = 0; i <= body; ++i) {
    const Vec3 a = state.jointAxis(model, i);
    if (isRevolute(model, i)) {
      J.col(i) << a, a.cross(p - state.jointOrigin(i));
    } else {
      J.col(i) << Vec3::Zero(), a;
    }
  }
  return J;
}

Vec6 geometricJacobianDotTimesVelocity(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(qd, "geometricJacobianDotTimesVelocity: qd");
  const KinematicState state = forwardKinematics(model, q);
  const int n = model.dof();
  std::vector<Vec3> axes(static_cast<std::size_t>(n));
  std::vector<Vec3> origins(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    axes[static_cast<std::size_t>(i)] = state.jointAxis(model, i);
    origins[static_cast<std::size_t>(i)] = state.jointOrigin(i);
  }
  // Velocity of point p rigidly attached to body `last` (joints 0..last move it).
  auto pointVelocity = [&](const Vec3& p, int last) {
    Vec3 v = Vec3::Zero();
    for (int k = 0; k <= last; ++k) {
      const Vec3& a = axes[static_cast<std::size_t>(k)];
      v += isRevolute(model, k) ? Vec3(a.cross(p - origins[static_cast<std::size_t>(k)]) * qd(k)) : Vec3(a * qd(k));
    }
    return v;
  };
  const Vec3 p_ee = state.T_base_ee.translation();
  const Vec3 v_ee = pointVelocity(p_ee, n - 1);

  Vec6 bias = Vec6::Zero();
  Vec3 omega_parent = Vec3::Zero();  // angular velocity of the body carrying joint i's axis
  for (int i = 0; i < n; ++i) {
    const Vec3& a = axes[static_cast<std::size_t>(i)];
    const Vec3 a_dot = omega_parent.cross(a);
    if (isRevolute(model, i)) {
      const Vec3& p_i = origins[static_cast<std::size_t>(i)];
      const Vec3 p_i_dot = pointVelocity(p_i, i - 1);
      bias.head<3>() += a_dot * qd(i);
      bias.tail<3>() += (a_dot.cross(p_ee - p_i) + a.cross(v_ee - p_i_dot)) * qd(i);
      omega_parent += a * qd(i);
    } else {
      bias.tail<3>() += a_dot * qd(i);
    }
  }
  return bias;
}

MatX planarTaskJacobian(const Mat6X& geometric_jacobian) {
  MatX J(3, geometric_jacobian.cols());
  J.row(0) = geometric_jacobian.row(3);
  J.row(1) = geometric_jacobian.row(4);
  J.row(2) = geometric_jacobian.row(2);
  return J;
}

}  // namespace mc
