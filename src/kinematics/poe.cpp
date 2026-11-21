#include "manipulatorcore/kinematics/poe.hpp"

#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/error.hpp"

namespace mc {

ProductOfExponentials productOfExponentials(const RobotModel& model) {
  const KinematicState home = forwardKinematics(model, model.zeroConfiguration());
  ProductOfExponentials poe;
  poe.home = home.T_base_ee;
  const Mat6 Ad_M_inv = home.T_base_ee.inverse().adjoint();
  for (int i = 0; i < model.dof(); ++i) {
    const Vec3 a = home.jointAxis(model, i);
    Vec6 S;
    if (model.joint(i).type == JointType::Revolute) {
      S << a, home.jointOrigin(i).cross(a);  // [omega; -omega x r]
    } else {
      S << Vec3::Zero(), a;
    }
    poe.space_screws.push_back(S);
    poe.body_screws.emplace_back(Ad_M_inv * S);
  }
  return poe;
}

SE3 poeForwardSpace(const ProductOfExponentials& poe, const VecX& q) {
  requireVector(q, static_cast<Index>(poe.space_screws.size()), "poeForwardSpace: q");
  SE3 T;
  for (std::size_t i = 0; i < poe.space_screws.size(); ++i) {
    T = T * expSE3(poe.space_screws[i] * q(static_cast<Index>(i)));
  }
  return T * poe.home;
}

SE3 poeForwardBody(const ProductOfExponentials& poe, const VecX& q) {
  requireVector(q, static_cast<Index>(poe.body_screws.size()), "poeForwardBody: q");
  SE3 T = poe.home;
  for (std::size_t i = 0; i < poe.body_screws.size(); ++i) {
    T = T * expSE3(poe.body_screws[i] * q(static_cast<Index>(i)));
  }
  return T;
}

Mat6X poeSpaceJacobian(const ProductOfExponentials& poe, const VecX& q) {
  const auto n = static_cast<Index>(poe.space_screws.size());
  requireVector(q, n, "poeSpaceJacobian: q");
  Mat6X J(6, n);
  SE3 T;
  for (Index i = 0; i < n; ++i) {
    const Vec6& S = poe.space_screws[static_cast<std::size_t>(i)];
    J.col(i) = transformTwist(T, S);
    T = T * expSE3(S * q(i));
  }
  return J;
}

Mat6X poeBodyJacobian(const ProductOfExponentials& poe, const VecX& q) {
  const auto n = static_cast<Index>(poe.body_screws.size());
  requireVector(q, n, "poeBodyJacobian: q");
  Mat6X J(6, n);
  SE3 T;
  for (Index i = n - 1; i >= 0; --i) {
    const Vec6& B = poe.body_screws[static_cast<std::size_t>(i)];
    J.col(i) = transformTwist(T, B);
    T = T * expSE3(-B * q(i));
  }
  return J;
}

}  // namespace mc
