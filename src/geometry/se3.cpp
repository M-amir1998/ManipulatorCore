#include "manipulatorcore/geometry/se3.hpp"

#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <cmath>

namespace mc {

SE3 SE3::fromQuaternion(const Quat& q, const Vec3& translation) {
  return {rotationFromQuaternion(q), translation};
}

SE3 SE3::fromMatrix(const Mat4& T, double tolerance) {
  requireFinite(T, "SE3::fromMatrix");
  const Mat3 R = T.topLeftCorner<3, 3>();
  require(isRotationMatrix(R, tolerance), "SE3::fromMatrix: rotation block is not in SO(3)");
  const Eigen::RowVector4d last_row = T.row(3);
  require((last_row - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).cwiseAbs().maxCoeff() <= tolerance,
          "SE3::fromMatrix: last row must be [0 0 0 1]");
  return {R, T.topRightCorner<3, 1>()};
}

Quat SE3::quaternion() const { return quaternionFromRotation(R_); }

Mat4 SE3::matrix() const {
  Mat4 T = Mat4::Identity();
  T.topLeftCorner<3, 3>() = R_;
  T.topRightCorner<3, 1>() = p_;
  return T;
}

Mat6 SE3::adjoint() const {
  Mat6 Ad = Mat6::Zero();
  Ad.topLeftCorner<3, 3>() = R_;
  Ad.bottomRightCorner<3, 3>() = R_;
  Ad.bottomLeftCorner<3, 3>() = hat(p_) * R_;
  return Ad;
}

bool SE3::isApprox(const SE3& other, double tolerance) const {
  return (R_ - other.R_).cwiseAbs().maxCoeff() <= tolerance && (p_ - other.p_).cwiseAbs().maxCoeff() <= tolerance;
}

Mat4 hatSE3(const Vec6& twist) {
  Mat4 M = Mat4::Zero();
  M.topLeftCorner<3, 3>() = hat(twist.head<3>());
  M.topRightCorner<3, 1>() = twist.tail<3>();
  return M;
}

Vec6 veeSE3(const Mat4& twist_matrix) {
  Vec6 V;
  V.head<3>() = vee(twist_matrix.topLeftCorner<3, 3>());
  V.tail<3>() = twist_matrix.topRightCorner<3, 1>();
  return V;
}

SE3 expSE3(const Vec6& xi) {
  const Vec3 phi = xi.head<3>();
  return {expSO3(phi), leftJacobianSO3(phi) * xi.tail<3>()};
}

Vec6 logSE3(const SE3& T) {
  const Vec3 phi = logSO3(T.rotation());
  Vec6 xi;
  xi.head<3>() = phi;
  xi.tail<3>() = leftJacobianInverseSO3(phi) * T.translation();
  return xi;
}

Mat6 adjoint(const SE3& T) { return T.adjoint(); }

Mat6 adSE3(const Vec6& twist) {
  Mat6 ad = Mat6::Zero();
  const Mat3 W = hat(twist.head<3>());
  ad.topLeftCorner<3, 3>() = W;
  ad.bottomRightCorner<3, 3>() = W;
  ad.bottomLeftCorner<3, 3>() = hat(twist.tail<3>());
  return ad;
}

Vec6 transformTwist(const SE3& T_a_b, const Vec6& twist_b) {
  const Vec3 w = T_a_b.rotation() * twist_b.head<3>();
  Vec6 out;
  out.head<3>() = w;
  out.tail<3>() = T_a_b.translation().cross(w) + T_a_b.rotation() * twist_b.tail<3>();
  return out;
}

Vec6 transformWrench(const SE3& T_a_b, const Vec6& wrench_b) {
  const Vec3 f = T_a_b.rotation() * wrench_b.tail<3>();
  Vec6 out;
  out.head<3>() = T_a_b.rotation() * wrench_b.head<3>() + T_a_b.translation().cross(f);
  out.tail<3>() = f;
  return out;
}

PoseDistance poseDistance(const SE3& T_a, const SE3& T_b) {
  return {(T_a.translation() - T_b.translation()).norm(), rotationDistance(T_a.rotation(), T_b.rotation())};
}

}  // namespace mc
