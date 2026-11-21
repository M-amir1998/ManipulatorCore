#pragma once

/// @file se3.hpp
/// @brief Rigid-body transforms SE(3), twists, wrenches and adjoint maps.
///
/// Notation: T_a_b is the pose of frame b expressed in frame a (p_a = T_a_b * p_b). Twists are
/// ordered [omega; v] and wrenches [moment; force]. See docs/mathematics/conventions.md.

#include "manipulatorcore/math/types.hpp"

namespace mc {

/// Homogeneous rigid-body transform stored as a rotation matrix and a translation.
class SE3 {
 public:
  /// Identity transform.
  SE3() : R_(Mat3::Identity()), p_(Vec3::Zero()) {}

  /// Transform from a rotation and translation. The rotation is not re-orthogonalized.
  SE3(const Mat3& rotation, const Vec3& translation) : R_(rotation), p_(translation) {}

  [[nodiscard]] static SE3 Identity() { return {}; }

  /// Pure translation.
  [[nodiscard]] static SE3 fromTranslation(const Vec3& translation) { return {Mat3::Identity(), translation}; }

  /// Pure rotation.
  [[nodiscard]] static SE3 fromRotation(const Mat3& rotation) { return {rotation, Vec3::Zero()}; }

  /// Rotation from a quaternion (normalized internally) and translation.
  [[nodiscard]] static SE3 fromQuaternion(const Quat& q, const Vec3& translation);

  /// Validates and converts a 4x4 homogeneous matrix. Throws InvalidArgument if the rotation block
  /// is not a rotation within @p tolerance or the last row is not [0 0 0 1].
  [[nodiscard]] static SE3 fromMatrix(const Mat4& T, double tolerance = 1e-9);

  [[nodiscard]] const Mat3& rotation() const { return R_; }
  [[nodiscard]] const Vec3& translation() const { return p_; }
  Mat3& rotation() { return R_; }
  Vec3& translation() { return p_; }

  /// Unit quaternion of the rotation (canonical, w >= 0).
  [[nodiscard]] Quat quaternion() const;

  /// 4x4 homogeneous matrix.
  [[nodiscard]] Mat4 matrix() const;

  /// Inverse transform: (R^T, -R^T p).
  [[nodiscard]] SE3 inverse() const { return {R_.transpose(), -(R_.transpose() * p_)}; }

  /// Composition T_a_c = T_a_b * T_b_c.
  [[nodiscard]] SE3 operator*(const SE3& other) const { return {R_ * other.R_, R_ * other.p_ + p_}; }

  /// Transforms a point: p_a = T_a_b * p_b.
  [[nodiscard]] Vec3 operator*(const Vec3& point) const { return R_ * point + p_; }

  /// Rotates a free vector (no translation).
  [[nodiscard]] Vec3 rotate(const Vec3& vector) const { return R_ * vector; }

  /// 6x6 adjoint Ad_T = [R 0; [p]R R] mapping twists from this frame's child to its parent.
  [[nodiscard]] Mat6 adjoint() const;

  /// True if rotation and translation agree within @p tolerance (max-abs).
  [[nodiscard]] bool isApprox(const SE3& other, double tolerance = 1e-9) const;

 private:
  Mat3 R_;
  Vec3 p_;
};

/// 4x4 matrix representation [V] of a twist V = [omega; v].
[[nodiscard]] Mat4 hatSE3(const Vec6& twist);

/// Inverse of hatSE3().
[[nodiscard]] Vec6 veeSE3(const Mat4& twist_matrix);

/// Exponential map se(3) -> SE(3). The argument is the scaled twist xi = [omega theta; v theta].
[[nodiscard]] SE3 expSE3(const Vec6& xi);

/// Logarithm map SE(3) -> se(3). Returns xi with rotation angle in [0, pi].
[[nodiscard]] Vec6 logSE3(const SE3& T);

/// Adjoint matrix Ad_T (same as T.adjoint()).
[[nodiscard]] Mat6 adjoint(const SE3& T);

/// Lie bracket matrix ad_V = [[omega] 0; [v] [omega]] so that ad_V1 V2 = [V1, V2].
[[nodiscard]] Mat6 adSE3(const Vec6& twist);

/// Expresses a twist given in frame b in frame a: V_a = Ad_{T_a_b} V_b.
[[nodiscard]] Vec6 transformTwist(const SE3& T_a_b, const Vec6& twist_b);

/// Expresses a wrench given in frame b in frame a: F_a = Ad_{T_a_b}^{-T} F_b.
[[nodiscard]] Vec6 transformWrench(const SE3& T_a_b, const Vec6& wrench_b);

/// Translational and rotational distance between two poses.
struct PoseDistance {
  double translation = 0.0;  ///< |p_a - p_b| [m]
  double rotation = 0.0;     ///< geodesic angle [rad]
};

/// Decoupled distance between two poses.
[[nodiscard]] PoseDistance poseDistance(const SE3& T_a, const SE3& T_b);

}  // namespace mc
