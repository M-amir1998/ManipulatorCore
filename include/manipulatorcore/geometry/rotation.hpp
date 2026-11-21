#pragma once

/// @file rotation.hpp
/// @brief Conversions between rotation matrices, quaternions, axis-angle and Euler angles.
///
/// Quaternions follow the Hamilton convention and are documented scalar-first (w, x, y, z).
/// Euler sequences are intrinsic: R = R_A(alpha) R_B(beta) R_C(gamma). See
/// docs/mathematics/conventions.md for ranges and gimbal-lock handling.

#include "manipulatorcore/math/types.hpp"

namespace mc {

/// Supported intrinsic Euler sequences.
enum class EulerSequence {
  ZYX,  ///< yaw-pitch-roll; identical to extrinsic XYZ roll-pitch-yaw
  XYZ,  ///< intrinsic x-y'-z''
  ZYZ   ///< proper Euler z-y'-z''
};

/// Unit axis and angle. The angle is in [0, pi] when produced by axisAngleFromRotation().
struct AxisAngle {
  Vec3 axis = Vec3::UnitX();
  double angle = 0.0;
};

/// Normalizes a quaternion. Throws InvalidArgument if its norm is below 1e-12 or not finite.
[[nodiscard]] Quat normalizeQuaternion(const Quat& q);

/// Returns the representative of q with non-negative scalar part (q and -q are the same rotation).
[[nodiscard]] Quat canonicalQuaternion(const Quat& q);

/// Rotation matrix of a quaternion (normalized internally).
[[nodiscard]] Mat3 rotationFromQuaternion(const Quat& q);

/// Unit quaternion of a rotation matrix (Shepperd's method), canonical with w >= 0.
[[nodiscard]] Quat quaternionFromRotation(const Mat3& R);

/// Rotation about @p axis (normalized internally) by @p angle. Throws if the axis is zero.
[[nodiscard]] Mat3 rotationFromAxisAngle(const Vec3& axis, double angle);

/// Axis-angle of a rotation with angle in [0, pi]. The axis is +x when the angle is zero.
[[nodiscard]] AxisAngle axisAngleFromRotation(const Mat3& R);

/// Rotation from intrinsic Euler angles (alpha, beta, gamma) for the given sequence.
[[nodiscard]] Mat3 rotationFromEuler(const Vec3& angles, EulerSequence sequence);

/// Intrinsic Euler angles (alpha, beta, gamma) of R. At gimbal lock gamma is set to zero and the
/// returned angles still reproduce R.
[[nodiscard]] Vec3 eulerFromRotation(const Mat3& R, EulerSequence sequence);

/// R = Rz(yaw) Ry(pitch) Rx(roll).
[[nodiscard]] Mat3 rotationFromRPY(double roll, double pitch, double yaw);

/// Returns (roll, pitch, yaw) such that rotationFromRPY(roll, pitch, yaw) == R.
[[nodiscard]] Vec3 rpyFromRotation(const Mat3& R);

}  // namespace mc
