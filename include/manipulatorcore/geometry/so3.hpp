#pragma once

/// @file so3.hpp
/// @brief The rotation group SO(3): hat/vee, exponential and logarithm maps, and Jacobians.
///
/// Rotations are active and right-handed. An element of the Lie algebra so(3) is written as a
/// rotation vector phi = omega * theta (unit axis times angle).

#include "manipulatorcore/math/types.hpp"

namespace mc {

/// Skew-symmetric matrix [w] such that [w] v = w x v.
[[nodiscard]] Mat3 hat(const Vec3& w);

/// Inverse of hat(). Uses the antisymmetric part, so slightly non-skew input is projected.
[[nodiscard]] Vec3 vee(const Mat3& W);

/// Exponential map so(3) -> SO(3) (Rodrigues' formula). Uses a Taylor expansion for small angles.
[[nodiscard]] Mat3 expSO3(const Vec3& phi);

/// Logarithm map SO(3) -> so(3). Returns phi = omega * theta with theta in [0, pi].
///
/// Robust near theta = 0 (series expansion) and theta = pi (axis recovered from the symmetric
/// part of R). At exactly theta = pi both +axis and -axis are valid; one is returned.
[[nodiscard]] Vec3 logSO3(const Mat3& R);

/// Left Jacobian of SO(3): J_l(phi) = sum_k [phi]^k / (k+1)!. Used by the SE(3) exponential.
[[nodiscard]] Mat3 leftJacobianSO3(const Vec3& phi);

/// Inverse of the left Jacobian, valid for |phi| < 2 pi.
[[nodiscard]] Mat3 leftJacobianInverseSO3(const Vec3& phi);

/// Elementary rotation about the x axis.
[[nodiscard]] Mat3 rotX(double angle);
/// Elementary rotation about the y axis.
[[nodiscard]] Mat3 rotY(double angle);
/// Elementary rotation about the z axis.
[[nodiscard]] Mat3 rotZ(double angle);

/// True if R is orthonormal with determinant +1 within @p tolerance.
[[nodiscard]] bool isRotationMatrix(const Mat3& R, double tolerance = 1e-9);

/// Nearest rotation matrix in the Frobenius norm (SVD-based orthogonalization).
[[nodiscard]] Mat3 projectToSO3(const Mat3& M);

/// Geodesic angle between two rotations, |log(R_a^T R_b)| in [0, pi].
[[nodiscard]] double rotationDistance(const Mat3& R_a, const Mat3& R_b);

}  // namespace mc
