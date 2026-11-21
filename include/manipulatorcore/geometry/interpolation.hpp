#pragma once

/// @file interpolation.hpp
/// @brief Interpolation on SO(3) and SE(3).

#include "manipulatorcore/geometry/se3.hpp"

namespace mc {

/// Spherical linear interpolation along the shortest arc, s in [0, 1].
/// Falls back to normalized linear interpolation when the quaternions are nearly parallel.
[[nodiscard]] Quat slerp(const Quat& q0, const Quat& q1, double s);

/// Geodesic interpolation R(s) = R0 exp(s log(R0^T R1)).
[[nodiscard]] Mat3 interpolateRotation(const Mat3& R0, const Mat3& R1, double s);

/// Decoupled interpolation: straight-line translation and geodesic (SLERP-equivalent) rotation.
/// This is the usual choice for Cartesian straight-line motions.
[[nodiscard]] SE3 interpolatePoseDecoupled(const SE3& T0, const SE3& T1, double s);

/// Screw-motion interpolation T(s) = T0 exp(s log(T0^{-1} T1)). The origin follows a helix,
/// not a straight line, when the rotation is non-zero.
[[nodiscard]] SE3 interpolatePoseScrew(const SE3& T0, const SE3& T1, double s);

}  // namespace mc
