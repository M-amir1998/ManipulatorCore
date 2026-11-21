#pragma once

/// @file inertia.hpp
/// @brief Rigid-body inertial parameters and 6x6 spatial inertia.

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/math/types.hpp"

namespace mc {

/// Mass, centre of mass and rotational inertia of a rigid body, written in the body frame.
struct Inertia {
  double mass = 0.0;                      ///< [kg]
  Vec3 com = Vec3::Zero();                ///< centre of mass in the body frame [m]
  Mat3 inertia_com = Mat3::Zero();        ///< rotational inertia about the COM, body axes [kg·m²]

  /// Point mass at @p com.
  [[nodiscard]] static Inertia pointMass(double mass, const Vec3& com);

  /// Solid cylinder whose axis runs from @p a to @p b (body frame).
  [[nodiscard]] static Inertia solidCylinder(double mass, double radius, const Vec3& a, const Vec3& b);

  /// Solid box with full side lengths @p size, centred at @p center, axes aligned with the body frame.
  [[nodiscard]] static Inertia solidBox(double mass, const Vec3& size, const Vec3& center);

  /// Solid sphere centred at @p center.
  [[nodiscard]] static Inertia solidSphere(double mass, double radius, const Vec3& center);

  /// Inertia of the union of two rigid bodies expressed in the same frame (parallel-axis theorem).
  [[nodiscard]] static Inertia combine(const Inertia& a, const Inertia& b);

  /// The same body expressed in another frame: returns the parameters in frame a, given
  /// T_a_body. (com_a = T_a_body * com, I_a = R I R^T.)
  [[nodiscard]] Inertia transformed(const SE3& T_a_body) const;

  /// Rotational inertia about the body-frame origin: I_c - m [c]^2.
  [[nodiscard]] Mat3 inertiaAboutOrigin() const;

  /// 6x6 spatial inertia about the body origin, ordered [omega; v]:
  ///   [ I_c - m[c]^2   m[c] ]
  ///   [ -m[c]          m I  ]
  [[nodiscard]] Mat6 spatialMatrix() const;

  /// Throws InvalidArgument for negative/non-finite mass, asymmetric inertia, or principal moments
  /// that are negative or violate the triangle inequality.
  void validate() const;
};

}  // namespace mc
