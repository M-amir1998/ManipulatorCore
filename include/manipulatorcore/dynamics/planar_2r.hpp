#pragma once

/// @file planar_2r.hpp
/// @brief Closed-form Lagrangian dynamics of the planar two-link arm, used as a test reference.
///
/// Derived with the Euler-Lagrange equations (M. W. Spong, S. Hutchinson and M. Vidyasagar, Robot
/// Modeling and Control, 2006, section 7.3). Gravity acts along -y in the arm plane.

#include "manipulatorcore/math/types.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

namespace mc {

struct Planar2RDynamicsTerms {
  Mat2 M;  ///< inertia matrix
  Mat2 C;  ///< Coriolis matrix (Christoffel form)
  Vec2 g;  ///< gravity torques
};

/// Dynamics of makePlanar2R(params): uniform solid cylinders with centres of mass at mid-link and
/// inertia m (3 r^2 + l^2) / 12 about the centre of mass.
[[nodiscard]] Planar2RDynamicsTerms planar2RLagrangianDynamics(const Planar2RParameters& params, const Vec2& q,
                                                               const Vec2& qd, double gravity = kStandardGravity);

}  // namespace mc
