#include "manipulatorcore/dynamics/planar_2r.hpp"

#include <cmath>

namespace mc {

Planar2RDynamicsTerms planar2RLagrangianDynamics(const Planar2RParameters& p, const Vec2& q, const Vec2& qd,
                                                 double gravity) {
  const double lc1 = 0.5 * p.l1;
  const double lc2 = 0.5 * p.l2;
  const double r2 = p.link_radius * p.link_radius;
  const double I1 = p.m1 * (3.0 * r2 + p.l1 * p.l1) / 12.0;  // about the COM, perpendicular to the link
  const double I2 = p.m2 * (3.0 * r2 + p.l2 * p.l2) / 12.0;
  const double c2 = std::cos(q(1));
  const double s2 = std::sin(q(1));

  Planar2RDynamicsTerms out;
  const double d11 = p.m1 * lc1 * lc1 + p.m2 * (p.l1 * p.l1 + lc2 * lc2 + 2.0 * p.l1 * lc2 * c2) + I1 + I2;
  const double d12 = p.m2 * (lc2 * lc2 + p.l1 * lc2 * c2) + I2;
  const double d22 = p.m2 * lc2 * lc2 + I2;
  out.M << d11, d12, d12, d22;

  const double h = -p.m2 * p.l1 * lc2 * s2;
  out.C << h * qd(1), h * (qd(0) + qd(1)),  //
      -h * qd(0), 0.0;

  const double c1 = std::cos(q(0));
  const double c12 = std::cos(q(0) + q(1));
  out.g << (p.m1 * lc1 + p.m2 * p.l1) * gravity * c1 + p.m2 * lc2 * gravity * c12,  //
      p.m2 * lc2 * gravity * c12;
  return out;
}

}  // namespace mc
