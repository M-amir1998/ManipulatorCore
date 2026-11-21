#include "manipulatorcore/dynamics/integrators.hpp"

#include "manipulatorcore/math/error.hpp"

#include <cmath>

namespace mc {

const char* toString(Integrator integrator) {
  switch (integrator) {
    case Integrator::ExplicitEuler:
      return "explicit-euler";
    case Integrator::SemiImplicitEuler:
      return "semi-implicit-euler";
    case Integrator::RK4:
      return "rk4";
  }
  return "unknown";
}

JointState integrateStep(Integrator integrator, const AccelerationFunction& f, double t, const JointState& s,
                         double dt) {
  require(std::isfinite(dt) && dt > 0.0, "integrateStep: dt must be positive");
  require(s.q.size() == s.qd.size(), "integrateStep: q and qd sizes differ");
  switch (integrator) {
    case Integrator::ExplicitEuler: {
      const VecX qdd = f(t, s.q, s.qd);
      return {s.q + dt * s.qd, s.qd + dt * qdd};
    }
    case Integrator::SemiImplicitEuler: {
      const VecX qd_next = s.qd + dt * f(t, s.q, s.qd);
      return {s.q + dt * qd_next, qd_next};
    }
    case Integrator::RK4: {
      const double h2 = 0.5 * dt;
      const VecX k1_q = s.qd;
      const VecX k1_v = f(t, s.q, s.qd);
      const VecX k2_q = s.qd + h2 * k1_v;
      const VecX k2_v = f(t + h2, s.q + h2 * k1_q, k2_q);
      const VecX k3_q = s.qd + h2 * k2_v;
      const VecX k3_v = f(t + h2, s.q + h2 * k2_q, k3_q);
      const VecX k4_q = s.qd + dt * k3_v;
      const VecX k4_v = f(t + dt, s.q + dt * k3_q, k4_q);
      return {s.q + dt / 6.0 * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q),
              s.qd + dt / 6.0 * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)};
    }
  }
  throw InvalidArgument("integrateStep: unknown integrator");
}

}  // namespace mc
