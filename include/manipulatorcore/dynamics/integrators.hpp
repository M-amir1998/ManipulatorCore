#pragma once

/// @file integrators.hpp
/// @brief Fixed-step integrators for second-order systems qdd = f(t, q, qd).
///
///  - ExplicitEuler:      first order; energy grows for oscillatory systems (reference only)
///  - SemiImplicitEuler:  first order, symplectic; energy error stays bounded for conservative
///                        systems (velocity updated first, then position with the new velocity)
///  - RK4:                classical fourth-order Runge-Kutta; small dissipative energy drift
///
/// All three are explicit and therefore conditionally stable: stiff controllers or contacts need
/// a time step well below the fastest closed-loop time constant.

#include "manipulatorcore/math/types.hpp"

#include <functional>

namespace mc {

enum class Integrator { ExplicitEuler, SemiImplicitEuler, RK4 };

[[nodiscard]] const char* toString(Integrator integrator);

struct JointState {
  VecX q;
  VecX qd;
};

/// qdd = f(t, q, qd)
using AccelerationFunction = std::function<VecX(double t, const VecX& q, const VecX& qd)>;

/// Advances (q, qd) from time t by one step dt.
[[nodiscard]] JointState integrateStep(Integrator integrator, const AccelerationFunction& f, double t,
                                       const JointState& state, double dt);

}  // namespace mc
