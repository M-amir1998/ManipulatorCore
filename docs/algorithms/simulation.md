# Simulation

## Purpose

A lightweight, deterministic, CPU-only simulator for closed-loop studies of the controllers: joint
dynamics under gravity, actuator saturation, joint friction, joint-limit stops and simple compliant
contact. It is a tool for validating control laws, not a replacement for MuJoCo, Gazebo or Isaac Sim.

## Mathematical formulation

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \mathrm{sat}(\tau) - B\dot q + \tau_{lim}(q,\dot q) + J_c^\top F_c(q,\dot q)
$$

* $\mathrm{sat}$: clamp to the effort limits; $B$: viscous joint damping (`Joint::damping`).
* $\tau_{lim}$: spring-damper stops active only beyond the position limits.
* Contact (penalty model): a sphere of radius $r$ attached to a body touches planar half-spaces
  $\{x : n^\top x \le d\}$; with penetration $\delta = -(n^\top p - d - r) > 0$ and normal velocity
  $v_n$, $F_n = \max(0,\ k\delta - c\, v_n)$ (no adhesion), plus optional viscous tangential friction.
  $J_c$ is the linear part of the point Jacobian of the contact point.

## Assumptions

* Rigid links, ideal revolute/prismatic joints, a single contact point.
* The controller is sampled once per step (zero-order hold); contact, damping and limit forces are
  re-evaluated at every integrator stage.

## Conventions

Base-frame forces; the log stores the applied (saturated) torque and the contact force at the
beginning of each step.

## Algorithm

Each step: build `ControllerInput` (state and the contact wrench about the end-effector origin) →
call the controller → saturate → integrate one step with RK4 (default) or semi-implicit Euler, using
forward dynamics (ABA) on the plant torque. Time is computed as $k\,\Delta t$ to avoid accumulated
round-off.

## Implementation

[simulator.hpp](../../include/manipulatorcore/simulation/simulator.hpp) /
[simulator.cpp](../../src/simulation/simulator.cpp): `Simulator`, `SimulatorOptions`,
`ContactSurface`, `ContactPoint`, `SimulationLog`;
[integrators.hpp](../../include/manipulatorcore/dynamics/integrators.hpp).

## Numerical considerations

* **Integrators.** Explicit Euler injects energy into oscillatory systems; semi-implicit Euler is
  symplectic and keeps the energy error bounded; RK4 has small dissipative drift (below 0.1 mJ over
  3 s at 1 ms on the planar arm). All three are explicit and therefore conditionally stable.
* **Time step.** The step must resolve the fastest closed-loop mode: for a contact of stiffness $k$
  against an apparent mass $m$, $\omega = \sqrt{k/m}$ and RK4 needs roughly $\omega\,\Delta t \lesssim 1$;
  high controller gains impose the same requirement. A non-finite state raises an exception rather
  than silently producing NaNs.
* **Determinism.** No randomness and a fixed evaluation order: identical inputs give bit-identical
  logs (tested).

## Configuration

`SimulatorOptions`: `dt` (1 ms), `integrator` (RK4), `joint_damping`, `saturate_torques`,
`joint_limit_stops` (on), `joint_limit_stiffness` (5000), `joint_limit_damping` (50), `contacts`,
`contact_point`, `record_every` (1).

## Example

```cpp
#include "manipulatorcore/simulation/simulator.hpp"

using namespace mc;
SimulatorOptions options;
options.contacts = {ContactSurface{-Vec3::UnitX(), -0.6, 8000.0, 150.0, 20.0, "wall"}};  // solid for x >= 0.6
Simulator sim(makeRedundant7R(), options);
sim.reset(redundant7RHome());
GravityCompensationController hold(sim.model());
const SimulationLog& log = sim.run(hold, 1.0);
```

## Benchmark

The [control benchmark](../benchmarks/README.md) runs entirely in this simulator.

## Limitations

* Single contact point against planes; no Coulomb friction cone, no stiction, no multi-body contact.
* Penalty contact requires small steps for stiff surfaces.
* No actuator dynamics, sensor noise or communication delays.

Validated in [test_dynamics_numerics.cpp](../../tests/numerical/test_dynamics_numerics.cpp) (energy
behaviour and fourth-order convergence of RK4) and
[test_control_loops.cpp](../../tests/integration/test_control_loops.cpp) (determinism, saturation,
joint-limit stops and contact forces).

## References

* E. Hairer, C. Lubich and G. Wanner, *Geometric Numerical Integration*, 2nd ed., Springer, 2006 (symplectic integration).
* R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008.
* B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009 (ch. 9, compliance and contact).
