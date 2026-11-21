# Dynamics

## Purpose

The dynamics module evaluates the joint-space equations of motion of a serial manipulator:
inverse dynamics with the Recursive Newton–Euler Algorithm (RNEA), the mass matrix with the
Composite Rigid Body Algorithm (CRBA), forward dynamics with the Articulated Body Algorithm (ABA) or
a Cholesky solve, plus the Christoffel-symbol Coriolis matrix, energies and the centre of mass. The
controllers (gravity compensation, computed torque, impedance, operational space) and the
[simulator](simulation.md) are built on these functions.

## Mathematical formulation

### Equations of motion

$$
M(q)\ddot q + C(q, \dot q)\dot q + g(q) = \tau, \qquad h(q, \dot q) = C(q, \dot q)\dot q + g(q).
$$

$M$ includes the joint armature: $M \leftarrow M + \mathrm{diag}(I_{a,1},\dots,I_{a,n})$, so joint
$i$ needs the extra torque $I_{a,i}\ddot q_i$. Viscous joint damping is not part of this model.
Only the simulator adds it.

### Spatial notation

All spatial vectors are expressed in body-frame coordinates and ordered $(\omega, v)$. For joint $i$
with $T_i = T_{p,i}(q_i)$, motion vectors are carried from parent to child by
$X_i = \mathrm{Ad}_{T_i^{-1}}$, and force vectors from child to parent by $X_i^\top$
(`transformWrench(T_i, ·)`). The cross products are

```math
v \times m = \mathrm{ad}_v\, m = \begin{bmatrix} \omega\times m_\omega \\ v_\text{lin}\times m_\omega + \omega\times m_v \end{bmatrix},
\qquad
v \times^{*} f = -\mathrm{ad}_v^\top f = \begin{bmatrix} \omega\times f_m + v_\text{lin}\times f_f \\ \omega\times f_f \end{bmatrix}.
```

$G_i$ is the spatial inertia of body $i$ about its frame origin (see [robot model](robot-model.md)).
$S_i$ is the joint motion subspace, $(a_i, 0)$ for a revolute joint and $(0, a_i)$ for a prismatic one.

### RNEA (inverse dynamics, $O(n)$)

Gravity is applied as a fictitious base acceleration $a_0 = (0, -g)$, with $v_0 = 0$:

```math
\begin{aligned}
v_i &= X_i v_{i-1} + S_i\dot q_i, \\
a_i &= X_i a_{i-1} + S_i\ddot q_i + v_i\times S_i\dot q_i, \\
f_i &= G_i a_i + v_i\times^{*} G_i v_i, \\
\tau_i &= S_i^\top f_i + I_{a,i}\ddot q_i, \qquad f_{i-1} \mathrel{+}= X_i^\top f_i \quad (i = n-1,\dots,0).
\end{aligned}
```

`gravityTorques` is $\mathrm{RNEA}(q, 0, 0)$, `biasTorques` is $\mathrm{RNEA}(q, \dot q, 0) = h$, and
`coriolisTorques` is RNEA with $\ddot q = 0$ and gravity switched off, which gives $C\dot q$.

### CRBA (mass matrix, $O(n^2)$)

Composite inertias are accumulated tip to base, $I^c_{i-1} \mathrel{+}= X_i^\top I^c_i X_i$. Then, for
each $i$, $F = I^c_i S_i$, $M_{ii} = S_i^\top F + I_{a,i}$, and $F$ is transported towards the base with
$X_j^\top$ to fill $M_{i,j-1} = M_{j-1,i} = S_{j-1}^\top F$ for every ancestor.

### ABA (forward dynamics, $O(n)$)

```math
\begin{aligned}
&\text{pass 1 (base to tip):}\quad v_i = X_i v_{i-1} + S_i\dot q_i,\quad c_i = v_i\times S_i\dot q_i,\quad I^A_i = G_i,\quad p^A_i = v_i\times^{*}G_i v_i \\
&\text{pass 2 (tip to base):}\quad U_i = I^A_i S_i,\quad D_i = S_i^\top U_i + I_{a,i},\quad u_i = \tau_i - S_i^\top p^A_i, \\
&\qquad I^a = I^A_i - U_iU_i^\top/D_i,\quad p^a = p^A_i + I^a c_i + U_i u_i/D_i,\quad
I^A_{i-1} \mathrel{+}= X_i^\top I^a X_i,\quad p^A_{i-1} \mathrel{+}= X_i^\top p^a \\
&\text{pass 3 (base to tip):}\quad a'_i = X_i a_{i-1} + c_i,\quad \ddot q_i = (u_i - U_i^\top a'_i)/D_i,\quad a_i = a'_i + S_i\ddot q_i,
\end{aligned}
```

with $a_0 = (0, -g)$. The armature enters only through $D_i$, because the armature torque acts on
joint $i$ alone.

### Christoffel-symbol Coriolis matrix

$C$ is not unique. When an explicit matrix is needed, the Christoffel construction is used:

$$
C_{ij} = \sum_k \tfrac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)\dot q_k ,
$$

with $\partial M/\partial q_k$ from central differences of CRBA. With this choice $\dot M - 2C$ is
skew-symmetric, the property used in passivity-based stability proofs.
`massMatrixDerivative` returns $\dot M = \big(M(q + h\dot q) - M(q - h\dot q)\big)/(2h)$.

### Energy and centre of mass

$$
T = \tfrac{1}{2}\dot q^\top M(q)\dot q, \qquad V = -\sum_i m_i\, g^\top c_i(q), \qquad c(q) = \frac{\sum_i m_i c_i(q)}{\sum_i m_i},
$$

with $c_i$ the world position of body $i$'s centre of mass. $T$ includes the armature. $V$ is zero
at the height of the base origin.

### Closed-form reference: planar 2R

`planar2RLagrangianDynamics` gives the Euler–Lagrange model of `makePlanar2R` (uniform cylinders,
$\ell_{c,k} = \ell_k/2$, $I_k = m_k(3r^2 + \ell_k^2)/12$, gravity $g$ along $-y$). With
$c_2 = \cos q_2$, $s_2 = \sin q_2$, $c_1 = \cos q_1$, $c_{12} = \cos(q_1 + q_2)$, $\eta = -m_2\ell_1\ell_{c2}s_2$:

```math
M = \begin{bmatrix}
m_1\ell_{c1}^2 + m_2(\ell_1^2 + \ell_{c2}^2 + 2\ell_1\ell_{c2}c_2) + I_1 + I_2 & m_2(\ell_{c2}^2 + \ell_1\ell_{c2}c_2) + I_2 \\
m_2(\ell_{c2}^2 + \ell_1\ell_{c2}c_2) + I_2 & m_2\ell_{c2}^2 + I_2
\end{bmatrix},
\quad
C = \begin{bmatrix} \eta\dot q_2 & \eta(\dot q_1 + \dot q_2) \\ -\eta\dot q_1 & 0 \end{bmatrix},
\quad
g = \begin{bmatrix} (m_1\ell_{c1} + m_2\ell_1)\,g\,c_1 + m_2\ell_{c2}\,g\,c_{12} \\ m_2\ell_{c2}\,g\,c_{12} \end{bmatrix}.
```

This $C$ is the Christoffel form.

## Assumptions

- Rigid links, ideal joints, no friction or actuator dynamics. The joint armature is the only
  actuator effect in the rigid-body model.
- Gravity is the model's base-frame vector `RobotModel::gravity()`.
- Forward dynamics requires $M(q) \succ 0$. A massless subtree without armature makes $D_i = 0$ (ABA)
  or $M$ singular (Cholesky), and both methods then throw `InvalidArgument`.

## Conventions

See [conventions](../mathematics/conventions.md). $M\ddot q + C\dot q + g = \tau + \tau_\text{ext}$. The
bias vector is $h = C\dot q + g$, computed exactly by RNEA. Spatial vectors are in body coordinates,
ordered $(\omega, v)$, motion vectors transform with $\mathrm{Ad}$ and force vectors with its dual.
Featherstone orders spatial vectors angular-first as well, so the recursions match his book. His
Plücker transforms ${}^iX_{\lambda(i)}$ correspond to $X_i = \mathrm{Ad}_{T_i^{-1}}$ here. An external
wrench $\mathcal{F}$ maps to joint torques as $J^\top\mathcal{F}$, with the Jacobian that matches the
wrench's frame and reference point.

## Algorithm

```text
forwardDynamics(q, qd, tau, method):
  ArticulatedBody:     three passes above, O(n); throws if some D_i <= 1e-300
  CompositeRigidBody:  M <- CRBA(q); h <- RNEA(q, qd, 0);
                       LLT(M) (throws if M is not positive definite); qdd <- LLT.solve(tau - h)

coriolisMatrix(q, qd):        n+... CRBA evaluations: dM/dq_k by central differences (h = 1e-6),
                              then the triple sum above, O(n^3) after the derivatives
```

## Implementation

Headers: [rigid_body_dynamics.hpp](../../include/manipulatorcore/dynamics/rigid_body_dynamics.hpp),
[planar_2r.hpp](../../include/manipulatorcore/dynamics/planar_2r.hpp).
Sources: [rigid_body_dynamics.cpp](../../src/dynamics/rigid_body_dynamics.cpp),
[planar_2r.cpp](../../src/dynamics/planar_2r.cpp), [inertia.cpp](../../src/model/inertia.cpp).

```cpp
VecX inverseDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& qdd);  // RNEA
VecX gravityTorques(const RobotModel& model, const VecX& q);
VecX biasTorques(const RobotModel& model, const VecX& q, const VecX& qd);       // C qd + g
VecX coriolisTorques(const RobotModel& model, const VecX& q, const VecX& qd);   // C qd
MatX massMatrix(const RobotModel& model, const VecX& q);                        // CRBA
VecX forwardDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& tau,
                     ForwardDynamicsMethod method = ForwardDynamicsMethod::ArticulatedBody);
MatX coriolisMatrix(const RobotModel& model, const VecX& q, const VecX& qd);
MatX massMatrixDerivative(const RobotModel& model, const VecX& q, const VecX& qd);
double kineticEnergy(const RobotModel& model, const VecX& q, const VecX& qd);
double potentialEnergy(const RobotModel& model, const VecX& q);
Vec3 centerOfMass(const RobotModel& model, const VecX& q);
Planar2RDynamicsTerms planar2RLagrangianDynamics(const Planar2RParameters& params, const Vec2& q,
                                                 const Vec2& qd, double gravity = kStandardGravity);
```

### Validation

[test_dynamics.cpp](../../tests/unit/test_dynamics.cpp):

- `Dynamics.Planar2RMatchesLagrangianReference`: on 100 random states, $M$, $g$ and $C\dot q$ match the
  closed form within $10^{-12}$, the full RNEA torque within $10^{-11}$, and the finite-difference
  Christoffel $C$ matches the analytical $C$ within $10^{-7}$.
- `AllRobots/DynamicsProperties.*`, over the four built-in robots and a mixed revolute/prismatic chain
  with armature:
  - `MassMatrixIsSymmetricPositiveDefinite`;
  - `CrbaMatchesRneaColumns`: column $j$ of $M$ equals $\mathrm{RNEA}(q, 0, e_j) - g$ within $10^{-12}$;
  - `EquationOfMotionDecomposition`: $\mathrm{RNEA} = M\ddot q + C\dot q + g$;
  - `ForwardDynamicsInvertsInverseDynamics`: ABA and LLT agree within $10^{-9}$, and RNEA of the result returns $\tau$;
  - `GravityIsGradientOfPotentialEnergy`: $g = \partial V/\partial q$ by central differences;
  - `ChristoffelCoriolisIsConsistent`: $C\dot q$ equals `coriolisTorques`, and $\dot M - 2C$ is skew-symmetric (tolerance $10^{-6}$);
  - `PowerBalance`: $\tfrac{d}{dt}(T + V) = \dot q^\top\tau$ along the forward-dynamics motion;
  - `ZeroVelocityAndAcceleration`: holding $\tau = g(q)$ gives $\ddot q = 0$.
- `Dynamics.ZeroGravityRemovesGravityTerms`, `Dynamics.ArmatureAddsToMassMatrixDiagonal`,
  `Dynamics.MasslessDistalBodyIsRejectedByForwardDynamics` (both methods throw),
  `Dynamics.InputValidation`, `Dynamics.CenterOfMassOfStretchedPlanarArm`.
- Energy conservation of passive motion under the integrators is tested in
  [test_dynamics_numerics.cpp](../../tests/numerical/test_dynamics_numerics.cpp) (see [simulation](simulation.md)).

## Numerical considerations

- $M^{-1}$ is never formed. The CRBA path uses a Cholesky (LLT) solve, and ABA solves the scalar
  pivots $D_i$ recursively. ABA avoids forming $M$ and costs $O(n)$. LLT costs $O(n^3)$ but is
  competitive for $n \le 7$. Both agree within $10^{-9}$ in the tests.
- The Christoffel matrix and $\dot M$ use central differences with $h = 10^{-6}$. Their accuracy is
  about $10^{-7}$ to $10^{-9}$, adequate for analysis and tests but not exact. `coriolisMatrix` needs
  $2n$ mass-matrix evaluations. Control loops should use `coriolisTorques` or `biasTorques` (one
  RNEA call) instead.
- A small but positive $D_i$ (for example a very light distal body without armature) makes the
  dynamics stiff. A non-zero armature is physically realistic and improves conditioning.
- `combine` and the spatial inertia keep $G_i$ symmetric. CRBA fills $M$ symmetrically by
  construction.

## Configuration

There are no option structs. `forwardDynamics` takes `ForwardDynamicsMethod`
(`ArticulatedBody` by default, or `CompositeRigidBody`). `planar2RLagrangianDynamics` takes the
gravity magnitude (default `kStandardGravity` = 9.81 m/s²). The finite-difference step of
`coriolisMatrix` and `massMatrixDerivative` is fixed at $10^{-6}$.

## Example

```cpp
#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

int main() {
  using namespace mc;
  const RobotModel arm = makeIndustrial6R();
  const VecX q = industrial6RHome();
  const VecX qd = VecX::Constant(arm.dof(), 0.2);
  const VecX qdd = VecX::Constant(arm.dof(), 0.5);

  const VecX tau = inverseDynamics(arm, q, qd, qdd);  // RNEA: M qdd + C qd + g
  const MatX M = massMatrix(arm, q);                   // CRBA (includes armature)
  const VecX h = biasTorques(arm, q, qd);              // C qd + g

  // Both forward-dynamics methods recover qdd.
  const VecX qdd_aba = forwardDynamics(arm, q, qd, tau);
  const VecX qdd_llt = forwardDynamics(arm, q, qd, tau, ForwardDynamicsMethod::CompositeRigidBody);

  const bool consistent = (M * qdd + h - tau).norm() < 1e-9 && (qdd_aba - qdd).norm() < 1e-9 &&
                          (qdd_llt - qdd).norm() < 1e-9;
  return consistent ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- Serial chains with fixed bases only. There are no floating bases, closed loops or contact
  constraints. External forces enter only through $J^\top\mathcal{F}$, which the caller or the
  simulator assembles.
- No friction model (Coulomb, Stribeck) and no motor dynamics. Viscous damping exists only in the
  simulator.
- `coriolisMatrix` and `massMatrixDerivative` are finite-difference approximations. No analytical
  derivatives of RNEA, CRBA or ABA with respect to $q$ or $\dot q$ are provided.
- Parameters are taken as given. There is no parameter identification and no check of physical
  consistency beyond `Inertia::validate`.

## References

- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008. Ch. 2 (spatial vector algebra), ch. 5 (RNEA), ch. 6 (CRBA), ch. 7 (ABA).
- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 8 (dynamics of open chains).
- M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006 (2nd ed. 2020). Euler–Lagrange equations, Christoffel symbols, the planar elbow manipulator.
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 7 (dynamics, skew-symmetry of $\dot B - 2C$).
