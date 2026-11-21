# ADR 0001 — Twist and Jacobian conventions

## Status

Accepted.

## Context

Robotics texts disagree on the ordering of spatial vectors and on which Jacobian is "the" Jacobian.
*Modern Robotics* and Featherstone order twists angular-first ($[\omega; v]$); Siciliano et al. and
Spong et al. present the geometric Jacobian linear-first ($[\dot p; \omega]$); Pinocchio uses
$[v; \omega]$. Mixing conventions inside one code base is a classic source of silent bugs: a
pose error built for one Jacobian still converges for small errors when paired with another, so tests
rarely catch it.

## Decision

* Every 6-vector is ordered angular-first: twists $[\omega; v]$, wrenches $[m; f]$, gain matrices
  [rotation; translation].
* Three Jacobians are provided and always named: space $J_s$, body $J_b$ and geometric $J_g$
  (end-effector-origin velocity in the base frame, rows reordered angular-first).
* Each pose error is paired with its Jacobian: the world-aligned error
  $[\log(R_d R^\top)^\vee; p_d - p]$ with $J_g$, and the body error $\log(T^{-1}T_d)^\vee$ with $J_b$.
* Pose notation `T_a_b` is the pose of frame b in frame a.
* The analytic (Euler-rate) Jacobian is not provided.

## Consequences

* One ordering in the whole library, including dynamics (Featherstone's spatial algebra uses the same
  ordering), so no permutations appear in the algorithms.
* Users porting formulas from linear-first texts must swap blocks; the conventions page states this
  explicitly.
* Every Jacobian is validated against finite differences of its own definition, which would expose a
  pairing mistake.
