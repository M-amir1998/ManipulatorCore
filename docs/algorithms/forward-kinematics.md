# Forward kinematics

## Purpose

Forward kinematics maps a joint configuration $q$ to the poses of all body frames and of the
end-effector (tool) frame in the base frame. ManipulatorCore has three independent
implementations: the general serial chain used by every algorithm, a direct product of DH matrices,
and the product of exponentials (PoE) in space and body form. The second and third exist so that
the first can be cross-validated.

## Mathematical formulation

### General chain

With the joint description of the [robot model](robot-model.md),

```math
T_{s,0} = T^{(0)}_{p,j}\,\mathrm{motion}_0(q_0), \qquad
T_{s,i} = T_{s,i-1}\,T^{(i)}_{p,j}\,\mathrm{motion}_i(q_i), \qquad
T_{se} = T_{s,n-1}\,T_{\text{flange},e}.
```

The world-frame axis of joint $i$ is $\hat a_i = R_{s,i}\,a_i$. Its origin is $p_i$, the origin of
body frame $i$. For a revolute joint this origin lies on the joint axis.

### Direct Denavit–Hartenberg product

```math
T_{0,i} = \prod_{k=1}^{i} A_k(q_k), \qquad T_{se} = T_{0,n}\,T_\text{tool},
```

with $A_k$ the standard ($R_zT_zT_xR_x$) or modified ($R_xT_xR_zT_z$) row transform (see
[robot model](robot-model.md#mathematical-formulation)). In the standard convention, DH frame $i$
and body frame $i$ differ by `dhFrameInBody`.

### Product of exponentials

The PoE description is extracted from the model at $q = 0$. $M = T_{se}(0)$ is the home pose. With
$\hat a_i$ and $p_i$ evaluated at $q = 0$, the screw axes are

```math
S_i = \begin{bmatrix} \hat a_i \\ -\hat a_i \times p_i \end{bmatrix} \;(\text{revolute}), \qquad
S_i = \begin{bmatrix} 0 \\ \hat a_i \end{bmatrix} \;(\text{prismatic}), \qquad
B_i = \mathrm{Ad}_{M^{-1}} S_i .
```

The code writes $-\hat a_i \times p_i$ as $p_i \times \hat a_i$. Forward kinematics is then

$$
T_{se}(q) = e^{[S_1]q_1}\cdots e^{[S_n]q_n} M = M e^{[B_1]q_1}\cdots e^{[B_n]q_n}.
$$

Each factor is evaluated with `expSE3` (see [rigid-body transforms](rigid-body-transforms.md)).

## Assumptions

- $q$ has `dof()` finite entries. `checkConfiguration` throws `InvalidArgument` otherwise.
  Configurations outside the joint limits are accepted: forward kinematics is defined everywhere.
- The PoE description is valid for the model it was extracted from. Changing the model's joints or
  tool transform requires a new extraction.

## Conventions

See [conventions](../mathematics/conventions.md). $T_{ab}$ is the pose of $b$ in $a$, so chain
composition is right-multiplication. Body frames are the joint frames after joint motion. Screw
axes are ordered $(\omega, v)$. The space form uses screws in the base frame and the body form uses
screws in the end-effector frame at home.

## Algorithm

```text
computeForwardKinematics(model, q, state):
  T <- identity
  for i in 0..n-1:
      T <- T * joint_i.origin * motion_i(q_i)      # motion: exp([a] q) or translation a q
      state.T_base_body[i] <- T
  state.T_base_ee <- T * T_flange_ee
```

The cost is $O(n)$ rigid-transform products. `computeForwardKinematics` reuses the storage of an
existing `KinematicState`. Jacobian functions accept a precomputed state, which avoids repeating the
forward pass.

## Implementation

Headers: [forward_kinematics.hpp](../../include/manipulatorcore/kinematics/forward_kinematics.hpp),
[poe.hpp](../../include/manipulatorcore/kinematics/poe.hpp),
[dh.hpp](../../include/manipulatorcore/model/dh.hpp).
Sources: [forward_kinematics.cpp](../../src/kinematics/forward_kinematics.cpp),
[poe.cpp](../../src/kinematics/poe.cpp), [dh.cpp](../../src/model/dh.cpp).

```cpp
struct KinematicState {
  VecX q;
  std::vector<SE3> T_base_body;   // pose of body i in the base frame
  SE3 T_base_ee;                  // tool pose in the base frame
  Vec3 jointAxis(const RobotModel& model, int i) const;  // world-frame unit axis of joint i
  Vec3 jointOrigin(int i) const;                         // world-frame origin of joint i (= body i)
};
void computeForwardKinematics(const RobotModel& model, const VecX& q, KinematicState& state);
KinematicState forwardKinematics(const RobotModel& model, const VecX& q);
SE3 endEffectorPose(const RobotModel& model, const VecX& q);

std::vector<SE3> dhFrames(const DHTable& table, const VecX& q);
SE3 dhForwardKinematics(const DHTable& table, const VecX& q);

ProductOfExponentials productOfExponentials(const RobotModel& model);  // S_i, B_i, M
SE3 poeForwardSpace(const ProductOfExponentials& poe, const VecX& q);
SE3 poeForwardBody(const ProductOfExponentials& poe, const VecX& q);
```

### Validation

- [test_kinematics.cpp](../../tests/unit/test_kinematics.cpp):
  - `ForwardKinematics.Planar2RMatchesClosedForm`: $p = (\ell_1 c_1 + \ell_2 c_{12},\ \ell_1 s_1 + \ell_2 s_{12},\ 0)$ and $R = R_z(q_1 + q_2)$ within $10^{-14}$.
  - `ForwardKinematics.Planar3RKnownConfigurations`, `ForwardKinematics.HomeConfigurationOfRedundant7RIsVertical` (tool at $z = 1.36$ m with identity rotation).
  - `ForwardKinematics.StateIsConsistent`: the state, `endEffectorPose` and the tool transform agree.
  - `ForwardKinematics.PoEMatchesChainForAllRobots`: space and body PoE equal the chain within
    $10^{-12}$ on 30 random configurations of every built-in robot and of a mixed
    revolute/prismatic chain with non-trivial joint origins.
- [test_model.cpp](../../tests/unit/test_model.cpp): `DHEquivalence.ChainModelReproducesDirectDHProduct`
  (standard and modified conventions; every DH frame and the tool pose within $10^{-12}$) and
  `DH.Industrial6RMatchesItsDHTable`.

## Numerical considerations

- Every step is a product of exactly orthonormal factors, $\exp([a]q)$ from Rodrigues' formula and
  fixed origins, so the accumulated rotation stays orthonormal to round-off. No
  re-orthogonalization is performed.
- The PoE forms evaluate `expSE3` once per joint and are more expensive than the chain recursion.
  They are provided for analysis and cross-checking, not as the primary implementation.
- The test tolerance of $10^{-12}$ between the three implementations is limited by round-off in
  $\exp$ and in products of up to eight transforms.

## Configuration

There are no options. The model fixes the kinematic parameters, and `RobotModel::setToolTransform`
changes the tool frame.

## Example

```cpp
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/poe.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

int main() {
  using namespace mc;
  const RobotModel arm = makeRedundant7R();
  const VecX q = redundant7RHome();

  // All body frames and the tool frame in one pass.
  const KinematicState state = forwardKinematics(arm, q);
  const Vec3 elbow_axis_world = state.jointAxis(arm, 3);

  // Independent product-of-exponentials evaluation of the same pose.
  const ProductOfExponentials poe = productOfExponentials(arm);
  const bool agree = poeForwardSpace(poe, q).isApprox(state.T_base_ee, 1e-12) &&
                     poeForwardBody(poe, q).isApprox(state.T_base_ee, 1e-12);

  return (agree && elbow_axis_world.allFinite()) ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- Serial chains only (see [robot model](robot-model.md)).
- The PoE description is a snapshot of the model at extraction time.
- There is no batched or vectorized multi-configuration evaluation, and no caching across calls
  beyond reusing a `KinematicState`.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 4 (product of exponentials, space and body forms).
- M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006 (2nd ed. 2020). Forward kinematics with the standard DH convention.
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson Prentice Hall, 2005. Modified DH convention.
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 2 (direct kinematics).
