# Rigid-body transforms

## Purpose

The geometry module implements the rotation group SO(3) and the rigid-motion group SE(3): the hat
and vee maps, exponential and logarithm maps, the left Jacobian of SO(3), adjoint maps, conversions
between rotation representations, and interpolation on both groups. Every other module builds on
these kernels: forward kinematics (product of exponentials), pose errors in inverse kinematics and
task-space control, Cartesian paths, and the collision primitives' poses.

## Mathematical formulation

### Hat and vee

For $\omega \in \mathbb{R}^3$, $[\omega]$ is the skew-symmetric matrix with $[\omega] v = \omega \times v$.
`vee` returns the vector of the antisymmetric part, $\tfrac{1}{2}(W - W^\top)^\vee$, so slightly
non-skew input is projected rather than rejected. For a twist $\mathcal{V} = (\omega, v)$:

```math
[\mathcal{V}] = \begin{bmatrix} [\omega] & v \\ 0 & 0 \end{bmatrix} \in \mathfrak{se}(3).
```

### SO(3) exponential (Rodrigues' formula)

For a rotation vector $\phi = u\,\theta$ ($\lVert u \rVert = 1$, $\theta = \lVert \phi \rVert$):

$$
\exp([\phi]) = I + a(\theta)\,[\phi] + b(\theta)\,[\phi]^2, \qquad a = \frac{\sin\theta}{\theta}, \quad b = \frac{1-\cos\theta}{\theta^2}.
$$

$b$ is evaluated as $2\sin^2(\theta/2)/\theta^2$, which has no cancellation for small $\theta$.

### SO(3) logarithm

With $s = \tfrac{1}{2}(R - R^\top)^\vee = \sin\theta\, u$ and $\cos\theta = \mathrm{clamp}\big(\tfrac{1}{2}(\mathrm{tr}\,R - 1), -1, 1\big)$,
the angle is $\theta = \mathrm{atan2}(\lVert s \rVert, \cos\theta) \in [0, \pi]$ and, away from the
special regions, $\log(R)^\vee = \dfrac{\theta}{\sin\theta}\, s$.

Near $\theta = \pi$ the skew part vanishes, so the axis is taken from the symmetric part:

$$
\tfrac{1}{2}(R + R^\top) - \cos\theta\, I = (1 - \cos\theta)\, u u^\top .
$$

The column of this matrix with the largest diagonal entry is normalized to give $\pm u$. Its sign is
chosen so that $u \cdot s \ge 0$, which keeps the axis consistent with $\sin\theta \ge 0$ below $\pi$.
At exactly $\theta = \pi$ both signs are valid and either may be returned.

### Left Jacobian of SO(3)

```math
J_l(\phi) = \sum_{k \ge 0} \frac{[\phi]^k}{(k+1)!}
          = I + \frac{1-\cos\theta}{\theta^2}[\phi] + \frac{\theta - \sin\theta}{\theta^3}[\phi]^2,
```

```math
J_l^{-1}(\phi) = I - \tfrac{1}{2}[\phi] + c(\theta)[\phi]^2, \qquad
c = \frac{1 - \tfrac{\theta}{2}\cot\tfrac{\theta}{2}}{\theta^2}.
```

$J_l^{-1}$ exists for $\theta < 2\pi$. $J_l$ satisfies $\exp([\phi + \delta]) \approx \exp([J_l(\phi)\delta])\exp([\phi])$ to first order in $\delta$.

### SE(3) exponential and logarithm

For the scaled twist $\xi = (\phi, \rho) = (\omega\theta, v\theta)$:

```math
\exp([\xi]) = \begin{bmatrix} \exp([\phi]) & J_l(\phi)\,\rho \\ 0 & 1 \end{bmatrix},
\qquad
\log(T)^\vee = \big(\phi,\; J_l^{-1}(\phi)\, p\big), \quad \phi = \log(R)^\vee .
```

$J_l(\phi)\rho$ equals the matrix $G(\theta)v$ of Lynch and Park (ch. 3); the two forms differ only in
whether $\theta$ is factored out. The rotation angle of `logSE3` is in $[0, \pi]$.

### Adjoint maps and change of frame

```math
\mathrm{Ad}_T = \begin{bmatrix} R & 0 \\ [p]R & R \end{bmatrix},
\qquad
\mathrm{ad}_{\mathcal{V}} = \begin{bmatrix} [\omega] & 0 \\ [v] & [\omega] \end{bmatrix}.
```

A twist written in frame $b$ is expressed in frame $a$ as $\mathcal{V}_a = \mathrm{Ad}_{T_{ab}}\mathcal{V}_b$,
that is $\omega_a = R\,\omega_b$ and $v_a = p \times \omega_a + R\, v_b$. A wrench transforms with
$\mathcal{F}_a = \mathrm{Ad}_{T_{ab}}^{-\top}\mathcal{F}_b$: $f_a = R f_b$ and $m_a = R m_b + p \times f_a$.
Power is invariant, $\mathcal{F}_a^\top \mathcal{V}_a = \mathcal{F}_b^\top \mathcal{V}_b$.
$\mathrm{ad}_{\mathcal{V}_1}\mathcal{V}_2$ is the Lie bracket $[\mathcal{V}_1, \mathcal{V}_2]$, the vee of the matrix commutator.

### Rotation representations

- **Quaternion to matrix** (Hamilton, $q = (w, x, y, z)$, normalized first):

```math
R(q) = \begin{bmatrix}
1 - 2(y^2+z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2+z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2+y^2)
\end{bmatrix}.
```

- **Matrix to quaternion**: Shepperd's method. If $\mathrm{tr}\,R > 0$ the scalar part is computed
  first. Otherwise the branch of the largest diagonal entry of $R$ is used, so the divisor
  $s = 2\sqrt{1 + R_{kk} - \dots}$ is never small. The result is normalized and made canonical ($w \ge 0$).
- **Axis-angle**: $(u, \theta) = (\phi/\lVert\phi\rVert, \lVert\phi\rVert)$ with $\phi = \log(R)^\vee$, so $\theta \in [0, \pi]$. Below $10^{-15}$ the axis is reported as $+x$ with angle $0$.
- **Euler angles** (intrinsic, $R = R_A(\alpha)R_B(\beta)R_C(\gamma)$):

| Sequence | $\beta$ | $\alpha$ (regular) | Gimbal-lock test | $\alpha$ at gimbal lock ($\gamma = 0$) |
| --- | --- | --- | --- | --- |
| ZYX | $\mathrm{atan2}(-R_{20}, \sqrt{R_{00}^2 + R_{10}^2})$ | $\mathrm{atan2}(R_{10}, R_{00})$ | $\cos\beta < 10^{-12}$ | $\mathrm{atan2}(-R_{01}, R_{11})$ |
| XYZ | $\mathrm{atan2}(R_{02}, \sqrt{R_{12}^2 + R_{22}^2})$ | $\mathrm{atan2}(-R_{12}, R_{22})$ | $\cos\beta < 10^{-12}$ | $\mathrm{atan2}(R_{21}, R_{11})$ |
| ZYZ | $\mathrm{atan2}(\sqrt{R_{02}^2 + R_{12}^2}, R_{22})$ | $\mathrm{atan2}(R_{12}, R_{02})$ | $\sin\beta < 10^{-12}$ | $\mathrm{atan2}(R_{10}, R_{00})$ if $R_{22} > 0$, else $\mathrm{atan2}(-R_{10}, R_{11})$ |

  Outside gimbal lock, $\gamma$ is not read from single entries of $R$. It is taken from the
  residual rotation $(R_A(\alpha)R_B(\beta))^\top R$, so the angles reproduce $R$ to round-off even
  when $\cos\beta$ (or $\sin\beta$) is small but above the threshold.
- **Roll-pitch-yaw**: `rotationFromRPY(roll, pitch, yaw)` $= R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})$; `rpyFromRotation` is ZYX with the output reordered.

### Interpolation

- `slerp(q0, q1, s)`: both inputs are normalized and $q_1 \leftarrow -q_1$ if $q_0 \cdot q_1 < 0$ (shortest arc). With $\cos\Omega = q_0\cdot q_1$:

$$
q(s) = \frac{\sin((1-s)\Omega)}{\sin\Omega} q_0 + \frac{\sin(s\Omega)}{\sin\Omega} q_1 .
$$

  If $q_0 \cdot q_1 > 0.9995$, normalized linear interpolation is used instead.
- `interpolateRotation`: the geodesic $R(s) = R_0 \exp\big(s \log(R_0^\top R_1)\big)$, which equals SLERP.
- `interpolatePoseDecoupled`: geodesic rotation and straight-line translation $p_0 + s(p_1 - p_0)$.
- `interpolatePoseScrew`: $T(s) = T_0 \exp\big(s \log(T_0^{-1}T_1)\big)$. The origin follows a helix when the rotation is non-zero.

### Uniform random rotations

`Random::uniformQuaternion` uses Shoemake's subgroup algorithm. For $u_1, u_2, u_3 \sim U[0,1)$:
$q = \big(\sqrt{u_1}\cos 2\pi u_3,\ \sqrt{1-u_1}\sin 2\pi u_2,\ \sqrt{1-u_1}\cos 2\pi u_2,\ \sqrt{u_1}\sin 2\pi u_3\big)$ in $(w, x, y, z)$ order.

## Assumptions

- Inputs to `logSO3`, `logSE3`, the Euler and quaternion conversions are valid rotations, up to
  round-off. `SE3(R, p)` does not re-orthogonalize. Use `projectToSO3` after long products or for
  data from external sources, and `SE3::fromMatrix`, which validates its input.
- Rotation vectors passed to `leftJacobianInverseSO3` have norm below $2\pi$.
- Euler angles are used for input and output only. No algorithm subtracts them.

## Conventions

See [conventions](../mathematics/conventions.md). This page relies on:

- active, right-handed rotations; $T_{ab}$ is the pose of frame $b$ in frame $a$, and $T_{ac} = T_{ab}T_{bc}$;
- twists ordered $(\omega, v)$ and wrenches $(m, f)$, angular first, with $\mathrm{Ad}_T$ as above.
  Some texts, for example parts of Siciliano et al., stack linear components first. Their adjoint
  blocks are permuted with respect to the ones here;
- Hamilton quaternions documented scalar-first. `Eigen::Quaterniond` is constructed as
  `Quat(w, x, y, z)`. The library never relies on the storage order of `coeffs()`;
- intrinsic Euler sequences, with the ranges $\alpha, \gamma \in (-\pi, \pi]$, $\beta \in [-\pi/2, \pi/2]$ (ZYX, XYZ) or $[0, \pi]$ (ZYZ).

## Algorithm

```text
logSO3(R):
  s      <- vee of antisymmetric part of R           # = sin(theta) u
  c      <- clamp((trace(R) - 1) / 2, -1, 1)
  theta  <- atan2(|s|, c)
  if theta < 1e-3:            return (1 + theta^2/6 + 7 theta^4/360) * s
  if theta > pi - 1e-2:
      S <- (R + R^T)/2 - c I                          # = (1 - c) u u^T
      k <- argmax diag(S);  u <- S[:, k] / |S[:, k]|
      if u . s < 0: u <- -u
      return theta * u
  return theta / |s| * s
```

`expSE3` and `logSE3` are compositions of the SO(3) maps with $J_l$ and $J_l^{-1}$. No 4×4 matrix
exponential or logarithm is evaluated.

## Implementation

Headers: [so3.hpp](../../include/manipulatorcore/geometry/so3.hpp),
[se3.hpp](../../include/manipulatorcore/geometry/se3.hpp),
[rotation.hpp](../../include/manipulatorcore/geometry/rotation.hpp),
[interpolation.hpp](../../include/manipulatorcore/geometry/interpolation.hpp),
[random.hpp](../../include/manipulatorcore/math/random.hpp).
Sources: [so3.cpp](../../src/geometry/so3.cpp), [se3.cpp](../../src/geometry/se3.cpp),
[rotation.cpp](../../src/geometry/rotation.cpp), [interpolation.cpp](../../src/geometry/interpolation.cpp),
[random.cpp](../../src/math/random.cpp).

```cpp
Mat3 hat(const Vec3& w);                 Vec3 vee(const Mat3& W);
Mat3 expSO3(const Vec3& phi);            Vec3 logSO3(const Mat3& R);
Mat3 leftJacobianSO3(const Vec3& phi);   Mat3 leftJacobianInverseSO3(const Vec3& phi);
Mat3 projectToSO3(const Mat3& M);        double rotationDistance(const Mat3& R_a, const Mat3& R_b);

class SE3;  // rotation(), translation(), inverse(), operator*, adjoint(), matrix(), quaternion()
Mat4 hatSE3(const Vec6& twist);          Vec6 veeSE3(const Mat4& twist_matrix);
SE3  expSE3(const Vec6& xi);             Vec6 logSE3(const SE3& T);
Mat6 adjoint(const SE3& T);              Mat6 adSE3(const Vec6& twist);
Vec6 transformTwist(const SE3& T_a_b, const Vec6& twist_b);
Vec6 transformWrench(const SE3& T_a_b, const Vec6& wrench_b);
PoseDistance poseDistance(const SE3& T_a, const SE3& T_b);

Quat quaternionFromRotation(const Mat3& R);  Mat3 rotationFromQuaternion(const Quat& q);
AxisAngle axisAngleFromRotation(const Mat3& R);
Mat3 rotationFromEuler(const Vec3& angles, EulerSequence sequence);
Vec3 eulerFromRotation(const Mat3& R, EulerSequence sequence);
Mat3 rotationFromRPY(double roll, double pitch, double yaw);  Vec3 rpyFromRotation(const Mat3& R);

Quat slerp(const Quat& q0, const Quat& q1, double s);
Mat3 interpolateRotation(const Mat3& R0, const Mat3& R1, double s);
SE3  interpolatePoseDecoupled(const SE3& T0, const SE3& T1, double s);
SE3  interpolatePoseScrew(const SE3& T0, const SE3& T1, double s);
```

### Validation

- [test_so3.cpp](../../tests/unit/test_so3.cpp): `SO3.HatProducesCrossProduct`, `SO3.ExpMatchesElementaryRotations`,
  `SO3.ElementaryRotationsAreCounterClockwise` (sign convention), `SO3.LogInvertsExpForAnglesBelowPi`,
  `SO3.ExpInvertsLog`, `SO3.LogAngleIsInZeroPi`, `SO3.LeftJacobianInverse` ($J_l J_l^{-1} = I$),
  `SO3.LeftJacobianMatchesFiniteDifference` (columns of $J_l$ against
  $\log(\exp(\phi + h e_k)\exp(\phi)^\top)/h$ with $h = 10^{-6}$), `SO3.ProjectToSO3RecoversRotation`,
  `SO3.IsRotationMatrixRejectsReflections`, `SO3.RotationDistanceIsGeodesicAngle`.
- [test_se3.cpp](../../tests/unit/test_se3.cpp): `SE3.CompositionFollowsFrameNotation`,
  `SE3.ExpOfRevoluteScrewAboutOffsetAxis`, `SE3.ExpMatchesMatrixExponentialSeries` (40-term power
  series of the 4×4 matrix exponential, tolerance $10^{-12}$), `SE3.LogInvertsExp`, `SE3.ExpInvertsLog`,
  `SE3.AdjointMapsTwistMatrices` ($[\mathrm{Ad}_T\mathcal{V}] = T[\mathcal{V}]T^{-1}$),
  `SE3.AdjointIsGroupHomomorphism`, `SE3.WrenchTransformPreservesPower`,
  `SE3.LieBracketMatchesMatrixCommutator`, `SE3.PoseDistance`.
- [test_lie_numerics.cpp](../../tests/numerical/test_lie_numerics.cpp): `LieNumerics.ExpLogAtTinyAngles`
  (angles from $10^{-15}$ to just above the series threshold, relative error $\le 10^{-12}$),
  `LieNumerics.ExpLogContinuityAcrossSeriesThreshold` (series and closed-form branches agree at
  $10^{-3}$ for exp, $J_l$, $J_l^{-1}$), `LieNumerics.LogAtExactlyPi`, `LieNumerics.LogNearPi`
  (distances to $\pi$ from $10^{-12}$ to $0.1$, on both sides of the $10^{-2}$ switch; the axis sign is
  preserved), `LieNumerics.SE3LogNearPiRotation`, `LieNumerics.QuaternionNearPiAndIdentity`,
  `LieNumerics.LongCompositionStaysNearSO3` ($10^5$ products).
- [test_rotation.cpp](../../tests/unit/test_rotation.cpp): `Rotation.QuaternionFromRotationCoversAllShepperdBranches`,
  `Rotation.QuaternionOfPiRotation`, `Rotation.QuaternionHamiltonConvention`, `Rotation.AxisAngleRoundTrip`,
  the parameterized `EulerRoundTrip.RandomRotations`, `EulerRoundTrip.GimbalLockReconstructs` and
  `EulerRoundTrip.NearGimbalLockReconstructs` for all three sequences, `Rotation.EulerAnglesInDocumentedRanges`,
  `Rotation.RpyMatchesZyxIntrinsic`.
- [test_interpolation.cpp](../../tests/unit/test_interpolation.cpp): `Interpolation.SlerpHasConstantAngularVelocity`,
  `Interpolation.SlerpTakesShortestArc`, `Interpolation.SlerpAgreesWithGeodesic`,
  `Interpolation.DecoupledPoseMovesOriginOnStraightLine`, `Interpolation.ScrewInterpolationEndpoints`.
- [test_random.cpp](../../tests/unit/test_random.cpp): `Random.UnitVectorsAndQuaternionsAreNormalized`.

## Numerical considerations

- **Small angles.** For $\theta < 10^{-3}$ the coefficients use Taylor series through $\theta^4$:
  $a = 1 - \theta^2/6 + \theta^4/120$ and $b = 1/2 - \theta^2/24 + \theta^4/720$ (exp). For $J_l$ they are
  $1/2 - \theta^2/24 + \theta^4/720$ and $1/6 - \theta^2/120 + \theta^4/5040$. The log uses
  $\theta/\sin\theta = 1 + \theta^2/6 + 7\theta^4/360$, and $J_l^{-1}$ uses $c = 1/12 + \theta^2/720$.
  The truncation error is $O(\theta^6) < 10^{-18}$ at the threshold.
- **Cancellation.** $1 - \cos\theta$ is evaluated as $2\sin^2(\theta/2)$. The $J_l^{-1}$ coefficient uses
  the half-angle form $\tfrac{\theta}{2}\cot\tfrac{\theta}{2}$.
- **Near $\pi$.** The symmetric-part recovery is used for $\theta > \pi - 10^{-2}$. There, the
  relative precision of $\sin\theta$ obtained from the skew part degrades.
- **Angle extraction** uses $\mathrm{atan2}(\lVert s\rVert, \cos\theta)$ with the cosine clamped to
  $[-1, 1]$. This is accurate over the whole range, unlike $\arccos$ near $0$ and $\pi$.
- **Quaternion normalization** throws `InvalidArgument` if the norm is not finite or is at most
  $10^{-12}$.
- **Drift.** `isRotationMatrix` checks $\max\lvert R^\top R - I\rvert \le \text{tol}$ and
  $\lvert\det R - 1\rvert \le \text{tol}$ (default $10^{-9}$). `projectToSO3` returns
  $U\,\mathrm{diag}(1, 1, \det(UV^\top))\,V^\top$ from the SVD $M = U\Sigma V^\top$, the closest rotation
  in the Frobenius norm.
- **SLERP** switches to normalized linear interpolation when $q_0\cdot q_1 > 0.9995$, where
  $\sin\Omega$ in the denominator becomes small.

## Configuration

There are no option structs. Tolerances are function parameters with defaults:

| Function | Parameter | Default |
| --- | --- | --- |
| `isRotationMatrix(R, tolerance)` | max-abs orthogonality and determinant error | `1e-9` |
| `SE3::fromMatrix(T, tolerance)` | rotation-block and last-row check | `1e-9` |
| `SE3::isApprox(other, tolerance)` | max-abs difference of $R$ and $p$ | `1e-9` |

Internal thresholds (constants in the sources): series threshold $10^{-3}$ rad, near-$\pi$ band
$10^{-2}$ rad, gimbal-lock threshold $10^{-12}$, SLERP linear fallback $0.9995$.

## Example

```cpp
#include "manipulatorcore/geometry/interpolation.hpp"
#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/geometry/so3.hpp"

int main() {
  using namespace mc;
  // Pose of the tool frame in the base frame from a scaled twist [omega*theta; v*theta].
  Vec6 xi;
  xi << 0.0, 0.0, kHalfPi, 0.3, 0.0, 0.1;
  const SE3 T_base_tool = expSE3(xi);

  // The logarithm recovers the twist (rotation angle in [0, pi]).
  const bool round_trip = (logSE3(T_base_tool) - xi).norm() < 1e-12;

  // A wrench measured in the tool frame, expressed in the base frame: [moment; force].
  Vec6 F_tool;
  F_tool << 0.0, 0.0, 0.0, 0.0, 0.0, -10.0;
  const Vec6 F_base = transformWrench(T_base_tool, F_tool);

  // Halfway pose: straight-line translation, geodesic rotation.
  const SE3 T_mid = interpolatePoseDecoupled(SE3::Identity(), T_base_tool, 0.5);
  const Quat q_mid = T_mid.quaternion();  // canonical, w >= 0

  return (round_trip && F_base.allFinite() && q_mid.w() >= 0.0) ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- `SE3` stores a $3\times 3$ matrix and a vector and does not re-orthogonalize after composition.
  Drift is at round-off level for long products (see `LieNumerics.LongCompositionStaysNearSO3`). It is
  not corrected automatically.
- At exactly $\theta = \pi$ the sign of the logarithm's axis is arbitrary, so `logSO3`, `logSE3`,
  `interpolateRotation` and the screw interpolation are not continuous there.
- `leftJacobianInverseSO3` is singular at $\theta = 2\pi$. Inputs produced by `logSO3` have
  $\theta \le \pi$.
- No analytic (Euler-rate) Jacobian and no dual-quaternion representation are provided.
- `slerp` and `interpolateRotation` interpolate between two rotations only. There is no spline on
  SO(3) (for example SQUAD).

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 3 (exponential coordinates, twists, wrenches, adjoint).
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008. Ch. 2 (spatial vector algebra).
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 2 (rotation representations, Euler angles, unit quaternions).
- K. Shoemake, "Uniform random rotations," in D. Kirk (ed.), *Graphics Gems III*, Academic Press, 1992.
