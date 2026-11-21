# Conventions

Ambiguous conventions are among the most common sources of bugs in robotics software. ManipulatorCore
uses exactly one convention for each concept listed here, everywhere in the code, tests and documentation.
If you find a place that disagrees with this page, it is a bug.

## Units

All quantities use SI units unless a symbol's documentation states otherwise.

| Quantity | Unit |
| --- | --- |
| length, position | metre (m) |
| time | second (s) |
| angle | radian (rad) |
| mass | kilogram (kg) |
| force | newton (N) |
| torque, moment | newton-metre (N·m) |
| linear stiffness | N/m |
| rotational stiffness | N·m/rad |
| rotational inertia | kg·m² |

## Coordinate frames

* All frames are **right-handed** and orthonormal.
* The robot base frame is the reference ("space" / "world") frame unless stated otherwise.
* Gravity is a vector expressed in the base frame. The default is `g = (0, 0, -9.81) m/s²`
  (base `z` axis points up).

### Pose notation `T_a_b`

`T_a_b` is the **pose of frame `b` expressed in frame `a`**. Equivalently, it maps the coordinates of a
point written in frame `b` into frame `a`:

```text
p_a = T_a_b * p_b
```

Composition chains the inner frames:

```text
T_a_c = T_a_b * T_b_c
T_b_a = T_a_b⁻¹
```

The same convention applies to rotations (`R_a_b`) and positions (`p_a_b`, the origin of `b` written in `a`).
Code uses the same spelling for variables, for example `T_base_ee`, `R_world_tool`.

## Rotations

* Rotations are **active**: `R_a_b` rotates vectors written in `b` into `a`. A rotation matrix is
  orthogonal with determinant `+1`.
* Elementary rotations `Rx(θ)`, `Ry(θ)`, `Rz(θ)` rotate counter-clockwise by `θ` about the given axis
  when looking down the axis toward the origin (right-hand rule).

### Quaternions

* **Hamilton** convention: `q = w + x i + y j + z k`, `i² = j² = k² = ijk = -1`.
* Written and documented in **scalar-first** order `(w, x, y, z)`.
* The C++ API uses `Eigen::Quaterniond`. Its constructor takes `(w, x, y, z)`, but
  `Eigen::Quaterniond::coeffs()` stores `(x, y, z, w)`. ManipulatorCore never relies on `coeffs()`
  ordering in its public API.
* `q` and `-q` represent the same rotation. Conversions from a rotation matrix return the
  **canonical** representative with `w ≥ 0`.
* Rotation of a vector: `v' = q v q*`, which equals `R(q) v`.

### Euler angles

Euler angles are supported as a convenience for input and output only. Algorithms never use Euler
angles internally, and never subtract them to form an orientation error.

* Sequences are **intrinsic** (rotations about the moving axes). For a sequence `(A, B, C)` and
  angles `(α, β, γ)`:

  ```text
  R = R_A(α) · R_B(β) · R_C(γ)
  ```

* Supported sequences: `ZYX` (yaw–pitch–roll), `XYZ` and `ZYZ`.
* Intrinsic `ZYX` with angles `(yaw, pitch, roll)` is identical to extrinsic `XYZ` roll–pitch–yaw.
  The helper `rotationFromRPY(roll, pitch, yaw)` returns `Rz(yaw) · Ry(pitch) · Rx(roll)`.
* **Gimbal lock.** At `β = ±π/2` for `ZYX`/`XYZ`, and at `β ∈ {0, π}` for `ZYZ`, only the sum or
  difference of `α` and `γ` is observable. The conversion sets `γ = 0` and assigns the whole
  rotation to `α`. The returned angles still reproduce the input matrix exactly.
* Ranges: `α, γ ∈ (-π, π]`; `β ∈ [-π/2, π/2]` for `ZYX`/`XYZ` and `β ∈ [0, π]` for `ZYZ`.

### Axis–angle and the SO(3) logarithm

* `exp([ω]θ)` with unit axis `ω` and angle `θ` is evaluated with the Rodrigues formula. A
  second-order Taylor expansion is used when `θ` is below a small threshold.
* `log(R)` returns `ω θ` with `θ ∈ [0, π]`. At `θ = π` the axis is recovered from the symmetric
  part of `R`, because the skew-symmetric part vanishes there. Near `θ = 0` a series expansion is used.

## Twists, wrenches and spatial vectors

ManipulatorCore orders every 6-vector **angular first**. This matches *Modern Robotics* (Lynch & Park)
and Featherstone's spatial algebra.

```text
twist   V = [ω; v] ∈ ℝ⁶   (rad/s, m/s)
wrench  F = [m; f] ∈ ℝ⁶   (N·m, N)
power   P = Fᵀ V
```

* **Spatial twist** `V_s` (Modern Robotics "space twist"): `[V_s] = Ṫ T⁻¹` for `T = T_s_b`.
  `ω_s` is the angular velocity in the space frame. `v_s` is the velocity of the (imaginary)
  body-fixed point that is momentarily at the **space-frame origin**, also written in the space frame.
* **Body twist** `V_b`: `[V_b] = T⁻¹ Ṫ`, written in the body frame.
* `V_s = Ad_T V_b`, with

  ```text
  Ad_T = [ R      0 ]
         [ [p]R   R ]
  ```

* Wrenches transform with the dual (transpose-inverse) adjoint: `F_b = Ad_Tᵀ F_s`.
* Stiffness and damping gain matrices for 6-D tasks use the same block ordering
  (rotational block first, then translational).

## Jacobians

All Jacobians are `6 × n` with rows ordered `[ω; v]`. ManipulatorCore provides three of them, and the
documentation always says which one an API expects.

| Name | Definition | Notes |
| --- | --- | --- |
| Space Jacobian `J_s(q)` | `V_s = J_s q̇` | Column `i` is the screw axis of joint `i` in its current configuration, written in the base frame. |
| Body Jacobian `J_b(q)` | `V_b = J_b q̇` | `J_b = Ad_{T_s_b⁻¹} J_s`. Written in the end-effector frame. |
| Geometric Jacobian `J_g(q)` | `[ω_s; ṗ_ee] = J_g q̇` | `ṗ_ee` is the velocity of the end-effector origin, written in the base frame. This is the classical "geometric Jacobian" of Siciliano et al. and Spong et al., but with the rows reordered to angular-first. |

The geometric and space Jacobians are related by

```text
J_g = [  I     0 ] J_s ,    p = position of the end-effector origin
      [ -[p]   I ]
```

The **analytic Jacobian** (Euler-angle rates) is intentionally not provided. It is
representation-dependent and singular at gimbal lock.

### Pose errors and their matching Jacobians

Each pose error is paired with the Jacobian whose rows describe the same quantity:

| Error | Definition | Matching Jacobian |
| --- | --- | --- |
| World-aligned error `e_w` | `[ log(R_d Rᵀ)^∨ ; p_d - p ]` | geometric `J_g` |
| Body-frame error `e_b` | `log(T⁻¹ T_d)^∨` | body `J_b` |

`log` is the SO(3)/SE(3) logarithm. Euler angles are never subtracted.

## Robot model

* A serial manipulator has `n` joints and `n` moving **bodies**. Body `i` is attached to the base
  (`i = 0`) or to body `i - 1` by joint `i`.
* Joint `i` is described by:
  * `origin`: the fixed transform from the parent frame to the joint frame at `q_i = 0`;
  * `axis`: a unit vector in the joint frame;
  * `type`: *revolute* (rotation about `axis` by `q_i` rad) or *prismatic* (translation along `axis`
    by `q_i` m).
* The frame of body `i` **is** the joint frame after the joint motion (the URDF convention):

  ```text
  T_parent_body_i(q_i) = origin_i · motion_i(q_i)
  ```

* The end-effector (tool) frame is a fixed transform `T_flange_ee` from the last body frame.
* Inertial parameters of a body are its mass `m`, the centre of mass `c` in the body frame, and the
  rotational inertia `I_c` about the centre of mass, written in the body-frame axes.
* Collision geometry of a body is written in the body frame.

### Denavit–Hartenberg

Both DH conventions are supported. In each, joint `i` uses `θ_i = q_i + θ_offset,i` (revolute) or
`d_i = q_i + d_offset,i` (prismatic).

* **Standard (distal) DH** (Spong et al., Siciliano et al.):

  ```text
  T_{i-1}_{i} = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
  ```

* **Modified (proximal) DH** (Craig):

  ```text
  T_{i-1}_{i} = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
  ```

A DH table is converted exactly into the general joint description above. The direct DH product is
kept as an independent forward-kinematics implementation and is used to cross-check the general
chain in the tests.

### Product of exponentials

* Space form: `T_s_ee(q) = e^{[S_1]q_1} ··· e^{[S_n]q_n} M`
* Body form: `T_s_ee(q) = M e^{[B_1]q_1} ··· e^{[B_n]q_n}`, with `B_i = Ad_{M⁻¹} S_i`

`M` is the end-effector pose at `q = 0`. Screw axes are ordered `[ω; v]`. For a revolute joint
`S = [ω; -ω × r]`, where `r` is any point on the axis. For a prismatic joint `S = [0; v]`.

## Dynamics

The joint-space equations of motion are

```text
M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext
```

* `h(q, q̇) = C(q, q̇) q̇ + g(q)` is the **bias vector**. ManipulatorCore computes `h` and `g`
  exactly with the Recursive Newton–Euler Algorithm. The matrix `C` is not unique. When an explicit
  `C` is needed (for example, to check that `Ṁ - 2C` is skew-symmetric), the Christoffel-symbol
  construction is used.
* An external wrench `F` (`[m; f]`, base frame, reference point given by the matching Jacobian)
  maps to joint torques as `τ_ext = Jᵀ F`.
* Spatial vectors in the recursive algorithms are written in body-frame coordinates and ordered
  `[ω; v]`. Motion vectors are transformed by `Ad`, force vectors by its dual.

## Numerical tolerances

* Rank and singularity decisions use singular values from a full SVD. The default rank tolerance is
  `σ_max · max(m, n) · ε_machine`, and callers can override it.
* Linear systems are solved with factorizations (`LLT`/`LDLT` for symmetric positive-definite
  matrices, `ColPivHouseholderQR` or SVD otherwise). An explicit inverse is never formed to solve
  `A x = b`.

## Error-handling policy

* **Invalid input or configuration** (wrong vector sizes, non-finite numbers, inconsistent models,
  negative masses, non-unit axes) throws `mc::InvalidArgument`, which derives from
  `std::invalid_argument`.
* **Expected algorithmic outcomes** (an IK solve that does not converge, a planner that runs out of
  iterations, an unreachable target) are **not** exceptions. They are reported through result
  objects that carry a status enum and diagnostics.
* No public function returns NaN or infinity for finite, valid input.
* Library code never prints. Only applications, demos and benchmarks write to stdout.

## Randomness

Randomized algorithms (sampling planners, multi-start IK, randomized tests) take an explicit seed
through a configuration struct. They use `mc::Random`, which wraps `std::mt19937_64` and maps its
raw output to uniform numbers itself. Standard-library distributions are not used, because their
output is not specified identically across standard-library implementations. The same seed
therefore yields bit-identical samples on every platform.
