# Robot model

## Purpose

`RobotModel` describes a serial manipulator: one single-degree-of-freedom joint per moving body,
inertial parameters, collision primitives, joint limits, a tool frame and gravity. Kinematics,
dynamics, collision checking, planning, control and simulation all read this one description. The
model can be built directly, from a standard or modified Denavit–Hartenberg (DH) table, from one of
four built-in generic arms, or from a URDF document. It can also be exported to URDF.

## Mathematical formulation

### Joints and bodies

Body $i$ ($i = 0,\dots,n-1$) is attached to its parent (the base for $i = 0$, otherwise body $i-1$)
by joint $i$. With the fixed joint origin $T_{p,j}$ and the unit axis $a$ written in the joint frame:

```math
T_{p,i}(q_i) = T_{p,j}\,\mathrm{motion}_i(q_i), \qquad
\mathrm{motion}_i(q_i) = \begin{cases} (\exp([a]q_i),\ 0) & \text{revolute} \\ (I,\ a\,q_i) & \text{prismatic} \end{cases}
```

The body frame is the joint frame after the motion (the URDF convention). The motion subspace, in
the body frame and ordered $(\omega, v)$, is $S_i = (a, 0)$ for a revolute joint and $S_i = (0, a)$
for a prismatic joint. The end-effector pose is $T_{se} = T_{s,n-1}\,T_{\text{flange},e}$.

### Inertial parameters

A body has mass $m$, centre of mass $c$ (body frame) and rotational inertia $I_c$ about the centre
of mass, in body axes. The spatial inertia about the body origin, ordered $(\omega, v)$, is

```math
G = \begin{bmatrix} I_c - m[c]^2 & m[c] \\ -m[c] & m I \end{bmatrix},
```

so that the body's momentum is $G\mathcal{V}_b$ and its kinetic energy is
$\tfrac{1}{2}\mathcal{V}_b^\top G \mathcal{V}_b$. Further operations:

- `inertiaAboutOrigin()` $= I_c - m[c]^2$ (parallel-axis theorem);
- `combine(a, b)`: $m = m_a + m_b$, $c = (m_a c_a + m_b c_b)/m$, $I = \sum_k \big(I_{c,k} - m_k [c_k - c]^2\big)$, symmetrized;
- `transformed(T_a_body)`: $c_a = T\,c$, $I_a = R I_c R^\top$;
- primitives: point mass, solid cylinder between two points ($\tfrac{1}{2}mr^2$ axially,
  $m(3r^2 + \ell^2)/12$ transversally, rotated onto the segment), solid box
  ($\tfrac{m}{12}(s_y^2 + s_z^2)$, ...), solid sphere ($\tfrac{2}{5}mr^2$).

### Denavit–Hartenberg conversion

`dhTransform` evaluates one row with $\theta = \theta_\text{off} + q$ (revolute) or $d = d_\text{off} + q$ (prismatic):

```math
\text{standard: } T_{i-1,i} = R_z(\theta_i)\,T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i),
\qquad
\text{modified: } T_{i-1,i} = R_x(\alpha_{i-1})\,T_x(a_{i-1})\,R_z(\theta_i)\,T_z(d_i).
```

`robotFromDH` converts a table exactly into the general joint description, with every joint axis
$+z$ in its joint frame. Because $R_z(\theta)$ and $T_z(d)$ commute, the constant part of a row can
precede the joint motion. Rows are numbered $i = 1,\dots,n$ as in DH notation, and row $i$ defines
model joint $i-1$ (0-based):

| Convention | Joint origin $T_{p,j}$ of row $i$ | Tool transform |
| --- | --- | --- |
| Standard | $T_x(a_{i-1})R_x(\alpha_{i-1})\cdot T_z(d_i)R_z(\theta_i)$ (the first factor is the identity for $i = 1$) | $T_x(a_{n})R_x(\alpha_{n})\cdot T_\text{tool}$ |
| Modified | $R_x(\alpha_{i-1})T_x(a_{i-1})\cdot T_z(d_i)R_z(\theta_i)$ | $T_\text{tool}$ |

In the standard convention, the trailing $T_x(a_i)R_x(\alpha_i)$ of row $i$ therefore belongs to the
origin of joint $i+1$, and the last one belongs to the tool frame. The model's body frame $i$
differs from DH frame $i$. `dhFrameInBody(table, i)` returns DH frame $i$ in body frame $i$:
$T_x(a_i)R_x(\alpha_i)$ (standard) or the identity (modified). Data given in DH frames, such as
link inertias, must be transformed with it.

## Assumptions

- Serial chains only. Every joint is revolute or prismatic with one degree of freedom.
- Inertial parameters are physically consistent: $m \ge 0$, $I_c$ symmetric positive semi-definite,
  and the principal moments satisfy the triangle inequality. Massless bodies are allowed, but
  forward dynamics then needs armature or a distal mass to keep $M(q)$ positive definite (see
  [dynamics](dynamics.md)).
- Joint damping (`Joint::damping`) is a plant property used only by the [simulator](simulation.md).
  Armature (`Joint::armature`, the reflected rotor inertia) is part of the rigid-body model and is
  added to the diagonal of $M(q)$.

## Conventions

See [conventions](../mathematics/conventions.md). The model uses $T_{ab}$ = pose of $b$ in $a$,
joint frames after motion as body frames (URDF convention), motion subspaces and spatial inertias
ordered $(\omega, v)$, gravity as a base-frame vector (default $(0, 0, -9.81)$ m/s²), and SI units.
Revolute joint values are in rad and prismatic values in m. Limits of revolute joints are in rad,
rad/s, rad/s², N·m; for prismatic joints they are in m, m/s, m/s², N.

## Algorithm

```text
robotFromDH(table):
  fixed <- identity
  for each row i:
     joint_i.axis <- +z;  joint_i.type, limits <- row
     if standard:  joint_i.origin <- fixed * Tz(d_i) * Rz(theta_i);  fixed <- Tx(a_i) * Rx(alpha_i)
     else:         joint_i.origin <- Rx(alpha_i) * Tx(a_i) * Tz(d_i) * Rz(theta_i)   # row stores a_{i-1}, alpha_{i-1}
  tool <- (standard ? fixed : identity) * table.tool
  bodies get zero inertia and no collision geometry

fromURDF(xml):
  parse links and joints; each link has at most one parent joint
  base <- options.base_link or the unique root link
  chain <- tip-to-base walk if options.tip_link is set, else follow children until the chain ends
           (throws if a link has more than one child)
  pending <- identity                                  # fixed transforms since the last movable joint
  for each joint along the chain:
     if fixed:  pending <- pending * origin
     else:      new body; joint.origin <- pending * origin; pending <- identity
     merge the child link's collision shapes and inertia (transformed by pending) into the current
     body, or into the base collision set before the first movable joint
  tool transform <- pending (the trailing fixed joints)
```

## Implementation

Headers: [joint.hpp](../../include/manipulatorcore/model/joint.hpp),
[inertia.hpp](../../include/manipulatorcore/model/inertia.hpp),
[robot_model.hpp](../../include/manipulatorcore/model/robot_model.hpp),
[dh.hpp](../../include/manipulatorcore/model/dh.hpp),
[builtin_robots.hpp](../../include/manipulatorcore/model/builtin_robots.hpp),
[urdf.hpp](../../include/manipulatorcore/model/urdf.hpp).
Sources: [joint.cpp](../../src/model/joint.cpp), [inertia.cpp](../../src/model/inertia.cpp),
[robot_model.cpp](../../src/model/robot_model.cpp), [dh.cpp](../../src/model/dh.cpp),
[builtin_robots.cpp](../../src/model/builtin_robots.cpp), [urdf.cpp](../../src/model/urdf.cpp),
[xml_lite.cpp](../../src/model/xml_lite.cpp) (internal XML reader).

```cpp
RobotModel(std::string name, std::vector<Body> bodies, const SE3& T_flange_ee = SE3::Identity(),
           const Vec3& gravity = Vec3(0.0, 0.0, -kStandardGravity));
int dof() const;  const Joint& joint(int i) const;  const SE3& toolTransform() const;
const VecX& lowerLimits() const;  /* upper, velocity, acceleration, effort likewise */
bool withinLimits(const VecX& q, double tolerance = 0.0) const;
VecX clampToLimits(const VecX& q) const;  VecX midConfiguration() const;

SE3 dhTransform(DHConvention convention, const DHRow& row, double q);
RobotModel robotFromDH(const DHTable& table, const std::string& name, const Vec3& gravity = ...);
SE3 dhFrameInBody(const DHTable& table, int body);

std::string toURDF(const RobotModel& model, const UrdfExportOptions& options = {});
RobotModel  fromURDF(std::string_view urdf_xml, const UrdfImportOptions& options = {});
RobotModel  loadURDF(const std::string& path, const UrdfImportOptions& options = {});
```

### Validation rules

The constructor and every mutator validate their input and throw `InvalidArgument`:

- joint axis finite with $\lvert\lVert a\rVert - 1\rvert < 10^{-9}$; origin rotation valid within $10^{-6}$;
- position limits finite with lower $\le$ upper; velocity, acceleration and effort limits finite and
  strictly positive; damping and armature finite and non-negative;
- inertia: mass finite and $\ge 0$; $I_c$ symmetric within $10^{-9}\max(1, \max\lvert I_c\rvert)$;
  principal moments $\ge -\text{tol}$ and $\lambda_1 + \lambda_2 \ge \lambda_3 - \text{tol}$ (sorted ascending);
- tool transform valid; gravity finite; at least one body; all collision shapes valid.

### Built-in robots

Generic models with plausible parameters. None reproduces a commercial robot.

| Factory | DOF | Kinematics | Mass, inertia | Limits | Gravity, extras |
| --- | --- | --- | --- | --- | --- |
| `makePlanar2R()` | 2 | revolute about $+z$, links along $+x$; $\ell = (1.0, 0.8)$ m; tool at the tip of link 2 | $m = (2.0, 1.5)$ kg, uniform solid cylinders of radius 0.04 m | $\pm\pi$ rad, 3 rad/s, 15 rad/s², 150 N·m | $(0, -9.81, 0)$ (in the arm plane); one capsule per link |
| `makePlanar3R()` | 3 | as above, $\ell = (1.0, 0.8, 0.4)$ m | $m = (2.0, 1.5, 0.6)$ kg | as above | as above |
| `makeIndustrial6R()` | 6 | elbow arm with spherical wrist from `industrial6RDHTable()` (standard DH, table below); tool $T_z(0.05)$ | segmented cylinders, 20/15/10/3/1.5/0.8 kg; armature 0.5/0.5/0.3/0.1/0.1/0.05; damping 1.0/1.0/0.8/0.3/0.3/0.1 | from the DH table | $(0, 0, -9.81)$; capsule links and a pedestal capsule (radius 0.13 m, 0.30 m tall) on the base; home `industrial6RHome()` $= (0, 0.25, -0.45, 0, -1.2, 0)$ |
| `makeRedundant7R()` | 7 | S-R-S arm, axes $z, y, z, y, z, y, z$, joint offsets along $z$: 0.15, 0.21, 0.21, 0.21, 0.20, 0.20, 0.08 m; tool $T_z(0.10)$; points straight up at $q = 0$ (singular) | 4.0/4.0/3.0/2.7/1.7/1.8/0.7 kg; armature 0.20/0.20/0.15/0.15/0.08/0.08/0.04 | $\pm 170°$ (joints 1, 3, 5), $\pm 120°$ (joints 2, 4, 6), $\pm 175°$ (joint 7); velocity 2.0 to 3.0 rad/s; effort 180 to 40 N·m | $(0, 0, -9.81)$; capsule links, base capsule; home `redundant7RHome()` $= (0, 0.5, 0, 1.5, 0, 1.0, 0)$ |

`industrial6RDHTable()` (standard DH; limits as lower/upper in degrees, then velocity rad/s,
acceleration rad/s², effort N·m):

| $i$ | $a$ [m] | $\alpha$ | $d$ [m] | $\theta_\text{off}$ | limits |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.15 | $-\pi/2$ | 0.45 | 0 | $\pm 170$, 2.5, 8, 400 |
| 2 | 0.60 | 0 | 0 | $-\pi/2$ | $\pm 110$, 2.5, 8, 400 |
| 3 | 0.10 | $-\pi/2$ | 0 | 0 | $[-140, 80]$, 3, 10, 200 |
| 4 | 0 | $\pi/2$ | 0.62 | 0 | $\pm 175$, 4, 15, 60 |
| 5 | 0 | $-\pi/2$ | 0 | 0 | $\pm 120$, 4, 15, 60 |
| 6 | 0 | 0 | 0.10 | 0 | $\pm 175$, 5, 20, 30 |

Link inertias of the built-in arms are sums of solid cylinders along the link segments. The mass
of a segment is proportional to its length. The capsule collision geometry is built along the same
segments.

### URDF export and import

Export (`toURDF`) writes a base link holding the base collision shapes, one `revolute`/`prismatic`
joint and link per body, and a fixed joint to the tool link. Origins are written as `xyz` plus
`rpy`, with $R = R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})$. Numbers use the shortest
representation that round-trips exactly (`std::to_chars`). Each link carries an `<inertial>` block
with the centre of mass and $I_c$ in link axes. Joint damping goes to `<dynamics damping>`.
Acceleration limits and armature, which URDF cannot express, go to a
`<manipulatorcore acceleration=... armature=.../>` element that other tools ignore. A capsule is
written as a `<cylinder>` of the core length. With `include_visuals`, the visuals add the two end
caps as spheres. Half-spaces have no URDF equivalent and are omitted.

Import (`fromURDF`, `loadURDF`) supports the subset needed for serial chains:

- joint types `revolute`, `continuous`, `prismatic`, and `fixed`. Fixed joints are merged: their
  transforms are folded into the next movable joint's origin, and their child links' inertias (by
  `Inertia::combine`) and collision shapes are added to the preceding body. Trailing fixed joints
  become the tool transform. `floating`, `planar` and any other type throw;
- `<collision>` primitives `sphere`, `box`, `cylinder` and `capsule`. A cylinder of radius $r$ and
  length $\ell$ becomes a capsule with the same radius and core length $\ell$. This over-approximates
  the volume by a hemisphere at each end, which is conservative for collision checking. Meshes and
  `<visual>` elements are ignored;
- missing `<axis>` defaults to $+x$ (URDF default). Continuous joints get
  $\pm\tfrac{1}{2}$`continuous_joint_range`. Other movable joints require `<limit>`, whose missing
  `lower`/`upper` default to 0. Non-positive or missing velocity/effort limits fall back to the
  option defaults. Acceleration limit and armature are read from the `<manipulatorcore>` element when
  present;
- the chain runs from `base_link` (default: the unique root) to `tip_link` (default: follow children
  until the chain ends; a branching link throws).

The XML reader handles elements, attributes, character data, comments, processing instructions,
DOCTYPE and CDATA. It does not support namespaces, DTD processing or external entities.

### Validation

- [test_model.cpp](../../tests/unit/test_model.cpp): `Joint.MotionAndSubspace`, `Joint.ValidationRejectsBadInput`;
  `Inertia.SpatialMatrixIsSymmetricPositiveDefinite`, `Inertia.KineticEnergyMatchesCentroidalFormula`,
  `Inertia.CylinderMomentsAreCorrect`, `Inertia.CombineMatchesParallelAxisTheorem`,
  `Inertia.TransformedPreservesSpatialInertia`, `Inertia.ValidationRejectsUnphysicalParameters`;
  `RobotModel.LimitsAndClamping`, `RobotModel.BuiltinRobotsAreValid`, `RobotModel.RejectsEmptyModels`.
- DH: `DH.StandardTransformMatchesDefinition` and `DH.ModifiedTransformMatchesDefinition` (one row
  against the defining product). `DHEquivalence.ChainModelReproducesDirectDHProduct`, for both
  conventions on a mixed revolute/prismatic table with non-zero offsets: on 50 random configurations,
  every body frame times `dhFrameInBody` equals the directly multiplied DH frame within $10^{-12}$, and
  so does the end-effector pose. `DH.Industrial6RMatchesItsDHTable`.
- URDF: `URDF.RoundTripPreservesKinematicsAndDynamics` (all built-in robots: origins and poses within
  $10^{-12}$; limits, damping and armature exactly; inertias within $10^{-14}$; same number of
  collision shapes), `URDF.MergesFixedJointsAndSelectsChain` (fixed-joint folding, inertia merging,
  continuous-joint range, tool from trailing fixed joints, branching without `tip_link` throws),
  `URDF.RejectsMalformedDocuments`.

## Numerical considerations

- `combine` symmetrizes the result, $\tfrac{1}{2}(I + I^\top)$, to remove round-off asymmetry.
- `solidCylinder` aligns the cylinder axis with the minimal rotation taking $+z$ to the segment
  direction. Anti-parallel segments use $R_x(\pi)$.
- URDF numbers are parsed with the classic locale, and every attribute must contain exactly the
  expected count of finite numbers. Export writes shortest round-trip decimals, so export followed by
  import reproduces joint origins to round-off. The RPY conversion is the only inexact step.
- The rotation tolerance for joint origins and the tool transform is $10^{-6}$, looser than the
  $10^{-9}$ default of `isRotationMatrix`, to accept rotations parsed from limited-precision text.

## Configuration

`JointLimits` (defaults):

| Field | Default | Meaning |
| --- | --- | --- |
| `lower`, `upper` | $-\pi$, $\pi$ | position limits [rad or m] |
| `velocity` | 2.0 | [rad/s or m/s] |
| `acceleration` | 10.0 | [rad/s² or m/s²] |
| `effort` | 100.0 | [N·m or N] |

`UrdfExportOptions`:

| Field | Default | Meaning |
| --- | --- | --- |
| `base_link` | `"base_link"` | name of the root link |
| `tool_link` | `"tool0"` | name of the tool link (fixed joint `<tool_link>_joint`) |
| `include_visuals` | `true` | emit `<visual>` elements mirroring the collision shapes |

`UrdfImportOptions`:

| Field | Default | Meaning |
| --- | --- | --- |
| `base_link` | empty | empty: the unique root link |
| `tip_link` | empty | empty: follow the chain until it ends (must not branch) |
| `continuous_joint_range` | $2\pi$ | position range of continuous joints [rad] |
| `default_velocity_limit` | 2.0 | used when missing or non-positive |
| `default_acceleration_limit` | 10.0 | used without the `<manipulatorcore>` extension |
| `default_effort_limit` | 100.0 | used when missing or non-positive |
| `gravity` | $(0, 0, -9.81)$ | gravity of the imported model |

`Planar2RParameters` / `Planar3RParameters` defaults are listed in the built-in robots table.

## Example

```cpp
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"
#include "manipulatorcore/model/dh.hpp"
#include "manipulatorcore/model/urdf.hpp"

#include <string>

int main() {
  using namespace mc;
  // Kinematic model from a standard DH table; frames follow the RobotModel convention.
  const DHTable table = industrial6RDHTable();
  const RobotModel dh_model = robotFromDH(table, "from_dh");
  const VecX q = industrial6RHome();
  const bool same_pose = endEffectorPose(dh_model, q).isApprox(dhForwardKinematics(table, q), 1e-12);

  // Full model (inertias, collision geometry) exported to URDF and read back.
  const RobotModel arm = makeIndustrial6R();
  const std::string xml = toURDF(arm);
  UrdfImportOptions options;
  options.gravity = arm.gravity();
  const RobotModel parsed = fromURDF(xml, options);

  return (same_pose && parsed.dof() == arm.dof() && parsed.withinLimits(q)) ? 0 : 1;
}
```

## Benchmark

See [benchmarks](../benchmarks/README.md) for measured timings and how to reproduce them.

## Limitations

- Serial chains only. No branching trees, closed loops, mimic joints, floating or planar joints, or
  multi-DOF joints. A URDF tree must reduce to a chain, selected with `tip_link` when it branches.
- URDF import ignores meshes and `<visual>` elements, maps cylinders to capsules (conservative
  over-approximation), ignores `<dynamics friction>`, `<safety_controller>`, `<calibration>` and
  `<transmission>`, and reads only one inertial block per link. Half-spaces cannot be exported.
- `robotFromDH` produces kinematics only. Inertias are zero and no collision geometry is attached.
- Joint friction is limited to viscous damping, and only the simulator applies it. There is no
  Coulomb friction, no gear backlash, and no actuator model beyond the effort limit.
- The built-in robots are generic test articles. Their parameters are plausible but are not
  identified from any real robot.

## References

- K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Ch. 4 (product of exponentials, URDF), ch. 8 (spatial inertia).
- M. W. Spong, S. Hutchinson and M. Vidyasagar, *Robot Modeling and Control*, Wiley, 2006 (2nd ed. 2020). Standard DH convention.
- J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson Prentice Hall, 2005. Modified DH convention.
- B. Siciliano, L. Sciavicco, L. Villani and G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. Ch. 2 (DH convention), ch. 7 (inertial parameters).
- R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008. Ch. 2 (spatial inertia).
