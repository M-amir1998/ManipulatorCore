# Collision detection

## Purpose

Signed distances, closest points and boolean collision queries between geometric primitives, and a
robot-level checker for link-environment and self-collision used by IK filtering, planning,
trajectory verification and visualization.

## Mathematical formulation

Primitives (in their local frame, placed by a pose): sphere (radius $r$), capsule (segment
$[-h, h]$ along $z$ with radius $r$), box (half extents $b$) and half-space
$\{x : n^\top x \le d\}$. Spheres and capsules are **rounded cores** (a point or a segment inflated by
$r$), so their distances reduce to distances between cores minus the radii.

| Pair | Method | Exactness |
| --- | --- | --- |
| sphere/capsule vs sphere/capsule | closest points of two segments (Ericson 5.1.9) | exact, signed |
| sphere/capsule vs box | golden-section minimization of the box signed-distance function along the core segment | exact when separated; penetration = deepest core point |
| box vs box | GJK when separated; separating-axis theorem (15 axes) overlap when penetrating | exact when separated; SAT depth when penetrating |
| anything vs half-space | support point of the core in direction $-n$, minus the radius | exact, signed |

The signed distance of a point to a box with pose $(R, c)$: with $\ell = R^\top(p - c)$ and
$q = |\ell| - b$, $\;\mathrm{sd} = \lVert\max(q, 0)\rVert + \min(\max_i q_i, 0)$. It is convex, so
its minimum along a segment is found reliably by golden-section search (80 iterations, interval
$< 10^{-13}$), with an explicit endpoint check.

GJK iterates on the Minkowski difference $A - B$ with support mappings
$s_{A-B}(d) = s_A(d) - s_B(-d)$; the simplex is reduced to the sub-simplex closest to the origin
(Ericson's closest-point-on-triangle region tests; tetrahedra test the origin against each face).
Witness points are the barycentric combinations of the stored support points.

## Assumptions

* All geometry is convex; robots are approximated by capsules (built-in models) and environments by
  boxes, spheres, capsules and half-spaces. Meshes are not supported.
* Queries are discrete (static configurations). Continuous (swept) collision checking is not
  implemented; planners check edges at a configurable resolution and the pose-goal pipeline adds an
  environment clearance to cover the motion between checks.

## Conventions

Distances are signed (negative = penetration). `DistanceResult::normal` points from A towards B.
Robot bodies are indexed $0 \dots n-1$, the base is $-1$.

## Algorithm

`RobotCollisionChecker` evaluates forward kinematics once per query, poses every collision shape and
computes its axis-aligned bounding box (AABB).

* **Broad phase**: a pair is skipped only if the gap between the AABBs is **strictly positive** and
  not smaller than the current cutoff. An AABB gap bounds the *unsigned* distance; overlapping boxes
  give a bound of zero while the signed distance may be negative, so zero bounds never prune.
* **Allowed-collision matrix**: adjacent bodies ($|i - j| = 1$, including base/body 0) are excluded by
  default because their capsules overlap at the shared joint; `allowCollision(a, b)` adds exclusions.
* **Margins**: `safety_margin` applies to robot-environment pairs, `self_collision_margin` to
  self-pairs (kept separate because the built-in arms have small permanent clearances between
  non-adjacent wrist bodies).
* Queries: `isCollisionFree` (early exit), `minimumDistance` (cutoff shrinks with the best distance),
  `pairDistances` (all pairs below a report distance, sorted) and `bodyClearances` (per-link minima).

## Implementation

* [distance.hpp](../../include/manipulatorcore/collision/distance.hpp): `computeDistance`,
  `shapesCollide`, `closestPointsSegmentSegment`, `signedDistancePointBox`, `gjkDistance`.
* [aabb.hpp](../../include/manipulatorcore/collision/aabb.hpp): bounds and the lower-bound distance.
* [robot_collision.hpp](../../include/manipulatorcore/collision/robot_collision.hpp): `RobotCollisionChecker`.

## Numerical considerations

* Degenerate segments (spheres, zero-length capsules) and parallel segments are handled explicitly.
* When two cores touch, the normal falls back to the direction between shape centres.
* GJK terminates on a relative progress test ($\lVert v\rVert^2 - v^\top w \le 10^{-12}\lVert v\rVert^2$),
  on repeated support points, or when the origin lies in the simplex.

## Configuration

`CollisionCheckerOptions`: `check_self_collision` (true), `check_environment` (true),
`check_base_against_environment` (false), `safety_margin` (0), `self_collision_margin` (0).

## Example

```cpp
#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

using namespace mc;
RobotCollisionChecker checker(makeRedundant7R(),
                              {makeBox(SE3::fromTranslation(Vec3(0.6, 0.0, 0.3)), Vec3(0.1, 0.2, 0.3), "crate"),
                               makeHalfSpace(Vec3::UnitZ(), 0.0, "floor")});
const VecX q = redundant7RHome();
const bool free = checker.isCollisionFree(q);
const double clearance = checker.minimumDistance(q);
for (const CollisionPairInfo& p : checker.pairDistances(q, 0.1)) { /* p.result.point_a, point_b */ }
```

## Benchmark

See [benchmarks](../benchmarks/README.md) (`BM_CollisionFreeQuery`, `BM_MinimumDistanceQuery`).

## Limitations

* Convex primitives only; no meshes, octrees or point clouds.
* Penetration depth is approximate for capsule-box and box-box pairs.
* No continuous collision detection; the broad phase is a flat AABB loop (adequate for tens of
  shapes, not for large scenes).

Validated in [test_collision.cpp](../../tests/unit/test_collision.cpp): analytic pairs, brute-force
capsule-box comparison, GJK against analytic spheres, box-box witness points, half-space cases,
broad-phase regression, and agreement between boolean and distance queries on random configurations.

## References

* C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005 (ch. 4-5, 9).
* E. G. Gilbert, D. W. Johnson and S. S. Keerthi, "A fast procedure for computing the distance between complex objects in three-dimensional space", *IEEE Journal of Robotics and Automation*, 4(2), 1988.
* S. Gottschalk, M. C. Lin and D. Manocha, "OBBTree: a hierarchical structure for rapid interference detection", SIGGRAPH 1996 (separating-axis test for oriented boxes).
