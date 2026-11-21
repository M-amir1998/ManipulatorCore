#include "manipulatorcore/collision/aabb.hpp"
#include "manipulatorcore/collision/distance.hpp"
#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/error.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace mc {
namespace {

// ------------------------------------------------------------------------------------------------
// Elementary queries
// ------------------------------------------------------------------------------------------------

TEST(Collision, SegmentSegmentKnownCases) {
  Vec3 c1;
  Vec3 c2;
  // Skew perpendicular segments, 1 apart along z.
  EXPECT_NEAR(closestPointsSegmentSegment(Vec3(-1, 0, 0), Vec3(1, 0, 0), Vec3(0, -1, 1), Vec3(0, 1, 1), c1, c2), 1.0,
              1e-15);
  EXPECT_MATRIX_NEAR(c1, Vec3(0, 0, 0), 1e-15);
  EXPECT_MATRIX_NEAR(c2, Vec3(0, 0, 1), 1e-15);
  // Parallel overlapping segments.
  EXPECT_NEAR(closestPointsSegmentSegment(Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(1, 0.5, 0), Vec3(3, 0.5, 0), c1, c2), 0.25,
              1e-15);
  // Endpoint regions.
  EXPECT_NEAR(closestPointsSegmentSegment(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 1, 0), Vec3(3, 1, 0), c1, c2), 2.0,
              1e-15);
  // Degenerate (points).
  EXPECT_NEAR(closestPointsSegmentSegment(Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 3, 4), Vec3(0, 3, 4), c1, c2), 25.0,
              1e-15);
}

TEST(Collision, PointBoxSignedDistance) {
  const SE3 T(rotZ(0.3), Vec3(1.0, 0.0, 0.0));
  const Vec3 h(0.5, 0.2, 0.1);
  Vec3 closest;
  // Straight above the top face.
  EXPECT_NEAR(signedDistancePointBox(T * Vec3(0.1, 0.0, 0.6), T, h, &closest), 0.5, 1e-14);
  EXPECT_MATRIX_NEAR(closest, (T * Vec3(0.1, 0.0, 0.1)).eval(), 1e-14);
  // Corner region.
  EXPECT_NEAR(signedDistancePointBox(T * Vec3(0.8, 0.6, 0.1), T, h, &closest), 0.5, 1e-14);
  // Inside: distance to the nearest face is negative.
  EXPECT_NEAR(signedDistancePointBox(T * Vec3(0.0, 0.0, 0.05), T, h, &closest), -0.05, 1e-14);
}

// ------------------------------------------------------------------------------------------------
// Primitive pairs
// ------------------------------------------------------------------------------------------------

TEST(Collision, SphereSphere) {
  const DistanceResult r =
      computeDistance(Sphere{0.5}, SE3::Identity(), Sphere{0.25}, SE3::fromTranslation(Vec3(2, 0, 0)));
  EXPECT_NEAR(r.distance, 1.25, 1e-15);
  EXPECT_MATRIX_NEAR(r.point_a, Vec3(0.5, 0, 0), 1e-15);
  EXPECT_MATRIX_NEAR(r.point_b, Vec3(1.75, 0, 0), 1e-15);
  EXPECT_MATRIX_NEAR(r.normal, Vec3(1, 0, 0), 1e-15);
  const DistanceResult p =
      computeDistance(Sphere{0.5}, SE3::Identity(), Sphere{0.5}, SE3::fromTranslation(Vec3(0.6, 0, 0)));
  EXPECT_NEAR(p.distance, -0.4, 1e-15);
}

TEST(Collision, CapsuleCapsuleCrossing) {
  const Shape a = makeCapsule(Vec3(-1, 0, 0), Vec3(1, 0, 0), 0.1);
  const Shape b = makeCapsule(Vec3(0, -1, 0.5), Vec3(0, 1, 0.5), 0.15);
  const DistanceResult r = computeDistance(a, SE3::Identity(), b, SE3::Identity());
  EXPECT_NEAR(r.distance, 0.25, 1e-14);
  EXPECT_MATRIX_NEAR(r.point_a, Vec3(0, 0, 0.1), 1e-14);
  EXPECT_MATRIX_NEAR(r.point_b, Vec3(0, 0, 0.35), 1e-14);
}

TEST(Collision, SymmetryOfDistance) {
  Random rng(301);
  const std::vector<ShapeGeometry> geoms = {Sphere{0.2}, Capsule{0.1, 0.3}, Box{Vec3(0.2, 0.1, 0.3)}};
  for (int k = 0; k < 50; ++k) {
    for (const auto& ga : geoms) {
      for (const auto& gb : geoms) {
        const SE3 Ta = test::randomPose(rng, 1.0);
        const SE3 Tb = test::randomPose(rng, 1.0);
        const DistanceResult ab = computeDistance(ga, Ta, gb, Tb);
        const DistanceResult ba = computeDistance(gb, Tb, ga, Ta);
        if (ab.distance > 1e-9) {  // penetration measures may differ in approximation; separation must agree
          EXPECT_NEAR(ab.distance, ba.distance, 1e-9);
          EXPECT_NEAR((ab.point_b - ab.point_a).norm(), ab.distance, 1e-9);
        }
      }
    }
  }
}

/// Brute-force minimum of the box SDF over points sampled on a capsule core minus the radius.
double bruteForceCapsuleBox(const Capsule& c, const SE3& Tc, const Vec3& h, const SE3& Tb) {
  double best = std::numeric_limits<double>::infinity();
  const int samples = 20000;
  for (int i = 0; i <= samples; ++i) {
    const double t = -1.0 + 2.0 * i / samples;
    best = std::min(best, signedDistancePointBox(Tc * Vec3(0, 0, t * c.half_length), Tb, h));
  }
  return best - c.radius;
}

TEST(Collision, CapsuleBoxMatchesBruteForce) {
  Random rng(307);
  const Vec3 h(0.3, 0.2, 0.1);
  const Capsule capsule{0.05, 0.4};
  for (int k = 0; k < 100; ++k) {
    const SE3 Tc = test::randomPose(rng, 0.8);
    const SE3 Tb = test::randomPose(rng, 0.3);
    const double expected = bruteForceCapsuleBox(capsule, Tc, h, Tb);
    const DistanceResult r = computeDistance(capsule, Tc, Box{h}, Tb);
    EXPECT_NEAR(r.distance, expected, 1e-4) << "trial " << k;
    if (r.distance > 0.0) {
      EXPECT_NEAR((r.point_b - r.point_a).norm(), r.distance, 1e-9);
    }
  }
}

TEST(Collision, GjkMatchesAnalyticSpheres) {
  Random rng(311);
  for (int k = 0; k < 50; ++k) {
    const Vec3 ca(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1));
    const Vec3 cb(rng.uniform(-1, 1) + 3.0, rng.uniform(-1, 1), rng.uniform(-1, 1));
    const double ra = rng.uniform(0.1, 0.5);
    const double rb = rng.uniform(0.1, 0.5);
    const auto sa = [&](const Vec3& d) -> Vec3 { return ca + ra * d.normalized(); };
    const auto sb = [&](const Vec3& d) -> Vec3 { return cb + rb * d.normalized(); };
    const GjkResult g = gjkDistance(sa, sb, ca - cb, 200);
    EXPECT_FALSE(g.intersecting);
    EXPECT_NEAR(g.distance, (cb - ca).norm() - ra - rb, 1e-5);  // curved sets converge asymptotically
  }
}

TEST(Collision, BoxBoxSeparatedWitnessPoints) {
  Random rng(313);
  const Vec3 ha(0.3, 0.2, 0.1);
  const Vec3 hb(0.15, 0.25, 0.2);
  int separated = 0;
  for (int k = 0; k < 200; ++k) {
    const SE3 Ta = test::randomPose(rng, 0.6);
    const SE3 Tb = test::randomPose(rng, 0.6);
    const DistanceResult r = computeDistance(Box{ha}, Ta, Box{hb}, Tb);
    if (r.distance > 1e-6) {
      ++separated;
      // Witness points lie on the respective surfaces and realize the distance.
      EXPECT_NEAR(signedDistancePointBox(r.point_a, Ta, ha), 0.0, 1e-9);
      EXPECT_NEAR(signedDistancePointBox(r.point_b, Tb, hb), 0.0, 1e-9);
      EXPECT_NEAR((r.point_b - r.point_a).norm(), r.distance, 1e-9);
      // No corner of A is closer to B than the reported distance.
      for (int c = 0; c < 8; ++c) {
        const Vec3 corner(c & 1 ? ha.x() : -ha.x(), c & 2 ? ha.y() : -ha.y(), c & 4 ? ha.z() : -ha.z());
        EXPECT_GE(signedDistancePointBox(Ta * corner, Tb, hb), r.distance - 1e-9);
      }
    } else {
      EXPECT_LE(r.distance, 0.0);
    }
  }
  EXPECT_GT(separated, 50);
}

TEST(Collision, BoxBoxOverlapDepth) {
  const DistanceResult r = computeDistance(Box{Vec3(0.5, 0.5, 0.5)}, SE3::Identity(), Box{Vec3(0.5, 0.5, 0.5)},
                                           SE3::fromTranslation(Vec3(0.8, 0.1, 0)));
  EXPECT_NEAR(r.distance, -0.2, 1e-12);
  EXPECT_MATRIX_NEAR(r.normal, Vec3(1, 0, 0), 1e-12);
}

TEST(Collision, HalfSpacePairs) {
  const Shape floor = makeHalfSpace(Vec3::UnitZ(), 0.0);
  EXPECT_NEAR(computeDistance(Sphere{0.1}, SE3::fromTranslation(Vec3(0, 0, 0.5)), floor.geometry, floor.pose).distance,
              0.4, 1e-15);
  EXPECT_NEAR(computeDistance(Sphere{0.1}, SE3::fromTranslation(Vec3(0, 0, 0.05)), floor.geometry, floor.pose).distance,
              -0.05, 1e-15);
  // Box rotated 45 degrees about x: lowest edge at z = c - sqrt(2) * 0.5 * h.
  const SE3 T(rotX(kPi / 4), Vec3(0, 0, 1.0));
  const DistanceResult r = computeDistance(floor.geometry, floor.pose, Box{Vec3(0.5, 0.5, 0.5)}, T);
  EXPECT_NEAR(r.distance, 1.0 - std::sqrt(0.5), 1e-12);
  EXPECT_MATRIX_NEAR(r.normal, Vec3(0, 0, 1), 1e-15);  // from the floor (A) towards the box (B)
  const Shape tilted_capsule = makeCapsule(Vec3(0, 0, 0.3), Vec3(1, 0, -0.2), 0.05);
  EXPECT_NEAR(computeDistance(tilted_capsule, SE3::Identity(), floor, SE3::Identity()).distance, -0.25, 1e-15);
  EXPECT_THROW((void)computeDistance(floor, SE3::Identity(), floor, SE3::Identity()), InvalidArgument);
}

TEST(Collision, AabbBounds) {
  const AABB s = AABB::of(Sphere{0.5}, SE3::fromTranslation(Vec3(1, 2, 3)));
  EXPECT_MATRIX_NEAR(s.min, Vec3(0.5, 1.5, 2.5), 0.0);
  const AABB b = AABB::of(Box{Vec3(1, 1, 1)}, SE3::fromTranslation(Vec3(5, 2, 3)));
  EXPECT_FALSE(s.overlaps(b));
  EXPECT_NEAR(s.distanceLowerBound(b), 2.5, 1e-15);
  EXPECT_TRUE(s.overlaps(b, 2.6));
  const AABB plane = AABB::of(HalfSpace{}, SE3::Identity());
  EXPECT_TRUE(plane.isUnbounded());
  EXPECT_TRUE(plane.overlaps(s));
  // A rotated box's AABB contains all of its corners.
  Random rng(317);
  const SE3 T = test::randomPose(rng);
  const Vec3 h(0.3, 0.1, 0.2);
  const AABB rb = AABB::of(Box{h}, T);
  for (int c = 0; c < 8; ++c) {
    const Vec3 p = T * Vec3(c & 1 ? h.x() : -h.x(), c & 2 ? h.y() : -h.y(), c & 4 ? h.z() : -h.z());
    EXPECT_TRUE(((p.array() >= rb.min.array() - 1e-12) && (p.array() <= rb.max.array() + 1e-12)).all());
  }
}

// ------------------------------------------------------------------------------------------------
// Robot collision checking
// ------------------------------------------------------------------------------------------------

TEST(RobotCollision, BuiltinHomeAndZeroConfigurationsAreSelfCollisionFree) {
  for (const RobotModel& model : test::allBuiltinRobots()) {
    const RobotCollisionChecker checker(model);
    EXPECT_TRUE(checker.isCollisionFree(model.zeroConfiguration())) << model.name();
  }
  EXPECT_TRUE(RobotCollisionChecker(makeIndustrial6R()).isCollisionFree(industrial6RHome()));
  EXPECT_TRUE(RobotCollisionChecker(makeRedundant7R()).isCollisionFree(redundant7RHome()));
}

TEST(RobotCollision, ObstacleNearEndEffector) {
  const RobotModel model = makeRedundant7R();
  const VecX q = redundant7RHome();
  const Vec3 tip = endEffectorPose(model, q).translation();
  RobotCollisionChecker checker(model, {makeSphere(tip, 0.05, "at-tip")});
  EXPECT_FALSE(checker.isCollisionFree(q));
  EXPECT_LT(checker.minimumDistance(q), 0.0);
  checker.setObstacles({makeSphere(tip + Vec3(0.0, 0.0, 2.0), 0.05, "far")});
  EXPECT_TRUE(checker.isCollisionFree(q));
  // The overall minimum includes the arm's own (small) self-clearance; the environment alone is far.
  EXPECT_GT(checker.minimumDistance(q), 0.0);
  checker.setOptions({.check_self_collision = false});
  EXPECT_GT(checker.minimumDistance(q), 1.0);
}

TEST(RobotCollision, PenetratingPairsAreNotPrunedByBroadPhase) {
  // Regression: overlapping AABBs give a zero lower bound while the signed distance is negative.
  const RobotModel model = makePlanar2R();
  RobotCollisionChecker checker(model, {makeSphere(Vec3(0.5, 0.0, 0.0), 0.1)});
  EXPECT_FALSE(checker.isCollisionFree(Vec2::Zero()));
  EXPECT_NEAR(checker.minimumDistance(Vec2::Zero()), -0.14, 1e-12);
}

TEST(RobotCollision, Planar2RDistanceToSphereIsExact) {
  // Arm stretched along x (links are capsules of radius 0.04 on the x axis).
  const RobotModel model = makePlanar2R();
  RobotCollisionChecker checker(model, {makeSphere(Vec3(0.5, 0.6, 0.0), 0.1)});
  EXPECT_NEAR(checker.minimumDistance(Vec2::Zero()), 0.6 - 0.1 - 0.04, 1e-12);
  const auto pairs = checker.pairDistances(Vec2::Zero(), 10.0);
  ASSERT_FALSE(pairs.empty());
  EXPECT_EQ(pairs.front().body_a, 0);
  EXPECT_NEAR(pairs.front().result.distance, 0.46, 1e-12);
  for (std::size_t i = 1; i < pairs.size(); ++i) {
    EXPECT_LE(pairs[i - 1].result.distance, pairs[i].result.distance);
  }
}

TEST(RobotCollision, SafetyMarginTreatsNearContactAsCollision) {
  const RobotModel model = makePlanar2R();
  RobotCollisionChecker checker(model, {makeSphere(Vec3(0.5, 0.2, 0.0), 0.1)});  // clearance 0.06
  EXPECT_TRUE(checker.isCollisionFree(Vec2::Zero()));
  checker.setOptions({.safety_margin = 0.1});
  EXPECT_FALSE(checker.isCollisionFree(Vec2::Zero()));
}

TEST(RobotCollision, SelfCollisionOfFoldedPlanar3R) {
  const RobotModel model = makePlanar3R();
  RobotCollisionChecker checker(model);
  // Fold link 2 back onto link 1 and link 3 forward again: link 3 crosses link 1.
  const VecX folded = Vec3(0.0, 3.0, 3.0);
  EXPECT_FALSE(checker.isCollisionFree(folded));
  checker.allowCollision(0, 2);
  EXPECT_TRUE(checker.isCollisionAllowed(2, 0));
  EXPECT_TRUE(checker.isCollisionFree(folded));
}

TEST(RobotCollision, AdjacentBodiesAreExcludedByDefault) {
  const RobotCollisionChecker checker(makeRedundant7R());
  EXPECT_TRUE(checker.isCollisionAllowed(-1, 0));
  EXPECT_TRUE(checker.isCollisionAllowed(3, 4));
  EXPECT_FALSE(checker.isCollisionAllowed(2, 4));
  EXPECT_THROW(RobotCollisionChecker(makePlanar2R()).allowCollision(0, 5), InvalidArgument);
}

TEST(RobotCollision, DistanceAndBooleanQueriesAgree) {
  const RobotModel model = makeIndustrial6R();
  const std::vector<Shape> obstacles = {makeBox(SE3::fromTranslation(Vec3(0.6, 0.0, 0.4)), Vec3(0.15, 0.4, 0.4)),
                                        makeSphere(Vec3(0.3, 0.5, 1.0), 0.15), makeHalfSpace(Vec3::UnitZ(), 0.0)};
  const RobotCollisionChecker checker(model, obstacles);
  Random rng(331);
  int collisions = 0;
  for (int k = 0; k < 300; ++k) {
    const VecX q = test::randomConfiguration(model, rng);
    const bool free = checker.isCollisionFree(q);
    const double d = checker.minimumDistance(q);
    EXPECT_EQ(free, d >= 0.0) << "distance " << d;
    collisions += free ? 0 : 1;
    const VecX clearances = checker.bodyClearances(q);
    EXPECT_NEAR(clearances.minCoeff(), d, 1e-12);
  }
  EXPECT_GT(collisions, 10);
  EXPECT_LT(collisions, 290);
}

}  // namespace
}  // namespace mc
