#include "manipulatorcore/geometry/shapes.hpp"

#include "manipulatorcore/math/error.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

TEST(Shapes, CapsuleBetweenPoints) {
  const Vec3 a(1.0, 0.0, 0.0);
  const Vec3 b(1.0, 2.0, 0.0);
  const Shape s = makeCapsule(a, b, 0.1, "link");
  const auto& capsule = std::get<Capsule>(s.geometry);
  EXPECT_NEAR(capsule.half_length, 1.0, 1e-15);
  // The local segment endpoints map back to a and b.
  EXPECT_MATRIX_NEAR(s.pose * Vec3(0.0, 0.0, -1.0), a, 1e-14);
  EXPECT_MATRIX_NEAR(s.pose * Vec3(0.0, 0.0, 1.0), b, 1e-14);
}

TEST(Shapes, CapsuleAntiParallelToZ) {
  const Shape s = makeCapsule(Vec3(0, 0, 1), Vec3(0, 0, -1), 0.2);
  EXPECT_MATRIX_NEAR(s.pose * Vec3(0.0, 0.0, 1.0), Vec3(0, 0, -1), 1e-14);
}

TEST(Shapes, DegenerateCapsuleIsSphereLike) {
  const Shape s = makeCapsule(Vec3(1, 1, 1), Vec3(1, 1, 1), 0.2);
  EXPECT_EQ(std::get<Capsule>(s.geometry).half_length, 0.0);
}

TEST(Shapes, ValidationRejectsBadDimensions) {
  EXPECT_THROW((void)makeSphere(Vec3::Zero(), -1.0), InvalidArgument);
  EXPECT_THROW((void)makeSphere(Vec3::Zero(), 0.0), InvalidArgument);
  EXPECT_THROW((void)makeBox(SE3::Identity(), Vec3(1.0, 0.0, 1.0)), InvalidArgument);
  EXPECT_THROW((void)makeCapsule(Vec3::Zero(), Vec3::UnitX(), 0.0), InvalidArgument);
  EXPECT_THROW((void)makeHalfSpace(Vec3::Zero(), 0.0), InvalidArgument);
}

TEST(Shapes, HalfSpaceNormalization) {
  const Shape s = makeHalfSpace(Vec3(0.0, 0.0, 2.0), 4.0);
  const auto& h = std::get<HalfSpace>(s.geometry);
  EXPECT_MATRIX_NEAR(h.normal, Vec3::UnitZ(), 0.0);
  EXPECT_NEAR(h.offset, 2.0, 0.0);
}

TEST(Shapes, TransformShapeComposesPose) {
  const Shape s = makeSphere(Vec3(1.0, 0.0, 0.0), 0.5);
  const Shape moved = transformShape(SE3::fromTranslation(Vec3(0.0, 2.0, 0.0)), s);
  EXPECT_MATRIX_NEAR(moved.pose.translation(), Vec3(1.0, 2.0, 0.0), 0.0);
}

}  // namespace
}  // namespace mc
