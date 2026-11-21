// Numerical edge cases of the Lie-group maps: tiny angles, angles at and near pi, and accumulated
// round-off from long products.

#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace mc {
namespace {

TEST(LieNumerics, ExpLogAtTinyAngles) {
  Random rng(101);
  for (const double angle : {1e-15, 1e-12, 1e-9, 1e-6, 1e-4, 9.99e-4, 1.001e-3}) {
    for (int i = 0; i < 10; ++i) {
      const Vec3 phi = rng.unitVector3() * angle;
      const Mat3 R = expSO3(phi);
      EXPECT_TRUE(isRotationMatrix(R, 1e-14));
      // Relative accuracy of the recovered rotation vector.
      EXPECT_LE((logSO3(R) - phi).norm(), 1e-12 * angle + 1e-17) << "angle = " << angle;
    }
  }
}

TEST(LieNumerics, ExpLogContinuityAcrossSeriesThreshold) {
  // The Taylor and closed-form branches must agree where they meet.
  const Vec3 axis = Vec3(1.0, 2.0, -0.5).normalized();
  const Mat3 below = expSO3(axis * (1e-3 - 1e-12));
  const Mat3 above = expSO3(axis * (1e-3 + 1e-12));
  EXPECT_MATRIX_NEAR(below, above, 1e-11);
  EXPECT_MATRIX_NEAR(leftJacobianSO3(axis * (1e-3 - 1e-12)), leftJacobianSO3(axis * (1e-3 + 1e-12)), 1e-11);
  EXPECT_MATRIX_NEAR(leftJacobianInverseSO3(axis * (1e-3 - 1e-12)), leftJacobianInverseSO3(axis * (1e-3 + 1e-12)),
                     1e-11);
}

TEST(LieNumerics, LogAtExactlyPi) {
  const std::array<Vec3, 4> axes = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ(), Vec3(1.0, -1.0, 2.0).normalized()};
  for (const Vec3& axis : axes) {
    const Mat3 R = rotationFromAxisAngle(axis, kPi);
    const Vec3 phi = logSO3(R);
    EXPECT_NEAR(phi.norm(), kPi, 1e-12);
    // Either sign of the axis is a valid answer at exactly pi.
    EXPECT_NEAR(std::abs(phi.normalized().dot(axis)), 1.0, 1e-12);
    EXPECT_MATRIX_NEAR(expSO3(phi), R, 1e-12);
  }
}

TEST(LieNumerics, LogNearPi) {
  Random rng(103);
  for (const double delta : {1e-12, 1e-9, 1e-6, 1e-4, 5e-3, 9.9e-3, 1.01e-2, 0.1}) {
    for (int i = 0; i < 20; ++i) {
      const Vec3 axis = rng.unitVector3();
      const double angle = kPi - delta;
      const Vec3 phi = logSO3(expSO3(axis * angle));
      EXPECT_NEAR(phi.norm(), angle, 1e-10) << "delta = " << delta;
      EXPECT_GT(phi.normalized().dot(axis), 1.0 - 1e-9) << "axis sign must be preserved below pi";
    }
  }
}

TEST(LieNumerics, SE3LogNearPiRotation) {
  Random rng(107);
  for (int i = 0; i < 50; ++i) {
    const SE3 T(rotationFromAxisAngle(rng.unitVector3(), kPi - 1e-7), Vec3(0.3, -0.2, 1.0));
    EXPECT_POSE_NEAR(expSE3(logSE3(T)), T, 1e-9);
  }
}

TEST(LieNumerics, QuaternionNearPiAndIdentity) {
  for (const double angle : {0.0, 1e-12, kPi - 1e-12, kPi}) {
    const Mat3 R = rotationFromAxisAngle(Vec3(0.2, 0.3, -0.9), angle);
    const Quat q = quaternionFromRotation(R);
    EXPECT_NEAR(q.norm(), 1.0, 1e-15);
    EXPECT_MATRIX_NEAR(rotationFromQuaternion(q), R, 1e-14);
  }
}

TEST(LieNumerics, LongCompositionStaysNearSO3) {
  // 100k compositions of small rotations: drift away from SO(3) must stay at round-off level, and
  // projectToSO3 must restore orthonormality.
  Mat3 R = Mat3::Identity();
  const Mat3 step = expSO3(Vec3(1e-3, -2e-3, 0.5e-3));
  for (int i = 0; i < 100000; ++i) {
    R = R * step;
  }
  EXPECT_TRUE(isRotationMatrix(R, 1e-10));
  EXPECT_TRUE(isRotationMatrix(projectToSO3(R), 1e-14));
}

}  // namespace
}  // namespace mc
