#include "manipulatorcore/geometry/interpolation.hpp"

#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

TEST(Interpolation, SlerpEndpoints) {
  Random rng(51);
  const Quat q0 = rng.uniformQuaternion();
  const Quat q1 = rng.uniformQuaternion();
  EXPECT_NEAR(std::abs(slerp(q0, q1, 0.0).dot(q0)), 1.0, 1e-14);
  EXPECT_NEAR(std::abs(slerp(q0, q1, 1.0).dot(q1)), 1.0, 1e-14);
}

TEST(Interpolation, SlerpHasConstantAngularVelocity) {
  const Quat q0 = quaternionFromRotation(Mat3::Identity());
  const Quat q1 = quaternionFromRotation(rotZ(2.0));
  for (const double s : {0.1, 0.25, 0.5, 0.9}) {
    EXPECT_MATRIX_NEAR(rotationFromQuaternion(slerp(q0, q1, s)), rotZ(2.0 * s), 1e-12);
  }
}

TEST(Interpolation, SlerpTakesShortestArc) {
  const Quat q0 = quaternionFromRotation(Mat3::Identity());
  const Quat q1 = quaternionFromRotation(rotZ(0.5));
  const Quat q1_negated(-q1.w(), -q1.x(), -q1.y(), -q1.z());
  EXPECT_MATRIX_NEAR(rotationFromQuaternion(slerp(q0, q1_negated, 0.5)), rotZ(0.25), 1e-12);
}

TEST(Interpolation, SlerpAgreesWithGeodesic) {
  Random rng(53);
  for (int i = 0; i < 50; ++i) {
    const Mat3 R0 = rng.uniformQuaternion().toRotationMatrix();
    const Mat3 R1 = rng.uniformQuaternion().toRotationMatrix();
    const double s = rng.uniform();
    const Mat3 via_slerp = rotationFromQuaternion(slerp(quaternionFromRotation(R0), quaternionFromRotation(R1), s));
    EXPECT_MATRIX_NEAR(via_slerp, interpolateRotation(R0, R1, s), 1e-9);
  }
}

TEST(Interpolation, DecoupledPoseMovesOriginOnStraightLine) {
  const SE3 T0(Mat3::Identity(), Vec3(0.0, 0.0, 0.0));
  const SE3 T1(rotZ(1.0), Vec3(1.0, 2.0, 0.0));
  const SE3 mid = interpolatePoseDecoupled(T0, T1, 0.5);
  EXPECT_MATRIX_NEAR(mid.translation(), Vec3(0.5, 1.0, 0.0), 1e-15);
  EXPECT_MATRIX_NEAR(mid.rotation(), rotZ(0.5), 1e-14);
}

TEST(Interpolation, ScrewInterpolationEndpoints) {
  Random rng(57);
  const SE3 T0 = test::randomPose(rng);
  const SE3 T1 = test::randomPose(rng);
  EXPECT_POSE_NEAR(interpolatePoseScrew(T0, T1, 0.0), T0, 1e-12);
  EXPECT_POSE_NEAR(interpolatePoseScrew(T0, T1, 1.0), T1, 1e-11);
}

}  // namespace
}  // namespace mc
