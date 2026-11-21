#include "manipulatorcore/geometry/rotation.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <array>

namespace mc {
namespace {

TEST(Rotation, QuaternionMatrixRoundTrip) {
  Random rng(31);
  for (int i = 0; i < 500; ++i) {
    const Quat q = rng.uniformQuaternion();
    const Mat3 R = rotationFromQuaternion(q);
    EXPECT_TRUE(isRotationMatrix(R, 1e-12));
    EXPECT_MATRIX_NEAR(R, q.toRotationMatrix(), 1e-14);  // agrees with Eigen
    const Quat back = quaternionFromRotation(R);
    EXPECT_GE(back.w(), 0.0);
    EXPECT_NEAR(std::abs(back.dot(q)), 1.0, 1e-12);  // same rotation up to sign
    EXPECT_MATRIX_NEAR(rotationFromQuaternion(back), R, 1e-13);
  }
}

TEST(Rotation, QuaternionFromRotationCoversAllShepperdBranches) {
  // Rotations by ~pi about each axis exercise the x, y and z branches; small ones the trace branch.
  const std::array<Mat3, 4> cases = {rotX(3.1), rotY(3.1), rotZ(3.1), rotX(0.1)};
  for (const Mat3& R : cases) {
    EXPECT_MATRIX_NEAR(rotationFromQuaternion(quaternionFromRotation(R)), R, 1e-14);
  }
}

TEST(Rotation, QuaternionOfPiRotation) {
  const Quat q = quaternionFromRotation(rotZ(kPi));
  EXPECT_NEAR(q.w(), 0.0, 1e-15);
  EXPECT_NEAR(std::abs(q.z()), 1.0, 1e-15);
}

TEST(Rotation, QuaternionHamiltonConvention) {
  // i * j = k under the Hamilton convention.
  const Quat i(0.0, 1.0, 0.0, 0.0);
  const Quat j(0.0, 0.0, 1.0, 0.0);
  const Quat k = i * j;
  EXPECT_NEAR(k.w(), 0.0, 0.0);
  EXPECT_NEAR(k.z(), 1.0, 0.0);
  // Quaternion (cos(t/2), 0, 0, sin(t/2)) is a rotation by t about +z.
  const double t = 0.8;
  EXPECT_MATRIX_NEAR(rotationFromQuaternion(Quat(std::cos(t / 2), 0.0, 0.0, std::sin(t / 2))), rotZ(t), 1e-15);
}

TEST(Rotation, NormalizeQuaternion) {
  const Quat q = normalizeQuaternion(Quat(2.0, 0.0, 0.0, 0.0));
  EXPECT_NEAR(q.w(), 1.0, 0.0);
  EXPECT_THROW((void)normalizeQuaternion(Quat(0.0, 0.0, 0.0, 0.0)), InvalidArgument);
  EXPECT_THROW((void)normalizeQuaternion(Quat(std::nan(""), 0.0, 0.0, 1.0)), InvalidArgument);
}

TEST(Rotation, NormalizedQuaternionsHaveUnitNorm) {
  Random rng(37);
  for (int i = 0; i < 200; ++i) {
    const Quat q(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5));
    EXPECT_NEAR(normalizeQuaternion(q).norm(), 1.0, 1e-15);
  }
}

TEST(Rotation, AxisAngleRoundTrip) {
  Random rng(41);
  for (int i = 0; i < 200; ++i) {
    const Vec3 axis = rng.unitVector3();
    const double angle = rng.uniform(1e-3, kPi - 1e-3);
    const AxisAngle aa = axisAngleFromRotation(rotationFromAxisAngle(axis, angle));
    EXPECT_NEAR(aa.angle, angle, 1e-12);
    EXPECT_MATRIX_NEAR(aa.axis, axis, 1e-10);
  }
}

TEST(Rotation, AxisAngleOfIdentity) {
  const AxisAngle aa = axisAngleFromRotation(Mat3::Identity());
  EXPECT_EQ(aa.angle, 0.0);
  EXPECT_NEAR(aa.axis.norm(), 1.0, 0.0);
  EXPECT_THROW((void)rotationFromAxisAngle(Vec3::Zero(), 1.0), InvalidArgument);
}

class EulerRoundTrip : public ::testing::TestWithParam<EulerSequence> {};

TEST_P(EulerRoundTrip, RandomRotations) {
  Random rng(43);
  for (int i = 0; i < 500; ++i) {
    const Mat3 R = rng.uniformQuaternion().toRotationMatrix();
    const Vec3 angles = eulerFromRotation(R, GetParam());
    EXPECT_MATRIX_NEAR(rotationFromEuler(angles, GetParam()), R, 1e-12);
  }
}

TEST_P(EulerRoundTrip, GimbalLockReconstructs) {
  const EulerSequence seq = GetParam();
  const std::array<double, 2> locked_betas =
      seq == EulerSequence::ZYZ ? std::array<double, 2>{0.0, kPi} : std::array<double, 2>{kHalfPi, -kHalfPi};
  for (const double beta : locked_betas) {
    const Mat3 R = rotationFromEuler(Vec3(0.4, beta, -1.1), seq);
    const Vec3 angles = eulerFromRotation(R, seq);
    EXPECT_NEAR(angles(2), 0.0, 1e-12) << "gamma must be zero at gimbal lock";
    EXPECT_MATRIX_NEAR(rotationFromEuler(angles, seq), R, 1e-12);
  }
}

TEST_P(EulerRoundTrip, NearGimbalLockReconstructs) {
  const EulerSequence seq = GetParam();
  const double beta = seq == EulerSequence::ZYZ ? 1e-9 : kHalfPi - 1e-9;
  const Mat3 R = rotationFromEuler(Vec3(0.4, beta, -1.1), seq);
  EXPECT_MATRIX_NEAR(rotationFromEuler(eulerFromRotation(R, seq), seq), R, 1e-12);
}

INSTANTIATE_TEST_SUITE_P(AllSequences, EulerRoundTrip,
                         ::testing::Values(EulerSequence::ZYX, EulerSequence::XYZ, EulerSequence::ZYZ));

TEST(Rotation, EulerAnglesInDocumentedRanges) {
  Random rng(47);
  for (int i = 0; i < 300; ++i) {
    const Mat3 R = rng.uniformQuaternion().toRotationMatrix();
    const Vec3 zyx = eulerFromRotation(R, EulerSequence::ZYX);
    EXPECT_LE(std::abs(zyx(1)), kHalfPi + 1e-12);
    const Vec3 zyz = eulerFromRotation(R, EulerSequence::ZYZ);
    EXPECT_GE(zyz(1), 0.0);
    EXPECT_LE(zyz(1), kPi);
  }
}

TEST(Rotation, RpyMatchesZyxIntrinsic) {
  const double roll = 0.3;
  const double pitch = -0.5;
  const double yaw = 1.2;
  const Mat3 R = rotationFromRPY(roll, pitch, yaw);
  EXPECT_MATRIX_NEAR(R, rotationFromEuler(Vec3(yaw, pitch, roll), EulerSequence::ZYX), 1e-15);
  EXPECT_MATRIX_NEAR(rpyFromRotation(R), Vec3(roll, pitch, yaw), 1e-14);
}

}  // namespace
}  // namespace mc
