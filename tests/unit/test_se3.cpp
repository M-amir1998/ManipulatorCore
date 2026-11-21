#include "manipulatorcore/geometry/se3.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

TEST(SE3, IdentityAndInverse) {
  Random rng(1);
  for (int i = 0; i < 100; ++i) {
    const SE3 T = test::randomPose(rng, 2.0);
    EXPECT_POSE_NEAR(T * T.inverse(), SE3::Identity(), 1e-14);
    EXPECT_POSE_NEAR(T.inverse() * T, SE3::Identity(), 1e-14);
    EXPECT_MATRIX_NEAR((T.matrix() * T.inverse().matrix()).eval(), Mat4::Identity(), 1e-14);
  }
}

TEST(SE3, CompositionFollowsFrameNotation) {
  // T_a_c = T_a_b * T_b_c, and p_a = T_a_b * p_b.
  const SE3 T_a_b(rotZ(kHalfPi), Vec3(1.0, 0.0, 0.0));
  const SE3 T_b_c(Mat3::Identity(), Vec3(0.0, 2.0, 0.0));
  const SE3 T_a_c = T_a_b * T_b_c;
  // Origin of c is 2 along b's y axis, which is a's -x axis, offset by 1 along a's x.
  EXPECT_MATRIX_NEAR(T_a_c.translation(), Vec3(-1.0, 0.0, 0.0), 1e-15);
  EXPECT_MATRIX_NEAR(T_a_b * Vec3(1.0, 0.0, 0.0), Vec3(1.0, 1.0, 0.0), 1e-15);
}

TEST(SE3, MatrixRoundTrip) {
  Random rng(2);
  const SE3 T = test::randomPose(rng);
  EXPECT_POSE_NEAR(SE3::fromMatrix(T.matrix()), T, 1e-15);
}

TEST(SE3, FromMatrixRejectsInvalidInput) {
  Mat4 M = Mat4::Identity();
  M(0, 0) = 2.0;
  EXPECT_THROW((void)SE3::fromMatrix(M), InvalidArgument);
  Mat4 bad_row = Mat4::Identity();
  bad_row(3, 0) = 0.5;
  EXPECT_THROW((void)SE3::fromMatrix(bad_row), InvalidArgument);
}

TEST(SE3, HatVeeRoundTrip) {
  Random rng(3);
  const Vec6 V = test::randomTwist(rng);
  EXPECT_MATRIX_NEAR(veeSE3(hatSE3(V)), V, 0.0);
}

TEST(SE3, ExpOfPureTranslation) {
  Vec6 xi = Vec6::Zero();
  xi.tail<3>() = Vec3(1.0, -2.0, 0.5);
  const SE3 T = expSE3(xi);
  EXPECT_MATRIX_NEAR(T.rotation(), Mat3::Identity(), 0.0);
  EXPECT_MATRIX_NEAR(T.translation(), Vec3(1.0, -2.0, 0.5), 1e-15);
}

TEST(SE3, ExpOfRevoluteScrewAboutOffsetAxis) {
  // Screw axis: rotation about z through point r = (1, 0, 0): S = [w; -w x r].
  const Vec3 w = Vec3::UnitZ();
  const Vec3 r(1.0, 0.0, 0.0);
  Vec6 S;
  S << w, -w.cross(r);
  const SE3 T = expSE3(S * kPi);
  // Rotating the origin by pi about that axis moves it to (2, 0, 0).
  EXPECT_MATRIX_NEAR(T * Vec3::Zero(), Vec3(2.0, 0.0, 0.0), 1e-14);
  EXPECT_MATRIX_NEAR(T.rotation(), rotZ(kPi), 1e-14);
}

TEST(SE3, ExpMatchesMatrixExponentialSeries) {
  Random rng(4);
  for (int trial = 0; trial < 20; ++trial) {
    const Vec6 xi = test::randomTwist(rng, 1.5);
    // Truncated power series of the 4x4 matrix exponential.
    const Mat4 X = hatSE3(xi);
    Mat4 term = Mat4::Identity();
    Mat4 sum = Mat4::Identity();
    for (int k = 1; k < 40; ++k) {
      term = term * X / static_cast<double>(k);
      sum += term;
    }
    EXPECT_MATRIX_NEAR(expSE3(xi).matrix(), sum, 1e-12);
  }
}

TEST(SE3, LogInvertsExp) {
  Random rng(5);
  for (int i = 0; i < 300; ++i) {
    Vec6 xi = test::randomTwist(rng, 2.0);
    const double angle = xi.head<3>().norm();
    if (angle >= kPi - 1e-6) {
      xi.head<3>() *= (kPi - 0.1) / angle;
    }
    EXPECT_MATRIX_NEAR(logSE3(expSE3(xi)), xi, 1e-10);
  }
}

TEST(SE3, ExpInvertsLog) {
  Random rng(6);
  for (int i = 0; i < 300; ++i) {
    const SE3 T = test::randomPose(rng, 3.0);
    EXPECT_POSE_NEAR(expSE3(logSE3(T)), T, 1e-11);
  }
}

TEST(SE3, AdjointMapsTwistMatrices) {
  // [Ad_T V] = T [V] T^{-1}
  Random rng(7);
  for (int i = 0; i < 50; ++i) {
    const SE3 T = test::randomPose(rng);
    const Vec6 V = test::randomTwist(rng);
    const Mat4 lhs = hatSE3(T.adjoint() * V);
    const Mat4 rhs = T.matrix() * hatSE3(V) * T.inverse().matrix();
    EXPECT_MATRIX_NEAR(lhs, rhs, 1e-13);
    EXPECT_MATRIX_NEAR(transformTwist(T, V), (T.adjoint() * V).eval(), 1e-14);
  }
}

TEST(SE3, AdjointIsGroupHomomorphism) {
  Random rng(8);
  const SE3 A = test::randomPose(rng);
  const SE3 B = test::randomPose(rng);
  EXPECT_MATRIX_NEAR(adjoint(A * B), (adjoint(A) * adjoint(B)).eval(), 1e-13);
  EXPECT_MATRIX_NEAR(adjoint(A.inverse()), adjoint(A).inverse().eval(), 1e-12);
}

TEST(SE3, WrenchTransformPreservesPower) {
  // Power F^T V is frame-independent.
  Random rng(9);
  for (int i = 0; i < 50; ++i) {
    const SE3 T_a_b = test::randomPose(rng);
    const Vec6 V_b = test::randomTwist(rng);
    const Vec6 F_b = test::randomTwist(rng);
    const double power_b = F_b.dot(V_b);
    const double power_a = transformWrench(T_a_b, F_b).dot(transformTwist(T_a_b, V_b));
    EXPECT_NEAR(power_a, power_b, 1e-13);
  }
}

TEST(SE3, LieBracketMatchesMatrixCommutator) {
  Random rng(10);
  const Vec6 V1 = test::randomTwist(rng);
  const Vec6 V2 = test::randomTwist(rng);
  const Mat4 commutator = hatSE3(V1) * hatSE3(V2) - hatSE3(V2) * hatSE3(V1);
  EXPECT_MATRIX_NEAR(hatSE3(adSE3(V1) * V2), commutator, 1e-14);
}

TEST(SE3, PoseDistance) {
  const SE3 A(rotZ(0.1), Vec3(0.0, 0.0, 0.0));
  const SE3 B(rotZ(0.6), Vec3(3.0, 4.0, 0.0));
  const PoseDistance d = poseDistance(A, B);
  EXPECT_NEAR(d.translation, 5.0, 1e-14);
  EXPECT_NEAR(d.rotation, 0.5, 1e-14);
}

}  // namespace
}  // namespace mc
