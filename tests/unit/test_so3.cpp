#include "manipulatorcore/geometry/so3.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace mc {
namespace {

TEST(SO3, HatProducesCrossProduct) {
  const Vec3 a(0.3, -1.2, 2.0);
  const Vec3 b(-0.7, 0.4, 0.9);
  EXPECT_MATRIX_NEAR(hat(a) * b, a.cross(b), 1e-15);
  EXPECT_MATRIX_NEAR(hat(a).transpose(), (-hat(a)).eval(), 0.0);
}

TEST(SO3, VeeInvertsHat) {
  const Vec3 w(1.5, -0.25, 3.0);
  EXPECT_MATRIX_NEAR(vee(hat(w)), w, 0.0);
}

TEST(SO3, ExpOfZeroIsIdentity) { EXPECT_MATRIX_NEAR(expSO3(Vec3::Zero()), Mat3::Identity(), 0.0); }

TEST(SO3, ExpMatchesElementaryRotations) {
  for (const double angle : {-2.5, -0.3, 0.0, 0.7, 1.9, kPi}) {
    EXPECT_MATRIX_NEAR(expSO3(Vec3::UnitX() * angle), rotX(angle), 1e-14);
    EXPECT_MATRIX_NEAR(expSO3(Vec3::UnitY() * angle), rotY(angle), 1e-14);
    EXPECT_MATRIX_NEAR(expSO3(Vec3::UnitZ() * angle), rotZ(angle), 1e-14);
  }
}

TEST(SO3, ElementaryRotationsAreCounterClockwise) {
  // Rotating x by +90 degrees about z gives y (right-hand rule).
  EXPECT_MATRIX_NEAR(rotZ(kHalfPi) * Vec3::UnitX(), Vec3::UnitY(), 1e-15);
  EXPECT_MATRIX_NEAR(rotX(kHalfPi) * Vec3::UnitY(), Vec3::UnitZ(), 1e-15);
  EXPECT_MATRIX_NEAR(rotY(kHalfPi) * Vec3::UnitZ(), Vec3::UnitX(), 1e-15);
}

TEST(SO3, ExpProducesRotationMatrices) {
  Random rng(7);
  for (int i = 0; i < 200; ++i) {
    const Vec3 phi = rng.unitVector3() * rng.uniform(0.0, 3.0 * kPi);
    const Mat3 R = expSO3(phi);
    EXPECT_TRUE(isRotationMatrix(R, 1e-12)) << "phi = " << phi.transpose();
    EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
    EXPECT_MATRIX_NEAR(R.transpose() * R, Mat3::Identity(), 1e-12);
  }
}

TEST(SO3, LogInvertsExpForAnglesBelowPi) {
  Random rng(11);
  for (int i = 0; i < 500; ++i) {
    const Vec3 phi = rng.unitVector3() * rng.uniform(0.0, kPi - 1e-6);
    EXPECT_MATRIX_NEAR(logSO3(expSO3(phi)), phi, 1e-10);
  }
}

TEST(SO3, ExpInvertsLog) {
  Random rng(13);
  for (int i = 0; i < 500; ++i) {
    const Mat3 R = rng.uniformQuaternion().toRotationMatrix();
    EXPECT_MATRIX_NEAR(expSO3(logSO3(R)), R, 1e-12);
  }
}

TEST(SO3, LogAngleIsInZeroPi) {
  Random rng(17);
  for (int i = 0; i < 200; ++i) {
    const Vec3 phi = rng.unitVector3() * rng.uniform(0.0, 4.0 * kPi);
    const double angle = logSO3(expSO3(phi)).norm();
    EXPECT_GE(angle, 0.0);
    EXPECT_LE(angle, kPi + 1e-12);
  }
}

TEST(SO3, LeftJacobianInverse) {
  Random rng(19);
  for (int i = 0; i < 100; ++i) {
    const Vec3 phi = rng.unitVector3() * rng.uniform(0.0, kPi);
    EXPECT_MATRIX_NEAR(leftJacobianSO3(phi) * leftJacobianInverseSO3(phi), Mat3::Identity(), 1e-10);
  }
}

TEST(SO3, LeftJacobianMatchesFiniteDifference) {
  // exp(phi + d) ~= exp(J_l(phi) d) exp(phi)
  Random rng(23);
  const double h = 1e-6;
  for (int i = 0; i < 20; ++i) {
    const Vec3 phi = rng.unitVector3() * rng.uniform(0.1, 2.5);
    Mat3 numeric;
    for (int k = 0; k < 3; ++k) {
      const Vec3 d = Vec3::Unit(k) * h;
      numeric.col(k) = logSO3(expSO3(phi + d) * expSO3(phi).transpose()) / h;
    }
    EXPECT_MATRIX_NEAR(numeric, leftJacobianSO3(phi), 1e-5);
  }
}

TEST(SO3, ProjectToSO3RecoversRotation) {
  Random rng(29);
  const Mat3 R = rng.uniformQuaternion().toRotationMatrix();
  Mat3 noisy = R;
  noisy(0, 1) += 1e-4;
  noisy(2, 0) -= 2e-4;
  const Mat3 projected = projectToSO3(noisy);
  EXPECT_TRUE(isRotationMatrix(projected, 1e-12));
  EXPECT_MATRIX_NEAR(projected, R, 5e-4);
}

TEST(SO3, IsRotationMatrixRejectsReflections) {
  Mat3 reflection = Mat3::Identity();
  reflection(2, 2) = -1.0;
  EXPECT_FALSE(isRotationMatrix(reflection));
  EXPECT_FALSE(isRotationMatrix(2.0 * Mat3::Identity()));
}

TEST(SO3, RotationDistanceIsGeodesicAngle) {
  EXPECT_NEAR(rotationDistance(rotZ(0.2), rotZ(1.1)), 0.9, 1e-14);
  EXPECT_NEAR(rotationDistance(Mat3::Identity(), rotX(kPi)), kPi, 1e-12);
}

}  // namespace
}  // namespace mc
