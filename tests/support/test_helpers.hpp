#pragma once

#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/math/types.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace mc::test {

/// GoogleTest predicate: max-abs difference between two matrices is within tolerance.
template <typename A, typename B>
::testing::AssertionResult matrixNear(const char* expr_a, const char* expr_b, const char* /*expr_tol*/,
                                      const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b,
                                      double tolerance) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return ::testing::AssertionFailure() << expr_a << " and " << expr_b << " have different shapes (" << a.rows()
                                         << "x" << a.cols() << " vs " << b.rows() << "x" << b.cols() << ")";
  }
  const MatX diff = (a.template cast<double>() - b.template cast<double>()).cwiseAbs();
  const double err = diff.size() == 0 ? 0.0 : diff.maxCoeff();
  if (err <= tolerance && a.allFinite() && b.allFinite()) {
    return ::testing::AssertionSuccess();
  }
  std::ostringstream os;
  os << expr_a << " and " << expr_b << " differ by " << err << " (tolerance " << tolerance << ")\n"
     << expr_a << " =\n"
     << a << "\n"
     << expr_b << " =\n"
     << b;
  return ::testing::AssertionFailure() << os.str();
}

inline ::testing::AssertionResult poseNear(const char* expr_a, const char* expr_b, const char* expr_tol,
                                           const SE3& a, const SE3& b, double tolerance) {
  return matrixNear(expr_a, expr_b, expr_tol, a.matrix(), b.matrix(), tolerance);
}

/// Random rigid transform with translation in [-scale, scale]^3.
inline SE3 randomPose(Random& rng, double scale = 1.0) {
  return SE3::fromQuaternion(rng.uniformQuaternion(),
                             Vec3(rng.uniform(-scale, scale), rng.uniform(-scale, scale), rng.uniform(-scale, scale)));
}

/// Random twist with components in [-scale, scale].
inline Vec6 randomTwist(Random& rng, double scale = 1.0) {
  Vec6 v;
  for (Index i = 0; i < 6; ++i) {
    v(i) = rng.uniform(-scale, scale);
  }
  return v;
}

}  // namespace mc::test

#define EXPECT_MATRIX_NEAR(a, b, tol) EXPECT_PRED_FORMAT3(::mc::test::matrixNear, a, b, tol)
#define ASSERT_MATRIX_NEAR(a, b, tol) ASSERT_PRED_FORMAT3(::mc::test::matrixNear, a, b, tol)
#define EXPECT_POSE_NEAR(a, b, tol) EXPECT_PRED_FORMAT3(::mc::test::poseNear, a, b, tol)
