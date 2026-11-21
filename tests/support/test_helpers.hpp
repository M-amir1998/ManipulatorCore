#pragma once

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

}  // namespace mc::test

#define EXPECT_MATRIX_NEAR(a, b, tol) EXPECT_PRED_FORMAT3(::mc::test::matrixNear, a, b, tol)
#define ASSERT_MATRIX_NEAR(a, b, tol) ASSERT_PRED_FORMAT3(::mc::test::matrixNear, a, b, tol)
