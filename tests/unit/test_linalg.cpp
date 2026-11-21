#include "manipulatorcore/math/linalg.hpp"

#include "manipulatorcore/math/error.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace mc {
namespace {

MatX randomMatrix(Random& rng, Index rows, Index cols) {
  MatX A(rows, cols);
  for (Index i = 0; i < A.size(); ++i) {
    A(i) = rng.uniform(-1.0, 1.0);
  }
  return A;
}

TEST(Linalg, PseudoInverseSatisfiesPenroseConditions) {
  Random rng(61);
  for (const auto& [rows, cols] : {std::pair<Index, Index>{6, 7}, {6, 6}, {3, 2}, {6, 3}}) {
    const MatX A = randomMatrix(rng, rows, cols);
    const MatX P = pseudoInverse(A);
    EXPECT_MATRIX_NEAR((A * P * A).eval(), A, 1e-12);
    EXPECT_MATRIX_NEAR((P * A * P).eval(), P, 1e-12);
    EXPECT_MATRIX_NEAR((A * P).transpose().eval(), (A * P).eval(), 1e-12);
    EXPECT_MATRIX_NEAR((P * A).transpose().eval(), (P * A).eval(), 1e-12);
  }
}

TEST(Linalg, PseudoInverseOfRankDeficientMatrix) {
  MatX A(3, 3);
  A << 1, 2, 3,  //
      2, 4, 6,   //
      1, 0, 1;
  const SvdMetrics m = computeSvdMetrics(A);
  EXPECT_EQ(m.rank, 2);
  const MatX P = pseudoInverse(A);
  EXPECT_TRUE(P.allFinite());
  EXPECT_MATRIX_NEAR((A * P * A).eval(), A, 1e-12);
}

TEST(Linalg, MinimumNormSolutionMatchesPseudoInverse) {
  Random rng(67);
  const MatX A = randomMatrix(rng, 6, 7);
  VecX b(6);
  for (Index i = 0; i < 6; ++i) {
    b(i) = rng.uniform(-1, 1);
  }
  const VecX x = solveMinimumNorm(A, b);
  EXPECT_MATRIX_NEAR(x, (pseudoInverse(A) * b).eval(), 1e-12);
  EXPECT_MATRIX_NEAR((A * x).eval(), b, 1e-12);
}

TEST(Linalg, DampedLeastSquaresMatchesNormalEquations) {
  Random rng(71);
  for (const auto& [rows, cols] : {std::pair<Index, Index>{6, 7}, {6, 4}}) {
    const MatX A = randomMatrix(rng, rows, cols);
    VecX b(rows);
    for (Index i = 0; i < rows; ++i) {
      b(i) = rng.uniform(-1, 1);
    }
    const double lambda = 0.3;
    // Reference: minimize |Ax - b|^2 + lambda^2 |x|^2 via (A^T A + lambda^2 I) x = A^T b.
    const MatX H = A.transpose() * A + lambda * lambda * MatX::Identity(cols, cols);
    const VecX reference = H.ldlt().solve(A.transpose() * b);
    EXPECT_MATRIX_NEAR(solveDampedLeastSquares(A, b, lambda), reference, 1e-12);
    EXPECT_MATRIX_NEAR((dampedPseudoInverse(A, lambda) * b).eval(), reference, 1e-12);
  }
}

TEST(Linalg, DampedLeastSquaresIsBoundedAtSingularity) {
  // A singular Jacobian-like matrix: the pseudoinverse of a nearly singular matrix explodes,
  // damped least squares stays bounded by |b| / (2 lambda).
  MatX A(2, 2);
  A << 1.0, 1.0,  //
      1.0, 1.0 + 1e-10;
  const VecX b = Vec2(0.0, 1.0);
  const double lambda = 0.05;
  const VecX x = solveDampedLeastSquares(A, b, lambda);
  EXPECT_TRUE(x.allFinite());
  EXPECT_LE(x.norm(), b.norm() / (2.0 * lambda) + 1e-12);
}

TEST(Linalg, NullSpaceProjector) {
  Random rng(73);
  const MatX A = randomMatrix(rng, 6, 7);
  const MatX N = nullSpaceProjector(A);
  EXPECT_MATRIX_NEAR((A * N).eval(), MatX::Zero(6, 7), 1e-12);
  EXPECT_MATRIX_NEAR((N * N).eval(), N, 1e-12);  // idempotent
  EXPECT_TRUE(isSymmetric(N, 1e-12));
}

TEST(Linalg, SvdMetricsConditionNumber) {
  MatX A = MatX::Zero(2, 2);
  A(0, 0) = 4.0;
  A(1, 1) = 0.5;
  const SvdMetrics m = computeSvdMetrics(A);
  EXPECT_NEAR(m.condition_number, 8.0, 1e-14);
  A(1, 1) = 0.0;
  EXPECT_TRUE(std::isinf(computeSvdMetrics(A).condition_number));
  EXPECT_EQ(computeSvdMetrics(A).rank, 1);
}

TEST(Linalg, PositiveDefiniteness) {
  MatX A(2, 2);
  A << 2.0, 0.5,  //
      0.5, 1.0;
  EXPECT_TRUE(isPositiveDefinite(A));
  A(1, 1) = -1.0;
  EXPECT_FALSE(isPositiveDefinite(A));
  A(0, 1) = 3.0;  // not symmetric
  EXPECT_FALSE(isSymmetric(A));
}

TEST(Linalg, NumericalJacobianOfLinearMap) {
  MatX A(2, 3);
  A << 1, 2, 3,  //
      -1, 0.5, 4;
  const auto f = [&A](const VecX& x) -> VecX { return A * x; };
  EXPECT_MATRIX_NEAR(numericalJacobian(f, Eigen::Vector3d(0.1, 0.2, 0.3)), A, 1e-9);
}

TEST(Linalg, RejectsNonFiniteInput) {
  MatX A = MatX::Identity(2, 2);
  A(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)pseudoInverse(A), InvalidArgument);
  EXPECT_THROW((void)solveDampedLeastSquares(MatX::Identity(2, 2), Vec2(1, 1), -1.0), InvalidArgument);
  EXPECT_THROW((void)solveMinimumNorm(MatX::Identity(2, 2), VecX::Ones(3)), InvalidArgument);
}

}  // namespace
}  // namespace mc
