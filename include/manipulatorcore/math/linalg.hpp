#pragma once

/// @file linalg.hpp
/// @brief Numerically robust linear-algebra kernels used by kinematics, IK and control.
///
/// Rank decisions and pseudoinverses use a full SVD with an explicit tolerance. Systems are
/// solved with factorizations; explicit inverses are never formed to solve A x = b.

#include "manipulatorcore/math/types.hpp"

#include <functional>

namespace mc {

/// Default singular-value threshold: sigma_max * max(rows, cols) * machine epsilon.
[[nodiscard]] double defaultRankTolerance(double sigma_max, Index rows, Index cols);

/// Singular values and derived rank metrics of a matrix.
struct SvdMetrics {
  VecX singular_values;          ///< Descending order.
  int rank = 0;                  ///< Number of singular values above the tolerance.
  double tolerance = 0.0;        ///< Tolerance used for the rank decision.
  double sigma_max = 0.0;        ///< Largest singular value.
  double sigma_min = 0.0;        ///< Smallest of the min(rows, cols) singular values.
  double condition_number = 0.0; ///< sigma_max / sigma_min; +infinity if sigma_min == 0.
};

/// Computes singular values and rank metrics. A negative @p tolerance selects
/// defaultRankTolerance().
[[nodiscard]] SvdMetrics computeSvdMetrics(const MatX& A, double tolerance = -1.0);

/// Moore-Penrose pseudoinverse via SVD. Singular values at or below @p tolerance are treated as
/// zero. A negative tolerance selects defaultRankTolerance().
[[nodiscard]] MatX pseudoInverse(const MatX& A, double tolerance = -1.0);

/// Minimum-norm least-squares solution x = A^+ b computed from the SVD (no explicit inverse).
[[nodiscard]] VecX solveMinimumNorm(const MatX& A, const VecX& b, double tolerance = -1.0);

/// Damped least-squares solution x = A^T (A A^T + lambda^2 I)^{-1} b.
///
/// Equivalent to minimizing |A x - b|^2 + lambda^2 |x|^2. The system is solved by Cholesky (LLT)
/// in the smaller of the two dimensions. Requires lambda >= 0; lambda == 0 requires A to have
/// full rank in the solved dimension.
[[nodiscard]] VecX solveDampedLeastSquares(const MatX& A, const VecX& b, double lambda);

/// Damped pseudoinverse A^T (A A^T + lambda^2 I)^{-1}, assembled from the SVD:
/// sigma_i / (sigma_i^2 + lambda^2).
[[nodiscard]] MatX dampedPseudoInverse(const MatX& A, double lambda);

/// Orthogonal projector onto the null space of A: I - A^+ A.
[[nodiscard]] MatX nullSpaceProjector(const MatX& A, double tolerance = -1.0);

/// True if |A - A^T|_max <= tolerance * max(1, |A|_max).
[[nodiscard]] bool isSymmetric(const MatX& A, double tolerance = 1e-12);

/// True if A is symmetric and its Cholesky (LLT) factorization succeeds.
[[nodiscard]] bool isPositiveDefinite(const MatX& A);

/// Central-difference Jacobian of f at x: column j is (f(x + h e_j) - f(x - h e_j)) / (2 h).
/// Truncation error is O(h^2); round-off error is O(eps / h). h ~ 1e-6 balances both for
/// smooth functions with O(1) derivatives.
[[nodiscard]] MatX numericalJacobian(const std::function<VecX(const VecX&)>& f, const VecX& x,
                                     double step = 1e-6);

}  // namespace mc
