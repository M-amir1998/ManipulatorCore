#include "manipulatorcore/math/linalg.hpp"

#include "manipulatorcore/math/error.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <limits>

namespace mc {

namespace {

Eigen::JacobiSVD<MatX> thinSvd(const MatX& A) {
  return Eigen::JacobiSVD<MatX>(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
}

double resolveTolerance(double tolerance, double sigma_max, Index rows, Index cols) {
  return tolerance >= 0.0 ? tolerance : defaultRankTolerance(sigma_max, rows, cols);
}

}  // namespace

double defaultRankTolerance(double sigma_max, Index rows, Index cols) {
  return sigma_max * static_cast<double>(std::max(rows, cols)) * std::numeric_limits<double>::epsilon();
}

SvdMetrics computeSvdMetrics(const MatX& A, double tolerance) {
  requireFinite(A, "computeSvdMetrics: matrix");
  SvdMetrics metrics;
  if (A.size() == 0) {
    return metrics;
  }
  const Eigen::JacobiSVD<MatX> svd(A);
  metrics.singular_values = svd.singularValues();
  metrics.sigma_max = metrics.singular_values(0);
  metrics.sigma_min = metrics.singular_values(metrics.singular_values.size() - 1);
  metrics.tolerance = resolveTolerance(tolerance, metrics.sigma_max, A.rows(), A.cols());
  metrics.rank = static_cast<int>((metrics.singular_values.array() > metrics.tolerance).count());
  metrics.condition_number =
      metrics.sigma_min > 0.0 ? metrics.sigma_max / metrics.sigma_min : std::numeric_limits<double>::infinity();
  return metrics;
}

MatX pseudoInverse(const MatX& A, double tolerance) {
  requireFinite(A, "pseudoInverse: matrix");
  if (A.size() == 0) {
    return MatX::Zero(A.cols(), A.rows());
  }
  const auto svd = thinSvd(A);
  const VecX& sigma = svd.singularValues();
  const double tol = resolveTolerance(tolerance, sigma(0), A.rows(), A.cols());
  VecX sigma_inv(sigma.size());
  for (Index i = 0; i < sigma.size(); ++i) {
    sigma_inv(i) = sigma(i) > tol ? 1.0 / sigma(i) : 0.0;
  }
  return svd.matrixV() * sigma_inv.asDiagonal() * svd.matrixU().transpose();
}

VecX solveMinimumNorm(const MatX& A, const VecX& b, double tolerance) {
  requireFinite(A, "solveMinimumNorm: matrix");
  requireVector(b, A.rows(), "solveMinimumNorm: right-hand side");
  if (A.size() == 0) {
    return VecX::Zero(A.cols());
  }
  const auto svd = thinSvd(A);
  const VecX& sigma = svd.singularValues();
  const double tol = resolveTolerance(tolerance, sigma(0), A.rows(), A.cols());
  VecX coeffs = svd.matrixU().transpose() * b;
  for (Index i = 0; i < sigma.size(); ++i) {
    coeffs(i) = sigma(i) > tol ? coeffs(i) / sigma(i) : 0.0;
  }
  return svd.matrixV() * coeffs;
}

VecX solveDampedLeastSquares(const MatX& A, const VecX& b, double lambda) {
  requireFinite(A, "solveDampedLeastSquares: matrix");
  requireVector(b, A.rows(), "solveDampedLeastSquares: right-hand side");
  require(std::isfinite(lambda) && lambda >= 0.0, "solveDampedLeastSquares: damping must be finite and >= 0");
  if (lambda == 0.0) {
    return solveMinimumNorm(A, b);
  }
  const double lambda_sq = lambda * lambda;
  if (A.rows() <= A.cols()) {
    MatX gram = A * A.transpose();
    gram.diagonal().array() += lambda_sq;
    const Eigen::LLT<MatX> llt(gram);
    if (llt.info() == Eigen::Success) {
      return A.transpose() * llt.solve(b);
    }
  } else {
    MatX gram = A.transpose() * A;
    gram.diagonal().array() += lambda_sq;
    const Eigen::LLT<MatX> llt(gram);
    if (llt.info() == Eigen::Success) {
      return llt.solve(A.transpose() * b);
    }
  }
  // Extremely small damping on a singular matrix: fall back to the SVD form of the same solution.
  return dampedPseudoInverse(A, lambda) * b;
}

MatX dampedPseudoInverse(const MatX& A, double lambda) {
  requireFinite(A, "dampedPseudoInverse: matrix");
  require(std::isfinite(lambda) && lambda >= 0.0, "dampedPseudoInverse: damping must be finite and >= 0");
  if (A.size() == 0) {
    return MatX::Zero(A.cols(), A.rows());
  }
  if (lambda == 0.0) {
    return pseudoInverse(A);
  }
  const auto svd = thinSvd(A);
  const VecX& sigma = svd.singularValues();
  const VecX gains = sigma.array() / (sigma.array().square() + lambda * lambda);
  return svd.matrixV() * gains.asDiagonal() * svd.matrixU().transpose();
}

MatX nullSpaceProjector(const MatX& A, double tolerance) {
  return MatX::Identity(A.cols(), A.cols()) - pseudoInverse(A, tolerance) * A;
}

bool isSymmetric(const MatX& A, double tolerance) {
  if (A.rows() != A.cols()) {
    return false;
  }
  const double scale = std::max(1.0, A.cwiseAbs().maxCoeff());
  return (A - A.transpose()).cwiseAbs().maxCoeff() <= tolerance * scale;
}

bool isPositiveDefinite(const MatX& A) {
  if (!isSymmetric(A, 1e-9)) {
    return false;
  }
  const Eigen::LLT<MatX> llt(A);
  return llt.info() == Eigen::Success;
}

MatX numericalJacobian(const std::function<VecX(const VecX&)>& f, const VecX& x, double step) {
  require(step > 0.0 && std::isfinite(step), "numericalJacobian: step must be positive");
  const VecX f0 = f(x);
  MatX J(f0.size(), x.size());
  VecX xp = x;
  VecX xm = x;
  for (Index j = 0; j < x.size(); ++j) {
    xp(j) = x(j) + step;
    xm(j) = x(j) - step;
    J.col(j) = (f(xp) - f(xm)) / (2.0 * step);
    xp(j) = x(j);
    xm(j) = x(j);
  }
  return J;
}

}  // namespace mc
