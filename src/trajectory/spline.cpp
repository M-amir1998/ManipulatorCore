#include "manipulatorcore/trajectory/spline.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>

namespace mc {

CubicSpline::CubicSpline(std::vector<double> knots, std::vector<VecX> values, Boundary boundary,
                         const VecX& start_derivative, const VecX& end_derivative)
    : knots_(std::move(knots)), values_(std::move(values)) {
  require(knots_.size() >= 2 && knots_.size() == values_.size(),
          "CubicSpline: need at least two knots with matching values");
  const Index dim = values_.front().size();
  require(dim > 0, "CubicSpline: values must not be empty");
  for (std::size_t k = 0; k < knots_.size(); ++k) {
    requireFinite(knots_[k], "CubicSpline: knot");
    requireVector(values_[k], dim, "CubicSpline: value");
    if (k > 0) {
      require(knots_[k] > knots_[k - 1], "CubicSpline: knots must be strictly increasing");
    }
  }
  const std::size_t n = knots_.size() - 1;  // number of intervals
  std::vector<double> h(n);
  for (std::size_t i = 0; i < n; ++i) {
    h[i] = knots_[i + 1] - knots_[i];
  }
  // Tridiagonal system  sub[i] M[i-1] + diag[i] M[i] + sup[i] M[i+1] = rhs[i]  for i = 0..n.
  std::vector<double> sub(n + 1, 0.0);
  std::vector<double> diag(n + 1, 0.0);
  std::vector<double> sup(n + 1, 0.0);
  std::vector<VecX> rhs(n + 1, VecX::Zero(dim));
  if (boundary == Boundary::Natural) {
    diag[0] = 1.0;
    diag[n] = 1.0;
  } else {
    requireVector(start_derivative, dim, "CubicSpline: start derivative");
    requireVector(end_derivative, dim, "CubicSpline: end derivative");
    diag[0] = 2.0 * h[0];
    sup[0] = h[0];
    rhs[0] = 6.0 * ((values_[1] - values_[0]) / h[0] - start_derivative);
    sub[n] = h[n - 1];
    diag[n] = 2.0 * h[n - 1];
    rhs[n] = 6.0 * (end_derivative - (values_[n] - values_[n - 1]) / h[n - 1]);
  }
  for (std::size_t i = 1; i < n; ++i) {
    sub[i] = h[i - 1];
    diag[i] = 2.0 * (h[i - 1] + h[i]);
    sup[i] = h[i];
    rhs[i] = 6.0 * ((values_[i + 1] - values_[i]) / h[i] - (values_[i] - values_[i - 1]) / h[i - 1]);
  }
  // Thomas algorithm (the matrix is strictly diagonally dominant, so no pivoting is needed).
  for (std::size_t i = 1; i <= n; ++i) {
    const double w = sub[i] / diag[i - 1];
    diag[i] -= w * sup[i - 1];
    rhs[i] -= w * rhs[i - 1];
  }
  second_.assign(n + 1, VecX::Zero(dim));
  second_[n] = rhs[n] / diag[n];
  for (std::size_t i = n; i-- > 0;) {
    second_[i] = (rhs[i] - sup[i] * second_[i + 1]) / diag[i];
  }
}

CubicSpline::Sample CubicSpline::evaluate(double u) const {
  u = std::clamp(u, knots_.front(), knots_.back());
  std::size_t i = static_cast<std::size_t>(std::upper_bound(knots_.begin(), knots_.end(), u) - knots_.begin());
  i = std::clamp<std::size_t>(i, 1, knots_.size() - 1) - 1;
  const double h = knots_[i + 1] - knots_[i];
  const double A = (knots_[i + 1] - u) / h;
  const double B = (u - knots_[i]) / h;
  const VecX& y0 = values_[i];
  const VecX& y1 = values_[i + 1];
  const VecX& M0 = second_[i];
  const VecX& M1 = second_[i + 1];
  Sample s;
  s.value = A * y0 + B * y1 + ((A * A * A - A) * M0 + (B * B * B - B) * M1) * (h * h / 6.0);
  s.first = (y1 - y0) / h - (3.0 * A * A - 1.0) / 6.0 * h * M0 + (3.0 * B * B - 1.0) / 6.0 * h * M1;
  s.second = A * M0 + B * M1;
  return s;
}

namespace {

VecX zeroIfEmpty(const VecX& v, Index n) { return v.size() == 0 ? VecX::Zero(n) : v; }

CubicSpline makeClampedSpline(std::vector<double> times, std::vector<VecX> waypoints, const VecX& start_velocity,
                              const VecX& end_velocity) {
  const Index dim = waypoints.empty() ? 0 : waypoints.front().size();
  return {std::move(times), std::move(waypoints), CubicSpline::Boundary::Clamped, zeroIfEmpty(start_velocity, dim),
          zeroIfEmpty(end_velocity, dim)};
}

}  // namespace

CubicSplineTrajectory::CubicSplineTrajectory(std::vector<double> times, std::vector<VecX> waypoints,
                                             const VecX& start_velocity, const VecX& end_velocity)
    : spline_(makeClampedSpline(std::move(times), std::move(waypoints), start_velocity, end_velocity)) {
  require(spline_.start() == 0.0, "CubicSplineTrajectory: times must start at 0");
}

TrajectoryPoint CubicSplineTrajectory::sample(double t) const {
  const double tc = std::clamp(t, 0.0, duration());
  CubicSpline::Sample s = spline_.evaluate(tc);
  return {tc, std::move(s.value), std::move(s.first), std::move(s.second)};
}

}  // namespace mc
