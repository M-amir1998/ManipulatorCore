#include "manipulatorcore/trajectory/time_parameterization.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace mc {

// ------------------------------------------------------------------------------------------------
// Paths
// ------------------------------------------------------------------------------------------------

CubicSpline SplineJointPath::build(const std::vector<VecX>& waypoints) {
  require(waypoints.size() >= 2, "SplineJointPath: need at least two waypoints");
  std::vector<double> knots{0.0};
  std::vector<VecX> values{waypoints.front()};
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    requireVector(waypoints[i], waypoints.front().size(), "SplineJointPath: waypoint");
    const double step = (waypoints[i] - values.back()).norm();
    if (step > 1e-12) {  // drop repeated waypoints, which would create zero-length intervals
      knots.push_back(knots.back() + step);
      values.push_back(waypoints[i]);
    }
  }
  require(values.size() >= 2, "SplineJointPath: all waypoints coincide");
  return {knots, values, CubicSpline::Boundary::Natural};
}

SplineJointPath::SplineJointPath(const std::vector<VecX>& waypoints) : spline_(build(waypoints)) {}

StraightJointPath::StraightJointPath(const VecX& q0, const VecX& q1) : q0_(q0) {
  requireVector(q1, q0.size(), "StraightJointPath: q1");
  requireFinite(q0, "StraightJointPath: q0");
  length_ = (q1 - q0).norm();
  require(length_ > 0.0, "StraightJointPath: q0 and q1 coincide");
  direction_ = (q1 - q0) / length_;
}

// ------------------------------------------------------------------------------------------------
// Reachability-based time-optimal parameterization
// ------------------------------------------------------------------------------------------------

namespace {

/// Linear constraint a * u + b * x <= c in the variables u = sdd and x = sd^2.
struct Constraint {
  double a;
  double b;
  double c;
};

constexpr double kTiny = 1e-14;

/// Range of x for which some u satisfies every constraint, intersected with [lo, hi].
std::optional<std::pair<double, double>> feasibleX(const std::vector<Constraint>& cs, double lo, double hi) {
  // Each constraint with a != 0 bounds u by p + q x from above (a > 0) or below (a < 0).
  std::vector<std::pair<double, double>> lower;
  std::vector<std::pair<double, double>> upper;
  for (const Constraint& c : cs) {
    if (std::abs(c.a) <= kTiny) {
      if (c.b > kTiny) {
        hi = std::min(hi, c.c / c.b);
      } else if (c.b < -kTiny) {
        lo = std::max(lo, c.c / c.b);
      } else if (c.c < -1e-12) {
        return std::nullopt;
      }
    } else if (c.a > 0.0) {
      upper.emplace_back(c.c / c.a, -c.b / c.a);
    } else {
      lower.emplace_back(c.c / c.a, -c.b / c.a);
    }
  }
  // u exists iff every lower bound is below every upper bound: (q_l - q_u) x <= p_u - p_l.
  for (const auto& [pl, ql] : lower) {
    for (const auto& [pu, qu] : upper) {
      const double k = ql - qu;
      const double r = pu - pl;
      if (k > kTiny) {
        hi = std::min(hi, r / k);
      } else if (k < -kTiny) {
        lo = std::max(lo, r / k);
      } else if (r < -1e-12) {
        return std::nullopt;
      }
    }
  }
  if (lo > hi + 1e-12 * std::max(1.0, std::abs(hi))) {
    return std::nullopt;
  }
  return std::make_pair(lo, std::max(lo, hi));
}

/// Largest u satisfying every constraint for fixed x (the greedy forward step).
double maxU(const std::vector<Constraint>& cs, double x) {
  double lo = -std::numeric_limits<double>::infinity();
  double hi = std::numeric_limits<double>::infinity();
  for (const Constraint& c : cs) {
    if (std::abs(c.a) <= kTiny) {
      continue;
    }
    const double bound = (c.c - c.b * x) / c.a;
    if (c.a > 0.0) {
      hi = std::min(hi, bound);
    } else {
      lo = std::max(lo, bound);
    }
  }
  // Round-off can make the interval marginally empty; the lower bound then keeps x_{i+1} >= 0
  // and inside the controllable set.
  return hi >= lo ? hi : lo;
}

}  // namespace

TimeOptimalTrajectory timeOptimalParameterization(std::shared_ptr<const JointPath> path, const VecX& velocity_limits,
                                                  const VecX& acceleration_limits,
                                                  const TimeParameterizationOptions& options) {
  require(path != nullptr, "timeOptimalParameterization: null path");
  const int n = path->dof();
  requireVector(velocity_limits, n, "timeOptimalParameterization: velocity limits");
  requireVector(acceleration_limits, n, "timeOptimalParameterization: acceleration limits");
  require((velocity_limits.array() > 0.0).all() && (acceleration_limits.array() > 0.0).all(),
          "timeOptimalParameterization: limits must be positive");
  require(options.grid_segments >= 2, "timeOptimalParameterization: grid_segments must be >= 2");
  require(options.velocity_scale > 0.0 && options.velocity_scale <= 1.0 && options.acceleration_scale > 0.0 &&
              options.acceleration_scale <= 1.0,
          "timeOptimalParameterization: scales must be in (0, 1]");
  require(options.safety_margin >= 0.0 && options.safety_margin < 0.5,
          "timeOptimalParameterization: safety_margin must be in [0, 0.5)");
  const double L = path->length();
  require(std::isfinite(L) && L > 0.0, "timeOptimalParameterization: path length must be positive");

  require(options.segments_per_breakpoint_interval >= 1 && options.max_grid_segments >= options.grid_segments,
          "timeOptimalParameterization: invalid grid refinement options");
  require(options.verification_samples_per_segment >= 0,
          "timeOptimalParameterization: verification_samples_per_segment must be >= 0");

  const VecX vmax = velocity_limits * (options.velocity_scale * (1.0 - options.safety_margin));
  const VecX amax = acceleration_limits * (options.acceleration_scale * (1.0 - options.safety_margin));

  // Grid: uniform spacing, refined so that every interval between path breakpoints (spline knots)
  // holds several segments and every breakpoint is a grid point.
  std::vector<double> marks{0.0};
  for (const double b : path->breakpoints()) {
    if (b > marks.back() + 1e-12 && b < L - 1e-12) {
      marks.push_back(b);
    }
  }
  marks.push_back(L);
  const double nominal = L / static_cast<double>(options.grid_segments);
  std::vector<double> s{0.0};
  for (std::size_t k = 1; k < marks.size(); ++k) {
    const double len = marks[k] - marks[k - 1];
    const int pieces = std::max(options.segments_per_breakpoint_interval, static_cast<int>(std::ceil(len / nominal)));
    for (int p = 1; p <= pieces; ++p) {
      s.push_back(p == pieces ? marks[k] : marks[k - 1] + len * p / pieces);
    }
  }
  require(static_cast<int>(s.size()) - 1 <= options.max_grid_segments,
          "timeOptimalParameterization: refined grid exceeds max_grid_segments");
  const std::size_t N = s.size() - 1;
  std::vector<VecX> d1(N + 1);
  std::vector<VecX> d2(N + 1);
  for (std::size_t i = 0; i <= N; ++i) {
    d1[i] = path->firstDerivative(s[i]);
    d2[i] = path->secondDerivative(s[i]);
  }

  std::vector<Constraint> cs;
  cs.reserve(static_cast<std::size_t>(6 * n + 2));
  // Constraints of grid segment i (from s_i to s_{i+1}, length ds_i) with the next state
  // x + 2 ds_i u bounded by [0, x_next_max]. Joint limits are imposed at both ends of the segment.
  auto buildConstraints = [&](std::size_t i, double x_next_max) {
    cs.clear();
    const double two_ds = 2.0 * (s[i + 1] - s[i]);
    for (int j = 0; j < n; ++j) {
      const double qp0 = d1[i](j);
      const double qpp0 = d2[i](j);
      const double qp1 = d1[i + 1](j);
      const double qpp1 = d2[i + 1](j);
      const double A = amax(j);
      const double V2 = vmax(j) * vmax(j);
      cs.push_back({qp0, qpp0, A});
      cs.push_back({-qp0, -qpp0, A});
      cs.push_back({qp1 + two_ds * qpp1, qpp1, A});
      cs.push_back({-(qp1 + two_ds * qpp1), -qpp1, A});
      cs.push_back({0.0, qp0 * qp0, V2});
      cs.push_back({two_ds * qp1 * qp1, qp1 * qp1, V2});
    }
    cs.push_back({two_ds, 1.0, x_next_max});
    cs.push_back({-two_ds, -1.0, 0.0});
  };

  // Backward pass: controllable sets K_i = [0, K[i]], ending at rest (K_N = {0}).
  constexpr double kUnbounded = 1e12;
  std::vector<double> K(N + 1, 0.0);
  for (std::size_t i = N; i-- > 0;) {
    buildConstraints(i, K[i + 1]);
    const auto range = feasibleX(cs, 0.0, kUnbounded);
    require(range.has_value() && range->first <= 1e-12,
            "timeOptimalParameterization: the path cannot be traversed from rest at grid point " + std::to_string(i));
    K[i] = range->second;
  }

  // Forward pass: greedy maximal acceleration inside the controllable sets.
  TimeOptimalTrajectory out;
  out.path_ = std::move(path);
  out.s_ = s;
  out.x_.assign(N + 1, 0.0);
  out.u_.assign(N, 0.0);
  out.times_.assign(N + 1, 0.0);
  for (std::size_t i = 0; i < N; ++i) {
    const double x = std::min(out.x_[i], K[i]);
    buildConstraints(i, K[i + 1]);
    const double u = maxU(cs, x);
    out.x_[i + 1] = std::clamp(x + 2.0 * (s[i + 1] - s[i]) * u, 0.0, K[i + 1]);
  }
  out.x_[N] = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    const double ds = s[i + 1] - s[i];
    out.u_[i] = (out.x_[i + 1] - out.x_[i]) / (2.0 * ds);
    const double speed_sum = std::sqrt(out.x_[i]) + std::sqrt(out.x_[i + 1]);
    require(speed_sum > 1e-12, "timeOptimalParameterization: zero path speed inside the path");
    out.times_[i + 1] = out.times_[i] + 2.0 * ds / speed_sum;  // exact for constant sdd
  }

  // Verification: sample inside every grid segment against the true (scaled, untightened) limits
  // and slow the whole trajectory down uniformly if the discretization let a peak through.
  if (options.verification_samples_per_segment > 0) {
    const VecX v_lim = velocity_limits * options.velocity_scale;
    const VecX a_lim = acceleration_limits * options.acceleration_scale;
    double v_ratio = 0.0;
    double a_ratio = 0.0;
    const int m = options.verification_samples_per_segment;
    for (std::size_t i = 0; i < N; ++i) {
      for (int k = 0; k <= m; ++k) {
        const double t = out.times_[i] + (out.times_[i + 1] - out.times_[i]) * k / m;
        const TrajectoryPoint p = out.sampleUnscaled(t);
        v_ratio = std::max(v_ratio, p.qd.cwiseAbs().cwiseQuotient(v_lim).maxCoeff());
        a_ratio = std::max(a_ratio, p.qdd.cwiseAbs().cwiseQuotient(a_lim).maxCoeff());
      }
    }
    const double needed = std::max({1.0, v_ratio, std::sqrt(a_ratio)});
    if (needed > 1.0) {
      out.time_scale_ = needed * (1.0 + 1e-9);
    }
  }
  return out;
}

TrajectoryPoint TimeOptimalTrajectory::sampleUnscaled(double t) const {
  const double T = times_.back();
  if (t >= T) {
    return {T, path_->position(s_.back()), VecX::Zero(dof()), VecX::Zero(dof())};
  }
  t = std::max(t, 0.0);
  std::size_t i = static_cast<std::size_t>(std::upper_bound(times_.begin(), times_.end(), t) - times_.begin());
  i = std::clamp<std::size_t>(i, 1, times_.size() - 1) - 1;
  const double tau = t - times_[i];
  const double sd0 = std::sqrt(x_[i]);
  const double u = u_[i];
  const double s = std::clamp(s_[i] + sd0 * tau + 0.5 * u * tau * tau, s_[i], s_[i + 1]);
  const double sd = std::max(0.0, sd0 + u * tau);
  const VecX d1 = path_->firstDerivative(s);
  return {t, path_->position(s), d1 * sd, d1 * u + path_->secondDerivative(s) * (sd * sd)};
}

TrajectoryPoint TimeOptimalTrajectory::sample(double t) const {
  // Uniform time scaling t' = t / k: velocities scale by 1/k and accelerations by 1/k^2.
  const double k = time_scale_;
  TrajectoryPoint p = sampleUnscaled(t / k);
  p.time = std::clamp(t, 0.0, duration());
  p.qd /= k;
  p.qdd /= k * k;
  return p;
}
}  // namespace mc
