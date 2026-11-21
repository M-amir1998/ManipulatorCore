#include "manipulatorcore/trajectory/polynomial.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mc {

namespace {

VecX orZero(const VecX& v, Index n, const char* what) {
  if (v.size() == 0) {
    return VecX::Zero(n);
  }
  requireVector(v, n, what);
  return v;
}

/// Largest admissible scalar rate along q0 -> q1 such that every joint respects its limit.
double lineScaling(const VecX& delta, const VecX& limits, const char* what) {
  requireVector(limits, delta.size(), what);
  require((limits.array() > 0.0).all(), std::string(what) + " must be positive");
  double scale = std::numeric_limits<double>::infinity();
  for (Index i = 0; i < delta.size(); ++i) {
    if (std::abs(delta(i)) > 0.0) {
      scale = std::min(scale, limits(i) / std::abs(delta(i)));
    }
  }
  return scale;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// PolynomialTrajectory
// ------------------------------------------------------------------------------------------------

PolynomialTrajectory::PolynomialTrajectory(MatX coefficients, double duration)
    : coefficients_(std::move(coefficients)), duration_(duration) {}

PolynomialTrajectory PolynomialTrajectory::cubic(const VecX& q0, const VecX& qf, double duration, const VecX& v0_in,
                                                 const VecX& vf_in) {
  const Index n = q0.size();
  requireVector(q0, n, "PolynomialTrajectory::cubic: q0");
  requireVector(qf, n, "PolynomialTrajectory::cubic: qf");
  const double T = duration;
  require(std::isfinite(T) && T > 0.0, "PolynomialTrajectory::cubic: duration must be positive");
  const VecX v0 = orZero(v0_in, n, "PolynomialTrajectory::cubic: v0");
  const VecX vf = orZero(vf_in, n, "PolynomialTrajectory::cubic: vf");
  MatX c(n, 4);
  const VecX h = qf - q0;
  c.col(0) = q0;
  c.col(1) = v0;
  c.col(2) = (3.0 * h - (2.0 * v0 + vf) * T) / (T * T);
  c.col(3) = (-2.0 * h + (v0 + vf) * T) / (T * T * T);
  return {c, T};
}

PolynomialTrajectory PolynomialTrajectory::quintic(const VecX& q0, const VecX& qf, double duration, const VecX& v0_in,
                                                   const VecX& vf_in, const VecX& a0_in, const VecX& af_in) {
  const Index n = q0.size();
  requireVector(q0, n, "PolynomialTrajectory::quintic: q0");
  requireVector(qf, n, "PolynomialTrajectory::quintic: qf");
  const double T = duration;
  require(std::isfinite(T) && T > 0.0, "PolynomialTrajectory::quintic: duration must be positive");
  const VecX v0 = orZero(v0_in, n, "PolynomialTrajectory::quintic: v0");
  const VecX vf = orZero(vf_in, n, "PolynomialTrajectory::quintic: vf");
  const VecX a0 = orZero(a0_in, n, "PolynomialTrajectory::quintic: a0");
  const VecX af = orZero(af_in, n, "PolynomialTrajectory::quintic: af");
  const VecX h = qf - q0;
  const double T2 = T * T;
  const double T3 = T2 * T;
  MatX c(n, 6);
  c.col(0) = q0;
  c.col(1) = v0;
  c.col(2) = 0.5 * a0;
  c.col(3) = (20.0 * h - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
  c.col(4) = (-30.0 * h + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / (2.0 * T3 * T);
  c.col(5) = (12.0 * h - 6.0 * (vf + v0) * T + (af - a0) * T2) / (2.0 * T3 * T2);
  return {c, T};
}

TrajectoryPoint PolynomialTrajectory::sample(double t) const {
  t = std::clamp(t, 0.0, duration_);
  const Index n = coefficients_.rows();
  const Index order = coefficients_.cols() - 1;
  TrajectoryPoint p{t, VecX::Zero(n), VecX::Zero(n), VecX::Zero(n)};
  // Horner evaluation of the polynomial and its first two derivatives.
  for (Index k = order; k >= 0; --k) {
    p.qdd = p.qdd * t + 2.0 * p.qd;
    p.qd = p.qd * t + p.q;
    p.q = p.q * t + coefficients_.col(k);
  }
  return p;
}

// ------------------------------------------------------------------------------------------------
// LinearInterpolationTrajectory
// ------------------------------------------------------------------------------------------------

LinearInterpolationTrajectory::LinearInterpolationTrajectory(std::vector<double> times, std::vector<VecX> waypoints)
    : times_(std::move(times)), waypoints_(std::move(waypoints)) {
  require(times_.size() >= 2 && times_.size() == waypoints_.size(),
          "LinearInterpolationTrajectory: need at least two waypoints with matching times");
  require(times_.front() == 0.0, "LinearInterpolationTrajectory: times must start at 0");
  for (std::size_t i = 0; i < times_.size(); ++i) {
    requireVector(waypoints_[i], waypoints_.front().size(), "LinearInterpolationTrajectory: waypoint");
    if (i > 0) {
      require(times_[i] > times_[i - 1], "LinearInterpolationTrajectory: times must be strictly increasing");
    }
  }
}

int LinearInterpolationTrajectory::dof() const { return static_cast<int>(waypoints_.front().size()); }

TrajectoryPoint LinearInterpolationTrajectory::sample(double t) const {
  t = std::clamp(t, 0.0, times_.back());
  std::size_t i = static_cast<std::size_t>(std::upper_bound(times_.begin(), times_.end(), t) - times_.begin());
  i = std::clamp<std::size_t>(i, 1, times_.size() - 1) - 1;
  const double h = times_[i + 1] - times_[i];
  const VecX velocity = (waypoints_[i + 1] - waypoints_[i]) / h;
  return {t, waypoints_[i] + (t - times_[i]) * velocity, velocity, VecX::Zero(velocity.size())};
}

// ------------------------------------------------------------------------------------------------
// LineTrajectory
// ------------------------------------------------------------------------------------------------

LineTrajectory::LineTrajectory(VecX q0, const VecX& q1, std::shared_ptr<const ScalarProfile> profile)
    : q0_(std::move(q0)), delta_(q1 - q0_), profile_(std::move(profile)) {
  requireVector(q1, q0_.size(), "LineTrajectory: q1");
  requireFinite(q0_, "LineTrajectory: q0");
  require(profile_ != nullptr, "LineTrajectory: profile must not be null");
  require(profile_->distance() == 0.0 || std::abs(profile_->distance() - 1.0) < 1e-12,
          "LineTrajectory: the profile must run from s = 0 to s = 1");
}

LineTrajectory LineTrajectory::trapezoidal(const VecX& q0, const VecX& q1, const VecX& velocity_limits,
                                           const VecX& acceleration_limits) {
  const VecX delta = q1 - q0;
  const double v = lineScaling(delta, velocity_limits, "velocity limits");
  const double a = lineScaling(delta, acceleration_limits, "acceleration limits");
  if (!std::isfinite(v)) {  // q0 == q1
    return {q0, q1, std::make_shared<TrapezoidalProfile>(TrapezoidalProfile::minimumTime(0.0, 1.0, 1.0))};
  }
  return {q0, q1, std::make_shared<TrapezoidalProfile>(TrapezoidalProfile::minimumTime(1.0, v, a))};
}

LineTrajectory LineTrajectory::doubleS(const VecX& q0, const VecX& q1, const VecX& velocity_limits,
                                       const VecX& acceleration_limits, const VecX& jerk_limits) {
  const VecX delta = q1 - q0;
  const double v = lineScaling(delta, velocity_limits, "velocity limits");
  const double a = lineScaling(delta, acceleration_limits, "acceleration limits");
  const double j = lineScaling(delta, jerk_limits, "jerk limits");
  if (!std::isfinite(v)) {
    return {q0, q1, std::make_shared<DoubleSProfile>(DoubleSProfile::minimumTime(0.0, 1.0, 1.0, 1.0))};
  }
  return {q0, q1, std::make_shared<DoubleSProfile>(DoubleSProfile::minimumTime(1.0, v, a, j))};
}

LineTrajectory LineTrajectory::quintic(const VecX& q0, const VecX& q1, double duration) {
  return {q0, q1, std::make_shared<PolynomialProfile>(PolynomialProfile::quintic(1.0, duration))};
}

TrajectoryPoint LineTrajectory::sample(double t) const {
  const ProfileSample p = profile_->sample(t);
  const double s = profile_->distance() > 0.0 ? p.s : 0.0;
  return {std::clamp(t, 0.0, duration()), q0_ + s * delta_, p.sd * delta_, p.sdd * delta_};
}

// ------------------------------------------------------------------------------------------------
// CompositeTrajectory
// ------------------------------------------------------------------------------------------------

CompositeTrajectory::CompositeTrajectory(std::vector<std::shared_ptr<const JointTrajectory>> segments)
    : segments_(std::move(segments)) {
  require(!segments_.empty(), "CompositeTrajectory: needs at least one segment");
  start_times_.push_back(0.0);
  for (std::size_t i = 0; i < segments_.size(); ++i) {
    require(segments_[i] != nullptr, "CompositeTrajectory: null segment");
    require(segments_[i]->dof() == segments_.front()->dof(), "CompositeTrajectory: inconsistent dof");
    if (i > 0) {
      const VecX end = segments_[i - 1]->sample(segments_[i - 1]->duration()).q;
      const VecX start = segments_[i]->sample(0.0).q;
      require((end - start).cwiseAbs().maxCoeff() < 1e-9, "CompositeTrajectory: segments are not continuous");
    }
    start_times_.push_back(start_times_.back() + segments_[i]->duration());
  }
}

int CompositeTrajectory::dof() const { return segments_.front()->dof(); }

TrajectoryPoint CompositeTrajectory::sample(double t) const {
  t = std::clamp(t, 0.0, duration());
  std::size_t i =
      static_cast<std::size_t>(std::upper_bound(start_times_.begin(), start_times_.end(), t) - start_times_.begin());
  i = std::clamp<std::size_t>(i, 1, segments_.size()) - 1;
  TrajectoryPoint p = segments_[i]->sample(t - start_times_[i]);
  p.time = t;
  return p;
}

CompositeTrajectory stopAtWaypointsTrajectory(const std::vector<VecX>& waypoints, const VecX& velocity_limits,
                                              const VecX& acceleration_limits) {
  require(!waypoints.empty(), "stopAtWaypointsTrajectory: no waypoints");
  std::vector<std::shared_ptr<const JointTrajectory>> segments;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    if ((waypoints[i] - waypoints[i - 1]).cwiseAbs().maxCoeff() == 0.0) {
      continue;
    }
    segments.push_back(std::make_shared<LineTrajectory>(
        LineTrajectory::trapezoidal(waypoints[i - 1], waypoints[i], velocity_limits, acceleration_limits)));
  }
  if (segments.empty()) {
    segments.push_back(std::make_shared<LineTrajectory>(
        LineTrajectory::trapezoidal(waypoints.front(), waypoints.front(), velocity_limits, acceleration_limits)));
  }
  return CompositeTrajectory(std::move(segments));
}

}  // namespace mc
