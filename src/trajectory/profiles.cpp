#include "manipulatorcore/trajectory/profiles.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

namespace {

void requireDistance(double distance) {
  require(std::isfinite(distance) && distance >= 0.0, "profile: distance must be finite and non-negative");
}

void requirePositive(double value, const char* what) {
  require(std::isfinite(value) && value > 0.0, std::string("profile: ") + what + " must be positive");
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Polynomial
// ------------------------------------------------------------------------------------------------

PolynomialProfile::PolynomialProfile(double distance, double duration, bool quintic)
    : distance_(distance), duration_(duration), quintic_(quintic) {
  requireDistance(distance);
  requirePositive(duration, "duration");
}

PolynomialProfile PolynomialProfile::cubic(double distance, double duration) {
  return {distance, duration, false};
}

PolynomialProfile PolynomialProfile::quintic(double distance, double duration) {
  return {distance, duration, true};
}

ProfileSample PolynomialProfile::sample(double t) const {
  const double T = duration_;
  const double tau = std::clamp(t / T, 0.0, 1.0);
  const double D = distance_;
  if (quintic_) {
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    return {D * (10.0 * t3 - 15.0 * t3 * tau + 6.0 * t3 * t2), D * (30.0 * t2 - 60.0 * t3 + 30.0 * t2 * t2) / T,
            D * (60.0 * tau - 180.0 * t2 + 120.0 * t3) / (T * T)};
  }
  return {D * (3.0 * tau * tau - 2.0 * tau * tau * tau), D * (6.0 * tau - 6.0 * tau * tau) / T,
          D * (6.0 - 12.0 * tau) / (T * T)};
}

// ------------------------------------------------------------------------------------------------
// Trapezoidal
// ------------------------------------------------------------------------------------------------

TrapezoidalProfile::TrapezoidalProfile(double distance, double v, double a, double ta, double duration)
    : distance_(distance), v_(v), a_(a), ta_(ta), duration_(duration) {}

TrapezoidalProfile TrapezoidalProfile::minimumTime(double distance, double vmax, double amax) {
  requireDistance(distance);
  requirePositive(vmax, "vmax");
  requirePositive(amax, "amax");
  if (distance == 0.0) {
    return {0.0, 0.0, amax, 0.0, 0.0};
  }
  if (distance >= vmax * vmax / amax) {
    const double ta = vmax / amax;  // cruise phase exists
    return {distance, vmax, amax, ta, distance / vmax + ta};
  }
  const double ta = std::sqrt(distance / amax);  // triangular profile
  return {distance, amax * ta, amax, ta, 2.0 * ta};
}

TrapezoidalProfile TrapezoidalProfile::withDuration(double distance, double duration, double amax) {
  requireDistance(distance);
  requirePositive(amax, "amax");
  require(std::isfinite(duration) && duration >= 0.0, "TrapezoidalProfile::withDuration: invalid duration");
  if (distance == 0.0) {
    return {0.0, 0.0, amax, 0.0, duration};
  }
  const double disc = amax * amax * duration * duration - 4.0 * amax * distance;
  require(disc >= -1e-12 * amax * amax * duration * duration,
          "TrapezoidalProfile::withDuration: duration too short for the acceleration limit");
  const double v = 0.5 * (amax * duration - std::sqrt(std::max(0.0, disc)));
  return {distance, v, amax, v / amax, duration};
}

ProfileSample TrapezoidalProfile::sample(double t) const {
  if (duration_ <= 0.0 || t >= duration_) {
    return {distance_, 0.0, 0.0};
  }
  if (t <= 0.0) {
    return {0.0, 0.0, distance_ > 0.0 ? a_ : 0.0};
  }
  if (t < ta_) {
    return {0.5 * a_ * t * t, a_ * t, a_};
  }
  if (t <= duration_ - ta_) {
    return {0.5 * a_ * ta_ * ta_ + v_ * (t - ta_), v_, 0.0};
  }
  const double tau = duration_ - t;
  return {distance_ - 0.5 * a_ * tau * tau, a_ * tau, -a_};
}

// ------------------------------------------------------------------------------------------------
// Double S
// ------------------------------------------------------------------------------------------------

DoubleSProfile DoubleSProfile::minimumTime(double distance, double vmax, double amax, double jmax) {
  requireDistance(distance);
  requirePositive(vmax, "vmax");
  requirePositive(amax, "amax");
  requirePositive(jmax, "jmax");
  DoubleSProfile p;
  p.distance_ = distance;
  if (distance == 0.0) {
    return p;
  }
  // Biagiotti & Melchiorri (2008), section 3.4, specialized to rest-to-rest motions.
  double Tj = 0.0;  // jerk phase duration
  double Ta = 0.0;  // acceleration (and deceleration) phase duration
  double alim = 0.0;
  if (vmax * jmax < amax * amax) {
    Tj = std::sqrt(vmax / jmax);  // amax is not reached before vmax
    Ta = 2.0 * Tj;
    alim = jmax * Tj;
  } else {
    Tj = amax / jmax;
    Ta = Tj + vmax / amax;
    alim = amax;
  }
  double vlim = vmax;
  double Tv = distance / vmax - Ta;
  if (Tv < 0.0) {  // vmax is not reached
    Tv = 0.0;
    if (distance >= 2.0 * amax * amax * amax / (jmax * jmax)) {
      Tj = amax / jmax;
      Ta = 0.5 * Tj + std::sqrt(0.25 * Tj * Tj + distance / amax);
      alim = amax;
    } else {
      Tj = std::cbrt(distance / (2.0 * jmax));
      Ta = 2.0 * Tj;
      alim = jmax * Tj;
    }
    vlim = alim * (Ta - Tj);
  }
  const double Tc = std::max(0.0, Ta - 2.0 * Tj);  // constant-acceleration phase
  const double phase_durations[7] = {Tj, Tc, Tj, Tv, Tj, Tc, Tj};
  const double phase_jerks[7] = {jmax, 0.0, -jmax, 0.0, -jmax, 0.0, jmax};
  double t = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  for (int k = 0; k < 7; ++k) {
    const double d = phase_durations[k];
    const double j = phase_jerks[k];
    p.phases_.push_back({t, d, j, s, v, a});
    s += v * d + 0.5 * a * d * d + j * d * d * d / 6.0;
    v += a * d + 0.5 * j * d * d;
    a += j * d;
    t += d;
  }
  p.duration_ = t;
  p.peak_velocity_ = vlim;
  p.peak_acceleration_ = alim;
  p.jerk_ = jmax;
  return p;
}

ProfileSample DoubleSProfile::sample(double t) const {
  if (phases_.empty() || t >= duration_) {
    return {distance_, 0.0, 0.0};
  }
  t = std::max(t, 0.0);
  std::size_t k = 0;
  while (k + 1 < phases_.size() && t >= phases_[k + 1].start_time) {
    ++k;
  }
  const Phase& ph = phases_[k];
  const double tau = t - ph.start_time;
  return {std::clamp(ph.s0 + ph.v0 * tau + 0.5 * ph.a0 * tau * tau + ph.jerk * tau * tau * tau / 6.0, 0.0, distance_),
          std::max(0.0, ph.v0 + ph.a0 * tau + 0.5 * ph.jerk * tau * tau), ph.a0 + ph.jerk * tau};
}

}  // namespace mc
