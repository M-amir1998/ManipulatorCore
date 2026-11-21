#pragma once

/// @file profiles.hpp
/// @brief Rest-to-rest scalar motion profiles s(t), s in [0, distance].
///
/// Profiles are used as time scalings: a path q(s) or a straight line q0 + s (q1 - q0) is
/// traversed with the profile's s(t). References: L. Biagiotti and C. Melchiorri, Trajectory
/// Planning for Automatic Machines and Robots, Springer, 2008, chapters 2 and 3.

#include <vector>

namespace mc {

struct ProfileSample {
  double s = 0.0;
  double sd = 0.0;
  double sdd = 0.0;
};

class ScalarProfile {
 public:
  ScalarProfile() = default;
  ScalarProfile(const ScalarProfile&) = default;
  ScalarProfile(ScalarProfile&&) = default;
  ScalarProfile& operator=(const ScalarProfile&) = default;
  ScalarProfile& operator=(ScalarProfile&&) = default;
  virtual ~ScalarProfile() = default;

  [[nodiscard]] virtual double duration() const = 0;
  [[nodiscard]] virtual double distance() const = 0;
  /// s(t), sd(t), sdd(t); t is clamped to [0, duration()].
  [[nodiscard]] virtual ProfileSample sample(double t) const = 0;
};

/// Cubic (zero end velocities) or quintic (zero end velocities and accelerations) polynomial.
class PolynomialProfile final : public ScalarProfile {
 public:
  [[nodiscard]] static PolynomialProfile cubic(double distance, double duration);
  [[nodiscard]] static PolynomialProfile quintic(double distance, double duration);

  [[nodiscard]] double duration() const override { return duration_; }
  [[nodiscard]] double distance() const override { return distance_; }
  [[nodiscard]] ProfileSample sample(double t) const override;

 private:
  PolynomialProfile(double distance, double duration, bool quintic);
  double distance_;
  double duration_;
  bool quintic_;
};

/// Trapezoidal velocity profile (triangular when the cruise velocity is not reached).
class TrapezoidalProfile final : public ScalarProfile {
 public:
  /// Minimum-time profile under |sd| <= vmax and |sdd| <= amax.
  [[nodiscard]] static TrapezoidalProfile minimumTime(double distance, double vmax, double amax);

  /// Profile with a prescribed duration using acceleration amax and the lowest cruise velocity
  /// that completes the motion in time. Throws if duration < 2 sqrt(distance / amax).
  [[nodiscard]] static TrapezoidalProfile withDuration(double distance, double duration, double amax);

  [[nodiscard]] double duration() const override { return duration_; }
  [[nodiscard]] double distance() const override { return distance_; }
  [[nodiscard]] ProfileSample sample(double t) const override;
  [[nodiscard]] double peakVelocity() const { return v_; }
  [[nodiscard]] double accelerationTime() const { return ta_; }

 private:
  TrapezoidalProfile(double distance, double v, double a, double ta, double duration);
  double distance_;
  double v_;
  double a_;
  double ta_;
  double duration_;
};

/// Jerk-limited double-S (7-segment) profile, rest to rest, minimum time under
/// |sd| <= vmax, |sdd| <= amax, |sddd| <= jmax. Handles the four cases (vmax and/or amax not
/// reached) in closed form.
class DoubleSProfile final : public ScalarProfile {
 public:
  [[nodiscard]] static DoubleSProfile minimumTime(double distance, double vmax, double amax, double jmax);

  [[nodiscard]] double duration() const override { return duration_; }
  [[nodiscard]] double distance() const override { return distance_; }
  [[nodiscard]] ProfileSample sample(double t) const override;
  [[nodiscard]] double peakVelocity() const { return peak_velocity_; }
  [[nodiscard]] double peakAcceleration() const { return peak_acceleration_; }
  [[nodiscard]] double jerk() const { return jerk_; }

 private:
  struct Phase {
    double start_time;
    double duration;
    double jerk;
    double s0, v0, a0;  // state at the start of the phase
  };
  DoubleSProfile() = default;
  double distance_ = 0.0;
  double duration_ = 0.0;
  double peak_velocity_ = 0.0;
  double peak_acceleration_ = 0.0;
  double jerk_ = 0.0;
  std::vector<Phase> phases_;
};

}  // namespace mc
