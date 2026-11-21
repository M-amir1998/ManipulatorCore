#pragma once

/// @file angles.hpp
/// @brief Scalar angle utilities.

#include "manipulatorcore/math/types.hpp"

#include <cmath>

namespace mc {

/// Wraps an angle to (-pi, pi].
[[nodiscard]] inline double wrapToPi(double angle) {
  double wrapped = std::remainder(angle, kTwoPi);  // [-pi, pi]
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

/// Smallest signed difference a - b, wrapped to (-pi, pi].
[[nodiscard]] inline double angleDifference(double a, double b) { return wrapToPi(a - b); }

/// Degrees to radians.
[[nodiscard]] constexpr double degToRad(double degrees) { return degrees * kPi / 180.0; }

/// Radians to degrees.
[[nodiscard]] constexpr double radToDeg(double radians) { return radians * 180.0 / kPi; }

}  // namespace mc
