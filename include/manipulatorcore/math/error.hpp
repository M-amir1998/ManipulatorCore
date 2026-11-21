#pragma once

/// @file error.hpp
/// @brief Error-handling policy.
///
/// Invalid input or configuration throws mc::InvalidArgument. Expected algorithmic outcomes (an IK
/// solve that does not converge, a planner that exhausts its budget) are reported through result
/// objects instead; see docs/mathematics/conventions.md.

#include "manipulatorcore/math/types.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace mc {

/// Thrown for invalid arguments, inconsistent models and invalid configuration.
class InvalidArgument : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

/// Throws InvalidArgument with @p message when @p condition is false.
inline void require(bool condition, const char* message) {
  if (!condition) {
    throw InvalidArgument(message);
  }
}

/// Throws InvalidArgument with @p message when @p condition is false.
inline void require(bool condition, const std::string& message) {
  if (!condition) {
    throw InvalidArgument(message);
  }
}

/// Throws InvalidArgument unless every coefficient of @p value is finite.
template <typename Derived>
void requireFinite(const Eigen::MatrixBase<Derived>& value, const char* what) {
  if (!value.allFinite()) {
    throw InvalidArgument(std::string(what) + ": contains non-finite values");
  }
}

/// Throws InvalidArgument unless @p value is finite.
inline void requireFinite(double value, const char* what) {
  if (!std::isfinite(value)) {
    throw InvalidArgument(std::string(what) + ": value is not finite");
  }
}

/// Throws InvalidArgument unless @p value has exactly @p expected entries.
template <typename Derived>
void requireSize(const Eigen::MatrixBase<Derived>& value, Index expected, const char* what) {
  if (value.size() != expected) {
    throw InvalidArgument(std::string(what) + ": expected size " + std::to_string(expected) + ", got " +
                          std::to_string(value.size()));
  }
}

/// Throws InvalidArgument unless @p value has exactly @p expected entries and all are finite.
template <typename Derived>
void requireVector(const Eigen::MatrixBase<Derived>& value, Index expected, const char* what) {
  requireSize(value, expected, what);
  requireFinite(value, what);
}

}  // namespace mc
