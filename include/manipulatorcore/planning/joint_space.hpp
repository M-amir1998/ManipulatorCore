#pragma once

/// @file joint_space.hpp
/// @brief Bounded joint configuration space, its metric, and discrete motion validation.

#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace mc {

/// Box-bounded configuration space with a weighted Euclidean metric
/// d(a, b) = sqrt(sum_i w_i (a_i - b_i)^2).
class JointSpace {
 public:
  JointSpace(VecX lower, VecX upper, VecX weights = VecX());

  /// The joint-limit box of a robot.
  [[nodiscard]] static JointSpace fromModel(const RobotModel& model, VecX weights = VecX());

  [[nodiscard]] int dimension() const { return static_cast<int>(lower_.size()); }
  [[nodiscard]] const VecX& lower() const { return lower_; }
  [[nodiscard]] const VecX& upper() const { return upper_; }
  [[nodiscard]] double distance(const VecX& a, const VecX& b) const;
  [[nodiscard]] VecX interpolate(const VecX& a, const VecX& b, double t) const { return a + t * (b - a); }
  /// Moves from @p from towards @p to by at most @p max_distance.
  [[nodiscard]] VecX steer(const VecX& from, const VecX& to, double max_distance) const;
  [[nodiscard]] VecX sampleUniform(Random& rng) const { return rng.uniformVector(lower_, upper_); }
  [[nodiscard]] bool contains(const VecX& q, double tolerance = 1e-12) const;
  /// Lebesgue measure of the box under the weighted metric (used by RRT*'s radius).
  [[nodiscard]] double measure() const;

 private:
  VecX lower_;
  VecX upper_;
  VecX weights_;
};

/// Returns true if a configuration is valid (inside the bounds and collision free).
using StateValidityFn = std::function<bool(const VecX&)>;

/// Validates states and straight-line edges, and counts the validity queries it performs.
///
/// Edges are checked at intermediate configurations spaced at most @p resolution apart in the
/// max-norm (largest single-joint step), so no joint moves more than the resolution between two
/// checked states. Endpoints are included.
class MotionValidator {
 public:
  MotionValidator(JointSpace space, StateValidityFn is_valid, double resolution = 0.02);

  [[nodiscard]] bool isValid(const VecX& q) const;
  /// Checks a -> b excluding a (assumed valid).
  [[nodiscard]] bool isEdgeValid(const VecX& a, const VecX& b) const;
  /// Checks every edge of a path including its first state.
  [[nodiscard]] bool isPathValid(const std::vector<VecX>& path) const;

  [[nodiscard]] const JointSpace& space() const { return space_; }
  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] std::uint64_t stateChecks() const { return state_checks_; }
  [[nodiscard]] std::uint64_t edgeChecks() const { return edge_checks_; }
  void resetCounters() const;

 private:
  JointSpace space_;
  StateValidityFn is_valid_;
  double resolution_;
  mutable std::uint64_t state_checks_ = 0;
  mutable std::uint64_t edge_checks_ = 0;
};

}  // namespace mc
