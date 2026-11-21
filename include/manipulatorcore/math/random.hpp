#pragma once

/// @file random.hpp
/// @brief Explicitly seeded, platform-reproducible random numbers.

#include "manipulatorcore/math/types.hpp"

#include <cstdint>
#include <random>

namespace mc {

/// Deterministic random-number source.
///
/// Wraps std::mt19937_64, whose output sequence is fully specified by the C++ standard, and maps
/// the raw 64-bit output to uniform numbers itself. Standard-library distributions are avoided
/// because their algorithms are implementation-defined. Uniform samples are therefore
/// bit-identical across compilers and platforms for a given seed. normal() uses the Box-Muller
/// transform and may differ in the last unit of precision between math libraries.
///
/// A Random object is not thread-safe; give each thread its own instance.
class Random {
 public:
  explicit Random(std::uint64_t seed = 0) : engine_(seed) {}

  /// Re-seeds the generator and clears cached state.
  void seed(std::uint64_t seed);

  /// Raw 64-bit output of the engine.
  [[nodiscard]] std::uint64_t nextU64() { return engine_(); }

  /// Uniform double in [0, 1) with 53 random mantissa bits.
  [[nodiscard]] double uniform();

  /// Uniform double in [lo, hi).
  [[nodiscard]] double uniform(double lo, double hi);

  /// Uniform integer in [0, n). Unbiased (rejection sampling). Requires n > 0.
  [[nodiscard]] std::uint64_t uniformIndex(std::uint64_t n);

  /// Standard normal sample (Box-Muller).
  [[nodiscard]] double normal();

  /// True with probability p.
  [[nodiscard]] bool bernoulli(double p) { return uniform() < p; }

  /// Component-wise uniform vector in the box [lo, hi).
  [[nodiscard]] VecX uniformVector(const VecX& lo, const VecX& hi);

  /// Uniformly distributed unit vector in R^3.
  [[nodiscard]] Vec3 unitVector3();

  /// Uniformly distributed rotation (Shoemake's subgroup algorithm).
  [[nodiscard]] Quat uniformQuaternion();

 private:
  std::mt19937_64 engine_;
  bool has_spare_normal_ = false;
  double spare_normal_ = 0.0;
};

}  // namespace mc
