#include "manipulatorcore/math/random.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

void Random::seed(std::uint64_t seed) {
  engine_.seed(seed);
  has_spare_normal_ = false;
  spare_normal_ = 0.0;
}

double Random::uniform() {
  // Keep the top 53 bits: every representable value k * 2^-53 in [0, 1) is equally likely.
  return static_cast<double>(engine_() >> 11U) * 0x1.0p-53;
}

double Random::uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

std::uint64_t Random::uniformIndex(std::uint64_t n) {
  require(n > 0, "Random::uniformIndex: n must be positive");
  // Reject the lowest (2^64 mod n) values so that the remaining range is a multiple of n.
  const std::uint64_t threshold = (0 - n) % n;
  std::uint64_t x = engine_();
  while (x < threshold) {
    x = engine_();
  }
  return x % n;
}

double Random::normal() {
  if (has_spare_normal_) {
    has_spare_normal_ = false;
    return spare_normal_;
  }
  const double u1 = 1.0 - uniform();  // (0, 1], so log(u1) is finite
  const double u2 = uniform();
  const double radius = std::sqrt(-2.0 * std::log(u1));
  const double angle = kTwoPi * u2;
  spare_normal_ = radius * std::sin(angle);
  has_spare_normal_ = true;
  return radius * std::cos(angle);
}

VecX Random::uniformVector(const VecX& lo, const VecX& hi) {
  require(lo.size() == hi.size(), "Random::uniformVector: bound sizes differ");
  VecX out(lo.size());
  for (Index i = 0; i < lo.size(); ++i) {
    out[i] = uniform(lo[i], hi[i]);
  }
  return out;
}

Vec3 Random::unitVector3() {
  const double z = uniform(-1.0, 1.0);
  const double phi = uniform(0.0, kTwoPi);
  const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
  return {r * std::cos(phi), r * std::sin(phi), z};
}

Quat Random::uniformQuaternion() {
  // K. Shoemake, "Uniform random rotations", Graphics Gems III, 1992.
  const double u1 = uniform();
  const double u2 = uniform();
  const double u3 = uniform();
  const double a = std::sqrt(1.0 - u1);
  const double b = std::sqrt(u1);
  Quat q(b * std::cos(kTwoPi * u3), a * std::sin(kTwoPi * u2), a * std::cos(kTwoPi * u2), b * std::sin(kTwoPi * u3));
  q.normalize();
  return q;
}

}  // namespace mc
