#include "manipulatorcore/math/random.hpp"

#include "manipulatorcore/math/error.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace mc {
namespace {

TEST(Random, Mt19937_64MatchesStandardReferenceValue) {
  // The C++ standard specifies that the 10000th output of a default-constructed mt19937_64 is
  // 9981545732273789042. Random(5489) uses the same default seed.
  Random rng(5489U);
  std::uint64_t value = 0;
  for (int i = 0; i < 10000; ++i) {
    value = rng.nextU64();
  }
  EXPECT_EQ(value, 9981545732273789042ULL);
}

TEST(Random, SameSeedSameSequence) {
  Random a(123);
  Random b(123);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(a.uniform(), b.uniform());
  }
  a.seed(9);
  b.seed(9);
  EXPECT_EQ(a.uniformIndex(17), b.uniformIndex(17));
}

TEST(Random, UniformInUnitInterval) {
  Random rng(1);
  double sum = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
    sum += u;
  }
  EXPECT_NEAR(sum / n, 0.5, 0.01);
}

TEST(Random, UniformIndexCoversRange) {
  Random rng(2);
  std::array<int, 5> counts{};
  for (int i = 0; i < 5000; ++i) {
    const auto k = rng.uniformIndex(5);
    ASSERT_LT(k, 5U);
    ++counts.at(static_cast<std::size_t>(k));
  }
  for (const int c : counts) {
    EXPECT_GT(c, 850);
    EXPECT_LT(c, 1150);
  }
  EXPECT_THROW((void)rng.uniformIndex(0), InvalidArgument);
}

TEST(Random, NormalMoments) {
  Random rng(3);
  const int n = 40000;
  double mean = 0.0;
  double sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = rng.normal();
    mean += x;
    sq += x * x;
  }
  mean /= n;
  EXPECT_NEAR(mean, 0.0, 0.02);
  EXPECT_NEAR(sq / n - mean * mean, 1.0, 0.03);
}

TEST(Random, UnitVectorsAndQuaternionsAreNormalized) {
  Random rng(4);
  for (int i = 0; i < 100; ++i) {
    EXPECT_NEAR(rng.unitVector3().norm(), 1.0, 1e-14);
    EXPECT_NEAR(rng.uniformQuaternion().norm(), 1.0, 1e-14);
  }
}

}  // namespace
}  // namespace mc
