#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/se3.hpp"
#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/random.hpp"

#include <benchmark/benchmark.h>

#include <vector>

namespace {

using namespace mc;

std::vector<Vec6> makeTwists(std::size_t n) {
  Random rng(1);
  std::vector<Vec6> twists(n);
  for (auto& xi : twists) {
    for (Index i = 0; i < 6; ++i) {
      xi(i) = rng.uniform(-1.5, 1.5);
    }
  }
  return twists;
}

void BM_ExpSO3(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(expSO3(twists[i++ & 255U].head<3>()));
  }
}
BENCHMARK(BM_ExpSO3);

void BM_LogSO3(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::vector<Mat3> rotations;
  for (const auto& xi : twists) {
    rotations.push_back(expSO3(xi.head<3>()));
  }
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(logSO3(rotations[i++ & 255U]));
  }
}
BENCHMARK(BM_LogSO3);

void BM_ExpSE3(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(expSE3(twists[i++ & 255U]));
  }
}
BENCHMARK(BM_ExpSE3);

void BM_LogSE3(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::vector<SE3> poses;
  for (const auto& xi : twists) {
    poses.push_back(expSE3(xi));
  }
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(logSE3(poses[i++ & 255U]));
  }
}
BENCHMARK(BM_LogSE3);

void BM_SE3Compose(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::vector<SE3> poses;
  for (const auto& xi : twists) {
    poses.push_back(expSE3(xi));
  }
  SE3 acc;
  std::size_t i = 0;
  for (auto _ : state) {
    acc = acc * poses[i++ & 255U];
    benchmark::DoNotOptimize(acc);
  }
}
BENCHMARK(BM_SE3Compose);

void BM_QuaternionFromRotation(benchmark::State& state) {
  const auto twists = makeTwists(256);
  std::vector<Mat3> rotations;
  for (const auto& xi : twists) {
    rotations.push_back(expSO3(xi.head<3>()));
  }
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(quaternionFromRotation(rotations[i++ & 255U]));
  }
}
BENCHMARK(BM_QuaternionFromRotation);

}  // namespace
