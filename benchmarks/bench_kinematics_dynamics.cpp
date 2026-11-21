// Kernel timings for kinematics, inverse kinematics and rigid-body dynamics on the built-in arms.

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/random.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

#include <benchmark/benchmark.h>

#include <vector>

namespace {

using namespace mc;

const RobotModel& robot(int index) {
  static const RobotModel six = makeIndustrial6R();
  static const RobotModel seven = makeRedundant7R();
  return index == 6 ? six : seven;
}

std::vector<VecX> configurations(const RobotModel& model, int count = 64) {
  Random rng(3);
  std::vector<VecX> out;
  for (int i = 0; i < count; ++i) {
    out.push_back(rng.uniformVector(model.lowerLimits(), model.upperLimits()));
  }
  return out;
}

void BM_ForwardKinematics(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  KinematicState ks;
  std::size_t i = 0;
  for (auto _ : state) {
    computeForwardKinematics(m, qs[i++ % qs.size()], ks);
    benchmark::DoNotOptimize(ks.T_base_ee);
  }
}
BENCHMARK(BM_ForwardKinematics)->Arg(6)->Arg(7);

void BM_GeometricJacobian(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(geometricJacobian(m, qs[i++ % qs.size()]));
  }
}
BENCHMARK(BM_GeometricJacobian)->Arg(6)->Arg(7);

void BM_InverseKinematicsDLS(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  Random rng(9);
  std::vector<std::pair<SE3, VecX>> problems;
  for (const VecX& q : qs) {
    const VecX seed = m.clampToLimits(q + 0.3 * rng.uniformVector(-VecX::Ones(m.dof()), VecX::Ones(m.dof())));
    problems.emplace_back(endEffectorPose(m, q), seed);
  }
  std::size_t i = 0;
  for (auto _ : state) {
    const auto& [target, seed] = problems[i++ % problems.size()];
    benchmark::DoNotOptimize(solveIK(m, target, seed));
  }
}
BENCHMARK(BM_InverseKinematicsDLS)->Arg(6)->Arg(7)->Unit(benchmark::kMicrosecond);

void BM_InverseDynamicsRNEA(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  const VecX qd = VecX::Constant(m.dof(), 0.3);
  const VecX qdd = VecX::Constant(m.dof(), 0.5);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(inverseDynamics(m, qs[i++ % qs.size()], qd, qdd));
  }
}
BENCHMARK(BM_InverseDynamicsRNEA)->Arg(6)->Arg(7);

void BM_MassMatrixCRBA(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(massMatrix(m, qs[i++ % qs.size()]));
  }
}
BENCHMARK(BM_MassMatrixCRBA)->Arg(6)->Arg(7);

void BM_ForwardDynamicsABA(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  const VecX qd = VecX::Constant(m.dof(), 0.3);
  const VecX tau = VecX::Constant(m.dof(), 5.0);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(forwardDynamics(m, qs[i++ % qs.size()], qd, tau, ForwardDynamicsMethod::ArticulatedBody));
  }
}
BENCHMARK(BM_ForwardDynamicsABA)->Arg(6)->Arg(7);

void BM_ForwardDynamicsCRBA_LLT(benchmark::State& state) {
  const RobotModel& m = robot(static_cast<int>(state.range(0)));
  const auto qs = configurations(m);
  const VecX qd = VecX::Constant(m.dof(), 0.3);
  const VecX tau = VecX::Constant(m.dof(), 5.0);
  std::size_t i = 0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        forwardDynamics(m, qs[i++ % qs.size()], qd, tau, ForwardDynamicsMethod::CompositeRigidBody));
  }
}
BENCHMARK(BM_ForwardDynamicsCRBA_LLT)->Arg(6)->Arg(7);

}  // namespace
