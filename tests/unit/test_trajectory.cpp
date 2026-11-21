#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/trajectory/cartesian.hpp"
#include "manipulatorcore/trajectory/polynomial.hpp"
#include "manipulatorcore/trajectory/profiles.hpp"
#include "manipulatorcore/trajectory/spline.hpp"
#include "manipulatorcore/trajectory/time_parameterization.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mc {
namespace {

// ------------------------------------------------------------------------------------------------
// Polynomial trajectories
// ------------------------------------------------------------------------------------------------

TEST(Polynomial, CubicBoundaryConditions) {
  Random rng(201);
  for (int k = 0; k < 20; ++k) {
    const VecX q0 = test::randomVector(4, rng, 2.0);
    const VecX qf = test::randomVector(4, rng, 2.0);
    const VecX v0 = test::randomVector(4, rng);
    const VecX vf = test::randomVector(4, rng);
    const double T = rng.uniform(0.5, 3.0);
    const PolynomialTrajectory traj = PolynomialTrajectory::cubic(q0, qf, T, v0, vf);
    const TrajectoryPoint a = traj.sample(0.0);
    const TrajectoryPoint b = traj.sample(T);
    EXPECT_MATRIX_NEAR(a.q, q0, 1e-13);
    EXPECT_MATRIX_NEAR(b.q, qf, 1e-12);
    EXPECT_MATRIX_NEAR(a.qd, v0, 1e-13);
    EXPECT_MATRIX_NEAR(b.qd, vf, 1e-12);
  }
}

TEST(Polynomial, QuinticBoundaryConditions) {
  Random rng(203);
  for (int k = 0; k < 20; ++k) {
    const VecX q0 = test::randomVector(3, rng, 2.0);
    const VecX qf = test::randomVector(3, rng, 2.0);
    const VecX v0 = test::randomVector(3, rng);
    const VecX vf = test::randomVector(3, rng);
    const VecX a0 = test::randomVector(3, rng, 3.0);
    const VecX af = test::randomVector(3, rng, 3.0);
    const double T = rng.uniform(0.5, 3.0);
    const PolynomialTrajectory traj = PolynomialTrajectory::quintic(q0, qf, T, v0, vf, a0, af);
    const TrajectoryPoint a = traj.sample(0.0);
    const TrajectoryPoint b = traj.sample(T);
    EXPECT_MATRIX_NEAR(a.q, q0, 1e-12);
    EXPECT_MATRIX_NEAR(b.q, qf, 1e-11);
    EXPECT_MATRIX_NEAR(a.qd, v0, 1e-12);
    EXPECT_MATRIX_NEAR(b.qd, vf, 1e-11);
    EXPECT_MATRIX_NEAR(a.qdd, a0, 1e-12);
    EXPECT_MATRIX_NEAR(b.qdd, af, 1e-10);
  }
}

TEST(Polynomial, DerivativesMatchFiniteDifferences) {
  const PolynomialTrajectory traj = PolynomialTrajectory::quintic(Vec3(0, 1, -1), Vec3(1, -1, 2), 2.0);
  const double h = 1e-6;
  for (const double t : {0.3, 0.9, 1.7}) {
    const VecX fd_v = (traj.sample(t + h).q - traj.sample(t - h).q) / (2 * h);
    const VecX fd_a = (traj.sample(t + h).qd - traj.sample(t - h).qd) / (2 * h);
    EXPECT_MATRIX_NEAR(traj.sample(t).qd, fd_v, 1e-7);
    EXPECT_MATRIX_NEAR(traj.sample(t).qdd, fd_a, 1e-7);
  }
}

TEST(Polynomial, RejectsInvalidInput) {
  EXPECT_THROW((void)PolynomialTrajectory::cubic(Vec2::Zero(), Vec2::Ones(), 0.0), InvalidArgument);
  EXPECT_THROW((void)PolynomialTrajectory::cubic(Vec2::Zero(), Vec3::Ones(), 1.0), InvalidArgument);
}

// ------------------------------------------------------------------------------------------------
// Scalar profiles
// ------------------------------------------------------------------------------------------------

void expectRestToRest(const ScalarProfile& p, double distance) {
  EXPECT_NEAR(p.sample(0.0).s, 0.0, 1e-12);
  EXPECT_NEAR(p.sample(0.0).sd, 0.0, 1e-12);
  EXPECT_NEAR(p.sample(p.duration()).s, distance, 1e-9 * std::max(1.0, distance));
  EXPECT_NEAR(p.sample(p.duration()).sd, 0.0, 1e-9);
}

TEST(Profiles, PolynomialRestToRest) {
  expectRestToRest(PolynomialProfile::cubic(2.0, 1.5), 2.0);
  expectRestToRest(PolynomialProfile::quintic(2.0, 1.5), 2.0);
  EXPECT_NEAR(PolynomialProfile::quintic(2.0, 1.5).sample(0.0).sdd, 0.0, 1e-15);
}

TEST(Profiles, TrapezoidalCases) {
  // Trapezoidal (cruise reached): D = 2, vmax = 1, amax = 2 -> ta = 0.5, T = 2.5.
  const TrapezoidalProfile trap = TrapezoidalProfile::minimumTime(2.0, 1.0, 2.0);
  EXPECT_NEAR(trap.duration(), 2.5, 1e-15);
  EXPECT_NEAR(trap.peakVelocity(), 1.0, 0.0);
  expectRestToRest(trap, 2.0);
  // Triangular: D = 0.1 < vmax^2 / amax.
  const TrapezoidalProfile tri = TrapezoidalProfile::minimumTime(0.1, 1.0, 2.0);
  EXPECT_NEAR(tri.duration(), 2.0 * std::sqrt(0.05), 1e-15);
  EXPECT_LT(tri.peakVelocity(), 1.0);
  expectRestToRest(tri, 0.1);
  // Prescribed duration.
  const TrapezoidalProfile slow = TrapezoidalProfile::withDuration(2.0, 4.0, 2.0);
  EXPECT_NEAR(slow.duration(), 4.0, 0.0);
  expectRestToRest(slow, 2.0);
  EXPECT_THROW((void)TrapezoidalProfile::withDuration(2.0, 1.0, 2.0), InvalidArgument);
}

struct DoubleSCase {
  double distance, vmax, amax, jmax;
};

class DoubleS : public ::testing::TestWithParam<DoubleSCase> {};

TEST_P(DoubleS, RespectsLimitsAndReachesTarget) {
  const DoubleSCase c = GetParam();
  const DoubleSProfile p = DoubleSProfile::minimumTime(c.distance, c.vmax, c.amax, c.jmax);
  expectRestToRest(p, c.distance);
  const double dt = p.duration() / 4000.0;
  double prev_sdd = p.sample(0.0).sdd;
  double prev_s = 0.0;
  for (int k = 1; k <= 4000; ++k) {
    const ProfileSample x = p.sample(k * dt);
    EXPECT_LE(x.sd, c.vmax * (1 + 1e-9));
    EXPECT_LE(std::abs(x.sdd), c.amax * (1 + 1e-9));
    EXPECT_LE(std::abs(x.sdd - prev_sdd), c.jmax * dt * (1 + 1e-6) + 1e-12);
    EXPECT_GE(x.s, prev_s - 1e-12);  // monotonic
    prev_sdd = x.sdd;
    prev_s = x.s;
  }
  // Limiting jerk can only lengthen the motion compared with the trapezoid.
  EXPECT_GE(p.duration(), TrapezoidalProfile::minimumTime(c.distance, c.vmax, c.amax).duration() - 1e-12);
}

INSTANTIATE_TEST_SUITE_P(AllCases, DoubleS,
                         ::testing::Values(DoubleSCase{10.0, 1.0, 2.0, 10.0},   // vmax and amax reached
                                           DoubleSCase{10.0, 1.0, 5.0, 2.0},    // vmax reached, amax not
                                           DoubleSCase{0.8, 2.0, 1.0, 10.0},    // amax reached, vmax not
                                           DoubleSCase{0.01, 2.0, 5.0, 10.0}),  // neither reached
                         [](const ::testing::TestParamInfo<DoubleSCase>& info) {
                           return "case" + std::to_string(info.index);
                         });

// ------------------------------------------------------------------------------------------------
// Line trajectories and composites
// ------------------------------------------------------------------------------------------------

TEST(LineTrajectory, SynchronizedTrapezoidStaysOnLineAndWithinLimits) {
  const VecX q0 = Vec3(0.0, 1.0, -0.5);
  const VecX q1 = Vec3(1.5, -0.2, 0.3);
  const VecX vmax = Vec3(1.0, 2.0, 0.5);
  const VecX amax = Vec3(3.0, 1.0, 2.0);
  const LineTrajectory traj = LineTrajectory::trapezoidal(q0, q1, vmax, amax);
  EXPECT_MATRIX_NEAR(traj.sample(traj.duration()).q, q1, 1e-12);
  const LimitUsage usage = evaluateLimitUsage(traj, vmax, amax, 1e-3);
  EXPECT_TRUE(usage.within(1e-9));
  EXPECT_GT(usage.max_velocity_ratio, 0.99);  // minimum time: some limit is active
  const VecX dir = (q1 - q0).normalized();
  for (const TrajectoryPoint& p : sampleUniform(traj, 0.01)) {
    const VecX offset = p.q - q0;
    EXPECT_LT((offset - offset.dot(dir) * dir).norm(), 1e-12);
  }
}

TEST(LineTrajectory, DoubleSAndQuintic) {
  const VecX q0 = Vec2(0.0, 0.0);
  const VecX q1 = Vec2(1.0, -2.0);
  const LineTrajectory s = LineTrajectory::doubleS(q0, q1, Vec2(1, 1), Vec2(2, 2), Vec2(10, 10));
  EXPECT_TRUE(evaluateLimitUsage(s, Vec2(1, 1), Vec2(2, 2)).within(1e-9));
  EXPECT_MATRIX_NEAR(s.sample(s.duration()).q, q1, 1e-9);
  const LineTrajectory quintic = LineTrajectory::quintic(q0, q1, 2.0);
  EXPECT_MATRIX_NEAR(quintic.sample(2.0).q, q1, 1e-12);
  EXPECT_MATRIX_NEAR(quintic.sample(1.0).q, (0.5 * (q0 + q1)).eval(), 1e-12);
}

TEST(LineTrajectory, ZeroLengthMotion) {
  const LineTrajectory traj = LineTrajectory::trapezoidal(Vec2(1, 2), Vec2(1, 2), Vec2(1, 1), Vec2(1, 1));
  EXPECT_EQ(traj.duration(), 0.0);
  EXPECT_MATRIX_NEAR(traj.sample(5.0).q, Vec2(1, 2), 0.0);
}

TEST(Composite, StopAtWaypoints) {
  const std::vector<VecX> waypoints = {Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(1, 1), Vec2(0, 2)};
  const VecX vmax = Vec2(1.0, 1.0);
  const VecX amax = Vec2(2.0, 2.0);
  const CompositeTrajectory traj = stopAtWaypointsTrajectory(waypoints, vmax, amax);
  EXPECT_TRUE(evaluateLimitUsage(traj, vmax, amax).within(1e-9));
  EXPECT_MATRIX_NEAR(traj.sample(0.0).q, waypoints.front(), 0.0);
  EXPECT_MATRIX_NEAR(traj.sample(traj.duration()).q, waypoints.back(), 1e-12);
}

TEST(LinearInterpolation, Basics) {
  const LinearInterpolationTrajectory traj({0.0, 1.0, 3.0}, {Vec2(0, 0), Vec2(1, 0), Vec2(1, 4)});
  EXPECT_MATRIX_NEAR(traj.sample(0.5).q, Vec2(0.5, 0.0), 1e-15);
  EXPECT_MATRIX_NEAR(traj.sample(2.0).qd, Vec2(0.0, 2.0), 1e-15);
  EXPECT_THROW(LinearInterpolationTrajectory({0.0, 0.0}, {Vec2(0, 0), Vec2(1, 0)}), InvalidArgument);
}

// ------------------------------------------------------------------------------------------------
// Splines
// ------------------------------------------------------------------------------------------------

TEST(Spline, InterpolatesAndIsC2) {
  const std::vector<double> t = {0.0, 0.7, 1.5, 2.0, 3.2};
  const std::vector<VecX> y = {Vec2(0, 1), Vec2(1, -1), Vec2(0.5, 0.5), Vec2(-1, 2), Vec2(0, 0)};
  const CubicSplineTrajectory traj(t, y, Vec2(0.3, -0.2), Vec2(0, 0));
  for (std::size_t k = 0; k < t.size(); ++k) {
    EXPECT_MATRIX_NEAR(traj.sample(t[k]).q, y[k], 1e-12);
  }
  EXPECT_MATRIX_NEAR(traj.sample(0.0).qd, Vec2(0.3, -0.2), 1e-12);
  EXPECT_MATRIX_NEAR(traj.sample(3.2).qd, Vec2(0.0, 0.0), 1e-12);
  // Continuity of the first and second derivatives at interior knots.
  for (std::size_t k = 1; k + 1 < t.size(); ++k) {
    const double e = 1e-9;
    EXPECT_MATRIX_NEAR(traj.sample(t[k] - e).qd, traj.sample(t[k] + e).qd, 1e-7);
    EXPECT_MATRIX_NEAR(traj.sample(t[k] - e).qdd, traj.sample(t[k] + e).qdd, 1e-6);
  }
}

TEST(Spline, NaturalBoundary) {
  const CubicSpline spline({0.0, 1.0, 2.0, 4.0}, {Vec2(0, 0), Vec2(1, 2), Vec2(0, 1), Vec2(3, 3)});
  EXPECT_MATRIX_NEAR(spline.evaluate(0.0).second, Vec2::Zero(), 1e-12);
  EXPECT_MATRIX_NEAR(spline.evaluate(4.0).second, Vec2::Zero(), 1e-12);
  EXPECT_THROW(CubicSpline({0.0, 0.0}, {Vec2(0, 0), Vec2(1, 1)}), InvalidArgument);
}

// ------------------------------------------------------------------------------------------------
// Cartesian paths
// ------------------------------------------------------------------------------------------------

/// Finite-difference world-aligned derivative [omega; pdot] of a path at s.
Vec6 fdTangent(const CartesianPath& path, double s, double h = 1e-6) {
  const SE3 a = path.pose(s - h);
  const SE3 b = path.pose(s + h);
  Vec6 d;
  d << logSO3(b.rotation() * a.rotation().transpose()) / (2 * h), (b.translation() - a.translation()) / (2 * h);
  return d;
}

TEST(CartesianPath, LineDerivatives) {
  const SE3 start(rotZ(0.2), Vec3(0.5, 0.0, 0.3));
  const SE3 end(rotY(0.8) * rotZ(-0.4), Vec3(0.2, 0.4, 0.6));
  const LinePath path(start, end);
  EXPECT_POSE_NEAR(path.pose(0.0), start, 1e-15);
  EXPECT_POSE_NEAR(path.pose(1.0), end, 1e-12);
  for (const double s : {0.2, 0.5, 0.8}) {
    EXPECT_MATRIX_NEAR(path.tangent(s), fdTangent(path, s), 1e-7);
  }
}

TEST(CartesianPath, ArcGeometryAndDerivatives) {
  const SE3 start(Mat3::Identity(), Vec3(0.6, 0.0, 0.4));
  const Vec3 center(0.4, 0.0, 0.4);
  const CircularArcPath arc(start, center, Vec3::UnitZ(), kPi, rotZ(kPi));
  EXPECT_NEAR(arc.radius(), 0.2, 1e-15);
  EXPECT_NEAR(arc.translationLength(), 0.2 * kPi, 1e-15);
  EXPECT_MATRIX_NEAR(arc.pose(1.0).translation(), Vec3(0.2, 0.0, 0.4), 1e-12);
  for (const double s : {0.1, 0.5, 0.9}) {
    EXPECT_NEAR((arc.pose(s).translation() - center).norm(), 0.2, 1e-12);
    EXPECT_MATRIX_NEAR(arc.tangent(s), fdTangent(arc, s), 1e-7);
    const double h = 1e-5;
    const Vec6 fd2 = (arc.tangent(s + h) - arc.tangent(s - h)) / (2 * h);
    EXPECT_MATRIX_NEAR(arc.curvature(s), fd2, 1e-6);
  }
  const CircularArcPath circle(start, center, Vec3::UnitZ(), kTwoPi, Mat3::Identity());
  EXPECT_POSE_NEAR(circle.pose(1.0), start, 1e-12);
}

TEST(CartesianTrajectory, TwistMatchesPoseDerivative) {
  const auto path = std::make_shared<LinePath>(SE3(Mat3::Identity(), Vec3(0.5, 0.0, 0.3)),
                                               SE3(rotX(0.6), Vec3(0.5, 0.4, 0.3)));
  const CartesianTrajectory traj = CartesianTrajectory::trapezoidal(path, 0.5, 1.0, 1.0, 2.0);
  const double h = 1e-6;
  for (const double frac : {0.1, 0.4, 0.75}) {
    const double t = frac * traj.duration();
    const CartesianSample a = traj.sample(t - h);
    const CartesianSample b = traj.sample(t + h);
    Vec6 fd;
    fd << logSO3(b.pose.rotation() * a.pose.rotation().transpose()) / (2 * h),
        (b.pose.translation() - a.pose.translation()) / (2 * h);
    EXPECT_MATRIX_NEAR(traj.sample(t).twist, fd, 1e-6);
    EXPECT_LE(traj.sample(t).twist.tail<3>().norm(), 0.5 + 1e-9);
    EXPECT_LE(traj.sample(t).twist.head<3>().norm(), 1.0 + 1e-9);
  }
}

// ------------------------------------------------------------------------------------------------
// Time-optimal parameterization
// ------------------------------------------------------------------------------------------------

TEST(TimeParameterization, StraightPathMatchesTrapezoidTime) {
  const VecX q0 = Vec3(0.0, 0.5, -0.3);
  const VecX q1 = Vec3(1.2, -0.4, 0.8);
  const VecX vmax = Vec3(1.0, 1.5, 0.8);
  const VecX amax = Vec3(2.0, 3.0, 1.5);
  TimeParameterizationOptions options;
  options.safety_margin = 0.0;
  const TimeOptimalTrajectory topp =
      timeOptimalParameterization(std::make_shared<StraightJointPath>(q0, q1), vmax, amax, options);
  const double trapezoid = LineTrajectory::trapezoidal(q0, q1, vmax, amax).duration();
  EXPECT_GE(topp.duration(), trapezoid * (1.0 - 1e-6));
  EXPECT_LE(topp.duration(), trapezoid * 1.01);
  EXPECT_MATRIX_NEAR(topp.sample(topp.duration()).q, q1, 1e-12);
  EXPECT_TRUE(evaluateLimitUsage(topp, vmax, amax, 1e-3).within(1e-6));
}

TEST(TimeParameterization, SplinePathRespectsLimitsUnderDenseSampling) {
  const RobotModel model = makeRedundant7R();
  Random rng(211);
  std::vector<VecX> waypoints;
  for (int k = 0; k < 6; ++k) {
    waypoints.push_back(0.5 * test::randomConfiguration(model, rng));
  }
  const auto path = std::make_shared<SplineJointPath>(waypoints);
  const TimeOptimalTrajectory traj =
      timeOptimalParameterization(path, model.velocityLimits(), model.accelerationLimits());
  // Dense sampling (5x finer than the grid on average) must respect the true limits.
  const LimitUsage usage = evaluateLimitUsage(traj, model.velocityLimits(), model.accelerationLimits(),
                                              traj.duration() / 5000.0);
  EXPECT_TRUE(usage.within(1e-9)) << "velocity ratio " << usage.max_velocity_ratio << ", acceleration ratio "
                                  << usage.max_acceleration_ratio;
  EXPECT_GT(std::max(usage.max_velocity_ratio, usage.max_acceleration_ratio), 0.9);  // near time-optimal
  // Endpoints and rest.
  EXPECT_MATRIX_NEAR(traj.sample(0.0).q, waypoints.front(), 1e-12);
  EXPECT_MATRIX_NEAR(traj.sample(traj.duration()).q, waypoints.back(), 1e-9);
  EXPECT_MATRIX_NEAR(traj.sample(0.0).qd, VecX::Zero(7), 1e-12);
  // The trajectory follows the path and passes through every waypoint.
  for (const VecX& w : waypoints) {
    double best = 1e9;
    for (const TrajectoryPoint& p : sampleUniform(traj, traj.duration() / 4000.0)) {
      best = std::min(best, (p.q - w).norm());
    }
    EXPECT_LT(best, 5e-3);
  }
}

TEST(TimeParameterization, VelocityScaleSlowsDown) {
  const auto path = std::make_shared<StraightJointPath>(Vec2(0, 0), Vec2(2, 1));
  const double fast = timeOptimalParameterization(path, Vec2(1, 1), Vec2(2, 2)).duration();
  TimeParameterizationOptions half;
  half.velocity_scale = 0.5;
  half.acceleration_scale = 0.5;
  const double slow = timeOptimalParameterization(path, Vec2(1, 1), Vec2(2, 2), half).duration();
  EXPECT_GT(slow, 1.5 * fast);
}

TEST(TimeParameterization, RejectsInvalidInput) {
  const auto path = std::make_shared<StraightJointPath>(Vec2(0, 0), Vec2(1, 1));
  EXPECT_THROW((void)timeOptimalParameterization(path, Vec2(1, -1), Vec2(1, 1)), InvalidArgument);
  EXPECT_THROW((void)timeOptimalParameterization(nullptr, Vec2(1, 1), Vec2(1, 1)), InvalidArgument);
  EXPECT_THROW(StraightJointPath(Vec2(1, 1), Vec2(1, 1)), InvalidArgument);
}

}  // namespace
}  // namespace mc
