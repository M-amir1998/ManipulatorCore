#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/planning/path_processing.hpp"
#include "manipulatorcore/planning/planner.hpp"
#include "manipulatorcore/planning/scenarios.hpp"
#include "robot_fixtures.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mc {
namespace {

// A 2-D unit-square world with a vertical wall at x in [0.45, 0.55]. `gap` opens a slit of the
// given width centred at y = 0.5; gap = 0 blocks the wall completely.
std::shared_ptr<const MotionValidator> wallWorld(double gap, double resolution = 0.005) {
  const JointSpace space(Vec2(0.0, 0.0), Vec2(1.0, 1.0));
  const auto valid = [gap](const VecX& q) {
    const bool in_wall = q(0) > 0.45 && q(0) < 0.55;
    const bool in_slit = std::abs(q(1) - 0.5) < 0.5 * gap;
    return !in_wall || in_slit;
  };
  return std::make_shared<const MotionValidator>(space, valid, resolution);
}

const PlanningProblem kAcrossWall{Vec2(0.1, 0.1), {Vec2(0.9, 0.9)}};

PlannerSettings settings2D(std::uint64_t seed = 1) {
  PlannerSettings s;
  s.seed = seed;
  s.max_step = 0.1;
  s.max_iterations = 20000;
  s.max_rewire_radius = 0.3;
  return s;
}

// RRT* always spends its whole (anytime) budget with linear neighbour scans; keep that budget small
// enough for unoptimized (Debug, sanitizer) builds.
PlannerSettings settingsFor(const std::string& planner, std::uint64_t seed = 1) {
  PlannerSettings s = settings2D(seed);
  if (planner == "RRT*") {
    s.max_iterations = 2500;
  }
  return s;
}

// ------------------------------------------------------------------------------------------------
// Joint space and validation
// ------------------------------------------------------------------------------------------------

TEST(JointSpace, MetricAndSteering) {
  const JointSpace space(Vec2(-1, -1), Vec2(1, 1), Vec2(4.0, 1.0));
  EXPECT_NEAR(space.distance(Vec2(0, 0), Vec2(1, 0)), 2.0, 1e-15);  // weight 4 -> factor 2
  const VecX s = space.steer(Vec2(0, 0), Vec2(0, 1), 0.25);
  EXPECT_MATRIX_NEAR(s, Vec2(0, 0.25), 1e-15);
  EXPECT_MATRIX_NEAR(space.steer(Vec2(0, 0), Vec2(0, 0.1), 0.25), Vec2(0, 0.1), 0.0);
  EXPECT_TRUE(space.contains(Vec2(1, -1)));
  EXPECT_FALSE(space.contains(Vec2(1.1, 0)));
  EXPECT_NEAR(space.measure(), 2.0 * 2.0 * 2.0, 1e-15);
  EXPECT_THROW(JointSpace(Vec2(1, 0), Vec2(0, 1)), InvalidArgument);
}

TEST(MotionValidator, ChecksIntermediateStatesNotOnlyEndpoints) {
  const auto validator = wallWorld(0.0);
  // Both endpoints are free but the straight edge crosses the wall.
  EXPECT_TRUE(validator->isValid(Vec2(0.1, 0.5)));
  EXPECT_TRUE(validator->isValid(Vec2(0.9, 0.5)));
  EXPECT_FALSE(validator->isEdgeValid(Vec2(0.1, 0.5), Vec2(0.9, 0.5)));
  EXPECT_TRUE(validator->isEdgeValid(Vec2(0.1, 0.5), Vec2(0.4, 0.9)));
  EXPECT_GT(validator->stateChecks(), 50U);
}

// ------------------------------------------------------------------------------------------------
// Planners
// ------------------------------------------------------------------------------------------------

class PlannerBehaviour : public ::testing::TestWithParam<std::string> {};

TEST_P(PlannerBehaviour, OpenEnvironment) {
  const JointSpace space(Vec2(0, 0), Vec2(1, 1));
  const auto validator = std::make_shared<const MotionValidator>(space, [](const VecX&) { return true; }, 0.01);
  auto planner = makePlanner(GetParam(), validator, settingsFor(GetParam()));
  const PlannerResult r = planner->solve(kAcrossWall);
  ASSERT_TRUE(r.success()) << toString(r.status);
  EXPECT_MATRIX_NEAR(r.path.front(), kAcrossWall.start, 0.0);
  EXPECT_MATRIX_NEAR(r.path.back(), kAcrossWall.goals.front(), 0.0);
  EXPECT_GE(r.path_length, std::sqrt(2.0) * 0.8 - 1e-12);
}

TEST_P(PlannerBehaviour, NarrowPassageYieldsValidPath) {
  const auto validator = wallWorld(0.08);
  auto planner = makePlanner(GetParam(), validator, settingsFor(GetParam(), 3));
  const PlannerResult r = planner->solve(kAcrossWall);
  ASSERT_TRUE(r.success()) << GetParam() << ": " << toString(r.status);
  EXPECT_TRUE(validator->isPathValid(r.path));
  for (const VecX& q : r.path) {
    EXPECT_TRUE(validator->space().contains(q));
  }
  EXPECT_GT(r.stats.collision_checks, 0U);
  EXPECT_GT(r.stats.nodes, 2U);
}

TEST_P(PlannerBehaviour, BlockedEnvironmentFailsGracefully) {
  const auto validator = wallWorld(0.0);
  PlannerSettings s = settings2D();
  s.max_iterations = 1500;
  auto planner = makePlanner(GetParam(), validator, s);
  const PlannerResult r = planner->solve(kAcrossWall);
  EXPECT_FALSE(r.success());
  EXPECT_EQ(r.status, PlannerStatus::IterationLimit);
  EXPECT_TRUE(r.path.empty());
}

TEST_P(PlannerBehaviour, DeterministicForSeed) {
  const auto validator = wallWorld(0.15);
  auto a = makePlanner(GetParam(), validator, settingsFor(GetParam(), 11));
  auto b = makePlanner(GetParam(), validator, settingsFor(GetParam(), 11));
  const PlannerResult ra = a->solve(kAcrossWall);
  const PlannerResult rb = b->solve(kAcrossWall);
  ASSERT_TRUE(ra.success());
  ASSERT_EQ(ra.path.size(), rb.path.size());
  for (std::size_t i = 0; i < ra.path.size(); ++i) {
    EXPECT_EQ(ra.path[i], rb.path[i]);
  }
  EXPECT_EQ(ra.stats.collision_checks, rb.stats.collision_checks);
}

TEST_P(PlannerBehaviour, InvalidStartAndGoal) {
  const auto validator = wallWorld(0.0);
  auto planner = makePlanner(GetParam(), validator, settingsFor(GetParam()));
  EXPECT_EQ(planner->solve({Vec2(0.5, 0.5), {Vec2(0.9, 0.9)}}).status, PlannerStatus::InvalidStart);
  EXPECT_EQ(planner->solve({Vec2(0.1, 0.1), {Vec2(0.5, 0.2)}}).status, PlannerStatus::InvalidGoal);
}

INSTANTIATE_TEST_SUITE_P(AllPlanners, PlannerBehaviour, ::testing::Values("RRT", "RRT-Connect", "RRT*", "PRM"),
                         [](const ::testing::TestParamInfo<std::string>& param) {
                           std::string name = param.param;
                           std::erase(name, '-');
                           std::replace(name.begin(), name.end(), '*', 'S');
                           return name;
                         });

TEST(RRTStar, MoreIterationsDoNotIncreaseCost) {
  const auto validator = wallWorld(0.2);
  PlannerSettings few = settings2D(5);
  few.max_iterations = 600;
  PlannerSettings many = few;
  many.max_iterations = 3000;
  const PlannerResult a = RRTStarPlanner(validator, few).solve(kAcrossWall);
  const PlannerResult b = RRTStarPlanner(validator, many).solve(kAcrossWall);
  ASSERT_TRUE(a.success());
  ASSERT_TRUE(b.success());
  EXPECT_LE(b.path_length, a.path_length + 1e-9);
  // Optimal cost through the slit is sqrt(0.4^2 + 0.4^2) * 2 ~= 1.131.
  EXPECT_LT(b.path_length, 1.25);
}

TEST(RRTStar, ShorterThanRRTOnAverage) {
  const auto validator = wallWorld(0.2);
  double rrt = 0.0;
  double star = 0.0;
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    PlannerSettings s = settings2D(seed);
    s.max_iterations = 2000;
    rrt += RRTPlanner(validator, s).solve(kAcrossWall).path_length;
    star += RRTStarPlanner(validator, s).solve(kAcrossWall).path_length;
  }
  EXPECT_LT(star, rrt);
}

TEST(PRM, RoadmapIsReusedAcrossQueries) {
  const auto validator = wallWorld(0.12);
  PRMPlanner prm(validator, settings2D(2));
  const PlannerResult first = prm.solve(kAcrossWall);
  ASSERT_TRUE(first.success());
  const std::size_t size_after_first = prm.roadmapSize();
  const PlannerResult second = prm.solve({Vec2(0.2, 0.8), {Vec2(0.8, 0.2)}});
  ASSERT_TRUE(second.success());
  EXPECT_LE(second.stats.iterations, first.stats.iterations);
  EXPECT_LE(prm.roadmapSize(), size_after_first + 2 + static_cast<std::size_t>(second.stats.iterations));
  EXPECT_GT(prm.roadmapEdges(), 0U);
}

// ------------------------------------------------------------------------------------------------
// Path processing
// ------------------------------------------------------------------------------------------------

TEST(PathProcessing, ShortcuttingShortensAndStaysValid) {
  const auto validator = wallWorld(0.1);
  const PlannerResult r = RRTPlanner(validator, settings2D(4)).solve(kAcrossWall);
  ASSERT_TRUE(r.success());
  const ShortcutResult s = shortcutPath(*validator, r.path, {.iterations = 300, .seed = 1});
  EXPECT_LE(s.final_length, s.initial_length + 1e-12);
  EXPECT_LT(s.final_length, 0.9 * s.initial_length);
  EXPECT_TRUE(validator->isPathValid(s.path));
  EXPECT_MATRIX_NEAR(s.path.front(), r.path.front(), 0.0);
  EXPECT_MATRIX_NEAR(s.path.back(), r.path.back(), 0.0);
}

TEST(PathProcessing, DensifyRespectsStep) {
  const JointSpace space(Vec2(0, 0), Vec2(1, 1));
  const auto dense = densifyPath(space, {Vec2(0, 0), Vec2(1, 0), Vec2(1, 1)}, 0.1);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    EXPECT_LE(space.distance(dense[i - 1], dense[i]), 0.1 + 1e-12);
  }
  EXPECT_NEAR(pathLength(space, dense), 2.0, 1e-12);
}

TEST(PathProcessing, SplineSmoothingIsRevalidated) {
  const auto validator = wallWorld(0.1, 0.002);
  const std::vector<VecX> corner_path = {Vec2(0.1, 0.5), Vec2(0.5, 0.5), Vec2(0.9, 0.5), Vec2(0.9, 0.9)};
  ASSERT_TRUE(validator->isPathValid(corner_path));
  const auto spline = smoothPathWithSpline(*validator, corner_path, {.check_resolution = 0.002});
  ASSERT_TRUE(spline.has_value());
  const double L = (*spline)->length();
  for (int k = 0; k <= 2000; ++k) {
    EXPECT_TRUE(validator->isValid((*spline)->position(L * k / 2000.0)));
  }
}

// ------------------------------------------------------------------------------------------------
// Scenarios
// ------------------------------------------------------------------------------------------------

TEST(Scenarios, StartsAndGoalsAreCollisionFree) {
  std::vector<PlanningScenario> all = redundant7RScenarios();
  all.push_back(planar2RScenario());
  all.push_back(planar2RNarrowPassageScenario());
  EXPECT_EQ(all.size(), 8U);
  for (const PlanningScenario& s : all) {
    const RobotCollisionChecker checker(s.robot, s.obstacles);
    EXPECT_TRUE(s.robot.withinLimits(s.start)) << s.name;
    EXPECT_TRUE(s.robot.withinLimits(s.goal)) << s.name;
    EXPECT_TRUE(checker.isCollisionFree(s.start)) << s.name << " start";
    EXPECT_TRUE(checker.isCollisionFree(s.goal)) << s.name << " goal";
  }
}

TEST(Scenarios, ObstacleScenariosRequirePlanning) {
  // The straight joint-space line between start and goal collides for every cluttered scenario.
  for (const PlanningScenario& s : redundant7RScenarios()) {
    if (s.name == "open_reach") {
      continue;
    }
    const RobotCollisionChecker checker(s.robot, s.obstacles);
    const MotionValidator validator(
        JointSpace::fromModel(s.robot), [&checker](const VecX& q) { return checker.isCollisionFree(q); }, 0.01);
    EXPECT_FALSE(validator.isEdgeValid(s.start, s.goal)) << s.name;
  }
}

}  // namespace
}  // namespace mc
