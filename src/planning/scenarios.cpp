#include "manipulatorcore/planning/scenarios.hpp"

#include "manipulatorcore/model/builtin_robots.hpp"

#include <initializer_list>

namespace mc {

namespace {

/// Axis-aligned box from its min and max corners.
Shape box(const Vec3& lo, const Vec3& hi, const char* name) {
  return makeBox(SE3::fromTranslation(0.5 * (lo + hi)), 0.5 * (hi - lo), name);
}

VecX config(std::initializer_list<double> values) {
  VecX q(static_cast<Index>(values.size()));
  Index i = 0;
  for (const double v : values) {
    q(i++) = v;
  }
  return q;
}

}  // namespace

std::vector<PlanningScenario> redundant7RScenarios() {
  const RobotModel robot = makeRedundant7R();
  const Shape floor = makeHalfSpace(Vec3::UnitZ(), 0.0, "floor");
  std::vector<PlanningScenario> scenarios;

  // Joint configurations were obtained once with collision-filtered multi-start IK for the
  // end-effector poses described below and are fixed here so the scenarios stay deterministic.
  scenarios.push_back({"open_reach",
                       "Free space apart from the floor; reach from the home pose to the side.",
                       robot,
                       {floor},
                       redundant7RHome(),
                       config({-0.3122, 0.3932, -0.8544, 0.9604, -0.0920, 1.2208, 2.1223})});

  scenarios.push_back({"obstacle_avoidance",
                       "A pillar in front of the robot separates two low, downward-pointing grasp poses.",
                       robot,
                       {floor, box(Vec3(0.48, -0.07, 0.0), Vec3(0.62, 0.07, 0.70), "pillar")},
                       config({0.5137, 0.8521, 0.4530, 1.3503, -0.3948, 1.0282, -2.1059}),
                       config({-0.5137, 0.8521, -0.4530, 1.3503, 0.3948, 1.0282, 2.1059})});

  scenarios.push_back({"narrow_passage",
                       "Reach through a 0.36 m x 0.30 m window in a wall in front of the robot.",
                       robot,
                       {floor, box(Vec3(0.50, -0.70, 0.0), Vec3(0.54, 0.70, 0.45), "wall_bottom"),
                        box(Vec3(0.50, -0.70, 0.75), Vec3(0.54, 0.70, 1.40), "wall_top"),
                        box(Vec3(0.50, 0.18, 0.45), Vec3(0.54, 0.70, 0.75), "wall_left"),
                        box(Vec3(0.50, -0.70, 0.45), Vec3(0.54, -0.18, 0.75), "wall_right")},
                       config({-0.4355, -0.6614, 0.8304, 1.4235, -0.1212, 1.0205, 0.5953}),
                       config({0.0, 0.3655, 0.0, 1.5958, 0.0, -0.3905, 0.0})});

  scenarios.push_back({"shelf_reach",
                       "Insert the tool into the middle compartment of a three-level shelf.",
                       robot,
                       {floor, box(Vec3(0.45, -0.35, 0.37), Vec3(0.78, 0.35, 0.40), "shelf_bottom"),
                        box(Vec3(0.45, -0.35, 0.70), Vec3(0.78, 0.35, 0.73), "shelf_middle"),
                        box(Vec3(0.45, -0.35, 1.00), Vec3(0.78, 0.35, 1.03), "shelf_top"),
                        box(Vec3(0.45, 0.35, 0.0), Vec3(0.78, 0.38, 1.03), "shelf_left"),
                        box(Vec3(0.45, -0.38, 0.0), Vec3(0.78, -0.35, 1.03), "shelf_right"),
                        box(Vec3(0.78, -0.38, 0.0), Vec3(0.81, 0.38, 1.03), "shelf_back")},
                       config({-0.4355, -0.6614, 0.8304, 1.4235, -0.1212, 1.0205, 0.5953}),
                       config({0.0, 0.2501, 0.0, 1.8940, 0.0, -0.5732, 0.0})});

  scenarios.push_back({"tabletop_reach",
                       "Move between two top-down grasps on a table with a box standing between them.",
                       robot,
                       {floor, box(Vec3(0.30, -0.45, 0.18), Vec3(0.80, 0.45, 0.22), "table"),
                        box(Vec3(0.48, -0.06, 0.22), Vec3(0.60, 0.06, 0.45), "box_on_table")},
                       config({0.2374, 0.7157, 0.4990, 1.4033, -0.3577, 1.1126, -2.3502}),
                       config({-0.2374, 0.7157, -0.4990, 1.4033, 0.3577, 1.1126, 2.3502})});

  scenarios.push_back({"around_obstacle",
                       "Swing from the left to the right side of a large ball at chest height.",
                       robot,
                       {floor, makeSphere(Vec3(0.55, 0.0, 0.70), 0.20, "ball")},
                       config({0.9866, 0.3269, 0.1630, 1.6539, -1.7961, 1.1982, 2.0080}),
                       config({-0.9866, 0.3269, -0.1630, 1.6539, 1.7961, 1.1982, -2.0080})});
  return scenarios;
}

PlanningScenario planar2RScenario() {
  return {"planar_2r_obstacles",
          "Planar two-link arm among three circular obstacles.",
          makePlanar2R(),
          {makeSphere(Vec3(1.05, 0.75, 0.0), 0.22, "disc_a"), makeSphere(Vec3(-0.55, 0.95, 0.0), 0.25, "disc_b"),
           makeSphere(Vec3(0.90, -0.85, 0.0), 0.18, "disc_c")},
          config({-1.3, 0.4}),
          config({1.2, -0.4})};
}

PlanningScenario planar2RNarrowPassageScenario() {
  return {"planar_2r_narrow_passage",
          "Planar two-link arm reaching through a 0.25 m slit in a wall.",
          makePlanar2R(),
          {box(Vec3(-2.0, 1.0, -0.1), Vec3(0.55, 1.1, 0.1), "wall_left"),
           box(Vec3(0.80, 1.0, -0.1), Vec3(2.0, 1.1, 0.1), "wall_right")},
          config({-0.2, 1.0}),
          config({0.600, 1.166})};
}

}  // namespace mc
