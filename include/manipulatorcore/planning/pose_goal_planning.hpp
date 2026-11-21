#pragma once

/// @file pose_goal_planning.hpp
/// @brief Integrated pipeline from a Cartesian goal pose to a time-parameterized, collision-free
/// joint trajectory:
///
///   target pose -> multi-start IK -> collision filtering -> plan to each IK candidate
///   -> shortcut -> select the shortest -> spline smoothing (revalidated) -> time-optimal timing

#include "manipulatorcore/collision/robot_collision.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/planning/path_processing.hpp"
#include "manipulatorcore/planning/planner.hpp"
#include "manipulatorcore/trajectory/joint_trajectory.hpp"

#include <memory>
#include <string>
#include <vector>

namespace mc {

struct PoseGoalPlanningOptions {
  MultiStartIKOptions ik;
  std::string planner = "RRT-Connect";
  PlannerSettings planner_settings;
  double edge_resolution = 0.02;
  /// Robot-environment clearance enforced during IK and planning [m]. Edges are checked at
  /// discrete states, so the clearance must cover the motion between two checks
  /// (about edge_resolution / 2 times the arm's reach). The final trajectory is re-verified
  /// against the caller's checker without this margin.
  double clearance = 0.02;
  /// Time step of the final collision verification of the timed trajectory [s].
  double verification_dt = 0.005;
  int max_goal_candidates = 4;
  ShortcutOptions shortcut;
  bool smooth = true;
  SplineSmoothingOptions smoothing;
  TimeParameterizationOptions timing;
};

struct PoseGoalPlanningResult {
  bool success = false;
  std::string message;
  std::vector<VecX> ik_candidates;      ///< collision-free IK solutions, closest to start first
  int chosen_candidate = -1;
  std::vector<VecX> raw_path;           ///< planner output for the chosen candidate
  std::vector<VecX> path;               ///< after shortcutting
  double raw_length = 0.0;
  double path_length = 0.0;
  bool smoothed = false;                ///< true if the timed trajectory follows a validated spline
  std::shared_ptr<const JointTrajectory> trajectory;
  PlannerStatistics planner_stats;      ///< accumulated over all candidates
};

/// Plans from @p start to a configuration whose end-effector pose is @p target.
[[nodiscard]] PoseGoalPlanningResult planToPose(const RobotCollisionChecker& checker, const VecX& start,
                                                const SE3& target, const PoseGoalPlanningOptions& options = {});

/// Times a validated joint path: spline + time-optimal parameterization when smoothing succeeds,
/// otherwise synchronized trapezoids that stop at each waypoint.
[[nodiscard]] std::shared_ptr<const JointTrajectory> timeParameterizePath(const MotionValidator& validator,
                                                                          const RobotModel& model,
                                                                          const std::vector<VecX>& path, bool smooth,
                                                                          const SplineSmoothingOptions& smoothing,
                                                                          const TimeParameterizationOptions& timing,
                                                                          bool* used_spline = nullptr);

}  // namespace mc
