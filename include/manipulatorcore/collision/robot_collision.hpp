#pragma once

/// @file robot_collision.hpp
/// @brief Collision checking of a serial manipulator against itself and a static environment.
///
/// Bodies are indexed 0..n-1; the base is index -1. Self-collision between adjacent bodies
/// (|i - j| == 1, including base/body 0) is excluded by default because their geometry overlaps at
/// the joint; further pairs can be excluded with allowCollision(). Each query computes forward
/// kinematics internally, so a const checker can be shared between threads.

#include "manipulatorcore/collision/aabb.hpp"
#include "manipulatorcore/collision/distance.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <vector>

namespace mc {

struct CollisionCheckerOptions {
  bool check_self_collision = true;
  bool check_environment = true;
  /// The base is usually mounted on (and touching) the environment; exclude it by default.
  bool check_base_against_environment = false;
  /// Robot-environment distances below this margin count as collisions [m].
  double safety_margin = 0.0;
  /// Robot self-distances below this margin count as collisions [m]. Kept separate because the
  /// built-in arms have small permanent clearances between non-adjacent wrist bodies.
  double self_collision_margin = 0.0;
};

/// Which pair of objects a distance refers to.
struct CollisionPairInfo {
  enum class Kind { Self, Environment };
  Kind kind = Kind::Environment;
  int body_a = 0;   ///< robot body (-1 = base)
  int shape_a = 0;  ///< shape index within body_a
  int body_b = 0;   ///< robot body for Self, obstacle index for Environment
  int shape_b = 0;  ///< shape index within body_b (0 for obstacles)
  DistanceResult result;
};

class RobotCollisionChecker {
 public:
  explicit RobotCollisionChecker(RobotModel model, std::vector<Shape> obstacles = {},
                                 const CollisionCheckerOptions& options = {});

  [[nodiscard]] const RobotModel& model() const { return model_; }
  [[nodiscard]] const std::vector<Shape>& obstacles() const { return obstacles_; }
  [[nodiscard]] const CollisionCheckerOptions& options() const { return options_; }

  void setObstacles(std::vector<Shape> obstacles);
  void addObstacle(Shape obstacle);
  void setOptions(const CollisionCheckerOptions& options);

  /// Excludes (allowed = true) or re-enables self-collision checks between two bodies (-1 = base).
  void allowCollision(int body_a, int body_b, bool allowed = true);
  [[nodiscard]] bool isCollisionAllowed(int body_a, int body_b) const;

  /// True if no checked pair is closer than the safety margin. Stops at the first collision.
  [[nodiscard]] bool isCollisionFree(const VecX& q) const;

  /// Smallest signed distance over all checked pairs (+infinity if nothing is checked).
  [[nodiscard]] double minimumDistance(const VecX& q) const;

  /// Every checked pair whose distance is below @p report_distance (broad phase prunes the rest),
  /// sorted by increasing distance.
  [[nodiscard]] std::vector<CollisionPairInfo> pairDistances(const VecX& q, double report_distance) const;

  /// For each body, the smallest distance to any checked partner (+infinity if none).
  [[nodiscard]] VecX bodyClearances(const VecX& q) const;

 private:
  struct PosedShape {
    int body;
    int shape;
    const ShapeGeometry* geometry;
    SE3 pose;
    AABB box;
  };

  void validateObstacles();
  std::vector<PosedShape> poseRobot(const VecX& q) const;
  [[nodiscard]] bool selfPairChecked(int a, int b) const;

  // Visits every checked pair that survives the broad phase with lower bound < cutoff(). The
  // visitor returns false to stop early.
  template <typename Visitor, typename Cutoff>
  void forEachPair(const std::vector<PosedShape>& robot, const Visitor& visit, const Cutoff& cutoff) const;

  RobotModel model_;
  std::vector<Shape> obstacles_;
  std::vector<AABB> obstacle_boxes_;
  CollisionCheckerOptions options_;
  std::vector<std::vector<bool>> allowed_;  // (n+1) x (n+1), index 0 = base
};

}  // namespace mc
