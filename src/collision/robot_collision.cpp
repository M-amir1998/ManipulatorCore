#include "manipulatorcore/collision/robot_collision.hpp"

#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

namespace mc {

RobotCollisionChecker::RobotCollisionChecker(RobotModel model, std::vector<Shape> obstacles,
                                             const CollisionCheckerOptions& options)
    : model_(std::move(model)), obstacles_(std::move(obstacles)), options_(options) {
  setOptions(options);
  validateObstacles();
  const auto n = static_cast<std::size_t>(model_.dof()) + 1;
  allowed_.assign(n, std::vector<bool>(n, false));
  for (int i = -1; i < model_.dof(); ++i) {
    allowCollision(i, i, true);
    if (i + 1 < model_.dof()) {
      allowCollision(i, i + 1, true);  // adjacent bodies overlap at their shared joint
    }
  }
}

void RobotCollisionChecker::setObstacles(std::vector<Shape> obstacles) {
  obstacles_ = std::move(obstacles);
  validateObstacles();
}

void RobotCollisionChecker::addObstacle(Shape obstacle) {
  obstacles_.push_back(std::move(obstacle));
  validateObstacles();
}

void RobotCollisionChecker::setOptions(const CollisionCheckerOptions& options) {
  require(std::isfinite(options.safety_margin) && options.safety_margin >= 0.0,
          "CollisionCheckerOptions: safety_margin must be finite and non-negative");
  require(std::isfinite(options.self_collision_margin) && options.self_collision_margin >= 0.0,
          "CollisionCheckerOptions: self_collision_margin must be finite and non-negative");
  options_ = options;
}

void RobotCollisionChecker::validateObstacles() {
  obstacle_boxes_.clear();
  for (const Shape& s : obstacles_) {
    validateShape(s);
    obstacle_boxes_.push_back(AABB::of(s.geometry, s.pose));
  }
}

void RobotCollisionChecker::allowCollision(int body_a, int body_b, bool allowed) {
  require(body_a >= -1 && body_a < model_.dof() && body_b >= -1 && body_b < model_.dof(),
          "RobotCollisionChecker::allowCollision: body index out of range");
  allowed_[static_cast<std::size_t>(body_a + 1)][static_cast<std::size_t>(body_b + 1)] = allowed;
  allowed_[static_cast<std::size_t>(body_b + 1)][static_cast<std::size_t>(body_a + 1)] = allowed;
}

bool RobotCollisionChecker::isCollisionAllowed(int body_a, int body_b) const {
  require(body_a >= -1 && body_a < model_.dof() && body_b >= -1 && body_b < model_.dof(),
          "RobotCollisionChecker::isCollisionAllowed: body index out of range");
  return allowed_[static_cast<std::size_t>(body_a + 1)][static_cast<std::size_t>(body_b + 1)];
}

bool RobotCollisionChecker::selfPairChecked(int a, int b) const {
  return !allowed_[static_cast<std::size_t>(a + 1)][static_cast<std::size_t>(b + 1)];
}

std::vector<RobotCollisionChecker::PosedShape> RobotCollisionChecker::poseRobot(const VecX& q) const {
  const KinematicState state = forwardKinematics(model_, q);
  std::vector<PosedShape> shapes;
  const auto& base = model_.baseCollision();
  for (std::size_t k = 0; k < base.size(); ++k) {
    shapes.push_back({-1, static_cast<int>(k), &base[k].geometry, base[k].pose, AABB::of(base[k].geometry, base[k].pose)});
  }
  for (int i = 0; i < model_.dof(); ++i) {
    const auto& body_shapes = model_.body(i).collision;
    const SE3& T = state.T_base_body[static_cast<std::size_t>(i)];
    for (std::size_t k = 0; k < body_shapes.size(); ++k) {
      const SE3 pose = T * body_shapes[k].pose;
      shapes.push_back({i, static_cast<int>(k), &body_shapes[k].geometry, pose, AABB::of(body_shapes[k].geometry, pose)});
    }
  }
  return shapes;
}

namespace {

// The AABB gap bounds the unsigned distance from below. It is 0 whenever the boxes overlap, while
// the signed distance may then be negative, so only a strictly positive bound allows pruning.
bool prunable(double lower_bound, double cutoff) { return lower_bound > 0.0 && lower_bound >= cutoff; }

}  // namespace

template <typename Visitor, typename Cutoff>
void RobotCollisionChecker::forEachPair(const std::vector<PosedShape>& robot, Visitor&& visit, Cutoff&& cutoff) const {
  if (options_.check_environment) {
    for (const PosedShape& rs : robot) {
      if (rs.body < 0 && !options_.check_base_against_environment) {
        continue;
      }
      for (std::size_t o = 0; o < obstacles_.size(); ++o) {
        if (prunable(rs.box.distanceLowerBound(obstacle_boxes_[o]), cutoff())) {
          continue;  // broad phase
        }
        CollisionPairInfo info;
        info.kind = CollisionPairInfo::Kind::Environment;
        info.body_a = rs.body;
        info.shape_a = rs.shape;
        info.body_b = static_cast<int>(o);
        info.shape_b = 0;
        info.result = computeDistance(*rs.geometry, rs.pose, obstacles_[o].geometry, obstacles_[o].pose);
        if (!visit(info)) {
          return;
        }
      }
    }
  }
  if (options_.check_self_collision) {
    for (std::size_t i = 0; i < robot.size(); ++i) {
      for (std::size_t j = i + 1; j < robot.size(); ++j) {
        const PosedShape& a = robot[i];
        const PosedShape& b = robot[j];
        if (a.body == b.body || !selfPairChecked(a.body, b.body)) {
          continue;
        }
        if (prunable(a.box.distanceLowerBound(b.box), cutoff())) {
          continue;
        }
        CollisionPairInfo info;
        info.kind = CollisionPairInfo::Kind::Self;
        info.body_a = a.body;
        info.shape_a = a.shape;
        info.body_b = b.body;
        info.shape_b = b.shape;
        info.result = computeDistance(*a.geometry, a.pose, *b.geometry, b.pose);
        if (!visit(info)) {
          return;
        }
      }
    }
  }
}

bool RobotCollisionChecker::isCollisionFree(const VecX& q) const {
  const std::vector<PosedShape> robot = poseRobot(q);
  bool free = true;
  const double env_margin = options_.safety_margin;
  const double self_margin = options_.self_collision_margin;
  const double cutoff = std::max(env_margin, self_margin);
  forEachPair(
      robot,
      [&](const CollisionPairInfo& info) {
        const double margin = info.kind == CollisionPairInfo::Kind::Self ? self_margin : env_margin;
        if (info.result.distance < margin) {
          free = false;
          return false;
        }
        return true;
      },
      [&] { return cutoff; });
  return free;
}

double RobotCollisionChecker::minimumDistance(const VecX& q) const {
  const std::vector<PosedShape> robot = poseRobot(q);
  double best = std::numeric_limits<double>::infinity();
  forEachPair(
      robot,
      [&](const CollisionPairInfo& info) {
        best = std::min(best, info.result.distance);
        return true;
      },
      [&] { return best; });
  return best;
}

std::vector<CollisionPairInfo> RobotCollisionChecker::pairDistances(const VecX& q, double report_distance) const {
  const std::vector<PosedShape> robot = poseRobot(q);
  std::vector<CollisionPairInfo> out;
  forEachPair(
      robot,
      [&](const CollisionPairInfo& info) {
        if (info.result.distance < report_distance) {
          out.push_back(info);
        }
        return true;
      },
      [&] { return report_distance; });
  std::sort(out.begin(), out.end(),
            [](const CollisionPairInfo& a, const CollisionPairInfo& b) { return a.result.distance < b.result.distance; });
  return out;
}

VecX RobotCollisionChecker::bodyClearances(const VecX& q) const {
  const std::vector<PosedShape> robot = poseRobot(q);
  VecX clearance = VecX::Constant(model_.dof(), std::numeric_limits<double>::infinity());
  auto update = [&](int body, double d) {
    if (body >= 0) {
      clearance(body) = std::min(clearance(body), d);
    }
  };
  forEachPair(
      robot,
      [&](const CollisionPairInfo& info) {
        update(info.body_a, info.result.distance);
        if (info.kind == CollisionPairInfo::Kind::Self) {
          update(info.body_b, info.result.distance);
        }
        return true;
      },
      [] { return std::numeric_limits<double>::infinity(); });
  return clearance;
}

}  // namespace mc
