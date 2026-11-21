#pragma once

/// @file planner.hpp
/// @brief Sampling-based joint-space motion planners.
///
/// All planners are deterministic for a given seed when time_limit is zero (iteration budgets
/// only). Edges are always validated at intermediate states, never only at their endpoints.
///
/// References:
///  - S. M. LaValle, "Rapidly-exploring random trees", TR 98-11, 1998; Planning Algorithms, 2006
///  - J. J. Kuffner and S. M. LaValle, "RRT-Connect", ICRA 2000
///  - S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning",
///    IJRR 30(7), 2011
///  - L. E. Kavraki, P. Svestka, J.-C. Latombe and M. H. Overmars, "Probabilistic roadmaps for path
///    planning in high-dimensional configuration spaces", IEEE T-RA 12(4), 1996

#include "manipulatorcore/planning/joint_space.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mc {

struct PlanningProblem {
  VecX start;
  std::vector<VecX> goals;  ///< reaching any goal solves the problem
};

struct PlannerSettings {
  std::uint64_t seed = 1;
  int max_iterations = 20000;  ///< samples drawn (RRT family) or roadmap growth budget (PRM)
  double time_limit = 0.0;     ///< seconds; 0 disables the wall-clock limit (keeps determinism)
  double max_step = 0.4;       ///< steering distance in the joint-space metric
  double goal_bias = 0.05;     ///< probability of sampling a goal (RRT, RRT*)
  double rewire_factor = 1.1;  ///< RRT*: multiplier on the asymptotic-optimality radius constant
  double max_rewire_radius = 1.0;
  int prm_neighbors = 10;  ///< PRM: k nearest neighbours per new roadmap node
  int prm_batch = 200;     ///< PRM: samples added per growth step
};

enum class PlannerStatus { Success, IterationLimit, TimeLimit, InvalidStart, InvalidGoal };

[[nodiscard]] const char* toString(PlannerStatus status);

struct PlannerStatistics {
  int iterations = 0;
  std::size_t nodes = 0;
  std::size_t edges = 0;               ///< PRM roadmap edges
  std::uint64_t collision_checks = 0;  ///< state-validity queries
  std::uint64_t edge_checks = 0;
  double planning_time = 0.0;  ///< seconds (informational; not used for decisions)
};

struct PlannerResult {
  PlannerStatus status = PlannerStatus::IterationLimit;
  std::vector<VecX> path;  ///< start ... goal, every edge validated
  double path_length = 0.0;
  int goal_index = -1;  ///< which goal was reached
  PlannerStatistics stats;
  [[nodiscard]] bool success() const { return status == PlannerStatus::Success; }
};

/// Common interface of joint-space planners.
class MotionPlanner {
 public:
  MotionPlanner(std::shared_ptr<const MotionValidator> validator, PlannerSettings settings);
  MotionPlanner(const MotionPlanner&) = delete;
  MotionPlanner& operator=(const MotionPlanner&) = delete;
  MotionPlanner(MotionPlanner&&) = default;
  MotionPlanner& operator=(MotionPlanner&&) = default;
  virtual ~MotionPlanner() = default;

  [[nodiscard]] virtual std::string name() const = 0;
  /// Plans from problem.start to any of problem.goals.
  [[nodiscard]] virtual PlannerResult solve(const PlanningProblem& problem) = 0;

  [[nodiscard]] const PlannerSettings& settings() const { return settings_; }
  void setSettings(const PlannerSettings& settings);
  [[nodiscard]] const MotionValidator& validator() const { return *validator_; }

 protected:
  /// Validates the problem; returns InvalidStart/InvalidGoal results, or nullopt when valid.
  [[nodiscard]] bool checkProblem(const PlanningProblem& problem, PlannerResult& result) const;
  void finalize(PlannerResult& result, double elapsed_seconds) const;

  std::shared_ptr<const MotionValidator> validator_;
  PlannerSettings settings_;
};

class RRTPlanner final : public MotionPlanner {
 public:
  using MotionPlanner::MotionPlanner;
  [[nodiscard]] std::string name() const override { return "RRT"; }
  [[nodiscard]] PlannerResult solve(const PlanningProblem& problem) override;
};

class RRTConnectPlanner final : public MotionPlanner {
 public:
  using MotionPlanner::MotionPlanner;
  [[nodiscard]] std::string name() const override { return "RRT-Connect"; }
  [[nodiscard]] PlannerResult solve(const PlanningProblem& problem) override;
};

/// RRT* with the joint-space path length as cost. Runs the full iteration budget (anytime) and
/// returns the best path found.
class RRTStarPlanner final : public MotionPlanner {
 public:
  using MotionPlanner::MotionPlanner;
  [[nodiscard]] std::string name() const override { return "RRT*"; }
  [[nodiscard]] PlannerResult solve(const PlanningProblem& problem) override;
};

/// k-nearest probabilistic roadmap. The roadmap is kept between solve() calls (multi-query); it
/// grows in batches until the query succeeds or the budget is exhausted. Queries use A*.
class PRMPlanner final : public MotionPlanner {
 public:
  PRMPlanner(std::shared_ptr<const MotionValidator> validator, PlannerSettings settings);
  [[nodiscard]] std::string name() const override { return "PRM"; }
  [[nodiscard]] PlannerResult solve(const PlanningProblem& problem) override;

  /// Adds up to @p samples valid configurations to the roadmap.
  void growRoadmap(int samples);
  [[nodiscard]] std::size_t roadmapSize() const { return nodes_.size(); }
  [[nodiscard]] std::size_t roadmapEdges() const;
  void clearRoadmap();

 private:
  int addNode(const VecX& q, bool connect);
  [[nodiscard]] std::vector<int> nearestNodes(const VecX& q, int k, int exclude) const;
  [[nodiscard]] std::vector<int> shortestPath(int from, const std::vector<int>& targets) const;

  Random rng_;
  std::vector<VecX> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adjacency_;
};

/// Factory by name: "RRT", "RRT-Connect", "RRT*", "PRM".
[[nodiscard]] std::unique_ptr<MotionPlanner> makePlanner(const std::string& name,
                                                         std::shared_ptr<const MotionValidator> validator,
                                                         const PlannerSettings& settings);

}  // namespace mc
