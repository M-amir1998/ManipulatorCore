#include "manipulatorcore/planning/planner.hpp"

#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/planning/path_processing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <utility>

namespace mc {

const char* toString(PlannerStatus status) {
  switch (status) {
    case PlannerStatus::Success:
      return "success";
    case PlannerStatus::IterationLimit:
      return "iteration-limit";
    case PlannerStatus::TimeLimit:
      return "time-limit";
    case PlannerStatus::InvalidStart:
      return "invalid-start";
    case PlannerStatus::InvalidGoal:
      return "invalid-goal";
  }
  return "unknown";
}

namespace {

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point t0) { return std::chrono::duration<double>(Clock::now() - t0).count(); }

/// Exploration tree with parent pointers, path costs and child lists (for RRT* cost updates).
struct Tree {
  std::vector<VecX> q;
  std::vector<int> parent;
  std::vector<double> cost;
  std::vector<std::vector<int>> children;
  std::vector<int> root_goal;  // for goal trees: which goal a node descends from

  int add(VecX x, int p, double c, int goal = -1) {
    q.push_back(std::move(x));
    parent.push_back(p);
    cost.push_back(c);
    children.emplace_back();
    root_goal.push_back(p >= 0 ? root_goal[static_cast<std::size_t>(p)] : goal);
    if (p >= 0) {
      children[static_cast<std::size_t>(p)].push_back(static_cast<int>(q.size()) - 1);
    }
    return static_cast<int>(q.size()) - 1;
  }

  [[nodiscard]] int nearest(const JointSpace& space, const VecX& x) const {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < q.size(); ++i) {
      const double d = space.distance(q[i], x);
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  /// Path from the root to node i.
  [[nodiscard]] std::vector<VecX> pathTo(int i) const {
    std::vector<VecX> path;
    for (int k = i; k >= 0; k = parent[static_cast<std::size_t>(k)]) {
      path.push_back(q[static_cast<std::size_t>(k)]);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }
};

void recordTree(PlannerTree& out, const Tree& t, int index) {
  const auto offset = static_cast<int>(out.nodes.size());
  for (std::size_t i = 0; i < t.q.size(); ++i) {
    out.nodes.push_back(t.q[i]);
    out.parents.push_back(t.parent[i] < 0 ? -1 : t.parent[i] + offset);
    out.tree_index.push_back(index);
  }
}

bool timeExceeded(const PlannerSettings& s, Clock::time_point t0) {
  return s.time_limit > 0.0 && secondsSince(t0) > s.time_limit;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// MotionPlanner base
// ------------------------------------------------------------------------------------------------

MotionPlanner::MotionPlanner(std::shared_ptr<const MotionValidator> validator, PlannerSettings settings)
    : validator_(std::move(validator)) {
  require(validator_ != nullptr, "MotionPlanner: validator must not be null");
  setSettings(settings);
}

void MotionPlanner::setSettings(const PlannerSettings& s) {
  require(s.max_iterations > 0, "PlannerSettings: max_iterations must be positive");
  require(s.time_limit >= 0.0, "PlannerSettings: time_limit must be non-negative");
  require(s.max_step > 0.0 && std::isfinite(s.max_step), "PlannerSettings: max_step must be positive");
  require(s.goal_bias >= 0.0 && s.goal_bias <= 1.0, "PlannerSettings: goal_bias must be in [0, 1]");
  require(s.rewire_factor > 0.0 && s.max_rewire_radius > 0.0, "PlannerSettings: invalid RRT* parameters");
  require(s.prm_neighbors > 0 && s.prm_batch > 0, "PlannerSettings: invalid PRM parameters");
  settings_ = s;
}

bool MotionPlanner::checkProblem(const PlanningProblem& problem, PlannerResult& result) const {
  validator_->resetCounters();
  const int n = validator_->space().dimension();
  requireVector(problem.start, n, "PlanningProblem: start");
  require(!problem.goals.empty(), "PlanningProblem: at least one goal is required");
  for (const VecX& g : problem.goals) {
    requireVector(g, n, "PlanningProblem: goal");
  }
  if (!validator_->isValid(problem.start)) {
    result.status = PlannerStatus::InvalidStart;
    return false;
  }
  const bool any_goal_valid =
      std::any_of(problem.goals.begin(), problem.goals.end(), [&](const VecX& g) { return validator_->isValid(g); });
  if (!any_goal_valid) {
    result.status = PlannerStatus::InvalidGoal;
    return false;
  }
  return true;
}

void MotionPlanner::finalize(PlannerResult& result, double elapsed_seconds) const {
  result.stats.collision_checks = validator_->stateChecks();
  result.stats.edge_checks = validator_->edgeChecks();
  result.stats.planning_time = elapsed_seconds;
  result.path_length = result.path.empty() ? 0.0 : pathLength(validator_->space(), result.path);
}

// ------------------------------------------------------------------------------------------------
// RRT
// ------------------------------------------------------------------------------------------------

PlannerResult RRTPlanner::solve(const PlanningProblem& problem) {
  const auto t0 = Clock::now();
  PlannerResult result;
  if (!checkProblem(problem, result)) {
    finalize(result, secondsSince(t0));
    return result;
  }
  const JointSpace& space = validator_->space();
  Random rng(settings_.seed);
  Tree tree;
  tree.add(problem.start, -1, 0.0);
  result.status = PlannerStatus::IterationLimit;
  for (int it = 0; it < settings_.max_iterations; ++it) {
    result.stats.iterations = it + 1;
    if (timeExceeded(settings_, t0)) {
      result.status = PlannerStatus::TimeLimit;
      break;
    }
    const VecX target = rng.bernoulli(settings_.goal_bias) ? problem.goals[rng.uniformIndex(problem.goals.size())]
                                                           : space.sampleUniform(rng);
    const int near = tree.nearest(space, target);
    const VecX q_new = space.steer(tree.q[static_cast<std::size_t>(near)], target, settings_.max_step);
    if (!validator_->isEdgeValid(tree.q[static_cast<std::size_t>(near)], q_new)) {
      continue;
    }
    const int id = tree.add(q_new, near, 0.0);
    for (std::size_t g = 0; g < problem.goals.size(); ++g) {
      if (space.distance(q_new, problem.goals[g]) <= settings_.max_step &&
          validator_->isEdgeValid(q_new, problem.goals[g])) {
        result.path = tree.pathTo(tree.add(problem.goals[g], id, 0.0));
        result.goal_index = static_cast<int>(g);
        result.status = PlannerStatus::Success;
        break;
      }
    }
    if (result.success()) {
      break;
    }
  }
  result.stats.nodes = tree.q.size();
  if (settings_.record_tree) {
    recordTree(result.tree, tree, 0);
  }
  finalize(result, secondsSince(t0));
  return result;
}

// ------------------------------------------------------------------------------------------------
// RRT-Connect
// ------------------------------------------------------------------------------------------------

PlannerResult RRTConnectPlanner::solve(const PlanningProblem& problem) {
  const auto t0 = Clock::now();
  PlannerResult result;
  if (!checkProblem(problem, result)) {
    finalize(result, secondsSince(t0));
    return result;
  }
  const JointSpace& space = validator_->space();
  Random rng(settings_.seed);
  Tree start_tree;
  Tree goal_tree;
  start_tree.add(problem.start, -1, 0.0);
  for (std::size_t g = 0; g < problem.goals.size(); ++g) {
    if (validator_->isValid(problem.goals[g])) {
      goal_tree.add(problem.goals[g], -1, 0.0, static_cast<int>(g));
    }
  }
  enum class Extend { Trapped, Advanced, Reached };
  const auto extend = [&](Tree& tree, const VecX& target, int& new_id) {
    const int near = tree.nearest(space, target);
    const VecX q_new = space.steer(tree.q[static_cast<std::size_t>(near)], target, settings_.max_step);
    if (!validator_->isEdgeValid(tree.q[static_cast<std::size_t>(near)], q_new)) {
      return Extend::Trapped;
    }
    new_id = tree.add(q_new, near, 0.0);
    return space.distance(q_new, target) <= 1e-12 ? Extend::Reached : Extend::Advanced;
  };

  Tree* a = &start_tree;
  Tree* b = &goal_tree;
  result.status = PlannerStatus::IterationLimit;
  for (int it = 0; it < settings_.max_iterations; ++it) {
    result.stats.iterations = it + 1;
    if (timeExceeded(settings_, t0)) {
      result.status = PlannerStatus::TimeLimit;
      break;
    }
    const VecX q_rand = space.sampleUniform(rng);
    int id_a = -1;
    if (extend(*a, q_rand, id_a) != Extend::Trapped) {
      const VecX target = a->q[static_cast<std::size_t>(id_a)];
      int id_b = -1;
      Extend status = Extend::Advanced;
      while (status == Extend::Advanced) {  // CONNECT: greedy extension towards the new node
        status = extend(*b, target, id_b);
      }
      if (status == Extend::Reached) {
        const bool a_is_start = a == &start_tree;
        std::vector<VecX> from_start = (a_is_start ? *a : *b).pathTo(a_is_start ? id_a : id_b);
        std::vector<VecX> from_goal = (a_is_start ? *b : *a).pathTo(a_is_start ? id_b : id_a);
        result.goal_index = (a_is_start ? *b : *a).root_goal[static_cast<std::size_t>(a_is_start ? id_b : id_a)];
        // Both halves end at the meeting configuration; drop the duplicate.
        from_goal.pop_back();
        std::reverse(from_goal.begin(), from_goal.end());
        from_start.insert(from_start.end(), from_goal.begin(), from_goal.end());
        result.path = std::move(from_start);
        result.status = PlannerStatus::Success;
        break;
      }
    }
    std::swap(a, b);
  }
  result.stats.nodes = start_tree.q.size() + goal_tree.q.size();
  if (settings_.record_tree) {
    recordTree(result.tree, start_tree, 0);
    recordTree(result.tree, goal_tree, 1);
  }
  finalize(result, secondsSince(t0));
  return result;
}

// ------------------------------------------------------------------------------------------------
// RRT*
// ------------------------------------------------------------------------------------------------

PlannerResult RRTStarPlanner::solve(const PlanningProblem& problem) {
  const auto t0 = Clock::now();
  PlannerResult result;
  if (!checkProblem(problem, result)) {
    finalize(result, secondsSince(t0));
    return result;
  }
  const JointSpace& space = validator_->space();
  const double d = space.dimension();
  // gamma > 2 (1 + 1/d)^(1/d) (mu(X) / zeta_d)^(1/d) guarantees asymptotic optimality.
  const double unit_ball = std::pow(kPi, 0.5 * d) / std::tgamma(0.5 * d + 1.0);
  const double gamma =
      settings_.rewire_factor * 2.0 * std::pow(1.0 + 1.0 / d, 1.0 / d) * std::pow(space.measure() / unit_ball, 1.0 / d);

  Random rng(settings_.seed);
  Tree tree;
  tree.add(problem.start, -1, 0.0);
  struct GoalLink {
    int node;
    int goal;
  };
  std::vector<GoalLink> goal_links;

  const auto propagate = [&](int root, double delta) {
    std::vector<int> stack{root};
    while (!stack.empty()) {
      const int k = stack.back();
      stack.pop_back();
      tree.cost[static_cast<std::size_t>(k)] += delta;
      for (const int c : tree.children[static_cast<std::size_t>(k)]) {
        stack.push_back(c);
      }
    }
  };

  result.status = PlannerStatus::IterationLimit;
  for (int it = 0; it < settings_.max_iterations; ++it) {
    result.stats.iterations = it + 1;
    if (timeExceeded(settings_, t0)) {
      break;
    }
    const VecX target = rng.bernoulli(settings_.goal_bias) ? problem.goals[rng.uniformIndex(problem.goals.size())]
                                                           : space.sampleUniform(rng);
    const int nearest = tree.nearest(space, target);
    const VecX q_new = space.steer(tree.q[static_cast<std::size_t>(nearest)], target, settings_.max_step);
    if (!validator_->isValid(q_new)) {
      continue;
    }
    const double n = static_cast<double>(tree.q.size()) + 1.0;
    const double radius = std::min(gamma * std::pow(std::log(n) / n, 1.0 / d), settings_.max_rewire_radius);
    std::vector<int> near;
    for (std::size_t i = 0; i < tree.q.size(); ++i) {
      if (space.distance(tree.q[i], q_new) <= radius) {
        near.push_back(static_cast<int>(i));
      }
    }
    if (std::find(near.begin(), near.end(), nearest) == near.end()) {
      near.push_back(nearest);
    }
    // Choose the parent minimizing cost-to-come; edges are validated lazily in cost order.
    std::vector<std::pair<double, int>> candidates;
    candidates.reserve(near.size());
    for (const int i : near) {
      candidates.emplace_back(
          tree.cost[static_cast<std::size_t>(i)] + space.distance(tree.q[static_cast<std::size_t>(i)], q_new), i);
    }
    std::sort(candidates.begin(), candidates.end());
    int parent = -1;
    double new_cost = 0.0;
    for (const auto& [c, i] : candidates) {
      if (validator_->isEdgeValid(tree.q[static_cast<std::size_t>(i)], q_new)) {
        parent = i;
        new_cost = c;
        break;
      }
    }
    if (parent < 0) {
      continue;
    }
    const int id = tree.add(q_new, parent, new_cost);
    // Rewire neighbours through the new node when that shortens their path.
    for (const int i : near) {
      if (i == parent) {
        continue;
      }
      const auto ui = static_cast<std::size_t>(i);
      const double via_new = new_cost + space.distance(q_new, tree.q[ui]);
      if (via_new < tree.cost[ui] - 1e-12 && validator_->isEdgeValid(q_new, tree.q[ui])) {
        auto& siblings = tree.children[static_cast<std::size_t>(tree.parent[ui])];
        siblings.erase(std::remove(siblings.begin(), siblings.end(), i), siblings.end());
        tree.parent[ui] = id;
        tree.children[static_cast<std::size_t>(id)].push_back(i);
        propagate(i, via_new - tree.cost[ui]);
      }
    }
    for (std::size_t g = 0; g < problem.goals.size(); ++g) {
      if (space.distance(q_new, problem.goals[g]) <= settings_.max_step &&
          validator_->isEdgeValid(q_new, problem.goals[g])) {
        goal_links.push_back({id, static_cast<int>(g)});
      }
    }
  }
  // Best goal connection under the final (rewired) costs.
  double best_cost = std::numeric_limits<double>::infinity();
  for (const GoalLink& link : goal_links) {
    const double c =
        tree.cost[static_cast<std::size_t>(link.node)] +
        space.distance(tree.q[static_cast<std::size_t>(link.node)], problem.goals[static_cast<std::size_t>(link.goal)]);
    if (c < best_cost) {
      best_cost = c;
      result.path = tree.pathTo(link.node);
      result.path.push_back(problem.goals[static_cast<std::size_t>(link.goal)]);
      result.goal_index = link.goal;
    }
  }
  if (!result.path.empty()) {
    result.status = PlannerStatus::Success;
  } else if (timeExceeded(settings_, t0)) {
    result.status = PlannerStatus::TimeLimit;
  }
  result.stats.nodes = tree.q.size();
  if (settings_.record_tree) {
    recordTree(result.tree, tree, 0);
  }
  finalize(result, secondsSince(t0));
  return result;
}

// ------------------------------------------------------------------------------------------------
// PRM
// ------------------------------------------------------------------------------------------------

PRMPlanner::PRMPlanner(std::shared_ptr<const MotionValidator> validator, PlannerSettings settings)
    : MotionPlanner(std::move(validator), settings), rng_(settings.seed) {}

void PRMPlanner::clearRoadmap() {
  nodes_.clear();
  adjacency_.clear();
  rng_.seed(settings_.seed);
}

std::size_t PRMPlanner::roadmapEdges() const {
  std::size_t total = 0;
  for (const auto& a : adjacency_) {
    total += a.size();
  }
  return total / 2;
}

std::vector<int> PRMPlanner::nearestNodes(const VecX& q, int k, int exclude) const {
  std::vector<std::pair<double, int>> d;
  d.reserve(nodes_.size());
  const JointSpace& space = validator_->space();
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (std::cmp_not_equal(i, exclude)) {
      d.emplace_back(space.distance(nodes_[i], q), static_cast<int>(i));
    }
  }
  const auto kk = std::min<std::size_t>(static_cast<std::size_t>(k), d.size());
  std::partial_sort(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(kk), d.end());
  std::vector<int> out;
  out.reserve(kk);
  for (std::size_t i = 0; i < kk; ++i) {
    out.push_back(d[i].second);
  }
  return out;
}

int PRMPlanner::addNode(const VecX& q, bool connect) {
  nodes_.push_back(q);
  adjacency_.emplace_back();
  const int id = static_cast<int>(nodes_.size()) - 1;
  if (connect) {
    for (const int nb : nearestNodes(q, settings_.prm_neighbors, id)) {
      if (validator_->isEdgeValid(nodes_[static_cast<std::size_t>(nb)], q)) {
        const double w = validator_->space().distance(nodes_[static_cast<std::size_t>(nb)], q);
        adjacency_[static_cast<std::size_t>(id)].emplace_back(nb, w);
        adjacency_[static_cast<std::size_t>(nb)].emplace_back(id, w);
      }
    }
  }
  return id;
}

void PRMPlanner::growRoadmap(int samples) {
  for (int s = 0; s < samples; ++s) {
    const VecX q = validator_->space().sampleUniform(rng_);
    if (validator_->isValid(q)) {
      addNode(q, true);
    }
  }
}

std::vector<int> PRMPlanner::shortestPath(int from, const std::vector<int>& targets) const {
  const JointSpace& space = validator_->space();
  const auto heuristic = [&](int i) {
    double h = std::numeric_limits<double>::infinity();
    for (const int t : targets) {
      h = std::min(h, space.distance(nodes_[static_cast<std::size_t>(i)], nodes_[static_cast<std::size_t>(t)]));
    }
    return h;
  };
  const std::size_t n = nodes_.size();
  std::vector<double> g(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);
  std::vector<bool> closed(n, false);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
  g[static_cast<std::size_t>(from)] = 0.0;
  open.emplace(heuristic(from), from);
  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    const auto uu = static_cast<std::size_t>(u);
    if (closed[uu]) {
      continue;
    }
    closed[uu] = true;
    if (std::find(targets.begin(), targets.end(), u) != targets.end()) {
      std::vector<int> path;
      for (int k = u; k >= 0; k = prev[static_cast<std::size_t>(k)]) {
        path.push_back(k);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    for (const auto& [v, w] : adjacency_[uu]) {
      const auto vv = static_cast<std::size_t>(v);
      if (g[uu] + w < g[vv]) {
        g[vv] = g[uu] + w;
        prev[vv] = u;
        open.emplace(g[vv] + heuristic(v), v);
      }
    }
  }
  return {};
}

PlannerResult PRMPlanner::solve(const PlanningProblem& problem) {
  const auto t0 = Clock::now();
  PlannerResult result;
  if (!checkProblem(problem, result)) {
    finalize(result, secondsSince(t0));
    return result;
  }
  const int start = addNode(problem.start, true);
  std::vector<int> goal_ids;
  std::vector<int> goal_index_of;
  for (std::size_t g = 0; g < problem.goals.size(); ++g) {
    if (validator_->isValid(problem.goals[g])) {
      goal_ids.push_back(addNode(problem.goals[g], true));
      goal_index_of.push_back(static_cast<int>(g));
    }
  }
  result.status = PlannerStatus::IterationLimit;
  int drawn = 0;
  for (;;) {
    const std::vector<int> ids = shortestPath(start, goal_ids);
    if (!ids.empty()) {
      for (const int id : ids) {
        result.path.push_back(nodes_[static_cast<std::size_t>(id)]);
      }
      const auto it = std::find(goal_ids.begin(), goal_ids.end(), ids.back());
      result.goal_index = goal_index_of[static_cast<std::size_t>(it - goal_ids.begin())];
      result.status = PlannerStatus::Success;
      break;
    }
    if (drawn >= settings_.max_iterations) {
      break;
    }
    if (timeExceeded(settings_, t0)) {
      result.status = PlannerStatus::TimeLimit;
      break;
    }
    const int batch = std::min(settings_.prm_batch, settings_.max_iterations - drawn);
    growRoadmap(batch);
    drawn += batch;
  }
  result.stats.iterations = drawn;
  result.stats.nodes = nodes_.size();
  if (settings_.record_tree) {
    result.tree.nodes = nodes_;
    result.tree.parents.assign(nodes_.size(), -1);
    result.tree.tree_index.assign(nodes_.size(), 0);
    for (std::size_t i = 0; i < adjacency_.size(); ++i) {
      for (const auto& [j, w] : adjacency_[i]) {
        if (std::cmp_less(i, j)) {
          result.tree.edges.emplace_back(static_cast<int>(i), j);
        }
      }
    }
  }
  result.stats.edges = roadmapEdges();
  finalize(result, secondsSince(t0));
  return result;
}

std::unique_ptr<MotionPlanner> makePlanner(const std::string& name, std::shared_ptr<const MotionValidator> validator,
                                           const PlannerSettings& settings) {
  if (name == "RRT") {
    return std::make_unique<RRTPlanner>(std::move(validator), settings);
  }
  if (name == "RRT-Connect") {
    return std::make_unique<RRTConnectPlanner>(std::move(validator), settings);
  }
  if (name == "RRT*") {
    return std::make_unique<RRTStarPlanner>(std::move(validator), settings);
  }
  if (name == "PRM") {
    return std::make_unique<PRMPlanner>(std::move(validator), settings);
  }
  throw InvalidArgument("makePlanner: unknown planner '" + name + "'");
}

}  // namespace mc
