#include "manipulatorcore/kinematics/inverse_kinematics.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/kinematics/differential_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <utility>

namespace mc {

const char* toString(IKMethod method) {
  switch (method) {
    case IKMethod::JacobianTranspose:
      return "jacobian-transpose";
    case IKMethod::PseudoInverse:
      return "pseudoinverse";
    case IKMethod::DampedLeastSquares:
      return "damped-least-squares";
    case IKMethod::LevenbergMarquardt:
      return "levenberg-marquardt";
  }
  return "unknown";
}

const char* toString(IKStatus status) {
  switch (status) {
    case IKStatus::Converged:
      return "converged";
    case IKStatus::MaxIterations:
      return "max-iterations";
    case IKStatus::Stalled:
      return "stalled";
  }
  return "unknown";
}

Vec6 poseErrorWorld(const SE3& current, const SE3& desired) {
  Vec6 e;
  e.head<3>() = logSO3(desired.rotation() * current.rotation().transpose());
  e.tail<3>() = desired.translation() - current.translation();
  return e;
}

Vec6 poseErrorBody(const SE3& current, const SE3& desired) { return logSE3(current.inverse() * desired); }

namespace {

void validateOptions(const IKOptions& o) {
  require(o.max_iterations >= 0, "IKOptions: max_iterations must be non-negative");
  require(o.position_tolerance > 0.0 && o.orientation_tolerance > 0.0, "IKOptions: tolerances must be positive");
  require(o.step_size > 0.0 && std::isfinite(o.step_size), "IKOptions: step_size must be positive");
  require(o.max_step > 0.0 && std::isfinite(o.max_step), "IKOptions: max_step must be positive");
  require(o.damping >= 0.0 && std::isfinite(o.damping), "IKOptions: damping must be non-negative");
  require(o.singular_region > 0.0, "IKOptions: singular_region must be positive");
  require(o.lm_bias >= 0.0, "IKOptions: lm_bias must be non-negative");
  require(o.task_weights.allFinite() && (o.task_weights.array() >= 0.0).all() && o.task_weights.maxCoeff() > 0.0,
          "IKOptions: task weights must be non-negative with at least one positive entry");
  require(o.null_space_gain >= 0.0, "IKOptions: null_space_gain must be non-negative");
}

/// Unweighted step dq for the selected method. Locked columns of J must already be zero.
VecX solveStep(const IKOptions& o, const MatX& J, const VecX& e) {
  switch (o.method) {
    case IKMethod::JacobianTranspose: {
      const VecX g = J.transpose() * e;
      const VecX Jg = J * g;
      const double denom = Jg.squaredNorm();
      // Buss' step length minimizes |e - alpha J J^T e| along the transpose direction.
      const double alpha = denom > 1e-300 ? e.dot(Jg) / denom : 0.0;
      return alpha * g;
    }
    case IKMethod::PseudoInverse:
      return solveMinimumNorm(J, e, o.pinv_tolerance);
    case IKMethod::DampedLeastSquares: {
      double lambda = o.damping;
      if (o.adaptive_damping) {
        lambda = adaptiveDamping(computeSvdMetrics(J).sigma_min, o.singular_region, o.damping);
      }
      return lambda > 0.0 ? solveDampedLeastSquares(J, e, lambda) : solveMinimumNorm(J, e, o.pinv_tolerance);
    }
    case IKMethod::LevenbergMarquardt: {
      const double mu = 0.5 * e.squaredNorm() + o.lm_bias;
      MatX H = J.transpose() * J;
      H.diagonal().array() += std::max(mu, 1e-12);
      return H.llt().solve(J.transpose() * e);
    }
  }
  throw InvalidArgument("solveIK: unknown method");
}

struct ErrorTerms {
  VecX weighted;             // selected, weighted error rows
  double position = 0.0;     // over selected translational rows
  double orientation = 0.0;  // over selected rotational rows
};

}  // namespace

IKResult solveIK(const RobotModel& model, const SE3& target, const VecX& seed, const IKOptions& options) {
  validateOptions(options);
  model.checkConfiguration(seed, "solveIK: seed");
  requireFinite(target.rotation(), "solveIK: target rotation");
  requireFinite(target.translation(), "solveIK: target translation");

  const int n = model.dof();
  std::vector<int> rows;
  for (int r = 0; r < 6; ++r) {
    if (options.task_weights(r) > 0.0) {
      rows.push_back(r);
    }
  }
  const auto m = static_cast<Index>(rows.size());
  VecX sqrt_w(m);
  for (Index k = 0; k < m; ++k) {
    sqrt_w(k) = std::sqrt(options.task_weights(rows[static_cast<std::size_t>(k)]));
  }

  VecX q = options.enforce_joint_limits ? model.clampToLimits(seed) : seed;
  KinematicState state;

  auto evaluate = [&](const VecX& config) {
    computeForwardKinematics(model, config, state);
    const Vec6 e6 = poseErrorWorld(state.T_base_ee, target);
    ErrorTerms terms;
    terms.weighted.resize(m);
    double pos_sq = 0.0;
    double ori_sq = 0.0;
    for (Index k = 0; k < m; ++k) {
      const int r = rows[static_cast<std::size_t>(k)];
      terms.weighted(k) = sqrt_w(k) * e6(r);
      (r < 3 ? ori_sq : pos_sq) += e6(r) * e6(r);
    }
    terms.position = std::sqrt(pos_sq);
    terms.orientation = std::sqrt(ori_sq);
    return terms;
  };
  auto converged = [&](const ErrorTerms& t) {
    return t.position <= options.position_tolerance && t.orientation <= options.orientation_tolerance;
  };

  IKResult result;
  result.status = IKStatus::MaxIterations;
  ErrorTerms err = evaluate(q);
  const bool has_objective = options.null_space_objective != NullSpaceObjective::None && options.null_space_gain > 0.0;
  int refinement_left = has_objective ? options.null_space_refinement_iterations : 0;
  int iteration = 0;
  for (; iteration < options.max_iterations; ++iteration) {
    if (converged(err)) {
      if (refinement_left <= 0) {
        result.status = IKStatus::Converged;
        break;
      }
      --refinement_left;
    }
    const Mat6X J6 = geometricJacobian(model, state);
    MatX J(m, n);
    for (Index k = 0; k < m; ++k) {
      J.row(k) = sqrt_w(k) * J6.row(rows[static_cast<std::size_t>(k)]);
    }

    // Secondary objective gradient (projected into the task null space below).
    VecX z = VecX::Zero(n);
    if (has_objective) {
      if (options.null_space_objective == NullSpaceObjective::JointCentering) {
        z = -options.null_space_gain * jointCenteringObjective(model, q, options.null_space_center).gradient;
      } else {
        z = -options.null_space_gain * evaluateObjective(options.null_space_objective, model, q).gradient;
      }
    }

    // Active-set clamping: joints that would leave their limits are fixed at the limit and the
    // step is recomputed for the remaining joints (Baerlocher & Boulic).
    std::vector<bool> locked(static_cast<std::size_t>(n), false);
    VecX locked_motion = VecX::Zero(n);
    VecX dq = VecX::Zero(n);
    for (int pass = 0; pass <= n; ++pass) {
      MatX J_free = J;
      for (int i = 0; i < n; ++i) {
        if (locked[static_cast<std::size_t>(i)]) {
          J_free.col(i).setZero();
        }
      }
      const VecX e_free = err.weighted - J * locked_motion;
      dq = solveStep(options, J_free, e_free);
      if (z.squaredNorm() > 0.0) {
        VecX z_free = z;
        for (int i = 0; i < n; ++i) {
          if (locked[static_cast<std::size_t>(i)]) {
            z_free(i) = 0.0;
          }
        }
        dq += nullSpaceProjector(J_free) * z_free;
      }
      dq *= options.step_size;
      const double peak = dq.cwiseAbs().maxCoeff();
      if (peak > options.max_step) {
        dq *= options.max_step / peak;
      }
      for (int i = 0; i < n; ++i) {
        if (locked[static_cast<std::size_t>(i)]) {
          dq(i) = locked_motion(i);
        }
      }
      if (!options.enforce_joint_limits) {
        break;
      }
      bool new_lock = false;
      for (int i = 0; i < n; ++i) {
        if (locked[static_cast<std::size_t>(i)]) {
          continue;
        }
        const double next = q(i) + dq(i);
        if (next < model.lowerLimits()(i) || next > model.upperLimits()(i)) {
          locked[static_cast<std::size_t>(i)] = true;
          locked_motion(i) = std::clamp(next, model.lowerLimits()(i), model.upperLimits()(i)) - q(i);
          new_lock = true;
        }
      }
      if (!new_lock) {
        break;
      }
    }

    VecX q_next = q + dq;
    if (options.enforce_joint_limits) {
      q_next = model.clampToLimits(q_next);
    }
    const double moved = (q_next - q).cwiseAbs().maxCoeff();
    q = q_next;
    err = evaluate(q);
    if (moved < options.stall_tolerance) {
      result.status = converged(err) ? IKStatus::Converged : IKStatus::Stalled;
      ++iteration;
      break;
    }
  }
  if (result.status == IKStatus::MaxIterations && converged(err)) {
    result.status = IKStatus::Converged;
  }
  result.q = q;
  result.iterations = iteration;
  result.position_error = err.position;
  result.orientation_error = err.orientation;
  result.within_limits = model.withinLimits(q, 1e-12);
  return result;
}

std::vector<IKResult> solveIKMultiStart(const RobotModel& model, const SE3& target, const VecX& seed,
                                        const MultiStartIKOptions& options) {
  require(options.attempts > 0 && options.max_solutions > 0,
          "MultiStartIKOptions: attempts and max_solutions must be positive");
  model.checkConfiguration(seed, "solveIKMultiStart: seed");
  Random rng(options.seed);
  std::vector<IKResult> solutions;
  for (int attempt = 0; attempt < options.attempts; ++attempt) {
    const VecX start = (attempt == 0 && options.use_initial_guess)
                           ? seed
                           : rng.uniformVector(model.lowerLimits(), model.upperLimits());
    IKResult r = solveIK(model, target, start, options.ik);
    if (!r.converged() || !r.within_limits) {
      continue;
    }
    if (options.validator && !options.validator(r.q)) {
      continue;
    }
    const bool duplicate = std::any_of(solutions.begin(), solutions.end(), [&](const IKResult& s) {
      return (s.q - r.q).cwiseAbs().maxCoeff() < options.distinct_threshold;
    });
    if (duplicate) {
      continue;
    }
    solutions.push_back(std::move(r));
    if (std::cmp_greater_equal(solutions.size(), options.max_solutions)) {
      break;
    }
  }
  std::stable_sort(solutions.begin(), solutions.end(),
                   [&](const IKResult& a, const IKResult& b) { return (a.q - seed).norm() < (b.q - seed).norm(); });
  return solutions;
}

}  // namespace mc
