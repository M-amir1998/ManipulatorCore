#include "manipulatorcore/kinematics/differential_kinematics.hpp"

#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"

#include <cmath>

namespace mc {

VecX taskVelocity(const MatX& J, const VecX& qd) {
  requireVector(qd, J.cols(), "taskVelocity: qd");
  return J * qd;
}

VecX jointVelocityForTask(const MatX& J, const VecX& task_velocity, DifferentialSolver solver, double damping) {
  requireVector(task_velocity, J.rows(), "jointVelocityForTask: task velocity");
  switch (solver) {
    case DifferentialSolver::Transpose:
      return J.transpose() * task_velocity;
    case DifferentialSolver::PseudoInverse:
      return solveMinimumNorm(J, task_velocity, damping);
    case DifferentialSolver::DampedLeastSquares:
      return solveDampedLeastSquares(J, task_velocity, damping);
  }
  throw InvalidArgument("jointVelocityForTask: unknown solver");
}

double adaptiveDamping(double sigma_min, double epsilon, double lambda_max) {
  require(epsilon > 0.0 && lambda_max >= 0.0, "adaptiveDamping: epsilon must be > 0 and lambda_max >= 0");
  if (sigma_min >= epsilon) {
    return 0.0;
  }
  const double ratio = sigma_min / epsilon;
  return lambda_max * std::sqrt(1.0 - ratio * ratio);
}

VecX jointVelocityWithNullSpace(const MatX& J, const VecX& task_velocity, const VecX& qd_null, double damping) {
  requireVector(task_velocity, J.rows(), "jointVelocityWithNullSpace: task velocity");
  requireVector(qd_null, J.cols(), "jointVelocityWithNullSpace: null-space velocity");
  const MatX J_sharp = damping > 0.0 ? dampedPseudoInverse(J, damping) : pseudoInverse(J);
  const MatX N = MatX::Identity(J.cols(), J.cols()) - J_sharp * J;
  return J_sharp * task_velocity + N * qd_null;
}

}  // namespace mc
