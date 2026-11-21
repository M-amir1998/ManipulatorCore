#include "manipulatorcore/kinematics/singularity.hpp"

#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/linalg.hpp"

#include <limits>

namespace mc {

double JacobianAnalysis::inverseConditionNumber() const { return sigma_max > 0.0 ? sigma_min / sigma_max : 0.0; }

bool JacobianAnalysis::isSingular() const { return rank < singular_values.size(); }

JacobianAnalysis analyzeJacobian(const MatX& J, double tolerance) {
  const SvdMetrics svd = computeSvdMetrics(J, tolerance);
  JacobianAnalysis out;
  out.singular_values = svd.singular_values;
  out.rank = svd.rank;
  out.tolerance = svd.tolerance;
  out.sigma_min = svd.sigma_min;
  out.sigma_max = svd.sigma_max;
  out.condition_number = svd.condition_number;
  out.manipulability = svd.singular_values.size() > 0 ? svd.singular_values.prod() : 0.0;
  return out;
}

double yoshikawaManipulability(const MatX& J) {
  if (J.size() == 0) {
    return 0.0;
  }
  return computeSvdMetrics(J).singular_values.prod();
}

SingularityReport analyzeSingularity(const RobotModel& model, const VecX& q) {
  const Mat6X J = geometricJacobian(model, q);
  SingularityReport report;
  report.full = analyzeJacobian(J);
  report.translational = analyzeJacobian(J.bottomRows<3>());
  report.rotational = analyzeJacobian(J.topRows<3>());
  return report;
}

}  // namespace mc
