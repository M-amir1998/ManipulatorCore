#pragma once

/// @file singularity.hpp
/// @brief Singularity and manipulability analysis from the SVD of a Jacobian.
///
/// Yoshikawa's manipulability w = sqrt(det(J J^T)) = prod(sigma_i) measures the volume of the
/// velocity ellipsoid. It is zero at singularities, but its magnitude depends on units and on the
/// number of rows, so values of different robots, or of mixed angular/linear Jacobians, are not
/// directly comparable. The condition number sigma_max / sigma_min is scale-invariant for a fixed
/// Jacobian and measures isotropy. Reference: T. Yoshikawa, "Manipulability of robotic
/// mechanisms", IJRR 4(2), 1985.

#include "manipulatorcore/kinematics/forward_kinematics.hpp"

namespace mc {

struct JacobianAnalysis {
  VecX singular_values;    ///< descending
  int rank = 0;            ///< numerical rank at the given tolerance
  double tolerance = 0.0;  ///< singular-value threshold used for the rank decision
  double sigma_min = 0.0;  ///< smallest of the min(rows, cols) singular values
  double sigma_max = 0.0;
  double condition_number = 0.0;  ///< sigma_max / sigma_min; +infinity at a singularity
  double manipulability = 0.0;    ///< Yoshikawa: product of the min(rows, cols) singular values

  /// sigma_min / sigma_max in [0, 1]; 0 at singularities, 1 for an isotropic Jacobian.
  [[nodiscard]] double inverseConditionNumber() const;
  [[nodiscard]] bool isSingular() const;
};

/// Analyzes an arbitrary task Jacobian. A negative tolerance selects the default rank tolerance.
[[nodiscard]] JacobianAnalysis analyzeJacobian(const MatX& J, double tolerance = -1.0);

/// sqrt(det(J J^T)) evaluated as the product of singular values (rows <= cols assumed).
[[nodiscard]] double yoshikawaManipulability(const MatX& J);

/// Analysis of the full, translational (rows 3-5) and rotational (rows 0-2) blocks of the
/// geometric Jacobian at q.
struct SingularityReport {
  JacobianAnalysis full;
  JacobianAnalysis translational;
  JacobianAnalysis rotational;
};

[[nodiscard]] SingularityReport analyzeSingularity(const RobotModel& model, const VecX& q);

}  // namespace mc
