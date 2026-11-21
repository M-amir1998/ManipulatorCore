#include "manipulatorcore/optimization/objectives.hpp"

#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/math/error.hpp"

#include <algorithm>

namespace mc {

const char* toString(NullSpaceObjective objective) {
  switch (objective) {
    case NullSpaceObjective::None:
      return "none";
    case NullSpaceObjective::JointCentering:
      return "joint-centering";
    case NullSpaceObjective::JointLimitBarrier:
      return "joint-limit-barrier";
    case NullSpaceObjective::JointLimitActivation:
      return "joint-limit-activation";
    case NullSpaceObjective::Manipulability:
      return "manipulability";
  }
  return "unknown";
}

ObjectiveValue jointCenteringObjective(const RobotModel& model, const VecX& q, const VecX& center) {
  model.checkConfiguration(q, "jointCenteringObjective: q");
  const VecX c = center.size() == 0 ? model.midConfiguration() : center;
  requireVector(c, model.dof(), "jointCenteringObjective: center");
  ObjectiveValue out{0.0, VecX::Zero(model.dof())};
  for (int i = 0; i < model.dof(); ++i) {
    const double range = model.upperLimits()(i) - model.lowerLimits()(i);
    if (range <= 0.0) {
      continue;
    }
    const double r2 = range * range;
    const double d = q(i) - c(i);
    out.value += 0.5 * d * d / r2;
    out.gradient(i) = d / r2;
  }
  return out;
}

ObjectiveValue jointLimitBarrierObjective(const RobotModel& model, const VecX& q) {
  model.checkConfiguration(q, "jointLimitBarrierObjective: q");
  const int n = model.dof();
  ObjectiveValue out{0.0, VecX::Zero(n)};
  for (int i = 0; i < n; ++i) {
    const double l = model.lowerLimits()(i);
    const double u = model.upperLimits()(i);
    const double range = u - l;
    if (range <= 0.0) {
      continue;
    }
    // Keep the distances strictly positive so the barrier stays finite at the boundary.
    const double floor = 1e-9 * range;
    const double du = std::max(u - q(i), floor);
    const double dl = std::max(q(i) - l, floor);
    const double r2 = range * range;
    out.value += r2 / (4.0 * du * dl) / n;
    out.gradient(i) = r2 * (2.0 * q(i) - u - l) / (4.0 * du * du * dl * dl) / n;
  }
  return out;
}

ObjectiveValue jointLimitActivationObjective(const RobotModel& model, const VecX& q, double activation_fraction) {
  model.checkConfiguration(q, "jointLimitActivationObjective: q");
  require(activation_fraction > 0.0 && activation_fraction <= 0.5,
          "jointLimitActivationObjective: activation fraction must be in (0, 0.5]");
  ObjectiveValue out{0.0, VecX::Zero(model.dof())};
  for (int i = 0; i < model.dof(); ++i) {
    const double l = model.lowerLimits()(i);
    const double u = model.upperLimits()(i);
    const double d = activation_fraction * (u - l);
    if (d <= 0.0) {
      continue;
    }
    double excess = 0.0;
    if (q(i) < l + d) {
      excess = q(i) - (l + d);
    } else if (q(i) > u - d) {
      excess = q(i) - (u - d);
    }
    out.value += 0.5 * excess * excess / (d * d);
    out.gradient(i) = excess / (d * d);
  }
  return out;
}

ObjectiveValue manipulabilityObjective(const RobotModel& model, const VecX& q, bool translational_only) {
  model.checkConfiguration(q, "manipulabilityObjective: q");
  auto value = [&](const VecX& x) {
    const Mat6X J = geometricJacobian(model, x);
    return -(translational_only ? yoshikawaManipulability(J.bottomRows<3>()) : yoshikawaManipulability(J));
  };
  ObjectiveValue out{value(q), VecX::Zero(model.dof())};
  constexpr double h = 1e-6;
  VecX x = q;
  for (int i = 0; i < model.dof(); ++i) {
    x(i) = q(i) + h;
    const double fp = value(x);
    x(i) = q(i) - h;
    const double fm = value(x);
    x(i) = q(i);
    out.gradient(i) = (fp - fm) / (2.0 * h);
  }
  return out;
}

ObjectiveValue evaluateObjective(NullSpaceObjective objective, const RobotModel& model, const VecX& q) {
  switch (objective) {
    case NullSpaceObjective::None:
      return {0.0, VecX::Zero(model.dof())};
    case NullSpaceObjective::JointCentering:
      return jointCenteringObjective(model, q);
    case NullSpaceObjective::JointLimitBarrier:
      return jointLimitBarrierObjective(model, q);
    case NullSpaceObjective::JointLimitActivation:
      return jointLimitActivationObjective(model, q);
    case NullSpaceObjective::Manipulability:
      return manipulabilityObjective(model, q);
  }
  throw InvalidArgument("evaluateObjective: unknown objective");
}

}  // namespace mc
