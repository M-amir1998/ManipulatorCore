#include "manipulatorcore/kinematics/workspace.hpp"

#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/math/error.hpp"
#include "manipulatorcore/math/random.hpp"

namespace mc {

std::vector<WorkspaceSample> sampleWorkspace(const RobotModel& model, const WorkspaceOptions& options) {
  require(options.samples > 0, "sampleWorkspace: samples must be positive");
  require(!options.manipulability_rows.empty(), "sampleWorkspace: manipulability_rows must not be empty");
  for (const int r : options.manipulability_rows) {
    require(r >= 0 && r < 6, "sampleWorkspace: manipulability rows must be in [0, 6)");
  }
  Random rng(options.seed);
  std::vector<WorkspaceSample> samples;
  samples.reserve(static_cast<std::size_t>(options.samples));
  KinematicState state;
  MatX J(static_cast<Index>(options.manipulability_rows.size()), model.dof());
  for (int s = 0; s < options.samples; ++s) {
    VecX q = rng.uniformVector(model.lowerLimits(), model.upperLimits());
    computeForwardKinematics(model, q, state);
    const Mat6X J6 = geometricJacobian(model, state);
    for (std::size_t k = 0; k < options.manipulability_rows.size(); ++k) {
      J.row(static_cast<Index>(k)) = J6.row(options.manipulability_rows[k]);
    }
    samples.push_back({std::move(q), state.T_base_ee.translation(), yoshikawaManipulability(J)});
  }
  return samples;
}

WorkspaceBounds workspaceBounds(const std::vector<WorkspaceSample>& samples) {
  WorkspaceBounds b;
  if (samples.empty()) {
    return b;
  }
  b.min = samples.front().position;
  b.max = samples.front().position;
  for (const WorkspaceSample& s : samples) {
    b.min = b.min.cwiseMin(s.position);
    b.max = b.max.cwiseMax(s.position);
    b.max_reach = std::max(b.max_reach, s.position.norm());
  }
  return b;
}

}  // namespace mc
