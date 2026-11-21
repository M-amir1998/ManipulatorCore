// Forward kinematics, Jacobians, singularity analysis, inverse kinematics and dynamics of the
// generic 7-DOF arm in a few lines.

#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/kinematics/jacobian.hpp"
#include "manipulatorcore/kinematics/singularity.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

#include <cstdio>

int main() {
  using namespace mc;
  const RobotModel robot = makeRedundant7R();
  const VecX q = redundant7RHome();

  // Forward kinematics: pose of the tool frame in the base frame (T_base_ee).
  const SE3 T_base_ee = endEffectorPose(robot, q);
  std::printf("end-effector position: [%.3f %.3f %.3f] m\n", T_base_ee.translation().x(), T_base_ee.translation().y(),
              T_base_ee.translation().z());

  // Geometric Jacobian (rows [omega; v]) and singularity metrics.
  const SingularityReport report = analyzeSingularity(robot, q);
  std::printf("rank %d, condition number %.1f, manipulability %.4f\n", report.full.rank, report.full.condition_number,
              report.full.manipulability);

  // Inverse kinematics of a shifted target with damped least squares, respecting joint limits.
  const SE3 target(T_base_ee.rotation(), T_base_ee.translation() + Vec3(0.05, -0.10, 0.08));
  IKOptions options;
  options.method = IKMethod::DampedLeastSquares;
  const IKResult ik = solveIK(robot, target, q, options);
  std::printf("IK %s after %d iterations: position error %.2e m, orientation error %.2e rad\n", toString(ik.status),
              ik.iterations, ik.position_error, ik.orientation_error);

  // Rigid-body dynamics: mass matrix (CRBA), gravity torques (RNEA), forward dynamics (ABA).
  const MatX M = massMatrix(robot, q);
  const VecX g = gravityTorques(robot, q);
  const VecX qdd = forwardDynamics(robot, q, VecX::Zero(7), g);  // gravity compensation: qdd = 0
  std::printf("M(0,0) = %.3f kg m^2, |g| = %.2f N m, |qdd| under gravity compensation = %.1e\n", M(0, 0), g.norm(),
              qdd.norm());
  return ik.converged() ? 0 : 1;
}
