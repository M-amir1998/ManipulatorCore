#pragma once

/// @file rigid_body_dynamics.hpp
/// @brief Rigid-body dynamics of serial manipulators.
///
///   M(q) qdd + C(q, qd) qd + g(q) = tau
///
/// Algorithms (R. Featherstone, Rigid Body Dynamics Algorithms, Springer, 2008):
///  - Recursive Newton-Euler (RNEA), O(n): inverse dynamics, gravity and bias torques
///  - Composite Rigid Body (CRBA), O(n^2): joint-space mass matrix
///  - Articulated Body (ABA), O(n): forward dynamics without forming M
///
/// Spatial vectors are written in body-frame coordinates and ordered [omega; v]. Gravity is applied
/// as a fictitious base acceleration a_0 = [0; -g]. Joint armature (reflected rotor inertia) adds
/// armature_i * qdd_i to the torque of joint i and armature_i to M_ii. Viscous joint damping is
/// not part of the rigid-body model; the simulator adds it.

#include "manipulatorcore/model/robot_model.hpp"

namespace mc {

/// Inverse dynamics tau = M(q) qdd + C(q, qd) qd + g(q) (RNEA).
[[nodiscard]] VecX inverseDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& qdd);

/// Gravity torques g(q) = RNEA(q, 0, 0).
[[nodiscard]] VecX gravityTorques(const RobotModel& model, const VecX& q);

/// Bias torques h(q, qd) = C(q, qd) qd + g(q) = RNEA(q, qd, 0).
[[nodiscard]] VecX biasTorques(const RobotModel& model, const VecX& q, const VecX& qd);

/// Coriolis and centrifugal torques C(q, qd) qd = h(q, qd) - g(q).
[[nodiscard]] VecX coriolisTorques(const RobotModel& model, const VecX& q, const VecX& qd);

/// Joint-space mass matrix M(q) (CRBA). Symmetric positive definite for valid models.
[[nodiscard]] MatX massMatrix(const RobotModel& model, const VecX& q);

enum class ForwardDynamicsMethod {
  CompositeRigidBody,  ///< qdd = M^{-1}(tau - h) via CRBA + Cholesky (LLT) solve
  ArticulatedBody      ///< Featherstone's O(n) articulated-body algorithm
};

/// Forward dynamics qdd = M(q)^{-1} (tau - h(q, qd)). Never forms M^{-1} explicitly. Throws
/// InvalidArgument if M is not positive definite (e.g. massless distal bodies without armature).
[[nodiscard]] VecX forwardDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& tau,
                                   ForwardDynamicsMethod method = ForwardDynamicsMethod::ArticulatedBody);

/// Coriolis matrix from Christoffel symbols of the first kind,
///   C_ij = sum_k 1/2 (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i) qd_k,
/// with dM/dq from central differences of CRBA (O(n) mass-matrix evaluations). With this choice
/// Mdot - 2C is skew-symmetric. Intended for analysis and validation, not for control loops (use
/// coriolisTorques() there).
[[nodiscard]] MatX coriolisMatrix(const RobotModel& model, const VecX& q, const VecX& qd);

/// Time derivative of the mass matrix along qd, Mdot = sum_k dM/dq_k qd_k (central differences).
[[nodiscard]] MatX massMatrixDerivative(const RobotModel& model, const VecX& q, const VecX& qd);

/// Kinetic energy 1/2 qd^T M(q) qd (includes armature).
[[nodiscard]] double kineticEnergy(const RobotModel& model, const VecX& q, const VecX& qd);

/// Gravitational potential energy -sum_i m_i g . c_i (base frame), zero at the base origin height.
[[nodiscard]] double potentialEnergy(const RobotModel& model, const VecX& q);

/// Centre of mass of the whole arm in the base frame.
[[nodiscard]] Vec3 centerOfMass(const RobotModel& model, const VecX& q);

}  // namespace mc
