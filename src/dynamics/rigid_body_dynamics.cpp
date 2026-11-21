#include "manipulatorcore/dynamics/rigid_body_dynamics.hpp"

#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/math/error.hpp"

#include <Eigen/Cholesky>

#include <vector>

namespace mc {

namespace {

/// Motion vector of the parent expressed in the child frame: Ad_{T^{-1}} v, with T = T_parent_child.
Vec6 motionToChild(const SE3& T, const Vec6& v_parent) {
  const Mat3& R = T.rotation();
  const Vec3 w = v_parent.head<3>();
  Vec6 out;
  out.head<3>() = R.transpose() * w;
  out.tail<3>() = R.transpose() * (v_parent.tail<3>() - T.translation().cross(w));
  return out;
}

/// Spatial cross product for motion vectors: v x_m m = ad_v m.
Vec6 crossMotion(const Vec6& v, const Vec6& m) {
  const Vec3 w = v.head<3>();
  Vec6 out;
  out.head<3>() = w.cross(m.head<3>());
  out.tail<3>() = v.tail<3>().cross(m.head<3>()) + w.cross(m.tail<3>());
  return out;
}

/// Spatial cross product for force vectors: v x* f = -ad_v^T f.
Vec6 crossForce(const Vec6& v, const Vec6& f) {
  const Vec3 w = v.head<3>();
  Vec6 out;
  out.head<3>() = w.cross(f.head<3>()) + v.tail<3>().cross(f.tail<3>());
  out.tail<3>() = w.cross(f.tail<3>());
  return out;
}

Vec6 baseAcceleration(const RobotModel& model, bool with_gravity) {
  Vec6 a = Vec6::Zero();
  if (with_gravity) {
    a.tail<3>() = -model.gravity();  // gravity as a fictitious upward base acceleration
  }
  return a;
}

VecX rnea(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& qdd, bool with_gravity) {
  const int n = model.dof();
  std::vector<SE3> T(static_cast<std::size_t>(n));
  std::vector<Vec6> f(static_cast<std::size_t>(n));
  Vec6 v_parent = Vec6::Zero();
  Vec6 a_parent = baseAcceleration(model, with_gravity);
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    const Joint& joint = model.joint(i);
    T[k] = joint.transform(q(i));
    const Vec6 S = joint.motionSubspace();
    const Vec6 v = motionToChild(T[k], v_parent) + S * qd(i);
    const Vec6 a = motionToChild(T[k], a_parent) + S * qdd(i) + crossMotion(v, S * qd(i));
    const Mat6 G = model.body(i).inertia.spatialMatrix();
    f[k] = G * a + crossForce(v, G * v);
    v_parent = v;
    a_parent = a;
  }
  VecX tau(n);
  for (int i = n - 1; i >= 0; --i) {
    const auto k = static_cast<std::size_t>(i);
    tau(i) = model.joint(i).motionSubspace().dot(f[k]) + model.joint(i).armature * qdd(i);
    if (i > 0) {
      f[k - 1] += transformWrench(T[k], f[k]);
    }
  }
  return tau;
}

}  // namespace

VecX inverseDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& qdd) {
  model.checkConfiguration(q, "inverseDynamics: q");
  model.checkConfiguration(qd, "inverseDynamics: qd");
  model.checkConfiguration(qdd, "inverseDynamics: qdd");
  return rnea(model, q, qd, qdd, true);
}

VecX gravityTorques(const RobotModel& model, const VecX& q) {
  model.checkConfiguration(q, "gravityTorques: q");
  const VecX zero = VecX::Zero(model.dof());
  return rnea(model, q, zero, zero, true);
}

VecX biasTorques(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(q, "biasTorques: q");
  model.checkConfiguration(qd, "biasTorques: qd");
  return rnea(model, q, qd, VecX::Zero(model.dof()), true);
}

VecX coriolisTorques(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(q, "coriolisTorques: q");
  model.checkConfiguration(qd, "coriolisTorques: qd");
  return rnea(model, q, qd, VecX::Zero(model.dof()), false);
}

MatX massMatrix(const RobotModel& model, const VecX& q) {
  model.checkConfiguration(q, "massMatrix: q");
  const int n = model.dof();
  std::vector<SE3> T(static_cast<std::size_t>(n));
  std::vector<Vec6> S(static_cast<std::size_t>(n));
  std::vector<Mat6> Ic(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    T[k] = model.joint(i).transform(q(i));
    S[k] = model.joint(i).motionSubspace();
    Ic[k] = model.body(i).inertia.spatialMatrix();
  }
  // Composite inertias: accumulate each subtree into its parent (Featherstone, table 6.2).
  for (int i = n - 1; i > 0; --i) {
    const auto k = static_cast<std::size_t>(i);
    const Mat6 X = T[k].inverse().adjoint();  // parent -> child motion transform
    Ic[k - 1] += X.transpose() * Ic[k] * X;
  }
  MatX M(n, n);
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    Vec6 F = Ic[k] * S[k];
    M(i, i) = S[k].dot(F) + model.joint(i).armature;
    for (int j = i; j > 0; --j) {
      F = transformWrench(T[static_cast<std::size_t>(j)], F);  // express in the parent frame
      M(i, j - 1) = S[static_cast<std::size_t>(j - 1)].dot(F);
      M(j - 1, i) = M(i, j - 1);
    }
  }
  return M;
}

namespace {

VecX articulatedBodyAlgorithm(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& tau) {
  const int n = model.dof();
  const auto un = static_cast<std::size_t>(n);
  std::vector<SE3> T(un);
  std::vector<Vec6> S(un), c(un), pA(un), U(un);
  std::vector<Mat6> IA(un);
  std::vector<double> D(un), u(un);

  // Pass 1: velocities, velocity-product accelerations and bias forces.
  Vec6 v_parent = Vec6::Zero();
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    T[k] = model.joint(i).transform(q(i));
    S[k] = model.joint(i).motionSubspace();
    const Vec6 v = motionToChild(T[k], v_parent) + S[k] * qd(i);
    c[k] = crossMotion(v, S[k] * qd(i));
    IA[k] = model.body(i).inertia.spatialMatrix();
    pA[k] = crossForce(v, IA[k] * v);
    v_parent = v;
  }
  // Pass 2: articulated-body inertias and bias forces, tip to base.
  for (int i = n - 1; i >= 0; --i) {
    const auto k = static_cast<std::size_t>(i);
    U[k] = IA[k] * S[k];
    D[k] = S[k].dot(U[k]) + model.joint(i).armature;
    require(D[k] > 1e-300, "forwardDynamics: articulated inertia of joint " + std::to_string(i) +
                               " is zero (massless subtree without armature)");
    u[k] = tau(i) - S[k].dot(pA[k]);
    if (i > 0) {
      const Mat6 Ia = IA[k] - U[k] * U[k].transpose() / D[k];
      const Vec6 pa = pA[k] + Ia * c[k] + U[k] * (u[k] / D[k]);
      const Mat6 X = T[k].inverse().adjoint();
      IA[k - 1] += X.transpose() * Ia * X;
      pA[k - 1] += transformWrench(T[k], pa);
    }
  }
  // Pass 3: accelerations, base to tip.
  VecX qdd(n);
  Vec6 a_parent = baseAcceleration(model, true);
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    const Vec6 a_prime = motionToChild(T[k], a_parent) + c[k];
    qdd(i) = (u[k] - U[k].dot(a_prime)) / D[k];
    a_parent = a_prime + S[k] * qdd(i);
  }
  return qdd;
}

}  // namespace

VecX forwardDynamics(const RobotModel& model, const VecX& q, const VecX& qd, const VecX& tau,
                     ForwardDynamicsMethod method) {
  model.checkConfiguration(q, "forwardDynamics: q");
  model.checkConfiguration(qd, "forwardDynamics: qd");
  model.checkConfiguration(tau, "forwardDynamics: tau");
  if (method == ForwardDynamicsMethod::ArticulatedBody) {
    return articulatedBodyAlgorithm(model, q, qd, tau);
  }
  const MatX M = massMatrix(model, q);
  const Eigen::LLT<MatX> llt(M);
  require(llt.info() == Eigen::Success, "forwardDynamics: mass matrix is not positive definite");
  return llt.solve(tau - biasTorques(model, q, qd));
}

MatX massMatrixDerivative(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(qd, "massMatrixDerivative: qd");
  constexpr double h = 1e-6;
  return (massMatrix(model, VecX(q + h * qd)) - massMatrix(model, VecX(q - h * qd))) / (2.0 * h);
}

MatX coriolisMatrix(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(q, "coriolisMatrix: q");
  model.checkConfiguration(qd, "coriolisMatrix: qd");
  const int n = model.dof();
  constexpr double h = 1e-6;
  std::vector<MatX> dM(static_cast<std::size_t>(n));
  VecX qp = q;
  VecX qm = q;
  for (int k = 0; k < n; ++k) {
    qp(k) = q(k) + h;
    qm(k) = q(k) - h;
    dM[static_cast<std::size_t>(k)] = (massMatrix(model, qp) - massMatrix(model, qm)) / (2.0 * h);
    qp(k) = q(k);
    qm(k) = q(k);
  }
  MatX C = MatX::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double c = 0.0;
      for (int k = 0; k < n; ++k) {
        const auto uk = static_cast<std::size_t>(k);
        c += 0.5 * (dM[uk](i, j) + dM[static_cast<std::size_t>(j)](i, k) - dM[static_cast<std::size_t>(i)](j, k)) *
             qd(k);
      }
      C(i, j) = c;
    }
  }
  return C;
}

double kineticEnergy(const RobotModel& model, const VecX& q, const VecX& qd) {
  model.checkConfiguration(qd, "kineticEnergy: qd");
  return 0.5 * qd.dot(massMatrix(model, q) * qd);
}

double potentialEnergy(const RobotModel& model, const VecX& q) {
  const KinematicState state = forwardKinematics(model, q);
  double energy = 0.0;
  for (int i = 0; i < model.dof(); ++i) {
    const Inertia& I = model.body(i).inertia;
    energy -= I.mass * model.gravity().dot(state.T_base_body[static_cast<std::size_t>(i)] * I.com);
  }
  return energy;
}

Vec3 centerOfMass(const RobotModel& model, const VecX& q) {
  const KinematicState state = forwardKinematics(model, q);
  Vec3 weighted = Vec3::Zero();
  double mass = 0.0;
  for (int i = 0; i < model.dof(); ++i) {
    const Inertia& I = model.body(i).inertia;
    weighted += I.mass * (state.T_base_body[static_cast<std::size_t>(i)] * I.com);
    mass += I.mass;
  }
  return mass > 0.0 ? Vec3(weighted / mass) : Vec3::Zero();
}

}  // namespace mc
