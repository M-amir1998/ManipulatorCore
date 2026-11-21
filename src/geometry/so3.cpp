#include "manipulatorcore/geometry/so3.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace mc {

namespace {

// Below this angle the trigonometric coefficients are evaluated with Taylor series. The truncation
// error of the retained terms is O(theta^6) < 1e-18, far below double precision.
constexpr double kSmallAngle = 1e-3;

// Within this distance of pi the rotation axis is recovered from the symmetric part of R, because
// the skew-symmetric part (proportional to sin(theta)) loses relative precision.
constexpr double kNearPi = 1e-2;

}  // namespace

Mat3 hat(const Vec3& w) {
  Mat3 W;
  W << 0.0, -w.z(), w.y(),  //
      w.z(), 0.0, -w.x(),   //
      -w.y(), w.x(), 0.0;
  return W;
}

Vec3 vee(const Mat3& W) {
  return {0.5 * (W(2, 1) - W(1, 2)), 0.5 * (W(0, 2) - W(2, 0)), 0.5 * (W(1, 0) - W(0, 1))};
}

Mat3 expSO3(const Vec3& phi) {
  const double theta_sq = phi.squaredNorm();
  const double theta = std::sqrt(theta_sq);
  double a = 0.0;  // sin(theta) / theta
  double b = 0.0;  // (1 - cos(theta)) / theta^2
  if (theta < kSmallAngle) {
    a = 1.0 - theta_sq / 6.0 + theta_sq * theta_sq / 120.0;
    b = 0.5 - theta_sq / 24.0 + theta_sq * theta_sq / 720.0;
  } else {
    a = std::sin(theta) / theta;
    const double half_sin = std::sin(0.5 * theta);
    b = 2.0 * half_sin * half_sin / theta_sq;  // 1 - cos(theta) = 2 sin^2(theta/2), no cancellation
  }
  const Mat3 Phi = hat(phi);
  return Mat3::Identity() + a * Phi + b * Phi * Phi;
}

Vec3 logSO3(const Mat3& R) {
  // s = sin(theta) * axis, recovered from the skew-symmetric part of R.
  const Vec3 s(0.5 * (R(2, 1) - R(1, 2)), 0.5 * (R(0, 2) - R(2, 0)), 0.5 * (R(1, 0) - R(0, 1)));
  const double sin_theta = s.norm();
  const double cos_theta = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
  const double theta = std::atan2(sin_theta, cos_theta);

  if (theta < kSmallAngle) {
    // theta / sin(theta) = 1 + theta^2/6 + 7 theta^4/360 + O(theta^6)
    const double theta_sq = theta * theta;
    return (1.0 + theta_sq / 6.0 + 7.0 * theta_sq * theta_sq / 360.0) * s;
  }

  if (theta > kPi - kNearPi) {
    // (R + R^T)/2 = cos(theta) I + (1 - cos(theta)) u u^T
    const Mat3 S = 0.5 * (R + R.transpose()) - cos_theta * Mat3::Identity();
    Index k = 0;
    S.diagonal().maxCoeff(&k);
    Vec3 axis = S.col(k);
    const double norm = axis.norm();
    if (norm <= 0.0) {
      return Vec3::Zero();  // unreachable for a valid rotation with theta near pi
    }
    axis /= norm;
    if (axis.dot(s) < 0.0) {
      axis = -axis;  // keep the axis consistent with sin(theta) >= 0
    }
    return theta * axis;
  }

  return (theta / sin_theta) * s;
}

Mat3 leftJacobianSO3(const Vec3& phi) {
  const double theta_sq = phi.squaredNorm();
  const double theta = std::sqrt(theta_sq);
  double a = 0.0;  // (1 - cos(theta)) / theta^2
  double b = 0.0;  // (theta - sin(theta)) / theta^3
  if (theta < kSmallAngle) {
    a = 0.5 - theta_sq / 24.0 + theta_sq * theta_sq / 720.0;
    b = 1.0 / 6.0 - theta_sq / 120.0 + theta_sq * theta_sq / 5040.0;
  } else {
    const double half_sin = std::sin(0.5 * theta);
    a = 2.0 * half_sin * half_sin / theta_sq;
    b = (theta - std::sin(theta)) / (theta_sq * theta);
  }
  const Mat3 Phi = hat(phi);
  return Mat3::Identity() + a * Phi + b * Phi * Phi;
}

Mat3 leftJacobianInverseSO3(const Vec3& phi) {
  const double theta_sq = phi.squaredNorm();
  const double theta = std::sqrt(theta_sq);
  double c = 0.0;  // (1 - theta sin(theta) / (2 (1 - cos(theta)))) / theta^2
  if (theta < kSmallAngle) {
    c = 1.0 / 12.0 + theta_sq / 720.0;
  } else {
    // theta sin(theta) / (2 (1 - cos(theta))) = (theta/2) cot(theta/2); the half-angle form avoids the
    // cancellation in 1 - cos(theta) that would otherwise dominate the error of c for small theta.
    const double half = 0.5 * theta;
    c = (1.0 - half * std::cos(half) / std::sin(half)) / theta_sq;
  }
  const Mat3 Phi = hat(phi);
  return Mat3::Identity() - 0.5 * Phi + c * Phi * Phi;
}

Mat3 rotX(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 R;
  R << 1.0, 0.0, 0.0,  //
      0.0, c, -s,      //
      0.0, s, c;
  return R;
}

Mat3 rotY(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 R;
  R << c, 0.0, s,    //
      0.0, 1.0, 0.0, //
      -s, 0.0, c;
  return R;
}

Mat3 rotZ(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Mat3 R;
  R << c, -s, 0.0,  //
      s, c, 0.0,    //
      0.0, 0.0, 1.0;
  return R;
}

bool isRotationMatrix(const Mat3& R, double tolerance) {
  if (!R.allFinite()) {
    return false;
  }
  const double orthogonality = (R.transpose() * R - Mat3::Identity()).cwiseAbs().maxCoeff();
  return orthogonality <= tolerance && std::abs(R.determinant() - 1.0) <= tolerance;
}

Mat3 projectToSO3(const Mat3& M) {
  const Eigen::JacobiSVD<Mat3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 D = Mat3::Identity();
  D(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
  return svd.matrixU() * D * svd.matrixV().transpose();
}

double rotationDistance(const Mat3& R_a, const Mat3& R_b) { return logSO3(R_a.transpose() * R_b).norm(); }

}  // namespace mc
