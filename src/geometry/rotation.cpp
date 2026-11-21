#include "manipulatorcore/geometry/rotation.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

#include <cmath>

namespace mc {

namespace {

// Below this value of cos(beta) (Tait-Bryan) or sin(beta) (proper Euler), the first and third
// axes are treated as aligned (gimbal lock) and gamma is fixed to zero.
constexpr double kGimbalTolerance = 1e-12;

}  // namespace

Quat normalizeQuaternion(const Quat& q) {
  const double n = q.norm();
  require(std::isfinite(n) && n > 1e-12, "normalizeQuaternion: quaternion norm is zero or not finite");
  return {q.w() / n, q.x() / n, q.y() / n, q.z() / n};
}

Quat canonicalQuaternion(const Quat& q) {
  if (q.w() < 0.0) {
    return {-q.w(), -q.x(), -q.y(), -q.z()};
  }
  return q;
}

Mat3 rotationFromQuaternion(const Quat& q_in) {
  const Quat q = normalizeQuaternion(q_in);
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();
  Mat3 R;
  R << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),  //
      2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),   //
      2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y);
  return R;
}

Quat quaternionFromRotation(const Mat3& R) {
  requireFinite(R, "quaternionFromRotation");
  // Shepperd's method: divide by the largest of 4w^2, 4x^2, 4y^2, 4z^2 for numerical stability.
  const double trace = R.trace();
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(trace + 1.0);
    w = 0.25 * s;
    x = (R(2, 1) - R(1, 2)) / s;
    y = (R(0, 2) - R(2, 0)) / s;
    z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
    w = (R(2, 1) - R(1, 2)) / s;
    x = 0.25 * s;
    y = (R(0, 1) + R(1, 0)) / s;
    z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
    w = (R(0, 2) - R(2, 0)) / s;
    x = (R(0, 1) + R(1, 0)) / s;
    y = 0.25 * s;
    z = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
    w = (R(1, 0) - R(0, 1)) / s;
    x = (R(0, 2) + R(2, 0)) / s;
    y = (R(1, 2) + R(2, 1)) / s;
    z = 0.25 * s;
  }
  return canonicalQuaternion(normalizeQuaternion(Quat(w, x, y, z)));
}

Mat3 rotationFromAxisAngle(const Vec3& axis, double angle) {
  requireFinite(axis, "rotationFromAxisAngle: axis");
  requireFinite(angle, "rotationFromAxisAngle: angle");
  const double n = axis.norm();
  require(n > 1e-12, "rotationFromAxisAngle: axis must be non-zero");
  return expSO3(axis / n * angle);
}

AxisAngle axisAngleFromRotation(const Mat3& R) {
  const Vec3 phi = logSO3(R);
  const double angle = phi.norm();
  if (angle < 1e-15) {
    return {Vec3::UnitX(), 0.0};
  }
  return {phi / angle, angle};
}

Mat3 rotationFromEuler(const Vec3& angles, EulerSequence sequence) {
  requireFinite(angles, "rotationFromEuler: angles");
  switch (sequence) {
    case EulerSequence::ZYX:
      return rotZ(angles(0)) * rotY(angles(1)) * rotX(angles(2));
    case EulerSequence::XYZ:
      return rotX(angles(0)) * rotY(angles(1)) * rotZ(angles(2));
    case EulerSequence::ZYZ:
      return rotZ(angles(0)) * rotY(angles(1)) * rotZ(angles(2));
  }
  throw InvalidArgument("rotationFromEuler: unknown sequence");
}

Vec3 eulerFromRotation(const Mat3& R, EulerSequence sequence) {
  requireFinite(R, "eulerFromRotation");
  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.0;
  switch (sequence) {
    case EulerSequence::ZYX: {
      // R = Rz(a) Ry(b) Rx(c); R(2,0) = -sin(b)
      const double cb = std::hypot(R(0, 0), R(1, 0));
      beta = std::atan2(-R(2, 0), cb);
      if (cb < kGimbalTolerance) {
        alpha = std::atan2(-R(0, 1), R(1, 1));
        gamma = 0.0;
      } else {
        alpha = std::atan2(R(1, 0), R(0, 0));
        // gamma from the residual rotation keeps the reconstruction exact even for small cos(b).
        const Mat3 residual = (rotZ(alpha) * rotY(beta)).transpose() * R;
        gamma = std::atan2(residual(2, 1), residual(2, 2));
      }
      break;
    }
    case EulerSequence::XYZ: {
      // R = Rx(a) Ry(b) Rz(c); R(0,2) = sin(b)
      const double cb = std::hypot(R(1, 2), R(2, 2));
      beta = std::atan2(R(0, 2), cb);
      if (cb < kGimbalTolerance) {
        alpha = std::atan2(R(2, 1), R(1, 1));
        gamma = 0.0;
      } else {
        alpha = std::atan2(-R(1, 2), R(2, 2));
        const Mat3 residual = (rotX(alpha) * rotY(beta)).transpose() * R;
        gamma = std::atan2(residual(1, 0), residual(0, 0));
      }
      break;
    }
    case EulerSequence::ZYZ: {
      // R = Rz(a) Ry(b) Rz(c); R(2,2) = cos(b), b in [0, pi]
      const double sb = std::hypot(R(0, 2), R(1, 2));
      beta = std::atan2(sb, R(2, 2));
      if (sb < kGimbalTolerance) {
        gamma = 0.0;
        alpha = R(2, 2) > 0.0 ? std::atan2(R(1, 0), R(0, 0)) : std::atan2(-R(1, 0), R(1, 1));
      } else {
        alpha = std::atan2(R(1, 2), R(0, 2));
        const Mat3 residual = (rotZ(alpha) * rotY(beta)).transpose() * R;
        gamma = std::atan2(residual(1, 0), residual(0, 0));
      }
      break;
    }
  }
  return {alpha, beta, gamma};
}

Mat3 rotationFromRPY(double roll, double pitch, double yaw) { return rotZ(yaw) * rotY(pitch) * rotX(roll); }

Vec3 rpyFromRotation(const Mat3& R) {
  const Vec3 zyx = eulerFromRotation(R, EulerSequence::ZYX);
  return {zyx(2), zyx(1), zyx(0)};
}

}  // namespace mc
