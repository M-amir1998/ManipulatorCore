#include "manipulatorcore/geometry/interpolation.hpp"

#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/geometry/so3.hpp"

#include <cmath>

namespace mc {

Quat slerp(const Quat& q0_in, const Quat& q1_in, double s) {
  const Quat q0 = normalizeQuaternion(q0_in);
  Quat q1 = normalizeQuaternion(q1_in);
  double dot = q0.dot(q1);
  if (dot < 0.0) {  // take the shorter arc
    q1 = Quat(-q1.w(), -q1.x(), -q1.y(), -q1.z());
    dot = -dot;
  }
  if (dot > 0.9995) {
    // Nearly parallel: linear interpolation followed by normalization is accurate and stable.
    const Quat q(q0.w() + s * (q1.w() - q0.w()), q0.x() + s * (q1.x() - q0.x()), q0.y() + s * (q1.y() - q0.y()),
                 q0.z() + s * (q1.z() - q0.z()));
    return normalizeQuaternion(q);
  }
  const double theta = std::acos(dot);
  const double sin_theta = std::sin(theta);
  const double w0 = std::sin((1.0 - s) * theta) / sin_theta;
  const double w1 = std::sin(s * theta) / sin_theta;
  return normalizeQuaternion(Quat(w0 * q0.w() + w1 * q1.w(), w0 * q0.x() + w1 * q1.x(), w0 * q0.y() + w1 * q1.y(),
                                  w0 * q0.z() + w1 * q1.z()));
}

Mat3 interpolateRotation(const Mat3& R0, const Mat3& R1, double s) {
  return R0 * expSO3(s * logSO3(R0.transpose() * R1));
}

SE3 interpolatePoseDecoupled(const SE3& T0, const SE3& T1, double s) {
  return {interpolateRotation(T0.rotation(), T1.rotation(), s),
          T0.translation() + s * (T1.translation() - T0.translation())};
}

SE3 interpolatePoseScrew(const SE3& T0, const SE3& T1, double s) {
  return T0 * expSE3(s * logSE3(T0.inverse() * T1));
}

}  // namespace mc
