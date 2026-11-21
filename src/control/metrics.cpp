#include "manipulatorcore/control/metrics.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <cmath>

namespace mc {

TrackingMetrics computeTrackingMetrics(const TrackingSignals& s, double settling_band) {
  const std::size_t N = s.time.size();
  require(N > 0, "computeTrackingMetrics: no samples");
  require(s.q.size() == N && s.q_desired.size() == N && s.tau.size() == N,
          "computeTrackingMetrics: q, q_desired and tau must have one entry per sample");
  require(s.position_error.empty() || s.position_error.size() == N,
          "computeTrackingMetrics: position_error must be empty or have one entry per sample");
  require(s.orientation_error.empty() || s.orientation_error.size() == N,
          "computeTrackingMetrics: orientation_error must be empty or have one entry per sample");
  TrackingMetrics m;
  const auto n = static_cast<double>(s.q.front().size());
  double joint_sq = 0.0;
  double tau_sq = 0.0;
  double pos_sq = 0.0;
  double ori_sq = 0.0;
  for (std::size_t k = 0; k < N; ++k) {
    const VecX e = s.q_desired[k] - s.q[k];
    joint_sq += e.squaredNorm() / n;
    m.max_joint_error = std::max(m.max_joint_error, e.cwiseAbs().maxCoeff());
    tau_sq += s.tau[k].squaredNorm() / n;
    m.peak_torque = std::max(m.peak_torque, s.tau[k].cwiseAbs().maxCoeff());
    if (k > 0) {
      const double dt = s.time[k] - s.time[k - 1];
      m.control_effort += 0.5 * dt * (s.tau[k].squaredNorm() + s.tau[k - 1].squaredNorm());
    }
    if (!s.position_error.empty()) {
      pos_sq += s.position_error[k] * s.position_error[k];
    }
    if (!s.orientation_error.empty()) {
      ori_sq += s.orientation_error[k] * s.orientation_error[k];
    }
  }
  const auto count = static_cast<double>(N);
  m.joint_rmse = std::sqrt(joint_sq / count);
  m.rms_torque = std::sqrt(tau_sq / count);
  m.position_rmse = std::sqrt(pos_sq / count);
  m.orientation_rmse = std::sqrt(ori_sq / count);
  // Settling time: the first sample after the last excursion outside the band.
  const bool use_position = !s.position_error.empty();
  std::size_t last_outside = N;  // N means "never outside"
  for (std::size_t k = 0; k < N; ++k) {
    const double err = use_position ? s.position_error[k] : (s.q_desired[k] - s.q[k]).cwiseAbs().maxCoeff();
    if (err > settling_band) {
      last_outside = k;
    }
  }
  if (last_outside == N) {
    m.settling_time = 0.0;
  } else if (last_outside + 1 < N) {
    m.settling_time = s.time[last_outside + 1] - s.time.front();
  } else {
    m.settling_time = s.time.back() - s.time.front();  // never settled within the record
  }
  return m;
}

}  // namespace mc
