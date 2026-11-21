#pragma once

/// @file metrics.hpp
/// @brief Tracking-performance metrics computed from sampled signals.

#include "manipulatorcore/math/types.hpp"

#include <vector>

namespace mc {

struct TrackingMetrics {
  double joint_rmse = 0.0;         ///< sqrt(mean over samples of |q_d - q|^2 / n) [rad]
  double max_joint_error = 0.0;    ///< max |q_d - q|_inf [rad]
  double position_rmse = 0.0;      ///< end-effector position RMSE [m]
  double orientation_rmse = 0.0;   ///< end-effector orientation RMSE (geodesic angle) [rad]
  double control_effort = 0.0;     ///< integral of |tau|^2 dt [N^2·m^2·s]
  double rms_torque = 0.0;         ///< sqrt(mean |tau|^2 / n) [N·m]
  double peak_torque = 0.0;        ///< max |tau_i| [N·m]
  double settling_time = 0.0;      ///< first time after which the position error stays below the band [s]
};

/// Inputs sampled on the same time grid. Position and orientation errors may be empty.
struct TrackingSignals {
  std::vector<double> time;
  std::vector<VecX> q_desired;
  std::vector<VecX> q;
  std::vector<VecX> tau;
  std::vector<double> position_error;     ///< |p_d - p| per sample [m]
  std::vector<double> orientation_error;  ///< geodesic angle per sample [rad]
};

/// Computes metrics; @p settling_band is the position-error band [m] used for the settling time
/// (joint-error band [rad] if no position errors are given).
[[nodiscard]] TrackingMetrics computeTrackingMetrics(const TrackingSignals& signals, double settling_band = 1e-3);

}  // namespace mc
