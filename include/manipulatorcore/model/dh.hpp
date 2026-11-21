#pragma once

/// @file dh.hpp
/// @brief Denavit-Hartenberg parameters (standard and modified) and conversion to RobotModel.
///
/// Standard (distal) DH:   T_{i-1}_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
/// Modified (proximal) DH: T_{i-1}_i = Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)
///
/// For the modified convention, row i stores (a_{i-1}, alpha_{i-1}, d_i, theta_i) as in Craig's
/// tables. For a revolute joint theta_i = q_i + theta; for a prismatic joint d_i = q_i + d.

#include "manipulatorcore/model/robot_model.hpp"

#include <string>
#include <vector>

namespace mc {

enum class DHConvention { Standard, Modified };

/// One row of a DH table.
struct DHRow {
  double a = 0.0;      ///< link length [m]
  double alpha = 0.0;  ///< link twist [rad]
  double d = 0.0;      ///< link offset [m]; joint-variable offset for prismatic joints
  double theta = 0.0;  ///< joint angle offset [rad]
  JointType type = JointType::Revolute;
  JointLimits limits;
  std::string name;
};

struct DHTable {
  DHConvention convention = DHConvention::Standard;
  std::vector<DHRow> rows;
  SE3 tool;  ///< end-effector pose relative to the last DH frame
};

/// Homogeneous transform of one DH row for joint value q.
[[nodiscard]] SE3 dhTransform(DHConvention convention, const DHRow& row, double q);

/// DH frames T_0_i for i = 1..n by direct multiplication of the DH matrices (independent of
/// RobotModel; used to cross-validate the general chain).
[[nodiscard]] std::vector<SE3> dhFrames(const DHTable& table, const VecX& q);

/// End-effector pose T_0_n * tool by direct DH multiplication.
[[nodiscard]] SE3 dhForwardKinematics(const DHTable& table, const VecX& q);

/// Converts a DH table into an equivalent RobotModel (kinematics only; inertias are zero and no
/// collision geometry is attached). Body frames follow the RobotModel convention; use
/// dhFrameInBody() to convert data given in DH frames.
[[nodiscard]] RobotModel robotFromDH(const DHTable& table, const std::string& name,
                                     const Vec3& gravity = Vec3(0.0, 0.0, -kStandardGravity));

/// Pose of DH frame i (1-based in DH notation, i.e. the frame after row i) in the RobotModel
/// body frame of joint i (0-based index body = i - 1). Identity for the modified convention;
/// Tx(a_i) Rx(alpha_i) for the standard convention.
[[nodiscard]] SE3 dhFrameInBody(const DHTable& table, int body);

}  // namespace mc
