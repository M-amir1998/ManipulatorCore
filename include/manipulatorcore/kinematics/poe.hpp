#pragma once

/// @file poe.hpp
/// @brief Product-of-exponentials (PoE) kinematics in the space and body forms.
///
///   space form: T(q) = exp([S_1] q_1) ... exp([S_n] q_n) M
///   body form:  T(q) = M exp([B_1] q_1) ... exp([B_n] q_n),  B_i = Ad_{M^-1} S_i
///
/// Reference: K. M. Lynch and F. C. Park, Modern Robotics, Cambridge University Press, 2017,
/// chapters 4 and 5.

#include "manipulatorcore/model/robot_model.hpp"

#include <vector>

namespace mc {

/// Screw axes ([omega; v]) of every joint at q = 0, and the home end-effector pose M.
struct ProductOfExponentials {
  std::vector<Vec6> space_screws;  ///< S_i, base frame
  std::vector<Vec6> body_screws;   ///< B_i, end-effector frame at home
  SE3 home;                        ///< M = T_base_ee(0)
};

/// Extracts the PoE description of a model.
[[nodiscard]] ProductOfExponentials productOfExponentials(const RobotModel& model);

[[nodiscard]] SE3 poeForwardSpace(const ProductOfExponentials& poe, const VecX& q);
[[nodiscard]] SE3 poeForwardBody(const ProductOfExponentials& poe, const VecX& q);

/// Space Jacobian: J_s column i = Ad_{exp([S_1]q_1)...exp([S_{i-1}]q_{i-1})} S_i.
[[nodiscard]] Mat6X poeSpaceJacobian(const ProductOfExponentials& poe, const VecX& q);

/// Body Jacobian: J_b column i = Ad_{exp(-[B_n]q_n)...exp(-[B_{i+1}]q_{i+1})} B_i.
[[nodiscard]] Mat6X poeBodyJacobian(const ProductOfExponentials& poe, const VecX& q);

}  // namespace mc
