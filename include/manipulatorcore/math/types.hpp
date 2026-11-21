#pragma once

/// @file types.hpp
/// @brief Fixed- and dynamic-size Eigen aliases used throughout ManipulatorCore.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <numbers>

namespace mc {

using Index = Eigen::Index;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecX = Eigen::VectorXd;

using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using MatX = Eigen::MatrixXd;
using Mat6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

/// Hamilton unit quaternion. Constructed as Quat(w, x, y, z); see docs/mathematics/conventions.md.
using Quat = Eigen::Quaterniond;

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kHalfPi = 0.5 * std::numbers::pi;

/// Standard gravitational acceleration [m/s^2].
inline constexpr double kStandardGravity = 9.81;

}  // namespace mc
