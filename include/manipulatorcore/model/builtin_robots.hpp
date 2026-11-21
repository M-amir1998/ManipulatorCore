#pragma once

/// @file builtin_robots.hpp
/// @brief Generic built-in manipulators with physically plausible (but not vendor-specific)
/// parameters. None of these models reproduces a commercial robot.
///
/// | Model          | DOF | Description                                                         |
/// | -------------- | --- | ------------------------------------------------------------------- |
/// | Planar2R       | 2   | two-link arm in the base x-y plane, gravity along -y                |
/// | Planar3R       | 3   | three-link arm in the base x-y plane, gravity along -y              |
/// | Industrial6R   | 6   | elbow manipulator with spherical wrist, defined by a standard DH table |
/// | Redundant7R    | 7   | anthropomorphic S-R-S arm (3-axis shoulder, elbow, 3-axis wrist)     |

#include "manipulatorcore/model/dh.hpp"
#include "manipulatorcore/model/robot_model.hpp"

namespace mc {

struct Planar2RParameters {
  double l1 = 1.0;            ///< [m]
  double l2 = 0.8;            ///< [m]
  double m1 = 2.0;            ///< [kg], uniform solid cylinder
  double m2 = 1.5;            ///< [kg]
  double link_radius = 0.04;  ///< [m], inertia and collision capsule radius
};

struct Planar3RParameters {
  double l1 = 1.0;
  double l2 = 0.8;
  double l3 = 0.4;
  double m1 = 2.0;
  double m2 = 1.5;
  double m3 = 0.6;
  double link_radius = 0.04;
};

/// Two-link planar arm. Joints rotate about +z; the arm moves in the x-y plane with gravity -y.
/// At q = 0 both links lie along +x. Tool frame at the tip of link 2.
[[nodiscard]] RobotModel makePlanar2R(const Planar2RParameters& params = {});

/// Three-link planar arm (same conventions as makePlanar2R()).
[[nodiscard]] RobotModel makePlanar3R(const Planar3RParameters& params = {});

/// Standard DH table of the generic 6R industrial-style arm (about 1.0 m reach).
[[nodiscard]] DHTable industrial6RDHTable();

/// Generic 6R industrial-style arm with a spherical wrist, built from industrial6RDHTable() with
/// cylinder-approximated link inertias, capsule collision geometry and a base pedestal.
[[nodiscard]] RobotModel makeIndustrial6R();

/// A comfortable, non-singular configuration of Industrial6R.
[[nodiscard]] VecX industrial6RHome();

/// Generic 7-DOF anthropomorphic arm (S-R-S): shoulder yaw/pitch/roll, elbow, wrist
/// roll/pitch/roll. At q = 0 the arm points straight up (a singular configuration).
[[nodiscard]] RobotModel makeRedundant7R();

/// A comfortable, non-singular configuration of Redundant7R (elbow bent, wrist bent).
[[nodiscard]] VecX redundant7RHome();

}  // namespace mc
