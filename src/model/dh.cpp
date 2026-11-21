#include "manipulatorcore/model/dh.hpp"

#include "manipulatorcore/geometry/so3.hpp"
#include "manipulatorcore/math/error.hpp"

namespace mc {

namespace {

SE3 tx(double a) { return SE3::fromTranslation(Vec3(a, 0.0, 0.0)); }
SE3 tz(double d) { return SE3::fromTranslation(Vec3(0.0, 0.0, d)); }
SE3 rx(double alpha) { return SE3::fromRotation(rotX(alpha)); }
SE3 rz(double theta) { return SE3::fromRotation(rotZ(theta)); }

void validateRow(const DHRow& row) {
  require(std::isfinite(row.a) && std::isfinite(row.alpha) && std::isfinite(row.d) && std::isfinite(row.theta),
          "DH row '" + row.name + "': parameters must be finite");
}

}  // namespace

SE3 dhTransform(DHConvention convention, const DHRow& row, double q) {
  const double theta = row.type == JointType::Revolute ? row.theta + q : row.theta;
  const double d = row.type == JointType::Prismatic ? row.d + q : row.d;
  if (convention == DHConvention::Standard) {
    return rz(theta) * tz(d) * tx(row.a) * rx(row.alpha);
  }
  return rx(row.alpha) * tx(row.a) * rz(theta) * tz(d);
}

std::vector<SE3> dhFrames(const DHTable& table, const VecX& q) {
  requireVector(q, static_cast<Index>(table.rows.size()), "dhFrames: q");
  std::vector<SE3> frames;
  frames.reserve(table.rows.size());
  SE3 T;
  for (std::size_t i = 0; i < table.rows.size(); ++i) {
    T = T * dhTransform(table.convention, table.rows[i], q(static_cast<Index>(i)));
    frames.push_back(T);
  }
  return frames;
}

SE3 dhForwardKinematics(const DHTable& table, const VecX& q) {
  const std::vector<SE3> frames = dhFrames(table, q);
  require(!frames.empty(), "dhForwardKinematics: empty DH table");
  return frames.back() * table.tool;
}

RobotModel robotFromDH(const DHTable& table, const std::string& name, const Vec3& gravity) {
  require(!table.rows.empty(), "robotFromDH: empty DH table");
  std::vector<Body> bodies;
  bodies.reserve(table.rows.size());
  SE3 previous_fixed;  // standard DH: Tx(a_{i-1}) Rx(alpha_{i-1}) of the previous row
  for (std::size_t i = 0; i < table.rows.size(); ++i) {
    const DHRow& row = table.rows[i];
    validateRow(row);
    Body body;
    body.name = row.name.empty() ? "link" + std::to_string(i + 1) : row.name;
    Joint& joint = body.joint;
    joint.name = "joint" + std::to_string(i + 1);
    joint.type = row.type;
    joint.axis = Vec3::UnitZ();
    joint.limits = row.limits;
    if (table.convention == DHConvention::Standard) {
      // Rz(theta) and Tz(d) commute, so the constant part of the row can precede the joint motion.
      joint.origin = previous_fixed * tz(row.d) * rz(row.theta);
      previous_fixed = tx(row.a) * rx(row.alpha);
    } else {
      joint.origin = rx(row.alpha) * tx(row.a) * tz(row.d) * rz(row.theta);
    }
    bodies.push_back(std::move(body));
  }
  const SE3 tool = table.convention == DHConvention::Standard ? previous_fixed * table.tool : table.tool;
  return RobotModel(name, std::move(bodies), tool, gravity);
}

SE3 dhFrameInBody(const DHTable& table, int body) {
  require(body >= 0 && body < static_cast<int>(table.rows.size()), "dhFrameInBody: index out of range");
  if (table.convention == DHConvention::Modified) {
    return SE3::Identity();
  }
  const DHRow& row = table.rows[static_cast<std::size_t>(body)];
  return tx(row.a) * rx(row.alpha);
}

}  // namespace mc
