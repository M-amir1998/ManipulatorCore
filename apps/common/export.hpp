#pragma once

// Conversion of library objects into JSON for the Python renderer. The renderer never performs
// kinematics itself: every frame carries world-space link geometry computed here.

#include "json.hpp"
#include "manipulatorcore/geometry/shapes.hpp"
#include "manipulatorcore/kinematics/forward_kinematics.hpp"
#include "manipulatorcore/model/robot_model.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace mc::apps {

inline Json vec(const Eigen::Ref<const Eigen::VectorXd>& v) {
  Json a = Json::array();
  for (Index i = 0; i < v.size(); ++i) {
    a.push(v(i));
  }
  return a;
}

inline Json vec3(const Vec3& v) { return vec(v); }

/// 4x4 homogeneous matrix, row-major.
inline Json pose(const SE3& T) {
  const Mat4 M = T.matrix();
  Json a = Json::array();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      a.push(M(r, c));
    }
  }
  return a;
}

/// A posed primitive: {"type", "pose" (4x4), dimensions...} in the world frame.
inline Json shape(const Shape& s, const SE3& T_world_parent = SE3::Identity()) {
  const SE3 T = T_world_parent * s.pose;
  Json o = Json::object();
  o.set("name", s.name).set("pose", pose(T));
  if (const auto* sp = std::get_if<Sphere>(&s.geometry)) {
    o.set("type", "sphere").set("radius", sp->radius);
  } else if (const auto* c = std::get_if<Capsule>(&s.geometry)) {
    o.set("type", "capsule")
        .set("radius", c->radius)
        .set("a", vec3(T * Vec3(0, 0, -c->half_length)))
        .set("b", vec3(T * Vec3(0, 0, c->half_length)));
  } else if (const auto* b = std::get_if<Box>(&s.geometry)) {
    o.set("type", "box").set("half_extents", vec3(b->half_extents));
  } else if (const auto* h = std::get_if<HalfSpace>(&s.geometry)) {
    o.set("type", "halfspace")
        .set("normal", vec3(T.rotation() * h->normal))
        .set("offset", h->offset + (T.rotation() * h->normal).dot(T.translation()));
  }
  return o;
}

inline Json shapes(const std::vector<Shape>& list) {
  Json a = Json::array();
  for (const Shape& s : list) {
    a.push(shape(s));
  }
  return a;
}

/// Robot geometry at configuration q: joint origins, link capsules [ax,ay,az,bx,by,bz,r,body],
/// and the end-effector pose.
inline Json robotFrame(const RobotModel& model, const VecX& q, double time = 0.0) {
  const KinematicState state = forwardKinematics(model, q);
  Json joints = Json::array();
  joints.push(vec3(Vec3::Zero()));
  for (int i = 0; i < model.dof(); ++i) {
    joints.push(vec3(state.jointOrigin(i)));
  }
  joints.push(vec3(state.T_base_ee.translation()));
  Json links = Json::array();
  auto addCapsules = [&](const std::vector<Shape>& list, const SE3& T, int body) {
    for (const Shape& s : list) {
      if (const auto* c = std::get_if<Capsule>(&s.geometry)) {
        const SE3 P = T * s.pose;
        const Vec3 a = P * Vec3(0, 0, -c->half_length);
        const Vec3 b = P * Vec3(0, 0, c->half_length);
        Json l = Json::array();
        for (int k = 0; k < 3; ++k) {
          l.push(a(k));
        }
        for (int k = 0; k < 3; ++k) {
          l.push(b(k));
        }
        l.push(c->radius).push(body);
        links.push(l);
      }
    }
  };
  addCapsules(model.baseCollision(), SE3::Identity(), -1);
  for (int i = 0; i < model.dof(); ++i) {
    addCapsules(model.body(i).collision, state.T_base_body[static_cast<std::size_t>(i)], i);
  }
  Json f = Json::object();
  f.set("t", time).set("q", vec(q)).set("joints", joints).set("links", links).set("ee", pose(state.T_base_ee));
  return f;
}

/// Writes a JSON document to <dir>/<name>.json and returns the path.
inline std::filesystem::path writeJson(const std::filesystem::path& dir, const std::string& name, const Json& doc) {
  std::filesystem::create_directories(dir);
  const std::filesystem::path path = dir / (name + ".json");
  std::ofstream out(path);
  doc.write(out);
  return path;
}

}  // namespace mc::apps
