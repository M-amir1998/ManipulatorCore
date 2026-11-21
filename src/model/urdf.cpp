#include "manipulatorcore/model/urdf.hpp"

#include "manipulatorcore/geometry/rotation.hpp"
#include "manipulatorcore/math/error.hpp"
#include "xml_lite.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <optional>
#include <fstream>
#include <locale>
#include <map>
#include <sstream>
#include <variant>

namespace mc {

namespace {

using detail::XmlElement;

// ------------------------------------------------------------------------------------------------
// Export
// ------------------------------------------------------------------------------------------------

/// Shortest decimal representation that round-trips exactly.
std::string num(double value) {
  std::array<char, 64> buffer{};
  const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  return {buffer.data(), result.ptr};
}

std::string vec3(const Vec3& v) { return num(v.x()) + " " + num(v.y()) + " " + num(v.z()); }

std::string originTag(const SE3& T) {
  return "<origin xyz=\"" + vec3(T.translation()) + "\" rpy=\"" + vec3(rpyFromRotation(T.rotation())) + "\"/>";
}

void writeShape(std::ostringstream& os, const Shape& shape, const std::string& indent, const char* tag,
                bool expand_capsules) {
  auto element = [&](const SE3& pose, const std::string& geometry, const std::string& name) {
    os << indent << "<" << tag;
    if (!name.empty()) {
      os << " name=\"" << name << "\"";
    }
    os << ">\n"
       << indent << "  " << originTag(pose) << "\n"
       << indent << "  <geometry>" << geometry << "</geometry>\n"
       << indent << "</" << tag << ">\n";
  };
  if (const auto* sphere = std::get_if<Sphere>(&shape.geometry)) {
    element(shape.pose, "<sphere radius=\"" + num(sphere->radius) + "\"/>", shape.name);
  } else if (const auto* box = std::get_if<Box>(&shape.geometry)) {
    element(shape.pose, "<box size=\"" + vec3(2.0 * box->half_extents) + "\"/>", shape.name);
  } else if (const auto* capsule = std::get_if<Capsule>(&shape.geometry)) {
    // URDF has no portable capsule primitive: the core cylinder is written as <cylinder>, which
    // fromURDF() reads back as a capsule. Visuals add the two hemispherical end caps as spheres.
    element(shape.pose,
            "<cylinder radius=\"" + num(capsule->radius) + "\" length=\"" + num(2.0 * capsule->half_length) + "\"/>",
            shape.name);
    if (expand_capsules) {
      for (const double sign : {-1.0, 1.0}) {
        const SE3 cap = shape.pose * SE3::fromTranslation(Vec3(0.0, 0.0, sign * capsule->half_length));
        element(cap, "<sphere radius=\"" + num(capsule->radius) + "\"/>", "");
      }
    }
  }
  // Half-spaces have no URDF equivalent and are omitted.
}

void writeLinkGeometry(std::ostringstream& os, const std::vector<Shape>& shapes, bool visuals) {
  for (const Shape& s : shapes) {
    writeShape(os, s, "    ", "collision", false);
  }
  if (visuals) {
    for (const Shape& s : shapes) {
      writeShape(os, s, "    ", "visual", true);
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Import helpers
// ------------------------------------------------------------------------------------------------

std::vector<double> parseNumbers(const std::string& text, std::size_t expected, const std::string& what) {
  std::istringstream is(text);
  is.imbue(std::locale::classic());
  std::vector<double> values;
  double v = 0.0;
  while (is >> v) {
    values.push_back(v);
  }
  require(is.eof() && values.size() == expected,
          "URDF: expected " + std::to_string(expected) + " numbers for " + what + ", got '" + text + "'");
  for (const double x : values) {
    requireFinite(x, "URDF number");
  }
  return values;
}

double numberAttribute(const XmlElement& e, const char* key, std::optional<double> fallback, const std::string& what) {
  const std::string* value = e.attribute(key);
  if (value == nullptr) {
    require(fallback.has_value(), "URDF: missing attribute '" + std::string(key) + "' in " + what);
    return *fallback;
  }
  return parseNumbers(*value, 1, what + "/" + key)[0];
}

Vec3 vec3Attribute(const XmlElement& e, const char* key, const Vec3& fallback, const std::string& what) {
  const std::string* value = e.attribute(key);
  if (value == nullptr) {
    return fallback;
  }
  const auto v = parseNumbers(*value, 3, what + "/" + key);
  return {v[0], v[1], v[2]};
}

SE3 parseOrigin(const XmlElement* parent, const std::string& what) {
  if (parent == nullptr) {
    return SE3::Identity();
  }
  const XmlElement* origin = parent->child("origin");
  if (origin == nullptr) {
    return SE3::Identity();
  }
  const Vec3 xyz = vec3Attribute(*origin, "xyz", Vec3::Zero(), what);
  const Vec3 rpy = vec3Attribute(*origin, "rpy", Vec3::Zero(), what);
  return {rotationFromRPY(rpy.x(), rpy.y(), rpy.z()), xyz};
}

std::optional<Shape> parseGeometry(const XmlElement& holder, const std::string& link_name) {
  const XmlElement* geometry = holder.child("geometry");
  require(geometry != nullptr, "URDF: <collision> without <geometry> in link '" + link_name + "'");
  const SE3 pose = parseOrigin(&holder, "collision origin of " + link_name);
  const std::string name = holder.attribute("name") != nullptr ? *holder.attribute("name") : link_name;
  if (const XmlElement* s = geometry->child("sphere")) {
    return Shape{Sphere{numberAttribute(*s, "radius", std::nullopt, "sphere")}, pose, name};
  }
  if (const XmlElement* b = geometry->child("box")) {
    return Shape{Box{0.5 * vec3Attribute(*b, "size", Vec3::Zero(), "box")}, pose, name};
  }
  const XmlElement* c = geometry->child("cylinder");
  if (c == nullptr) {
    c = geometry->child("capsule");
  }
  if (c != nullptr) {
    return Shape{Capsule{numberAttribute(*c, "radius", std::nullopt, "cylinder"),
                         0.5 * numberAttribute(*c, "length", std::nullopt, "cylinder")},
                 pose, name};
  }
  return std::nullopt;  // meshes and unsupported primitives are ignored
}

Inertia parseInertial(const XmlElement& link, const std::string& link_name) {
  const XmlElement* inertial = link.child("inertial");
  if (inertial == nullptr) {
    return {};
  }
  const SE3 frame = parseOrigin(inertial, "inertial origin of " + link_name);
  const XmlElement* mass = inertial->child("mass");
  require(mass != nullptr, "URDF: <inertial> without <mass> in link '" + link_name + "'");
  Inertia inertia;
  inertia.mass = numberAttribute(*mass, "value", std::nullopt, "mass of " + link_name);
  if (const XmlElement* I = inertial->child("inertia")) {
    const std::string what = "inertia of " + link_name;
    const double ixx = numberAttribute(*I, "ixx", 0.0, what);
    const double ixy = numberAttribute(*I, "ixy", 0.0, what);
    const double ixz = numberAttribute(*I, "ixz", 0.0, what);
    const double iyy = numberAttribute(*I, "iyy", 0.0, what);
    const double iyz = numberAttribute(*I, "iyz", 0.0, what);
    const double izz = numberAttribute(*I, "izz", 0.0, what);
    inertia.inertia_com << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  }
  // The inertia tensor is given in the (possibly rotated) inertial frame; express it in link axes.
  inertia.com = frame.translation();
  inertia.inertia_com = frame.rotation() * inertia.inertia_com * frame.rotation().transpose();
  return inertia;
}

struct UrdfJoint {
  const XmlElement* element = nullptr;
  std::string name;
  std::string type;
  std::string parent;
  std::string child;
};

}  // namespace

std::string toURDF(const RobotModel& model, const UrdfExportOptions& options) {
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os << "<?xml version=\"1.0\"?>\n";
  os << "<!-- Exported by ManipulatorCore from model '" << model.name() << "'. -->\n";
  os << "<robot name=\"" << model.name() << "\">\n";
  os << "  <link name=\"" << options.base_link << "\">\n";
  writeLinkGeometry(os, model.baseCollision(), options.include_visuals);
  os << "  </link>\n";

  std::string parent = options.base_link;
  for (int i = 0; i < model.dof(); ++i) {
    const Body& b = model.body(i);
    const Joint& j = b.joint;
    os << "  <joint name=\"" << j.name << "\" type=\"" << (j.type == JointType::Revolute ? "revolute" : "prismatic")
       << "\">\n"
       << "    <parent link=\"" << parent << "\"/>\n"
       << "    <child link=\"" << b.name << "\"/>\n"
       << "    " << originTag(j.origin) << "\n"
       << "    <axis xyz=\"" << vec3(j.axis) << "\"/>\n"
       << "    <limit lower=\"" << num(j.limits.lower) << "\" upper=\"" << num(j.limits.upper) << "\" effort=\""
       << num(j.limits.effort) << "\" velocity=\"" << num(j.limits.velocity) << "\"/>\n"
       << "    <dynamics damping=\"" << num(j.damping) << "\" friction=\"0\"/>\n"
       << "    <manipulatorcore acceleration=\"" << num(j.limits.acceleration) << "\" armature=\"" << num(j.armature)
       << "\"/>\n"
       << "  </joint>\n";
    os << "  <link name=\"" << b.name << "\">\n";
    const Inertia& I = b.inertia;
    os << "    <inertial>\n"
       << "      <origin xyz=\"" << vec3(I.com) << "\" rpy=\"0 0 0\"/>\n"
       << "      <mass value=\"" << num(I.mass) << "\"/>\n"
       << "      <inertia ixx=\"" << num(I.inertia_com(0, 0)) << "\" ixy=\"" << num(I.inertia_com(0, 1)) << "\" ixz=\""
       << num(I.inertia_com(0, 2)) << "\" iyy=\"" << num(I.inertia_com(1, 1)) << "\" iyz=\""
       << num(I.inertia_com(1, 2)) << "\" izz=\"" << num(I.inertia_com(2, 2)) << "\"/>\n"
       << "    </inertial>\n";
    writeLinkGeometry(os, b.collision, options.include_visuals);
    os << "  </link>\n";
    parent = b.name;
  }
  os << "  <joint name=\"" << options.tool_link << "_joint\" type=\"fixed\">\n"
     << "    <parent link=\"" << parent << "\"/>\n"
     << "    <child link=\"" << options.tool_link << "\"/>\n"
     << "    " << originTag(model.toolTransform()) << "\n"
     << "  </joint>\n"
     << "  <link name=\"" << options.tool_link << "\"/>\n"
     << "</robot>\n";
  return os.str();
}

RobotModel fromURDF(std::string_view urdf_xml, const UrdfImportOptions& options) {
  const XmlElement root = detail::parseXml(urdf_xml);
  require(root.name == "robot", "URDF: root element must be <robot>");
  const std::string robot_name = root.attribute("name") != nullptr ? *root.attribute("name") : "urdf_robot";

  std::map<std::string, const XmlElement*> links;
  for (const XmlElement* link : root.childrenNamed("link")) {
    const std::string* name = link->attribute("name");
    require(name != nullptr, "URDF: <link> without a name");
    require(links.emplace(*name, link).second, "URDF: duplicate link '" + *name + "'");
  }
  std::map<std::string, UrdfJoint> joint_by_child;
  std::map<std::string, std::vector<std::string>> children_of;
  for (const XmlElement* j : root.childrenNamed("joint")) {
    UrdfJoint uj;
    uj.element = j;
    require(j->attribute("name") != nullptr && j->attribute("type") != nullptr, "URDF: joint needs name and type");
    uj.name = *j->attribute("name");
    uj.type = *j->attribute("type");
    const XmlElement* p = j->child("parent");
    const XmlElement* c = j->child("child");
    require(p != nullptr && c != nullptr && p->attribute("link") != nullptr && c->attribute("link") != nullptr,
            "URDF: joint '" + uj.name + "' needs <parent link> and <child link>");
    uj.parent = *p->attribute("link");
    uj.child = *c->attribute("link");
    require(links.count(uj.parent) == 1 && links.count(uj.child) == 1,
            "URDF: joint '" + uj.name + "' references an unknown link");
    require(joint_by_child.count(uj.child) == 0, "URDF: link '" + uj.child + "' has more than one parent joint");
    children_of[uj.parent].push_back(uj.child);
    joint_by_child.emplace(uj.child, uj);
  }

  // Base link: explicit, or the unique root of the tree.
  std::string base = options.base_link;
  if (base.empty()) {
    for (const auto& [name, element] : links) {
      if (joint_by_child.count(name) == 0) {
        require(base.empty(), "URDF: more than one root link; set UrdfImportOptions::base_link");
        base = name;
      }
    }
  }
  require(links.count(base) == 1, "URDF: base link '" + base + "' not found");

  // Ordered list of links from base to tip.
  std::vector<std::string> path;
  if (!options.tip_link.empty()) {
    require(links.count(options.tip_link) == 1, "URDF: tip link '" + options.tip_link + "' not found");
    std::string current = options.tip_link;
    while (current != base) {
      path.push_back(current);
      const auto it = joint_by_child.find(current);
      require(it != joint_by_child.end(), "URDF: tip link is not a descendant of the base link");
      current = it->second.parent;
    }
    path.push_back(base);
    std::reverse(path.begin(), path.end());
  } else {
    std::string current = base;
    path.push_back(current);
    for (;;) {
      const auto it = children_of.find(current);
      if (it == children_of.end() || it->second.empty()) {
        break;
      }
      require(it->second.size() == 1,
              "URDF: link '" + current + "' branches; set UrdfImportOptions::tip_link to select a chain");
      current = it->second.front();
      path.push_back(current);
    }
  }

  std::vector<Body> bodies;
  std::vector<Shape> base_shapes;
  SE3 pending;  // accumulated fixed transforms since the last movable joint (or the base)
  auto addLinkContents = [&](const std::string& link_name) {
    const XmlElement& link = *links.at(link_name);
    std::vector<Shape> shapes;
    for (const XmlElement* collision : link.childrenNamed("collision")) {
      if (auto shape = parseGeometry(*collision, link_name)) {
        shapes.push_back(transformShape(pending, *shape));
      }
    }
    if (bodies.empty()) {
      base_shapes.insert(base_shapes.end(), shapes.begin(), shapes.end());
      return;
    }
    Body& body = bodies.back();
    body.collision.insert(body.collision.end(), shapes.begin(), shapes.end());
    body.inertia = Inertia::combine(body.inertia, parseInertial(link, link_name).transformed(pending));
  };

  addLinkContents(path.front());
  for (std::size_t k = 1; k < path.size(); ++k) {
    const UrdfJoint& uj = joint_by_child.at(path[k]);
    const XmlElement& je = *uj.element;
    const SE3 origin = parseOrigin(&je, "origin of joint " + uj.name);
    if (uj.type == "fixed") {
      pending = pending * origin;
    } else {
      require(uj.type == "revolute" || uj.type == "continuous" || uj.type == "prismatic",
              "URDF: unsupported joint type '" + uj.type + "' for joint '" + uj.name + "'");
      Body body;
      body.name = uj.child;
      Joint& joint = body.joint;
      joint.name = uj.name;
      joint.type = uj.type == "prismatic" ? JointType::Prismatic : JointType::Revolute;
      joint.origin = pending * origin;
      const Vec3 axis = je.child("axis") != nullptr ? vec3Attribute(*je.child("axis"), "xyz", Vec3::UnitX(), uj.name)
                                                     : Vec3::UnitX();
      require(axis.norm() > 1e-12, "URDF: joint '" + uj.name + "' has a zero axis");
      joint.axis = axis.normalized();
      joint.limits.velocity = options.default_velocity_limit;
      joint.limits.acceleration = options.default_acceleration_limit;
      joint.limits.effort = options.default_effort_limit;
      if (uj.type == "continuous") {
        joint.limits.lower = -0.5 * options.continuous_joint_range;
        joint.limits.upper = 0.5 * options.continuous_joint_range;
      }
      if (const XmlElement* limit = je.child("limit")) {
        const std::string what = "limit of joint " + uj.name;
        if (uj.type != "continuous") {
          joint.limits.lower = numberAttribute(*limit, "lower", 0.0, what);
          joint.limits.upper = numberAttribute(*limit, "upper", 0.0, what);
        }
        const double velocity = numberAttribute(*limit, "velocity", options.default_velocity_limit, what);
        const double effort = numberAttribute(*limit, "effort", options.default_effort_limit, what);
        joint.limits.velocity = velocity > 0.0 ? velocity : options.default_velocity_limit;
        joint.limits.effort = effort > 0.0 ? effort : options.default_effort_limit;
      } else {
        require(uj.type == "continuous", "URDF: joint '" + uj.name + "' requires a <limit> element");
      }
      if (const XmlElement* dynamics = je.child("dynamics")) {
        joint.damping = numberAttribute(*dynamics, "damping", 0.0, "dynamics of " + uj.name);
      }
      if (const XmlElement* ext = je.child("manipulatorcore")) {
        joint.limits.acceleration =
            numberAttribute(*ext, "acceleration", options.default_acceleration_limit, "extension of " + uj.name);
        joint.armature = numberAttribute(*ext, "armature", 0.0, "extension of " + uj.name);
      }
      bodies.push_back(std::move(body));
      pending = SE3::Identity();
    }
    addLinkContents(path[k]);
  }
  require(!bodies.empty(), "URDF: the selected chain has no movable joints");

  RobotModel model(robot_name, std::move(bodies), pending, options.gravity);
  model.setBaseCollision(std::move(base_shapes));
  return model;
}

RobotModel loadURDF(const std::string& path, const UrdfImportOptions& options) {
  std::ifstream file(path, std::ios::binary);
  require(file.good(), "loadURDF: cannot open '" + path + "'");
  std::ostringstream contents;
  contents << file.rdbuf();
  return fromURDF(contents.str(), options);
}

}  // namespace mc
