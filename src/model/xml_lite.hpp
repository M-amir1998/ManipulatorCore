#pragma once

// Minimal, dependency-free XML reader sufficient for URDF: elements, attributes, character data,
// comments, processing instructions, DOCTYPE and CDATA sections. No namespaces, no DTD
// processing, no external entities. Internal header; not installed.

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mc::detail {

struct XmlElement {
  std::string name;
  std::vector<std::pair<std::string, std::string>> attributes;
  std::vector<XmlElement> children;
  std::string text;

  [[nodiscard]] const std::string* attribute(std::string_view key) const;
  [[nodiscard]] const XmlElement* child(std::string_view child_name) const;
  [[nodiscard]] std::vector<const XmlElement*> childrenNamed(std::string_view child_name) const;
};

/// Parses a document and returns its root element. Throws mc::InvalidArgument on malformed input.
[[nodiscard]] XmlElement parseXml(std::string_view document);

}  // namespace mc::detail
