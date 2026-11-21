#pragma once

// Minimal JSON value builder for demo and benchmark output (applications only, not part of the
// library API). Numbers are written with a configurable number of significant digits to keep the
// files compact.

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace mc::apps {

class Json {
 public:
  using Array = std::vector<Json>;
  using Object = std::vector<std::pair<std::string, Json>>;  // insertion ordered

  Json() : value_(nullptr) {}
  Json(double v) : value_(v) {}                                       // NOLINT(google-explicit-constructor)
  Json(int v) : value_(static_cast<double>(v)) {}                     // NOLINT(google-explicit-constructor)
  Json(std::size_t v) : value_(static_cast<double>(v)) {}             // NOLINT(google-explicit-constructor)
  Json(bool v) : value_(v) {}                                         // NOLINT(google-explicit-constructor)
  Json(const char* v) : value_(std::string(v)) {}                     // NOLINT(google-explicit-constructor)
  Json(std::string v) : value_(std::move(v)) {}                       // NOLINT(google-explicit-constructor)
  Json(Array v) : value_(std::make_shared<Array>(std::move(v))) {}    // NOLINT(google-explicit-constructor)
  Json(Object v) : value_(std::make_shared<Object>(std::move(v))) {}  // NOLINT(google-explicit-constructor)

  static Json array() { return Json(Array{}); }
  static Json object() { return Json(Object{}); }

  /// Adds or appends a key (objects only).
  Json& set(const std::string& key, Json value) {
    std::get<std::shared_ptr<Object>>(value_)->emplace_back(key, std::move(value));
    return *this;
  }

  /// Appends an element (arrays only).
  Json& push(Json value) {
    std::get<std::shared_ptr<Array>>(value_)->push_back(std::move(value));
    return *this;
  }

  void write(std::ostream& os, int digits = 6) const {
    if (std::holds_alternative<std::nullptr_t>(value_)) {
      os << "null";
    } else if (const auto* d = std::get_if<double>(&value_)) {
      if (!std::isfinite(*d)) {
        os << "null";  // JSON has no inf/nan
      } else {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.*g", digits, *d);
        os << buf;
      }
    } else if (const auto* b = std::get_if<bool>(&value_)) {
      os << (*b ? "true" : "false");
    } else if (const auto* s = std::get_if<std::string>(&value_)) {
      os << '"';
      for (const char c : *s) {
        if (c == '"' || c == '\\') {
          os << '\\';
        }
        os << c;
      }
      os << '"';
    } else if (const auto* a = std::get_if<std::shared_ptr<Array>>(&value_)) {
      os << '[';
      for (std::size_t i = 0; i < (*a)->size(); ++i) {
        if (i > 0) {
          os << ',';
        }
        (**a)[i].write(os, digits);
      }
      os << ']';
    } else if (const auto* o = std::get_if<std::shared_ptr<Object>>(&value_)) {
      os << '{';
      for (std::size_t i = 0; i < (*o)->size(); ++i) {
        if (i > 0) {
          os << ',';
        }
        os << '"' << (**o)[i].first << "\":";
        (**o)[i].second.write(os, digits);
      }
      os << '}';
    }
  }

 private:
  std::variant<std::nullptr_t, double, bool, std::string, std::shared_ptr<Array>, std::shared_ptr<Object>> value_;
};

}  // namespace mc::apps
