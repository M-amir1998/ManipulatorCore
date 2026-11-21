#include "xml_lite.hpp"

#include "manipulatorcore/math/error.hpp"

#include <cctype>

namespace mc::detail {

const std::string* XmlElement::attribute(std::string_view key) const {
  for (const auto& [k, v] : attributes) {
    if (k == key) {
      return &v;
    }
  }
  return nullptr;
}

const XmlElement* XmlElement::child(std::string_view child_name) const {
  for (const XmlElement& c : children) {
    if (c.name == child_name) {
      return &c;
    }
  }
  return nullptr;
}

std::vector<const XmlElement*> XmlElement::childrenNamed(std::string_view child_name) const {
  std::vector<const XmlElement*> out;
  for (const XmlElement& c : children) {
    if (c.name == child_name) {
      out.push_back(&c);
    }
  }
  return out;
}

namespace {

constexpr int kMaxDepth = 256;

class Parser {
 public:
  explicit Parser(std::string_view text) : s_(text) {}

  XmlElement parseDocument() {
    skipMisc();
    require(peek('<'), error("expected root element"));
    XmlElement root = parseElement(0);
    skipMisc();
    require(pos_ == s_.size(), error("unexpected content after root element"));
    return root;
  }

 private:
  [[nodiscard]] std::string error(const std::string& what) const {
    return "XML parse error at offset " + std::to_string(pos_) + ": " + what;
  }

  [[nodiscard]] bool peek(char c) const { return pos_ < s_.size() && s_[pos_] == c; }
  [[nodiscard]] bool startsWith(std::string_view token) const { return s_.substr(pos_, token.size()) == token; }

  void skipWhitespace() {
    while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_])) != 0) {
      ++pos_;
    }
  }

  void skipUntil(std::string_view terminator) {
    const std::size_t end = s_.find(terminator, pos_);
    require(end != std::string_view::npos, error("unterminated construct"));
    pos_ = end + terminator.size();
  }

  // Whitespace, comments, processing instructions and DOCTYPE between elements.
  void skipMisc() {
    for (;;) {
      skipWhitespace();
      if (startsWith("<?")) {
        skipUntil("?>");
      } else if (startsWith("<!--")) {
        skipUntil("-->");
      } else if (startsWith("<!DOCTYPE") || startsWith("<!doctype")) {
        skipUntil(">");
      } else {
        return;
      }
    }
  }

  static bool isNameChar(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_' || c == '-' || c == '.' || c == ':';
  }

  std::string parseName() {
    const std::size_t start = pos_;
    while (pos_ < s_.size() && isNameChar(s_[pos_])) {
      ++pos_;
    }
    require(pos_ > start, error("expected a name"));
    return std::string(s_.substr(start, pos_ - start));
  }

  std::string decodeEntities(std::string_view raw) const {
    std::string out;
    out.reserve(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
      if (raw[i] != '&') {
        out.push_back(raw[i]);
        continue;
      }
      const std::size_t end = raw.find(';', i);
      require(end != std::string_view::npos, error("unterminated entity"));
      const std::string_view entity = raw.substr(i + 1, end - i - 1);
      if (entity == "lt") {
        out.push_back('<');
      } else if (entity == "gt") {
        out.push_back('>');
      } else if (entity == "amp") {
        out.push_back('&');
      } else if (entity == "quot") {
        out.push_back('"');
      } else if (entity == "apos") {
        out.push_back('\'');
      } else {
        throw InvalidArgument(error("unsupported entity '&" + std::string(entity) + ";'"));
      }
      i = end;
    }
    return out;
  }

  XmlElement parseElement(int depth) {
    require(depth < kMaxDepth, error("document nesting is too deep"));
    require(peek('<'), error("expected '<'"));
    ++pos_;
    XmlElement element;
    element.name = parseName();
    for (;;) {
      skipWhitespace();
      require(pos_ < s_.size(), error("unexpected end of document inside a tag"));
      if (startsWith("/>")) {
        pos_ += 2;
        return element;
      }
      if (peek('>')) {
        ++pos_;
        break;
      }
      std::string key = parseName();
      skipWhitespace();
      require(peek('='), error("expected '=' after attribute name"));
      ++pos_;
      skipWhitespace();
      require(peek('"') || peek('\''), error("expected quoted attribute value"));
      const char quote = s_[pos_++];
      const std::size_t end = s_.find(quote, pos_);
      require(end != std::string_view::npos, error("unterminated attribute value"));
      element.attributes.emplace_back(std::move(key), decodeEntities(s_.substr(pos_, end - pos_)));
      pos_ = end + 1;
    }
    // Content.
    for (;;) {
      require(pos_ < s_.size(), error("unexpected end of document; missing </" + element.name + ">"));
      if (startsWith("</")) {
        pos_ += 2;
        const std::string closing = parseName();
        require(closing == element.name, error("mismatched closing tag </" + closing + ">"));
        skipWhitespace();
        require(peek('>'), error("expected '>'"));
        ++pos_;
        return element;
      }
      if (startsWith("<!--")) {
        skipUntil("-->");
      } else if (startsWith("<![CDATA[")) {
        pos_ += 9;
        const std::size_t end = s_.find("]]>", pos_);
        require(end != std::string_view::npos, error("unterminated CDATA section"));
        element.text.append(s_.substr(pos_, end - pos_));
        pos_ = end + 3;
      } else if (startsWith("<?")) {
        skipUntil("?>");
      } else if (peek('<')) {
        element.children.push_back(parseElement(depth + 1));
      } else {
        const std::size_t end = s_.find('<', pos_);
        require(end != std::string_view::npos, error("unexpected end of document in text"));
        element.text += decodeEntities(s_.substr(pos_, end - pos_));
        pos_ = end;
      }
    }
  }

  std::string_view s_;
  std::size_t pos_ = 0;
};

}  // namespace

XmlElement parseXml(std::string_view document) { return Parser(document).parseDocument(); }

}  // namespace mc::detail
