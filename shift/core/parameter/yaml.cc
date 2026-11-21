#include "shift/core/parameter/yaml.h"

#include <yaml-cpp/yaml.h>

#include <charconv>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "shift/core/exception.h"

static constexpr int kMaxDepth = 256;

/** The line (counted from 1) of a node, for messages. */
static std::string Where(const YAML::Node& node) {
  const YAML::Mark mark = node.Mark();
  if(mark.is_null()) {
    return "";
  }
  return " at line " + std::to_string(mark.line + 1);
}

static bool IsDigits(const std::string& s, size_t first) {
  if(first >= s.size()) {
    return false;
  }
  for(size_t i = first; i < s.size(); ++i) {
    if(s[i] < '0' || s[i] > '9') {
      return false;
    }
  }
  return true;
}

static bool IsHex(const std::string& s) {
  if(s.size() < 3 || s[0] != '0' || (s[1] != 'x' && s[1] != 'X')) {
    return false;
  }
  for(size_t i = 2; i < s.size(); ++i) {
    if(!std::isxdigit(static_cast<unsigned char>(s[i]))) {
      return false;
    }
  }
  return true;
}

/** `[-+]?(\.[0-9]+|[0-9]+(\.[0-9]*)?)([eE][-+]?[0-9]+)?` */
static bool IsReal(const std::string& s) {
  size_t i = 0;
  if(i < s.size() && (s[i] == '-' || s[i] == '+')) {
    ++i;
  }
  size_t whole = 0;
  while(i < s.size() && s[i] >= '0' && s[i] <= '9') {
    ++i;
    ++whole;
  }
  size_t fraction = 0;
  if(i < s.size() && s[i] == '.') {
    ++i;
    while(i < s.size() && s[i] >= '0' && s[i] <= '9') {
      ++i;
      ++fraction;
    }
  }
  if(whole == 0 && fraction == 0) {
    return false;
  }
  if(i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
    ++i;
    if(i < s.size() && (s[i] == '-' || s[i] == '+')) {
      ++i;
    }
    if(!IsDigits(s, i)) {
      return false;
    }
    i = s.size();
  }
  return i == s.size();
}

static Json::Value PlainScalar(const std::string& s) {
  if(s.empty() || s == "~" || s == "null" || s == "Null" || s == "NULL") {
    return Json::Value(Json::nullValue);
  }
  if(s == "true" || s == "True" || s == "TRUE") {
    return Json::Value(true);
  }
  if(s == "false" || s == "False" || s == "FALSE") {
    return Json::Value(false);
  }
  const size_t sign = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  if(IsDigits(s, sign)) {
    errno = 0;
    char* end = nullptr;
    if(s[0] == '-') {
      const long long value = std::strtoll(s.c_str(), &end, 10);
      if(errno == 0) {
        return Json::Value(static_cast<Json::Int64>(value));
      }
    } else {
      const unsigned long long value = std::strtoull(s.c_str(), &end, 10);
      if(errno == 0) {
        if(value <= static_cast<unsigned long long>(INT64_MAX)) {
          return Json::Value(static_cast<Json::Int64>(value));
        }
        return Json::Value(static_cast<Json::UInt64>(value));
      }
    }
    /* too large for 64 bits: a real number */
    return Json::Value(std::strtod(s.c_str(), nullptr));
  }
  if(IsHex(s)) {
    const unsigned long long value = std::strtoull(s.c_str() + 2, nullptr, 16);
    if(value <= static_cast<unsigned long long>(INT64_MAX)) {
      return Json::Value(static_cast<Json::Int64>(value));
    }
    return Json::Value(static_cast<Json::UInt64>(value));
  }
  if(IsReal(s)) {
    return Json::Value(std::strtod(s.c_str(), nullptr));
  }
  if(s == ".inf" || s == ".Inf" || s == ".INF" || s == "+.inf" ||
     s == "+.Inf" || s == "+.INF") {
    return Json::Value(std::numeric_limits<double>::infinity());
  }
  if(s == "-.inf" || s == "-.Inf" || s == "-.INF") {
    return Json::Value(-std::numeric_limits<double>::infinity());
  }
  if(s == ".nan" || s == ".NaN" || s == ".NAN") {
    return Json::Value(std::numeric_limits<double>::quiet_NaN());
  }
  return Json::Value(s);
}

static Json::Value Convert(const YAML::Node& node, int depth);

/** The text of a key of a mapping. */
static std::string Key(const YAML::Node& key) {
  if(!key.IsScalar()) {
    throw shift::core::Exception("A key of a mapping must be a scalar" +
                                 Where(key));
  }
  return key.Scalar();
}

static void Merge(Json::Value& object, const YAML::Node& source, int depth) {
  if(source.IsMap()) {
    const Json::Value merged = Convert(source, depth + 1);
    for(const auto& name : merged.getMemberNames()) {
      if(!object.isMember(name)) {
        object[name] = merged[name];
      }
    }
  } else if(source.IsSequence()) {
    for(const auto& item : source) {
      Merge(object, item, depth + 1);
    }
  } else {
    throw shift::core::Exception(
      "The value of a merge key '<<' must be a mapping or a sequence of "
      "mappings" +
      Where(source));
  }
}

static Json::Value Convert(const YAML::Node& node, int depth) {
  if(depth > kMaxDepth) {
    throw shift::core::Exception(
      "The YAML document is nested too deeply (or an alias contains itself)" +
      Where(node));
  }
  switch(node.Type()) {
    case YAML::NodeType::Null:
      return Json::Value(Json::nullValue);
    case YAML::NodeType::Scalar: {
      /* a quoted scalar has the tag "!", a plain one "?" */
      const std::string& tag = node.Tag();
      if(tag == "?") {
        return PlainScalar(node.Scalar());
      }
      return Json::Value(node.Scalar());
    }
    case YAML::NodeType::Sequence: {
      Json::Value array(Json::arrayValue);
      for(const auto& item : node) {
        array.append(Convert(item, depth + 1));
      }
      return array;
    }
    case YAML::NodeType::Map: {
      Json::Value object(Json::objectValue);
      std::vector<YAML::Node> merges;
      for(const auto& entry : node) {
        const std::string key = Key(entry.first);
        if(key == "<<" && entry.first.Tag() == "?") {
          merges.push_back(entry.second);
        } else {
          object[key] = Convert(entry.second, depth + 1);
        }
      }
      for(const auto& source : merges) {
        Merge(object, source, depth);
      }
      return object;
    }
    default:
      return Json::Value(Json::nullValue);
  }
}

Json::Value shift::core::parameter::YamlToJson(const std::string& text) {
  try {
    const YAML::Node root = YAML::Load(text);
    return Convert(root, 0);
  } catch(const YAML::Exception& error) {
    throw shift::core::Exception("Invalid YAML at line " +
                                 std::to_string(error.mark.line + 1) +
                                 ", column " +
                                 std::to_string(error.mark.column + 1) + ": " +
                                 error.msg);
  }
}

/** A real number as the shortest text that reads back as the same real. */
static std::string RealText(double value) {
  if(std::isnan(value)) {
    return ".nan";
  }
  if(std::isinf(value)) {
    return value > 0 ? ".inf" : "-.inf";
  }
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  std::string text(buffer, result.ptr);
  /* 1.0 is written 1, which reads as an integer */
  if(IsDigits(text, text[0] == '-' ? 1 : 0)) {
    text += ".0";
  }
  return text;
}

/** A string, quoted when it would read as something else unquoted. */
static void EmitString(YAML::Emitter& out, const std::string& text) {
  if(!PlainScalar(text).isString()) {
    out << YAML::DoubleQuoted;
  }
  out << text;
}

static void EmitJson(YAML::Emitter& out, const Json::Value& value) {
  switch(value.type()) {
    case Json::nullValue:
      out << YAML::Null;
      break;
    case Json::booleanValue:
      out << value.asBool();
      break;
    case Json::intValue:
      out << value.asInt64();
      break;
    case Json::uintValue:
      out << value.asUInt64();
      break;
    case Json::realValue:
      out << RealText(value.asDouble());
      break;
    case Json::stringValue:
      EmitString(out, value.asString());
      break;
    case Json::arrayValue: {
      bool scalars = true;
      for(const Json::Value& element : value) {
        scalars = scalars && !element.isArray() && !element.isObject();
      }
      if(scalars) {
        out << YAML::Flow;
      }
      out << YAML::BeginSeq;
      for(const Json::Value& element : value) {
        EmitJson(out, element);
      }
      out << YAML::EndSeq;
      break;
    }
    case Json::objectValue:
      if(value.empty()) {
        out << YAML::Flow;
      }
      out << YAML::BeginMap;
      for(const std::string& key : value.getMemberNames()) {
        out << YAML::Key;
        EmitString(out, key);
        out << YAML::Value;
        EmitJson(out, value[key]);
      }
      out << YAML::EndMap;
      break;
  }
}

std::string shift::core::parameter::JsonToYaml(const Json::Value& value) {
  YAML::Emitter out;
  EmitJson(out, value);
  return std::string(out.c_str()) + "\n";
}

namespace shift::core::parameter {

/** Writing a YAML document with edits made (EditYaml). */
struct YamlWriting {
  YAML::Emitter out;
  /* the nodes of the document, by the position they are read at, and how
  many times the document names each */
  std::map<int, std::vector<std::pair<YAML::Node, int>>> nodes;
  /* the anchors written, each for a node written whole */
  std::vector<std::pair<YAML::Node, std::string>> anchors;
  std::set<const YamlEdit*> made;

  void Count(const YAML::Node& node) {
    const YAML::Mark mark = node.Mark();
    if(!mark.is_null()) {
      for(auto& [seen, count] : nodes[mark.pos]) {
        if(seen.is(node)) {
          count++;
          return;
        }
      }
      nodes[mark.pos].emplace_back(node, 1);
    }
    if(node.IsMap()) {
      for(const auto& entry : node) {
        Count(entry.second);
      }
    } else if(node.IsSequence()) {
      for(const auto& element : node) {
        Count(element);
      }
    }
  }

  bool Shared(const YAML::Node& node) const {
    const YAML::Mark mark = node.Mark();
    if(mark.is_null() || !nodes.count(mark.pos)) {
      return false;
    }
    for(const auto& [seen, count] : nodes.at(mark.pos)) {
      if(seen.is(node)) {
        return count > 1;
      }
    }
    return false;
  }

  /* the value of key in the entries that the merge keys of map add */
  static YAML::Node Merged(const YAML::Node& map, const std::string& key,
                           int depth) {
    if(depth > kMaxDepth) {
      return YAML::Node(YAML::NodeType::Undefined);
    }
    for(const auto& entry : map) {
      if(!entry.first.IsScalar() || entry.first.Scalar() != "<<" ||
         entry.first.Tag() != "?") {
        continue;
      }
      std::vector<YAML::Node> sources;
      if(entry.second.IsSequence()) {
        for(const auto& source : entry.second) {
          sources.push_back(source);
        }
      } else {
        sources.push_back(entry.second);
      }
      for(const YAML::Node& source : sources) {
        if(!source.IsMap()) {
          continue;
        }
        for(const auto& own : source) {
          if(own.first.IsScalar() && own.first.Scalar() == key) {
            return own.second;
          }
        }
        const YAML::Node found = Merged(source, key, depth + 1);
        if(found.IsDefined()) {
          return found;
        }
      }
    }
    return YAML::Node(YAML::NodeType::Undefined);
  }

  void Scalar(const YAML::Node& node) {
    if(node.Tag() != "?") {
      out << YAML::DoubleQuoted << node.Scalar();
    } else if(PlainScalar(node.Scalar()).isNull()) {
      out << YAML::Null;
    } else {
      out << node.Scalar();
    }
  }

  void Key(const YAML::Node& key) {
    out << YAML::Key;
    if(key.IsScalar()) {
      Scalar(key);
    } else {
      Write(key, {}, 0);
    }
    out << YAML::Value;
  }

  /* node, at depth in the document, with edits, those at or inside it */
  void Write(const YAML::Node& node, const std::vector<const YamlEdit*>& edits,
             Json::ArrayIndex depth) {
    for(const YamlEdit* edit : edits) {
      if(edit->path.size() == depth && !edit->remove) {
        made.insert(edit);
        EmitJson(out, edit->value);
        return;
      }
    }
    if(edits.empty() && Shared(node)) {
      for(const auto& [anchored, name] : anchors) {
        if(anchored.is(node)) {
          out << YAML::Alias(name);
          return;
        }
      }
      const std::string name = "a" + std::to_string(anchors.size() + 1);
      anchors.emplace_back(node, name);
      out << YAML::Anchor(name);
    }
    if(node.IsScalar()) {
      Scalar(node);
    } else if(node.IsSequence()) {
      if(node.Style() == YAML::EmitterStyle::Flow || node.size() == 0) {
        out << YAML::Flow;
      }
      out << YAML::BeginSeq;
      for(std::size_t i = 0; i < node.size(); i++) {
        std::vector<const YamlEdit*> inside;
        bool removed = false;
        for(const YamlEdit* edit : edits) {
          const Json::Value& step = edit->path[depth];
          if(step.isIntegral() && step.asLargestUInt() == i) {
            if(edit->remove && edit->path.size() == depth + 1) {
              made.insert(edit);
              removed = true;
            } else {
              inside.push_back(edit);
            }
          }
        }
        if(!removed) {
          Write(node[i], inside, depth + 1);
        }
      }
      out << YAML::EndSeq;
    } else if(node.IsMap() || (node.IsNull() && !edits.empty())) {
      if(node.Style() == YAML::EmitterStyle::Flow || node.size() == 0) {
        out << YAML::Flow;
      }
      out << YAML::BeginMap;
      std::set<std::string> keys;
      for(const auto& entry : node) {
        const std::string key =
          entry.first.IsScalar() ? entry.first.Scalar() : std::string();
        keys.insert(key);
        std::vector<const YamlEdit*> inside;
        bool removed = false;
        for(const YamlEdit* edit : edits) {
          const Json::Value& step = edit->path[depth];
          if(entry.first.IsScalar() && step.isString() &&
             step.asString() == key) {
            if(edit->remove && edit->path.size() == depth + 1) {
              made.insert(edit);
              removed = true;
            } else {
              inside.push_back(edit);
            }
          }
        }
        if(!removed) {
          Key(entry.first);
          Write(entry.second, inside, depth + 1);
        }
      }
      /* the entries that edits add, or that they make inside entries that
      merge keys add */
      std::vector<std::string> added;
      std::map<std::string, std::vector<const YamlEdit*>> adding;
      for(const YamlEdit* edit : edits) {
        const Json::Value& step = edit->path[depth];
        if(step.isString() && !keys.count(step.asString())) {
          if(!adding.count(step.asString())) {
            added.push_back(step.asString());
          }
          adding[step.asString()].push_back(edit);
        }
      }
      for(const std::string& key : added) {
        const YAML::Node merged = node.IsMap()
                                    ? Merged(node, key, 0)
                                    : YAML::Node(YAML::NodeType::Undefined);
        const std::vector<const YamlEdit*>& inside = adding[key];
        bool set = false;
        for(const YamlEdit* edit : inside) {
          if(edit->path.size() == depth + 1) {
            if(edit->remove) {
              if(merged.IsDefined()) {
                throw shift::core::Exception(
                  "'" + key +
                  "' cannot be removed: a merge key '<<' adds it" +
                  Where(node));
              }
            } else {
              set = true;
            }
          }
        }
        if(set || merged.IsDefined()) {
          out << YAML::Key;
          EmitString(out, key);
          out << YAML::Value;
          Write(set ? YAML::Node() : merged, inside, depth + 1);
        }
      }
      out << YAML::EndMap;
    } else {
      out << YAML::Null;
    }
  }
};

}  // namespace shift::core::parameter

std::string
shift::core::parameter::EditYaml(const std::string& text,
                                 const std::vector<YamlEdit>& edits) {
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch(const YAML::Exception& error) {
    throw shift::core::Exception("Invalid YAML at line " +
                                 std::to_string(error.mark.line + 1) +
                                 ", column " +
                                 std::to_string(error.mark.column + 1) + ": " +
                                 error.msg);
  }
  YamlWriting writing;
  writing.Count(root);
  std::vector<const YamlEdit*> all;
  for(const YamlEdit& edit : edits) {
    all.push_back(&edit);
  }
  writing.Write(root, all, 0);
  if(writing.made.size() != edits.size()) {
    throw shift::core::Exception(
      "the document no longer holds what was read from it");
  }
  if(!writing.out.good()) {
    throw shift::core::Exception("the document could not be written: " +
                                 writing.out.GetLastError());
  }
  /* the comments that open the document, which describe it, kept */
  std::string heading;
  std::istringstream lines(text);
  for(std::string line; std::getline(lines, line);) {
    const std::size_t first = line.find_first_not_of(" \t\r");
    if(first != std::string::npos && line[first] != '#') {
      break;
    }
    heading += line + "\n";
  }
  return heading + writing.out.c_str() + "\n";
}
