#include "shift/core/parameter/yaml.h"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <limits>
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
