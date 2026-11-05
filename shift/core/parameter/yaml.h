#ifndef SHIFT_CORE_PARAMETER_YAML_H_
#define SHIFT_CORE_PARAMETER_YAML_H_

#include <json/json.h>

#include <string>

namespace shift::core::parameter {

/**
 * Read a document in YAML as the JSON value with the same structure, so a
 * model file has the same organization in either language.
 *
 * Mappings become objects (their keys are text), sequences become arrays, and
 * a plain (unquoted) scalar becomes null, a boolean, an integer, a real number
 * or a string, as the core schema of YAML 1.2 reads it: `true`, `false`,
 * `null` or `~`, `12`, `-3`, `0x1F`, `1.5`, `1e-3`, `.inf` and `.nan`. A quoted
 * scalar is always a string. Anchors and aliases are resolved, and the merge
 * key `<<` adds the entries of a mapping (or a sequence of them) to the
 * mapping it is in, which keeps its own entries over the merged ones.
 *
 * Throws shift::core::Exception, with the line, for a document that is not
 * valid YAML or that a JSON value cannot hold (a key that is not a scalar).
 */
Json::Value YamlToJson(const std::string& text);

}  // namespace shift::core::parameter

#endif  // SHIFT_CORE_PARAMETER_YAML_H_
