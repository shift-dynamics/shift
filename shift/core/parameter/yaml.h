#ifndef SHIFT_CORE_PARAMETER_YAML_H_
#define SHIFT_CORE_PARAMETER_YAML_H_

#include <json/json.h>

#include <string>
#include <vector>

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

/**
 * value written as YAML, read back by YamlToJson as value: objects as block
 * mappings, in the order of their keys, arrays of scalars in flow style
 * ([1, 0, 0]), other arrays as block sequences, and a string quoted when it
 * would read as something else unquoted ("12", "true", "~").
 */
std::string JsonToYaml(const Json::Value& value);

/** a change to a document: the value at path set, or removed */
struct YamlEdit {
  Json::Value path; /* the keys and indices, as YamlToJson reads them */
  bool remove = false;
  Json::Value value;
};

/**
 * text, a YAML document, with edits made, each at a path of the document as
 * YamlToJson reads it. The rest is written as it is in text, in its order,
 * flow or block style, quoted or plain; the values of edits as JsonToYaml
 * writes them. A node that text names more than once (an anchor and its
 * aliases) is written once with an anchor and then as aliases, except where
 * an edit is made inside it: there it is written out with the edit, so the
 * edit is made only where it is. An edit inside the entries that a merge key
 * `<<` adds to a mapping is written as an entry of that mapping of its own.
 * The comments that open text are kept, but not the others, nor the names
 * of anchors.
 *
 * Throws shift::core::Exception for text that is not valid YAML, an edit at a
 * path that text does not hold, and the removal of an entry that a merge key
 * adds.
 */
std::string EditYaml(const std::string& text,
                     const std::vector<YamlEdit>& edits);

}  // namespace shift::core::parameter

#endif  // SHIFT_CORE_PARAMETER_YAML_H_
