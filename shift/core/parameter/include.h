#ifndef SHIFT_CORE_PARAMETER_INCLUDE_H_
#define SHIFT_CORE_PARAMETER_INCLUDE_H_

#include <json/json.h>

#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace shift::core::parameter {

/** where a part of a model comes from: a model file, and where in it */
struct Origin {
  std::string file; /* absolute */
  /* where the files that the file names are found, after its directory:
  its own include directories, absolute */
  std::vector<std::string> include_directories;
  Json::Value path = Json::Value(Json::arrayValue); /* keys and indices */
  /* given by an include's override: a value of the file included, written
  in the file that includes it, whose file names that file finds */
  bool patch = false;
};

/**
 * The origins of the parts of a model, its includes expanded, that are not
 * where they are in the model file: the parts that come from an included
 * file, and the values that an include's override gives. Each is kept by
 * the JSON pointer (RFC 6901) of the part in the expanded model; a part
 * without one comes from where its parent does, at its key or index there,
 * and a part none of whose parents has one is where it is in the model file.
 * Kept beside the model rather than in it, so that the model, as printed,
 * logged or saved, holds only what its files hold.
 */
class Origins {
public:
  void Set(const std::string& pointer, Origin origin) {
    entries_[pointer] = std::move(origin);
  }

  /** the origin given for the part at pointer itself, or null */
  const Origin* At(const std::string& pointer) const {
    const auto entry = entries_.find(pointer);
    return entry == entries_.end() ? nullptr : &entry->second;
  }

  /** whether a part strictly inside the part at pointer has an origin */
  bool HasInside(const std::string& pointer) const;

  /** the files that the parts come from, each once */
  std::vector<std::string> Files() const;

  const std::map<std::string, Origin>& get_entries() const {
    return entries_;
  }

  bool empty() const {
    return entries_.empty();
  }

  /** for a log: {pointer: {"file", "include_directories", "path",
   * "patch"}} */
  Json::Value ToJson() const;

  static Origins FromJson(const Json::Value& value);

  /** pointer with step (a key or an index) appended */
  static std::string Append(const std::string& pointer,
                            const Json::Value& step);

private:
  std::map<std::string, Origin> entries_;
};

/**
 * Model files that include other model files.
 *
 * "include" is allowed only in an assembly: a file name, an object
 * {"file": <name>, "override": {...}}, or an array of either. Each file included is a model file: its top level
 * assemblies become subassemblies of the assembly, after those it lists
 * itself; its model_parameters are folded into the model's, and its other
 * top level keys (terrain, surface_materials, scenery, environment, frames,
 * lights, ...) are merged into the top level of the model.
 *
 * Merging, the including file comes first: objects merge key by key, two
 * arrays of objects (terrain, loggers, ...) are joined with the including
 * file's elements first (an element that the same place of the same file
 * gave already is kept once), and anything else (a number, a string, a
 * vector such as gravity, a list of names) is the including file's when it
 * gives one. solvers and integrators are settings, each taken whole: every
 * one listed runs. A warning names each value of an included file's
 * model_parameters that the model's overrides.
 *
 * An override, laid out as the file is, replaces the file's values: objects
 * key by key, arrays of objects element by element by position, anything
 * else whole; it is applied after the file's own includes are expanded.
 * A file included more than once is placed by overriding its frames (and,
 * names being unique, their names and the names that refer to them).
 *
 * A relative name is taken from the directory of the file the include is
 * written in. The files that an included file names (meshes, textures,
 * maps) are found next to it, then in its own include_directories, taken
 * from its directory; they are not added to the model's. Where each part
 * that comes from an included file, or from an override, is written is
 * given in Origins, beside the model, not in it: those parts find their
 * files there (see Context::FindFile), and an edit to one is saved there
 * (ModelWriter).
 */
class Include {
public:
  /**
   * The model file at filename, JSON or YAML (by its extension), with its
   * includes expanded. files, when given, receives the absolute paths of the
   * files included; origins, when given, where the parts of the model that
   * come from them, or from an override, are written.
   *
   * Throws shift::core::Exception for a file that cannot be read or parsed,
   * an include anywhere but in an assembly, an include that is not a file
   * name, an include object or an array of them, an include object with a
   * key it does not take, an included file that does not exist or that
   * includes itself, an included file that does not hold an object, an
   * and an override that does not fit the file.
   */
  static Json::Value ReadFile(const std::string& filename,
                              std::vector<std::string>* files = nullptr,
                              Origins* origins = nullptr);

  /**
   * model, as read from the model file filename, with its includes
   * expanded (see ReadFile).
   */
  static Json::Value Expand(const Json::Value& model,
                            const std::string& filename,
                            std::vector<std::string>* files = nullptr,
                            Origins* origins = nullptr);

  /**
   * the JSON or YAML (by its extension) of file, as written: its includes
   * not expanded
   */
  static Json::Value Parse(const std::filesystem::path& file);

private:
  /* the key that marks an object while the includes are expanded: its
  "origin" and the "members" that come from elsewhere than it */
  static const char* SourceKey() {
    return "$source";
  }

  /* an include: a file, with an override (if given); path is
  where it is written in the file that includes it */
  struct Entry {
    std::string file;
    Json::Value override;
    Json::Value path;
  };

  /* the expansion of one model file and the files it includes */
  struct State {
    std::vector<std::filesystem::path> stack; /* the files being expanded */
    std::vector<std::string>* files = nullptr;
    Json::Value root; /* the top level of the model */
    std::set<std::string> warned;
  };

  /* secondary merged into primary, primary first: objects key by key,
  arrays of objects joined with primary's elements first, anything else
  primary's. origin is where secondary is written, if it is not marked.
  overridden, when given, receives the keys (dotted from the top,
  "units.angle") where primary keeps a value that differs from
  secondary's. */
  static Json::Value Merge(const Json::Value& primary,
                           const Json::Value& secondary,
                           const Json::Value& origin,
                           std::vector<std::string>* overridden = nullptr,
                           const std::string& key = "");

  static std::vector<Entry> Entries(const Json::Value& include,
                                    const std::filesystem::path& file,
                                    const Json::Value& path);

  static std::filesystem::path Open(const std::string& name,
                                    const std::filesystem::path& from,
                                    State& state, Json::Value* contents);

  /* source: the origin (without a path) of the file the assembly is in;
  path: where the assembly is in it */
  static void ExpandAssembly(Json::Value& assembly,
                             const std::filesystem::path& file,
                             const Json::Value& source,
                             const Json::Value& path, State& state);

  static void IncludeModel(const Entry& entry,
                           const std::filesystem::path& from,
                           const Json::Value& from_source,
                           Json::Value& assembly, State& state);

  /* override, laid out as target is, over it, its values marked as given at
  origin */
  static void Override(Json::Value& target, const Json::Value& override,
                       const Json::Value& origin,
                       const std::filesystem::path& file,
                       const std::string& where);

  static void CheckNoInclude(const Json::Value& value,
                             const std::filesystem::path& file);

  static void Mark(Json::Value& value, const Json::Value& source,
                   const Json::Value& path);

  /* origin with step appended to its path */
  static Json::Value At(const Json::Value& origin, const Json::Value& step);

  /* the member of object marked as written at origin */
  static void SetOrigin(Json::Value& object, const std::string& member,
                        const Json::Value& origin);

  static Json::Value Strip(const Json::Value& value);

  static void Extract(Json::Value& value, const std::string& pointer,
                      Origins* origins);

  static Origin ToOrigin(const Json::Value& origin);

  static bool IsObjectArray(const Json::Value& value);
};

}  // namespace shift::core::parameter

#endif  // SHIFT_CORE_PARAMETER_INCLUDE_H_
