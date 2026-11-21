#include "shift/core/parameter/include.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>

#include "shift/core/exception.h"
#include "shift/core/log.h"
#include "shift/core/parameter/yaml.h"

bool shift::core::parameter::Origins::HasInside(
  const std::string& pointer) const {
  const std::string prefix = pointer + "/";
  const auto entry = entries_.lower_bound(prefix);
  return entry != entries_.end() &&
         entry->first.compare(0, prefix.size(), prefix) == 0;
}

std::vector<std::string> shift::core::parameter::Origins::Files() const {
  std::vector<std::string> files;
  for(const auto& [pointer, origin] : entries_) {
    if(std::find(files.begin(), files.end(), origin.file) == files.end()) {
      files.push_back(origin.file);
    }
  }
  return files;
}

Json::Value shift::core::parameter::Origins::ToJson() const {
  Json::Value value(Json::objectValue);
  for(const auto& [pointer, origin] : entries_) {
    Json::Value& entry = value[pointer];
    entry["file"] = origin.file;
    entry["include_directories"] = Json::Value(Json::arrayValue);
    for(const std::string& directory : origin.include_directories) {
      entry["include_directories"].append(directory);
    }
    entry["path"] = origin.path;
    if(origin.patch) {
      entry["patch"] = true;
    }
  }
  return value;
}

shift::core::parameter::Origins
shift::core::parameter::Origins::FromJson(const Json::Value& value) {
  Origins origins;
  if(!value.isObject()) {
    return origins;
  }
  for(const std::string& pointer : value.getMemberNames()) {
    const Json::Value& entry = value[pointer];
    Origin origin;
    origin.file = entry["file"].asString();
    for(const Json::Value& directory : entry["include_directories"]) {
      origin.include_directories.push_back(directory.asString());
    }
    if(entry["path"].isArray()) {
      origin.path = entry["path"];
    }
    origin.patch = entry["patch"].asBool();
    origins.Set(pointer, std::move(origin));
  }
  return origins;
}

std::string
shift::core::parameter::Origins::Append(const std::string& pointer,
                                        const Json::Value& step) {
  if(step.isIntegral()) {
    return pointer + "/" + std::to_string(step.asLargestUInt());
  }
  /* "~" and "/" in a key are written "~0" and "~1" */
  std::string escaped;
  for(char c : step.asString()) {
    if(c == '~') {
      escaped += "~0";
    } else if(c == '/') {
      escaped += "~1";
    } else {
      escaped += c;
    }
  }
  return pointer + "/" + escaped;
}


static Json::Value Steps(Json::Value path, const Json::Value& step) {
  path.append(step);
  return path;
}

Json::Value shift::core::parameter::Include::ReadFile(
  const std::string& filename, std::vector<std::string>* files,
  Origins* origins) {
  return Expand(Parse(filename), filename, files, origins);
}

Json::Value shift::core::parameter::Include::Expand(
  const Json::Value& model, const std::string& filename,
  std::vector<std::string>* files, Origins* origins) {
  const std::filesystem::path file =
    std::filesystem::absolute(filename).lexically_normal();
  if(!model.isObject()) {
    return model;
  }
  State state;
  state.stack.push_back(file);
  state.files = files;
  state.root = model;
  for(const std::string& key : model.getMemberNames()) {
    if(key == "include") {
      throw shift::core::Exception(
        "\"include\" at the top level of '" + file.string() +
        "': a model file is included by an assembly");
    }
    if(key != "assemblies") {
      CheckNoInclude(model[key], file);
    }
  }
  if(model.isMember("assemblies")) {
    Json::Value assemblies = model["assemblies"];
    if(assemblies.isArray()) {
      /* the model file's own: found as the model's files are */
      Json::Value source(Json::objectValue);
      source["file"] = file.string();
      source["include_directories"] = Json::Value(Json::arrayValue);
      for(Json::ArrayIndex i = 0; i < assemblies.size(); i++) {
        ExpandAssembly(assemblies[i], file, source,
                       Steps(Steps(Json::Value(Json::arrayValue),
                                   "assemblies"),
                             i),
                       state);
      }
    } else {
      CheckNoInclude(assemblies, file);
    }
    state.root["assemblies"] = assemblies;
  }
  /* the marks, made while expanding, kept beside the model instead */
  Extract(state.root, "", origins);
  return state.root;
}

void shift::core::parameter::Include::Extract(Json::Value& value,
                                              const std::string& pointer,
                                              Origins* origins) {
  if(value.isObject()) {
    if(value.isMember(SourceKey())) {
      const Json::Value marker = value[SourceKey()];
      value.removeMember(SourceKey());
      if(origins) {
        if(marker.isMember("origin")) {
          origins->Set(pointer, ToOrigin(marker["origin"]));
        }
        /* the values of the object that come from elsewhere than it */
        const Json::Value& members = marker["members"];
        for(const std::string& member : members.getMemberNames()) {
          origins->Set(Origins::Append(pointer, member),
                       ToOrigin(members[member]));
        }
      }
    }
    for(const std::string& key : value.getMemberNames()) {
      Extract(value[key], Origins::Append(pointer, key), origins);
    }
  } else if(value.isArray()) {
    for(Json::ArrayIndex i = 0; i < value.size(); i++) {
      Extract(value[i], Origins::Append(pointer, i), origins);
    }
  }
}

shift::core::parameter::Origin
shift::core::parameter::Include::ToOrigin(const Json::Value& origin) {
  Origin result;
  result.file = origin["file"].asString();
  for(const Json::Value& directory : origin["include_directories"]) {
    result.include_directories.push_back(directory.asString());
  }
  if(origin["path"].isArray()) {
    result.path = origin["path"];
  }
  result.patch = origin["patch"].asBool();
  return result;
}

Json::Value
shift::core::parameter::Include::At(const Json::Value& origin,
                                    const Json::Value& step) {
  if(origin.isNull()) {
    return origin;
  }
  Json::Value at = origin;
  at["path"].append(step);
  return at;
}

void shift::core::parameter::Include::SetOrigin(Json::Value& object,
                                                const std::string& member,
                                                const Json::Value& origin) {
  if(origin.isNull()) {
    return;
  }
  Json::Value& value = object[member];
  if(value.isObject()) {
    Json::Value& marker = value[SourceKey()];
    if(!marker.isMember("origin")) {
      marker["origin"] = origin;
    }
    return;
  }
  object[SourceKey()]["members"][member] = origin;
}

Json::Value shift::core::parameter::Include::Merge(
  const Json::Value& primary, const Json::Value& secondary,
  const Json::Value& origin, std::vector<std::string>* overridden,
  const std::string& key) {
  if(primary.isObject() && secondary.isObject()) {
    Json::Value merged = primary;
    const Json::Value& marker = secondary[SourceKey()];
    const Json::Value& from =
      marker.isMember("origin") ? marker["origin"] : origin;
    for(const std::string& member : secondary.getMemberNames()) {
      if(member == SourceKey()) {
        continue; /* the object stays primary's */
      }
      /* where the member of secondary was written */
      const Json::Value at = marker["members"].isMember(member)
                               ? marker["members"][member]
                               : At(from, member);
      const std::string dotted = key.empty() ? member : key + "." + member;
      if(primary.isMember(member) && !primary[member].isNull()) {
        merged[member] =
          Merge(primary[member], secondary[member], at, overridden, dotted);
      } else {
        merged[member] = secondary[member];
        SetOrigin(merged, member, at);
      }
    }
    return merged;
  }
  /* the solver and the integrator are settings, each given whole: every
  one listed runs */
  const bool setting = key == "solvers" || key == "integrators";
  if(!setting && IsObjectArray(primary) && IsObjectArray(secondary)) {
    Json::Value joined = primary;
    for(Json::ArrayIndex i = 0; i < secondary.size(); i++) {
      Json::Value element = secondary[i];
      Json::Value& marker = element[SourceKey()];
      if(!marker.isMember("origin") && !origin.isNull()) {
        marker["origin"] = At(origin, i);
      }
      /* one that the same place of the same file gave already (two
      includes of the file) is there once */
      bool repeated = false;
      for(const Json::Value& kept : primary) {
        if(!marker.isMember("origin")) {
          break;
        }
        repeated = repeated ||
                   (kept[SourceKey()]["origin"]["file"] ==
                      marker["origin"]["file"] &&
                    kept[SourceKey()]["origin"]["path"] ==
                      marker["origin"]["path"] &&
                    Strip(kept) == Strip(element));
      }
      if(marker.empty()) {
        element.removeMember(SourceKey());
      }
      if(!repeated) {
        joined.append(element);
      }
    }
    return joined;
  }
  if(primary.isNull()) {
    return secondary;
  }
  if(overridden && !secondary.isNull() && Strip(primary) != Strip(secondary)) {
    overridden->push_back(key);
  }
  return primary;
}

std::vector<shift::core::parameter::Include::Entry>
shift::core::parameter::Include::Entries(const Json::Value& include,
                                         const std::filesystem::path& file,
                                         const Json::Value& path) {
  const std::string usage =
    "\"include\" in '" + file.string() +
    "' must be a file name, an object {\"file\": <name>, \"override\": "
    "{...}, \"T\": [...], ...}, or an array of them";
  std::vector<Entry> entries;
  auto add = [&](const Json::Value& given, const Json::Value& at) {
    Entry entry;
    entry.path = at;
    if(given.isString()) {
      entry.file = given.asString();
    } else if(given.isObject()) {
      if(!given["file"].isString()) {
        throw shift::core::Exception(usage + ": \"file\" is missing");
      }
      entry.file = given["file"].asString();
      for(const std::string& key : given.getMemberNames()) {
        if(key == "file") {
          continue;
        }
        if(key == "override") {
          if(!given[key].isObject()) {
            throw shift::core::Exception(
              "The override of '" + entry.file + "' in '" + file.string() +
              "' must be an object, laid out as the file is");
          }
          entry.override = given[key];
        } else {
          throw shift::core::Exception(
            usage + ": '" + key +
            "' is not one of its keys (file and override)");
        }
      }
    } else {
      throw shift::core::Exception(usage);
    }
    entries.push_back(entry);
  };
  if(include.isArray()) {
    for(Json::ArrayIndex i = 0; i < include.size(); i++) {
      add(include[i], Steps(path, i));
    }
  } else {
    add(include, path);
  }
  if(entries.empty()) {
    throw shift::core::Exception(usage);
  }
  return entries;
}

Json::Value
shift::core::parameter::Include::Parse(const std::filesystem::path& file) {
  std::ifstream in(file);
  if(!in) {
    throw shift::core::Exception("The file '" + file.string() +
                                 "' could not be opened");
  }
  std::string extension = file.extension().string();
  for(char& c : extension) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if(extension == ".yaml" || extension == ".yml") {
    /* the same structure as JSON, in YAML */
    std::stringstream text;
    text << in.rdbuf();
    try {
      return shift::core::parameter::YamlToJson(text.str());
    } catch(const std::exception& error) {
      throw shift::core::Exception("'" + file.string() + "': " + error.what());
    }
  }
  Json::CharReaderBuilder builder;
  Json::Value value;
  std::string errors;
  if(!Json::parseFromStream(builder, in, &value, &errors)) {
    throw shift::core::Exception("The file '" + file.string() +
                                 "' is not valid JSON: " + errors);
  }
  return value;
}

std::filesystem::path shift::core::parameter::Include::Open(
  const std::string& name, const std::filesystem::path& from, State& state,
  Json::Value* contents) {
  std::filesystem::path file(name);
  if(file.is_relative()) {
    file = from.parent_path() / file;
  }
  file = std::filesystem::absolute(file).lexically_normal();
  std::error_code error;
  if(!std::filesystem::is_regular_file(file, error)) {
    throw shift::core::Exception("The file '" + name + "', included in '" +
                                 from.string() + "', does not exist");
  }
  if(std::find(state.stack.begin(), state.stack.end(), file) !=
     state.stack.end()) {
    throw shift::core::Exception("'" + file.string() + "', included in '" +
                                 from.string() + "', includes itself");
  }
  *contents = Parse(file);
  if(!contents->isObject()) {
    throw shift::core::Exception("'" + file.string() + "', included in '" +
                                 from.string() + "', must hold an object");
  }
  if(state.files &&
     std::find(state.files->begin(), state.files->end(), file.string()) ==
       state.files->end()) {
    state.files->push_back(file.string());
  }
  return file;
}


void shift::core::parameter::Include::ExpandAssembly(
  Json::Value& assembly, const std::filesystem::path& file,
  const Json::Value& source, const Json::Value& path, State& state) {
  if(!assembly.isObject()) {
    CheckNoInclude(assembly, file);
    return;
  }
  for(const std::string& key : assembly.getMemberNames()) {
    if(key != "include" && key != "assemblies") {
      CheckNoInclude(assembly[key], file);
    }
  }
  if(assembly.isMember("assemblies")) {
    Json::Value& subassemblies = assembly["assemblies"];
    if(subassemblies.isArray()) {
      for(Json::ArrayIndex j = 0; j < subassemblies.size(); j++) {
        ExpandAssembly(subassemblies[j], file, source,
                       Steps(Steps(path, "assemblies"), j), state);
      }
    } else {
      CheckNoInclude(subassemblies, file);
    }
  }
  if(!assembly.isMember("include")) {
    return;
  }
  const std::vector<Entry> entries =
    Entries(assembly["include"], file, Steps(path, "include"));
  assembly.removeMember("include");
  for(const Entry& entry : entries) {
    IncludeModel(entry, file, source, assembly, state);
  }
}

void shift::core::parameter::Include::IncludeModel(
  const Entry& entry, const std::filesystem::path& from,
  const Json::Value& from_source, Json::Value& assembly, State& state) {
  Json::Value contents;
  const std::filesystem::path file = Open(entry.file, from, state, &contents);
  if(contents.isMember("include")) {
    throw shift::core::Exception(
      "\"include\" at the top level of '" + file.string() +
      "': a model file is included by an assembly");
  }
  for(const std::string& key : contents.getMemberNames()) {
    if(key != "assemblies") {
      CheckNoInclude(contents[key], file);
    }
  }
  /* where the files it names are found: next to it, then in its own
  include directories, taken from its directory */
  const std::filesystem::path directory = file.parent_path();
  Json::Value source(Json::objectValue);
  source["file"] = file.string();
  source["include_directories"] = Json::Value(Json::arrayValue);
  Json::Value& parameters = contents["model_parameters"];
  if(parameters.isMember("include_directories")) {
    const Json::Value& directories = parameters["include_directories"];
    if(!directories.isArray()) {
      throw shift::core::Exception("include_directories of '" + file.string() +
                                   "' must be an array of directory names");
    }
    for(const Json::Value& given : directories) {
      if(!given.isString()) {
        throw shift::core::Exception("include_directories of '" +
                                     file.string() +
                                     "' must be an array of directory names");
      }
      std::filesystem::path name(given.asString());
      if(name.is_relative()) {
        name = directory / name;
      }
      source["include_directories"].append(name.lexically_normal().string());
    }
    parameters.removeMember("include_directories");
  }
  if(parameters.isNull()) {
    contents.removeMember("model_parameters");
  }
  state.stack.push_back(file);
  /* its assemblies, with their own includes expanded */
  if(contents.isMember("assemblies")) {
    Json::Value& assemblies = contents["assemblies"];
    if(!assemblies.isArray()) {
      throw shift::core::Exception("The assemblies of '" + file.string() +
                                   "' must be an array");
    }
    for(Json::ArrayIndex i = 0; i < assemblies.size(); i++) {
      ExpandAssembly(assemblies[i], file, source,
                     Steps(Steps(Json::Value(Json::arrayValue), "assemblies"),
                           i),
                     state);
    }
  }
  /* everything in it marked as written there */
  for(const std::string& key : contents.getMemberNames()) {
    Mark(contents[key], source, Steps(Json::Value(Json::arrayValue), key));
  }
  /* the include's override, laid out as the file is, over it */
  if(!entry.override.isNull()) {
    if(entry.override["model_parameters"].isMember("include_directories")) {
      throw shift::core::Exception(
        "The override of '" + entry.file + "' in '" + from.string() +
        "' cannot give include_directories: they are the file's own");
    }
    Json::Value origin(Json::objectValue);
    origin["file"] = from.string();
    origin["include_directories"] = Json::Value(Json::arrayValue);
    origin["path"] = Steps(entry.path, "override");
    origin["patch"] = true;
    Override(contents, entry.override, origin, file, "");
    /* an array the override gives whole at the top level: its objects are
    the override's */
    const Json::Value members = contents[SourceKey()]["members"];
    for(const std::string& member : members.getMemberNames()) {
      Json::Value& value = contents[member];
      for(Json::ArrayIndex i = 0; value.isArray() && i < value.size(); i++) {
        if(value[i].isObject()) {
          value[i][SourceKey()]["origin"] = At(members[member], i);
        }
      }
    }
  }
  contents.removeMember(SourceKey());
  /* its model_parameters, folded into the model's: the model's own, and
  those folded in before, win where both give a value */
  if(contents.isMember("model_parameters")) {
    const Json::Value given = contents["model_parameters"];
    std::vector<std::string> overridden;
    /* the model's own, even when it gives none */
    const Json::Value model = state.root["model_parameters"].isObject()
                                ? state.root["model_parameters"]
                                : Json::Value(Json::objectValue);
    state.root["model_parameters"] =
      Merge(model, given, Json::Value(), &overridden);
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    writer["precision"] = 15; /* 0.03, not 0.029999999999999999 */
    for(const std::string& key : overridden) {
      const Json::Value* kept = &model;
      const Json::Value* value = &given;
      std::string::size_type begin = 0;
      while(begin <= key.size()) {
        const std::string::size_type end =
          std::min(key.find('.', begin), key.size());
        const std::string member = key.substr(begin, end - begin);
        kept = &(*kept)[member];
        value = &(*value)[member];
        begin = end + 1;
      }
      const std::string warning =
        "model_parameters: " + key + " of '" + file.string() + "' (" +
        Json::writeString(writer, Strip(*value)) +
        ") is overridden by the model's (" +
        Json::writeString(writer, Strip(*kept)) + ")";
      /* once, for a file included more than once */
      if(state.warned.insert(warning).second) {
        SHIFT_WARNING(warning);
      }
    }
  }
  /* its assemblies, subassemblies of the one that includes it */
  if(contents.isMember("assemblies")) {
    Json::Value& subassemblies = assembly["assemblies"];
    if(!subassemblies.isNull() && !subassemblies.isArray()) {
      throw shift::core::Exception("The assemblies of an assembly in '" +
                                   from.string() + "' must be an array");
    }
    for(Json::Value& subassembly : contents["assemblies"]) {
      subassemblies.append(subassembly);
    }
  }
  /* the rest of its top level, merged into the model's */
  for(const std::string& key : contents.getMemberNames()) {
    if(key == "model_parameters" || key == "assemblies") {
      continue;
    }
    const Json::Value& value = contents[key];
    if(state.root.isMember(key) && !state.root[key].isNull()) {
      state.root[key] = Merge(state.root[key], value, Json::Value());
    } else {
      state.root[key] = value;
    }
  }
  state.stack.pop_back();
}

void shift::core::parameter::Include::Override(
  Json::Value& target, const Json::Value& override, const Json::Value& origin,
  const std::filesystem::path& file, const std::string& where) {
  for(const std::string& member : override.getMemberNames()) {
    const std::string at = where.empty() ? member : where + "." + member;
    const Json::Value& given = override[member];
    const Json::Value given_origin = At(origin, member);
    auto mismatch = [&](const std::string& what) {
      return shift::core::Exception("The override of '" + file.string() +
                                    "' at " + at + ": " + what);
    };
    if(!target.isMember(member) || target[member].isNull()) {
      target[member] = given;
      SetOrigin(target, member, given_origin);
      continue;
    }
    Json::Value& value = target[member];
    if(value.isObject() != given.isObject() ||
       value.isArray() != given.isArray()) {
      throw mismatch("it gives " +
                     std::string(given.isObject()  ? "an object"
                                 : given.isArray() ? "an array"
                                                   : "a value") +
                     " where the file has " +
                     std::string(value.isObject()  ? "an object"
                                 : value.isArray() ? "an array"
                                                   : "a value"));
    }
    if(value.isObject()) {
      Override(value, given, given_origin, file, at);
    } else if(value.isArray() && !value.empty() && IsObjectArray(value)) {
      /* element by element, by position; {} leaves one as it is */
      if(given.size() > value.size()) {
        throw mismatch("it gives " + std::to_string(given.size()) +
                       " elements where the file has " +
                       std::to_string(value.size()));
      }
      for(Json::ArrayIndex i = 0; i < given.size(); i++) {
        if(!given[i].isObject()) {
          throw mismatch("element " + std::to_string(i) +
                         " must be an object, as the file's is");
        }
        Override(value[i], given[i], At(given_origin, i), file,
                 at + "[" + std::to_string(i) + "]");
      }
    } else {
      value = given;
      SetOrigin(target, member, given_origin);
    }
  }
}

void shift::core::parameter::Include::CheckNoInclude(
  const Json::Value& value, const std::filesystem::path& file) {
  if(value.isObject()) {
    if(value.isMember("include")) {
      throw shift::core::Exception(
        "\"include\" in '" + file.string() +
        "' is allowed only in an assembly");
    }
    for(const Json::Value& member : value) {
      CheckNoInclude(member, file);
    }
  } else if(value.isArray()) {
    for(const Json::Value& element : value) {
      CheckNoInclude(element, file);
    }
  }
}

void shift::core::parameter::Include::Mark(Json::Value& value,
                                           const Json::Value& source,
                                           const Json::Value& path) {
  if(value.isObject()) {
    Json::Value& marker = value[SourceKey()];
    if(!marker.isMember("origin")) {
      Json::Value origin = source;
      origin["path"] = path;
      marker["origin"] = origin;
    }
  } else if(value.isArray()) {
    for(Json::ArrayIndex i = 0; i < value.size(); i++) {
      Mark(value[i], source, Steps(path, i));
    }
  }
}

Json::Value
shift::core::parameter::Include::Strip(const Json::Value& value) {
  if(value.isObject()) {
    Json::Value stripped(Json::objectValue);
    for(const std::string& key : value.getMemberNames()) {
      if(key != SourceKey()) {
        stripped[key] = Strip(value[key]);
      }
    }
    return stripped;
  }
  if(value.isArray()) {
    Json::Value stripped(Json::arrayValue);
    for(const Json::Value& element : value) {
      stripped.append(Strip(element));
    }
    return stripped;
  }
  return value;
}

bool shift::core::parameter::Include::IsObjectArray(const Json::Value& value) {
  if(!value.isArray()) {
    return false;
  }
  for(const Json::Value& element : value) {
    if(!element.isObject()) {
      return false;
    }
  }
  return true;
}
