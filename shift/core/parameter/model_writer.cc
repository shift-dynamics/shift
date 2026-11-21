#include "shift/core/parameter/model_writer.h"

#include <cctype>
#include <fstream>
#include <set>
#include <sstream>

#include "shift/core/exception.h"
#include "shift/core/parameter/include.h"
#include "shift/core/parameter/yaml.h"
#include "shift/core/utility.h"

std::vector<std::string> shift::core::parameter::ModelWriter::SaveInPlace(
  const Json::Value& loaded, const Json::Value& edited,
  const std::string& filename, const Origins& origins) {
  const Location top{
    std::filesystem::absolute(filename).lexically_normal().string(),
    Json::Value(Json::arrayValue), ""};
  std::vector<Edit> edits;
  Diff(loaded, edited, top, origins, edits);
  /* the edits to each file, in the order the files are first edited */
  std::map<std::string, std::vector<const Edit*>> by_file;
  std::vector<std::string> order;
  for(const Edit& edit : edits) {
    if(!by_file.count(edit.location.file)) {
      order.push_back(edit.location.file);
    }
    by_file[edit.location.file].push_back(&edit);
  }
  /* each file as it is to be written: all are made before any is written */
  std::vector<std::string> texts;
  for(const std::string& file : order) {
    if(IsYaml(file)) {
      std::vector<YamlEdit> yaml_edits;
      for(const Edit* edit : by_file[file]) {
        yaml_edits.push_back(
          {edit->location.path, edit->remove, edit->value});
      }
      try {
        texts.push_back(EditYaml(ReadText(file), yaml_edits));
      } catch(const shift::core::Exception& error) {
        throw shift::core::Exception("The edits cannot be saved: '" + file +
                                     "': " + error.what());
      }
    } else {
      Json::Value document = Include::Parse(file);
      for(const Edit* edit : by_file[file]) {
        Apply(document, *edit);
      }
      texts.push_back(JsonText(document));
    }
  }
  for(std::size_t i = 0; i < order.size(); i++) {
    WriteText(texts[i], order[i]);
  }
  return order;
}

std::vector<std::string> shift::core::parameter::ModelWriter::SaveFlattened(
  const Json::Value& edited, const std::string& filename,
  const Origins& origins,
  const std::map<std::pair<std::string, std::string>, std::string>& found,
  const std::vector<std::string>& files, const std::string& output) {
  const std::filesystem::path out =
    std::filesystem::absolute(output).lexically_normal();
  const std::filesystem::path directory =
    out.parent_path() / (out.stem().string() + "_files");
  /* the model files are written into output, not copied */
  const std::string model =
    std::filesystem::absolute(filename).lexically_normal().string();
  std::set<std::string> models{model};
  for(const std::string& file : origins.Files()) {
    models.insert(file);
  }
  std::vector<std::filesystem::path> assets;
  for(const std::string& file : files) {
    std::error_code error;
    if(!models.count(file) && !shift::core::utility::IsResourcePath(file) &&
       std::filesystem::is_regular_file(file, error)) {
      assets.push_back(std::filesystem::path(file).lexically_normal());
    }
  }
  /* copied as they lay below the deepest directory that holds them all */
  std::filesystem::path common;
  for(const auto& asset : assets) {
    const std::filesystem::path parent = asset.parent_path();
    if(common.empty()) {
      common = parent;
      continue;
    }
    std::filesystem::path shared;
    auto a = common.begin();
    auto b = parent.begin();
    for(; a != common.end() && b != parent.end() && *a == *b; ++a, ++b) {
      shared /= *a;
    }
    common = shared;
  }
  std::vector<std::string> written{out.string()};
  std::map<std::string, std::string> copies; /* path -> name in output */
  for(const auto& asset : assets) {
    const std::filesystem::path copy =
      (directory / asset.lexically_relative(common)).lexically_normal();
    std::error_code error;
    std::filesystem::create_directories(copy.parent_path(), error);
    if(copy != asset &&
       !std::filesystem::copy_file(
         asset, copy, std::filesystem::copy_options::overwrite_existing,
         error)) {
      throw shift::core::Exception("Could not copy '" + asset.string() +
                                   "' to '" + copy.string() +
                                   "': " + error.message());
    }
    copies[asset.string()] =
      copy.lexically_relative(out.parent_path()).generic_string();
    written.push_back(copy.string());
  }
  Json::Value flat = Relocate(edited, "", "", model, origins, found, copies);
  /* every file it names is now named from output's directory */
  if(flat["model_parameters"].isObject()) {
    flat["model_parameters"].removeMember("include_directories");
  }
  WriteText(IsYaml(out) ? JsonToYaml(flat) : JsonText(flat), out);
  return written;
}

shift::core::parameter::ModelWriter::Location
shift::core::parameter::ModelWriter::Locate(const Location& parent,
                                            const Json::Value& step,
                                            const Origins& origins) {
  Location location = parent;
  location.pointer = Origins::Append(parent.pointer, step);
  if(const Origin* origin = origins.At(location.pointer)) {
    location.file = origin->file;
    location.path = origin->path;
  } else {
    location.path.append(step);
  }
  return location;
}

void shift::core::parameter::ModelWriter::Diff(const Json::Value& loaded,
                                               const Json::Value& edited,
                                               const Location& location,
                                               const Origins& origins,
                                               std::vector<Edit>& edits) {
  if(loaded.isObject() && edited.isObject()) {
    std::set<std::string> keys;
    for(const std::string& key : loaded.getMemberNames()) {
      keys.insert(key);
    }
    for(const std::string& key : edited.getMemberNames()) {
      keys.insert(key);
    }
    for(const std::string& key : keys) {
      const Location at = Locate(location, key, origins);
      if(!edited.isMember(key)) {
        Edit edit;
        edit.location = at;
        edit.remove = true;
        edits.push_back(edit);
      } else if(!loaded.isMember(key)) {
        Edit edit;
        edit.location = at;
        edit.value = edited[key];
        edits.push_back(edit);
      } else {
        Diff(loaded[key], edited[key], at, origins, edits);
      }
    }
    return;
  }
  if(loaded.isArray() && edited.isArray() && loaded.size() == edited.size()) {
    for(Json::ArrayIndex i = 0; i < edited.size(); i++) {
      Diff(loaded[i], edited[i], Locate(location, i, origins), origins,
           edits);
    }
    return;
  }
  if(loaded == edited) {
    return;
  }
  if(origins.HasInside(location.pointer)) {
    throw shift::core::Exception(
      "The edits cannot be saved: a value that holds parts of included "
      "files, or of an include's override, was replaced whole (" +
      location.pointer + ")");
  }
  Edit edit;
  edit.location = location;
  edit.value = edited;
  edits.push_back(edit);
}

void shift::core::parameter::ModelWriter::Apply(Json::Value& document,
                                                const Edit& edit) {
  const Json::Value& path = edit.location.path;
  auto mismatch = [&]() {
    return shift::core::Exception(
      "The edits cannot be saved: '" + edit.location.file +
      "' no longer holds what was read from it");
  };
  if(path.empty()) {
    if(!edit.remove) {
      document = edit.value;
    }
    return;
  }
  Json::Value* node = &document;
  for(Json::ArrayIndex k = 0; k + 1 < path.size(); k++) {
    if(path[k].isString()) {
      if(!node->isNull() && !node->isObject()) {
        throw mismatch();
      }
      node = &(*node)[path[k].asString()];
    } else {
      const Json::ArrayIndex index = path[k].asUInt();
      if(!node->isArray() || index >= node->size()) {
        throw mismatch();
      }
      node = &(*node)[index];
    }
  }
  const Json::Value& last = path[path.size() - 1];
  if(last.isString()) {
    if(!node->isNull() && !node->isObject()) {
      throw mismatch();
    }
    if(edit.remove) {
      node->removeMember(last.asString());
    } else {
      (*node)[last.asString()] = edit.value;
    }
    return;
  }
  const Json::ArrayIndex index = last.asUInt();
  if(!node->isArray() || index >= node->size()) {
    throw mismatch();
  }
  if(edit.remove) {
    Json::Value removed;
    node->removeIndex(index, &removed);
  } else {
    (*node)[index] = edit.value;
  }
}

bool shift::core::parameter::ModelWriter::IsYaml(
  const std::filesystem::path& file) {
  std::string extension = file.extension().string();
  for(char& c : extension) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return extension == ".yaml" || extension == ".yml";
}

std::string shift::core::parameter::ModelWriter::ReadText(
  const std::filesystem::path& file) {
  std::ifstream in(file);
  if(!in) {
    throw shift::core::Exception("The file '" + file.string() +
                                 "' could not be opened");
  }
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

std::string
shift::core::parameter::ModelWriter::JsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  return Json::writeString(builder, value) + "\n";
}

void shift::core::parameter::ModelWriter::WriteText(
  const std::string& text, const std::filesystem::path& file) {
  std::ofstream out(file);
  out << text;
  if(!out) {
    throw shift::core::Exception("Could not write '" + file.string() + "'");
  }
}

Json::Value shift::core::parameter::ModelWriter::Relocate(
  const Json::Value& value, const std::string& pointer,
  const std::string& source, const std::string& model,
  const Origins& origins,
  const std::map<std::pair<std::string, std::string>, std::string>& found,
  const std::map<std::string, std::string>& copies) {
  /* the file that names what is here: found by it, or, for the model file,
  by the empty name (Context::get_files_found) */
  std::string from = source;
  const Origin* origin = origins.At(pointer);
  /* an override's values are found as those of the file they patch */
  if(origin && !origin->patch) {
    from = origin->file == model ? std::string() : origin->file;
  }
  if(value.isObject()) {
    Json::Value relocated(Json::objectValue);
    for(const std::string& member : value.getMemberNames()) {
      relocated[member] =
        Relocate(value[member], Origins::Append(pointer, member), from, model,
                 origins, found, copies);
    }
    return relocated;
  }
  if(value.isArray()) {
    Json::Value relocated(Json::arrayValue);
    for(Json::ArrayIndex i = 0; i < value.size(); i++) {
      relocated.append(Relocate(value[i], Origins::Append(pointer, i), from,
                                model, origins, found, copies));
    }
    return relocated;
  }
  if(value.isString()) {
    /* a name the model found a file by, from the file that gives it */
    const auto file = found.find({from, value.asString()});
    if(file != found.end()) {
      const auto copy = copies.find(file->second);
      if(copy != copies.end()) {
        return copy->second;
      }
    }
  }
  return value;
}
