#ifndef SHIFT_CORE_PARAMETER_MODEL_WRITER_H_
#define SHIFT_CORE_PARAMETER_MODEL_WRITER_H_

#include <json/json.h>

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "shift/core/parameter/include.h"

namespace shift::core::parameter {

/**
 * Writing a model, as edited, back to model files.
 *
 * A model is read with its includes expanded (parameter::Include), with
 * Origins that give where each part that comes from an included file, or
 * from an include's override, is written. SaveInPlace writes each edit to
 * the file it belongs to, keeping the includes; SaveFlattened writes the
 * whole model as one file, without includes, with copies of the files it
 * names.
 */
class ModelWriter {
public:
  /**
   * Write the edits that make edited of loaded, both the model of the model
   * file filename with its includes expanded (with origins), to the files
   * they belong to: an edit to a value from an included file to that file,
   * to a value an override gives to the override, any other (the model's own
   * parameters among them) to filename. Only the files edited are
   * written, each as it is with the edits made, keeping its includes, in its
   * own language: a JSON file with its comments, its keys in alphabetical
   * order; a YAML file in its order and styles, without its comments (see
   * EditYaml). Returns the paths of the files written.
   *
   * Throws shift::core::Exception for a value replaced whole (an array whose
   * length changed) that holds parts from included files or overrides, an edit that a file no longer holds
   * the place of, the removal of an entry that a YAML merge key adds, and a
   * file that cannot be read or written; then no file has been written.
   */
  static std::vector<std::string> SaveInPlace(const Json::Value& loaded,
                                              const Json::Value& edited,
                                              const std::string& filename,
                                              const Origins& origins);

  /**
   * Write edited, the model of the model file filename with its includes
   * expanded (with origins), to the model file output, as one file without
   * includes, in YAML if output ends in .yaml or .yml and otherwise in JSON.
   * The files it names are copied into the directory <output's stem>_files
   * beside output, as they lay relative to one another, so that two files of
   * the same name stay apart and a mesh finds its materials and textures
   * beside it; the names in output name the copies.
   *
   * found gives the path at which each name was found, by the file that
   * names it ({included model file or empty for filename, name} -> path, as
   * Context::get_files_found); files the files read (Context::get_files_read),
   * among them those that the files named name in turn (an .mtl). Returns the
   * paths of the files written, output first.
   *
   * Throws shift::core::Exception for a file that cannot be copied or
   * written.
   */
  static std::vector<std::string> SaveFlattened(
    const Json::Value& edited, const std::string& filename,
    const Origins& origins,
    const std::map<std::pair<std::string, std::string>, std::string>& found,
    const std::vector<std::string>& files, const std::string& output);

private:
  /* where a value is: in a file, at the keys and indices of path; and its
  JSON pointer in the expanded model */
  struct Location {
    std::string file;
    Json::Value path; /* an array */
    std::string pointer;
  };

  struct Edit {
    Location location;
    bool remove = false;
    Json::Value value;
  };

  static Location Locate(const Location& parent, const Json::Value& step,
                         const Origins& origins);

  static void Diff(const Json::Value& loaded, const Json::Value& edited,
                   const Location& location, const Origins& origins,
                   std::vector<Edit>& edits);

  static void Apply(Json::Value& document, const Edit& edit);

  static bool IsYaml(const std::filesystem::path& file);

  static std::string ReadText(const std::filesystem::path& file);

  static std::string JsonText(const Json::Value& value);

  static void WriteText(const std::string& text,
                        const std::filesystem::path& file);

  static Json::Value Relocate(
    const Json::Value& value, const std::string& pointer,
    const std::string& source, const std::string& model,
    const Origins& origins,
    const std::map<std::pair<std::string, std::string>, std::string>& found,
    const std::map<std::string, std::string>& copies);
};

}  // namespace shift::core::parameter

#endif  // SHIFT_CORE_PARAMETER_MODEL_WRITER_H_
