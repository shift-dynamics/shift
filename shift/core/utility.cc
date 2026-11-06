#include "shift/core/utility.h"

#include <filesystem>
#include <fstream>

const std::string shift::core::utility::CurrentDateTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);
  return buf;
}

const std::string
shift::core::utility::StripFileExtension(const std::string& filename) {
  return filename.substr(0, filename.find_last_of("."));
}

const std::string shift::core::utility::GetFileExtension(const std::string& filename) {
  return filename.substr(filename.find_last_of("."));
}

const std::string shift::core::utility::GetParentPath(const std::string& filename) {
  const size_t separator = filename.find_last_of("/\\");
  return separator == std::string::npos ? "." : filename.substr(0, separator);
}

bool shift::core::utility::FileExists(const std::string& filename) {
  std::ifstream f(filename.c_str());
  return f.good();
}

std::string shift::core::utility::FindFile(
  const std::string& name, const std::string& model_filename,
  const std::vector<std::string>& include_directories) {
  if(std::filesystem::path(name).is_absolute()) {
    return FileExists(name) ? name : "";
  }
  const std::string parent = GetParentPath(model_filename);
  const std::string file = parent + "/" + name;
  if(FileExists(file)) {
    return file;
  }
  for(const auto& directory : include_directories) {
    const std::string candidate =
      (std::filesystem::path(directory).is_absolute()
         ? directory
         : parent + "/" + directory) +
      "/" + name;
    if(FileExists(candidate)) {
      return candidate;
    }
  }
  return "";
}

bool shift::core::utility::IsResourcePath(const std::string& path) {
  return path.rfind("qrc:/", 0) == 0;
}
