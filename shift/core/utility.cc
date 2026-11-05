#include "shift/core/utility.h"

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
  return filename.substr(0, filename.find_last_of("/\\"));
}

bool shift::core::utility::FileExists(const std::string& filename) {
  std::ifstream f(filename.c_str());
  return f.good();
}

bool shift::core::utility::IsResourcePath(const std::string& path) {
  return path.rfind("qrc:/", 0) == 0;
}
