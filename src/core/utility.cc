#include "shift/core/utility.h"

#include <fstream>

const std::string shift::core::utility::currentDateTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
  return buf;
}

const std::string
shift::core::utility::stripFileExtension(const std::string& filename) {
  return filename.substr(0, filename.find_last_of("."));
}

const std::string shift::core::utility::getFileExtension(const std::string& filename) {
  return filename.substr(filename.find_last_of("."));
}

const std::string shift::core::utility::getParentPath(const std::string& filename) {
  return filename.substr(0, filename.find_last_of("/\\"));
}

bool shift::core::utility::fileExists(const std::string& filename) {
  std::ifstream f(filename.c_str());
  return f.good();
}
