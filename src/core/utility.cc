#include "shift/core/utility.h"

const std::string shift::core::utility::currentDateTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
  return buf;
}

const std::string
shift::core::utility::stripFileExtension(std::string filename) {
  return filename.substr(0, filename.find_last_of("."));
}

const std::string shift::core::utility::getFileExtension(std::string filename) {
  return filename.substr(filename.find_last_of("."));
}
