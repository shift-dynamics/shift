#ifndef SHIFT_UTILITY_H_
#define SHIFT_UTILITY_H_

#include <string>
#include <time.h>

namespace shift {
namespace Utility {

const std::string currentDateTime() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);
  return buf;
}

const std::string stripFileExtension(std::string filename) {
  return filename.substr(0, filename.find_last_of("."));
}

const std::string getFileExtension(std::string filename) {
  return filename.substr(filename.find_last_of("."));
}

} // namespace Utility
} // namespace shift

#endif
