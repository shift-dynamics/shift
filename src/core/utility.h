#ifndef SHIFT_UTILITY_H_
#define SHIFT_UTILITY_H_

#include <string>
#include <time.h>

namespace shift {
namespace core {
namespace utility {

const std::string currentDateTime();
const std::string stripFileExtension(const std::string&);
const std::string getFileExtension(const std::string&);
const std::string getParentPath(const std::string&);
bool fileExists(const std::string&);

} // namespace utility
} // namespace core
} // namespace shift

#endif
