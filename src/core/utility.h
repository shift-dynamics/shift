#ifndef SHIFT_UTILITY_H_
#define SHIFT_UTILITY_H_

#include <string>
#include <time.h>

namespace shift {
namespace core {
namespace utility {

const std::string currentDateTime();
const std::string stripFileExtension(std::string);
const std::string getFileExtension(std::string);

} // namespace utility
} // namespace core
} // namespace shift

#endif
