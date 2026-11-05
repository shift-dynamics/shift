#ifndef SHIFT_UTILITY_H_
#define SHIFT_UTILITY_H_

#include <string>
#include <time.h>

namespace shift {
namespace core {
namespace utility {

const std::string CurrentDateTime();
const std::string StripFileExtension(const std::string&);
const std::string GetFileExtension(const std::string&);
const std::string GetParentPath(const std::string&);
bool FileExists(const std::string&);
/*! \brief true for a path to a resource built into the viewer,
 * "qrc:/textures/asphalt.jpg"; these are not files, so they are passed on
 * as they are rather than searched for */
bool IsResourcePath(const std::string&);

} // namespace utility
} // namespace core
} // namespace shift

#endif
