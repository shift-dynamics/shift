#ifndef SHIFT_UTILITY_H_
#define SHIFT_UTILITY_H_

#include <string>
#include <vector>
#include <time.h>

namespace shift {
namespace core {
namespace utility {

const std::string CurrentDateTime();
const std::string StripFileExtension(const std::string&);
const std::string GetFileExtension(const std::string&);
/*! the directory of a file, "." for a bare file name */
const std::string GetParentPath(const std::string&);
bool FileExists(const std::string&);
/*! \brief the path of a file named by a model, empty if it is not found
 *
 * An absolute name stands as it is. A relative name is looked for next to the
 * model file, then in each include directory in turn. A relative include
 * directory is taken from the directory of the model file, never from the
 * directory the process was started in.
 */
std::string FindFile(const std::string& name, const std::string& model_filename,
                     const std::vector<std::string>& include_directories);
/*! \brief true for a path to a resource built into the viewer,
 * "qrc:/textures/asphalt.jpg"; these are not files, so they are passed on
 * as they are rather than searched for */
bool IsResourcePath(const std::string&);

} // namespace utility
} // namespace core
} // namespace shift

#endif
