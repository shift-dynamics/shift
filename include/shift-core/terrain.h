#ifndef _SHIFT_TERRAIN_H_
#define _SHIFT_TERRAIN_H_

#include <string>
#include <vector>

#include "base.h"
#include "body.h"

namespace Shift {

/*!
@brief Shift Terrain datastructure
*/
class Terrain : Base {

public:
  Terrain(std::string name)
    : Base(name) {}
  ~Terrain() {}

private:
};

} // namespace Shift

#endif
