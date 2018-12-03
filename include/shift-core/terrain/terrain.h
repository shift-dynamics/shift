#ifndef _SHIFT_TERRAIN_H_
#define _SHIFT_TERRAIN_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

namespace Body {
class BodyBase;
}

class Frame;

namespace Terrain {

/*!
@brief Abstract TerrainBase base class
*/
class TerrainBase
  : public Base
  , public Serialize {

public:
  TerrainBase(std::string name)
    : Base(name) {
    addKeys();
  }
  TerrainBase()
    : Base("") {
    addKeys();
  }
  ~TerrainBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of TerrainBase type
};

} // namespace Terrain

} // namespace Shift

#endif
