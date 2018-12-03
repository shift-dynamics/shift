#ifndef _SHIFT_GEOMETRY_H_
#define _SHIFT_GEOMETRY_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;

namespace Geometry {

/*!
@brief Abstract Geometry base class
*/
class GeometryBase
  : public Base
  , public Serialize {

public:
  GeometryBase(std::string name)
    : Base(name) {
    addKeys();
  }
  GeometryBase()
    : Base("") {
    addKeys();
  }
  ~GeometryBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of Geometry type
};

} // namespace Geometry

} // namespace Shift

#endif
