#ifndef _SHIFT_GEOMETRY_H_
#define _SHIFT_GEOMETRY_H_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Frame;
class Context;

namespace Geometry {

/*!
@brief Abstract Geometry base class
*/
class GeometryBase
  : public Base
  , public Serialize {

public:
  GeometryBase(std::string name)
    : Base(name) {}
  GeometryBase()
    : Base("") {}
  ~GeometryBase() {}

  const std::string& getType() {
    return type_;
  }
  void setContext(Context* context) {
    context_ = context;
  }

  virtual void load();
  virtual void connect();
  virtual void readJSON(Json::Value&);

protected:
  Context* context_;
  virtual void addKeys();
  std::string type_; //!< descriptive name of Geometry type
};

class Modules {
public:
  static bool initialized();
};

} // namespace Geometry

} // namespace Shift

#endif
