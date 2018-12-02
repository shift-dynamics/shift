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

class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<GeometryBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& GeometryName,
                             CreateFunction pfnCreate);
  static GeometryBase* create(const std::string& GeometryName);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace Geometry

} // namespace Shift

#endif
