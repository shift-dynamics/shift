#ifndef _SHIFT_COLLISIONGEOMETRY_H_
#define _SHIFT_COLLISIONGEOMETRY_H_

#include <functional>
#include <map>
#include <string>

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

class Body;
class Frame;

namespace CollisionGeometry {

/*!
@brief Abstract CollisionGeometry base class
*/
class CollisionGeometryBase
  : public Base
  , public Serialize {

public:
  CollisionGeometryBase(std::string name)
    : Base(name) {
    addKeys();
  }
  CollisionGeometryBase()
    : Base("") {
    addKeys();
  }
  ~CollisionGeometryBase() {}

  const std::string& getType() {
    return type_;
  }

protected:
  void addKeys();

  std::string type_; //!< descriptive name of CollisionGeometry type
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

  typedef std::function<CollisionGeometryBase*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& CollisionGeometryName,
                             CreateFunction pfnCreate);
  static CollisionGeometryBase* create(const std::string& name);

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

class Registration {
public:
  Registration(std::string name, Factory::CreateFunction fcn) {
    Factory::registerModule(name, fcn);
  }
};

} // namespace CollisionGeometry

} // namespace Shift

#endif
