#ifndef SHIFT_FACTORY_
#define SHIFT_FACTORY_

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "shift-core/behavior/base.h"
#include "shift-core/body/base.h"
#include "shift-core/collisiongeometry/base.h"
#include "shift-core/constraint/base.h"
#include "shift-core/forcemodule/base.h"
#include "shift-core/geometry/base.h"
#include "shift-core/integrator/base.h"
#include "shift-core/joint/base.h"
#include "shift-core/light/base.h"
#include "shift-core/logger/base.h"
#include "shift-core/sensor/base.h"
#include "shift-core/solver/base.h"
#include "shift-core/terrain/base.h"

namespace Shift {

template <class T>
class Factory {

public:
  Factory() {}
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<std::unique_ptr<T>()> CreateFunction;

  static Factory<T>* get() {
    static Factory<T> instance;
    return &instance;
  }

  bool registerModule(const std::string& name, CreateFunction pfnCreate) {
    /* register module only if name not already taken */
    if(factoryMap_.find(name) == factoryMap_.end()) {
      factoryMap_[name] = pfnCreate;
      return true;
    }
    return false;
  }

  void listModules() {
    for(const auto& pair : factoryMap_) {
      std::cout << pair.first << " ";
    }
    std::cout << std::endl;
  }

  std::unique_ptr<T> create(const std::string& name) {
    /* if valid map does not exist, return null and throw exception */
    try {
      if(factoryMap_.find(name) == factoryMap_.end()) {
        throw "Error: Module not found:";
      }
    } catch(const char* msg) {
      std::cerr << msg << " " << name << " is not registered." << std::endl;
      return NULL;
    }
    return factoryMap_[name]();
  }

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

/*! indicate that factoryMap_ exists for bodies */
template <>
std::map<std::string, Shift::Factory<Shift::Behavior::Base>::CreateFunction>
  Shift::Factory<Shift::Behavior::Base>::factoryMap_;

/*! indicate that Behavior specialization exists */
extern template class Shift::Factory<Shift::Behavior::Base>;

/*! indicate that factoryMap_ exists for bodies */
template <>
std::map<std::string, Shift::Factory<Shift::Body::Base>::CreateFunction>
  Shift::Factory<Shift::Body::Base>::factoryMap_;

/*! indicate that Body specialization exists */
extern template class Shift::Factory<Shift::Body::Base>;

/*! indicate that factoryMap_ exists for CollisionGeometry */
template <>
std::map<std::string,
         Shift::Factory<Shift::CollisionGeometry::Base>::CreateFunction>
  Shift::Factory<Shift::CollisionGeometry::Base>::factoryMap_;

/*! indicate that CollisionGeometry specialization exists */
extern template class Shift::Factory<Shift::CollisionGeometry::Base>;

/*! indicate that factoryMap_ exists for Constraint */
template <>
std::map<std::string, Shift::Factory<Shift::Constraint::Base>::CreateFunction>
  Shift::Factory<Shift::Constraint::Base>::factoryMap_;

/*! indicate that Constraint specialization exists */
extern template class Shift::Factory<Shift::Constraint::Base>;

/*! indicate that factoryMap_ exists for ForceModule */
template <>
std::map<std::string, Shift::Factory<Shift::ForceModule::Base>::CreateFunction>
  Shift::Factory<Shift::ForceModule::Base>::factoryMap_;

/*! indicate that ForceModule specialization exists */
extern template class Shift::Factory<Shift::ForceModule::Base>;

/*! indicate that factoryMap_ exists for Geometry */
template <>
std::map<std::string, Shift::Factory<Shift::Geometry::Base>::CreateFunction>
  Shift::Factory<Shift::Geometry::Base>::factoryMap_;

/*! indicate that Geometry specialization exists */
extern template class Shift::Factory<Shift::Geometry::Base>;

/*! indicate that factoryMap_ exists for Integrator */
template <>
std::map<std::string, Shift::Factory<Shift::Integrator::Base>::CreateFunction>
  Shift::Factory<Shift::Integrator::Base>::factoryMap_;

/*! indicate that Integrator specialization exists */
extern template class Shift::Factory<Shift::Integrator::Base>;

/*! indicate that factoryMap_ exists for Joint */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::Base>::CreateFunction>
  Shift::Factory<Shift::Joint::Base>::factoryMap_;

/*! indicate that Joint specialization exists */
extern template class Shift::Factory<Shift::Joint::Base>;

/*! indicate that factoryMap_ exists for Light */
template <>
std::map<std::string, Shift::Factory<Shift::Light::Base>::CreateFunction>
  Shift::Factory<Shift::Light::Base>::factoryMap_;

/*! indicate that Light specialization exists */
extern template class Shift::Factory<Shift::Light::Base>;

/*! indicate that factoryMap_ exists for Logger */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::Base>::CreateFunction>
  Shift::Factory<Shift::Logger::Base>::factoryMap_;

/*! indicate that Logger specialization exists */
extern template class Shift::Factory<Shift::Logger::Base>;

/*! indicate that factoryMap_ exists for Sensor */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::Base>::CreateFunction>
  Shift::Factory<Shift::Sensor::Base>::factoryMap_;

/*! indicate that Sensor specialization exists */
extern template class Shift::Factory<Shift::Sensor::Base>;

/*! indicate that factoryMap_ exists for Solver */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::Base>::CreateFunction>
  Shift::Factory<Shift::Solver::Base>::factoryMap_;

/*! indicate that Solver specialization exists */
extern template class Shift::Factory<Shift::Solver::Base>;

/*! indicate that factoryMap_ exists for Terrain */
template <>
std::map<std::string, Shift::Factory<Shift::Terrain::Base>::CreateFunction>
  Shift::Factory<Shift::Terrain::Base>::factoryMap_;

/*! indicate that Terrain specialization exists */
extern template class Shift::Factory<Shift::Terrain::Base>;

template <class T>
class Registration {
public:
  Registration(std::string name, typename Factory<T>::CreateFunction fcn) {
    Factory<T>::get()->registerModule(name, fcn);
  }
};

} // namespace Shift

#endif
