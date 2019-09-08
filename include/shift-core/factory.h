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
    // int status;
    // std::string type_str = typeid(T).name();
    // char* demangled_name =
    //   abi::__cxa_demangle(type_str.c_str(), nullptr, nullptr, &status);
    // if(status == 0) {
    //   type_str = demangled_name;
    //   std::free(demangled_name);
    //   if(type_str.substr(type_str.length() - 1, type_str.length()) ==
    //      std::string("*")) {
    //     type_str = type_str.substr(0, type_str.length() - 1);
    //   }
    // }
    // std::cout << "~Factory(): " << type_str << std::endl;
    factory_map_.clear();
  }

  typedef std::function<std::unique_ptr<T>()> CreateFunction;

  static Factory<T>* get() {
    static Factory<T> instance;
    return &instance;
  }

  bool registerModule(const std::string& name, CreateFunction pfnCreate) {
    /* register module only if name not already taken */
    if(factory_map_.find(name) == factory_map_.end()) {
      factory_map_[name] = pfnCreate;
      return true;
    }
    return false;
  }

  void listModules() {
    for(const auto& pair : factory_map_) {
      std::cout << pair.first << " ";
    }
    std::cout << std::endl;
  }

  std::unique_ptr<T> create(const std::string& name) {
    /* if valid map does not exist, return null and throw exception */
    try {
      if(factory_map_.find(name) == factory_map_.end()) {
        throw "Error: Module not found:";
      }
    } catch(const char* msg) {
      std::cerr << msg << " " << name << " is not registered." << std::endl;
      return nullptr;
    }
    return factory_map_[name]();
  }

private:
  static std::map<std::string, CreateFunction> factory_map_;
};

/*! indicate that factory_map_ exists for bodies */
template <>
std::map<std::string, Shift::Factory<Shift::Behavior::Base>::CreateFunction>
  Shift::Factory<Shift::Behavior::Base>::factory_map_;

/*! indicate that Behavior specialization exists */
extern template class Shift::Factory<Shift::Behavior::Base>;

/*! indicate that factory_map_ exists for bodies */
template <>
std::map<std::string, Shift::Factory<Shift::Body::Base>::CreateFunction>
  Shift::Factory<Shift::Body::Base>::factory_map_;

/*! indicate that Body specialization exists */
extern template class Shift::Factory<Shift::Body::Base>;

/*! indicate that factory_map_ exists for CollisionGeometry */
template <>
std::map<std::string,
         Shift::Factory<Shift::CollisionGeometry::Base>::CreateFunction>
  Shift::Factory<Shift::CollisionGeometry::Base>::factory_map_;

/*! indicate that CollisionGeometry specialization exists */
extern template class Shift::Factory<Shift::CollisionGeometry::Base>;

/*! indicate that factory_map_ exists for Constraint */
template <>
std::map<std::string, Shift::Factory<Shift::Constraint::Base>::CreateFunction>
  Shift::Factory<Shift::Constraint::Base>::factory_map_;

/*! indicate that Constraint specialization exists */
extern template class Shift::Factory<Shift::Constraint::Base>;

/*! indicate that factory_map_ exists for ForceModule */
template <>
std::map<std::string, Shift::Factory<Shift::ForceModule::Base>::CreateFunction>
  Shift::Factory<Shift::ForceModule::Base>::factory_map_;

/*! indicate that ForceModule specialization exists */
extern template class Shift::Factory<Shift::ForceModule::Base>;

/*! indicate that factory_map_ exists for Geometry */
template <>
std::map<std::string, Shift::Factory<Shift::Geometry::Base>::CreateFunction>
  Shift::Factory<Shift::Geometry::Base>::factory_map_;

/*! indicate that Geometry specialization exists */
extern template class Shift::Factory<Shift::Geometry::Base>;

/*! indicate that factory_map_ exists for Integrator */
template <>
std::map<std::string, Shift::Factory<Shift::Integrator::Base>::CreateFunction>
  Shift::Factory<Shift::Integrator::Base>::factory_map_;

/*! indicate that Integrator specialization exists */
extern template class Shift::Factory<Shift::Integrator::Base>;

/*! indicate that factory_map_ exists for Joint */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::Base>::CreateFunction>
  Shift::Factory<Shift::Joint::Base>::factory_map_;

/*! indicate that Joint specialization exists */
extern template class Shift::Factory<Shift::Joint::Base>;

/*! indicate that factory_map_ exists for Light */
template <>
std::map<std::string, Shift::Factory<Shift::Light::Base>::CreateFunction>
  Shift::Factory<Shift::Light::Base>::factory_map_;

/*! indicate that Light specialization exists */
extern template class Shift::Factory<Shift::Light::Base>;

/*! indicate that factory_map_ exists for Logger */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::Base>::CreateFunction>
  Shift::Factory<Shift::Logger::Base>::factory_map_;

/*! indicate that Logger specialization exists */
extern template class Shift::Factory<Shift::Logger::Base>;

/*! indicate that factory_map_ exists for Sensor */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::Base>::CreateFunction>
  Shift::Factory<Shift::Sensor::Base>::factory_map_;

/*! indicate that Sensor specialization exists */
extern template class Shift::Factory<Shift::Sensor::Base>;

/*! indicate that factory_map_ exists for Solver */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::Base>::CreateFunction>
  Shift::Factory<Shift::Solver::Base>::factory_map_;

/*! indicate that Solver specialization exists */
extern template class Shift::Factory<Shift::Solver::Base>;

/*! indicate that factory_map_ exists for Terrain */
template <>
std::map<std::string, Shift::Factory<Shift::Terrain::Base>::CreateFunction>
  Shift::Factory<Shift::Terrain::Base>::factory_map_;

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
