#ifndef SHIFT_FACTORY_
#define SHIFT_FACTORY_

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "shift/core/behavior/Base.h"
#include "shift/core/body/Base.h"
#include "shift/core/collisiongeometry/Base.h"
#include "shift/core/constraint/Base.h"
#include "shift/core/forcemodule/Base.h"
#include "shift/core/geometry/Base.h"
#include "shift/core/integrator/Base.h"
#include "shift/core/joint/Base.h"
#include "shift/core/light/Base.h"
#include "shift/core/logger/Base.h"
#include "shift/core/sensor/Base.h"
#include "shift/core/solver/Base.h"
#include "shift/core/terrain/Base.h"
#include "shift/core/Exception.h"

namespace shift {
namespace core {

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

  bool RegisterModule(const std::string& name, CreateFunction pfnCreate) {
    /* register module only if name not already taken */
    if(factory_map_.find(name) == factory_map_.end()) {
      factory_map_[name] = pfnCreate;
      return true;
    }
    return false;
  }

  void ListModules() {
    for(const auto& pair : factory_map_) {
      std::cout << pair.first << " ";
    }
    std::cout << std::endl;
  }

  std::unique_ptr<T> Create(const std::string& name) {
    /* if valid map does not exist, return null and throw exception */
    try {
      if(factory_map_.find(name) == factory_map_.end()) {
        throw "Error: module not found:";
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
std::map<std::string, Factory<behavior::Base>::CreateFunction>
  Factory<behavior::Base>::factory_map_;

/*! indicate that Behavior specialization exists */
extern template class Factory<behavior::Base>;

/*! indicate that factory_map_ exists for bodies */
template <>
std::map<std::string, Factory<body::Base>::CreateFunction>
  Factory<body::Base>::factory_map_;

/*! indicate that Body specialization exists */
extern template class Factory<body::Base>;

/*! indicate that factory_map_ exists for CollisionGeometry */
template <>
std::map<std::string,
         Factory<collisiongeometry::Base>::CreateFunction>
  Factory<collisiongeometry::Base>::factory_map_;

/*! indicate that CollisionGeometry specialization exists */
extern template class Factory<collisiongeometry::Base>;

/*! indicate that factory_map_ exists for Constraint */
template <>
std::map<std::string, Factory<constraint::Base>::CreateFunction>
  Factory<constraint::Base>::factory_map_;

/*! indicate that Constraint specialization exists */
extern template class Factory<constraint::Base>;

/*! indicate that factory_map_ exists for forcemodule */
template <>
std::map<std::string, Factory<forcemodule::Base>::CreateFunction>
  Factory<forcemodule::Base>::factory_map_;

/*! indicate that forcemodule specialization exists */
extern template class Factory<forcemodule::Base>;

/*! indicate that factory_map_ exists for Geometry */
template <>
std::map<std::string, Factory<geometry::Base>::CreateFunction>
  Factory<geometry::Base>::factory_map_;

/*! indicate that Geometry specialization exists */
extern template class Factory<geometry::Base>;

/*! indicate that factory_map_ exists for Integrator */
template <>
std::map<std::string, Factory<integrator::Base>::CreateFunction>
  Factory<integrator::Base>::factory_map_;

/*! indicate that Integrator specialization exists */
extern template class Factory<integrator::Base>;

/*! indicate that factory_map_ exists for Joint */
template <>
std::map<std::string, Factory<joint::Base>::CreateFunction>
  Factory<joint::Base>::factory_map_;

/*! indicate that Joint specialization exists */
extern template class Factory<joint::Base>;

/*! indicate that factory_map_ exists for Light */
template <>
std::map<std::string, Factory<light::Base>::CreateFunction>
  Factory<light::Base>::factory_map_;

/*! indicate that Light specialization exists */
extern template class Factory<light::Base>;

/*! indicate that factory_map_ exists for Logger */
template <>
std::map<std::string, Factory<logger::Base>::CreateFunction>
  Factory<logger::Base>::factory_map_;

/*! indicate that Logger specialization exists */
extern template class Factory<logger::Base>;

/*! indicate that factory_map_ exists for Sensor */
template <>
std::map<std::string, Factory<sensor::Base>::CreateFunction>
  Factory<sensor::Base>::factory_map_;

/*! indicate that Sensor specialization exists */
extern template class Factory<sensor::Base>;

/*! indicate that factory_map_ exists for Solver */
template <>
std::map<std::string, Factory<solver::Base>::CreateFunction>
  Factory<solver::Base>::factory_map_;

/*! indicate that Solver specialization exists */
extern template class Factory<solver::Base>;

/*! indicate that factory_map_ exists for Terrain */
template <>
std::map<std::string, Factory<terrain::Base>::CreateFunction>
  Factory<terrain::Base>::factory_map_;

/*! indicate that Terrain specialization exists */
extern template class Factory<terrain::Base>;

template <class T>
class Registration {
public:
  Registration(std::string name, typename Factory<T>::CreateFunction fcn) {
    Factory<T>::get()->RegisterModule(name, fcn);
  }
};
}
} // namespace shift

#endif
