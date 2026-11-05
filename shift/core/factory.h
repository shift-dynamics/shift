#ifndef SHIFT_FACTORY_
#define SHIFT_FACTORY_

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "shift/core/controller/base.h"
#include "shift/core/body/base.h"
#include "shift/core/collision_geometry/base.h"
#include "shift/core/constraint/base.h"
#include "shift/core/force_module/base.h"
#include "shift/core/geometry/base.h"
#include "shift/core/integrator/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/sensor/base.h"
#include "shift/core/solver/base.h"
#include "shift/core/terrain/base.h"
#include "shift/core/exception.h"

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
  /* a member of the function-local instance returned by get(), so it is
     constructed on first use: Registration objects in other translation
     units register modules during static initialization */
  std::map<std::string, CreateFunction> factory_map_;
};

/*! indicate that Controller specialization exists */
extern template class Factory<controller::Base>;

/*! indicate that Body specialization exists */
extern template class Factory<body::Base>;

/*! indicate that CollisionGeometry specialization exists */
extern template class Factory<collision_geometry::Base>;

/*! indicate that Constraint specialization exists */
extern template class Factory<constraint::Base>;

/*! indicate that force_module specialization exists */
extern template class Factory<force_module::Base>;

/*! indicate that Geometry specialization exists */
extern template class Factory<geometry::Base>;

/*! indicate that Integrator specialization exists */
extern template class Factory<integrator::Base>;

/*! indicate that Joint specialization exists */
extern template class Factory<joint::Base>;

/*! indicate that Light specialization exists */
extern template class Factory<light::Base>;

/*! indicate that Logger specialization exists */
extern template class Factory<logger::Base>;

/*! indicate that Sensor specialization exists */
extern template class Factory<sensor::Base>;

/*! indicate that Solver specialization exists */
extern template class Factory<solver::Base>;

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
