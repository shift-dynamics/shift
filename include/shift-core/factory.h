#ifndef _SHIFT_FACTORY_
#define _SHIFT_FACTORY_

#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "shift-core/behaviors/behavior.h"
#include "shift-core/bodies/body.h"
#include "shift-core/collisiongeometry/collisiongeometry.h"
#include "shift-core/constraints/constraint.h"
#include "shift-core/forcemodules/forcemodule.h"
#include "shift-core/geometry/geometry.h"
#include "shift-core/integrators/integrator.h"
#include "shift-core/joints/joint.h"
#include "shift-core/lights/light.h"
#include "shift-core/loggers/logger.h"
#include "shift-core/sensors/sensor.h"
#include "shift-core/solvers/solver.h"
#include "shift-core/terrain/terrain.h"

namespace Shift {

template <class T> class Factory {

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
    if(auto it = factoryMap_.find(name); it == factoryMap_.end()) {
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
      if(auto it = factoryMap_.find(name); it == factoryMap_.end()) {
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
std::map<std::string,
         Shift::Factory<Shift::Behavior::BehaviorBase>::CreateFunction>
  Shift::Factory<Shift::Behavior::BehaviorBase>::factoryMap_;

/*! indicate that Behavior specialization exists */
extern template class Shift::Factory<Shift::Behavior::BehaviorBase>;

/*! indicate that factoryMap_ exists for bodies */
template <>
std::map<std::string, Shift::Factory<Shift::Body::BodyBase>::CreateFunction>
  Shift::Factory<Shift::Body::BodyBase>::factoryMap_;

/*! indicate that Body specialization exists */
extern template class Shift::Factory<Shift::Body::BodyBase>;

/*! indicate that factoryMap_ exists for CollisionGeometry */
template <>
std::map<std::string,
         Shift::Factory<
           Shift::CollisionGeometry::CollisionGeometryBase>::CreateFunction>
  Shift::Factory<Shift::CollisionGeometry::CollisionGeometryBase>::factoryMap_;

/*! indicate that CollisionGeometry specialization exists */
extern template class Shift::Factory<
  Shift::CollisionGeometry::CollisionGeometryBase>;

/*! indicate that factoryMap_ exists for Constraint */
template <>
std::map<std::string,
         Shift::Factory<Shift::Constraint::ConstraintBase>::CreateFunction>
  Shift::Factory<Shift::Constraint::ConstraintBase>::factoryMap_;

/*! indicate that Constraint specialization exists */
extern template class Shift::Factory<Shift::Constraint::ConstraintBase>;

/*! indicate that factoryMap_ exists for ForceModule */
template <>
std::map<std::string,
         Shift::Factory<Shift::ForceModule::ForceModuleBase>::CreateFunction>
  Shift::Factory<Shift::ForceModule::ForceModuleBase>::factoryMap_;

/*! indicate that ForceModule specialization exists */
extern template class Shift::Factory<Shift::ForceModule::ForceModuleBase>;

/*! indicate that factoryMap_ exists for Geometry */
template <>
std::map<std::string,
         Shift::Factory<Shift::Geometry::GeometryBase>::CreateFunction>
  Shift::Factory<Shift::Geometry::GeometryBase>::factoryMap_;

/*! indicate that Geometry specialization exists */
extern template class Shift::Factory<Shift::Geometry::GeometryBase>;

/*! indicate that factoryMap_ exists for Integrator */
template <>
std::map<std::string,
         Shift::Factory<Shift::Integrator::IntegratorBase>::CreateFunction>
  Shift::Factory<Shift::Integrator::IntegratorBase>::factoryMap_;

/*! indicate that Integrator specialization exists */
extern template class Shift::Factory<Shift::Integrator::IntegratorBase>;

/*! indicate that factoryMap_ exists for Joint */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::JointBase>::CreateFunction>
  Shift::Factory<Shift::Joint::JointBase>::factoryMap_;

/*! indicate that Joint specialization exists */
extern template class Shift::Factory<Shift::Joint::JointBase>;

/*! indicate that factoryMap_ exists for Light */
template <>
std::map<std::string, Shift::Factory<Shift::Light::LightBase>::CreateFunction>
  Shift::Factory<Shift::Light::LightBase>::factoryMap_;

/*! indicate that Light specialization exists */
extern template class Shift::Factory<Shift::Light::LightBase>;

/*! indicate that factoryMap_ exists for Logger */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::LoggerBase>::CreateFunction>
  Shift::Factory<Shift::Logger::LoggerBase>::factoryMap_;

/*! indicate that Logger specialization exists */
extern template class Shift::Factory<Shift::Logger::LoggerBase>;

/*! indicate that factoryMap_ exists for Sensor */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::SensorBase>::CreateFunction>
  Shift::Factory<Shift::Sensor::SensorBase>::factoryMap_;

/*! indicate that Sensor specialization exists */
extern template class Shift::Factory<Shift::Sensor::SensorBase>;

/*! indicate that factoryMap_ exists for Solver */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::SolverBase>::CreateFunction>
  Shift::Factory<Shift::Solver::SolverBase>::factoryMap_;

/*! indicate that Solver specialization exists */
extern template class Shift::Factory<Shift::Solver::SolverBase>;

/*! indicate that factoryMap_ exists for Terrain */
template <>
std::map<std::string,
         Shift::Factory<Shift::Terrain::TerrainBase>::CreateFunction>
  Shift::Factory<Shift::Terrain::TerrainBase>::factoryMap_;

/*! indicate that Terrain specialization exists */
extern template class Shift::Factory<Shift::Terrain::TerrainBase>;

template <class T> class Registration {
public:
  Registration(std::string name, typename Factory<T>::CreateFunction fcn) {
    Factory<T>::get()->registerModule(name, fcn);
  }
};

} // namespace Shift

#endif
