#include "shift-core/factory.h"

/* define factoryMap_ specialization for BehaviorBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::Behavior::BehaviorBase>::CreateFunction>
  Shift::Factory<Shift::Behavior::BehaviorBase>::factoryMap_{};

/* explicit instantiation of BehaviorBase class */
template class Shift::Factory<Shift::Behavior::BehaviorBase>;

/* define factoryMap_ specialization for BodyBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Body::BodyBase>::CreateFunction>
  Shift::Factory<Shift::Body::BodyBase>::factoryMap_{};

/* explicit instantiation of BodyBase class */
template class Shift::Factory<Shift::Body::BodyBase>;

/* define factoryMap_ specialization for CollisionGeometryBase class */
template <>
std::map<std::string,
         Shift::Factory<
           Shift::CollisionGeometry::CollisionGeometryBase>::CreateFunction>
  Shift::Factory<
    Shift::CollisionGeometry::CollisionGeometryBase>::factoryMap_{};

/* explicit instantiation of CollisionGeometryBase class */
template class Shift::Factory<Shift::CollisionGeometry::CollisionGeometryBase>;

/* define factoryMap_ specialization for ConstraintBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::Constraint::ConstraintBase>::CreateFunction>
  Shift::Factory<Shift::Constraint::ConstraintBase>::factoryMap_{};

/* explicit instantiation of ConstraintBase class */
template class Shift::Factory<Shift::Constraint::ConstraintBase>;

/* define factoryMap_ specialization for ForceModuleBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::ForceModule::ForceModuleBase>::CreateFunction>
  Shift::Factory<Shift::ForceModule::ForceModuleBase>::factoryMap_{};

/* explicit instantiation of ForceModuleBase class */
template class Shift::Factory<Shift::ForceModule::ForceModuleBase>;

/* define factoryMap_ specialization for GeometryBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::Geometry::GeometryBase>::CreateFunction>
  Shift::Factory<Shift::Geometry::GeometryBase>::factoryMap_{};

/* explicit instantiation of GeometryBase class */
template class Shift::Factory<Shift::Geometry::GeometryBase>;

/* define factoryMap_ specialization for IntegratorBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::Integrator::IntegratorBase>::CreateFunction>
  Shift::Factory<Shift::Integrator::IntegratorBase>::factoryMap_{};

/* explicit instantiation of IntegratorBase class */
template class Shift::Factory<Shift::Integrator::IntegratorBase>;

/* define factoryMap_ specialization for JointBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::JointBase>::CreateFunction>
  Shift::Factory<Shift::Joint::JointBase>::factoryMap_{};

/* explicit instantiation of JointBase class */
template class Shift::Factory<Shift::Joint::JointBase>;

/* define factoryMap_ specialization for LightBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Light::LightBase>::CreateFunction>
  Shift::Factory<Shift::Light::LightBase>::factoryMap_{};

/* explicit instantiation of LightBase class */
template class Shift::Factory<Shift::Light::LightBase>;

/* define factoryMap_ specialization for LoggerBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::LoggerBase>::CreateFunction>
  Shift::Factory<Shift::Logger::LoggerBase>::factoryMap_{};

/* explicit instantiation of LoggerBase class */
template class Shift::Factory<Shift::Logger::LoggerBase>;

/* define factoryMap_ specialization for SensorBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::SensorBase>::CreateFunction>
  Shift::Factory<Shift::Sensor::SensorBase>::factoryMap_{};

/* explicit instantiation of SensorBase class */
template class Shift::Factory<Shift::Sensor::SensorBase>;

/* define factoryMap_ specialization for SolverBase class */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::SolverBase>::CreateFunction>
  Shift::Factory<Shift::Solver::SolverBase>::factoryMap_{};

/* explicit instantiation of SolverBase class */
template class Shift::Factory<Shift::Solver::SolverBase>;

/* define factoryMap_ specialization for TerrainBase class */
template <>
std::map<std::string,
         Shift::Factory<Shift::Terrain::TerrainBase>::CreateFunction>
  Shift::Factory<Shift::Terrain::TerrainBase>::factoryMap_{};

/* explicit instantiation of TerrainBase class */
template class Shift::Factory<Shift::Terrain::TerrainBase>;
