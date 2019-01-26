#include "shift-core/factory.h"

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Behavior::Base>::CreateFunction>
  Shift::Factory<Shift::Behavior::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Behavior::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Body::Base>::CreateFunction>
  Shift::Factory<Shift::Body::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Body::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string,
         Shift::Factory<Shift::CollisionGeometry::Base>::CreateFunction>
  Shift::Factory<Shift::CollisionGeometry::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::CollisionGeometry::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Constraint::Base>::CreateFunction>
  Shift::Factory<Shift::Constraint::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Constraint::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::ForceModule::Base>::CreateFunction>
  Shift::Factory<Shift::ForceModule::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::ForceModule::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Geometry::Base>::CreateFunction>
  Shift::Factory<Shift::Geometry::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Geometry::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Integrator::Base>::CreateFunction>
  Shift::Factory<Shift::Integrator::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Integrator::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::Base>::CreateFunction>
  Shift::Factory<Shift::Joint::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Joint::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Light::Base>::CreateFunction>
  Shift::Factory<Shift::Light::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Light::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::Base>::CreateFunction>
  Shift::Factory<Shift::Logger::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Logger::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::Base>::CreateFunction>
  Shift::Factory<Shift::Sensor::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Sensor::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::Base>::CreateFunction>
  Shift::Factory<Shift::Solver::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Solver::Base>;

/* define factoryMap_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Terrain::Base>::CreateFunction>
  Shift::Factory<Shift::Terrain::Base>::factoryMap_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Terrain::Base>;
