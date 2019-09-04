#include "shift-core/factory.h"

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Behavior::Base>::CreateFunction>
  Shift::Factory<Shift::Behavior::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Behavior::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Body::Base>::CreateFunction>
  Shift::Factory<Shift::Body::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Body::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string,
         Shift::Factory<Shift::CollisionGeometry::Base>::CreateFunction>
  Shift::Factory<Shift::CollisionGeometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::CollisionGeometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Constraint::Base>::CreateFunction>
  Shift::Factory<Shift::Constraint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Constraint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::ForceModule::Base>::CreateFunction>
  Shift::Factory<Shift::ForceModule::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::ForceModule::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Geometry::Base>::CreateFunction>
  Shift::Factory<Shift::Geometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Geometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Integrator::Base>::CreateFunction>
  Shift::Factory<Shift::Integrator::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Integrator::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Joint::Base>::CreateFunction>
  Shift::Factory<Shift::Joint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Joint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Light::Base>::CreateFunction>
  Shift::Factory<Shift::Light::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Light::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Logger::Base>::CreateFunction>
  Shift::Factory<Shift::Logger::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Logger::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Sensor::Base>::CreateFunction>
  Shift::Factory<Shift::Sensor::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Sensor::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Solver::Base>::CreateFunction>
  Shift::Factory<Shift::Solver::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Solver::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, Shift::Factory<Shift::Terrain::Base>::CreateFunction>
  Shift::Factory<Shift::Terrain::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class Shift::Factory<Shift::Terrain::Base>;
