#include "shift/core/factory.h"

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Behavior::Base>::CreateFunction>
  shift::Factory<shift::Behavior::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Behavior::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Body::Base>::CreateFunction>
  shift::Factory<shift::Body::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Body::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string,
         shift::Factory<shift::CollisionGeometry::Base>::CreateFunction>
  shift::Factory<shift::CollisionGeometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::CollisionGeometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Constraint::Base>::CreateFunction>
  shift::Factory<shift::Constraint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Constraint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::ForceModule::Base>::CreateFunction>
  shift::Factory<shift::ForceModule::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::ForceModule::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Geometry::Base>::CreateFunction>
  shift::Factory<shift::Geometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Geometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Integrator::Base>::CreateFunction>
  shift::Factory<shift::Integrator::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Integrator::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Joint::Base>::CreateFunction>
  shift::Factory<shift::Joint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Joint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Light::Base>::CreateFunction>
  shift::Factory<shift::Light::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Light::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Logger::Base>::CreateFunction>
  shift::Factory<shift::Logger::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Logger::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Sensor::Base>::CreateFunction>
  shift::Factory<shift::Sensor::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Sensor::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Solver::Base>::CreateFunction>
  shift::Factory<shift::Solver::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Solver::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::Factory<shift::Terrain::Base>::CreateFunction>
  shift::Factory<shift::Terrain::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::Factory<shift::Terrain::Base>;
