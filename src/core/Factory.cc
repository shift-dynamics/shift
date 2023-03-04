#include "shift/core/Factory.h"

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::behavior::Base>::CreateFunction>
  shift::core::Factory<shift::core::behavior::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::behavior::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::body::Base>::CreateFunction>
  shift::core::Factory<shift::core::body::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::body::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string,
         shift::core::Factory<shift::core::collision_geometry::Base>::CreateFunction>
  shift::core::Factory<shift::core::collision_geometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::collision_geometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::constraint::Base>::CreateFunction>
  shift::core::Factory<shift::core::constraint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::constraint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::force_module::Base>::CreateFunction>
  shift::core::Factory<shift::core::force_module::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::force_module::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::geometry::Base>::CreateFunction>
  shift::core::Factory<shift::core::geometry::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::geometry::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::integrator::Base>::CreateFunction>
  shift::core::Factory<shift::core::integrator::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::integrator::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::joint::Base>::CreateFunction>
  shift::core::Factory<shift::core::joint::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::joint::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::light::Base>::CreateFunction>
  shift::core::Factory<shift::core::light::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::light::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::logger::Base>::CreateFunction>
  shift::core::Factory<shift::core::logger::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::logger::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::sensor::Base>::CreateFunction>
  shift::core::Factory<shift::core::sensor::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::sensor::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::solver::Base>::CreateFunction>
  shift::core::Factory<shift::core::solver::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::solver::Base>;

/* define factory_map_ specialization for Base class */
template <>
std::map<std::string, shift::core::Factory<shift::core::terrain::Base>::CreateFunction>
  shift::core::Factory<shift::core::terrain::Base>::factory_map_{};

/* explicit instantiation of Base class */
template class shift::core::Factory<shift::core::terrain::Base>;
