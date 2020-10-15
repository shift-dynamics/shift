#include "shift/core/parameter/factoryarray.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/sensor/base.h"

/* define specializations of FactoryArray class */
template class shift::core::parameter::FactoryArray<shift::core::behavior::Base>;
template class shift::core::parameter::FactoryArray<shift::core::body::Base>;
template class shift::core::parameter::FactoryArray<shift::core::collisiongeometry::Base>;
template class shift::core::parameter::FactoryArray<shift::core::constraint::Base>;
template class shift::core::parameter::FactoryArray<shift::core::forcemodule::Base>;
template class shift::core::parameter::FactoryArray<shift::core::geometry::Base>;
template class shift::core::parameter::FactoryArray<shift::core::integrator::Base>;
template class shift::core::parameter::FactoryArray<shift::core::joint::Base>;
template class shift::core::parameter::FactoryArray<shift::core::light::Base>;
template class shift::core::parameter::FactoryArray<shift::core::logger::Base>;
template class shift::core::parameter::FactoryArray<shift::core::sensor::Base>;
template class shift::core::parameter::FactoryArray<shift::core::solver::Base>;
template class shift::core::parameter::FactoryArray<shift::core::terrain::Base>;

template <class T>
void shift::core::parameter::FactoryArray<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void shift::core::parameter::FactoryArray<T>::readJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    std::string object_type(T::defaultType());
    if(json_value.isMember("type") && json_value["type"].isString()) {
      object_type.assign(json_value["type"].asString());
    }
    /* if there is no "type" specified in the object and
    there is no default type available, then skip over this object
    since Shift does not know which class specialization to use */
    if(!object_type.empty()) {
      auto object = shift::core::Factory<T>::get()->create(object_type);
      if(object) {
        try {
          object->readJSON(json_value);
        } catch(const char* msg) {
          std::cerr << T::arrayKey() << std::endl;
          std::cerr << msg << std::endl;
        }
        object_specializations_.push_back(std::move(object));
        objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
          object_specializations_.back().get()));
      }
    }
  }
}
