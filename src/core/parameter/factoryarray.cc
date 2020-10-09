#include "shift/core/parameter/factoryarray.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/sensor/base.h"

/* define specializations of FactoryArray class */
template class shift::Parameter::FactoryArray<shift::Behavior::Base>;
template class shift::Parameter::FactoryArray<shift::Body::Base>;
template class shift::Parameter::FactoryArray<shift::CollisionGeometry::Base>;
template class shift::Parameter::FactoryArray<shift::Constraint::Base>;
template class shift::Parameter::FactoryArray<shift::ForceModule::Base>;
template class shift::Parameter::FactoryArray<shift::Geometry::Base>;
template class shift::Parameter::FactoryArray<shift::Integrator::Base>;
template class shift::Parameter::FactoryArray<shift::Joint::Base>;
template class shift::Parameter::FactoryArray<shift::Light::Base>;
template class shift::Parameter::FactoryArray<shift::Logger::Base>;
template class shift::Parameter::FactoryArray<shift::Sensor::Base>;
template class shift::Parameter::FactoryArray<shift::Solver::Base>;
template class shift::Parameter::FactoryArray<shift::Terrain::Base>;

template <class T>
void shift::Parameter::FactoryArray<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::Parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void shift::Parameter::FactoryArray<T>::readJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    std::string object_type(T::defaultType());
    if(json_value.isMember("type") && json_value["type"].isString()) {
      object_type.assign(json_value["type"].asString());
    }
    /* if there is no "type" specified in the object and
    there is no default type available, then skip over this object
    since Shift does not know which class specialization to use */
    if(!object_type.empty()) {
      auto object = shift::Factory<T>::get()->create(object_type);
      if(object) {
        try {
          object->readJSON(json_value);
        } catch(const char* msg) {
          std::cerr << T::arrayKey() << std::endl;
          std::cerr << msg << std::endl;
        }
        object_specializations_.push_back(std::move(object));
        objects_.push_back(dynamic_cast<shift::Parameter::List*>(
          object_specializations_.back().get()));
      }
    }
  }
}
