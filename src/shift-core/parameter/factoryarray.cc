#include "shift-core/parameter/factoryarray.h"

#include "shift-core/assembly.h"
#include "shift-core/context.h"
#include "shift-core/factory.h"
#include "shift-core/frame.h"
#include "shift-core/sensor/base.h"

/* define specializations of FactoryArray class */
template class Shift::Parameter::FactoryArray<Shift::Behavior::Base>;
template class Shift::Parameter::FactoryArray<Shift::Body::Base>;
template class Shift::Parameter::FactoryArray<Shift::CollisionGeometry::Base>;
template class Shift::Parameter::FactoryArray<Shift::Constraint::Base>;
template class Shift::Parameter::FactoryArray<Shift::ForceModule::Base>;
template class Shift::Parameter::FactoryArray<Shift::Geometry::Base>;
template class Shift::Parameter::FactoryArray<Shift::Integrator::Base>;
template class Shift::Parameter::FactoryArray<Shift::Joint::Base>;
template class Shift::Parameter::FactoryArray<Shift::Light::Base>;
template class Shift::Parameter::FactoryArray<Shift::Logger::Base>;
template class Shift::Parameter::FactoryArray<Shift::Sensor::Base>;
template class Shift::Parameter::FactoryArray<Shift::Solver::Base>;
template class Shift::Parameter::FactoryArray<Shift::Terrain::Base>;

template <class T>
void Shift::Parameter::FactoryArray<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void Shift::Parameter::FactoryArray<T>::readJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    std::string object_type(T::defaultType());
    if(json_value.isMember("type") && json_value["type"].isString()) {
      object_type.assign(json_value["type"].asString());
    }
    /* if there is no "type" specified in the object and
    there is no default type available, then skip over this object
    since Shift does not know which class specialization to use */
    if(!object_type.empty()) {
      auto object = Shift::Factory<T>::get()->create(object_type);
      if(object) {
        try {
          object->readJSON(json_value);
        } catch(const char* msg) {
          std::cerr << T::arrayKey() << std::endl;
          std::cerr << msg << std::endl;
        }
        object_specializations_.push_back(std::move(object));
        objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
          object_specializations_.back().get()));
      }
    }
  }
}
