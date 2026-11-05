#include "shift/core/parameter/factory_array.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/frame.h"
#include "shift/core/sensor/base.h"

/* define specializations of FactoryArray class */
template class shift::core::parameter::FactoryArray<
  shift::core::controller::Base>;
template class shift::core::parameter::FactoryArray<shift::core::body::Base>;
template class shift::core::parameter::FactoryArray<
  shift::core::collision_geometry::Base>;
template class shift::core::parameter::FactoryArray<
  shift::core::constraint::Base>;
template class shift::core::parameter::FactoryArray<
  shift::core::force_module::Base>;
template class shift::core::parameter::FactoryArray<
  shift::core::geometry::Base>;
template class shift::core::parameter::FactoryArray<
  shift::core::integrator::Base>;
template class shift::core::parameter::FactoryArray<shift::core::joint::Base>;
template class shift::core::parameter::FactoryArray<shift::core::light::Base>;
template class shift::core::parameter::FactoryArray<shift::core::logger::Base>;
template class shift::core::parameter::FactoryArray<shift::core::sensor::Base>;
template class shift::core::parameter::FactoryArray<shift::core::solver::Base>;
template class shift::core::parameter::FactoryArray<shift::core::terrain::Base>;

template <class T>
void shift::core::parameter::FactoryArray<T>::PushBack(
  std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
    object_specializations_.back().get()));
}

template <class T>
void shift::core::parameter::FactoryArray<T>::ReadJSON(
  Json::Value& json_array) {
  for(auto& json_value : json_array) {
    std::string object_type(T::get_default_type());
    if(json_value.isMember("type") && json_value["type"].isString()) {
      object_type.assign(json_value["type"].asString());
    }
    /* if there is no "type" specified in the object and
    there is no default type available, then skip over this object
    since Shift does not know which class specialization to use */
    if(!object_type.empty()) {
      auto object = shift::core::Factory<T>::get()->Create(object_type);
      if(object) {
        object->ReadJSON(json_value);
        object->set_is_body(T::get_array_key() == "bodies");
        object->set_is_assembly(T::get_array_key() == "assemblies");
        object_specializations_.push_back(std::move(object));
        objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
          object_specializations_.back().get()));
      } else {
        throw shift::core::Exception("Unable to create object with type '" +
                                     object_type + "'");
      }
    }
  }
}

template 
void shift::core::parameter::FactoryArray<shift::core::solver::Base>::PushBack(
  std::unique_ptr<shift::core::solver::Base> object);

template 
void shift::core::parameter::FactoryArray<shift::core::solver::Base>::ReadJSON(
  Json::Value& json_array);

template
void shift::core::parameter::FactoryArray<shift::core::integrator::Base>::PushBack(
  std::unique_ptr<shift::core::integrator::Base> object);

template 
void shift::core::parameter::FactoryArray<shift::core::integrator::Base>::ReadJSON(
  Json::Value& json_array);

template 
void shift::core::parameter::FactoryArray<shift::core::logger::Base>::PushBack(
  std::unique_ptr<shift::core::logger::Base> object);

template 
void shift::core::parameter::FactoryArray<shift::core::logger::Base>::ReadJSON(
  Json::Value& json_array);



template void shift::core::parameter::FactoryArray<
  shift::core::collision_geometry::Base>::
  PushBack(std::unique_ptr<shift::core::collision_geometry::Base> object);
