#include "shift/core/parameter/Array.h"
#include "shift/core/Assembly.h"

/* define specializations of Array class */
template class shift::core::parameter::Array<shift::core::Assembly>;
template class shift::core::parameter::Array<shift::core::Frame>;

template <class T>
void shift::core::parameter::Array<T>::readJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    auto object = std::make_unique<T>();
    if(object) {
      object->readJSON(json_value);
      object->setIsBody(T::arrayKey() == "bodies");
      object->setIsAssembly(T::arrayKey() == "assemblies");
      object_specializations_.push_back(std::move(object));
      objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
        object_specializations_.back().get()));
    }
  }
}

template <class T>
void shift::core::parameter::Array<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
    object_specializations_.back().get()));
}
