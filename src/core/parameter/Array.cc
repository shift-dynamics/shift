#include "shift/core/parameter/Array.h"
#include "shift/core/Assembly.h"

/* define specializations of Array class */
template class shift::core::parameter::Array<shift::core::Assembly>;
template class shift::core::parameter::Array<shift::core::Frame>;

template <class T>
void shift::core::parameter::Array<T>::ReadJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    auto object = std::make_unique<T>();
    if(object) {
      object->ReadJSON(json_value);
      object->set_is_body(T::get_array_key() == "bodies");
      object->set_is_assembly(T::get_array_key() == "assemblies");
      object_specializations_.push_back(std::move(object));
      objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
        object_specializations_.back().get()));
    }
  }
}

template <class T>
void shift::core::parameter::Array<T>::PushBack(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<shift::core::parameter::List*>(
    object_specializations_.back().get()));
}
