#include "shift-core/parameter/array.h"
#include "shift-core/assembly.h"

/* define specializations of Array class */
template class Shift::Parameter::Array<Shift::Assembly>;
template class Shift::Parameter::Array<Shift::Frame>;

template <class T>
void Shift::Parameter::Array<T>::readJSON(Json::Value& json_array) {
  for(auto& json_value : json_array) {
    auto object = std::make_unique<T>();
    if(object) {
      try {
        object->readJSON(json_value);
        object->setIsBody(T::arrayKey() == "bodies");
        object->setIsAssembly(T::arrayKey() == "assemblies");
      } catch(const char* msg) {
        std::cerr << msg << std::endl;
      }
      object_specializations_.push_back(std::move(object));
      objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
        object_specializations_.back().get()));
    }
  }
}

template <class T>
void Shift::Parameter::Array<T>::push_back(std::unique_ptr<T> object) {
  object_specializations_.push_back(std::move(object));
  objects_.push_back(dynamic_cast<Shift::Parameter::List*>(
    object_specializations_.back().get()));
}
