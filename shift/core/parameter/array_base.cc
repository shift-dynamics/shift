#include "shift/core/parameter/array_base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/exception.h"

void shift::core::parameter::ArrayBase::Print(std::string prefix) const {
  std::cout << "[";
  if(objects_.empty()) {
    std::cout << " ]" << std::endl;
  } else {
    std::cout << std::endl;
    for(auto& parameter : objects_) {
      parameter->Print(prefix, false);
    }
    prefix.pop_back();
    prefix.pop_back();
    std::cout << prefix << "]" << std::endl;
  }
}

shift::core::parameter::Value& shift::core::parameter::ArrayBase::
operator[](const std::string key_str) {
  for(auto& object : objects_) {
    if(object->operator[]("name").get<std::string>() == key_str) {
      void_ = Value(object, "description", true);
      return void_;
    }
  }
  throw shift::core::Exception(
    "JSON Parsing error, parameter '" + key_str + "' not found");
  return void_;
}
