#include "shift/core/parameter/ArrayBase.h"
#include "shift/core/parameter/List.h"
#include "shift/core/Exception.h"

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
