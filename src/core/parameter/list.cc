#include "shift/core/parameter/list.h"
#include "shift/core/assembly.h"
#include "shift/core/body/base.h"
#include "shift/core/context.h"

void Shift::Parameter::List::print(std::string prefix, bool break_line) const {
  if(break_line) {
    std::cout << std::endl << prefix << "{";
  } else {
    std::cout << prefix << "{";
  }
  if(parameters_.empty()) {
    std::cout << " }" << std::endl;
  } else {
    std::cout << std::endl;
    for(auto& parameter : parameters_) {
      std::cout << prefix << "  " << parameter.first << ": ";
      parameter.second.print(prefix + "    ");
    }
    std::cout << prefix << "}" << std::endl;
  }
}

void Shift::Parameter::List::print() const {
  print("");
}

void Shift::Parameter::List::removeParameter(std::string key) {
  parameters_.erase(key);
}

void Shift::Parameter::List::readJSON(Json::Value& json_value) {
  for(auto& parameter : parameters_) {
    if(!json_value.isMember(parameter.first) && parameter.second.required()) {
      std::cout << "  error: " << parameter.first << std::endl;
      throw "JSON parsing error, required key not found";
    }
    if(json_value.isMember(parameter.first)) {
      parameter.second.readJSON(json_value[parameter.first]);
    }
  }
}

void Shift::Parameter::List::setParent(Shift::Context* context,
                                       Shift::Assembly* parent_assembly,
                                       Body::Base* parent_body) {
  context_ = context;
  parent_assembly_ptr_ = parent_assembly;
  parent_body_ptr_ = parent_body;

  Shift::Assembly* assembly = nullptr;
  is_assembly_ ? assembly = dynamic_cast<Shift::Assembly*>(this)
               : assembly = parent_assembly_ptr_;
  Shift::Body::Base* body = nullptr;
  is_body_ ? body = dynamic_cast<Shift::Body::Base*>(this)
           : body = parent_body_ptr_;
  for(auto& parameter : parameters_) {
    parameter.second.setParent(context_, assembly, body);
  }
}

Shift::Parameter::Value&
Shift::Parameter::List::addParameter(const std::string key,
                                     Shift::Parameter::Value value) {
  parameters_[key] = value;
  return parameters_[key];
}
