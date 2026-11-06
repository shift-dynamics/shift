#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/body/base.h"
#include "shift/core/parameter/list.h"

#include <set>

#include "shift/core/exception.h"

void shift::core::parameter::List::Print(std::string prefix,
                                         bool break_line) const {
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
      parameter.second.Print(prefix + "    ");
    }
    std::cout << prefix << "}" << std::endl;
  }
}

void shift::core::parameter::List::Print() const {
  Print("");
}

void shift::core::parameter::List::RemoveParameter(std::string key) {
  parameters_.erase(key);
}

void shift::core::parameter::List::ReadJSON(Json::Value& json_value) {
  for(auto& parameter : parameters_) {
    if(!json_value.isMember(parameter.first) &&
       parameter.second.is_required()) {
      std::cout << "  error: " << parameter.first << std::endl;
      throw "JSON parsing error, required key not found";
    }
    if(json_value.isMember(parameter.first)) {
      parameter.second.ReadJSON(json_value[parameter.first]);
    }
  }
}

std::string shift::core::parameter::List::FullyQualifiedName(
  const std::string& name) const {
  /* every object has the fixed body, the context, as a parent body; only a body
  of the model is part of the name */
  if(parent_body_ptr_ && !is_body_ && parent_body_ptr_ != context_) {
    return parent_body_ptr_->FullyQualifiedName(parent_body_ptr_->get_name()) +
           "__" + name;
  }
  return parent_assembly_ptr_
           ? parent_assembly_ptr_->FullyQualifiedName(
               parent_assembly_ptr_->get_name()) + "__" + name
           : name;
}

void shift::core::parameter::List::CheckNames(
  const std::vector<std::string>& names, const std::string& kind) {
  std::set<std::string> seen;
  for(const auto& name : names) {
    if(!seen.insert(name).second) {
      throw shift::core::Exception(kind + " name '" + name + "' is not unique");
    }
  }
}

void shift::core::parameter::List::set_parent(
  shift::core::Context* context, shift::core::Assembly* parent_assembly,
  shift::core::body::Base* parent_body) {
  context_ = context;
  parent_assembly_ptr_ = parent_assembly;
  parent_body_ptr_ = parent_body;

  shift::core::Assembly* assembly = nullptr;
  is_assembly_ ? assembly = dynamic_cast<shift::core::Assembly*>(this)
               : assembly = parent_assembly_ptr_;
  shift::core::body::Base* body = nullptr;
  is_body_ ? body = dynamic_cast<shift::core::body::Base*>(this)
           : body = parent_body_ptr_;
  for(auto& parameter : parameters_) {
    parameter.second.set_parent(context_, assembly, body);
  }
}

shift::core::parameter::Value& shift::core::parameter::List::DeclareParameter(
  const std::string key, shift::core::parameter::Value value) {
  value.set_key(key);
  value.set_parent(this);
  parameters_[key] = value;
  return parameters_[key];
}
