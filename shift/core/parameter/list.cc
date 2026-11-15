#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/body/base.h"
#include "shift/core/parameter/list.h"

#include <algorithm>
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
      if(parameter.second.is_output()) {
        throw shift::core::Exception(
          NameForMessages() + ": '" + parameter.first +
          "' is an output, which the component computes; it is not read from "
          "the model file");
      }
      parameter.second.ReadJSON(json_value[parameter.first]);
    }
  }
}

std::string shift::core::parameter::List::NameForMessages() const {
  for(const char* key : {"name", "type"}) {
    auto it = parameters_.find(key);
    if(it != parameters_.end() && it->second.has_data() &&
       it->second.get_value_type() == Value::STRING &&
       !it->second.get<std::string>().empty()) {
      return "'" + it->second.get<std::string>() + "'";
    }
  }
  return "a component";
}

const shift::core::parameter::Value&
shift::core::parameter::List::FindRole(const std::string& key,
                                       Value::Role role) const {
  auto it = parameters_.find(key);
  if(it == parameters_.end() || it->second.get_role() != role ||
     !it->second.has_data()) {
    const std::string kind = role == Value::INPUT ? "input" : "output";
    std::string message =
      NameForMessages() + " has no " + kind + " '" + key + "'";
    if(it != parameters_.end() && it->second.has_data()) {
      message += it->second.is_input()    ? " (it is an input)"
                 : it->second.is_output() ? " (it is an output)"
                                          : " (it is a parameter)";
    }
    std::string names;
    if(role == Value::OUTPUT) {
      for(const auto& output : output_keys_) {
        names += (names.empty() ? "" : ", ") + output;
      }
    } else {
      for(const auto& parameter : parameters_) {
        if(parameter.second.get_role() == role) {
          names += (names.empty() ? "" : ", ") + parameter.first;
        }
      }
    }
    message +=
      names.empty() ? "; it has none" : "; its " + kind + "s: " + names;
    throw shift::core::Exception(message);
  }
  return it->second;
}

bool shift::core::parameter::List::has_input(const std::string& key) const {
  auto it = parameters_.find(key);
  return it != parameters_.end() && it->second.is_input();
}

bool shift::core::parameter::List::has_output(const std::string& key) const {
  auto it = parameters_.find(key);
  return it != parameters_.end() && it->second.is_output();
}

void shift::core::parameter::List::ConnectInput(const std::string& key,
                                                const std::string& driver) {
  FindRole(key, Value::INPUT);
  try {
    parameters_[key].ConnectDriver(driver);
  } catch(const shift::core::Exception& e) {
    throw shift::core::Exception(NameForMessages() + ": " + e.what());
  }
}

void shift::core::parameter::List::SetInput(const std::string& key,
                                            double value) {
  *FindRole(key, Value::INPUT).get_data<double>() = value;
}

double
shift::core::parameter::List::GetInput(const std::string& key) const {
  return FindRole(key, Value::INPUT).get<double>();
}

double
shift::core::parameter::List::GetOutput(const std::string& key) const {
  return FindRole(key, Value::OUTPUT).get<double>();
}

const double*
shift::core::parameter::List::GetOutputAddress(const std::string& key) const {
  return static_cast<const double*>(
    FindRole(key, Value::OUTPUT).get_address());
}

shift::core::parameter::Value& shift::core::parameter::List::DeclareOutput(
  const std::string key, double* data, const std::string description) {
  Value value(data, description, false);
  value.set_role(Value::OUTPUT).set_load_from_json(false);
  Value& declared = DeclareParameter(key, value);
  if(std::find(output_keys_.begin(), output_keys_.end(), key) ==
     output_keys_.end()) {
    output_keys_.push_back(key);
  }
  return declared;
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
  /* a key is a parameter, an input or an output, not two of them: an output
  of the same key as a parameter would replace it */
  auto it = parameters_.find(key);
  if(it != parameters_.end() && it->second.has_data() &&
     it->second.get_role() != value.get_role()) {
    static const char* const kRoles[] = {"parameter", "input", "output"};
    throw shift::core::Exception(
      NameForMessages() + ": '" + key + "' is declared both as " +
      (it->second.get_role() == Value::INPUT ? "an " : "a ") +
      kRoles[it->second.get_role()] + " and as " +
      (value.get_role() == Value::PARAMETER ? "a " : "an ") +
      kRoles[value.get_role()]);
  }
  value.set_key(key);
  value.set_parent(this);
  parameters_[key] = value;
  return parameters_[key];
}
