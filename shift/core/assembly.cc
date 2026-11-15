#include "shift/core/assembly.h"
#include "shift/core/controller/base.h"
#include "shift/core/body/base.h"
#include "shift/core/body/fixed.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/sensor/base.h"

#include <iostream>

void shift::core::Assembly::DeclareParameters() {

  DeclareParameter("name", &name_, "User defined descriptive assembly name.", true);

  DeclareParameter(
    "active", &active_,
    "Indicates whether current assembly is active in simulation: if "
    "set to false, the data will be loaded into computer working memory,"
    "but will not be processed when setting up and solving the equations of"
    "motion. Use this key to selectively enable or disable portions of the"
    "model. Value defaults to 'true' if not provided.",
    false);

  DeclareParameter("assemblies", &assemblies_,
               "Specifies an array of Shift assembly datastructures; each "
               "assembly in the array must have a unique name identifier.",
               false);

  DeclareParameter(
    "bodies", &bodies_,
    "Specifies an array of Shift 'Body' data structures. Each body in "
    "array must have a unique name identifier.",
    false);

  DeclareParameter("joints", &joints_,
               "Specifies an array of Shift 'Joint' datastructures; each joint "
               "in the array must have a unique name identifier",
               false);

  DeclareParameter("force_modules", &force_modules_,
               "Specifies an array of Shift 'force_module' datastructures for "
               "feeding forces into a running module; each force module "
               "in the array must have a unique name identifier",
               false);

  DeclareParameter("sensors", &sensors_,
               "Specifies an array of Shift 'Sensor' datastructures, signals "
               "measured from the model state that controllers read by name; "
               "each sensor in the array must have a unique name identifier",
               false);

  DeclareParameter("controllers", &controllers_,
               "Specifies an array of Shift 'Controller' datastructures, e.g. "
               "PID controllers that read sensors and drive joints; each "
               "controller in the array must have a unique name identifier",
               false);

  DeclareParameter("constraints", &constraints_,
               "Specifies one or more user-defined constraints in an array of "
               "Shift 'Constraint' datastructures. Each "
               "constraint in the array must have a unique name identifier",
               false);
}

shift::core::parameter::List*
shift::core::Assembly::FindComponent(const std::string& name) {
  struct Kind {
    const char* key;
    shift::core::parameter::List* found;
  };
  const size_t slash = name.find('/');
  const std::string bare =
    slash == std::string::npos ? name : name.substr(slash + 1);
  Kind kinds[] = {
    {"bodies", FindByName(bodies_.get_data(), bare)},
    {"joints", FindByName(joints_.get_data(), bare)},
    {"force_modules", FindByName(force_modules_.get_data(), bare)},
    {"sensors", FindByName(sensors_.get_data(), bare)},
    {"controllers", FindByName(controllers_.get_data(), bare)},
    {"constraints", FindByName(constraints_.get_data(), bare)},
    {"assemblies", FindByName(assemblies_.get_data(), bare)},
  };
  if(slash != std::string::npos) {
    const std::string key = name.substr(0, slash);
    for(const auto& kind : kinds) {
      if(key == kind.key) {
        return kind.found;
      }
    }
    throw shift::core::Exception(
      "'" + name + "': '" + key +
      "' is not a kind of component of an assembly (bodies, joints, "
      "force_modules, sensors, controllers, constraints or assemblies)");
  }
  shift::core::parameter::List* found = nullptr;
  std::string keys;
  for(const auto& kind : kinds) {
    if(kind.found) {
      found = kind.found;
      keys += (keys.empty() ? "" : ", ") + std::string(kind.key) + "/" + name;
    }
  }
  if(keys.find(',') != std::string::npos) {
    throw shift::core::Exception("'" + name + "' names more than one "
                                 "component in assembly '" + get_name() +
                                 "': give one of " + keys);
  }
  return found;
}

shift::core::parameter::List* shift::core::Assembly::FindComponent(
  const std::string& name,
  std::vector<std::unique_ptr<shift::core::Assembly>>& assemblies) {
  shift::core::parameter::List* found = FindComponent(name);
  if(found != nullptr) {
    return found;
  }
  /* the kind, if given, then <assembly>__..__<name>: each split at a "__",
  from the last, into the name of an assembly and of a component in it */
  const size_t slash = name.find('/');
  const std::string kind =
    slash == std::string::npos ? "" : name.substr(0, slash + 1);
  const std::string qualified =
    slash == std::string::npos ? name : name.substr(slash + 1);
  for(size_t split = qualified.rfind("__");
      split != std::string::npos && split > 0;
      split = qualified.rfind("__", split - 1)) {
    shift::core::Assembly* assembly =
      FindQualifiedAssembly(assemblies, qualified.substr(0, split));
    if(assembly != nullptr) {
      found = assembly->FindComponent(kind + qualified.substr(split + 2));
      if(found != nullptr) {
        return found;
      }
    }
  }
  return nullptr;
}

shift::core::Assembly* shift::core::Assembly::FindQualifiedAssembly(
  std::vector<std::unique_ptr<shift::core::Assembly>>& assemblies,
  const std::string& qualified) {
  for(auto& assembly : assemblies) {
    if(assembly->FullyQualifiedName(assembly->get_name()) == qualified) {
      return assembly.get();
    }
    shift::core::Assembly* nested =
      FindQualifiedAssembly(assembly->get_assemblies(), qualified);
    if(nested != nullptr) {
      return nested;
    }
  }
  return nullptr;
}

void shift::core::Assembly::CheckUniqueNames() {
  CheckObjectNames(assemblies_.get_data(), "Assembly");
  CheckObjectNames(bodies_.get_data(), "Body");
  CheckObjectNames(joints_.get_data(), "Joint");
  CheckObjectNames(force_modules_.get_data(), "Force module");
  CheckObjectNames(sensors_.get_data(), "Sensor");
  CheckObjectNames(controllers_.get_data(), "Controller");
  CheckObjectNames(constraints_.get_data(), "Constraint");
  for(auto& body : bodies_.get_data()) {
    body->CheckUniqueNames();
  }
  for(auto& module : force_modules_.get_data()) {
    module->CheckUniqueNames();
  }
  for(auto& assembly : assemblies_.get_data()) {
    assembly->CheckUniqueNames();
  }
}
