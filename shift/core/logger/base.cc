#include "shift/core/logger/base.h"

#include <algorithm>
#include <cassert>

#include "shift/core/context.h"
#include "shift/core/controller/base.h"
#include "shift/core/exception.h"
#include "shift/core/force_module/base.h"
#include "shift/core/sensor/base.h"
#include "shift/core/logger/csv.h"
#include "shift/core/logger/hdf5.h"

bool shift::core::logger::Modules::is_initialized() {
  if(!shift::core::logger::CSV::is_registered()) {
    return false;
  }
  if(!shift::core::logger::HDF5::is_registered()) {
    return false;
  }
  return true;
}

void shift::core::logger::Base::DeclareParameters() {
  DeclareParameter("log_all_channels", &log_all_channels_,
               "indicates to log all generalized coordinates, defaults to true "
               "if not specified.",
               false);
  DeclareParameter("force_modules", &force_modules_,
                   "the force modules whose outputs are logged, each as "
                   "{\"name\": ..., \"outputs\": [...]}; optional (see "
                   "the logging page)",
                   false);
  DeclareParameter("sensors", &sensors_,
                   "the sensors whose outputs are logged, as force_modules; "
                   "optional",
                   false);
  DeclareParameter("controllers", &controllers_,
                   "the controllers whose outputs are logged, as "
                   "force_modules; optional",
                   false);
}

void shift::core::logger::LoggedComponent::DeclareParameters() {
  DeclareParameter("name", &name,
                   "name of the component, or its fully qualified name, "
                   "<assembly>__<name>, where the name alone is not unique",
                   true);
  DeclareParameter("outputs", &outputs,
                   "the outputs to log; optional, default all of them",
                   false);
}

void shift::core::logger::Base::ReadJSON(Json::Value& json_value) {
  chooses_outputs_ = json_value.isMember("force_modules") ||
                     json_value.isMember("sensors") ||
                     json_value.isMember("controllers");
  parameter::List::ReadJSON(json_value);
}

void shift::core::logger::Base::ConnectOutputs() {
  assert(context_);
  channels_.clear();
  if(!chooses_outputs_) {
    if(!log_all_channels_) {
      return;
    }
    auto add_all = [&](const std::string& kind, const parameter::List& list,
                       const std::string& component) {
      for(const auto& output : list.get_output_keys()) {
        channels_.push_back(
          {kind, component, output, list.GetOutputAddress(output)});
      }
    };
    for(auto* module : context_->get_active_force_modules()) {
      add_all("force_modules", *module,
              module->FullyQualifiedName(module->get_name()));
    }
    for(auto* sensor : context_->get_active_sensors()) {
      add_all("sensors", *sensor,
              sensor->FullyQualifiedName(sensor->get_name()));
    }
    for(auto* controller : context_->get_active_controllers()) {
      add_all("controllers", *controller,
              controller->FullyQualifiedName(controller->get_name()));
    }
    return;
  }
  for(const auto& entry : force_modules_.get_data()) {
    ConnectEntry(*entry, "force_modules",
                 context_->get_active_force_modules());
  }
  for(const auto& entry : sensors_.get_data()) {
    ConnectEntry(*entry, "sensors", context_->get_active_sensors());
  }
  for(const auto& entry : controllers_.get_data()) {
    ConnectEntry(*entry, "controllers", context_->get_active_controllers());
  }
}

/* the component of the name, as Context::FindActiveBody finds a body for an
object outside the assemblies: the one of that fully qualified name, else the
one of that name */
template <class T>
void shift::core::logger::Base::ConnectEntry(
  const LoggedComponent& entry, const std::string& kind,
  const std::vector<T*>& components) {
  const std::string singular =
    kind == "force_modules" ? "force module" : kind.substr(0, kind.size() - 1);
  const std::string prefix = "Logger: " + kind + ": '" + entry.name + "': ";
  const T* found = nullptr;
  std::vector<const T*> matches;
  for(const T* component : components) {
    if(component->FullyQualifiedName(component->get_name()) == entry.name) {
      found = component;
    } else if(component->get_name() == entry.name) {
      matches.push_back(component);
    }
  }
  if(found == nullptr) {
    if(matches.size() > 1) {
      std::string candidates;
      for(const T* component : matches) {
        candidates += (candidates.empty() ? "'" : ", '") +
                      component->FullyQualifiedName(component->get_name()) +
                      "'";
      }
      throw shift::core::Exception(prefix + "the name is ambiguous: use the "
                                            "fully qualified name, one of " +
                                   candidates);
    }
    if(matches.empty()) {
      throw shift::core::Exception(prefix + "there is no active " + singular +
                                   " of that name");
    }
    found = matches.front();
  }
  const std::string component = found->FullyQualifiedName(found->get_name());
  const std::vector<std::string>& outputs =
    entry.outputs.empty() ? found->get_output_keys() : entry.outputs;
  for(const auto& output : outputs) {
    const double* value = nullptr;
    try {
      value = found->GetOutputAddress(output);
    } catch(const shift::core::Exception& e) {
      throw shift::core::Exception(prefix + e.what());
    }
    const bool connected =
      std::any_of(channels_.begin(), channels_.end(), [&](const Channel& c) {
        return c.kind == kind && c.component == component && c.output == output;
      });
    if(!connected) {
      channels_.push_back({kind, component, output, value});
    }
  }
}

void shift::core::logger::Base::Load() {}

void shift::core::logger::Base::Connect() {}

void shift::core::logger::Base::ActivateObjects(bool parent_is_active) {
  assert(context_);
  context_->ActivateObject(this, parent_is_active && active_);
}
