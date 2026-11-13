#include "shift/core/controller/map.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::controller::Map::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "map", shift::core::controller::Map::Create);

std::unique_ptr<shift::core::controller::Map>
shift::core::controller::Map::Create() {
  return std::make_unique<shift::core::controller::Map>();
}

void shift::core::controller::Map::DeclareParameters() {
  shift::core::controller::Schedule::DeclareParameters();
  DeclareParameter("component", &component_name_,
                   "name of the component whose output the map reads: in "
                   "the same assembly, as \"brake_pedal\", or in another by "
                   "its fully qualified name, <assembly>__<name>, as "
                   "\"car__brake_pedal\"; with its kind before it where two "
                   "kinds of component have the name, as "
                   "\"controllers/brake_pedal\"",
                   true);
  DeclareParameter("output", &output_name_,
                   "the output of component that the map reads, as "
                   "\"value\" of a schedule",
                   true);
  DeclareParameter("points", &points_,
                   "the curve, [x0, v0, x1, v1, ...]: the value v at each "
                   "value x of the output, x increasing; beyond the first "
                   "and the last point the value is held. Without points "
                   "the value is the output itself",
                   false)
    .set_row_dim(-1);
  DeclareParameter("transition_time", &transition_time_,
                   "step: the change of the output over which the value "
                   "changes to that of each point, as a smooth cubic; "
                   "defaults to 0, a jump",
                   false);
  DeclareParameter("targets", &target_paths_,
                   "the parameters to set, by their paths from the assembly "
                   "of the map, such as "
                   "\"bodies/chassis/lights/brake_lamp/intensity\"; each must "
                   "be changeable at run time",
                   true)
    .set_row_dim(-1);
  DeclareOutput("value", &value_, "the value of the curve at the output");
}

void shift::core::controller::Map::Connect() {
  const std::string prefix = "Map '" + get_name() + "': ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for controller " +
                                 get_name());
  }
  identity_ = points_.empty();
  if(identity_) {
    points_ = {0.0, 0.0, 1.0, 1.0}; // checked, but not used
  }
  ConnectCurve();
  if(identity_) {
    points_.clear();
  }
  shift::core::parameter::List* component = nullptr;
  try {
    component = parent_assembly_ptr_->FindComponent(
      component_name_, context_->get_assemblies());
  } catch(const shift::core::Exception& e) {
    throw shift::core::Exception(prefix + "component " + e.what());
  }
  if(component == nullptr) {
    throw shift::core::Exception(prefix + "component '" + component_name_ +
                                 "' is not in assembly '" +
                                 parent_assembly_ptr_->get_name() +
                                 "', nor the fully qualified name of a "
                                 "component of another");
  }
  if(component == this) {
    throw shift::core::Exception(prefix + "a map cannot read its own output");
  }
  try {
    output_ = component->GetOutputAddress(output_name_);
  } catch(const shift::core::Exception& e) {
    throw shift::core::Exception(prefix + "component '" + component_name_ +
                                 "': " + e.what());
  }
  Apply();
}

double shift::core::controller::Map::Curve(double x) const {
  return identity_ ? x : Evaluate(x);
}

void shift::core::controller::Map::Apply() {
  input_ = *output_;
  value_ = Curve(input_);
  SetTargets();
}

void shift::core::controller::Map::Service(const double*, double*) {
  Apply();
}

void shift::core::controller::Map::UpdateOutputs(double, const double*) {
  Apply();
}
