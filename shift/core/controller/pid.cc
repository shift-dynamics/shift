#include "shift/core/controller/pid.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"
#include "shift/core/sensor/base.h"

bool shift::core::controller::PID::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "pid", shift::core::controller::PID::Create);

std::unique_ptr<shift::core::controller::PID>
shift::core::controller::PID::Create() {
  return std::make_unique<shift::core::controller::PID>();
}

void shift::core::controller::PID::DeclareParameters() {
  shift::core::controller::Base::DeclareParameters();
  DeclareParameter("kp", &kp_, "proportional gain", true).set_runtime();
  DeclareParameter("ki", &ki_, "integral gain", false).set_runtime();
  DeclareParameter("kd", &kd_,
                   "derivative gain, requires a sensor that provides the rate "
                   "of its signal (e.g. a position)",
                   false);
  DeclareInput("target", &target_,
               "target value of the sensor signal; optional, default 0");
  DeclareParameter("sensor", &sensor_name_,
                   "name of the input sensor, in the same assembly", true);
  DeclareParameter("joint", &joint_name_,
                   "name of the joint the effort acts on, in the same "
                   "assembly; give either joint, or component and input",
                   false);
  DeclareParameter("component", &component_name_,
                   "name of the component whose input the effort sets: in "
                   "the same assembly, as \"boom_cylinder\", or in another "
                   "by its fully qualified name, <assembly>__<name>, as "
                   "\"arm__boom_cylinder\"; with its kind before it where "
                   "two kinds of component have the name, as "
                   "\"force_modules/boom_cylinder\"; with input",
                   false);
  DeclareParameter("input", &input_name_,
                   "the input of component that the effort sets, as "
                   "\"command\" of a hydraulic cylinder",
                   false);
  DeclareOutput("error", &error_, "target minus the sensor signal");
  DeclareOutput("effort", &effort_, "the control effort");
}

void shift::core::controller::PID::Connect() {
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for controller " +
                                 get_name());
  }
  try {
    sensor_ = (*parent_assembly_ptr_)["sensors"][sensor_name_]
                .get_data<shift::core::sensor::Base>();
  } catch(...) {
    sensor_ = nullptr;
  }
  if(sensor_ == nullptr) {
    throw shift::core::Exception("Controller " + get_name() +
                                 ": sensor '" + sensor_name_ +
                                 "' not found in its assembly");
  }
  const bool sets_input = !component_name_.empty() || !input_name_.empty();
  if(sets_input == !joint_name_.empty()) {
    throw shift::core::Exception(
      "Controller " + get_name() +
      ": give either joint, for a force on a joint, or component and input, "
      "to set an input of another component");
  }
  joint_ = nullptr;
  component_ = nullptr;
  if(sets_input) {
    if(component_name_.empty() || input_name_.empty()) {
      throw shift::core::Exception("Controller " + get_name() +
                                   ": component and input go together");
    }
    try {
      component_ = parent_assembly_ptr_->FindComponent(
        component_name_, context_->get_assemblies());
    } catch(const shift::core::Exception& e) {
      throw shift::core::Exception("Controller " + get_name() +
                                   ": component " + e.what());
    }
    if(component_ == nullptr) {
      throw shift::core::Exception("Controller " + get_name() +
                                   ": component '" + component_name_ +
                                   "' not found in its assembly, nor as the "
                                   "fully qualified name of a component of "
                                   "another");
    }
    if(component_ == this) {
      throw shift::core::Exception("Controller " + get_name() +
                                   ": cannot set an input of itself");
    }
    try {
      component_->ConnectInput(input_name_, FullyQualifiedName(get_name()));
    } catch(const shift::core::Exception& e) {
      throw shift::core::Exception("Controller " + get_name() + ": " +
                                   e.what());
    }
  } else {
    try {
      joint_ = (*parent_assembly_ptr_)["joints"][joint_name_]
                 .get_data<shift::core::joint::Base>();
    } catch(...) {
      joint_ = nullptr;
    }
    if(joint_ == nullptr || joint_->get_dq_count() == 0) {
      throw shift::core::Exception("Controller " + get_name() + ": joint '" +
                                   joint_name_ +
                                   "' not found in its assembly or rigid");
    }
  }
  if(kd_ != 0.0 && !sensor_->has_rate()) {
    throw shift::core::Exception(
      "Controller " + get_name() + ": kd requires sensor '" + sensor_name_ +
      "' to provide the rate of its signal (use a position quantity)");
  }
}

void shift::core::controller::PID::Compute(const double* y) {
  error_ = target_ - sensor_->get_value();
  effort_ = kp_ * error_ + ki_ * y[state_offset_];
  if(kd_ != 0.0) {
    effort_ -= kd_ * sensor_->get_rate();
  }
}

void shift::core::controller::PID::UpdateOutputs(double, const double* y) {
  assert(sensor_);
  Compute(y);
  if(component_) {
    component_->SetInput(input_name_, effort_);
  }
}

void shift::core::controller::PID::Service(const double* y, double* dy) {
  assert(sensor_ && (joint_ || component_) && context_);
  Compute(y);
  dy[state_offset_] = error_;
  if(component_) {
    component_->SetInput(input_name_, effort_);
  } else {
    context_->get_joint_working_forces()[joint_->get_dq_index()] += effort_;
  }
}
