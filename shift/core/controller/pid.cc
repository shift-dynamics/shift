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
  DeclareParameter("target", &target_, "target value of the sensor signal",
                   false)
    .set_runtime();
  DeclareParameter("sensor", &sensor_name_,
                   "name of the input sensor, in the same assembly", true);
  DeclareParameter("joint", &joint_name_,
                   "name of the output joint, in the same assembly", true);
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
  if(kd_ != 0.0 && !sensor_->has_rate()) {
    throw shift::core::Exception(
      "Controller " + get_name() + ": kd requires sensor '" + sensor_name_ +
      "' to provide the rate of its signal (use a position quantity)");
  }
}

void shift::core::controller::PID::Service(const double* y, double* dy) {
  assert(sensor_ && joint_ && context_);
  error_ = target_ - sensor_->get_value();
  dy[state_offset_] = error_;
  effort_ = kp_ * error_ + ki_ * y[state_offset_];
  if(kd_ != 0.0) {
    effort_ -= kd_ * sensor_->get_rate();
  }
  context_->get_joint_working_forces()[joint_->get_dq_index()] += effort_;
}
