#include "shift/core/sensor/joint.h"

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"

bool shift::core::sensor::Joint::registered_ =
  shift::core::Factory<shift::core::sensor::Base>::get()->RegisterModule(
    "joint", shift::core::sensor::Joint::Create);

std::unique_ptr<shift::core::sensor::Joint>
shift::core::sensor::Joint::Create() {
  return std::make_unique<shift::core::sensor::Joint>();
}

void shift::core::sensor::Joint::DeclareParameters() {
  shift::core::sensor::Base::DeclareParameters();
  DeclareParameter("joint", &joint_name_,
                   "name of the measured joint, in the same assembly", true);
  DeclareParameter("quantity", &quantity_,
                   "measured quantity: \"position\" or \"velocity\"", true);
  DeclareParameter("index", &index_,
                   "degree of freedom of a multi-DOF joint, defaults to 0",
                   false);
}

void shift::core::sensor::Joint::Connect() {
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception("No parent assembly set for sensor " +
                                 get_name());
  }
  try {
    joint_ = (*parent_assembly_ptr_)["joints"][joint_name_]
               .get_data<shift::core::joint::Base>();
  } catch(...) {
    joint_ = nullptr;
  }
  if(joint_ == nullptr) {
    throw shift::core::Exception("Sensor " + get_name() + ": joint '" +
                                 joint_name_ + "' not found in its assembly");
  }
  if(quantity_ == "position") {
    position_ = true;
  } else if(quantity_ == "velocity") {
    position_ = false;
  } else {
    throw shift::core::Exception("Sensor " + get_name() +
                                 ": quantity must be \"position\" or "
                                 "\"velocity\", not \"" +
                                 quantity_ + "\"");
  }
  const int dq_count = static_cast<int>(joint_->get_dq_count());
  if(index_ < 0 || index_ >= dq_count) {
    throw shift::core::Exception("Sensor " + get_name() +
                                 ": index out of range for joint '" +
                                 joint_name_ + "'");
  }
  if(position_ && joint_->get_q_count() != joint_->get_dq_count()) {
    throw shift::core::Exception(
      "Sensor " + get_name() + ": positions of joint '" + joint_name_ +
      "' are quaternions; measure a frame instead");
  }
  has_rate_ = position_;
}

void shift::core::sensor::Joint::Update(const double* y) {
  assert(joint_ && context_);
  const double* dq = y + context_->get_q_count();
  const double velocity = dq[joint_->get_dq_index() + index_];
  if(position_) {
    value_ = y[joint_->get_q_index() + index_];
    rate_ = velocity;
  } else {
    value_ = velocity;
  }
}
