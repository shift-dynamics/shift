#include "shift/core/force_module/electric_motor.h"

#include <algorithm>
#include <cassert>

#include "shift/core/assembly.h"
#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/joint/revolute.h"

bool shift::core::force_module::ElectricMotor::registered_ =
  shift::core::Factory<shift::core::force_module::Base>::get()->RegisterModule(
    "electric_motor", shift::core::force_module::ElectricMotor::Create);

std::unique_ptr<shift::core::force_module::ElectricMotor>
shift::core::force_module::ElectricMotor::Create() {
  return std::make_unique<shift::core::force_module::ElectricMotor>();
}

void shift::core::force_module::ElectricMotor::DeclareParameters() {
  DeclareParameter("joint", &joint_name_,
                   "name of the revolute joint from the stator (parent) to "
                   "the rotor (child), in the same assembly",
                   true);
  DeclareParameter("torque_constant", &torque_constant_,
                   "torque per current, N m/A, which is also the back voltage "
                   "per rotor speed, V s/rad",
                   true);
  DeclareParameter("resistance", &resistance_,
                   "resistance of the winding, ohm", true);
  DeclareParameter("inductance", &inductance_,
                   "inductance of the winding, H, used by the voltage drive, "
                   "where it makes the current a state; optional, default 0",
                   false);
  DeclareParameter("drive", &drive_name_,
                   "\"current\": the drive makes the current follow the "
                   "input current; \"voltage\": it applies the input voltage; "
                   "optional, default \"current\"",
                   false);
  DeclareParameter("supply_voltage", &supply_voltage_,
                   "the largest voltage the drive can apply, V; optional, "
                   "default 0, no limit",
                   false);
  DeclareParameter("current_limit", &current_limit_,
                   "the largest current of the current drive, A; optional, "
                   "default 0, no limit",
                   false);
  DeclareInput("current", &current_,
               "the current the current drive makes, A; optional, default 0");
  DeclareInput("voltage", &voltage_,
               "the voltage the voltage drive applies, V; optional, default 0");
  DeclareOutput("winding_current", &winding_current_,
                "the current in the winding, A");
  DeclareOutput("terminal_voltage", &terminal_voltage_,
                "the voltage across the winding, V");
  DeclareOutput("back_voltage", &back_voltage_,
                "the back voltage of the turning rotor, V");
  DeclareOutput("torque", &torque_, "the torque on the joint, N m");
  DeclareOutput("speed", &speed_,
                "the speed of the rotor relative to the stator, rad/s");
  DeclareOutput("electrical_power", &electrical_power_,
                "the power from the supply, W");
  DeclareOutput("mechanical_power", &mechanical_power_,
                "the power to the rotor, W");
}

void shift::core::force_module::ElectricMotor::Connect() {
  const std::string prefix = "Electric motor " + get_name() + ": ";
  if(parent_assembly_ptr_ == nullptr) {
    throw shift::core::Exception(prefix + "no parent assembly");
  }
  try {
    joint_ = (*parent_assembly_ptr_)["joints"][joint_name_]
               .get_data<shift::core::joint::Base>();
  } catch(...) {
    joint_ = nullptr;
  }
  if(dynamic_cast<shift::core::joint::Revolute*>(joint_) == nullptr) {
    throw shift::core::Exception(prefix + "joint '" + joint_name_ +
                                 "' not found in its assembly or not a "
                                 "revolute joint");
  }
}

void shift::core::force_module::ElectricMotor::Load() {
  const std::string prefix = "Electric motor " + get_name() + ": ";
  if(drive_name_ == "current") {
    drive_ = Drive::kCurrent;
  } else if(drive_name_ == "voltage") {
    drive_ = Drive::kVoltage;
  } else {
    throw shift::core::Exception(prefix + "drive '" + drive_name_ +
                                 "' is not \"current\" or \"voltage\"");
  }
  if(!(torque_constant_ > 0.0) || !(resistance_ > 0.0)) {
    throw shift::core::Exception(prefix + "torque_constant and resistance "
                                 "must be positive");
  }
  if(!(inductance_ >= 0.0) || !(supply_voltage_ >= 0.0) ||
     !(current_limit_ >= 0.0)) {
    throw shift::core::Exception(prefix + "inductance, supply_voltage and "
                                 "current_limit must not be negative");
  }
  /* only the drive's input is used: the other must be neither given nor
  set */
  const std::string unused =
    drive_ == Drive::kCurrent ? "voltage" : "current";
  const shift::core::parameter::Value* value = Find(unused);
  assert(value);
  if(value->is_loaded_from_file() || !value->get_driver().empty()) {
    throw shift::core::Exception(
      prefix + "the drive is \"" + drive_name_ + "\", so the input '" +
      unused + "' is not used" +
      (value->get_driver().empty() ? ""
                                   : " (" + value->get_driver() + " sets it)") +
      "; set '" + (drive_ == Drive::kCurrent ? "current" : "voltage") +
      "' instead");
  }
  equation_count_ = has_current_state() ? 1 : 0;
  q_0_.assign(equation_count_, 0.0);
}

void shift::core::force_module::ElectricMotor::Evaluate(const double* y,
                                                        double* dy) {
  assert(joint_ && context_);
  const double k = torque_constant_;
  const double r = resistance_;
  speed_ = y[context_->get_q_count() + joint_->get_dq_index()];
  back_voltage_ = k * speed_;
  if(drive_ == Drive::kCurrent) {
    double i = current_;
    if(current_limit_ > 0.0) {
      i = std::clamp(i, -current_limit_, current_limit_);
    }
    if(supply_voltage_ > 0.0) {
      i = std::clamp(i, (-supply_voltage_ - back_voltage_) / r,
                     (supply_voltage_ - back_voltage_) / r);
    }
    winding_current_ = i;
    terminal_voltage_ = r * i + back_voltage_;
  } else {
    double v = voltage_;
    if(supply_voltage_ > 0.0) {
      v = std::clamp(v, -supply_voltage_, supply_voltage_);
    }
    terminal_voltage_ = v;
    if(has_current_state()) {
      winding_current_ = y[state_offset_];
      if(dy) {
        dy[state_offset_] =
          (v - r * winding_current_ - back_voltage_) / inductance_;
      }
    } else {
      winding_current_ = (v - back_voltage_) / r;
    }
  }
  torque_ = k * winding_current_;
  electrical_power_ = terminal_voltage_ * winding_current_;
  mechanical_power_ = torque_ * speed_;
}

void shift::core::force_module::ElectricMotor::Service(const double* y,
                                                       double* dy) {
  Evaluate(y, dy);
  context_->get_joint_working_forces()[joint_->get_dq_index()] += torque_;
}

void shift::core::force_module::ElectricMotor::UpdateOutputs(
  double, const double* y) {
  Evaluate(y, nullptr);
}
