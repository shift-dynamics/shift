#include "shift/core/logger/csv.h"

#include <iomanip>

#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/utility.h"


bool shift::core::logger::CSV::registered_ =
  shift::core::Factory<shift::core::logger::Base>::get()->RegisterModule(
    "csv", shift::core::logger::CSV::Create);

std::unique_ptr<shift::core::logger::CSV> shift::core::logger::CSV::Create() {
  return std::make_unique<shift::core::logger::CSV>();
}

void shift::core::logger::CSV::DeclareParameters() {
  shift::core::logger::Base::DeclareParameters();
  DeclareParameter("delimiter", &delimiter_,
               "delimiter used in csv file, defaults to \",\" if not provided",
               false);
  DeclareParameter(
    "csv_logfile", &csv_logfile_,
    "file to log csv states to, defaults to \"simulation.csv\" if not provided",
    false);
  DeclareParameter(
    "timestamp_logfile", &timestamp_logfile_,
    "indicates to append current data/time to base log file name. Time stamp "
    "will be appended before file extension, defaults to \"false\"",
    false);
  DeclareParameter(
    "precision", &precision_,
    "number of digits of precision after decimal point, defaults to 6",
    false);
}

void shift::core::logger::CSV::Initialize() {
  if(timestamp_logfile_) {
    csv_logfile_ = shift::core::utility::StripFileExtension(csv_logfile_) + "_" +
                   shift::core::utility::CurrentDateTime() +
                   shift::core::utility::GetFileExtension(csv_logfile_);
  }
  SHIFT_INFO("Creating log file '" + csv_logfile_ + "'");
  file_.open(csv_logfile_);
  WriteHeader();
}

void shift::core::logger::CSV::WriteHeader() {
  if(log_all_channels_) {
    assert(context_);
    int precision = (precision_ >= 0) ? precision_ : 6;
    file_ << std::setprecision(precision);
    file_ << "time" << delimiter_;
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_q_count(); i++) {
        file_ << joint->FullyQualifiedName(joint->get_name()) << "__q_" << i
              << delimiter_;
      }
    }
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_dq_count(); i++) {
        file_ << joint->FullyQualifiedName(joint->get_name()) << "__dq_" << i
              << delimiter_;
      }
    }
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_dq_count(); i++) {
        file_ << joint->FullyQualifiedName(joint->get_name()) << "__ddq_" << i
              << delimiter_;
      }
    }
    file_ << "rms_constraint_error" << delimiter_;
    file_ << "gravitational_potential_energy" << delimiter_;
    file_ << "kinetic_energy" << delimiter_;
    file_ << "total_energy" << delimiter_;

    for(size_t i = 0; i < context_->get_chord_constraint_count(); i++) {
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_tx" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_ty" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_tz" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_rx" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_ry" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_position_error_rz" << delimiter_;
    }

    for(size_t i = 0; i < context_->get_chord_constraint_count(); i++) {
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_tx" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_ty" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_tz" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_rx" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_ry" << delimiter_;
      file_ << "chord_constraint_" << std::to_string(i) << "_velocity_error_rz" << delimiter_;
    }

    /* the outputs of the force modules, such as cylinder pressures */
    for(auto& module : context_->get_active_force_modules()) {
      for(const auto& output : module->get_output_names()) {
        file_ << module->FullyQualifiedName(module->get_name()) << "__" << output
              << delimiter_;
      }
    }
  }

  file_ << "\n";

}

void shift::core::logger::CSV::WriteStates(const double t,
                                     const std::vector<double>& y) {
  file_ << t << delimiter_;
  size_t count = context_->get_q_count() + context_->get_dq_count();
  for(size_t i = 0; i < count; i++) {
    file_ << y[i] << delimiter_;
  }
  for(size_t i = 0; i < context_->get_dq_count(); i++) {
    file_ << context_->get_ddq()[i] << delimiter_;
  }
  file_ << context_->get_rms_constraint_error() << delimiter_;
  double pe = context_->GravitationalPotentialEnergy();
  double ke = context_->KineticEnergy(); 
  file_ << pe << delimiter_;
  file_ << ke << delimiter_;
  file_ << (pe + ke) << delimiter_;

  auto& jacobian = context_->get_jacobian();
  for(auto& e : jacobian.get_constraints()) {
    file_ << e << delimiter_;
  }
  for(auto& de : jacobian.get_de()) {
    file_ << de << delimiter_;
  }

  /* the force modules' outputs of y, see Context::WriteStates */
  if(log_all_channels_) {
    outputs_.clear();
    for(auto& module : context_->get_active_force_modules()) {
      module->AppendOutputs(outputs_);
    }
    for(double value : outputs_) {
      file_ << value << delimiter_;
    }
  }

  file_ << "\n";
  file_ << std::flush;
}
