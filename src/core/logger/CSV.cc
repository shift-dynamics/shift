#include "shift/core/logger/CSV.h"
#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/logger/Base.h"
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
}

void shift::core::logger::CSV::Initialize() {
  if(timestamp_logfile_) {
    csv_logfile_ = shift::core::utility::stripFileExtension(csv_logfile_) + "-" +
                   shift::core::utility::currentDateTime() +
                   shift::core::utility::getFileExtension(csv_logfile_);
  }
  file_.open(csv_logfile_);
  WriteHeader();
}

void shift::core::logger::CSV::WriteHeader() {
  if(log_all_channels_) {
    assert(context_);
    file_ << "time" << delimiter_;
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_q_count(); i++) {
        file_ << joint->get_name() << "_q_" << i << delimiter_;
      }
    }
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_dq_count(); i++) {
        file_ << joint->get_name() << "_dq_" << i << delimiter_;
      }
    }
    for(auto& joint : context_->get_active_joints()) {
      for(size_t i = 0; i < joint->get_dq_count(); i++) {
        file_ << joint->get_name() << "_ddq_" << i << delimiter_;
      }
    }
    file_ << "constraint_error" << delimiter_;
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
  file_ << context_->get_constraint_error() << delimiter_;
  file_ << "\n";
}
