#include "shift/core/logger/csv.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/logger/base.h"
#include "shift/core/utility.h"

bool shift::core::logger::CSV::registered_ =
  shift::core::Factory<shift::core::logger::Base>::get()->registerModule(
    "csv", shift::core::logger::CSV::create);

std::unique_ptr<shift::core::logger::CSV> shift::core::logger::CSV::create() {
  return std::make_unique<shift::core::logger::CSV>();
}

void shift::core::logger::CSV::addParameters() {
  shift::core::logger::Base::addParameters();
  addParameter("delimiter", &delimiter_,
               "delimiter used in csv file, defaults to \",\" if not provided",
               false);
  addParameter(
    "csv_logfile", &csv_logfile_,
    "file to log csv states to, defaults to \"simulation.csv\" if not provided",
    false);
  addParameter(
    "timestamp_logfile", &timestamp_logfile_,
    "indicates to append current data/time to base log file name. Time stamp "
    "will be appended before file extension, defaults to \"false\"",
    false);
}

void shift::core::logger::CSV::initialize() {
  if(timestamp_logfile_) {
    csv_logfile_ = shift::Utility::stripFileExtension(csv_logfile_) + "-" +
                   shift::Utility::currentDateTime() +
                   shift::Utility::getFileExtension(csv_logfile_);
  }
  file_.open(csv_logfile_);
  writeHeader();
}

void shift::core::logger::CSV::writeHeader() {
  if(log_all_channels_) {
    assert(context_);
    file_ << "time" << delimiter_;
    for(auto& joint : context_->activeJoints()) {
      for(size_t i = 0; i < joint->qCount(); i++) {
        file_ << joint->name() << "_q_" << i << delimiter_;
      }
    }
    for(auto& joint : context_->activeJoints()) {
      for(size_t i = 0; i < joint->dqCount(); i++) {
        file_ << joint->name() << "_dq_" << i << delimiter_;
      }
    }
    for(auto& joint : context_->activeJoints()) {
      for(size_t i = 0; i < joint->dqCount(); i++) {
        file_ << joint->name() << "_ddq_" << i << delimiter_;
      }
    }
    file_ << "constraint_error" << delimiter_;
  }
  file_ << "\n";
}

void shift::core::logger::CSV::writeStates(const double t,
                                     const std::vector<double>& y) {
  file_ << t << delimiter_;
  size_t count = context_->qCount() + context_->dqCount();
  for(size_t i = 0; i < count; i++) {
    file_ << y[i] << delimiter_;
  }
  for(size_t i = 0; i < context_->dqCount(); i++) {
    file_ << context_->ddq()[i] << delimiter_;
  }
  file_ << context_->constraintError() << delimiter_;
  file_ << "\n";
}
