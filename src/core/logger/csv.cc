#include "shift/core/logger/csv.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/logger/base.h"
#include "shift/core/utility.h"

bool Shift::Logger::CSV::registered_ =
  Shift::Factory<Shift::Logger::Base>::get()->registerModule(
    "csv", Shift::Logger::CSV::create);

std::unique_ptr<Shift::Logger::CSV> Shift::Logger::CSV::create() {
  return std::make_unique<Shift::Logger::CSV>();
}

void Shift::Logger::CSV::addParameters() {
  Shift::Logger::Base::addParameters();
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

void Shift::Logger::CSV::initialize() {
  if(timestamp_logfile_) {
    csv_logfile_ = Shift::Utility::stripFileExtension(csv_logfile_) + "-" +
                   Shift::Utility::currentDateTime() +
                   Shift::Utility::getFileExtension(csv_logfile_);
  }
  file_.open(csv_logfile_);
  writeHeader();
}

void Shift::Logger::CSV::writeHeader() {
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

void Shift::Logger::CSV::writeStates(const double t,
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
