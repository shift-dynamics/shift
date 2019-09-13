#include "shift-core/logger/csv.h"
#include "shift-core/factory.h"
#include "shift-core/logger/base.h"
#include "shift-core/utility.h"

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
    csv_logfile_ = Shift::Utility::stripFileExtension(csv_logfile_) +
                   Shift::Utility::currentDateTime() +
                   Shift::Utility::getFileExtension(csv_logfile_);
    std::cout << "logging to: " << csv_logfile_ << std::endl;
  }
  file_.open(csv_logfile_);
}

void Shift::Logger::CSV::writeHeader() {}

void Shift::Logger::CSV::writeStates() {}
