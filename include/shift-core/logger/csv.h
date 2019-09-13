#ifndef _SHIFT_LOGGER_CSV_H_
#define _SHIFT_LOGGER_CSV_H_

#include "shift-core/logger/base.h"

#include <fstream>
#include <string>

namespace Shift {

namespace Logger {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class CSV : public Base {

public:
  CSV(std::string name)
    : Base(name) {
    addParameters();
  }
  CSV()
    : Base("") {
    addParameters();
  }
  virtual ~CSV() override {}

  static bool isRegistered() {
    return registered_;
  }

  virtual void initialize() override;

private:
  virtual void addParameters() override;

  void writeHeader();
  void writeStates();

  std::string delimiter_ = ",";
  std::string csv_logfile_ = "simulation.csv";
  bool timestamp_logfile_ =
    false; ///!< indicates whether to append current time stamp to log file
  std::ofstream file_;

  static bool registered_;
  static std::unique_ptr<CSV> create();
};

} // namespace Logger

} // namespace Shift

#endif
