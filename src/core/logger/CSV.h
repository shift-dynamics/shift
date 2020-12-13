#ifndef _SHIFT_LOGGER_CSV_H_
#define _SHIFT_LOGGER_CSV_H_

#include "shift/core/logger/Base.h"

#include <fstream>
#include <string>

namespace shift {
namespace core {
namespace logger {

/*!
@brief Abstract base class for shift::Solver C++ objects
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
  virtual void writeHeader() override;
  virtual void writeStates(const double t,
                           const std::vector<double>& q) override;

private:
  virtual void addParameters() override;

  std::string delimiter_ = ",";
  std::string csv_logfile_ = "simulation.csv";
  bool timestamp_logfile_ =
    false; ///!< indicates whether to append current time stamp to log file
  std::ofstream file_;

  static bool registered_;
  static std::unique_ptr<CSV> create();
};

} // namespace Logger
}
} // namespace shift

#endif
