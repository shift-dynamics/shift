#ifndef _SHIFT_LOGGER_CSV_H_
#define _SHIFT_LOGGER_CSV_H_

#include "shift/core/logger/base.h"

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
    DeclareParameters();
  }
  CSV()
    : Base("") {
    DeclareParameters();
  }
  virtual ~CSV() override {}

  static bool is_registered() {
    return registered_;
  }

  virtual void Initialize() override;
  virtual void WriteHeader() override;
  virtual void WriteStates(const double t,
                           const std::vector<double>& q) override;

private:
  static bool registered_;
  static std::unique_ptr<CSV> Create();

  virtual void DeclareParameters() override;

  int precision_ = 6;
  std::string delimiter_ = ",";
  std::vector<double> outputs_; //!< force module outputs, reused per row
  std::string csv_logfile_ = "simulation.csv";
  bool timestamp_logfile_ =
    false; ///!< indicates whether to append current time stamp to log file
  std::ofstream file_;

};

} // namespace Logger
}
} // namespace shift

#endif
