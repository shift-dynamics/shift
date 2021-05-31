#ifndef SHIFT_LOGGER_LOGVALUE_H_
#define SHIFT_LOGGER_LOGVALUE_H_

#include <string>
#include <H5Cpp.h>

#include "shift/core/logger/Base.h"

namespace shift {
namespace core {
namespace logger {

/*!
@brief Abstract base class for shift::Solver C++ objects
*/
class LogValue : public Base {

public:
  HDF5(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  HDF5()
    : Base("") {
    DeclareParameters();
  }
  virtual ~HDF5() override {}

  const std::string& getDescription() {
    return description_;
  }

  static bool is_registered() {
    return registered_;
  }

  virtual void initialize() override;
  virtual void writeHeader() override;
  virtual void writeStates(const double t,
                           const std::vector<double>& q) override;

private:
  virtual void DeclareParameters() override;
  const std::string description_ = "";
  static bool registered_;
  static std::unique_ptr<HDF5> create();
  std::string logfile_ = "simulation.h5";
  bool timestamp_logfile_ = false;

  H5::H5File hdf5_file_;
  H5::Group state_meta_data_;
  H5::Group generalized_variables_group_;
  std::vector<H5::Group> generalized_variables_;

  std::vector<H5std_string> dataset_names_;
};

} // namespace logger
} // namespace core
} // namespace shift

#endif
