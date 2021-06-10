#ifndef SHIFT_LOGGER_HDF5_H_
#define SHIFT_LOGGER_HDF5_H_

#include <string>
#include <H5Cpp.h>

#include "shift/core/logger/Base.h"

namespace shift {
namespace core {
namespace logger {

/*!
@brief Abstract base class for shift::Solver C++ objects
*/
class HDF5 : public Base {

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

  static bool is_registered() {
    return registered_;
  }

  const std::string& get_description() {
    return description_;
  }

  virtual void Initialize() override;
  virtual void WriteHeader() override;
  virtual void WriteStates(const double t,
                           const std::vector<double>& q) override;

private:
  static std::unique_ptr<HDF5> Create();
  static bool registered_;
  
  virtual void DeclareParameters() override;
  const std::string description_ = "";
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
