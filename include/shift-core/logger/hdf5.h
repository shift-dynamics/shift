#ifndef SHIFT_LOGGER_HDF5_H_
#define SHIFT_LOGGER_HDF5_H_

#include "shift-core/logger/base.h"

#include <hdf5.h>
#include <string>

namespace Shift {

namespace Logger {

/*!
@brief Abstract base class for Shift::Solver C++ objects
*/
class HDF5 : public Base {

public:
  HDF5(std::string name)
    : Base(name) {
    addParameters();
  }
  HDF5()
    : Base("") {
    addParameters();
  }
  virtual ~HDF5() override {

    /* End access to the dataset and release resources used by it. */
    for(auto& dataset_id : joint_dataset_ids_) {
      hdf5_status_ = H5Dclose(dataset_id);
    }

    /* End access to the dataspace and release resources used by it. */
    for(auto& dataspace_id : joint_dataspace_ids_) {
      /* Terminate access to the data space. */
      hdf5_status_ = H5Sclose(dataspace_id);
    }

    /* Terminate access to the file. */
    hdf5_status_ = H5Fclose(hdf5_file_);
  }

  const std::string& getDescription() {
    return description_;
  }

  static bool isRegistered() {
    return registered_;
  }

  virtual void initialize() override;
  virtual void writeHeader() override;
  virtual void writeStates(const double t, const double q[]) override;

private:
  virtual void addParameters() override;
  const std::string description_ = "";
  static bool registered_;
  static std::unique_ptr<HDF5> create();
  std::string hdf5_logfile_ = "simulation.h5";
  bool timestamp_logfile_ = false;
  hid_t hdf5_file_;    /*!< file identifier */
  herr_t hdf5_status_; /*!< hd5f error status */
  std::vector<hid_t> joint_dataset_ids_;
  std::vector<hid_t> joint_dataspace_ids_;
};

} // namespace Logger

} // namespace Shift

#endif
