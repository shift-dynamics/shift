#include "shift/core/logger/hdf5.h"
#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/logger/base.h"
#include "shift/core/utility.h"

bool Shift::Logger::HDF5::registered_ =
  Shift::Factory<Shift::Logger::Base>::get()->registerModule(
    "csv", Shift::Logger::HDF5::create);

std::unique_ptr<Shift::Logger::HDF5> Shift::Logger::HDF5::create() {
  return std::make_unique<Shift::Logger::HDF5>();
}

void Shift::Logger::HDF5::addParameters() {
  Shift::Logger::Base::addParameters();
  addParameter("hdf5_logfile", &hdf5_logfile_,
               "file to log csv states to, defaults to \"simulation.hdf5\" if "
               "not provided",
               false);
  addParameter(
    "timestamp_logfile", &timestamp_logfile_,
    "indicates to append current data/time to base log file name. Time stamp "
    "will be appended before file extension, defaults to \"false\"",
    false);
}

void Shift::Logger::HDF5::initialize() {
  if(timestamp_logfile_) {
    hdf5_logfile_ = Shift::Utility::stripFileExtension(hdf5_logfile_) + "-" +
                    Shift::Utility::currentDateTime() +
                    Shift::Utility::getFileExtension(hdf5_logfile_);
  }
  /* Create a new file using default properties. */
  hdf5_file_ =
    H5Fcreate(hdf5_logfile_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

  writeHeader();
}

void Shift::Logger::HDF5::writeHeader() {
  if(log_all_channels_) {
    // for(auto& joint : context_->activeJoints()) {
    //
    //   unsigned int q_count = joint->qCount();
    //   if(q_count > 0) {
    //     dims[0] = 4;
    //     dims[1] = 6;
    //     joint_dataspace_ids_.push_back(H5Screate_simple(2, dims, NULL));
    //
    //     /* Create the dataset. */
    //     joint_dataset_ids_.push_back(
    //       H5Dcreate2(hdf5_file_, "/dset", H5T_STD_I32BE, dataspace_id,
    //                  H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT));
    //   }
    //   for(size_t i = 0; i < joint->qCount(); i++) {
    //     // file_ << joint->name() << "_q_" << i << delimiter_;
    //   }
    // }
    // for(auto& joint : context_->activeJoints()) {
    //   for(size_t i = 0; i < joint->dqCount(); i++) {
    //     // file_ << joint->name() << "_dq_" << i << delimiter_;
    //   }
    // }
  }
}

void Shift::Logger::HDF5::writeStates(const double /* t */,
                                      const std::vector<double>& q) {

  // size_t q_offset;
  // const auto& q_offsets = context_->qLookupCompressed();
  for(size_t i = 0; i < context_->nonRigidJointCount(); i++) {

    hdf5_status_ = H5Dwrite(joint_dataset_ids_[i], H5T_NATIVE_DOUBLE, H5S_ALL,
                            H5S_ALL, H5P_DEFAULT, q.data());
  }
}
