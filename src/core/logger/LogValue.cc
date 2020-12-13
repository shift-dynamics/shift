#include "shift/core/Context.h"
#include "shift/core/Factory.h"
#include "shift/core/logger/Base.h"
#include "shift/core/logger/HDF5.h"
#include "shift/core/utility.h"

bool shift::core::logger::HDF5::registered_ =
  shift::core::Factory<shift::core::logger::Base>::get()->registerModule(
    "hdf5", shift::core::logger::HDF5::create);

std::unique_ptr<shift::core::logger::HDF5> shift::core::logger::HDF5::create() {
  return std::make_unique<shift::core::logger::HDF5>();
}

void shift::core::logger::HDF5::addParameters() {
  shift::core::logger::Base::addParameters();
  addParameter("logfile", &logfile_,
               "hdf5 log file, defaults to \"simulation.hdf5\" if "
               "not provided",
               false);
  addParameter(
    "timestamp_logfile", &timestamp_logfile_,
    "indicates to append current data/time to base log file name. Time stamp "
    "will be appended before file extension, defaults to \"false\"",
    false);
}

void shift::core::logger::HDF5::initialize() {
  if(timestamp_logfile_) {
    logfile_ = shift::core::utility::stripFileExtension(logfile_) + "-" +
               shift::core::utility::currentDateTime() +
               shift::core::utility::getFileExtension(logfile_);
  }

  const H5std_string h5_filename(logfile_);
  hdf5_file_ = H5::H5File(h5_filename, H5F_ACC_TRUNC);

  state_meta_data_ = H5::Group(hdf5_file_.createGroup("/state_meta_data"));

  writeHeader();
}

void shift::core::logger::HDF5::writeHeader() {

  H5::StrType str_datatype(H5::PredType::C_S1, 1024);
  hsize_t dim = context_->activeJoints().size();
  H5::DataSpace space(1, &dim);

  const H5std_string q_count_("q_count");
  const H5std_string dq_count_("dq_count");
  const H5std_string q_offset_("q_offset");
  const H5std_string dq_offset_("dq_offset");
  const H5std_string name_("name");
  const H5std_string fully_qualified_name_("fully_qualified_name");

  struct Joint {
    unsigned int q_count;
    unsigned int dq_count;
    int q_offset;
    int dq_offset;
    char name[1024];
    char fully_qualified_name[1024];
  };

  H5::CompType joint_type(sizeof(Joint));
  joint_type.insertMember(q_count_, HOFFSET(Joint, q_count),
                          H5::PredType::NATIVE_INT);
  joint_type.insertMember(dq_count_, HOFFSET(Joint, dq_count),
                          H5::PredType::NATIVE_INT);
  joint_type.insertMember(q_offset_, HOFFSET(Joint, q_offset),
                          H5::PredType::NATIVE_INT);
  joint_type.insertMember(dq_offset_, HOFFSET(Joint, dq_offset),
                          H5::PredType::NATIVE_INT);
  joint_type.insertMember(name_, HOFFSET(Joint, name), str_datatype);
  joint_type.insertMember(fully_qualified_name_,
                          HOFFSET(Joint, fully_qualified_name), str_datatype);

  std::vector<Joint> joint_meta_data;
// joint_meta_data.reserve(context_->activeJoints().size());
  
  for(auto& joint : context_->activeJoints()) {

    Joint data = {
      joint->qCount(),
      joint->dqCount(),
      joint->qIndex(),
      joint->dqIndex(),
      {'\0'},
      {'\0'}
    };

    strcpy(data.name, joint->name().c_str());
    strcpy(data.fully_qualified_name, joint->name().c_str());

    joint_meta_data.push_back(data);
  }

  H5::Exception::dontPrint();

  const H5std_string dataset_name("/state_meta_data/generalized_variables");

  H5::DataSet dataset(hdf5_file_.createDataSet(dataset_name, joint_type, space));
  dataset.write(joint_meta_data.data(), joint_type);

#if 0
     
    H5std_string dataset_name("q");
    H5::DataSpace dataspace(2, dimsf);

    H5::IntType datatype(H5::PredType::NATIVE_DOUBLE);
    datatype.setOrder(H5T_ORDER_LE);
  
    H5::DataSet dataset = 
      generalized_variable.createDataSet(dataset_name, datatype, dataspace);
   

    dataset.write(data, H5::PredType::NATIVE_DOUBLE);
      
#endif
}

void shift::core::logger::HDF5::writeStates(const double /* t */,
                                            const std::vector<double>& q) {
#if 0
  // size_t q_offset;
  // const auto& q_offsets = context_->qLookupCompressed();
  for(size_t i = 0; i < context_->nonRigidJointCount(); i++) {

    hdf5_status_ = H5Dwrite(joint_dataset_ids_[i], H5T_NATIVE_DOUBLE, H5S_ALL,
                            H5S_ALL, H5P_DEFAULT, q.data());
  }
#endif
}
