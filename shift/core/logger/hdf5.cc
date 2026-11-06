#include <cassert>

#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/logger/hdf5.h"
#include "shift/core/utility.h"

bool shift::core::logger::HDF5::registered_ =
  shift::core::Factory<shift::core::logger::Base>::get()->RegisterModule(
    "hdf5", shift::core::logger::HDF5::Create);

std::unique_ptr<shift::core::logger::HDF5> shift::core::logger::HDF5::Create() {
  return std::make_unique<shift::core::logger::HDF5>();
}

void shift::core::logger::HDF5::DeclareParameters() {
  shift::core::logger::Base::DeclareParameters();
  DeclareParameter("logfile", &logfile_,
               "hdf5 log file, defaults to \"simulation.hdf5\" if "
               "not provided",
               false);
  DeclareParameter(
    "timestamp_logfile", &timestamp_logfile_,
    "indicates to append current data/time to base log file name. Time stamp "
    "will be appended before file extension, defaults to \"false\"",
    false);
}

void shift::core::logger::HDF5::Initialize() {
  if(timestamp_logfile_) {
    logfile_ = shift::core::utility::StripFileExtension(logfile_) + "-" +
               shift::core::utility::CurrentDateTime() +
               shift::core::utility::GetFileExtension(logfile_);
  }

  rows_ = 0;
  const H5std_string h5_filename(logfile_);
  hdf5_file_ = H5::H5File(h5_filename, H5F_ACC_TRUNC);

  state_meta_data_ = H5::Group(hdf5_file_.createGroup("/state_meta_data"));

  WriteHeader();
}

void shift::core::logger::HDF5::WriteHeader() {

  H5::StrType str_datatype(H5::PredType::C_S1, 1024);
  hsize_t dim = context_->get_active_joints().size();
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
  
  for(auto& joint : context_->get_active_joints()) {

    Joint data = {
      joint->get_q_count(),
      joint->get_dq_count(),
      joint->get_q_index(),
      joint->get_dq_index(),
      {'\0'},
      {'\0'}
    };

    strcpy(data.name, joint->get_name().c_str());
    strcpy(data.fully_qualified_name,
           joint->FullyQualifiedName(joint->get_name()).c_str());

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

shift::core::logger::HDF5::Series shift::core::logger::HDF5::CreateSeries(
  const H5::Group& group, const std::string& name, hsize_t columns) {
  Series series;
  series.columns = columns;
  const int rank = columns > 0 ? 2 : 1;
  const hsize_t dims[2] = {0, columns};
  const hsize_t max_dims[2] = {H5S_UNLIMITED, columns};
  const hsize_t chunk[2] = {256, columns};
  H5::DataSpace space(rank, dims, max_dims);
  H5::DSetCreatPropList properties;
  properties.setChunk(rank, chunk);
  series.dataset =
    group.createDataSet(name, H5::PredType::NATIVE_DOUBLE, space, properties);
  return series;
}

void shift::core::logger::HDF5::Append(Series& series, const double* values) {
  const int rank = series.columns > 0 ? 2 : 1;
  const hsize_t dims[2] = {rows_ + 1, series.columns};
  series.dataset.extend(dims);
  H5::DataSpace file_space = series.dataset.getSpace();
  const hsize_t start[2] = {rows_, 0};
  const hsize_t count[2] = {1, series.columns};
  file_space.selectHyperslab(H5S_SELECT_SET, count, start);
  H5::DataSpace memory_space(rank, count);
  series.dataset.write(values, H5::PredType::NATIVE_DOUBLE, memory_space,
                       file_space);
}

void shift::core::logger::HDF5::WriteStates(const double t,
                                            const std::vector<double>& y) {
  if(rows_ == 0) {
    /* the datasets, on the first row, once the model's force modules are
    connected */
    H5::Group root = hdf5_file_.openGroup("/");
    time_ = CreateSeries(root, "time", 0);
    states_.clear();
    state_offsets_.clear();
    state_counts_.clear();
    if(log_all_channels_) {
      H5::Group states = hdf5_file_.createGroup("/states");
      const size_t q_count = context_->get_q_count();
      const size_t dq_count = context_->get_dq_count();
      const std::vector<std::pair<std::string, size_t>> blocks = {
        {"q", q_count}, {"dq", dq_count}};
      size_t offset = 0;
      for(const auto& [name, count] : blocks) {
        if(count > 0) {
          states_.push_back(CreateSeries(states, name, count));
          state_offsets_.push_back(offset);
          state_counts_.push_back(count);
        }
        offset += count;
      }
      outputs_.clear();
      H5::Group modules = hdf5_file_.createGroup("/force_modules");
      for(auto& module : context_->get_active_force_modules()) {
        const auto names = module->get_output_names();
        if(names.empty()) {
          continue;
        }
        H5::Group group =
          modules.createGroup(module->FullyQualifiedName(module->get_name()));
        for(const auto& name : names) {
          outputs_.push_back(CreateSeries(group, name, 0));
        }
      }
    }
  }
  Append(time_, &t);
  for(size_t k = 0; k < states_.size(); k++) {
    Append(states_[k], y.data() + state_offsets_[k]);
  }
  if(!outputs_.empty()) {
    /* the force modules' outputs of y, see Context::WriteStates */
    output_values_.clear();
    for(auto& module : context_->get_active_force_modules()) {
      module->AppendOutputs(output_values_);
    }
    assert(output_values_.size() == outputs_.size());
    for(size_t k = 0; k < outputs_.size(); k++) {
      Append(outputs_[k], &output_values_[k]);
    }
  }
  rows_++;
  hdf5_file_.flush(H5F_SCOPE_LOCAL);
}
