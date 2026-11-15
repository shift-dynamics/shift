#include <cassert>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>

#include "shift/core/context.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/base.h"
#include "shift/core/logger/base.h"
#include "shift/core/logger/hdf5.h"
#include "shift/core/terrain/scm.h"
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
  WriteModel();
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
      const size_t free_count =
        context_->get_state_count() - context_->get_free_state_offset();
      const std::vector<std::pair<std::string, size_t>> blocks = {
        {"q", q_count}, {"dq", dq_count}, {"free", free_count}};
      size_t offset = 0;
      for(const auto& [name, count] : blocks) {
        if(count > 0) {
          states_.push_back(CreateSeries(states, name, count));
          state_offsets_.push_back(offset);
          state_counts_.push_back(count);
        }
        offset += count;
      }
    }
    /* /<kind>/<component>/<output> of each output the logger is connected
    to (ConnectOutputs); /force_modules is always there */
    outputs_.clear();
    std::map<std::string, H5::Group> groups;
    groups["/force_modules"] = hdf5_file_.createGroup("/force_modules");
    for(const auto& channel : channels_) {
      const std::string kind = "/" + channel.kind;
      const std::string name = kind + "/" + channel.component;
      if(!groups.count(kind)) {
        groups[kind] = hdf5_file_.createGroup(kind);
      }
      if(!groups.count(name)) {
        groups[name] = groups[kind].createGroup(channel.component);
      }
      outputs_.push_back(CreateSeries(groups[name], channel.output, 0));
    }
  }
  if(rows_ == 0) {
    CreateTerrainLogs();
  }
  AppendTerrainLogs();
  Append(time_, &t);
  for(size_t k = 0; k < states_.size(); k++) {
    Append(states_[k], y.data() + state_offsets_[k]);
  }
  /* the outputs of y, see Context::WriteStates */
  assert(channels_.size() == outputs_.size());
  for(size_t k = 0; k < outputs_.size(); k++) {
    Append(outputs_[k], channels_[k].value);
  }
  rows_++;
  hdf5_file_.flush(H5F_SCOPE_LOCAL);
}

/* a one-dimensional dataset of type that grows without bound */
static H5::DataSet CreateList(const H5::Group& group, const std::string& name,
                              const H5::PredType& type) {
  const hsize_t dims[1] = {0};
  const hsize_t max_dims[1] = {H5S_UNLIMITED};
  const hsize_t chunk[1] = {4096};
  H5::DataSpace space(1, dims, max_dims);
  H5::DSetCreatPropList properties;
  properties.setChunk(1, chunk);
  properties.setDeflate(1);
  return group.createDataSet(name, type, space, properties);
}

/* count values of type at the end of dataset, which holds length */
static void AppendList(H5::DataSet& dataset, hsize_t length, hsize_t count,
                       const void* values, const H5::PredType& type) {
  if(count == 0) {
    return;
  }
  const hsize_t dims[1] = {length + count};
  dataset.extend(dims);
  H5::DataSpace file_space = dataset.getSpace();
  const hsize_t start[1] = {length};
  const hsize_t counts[1] = {count};
  file_space.selectHyperslab(H5S_SELECT_SET, counts, start);
  H5::DataSpace memory_space(1, counts);
  dataset.write(values, type, memory_space, file_space);
}

void shift::core::logger::HDF5::CreateTerrainLogs() {
  terrain_logs_.clear();
  H5::Group group;
  for(auto terrain : shift::core::terrain::Base::WithRegions(
        context_->get_active_terrain())) {
    auto scm = dynamic_cast<const shift::core::terrain::Scm*>(terrain);
    if(!scm) {
      continue;
    }
    if(terrain_logs_.empty()) {
      group = hdf5_file_.createGroup("/terrain");
    }
    H5::Group node = group.createGroup(scm->get_name());
    const hsize_t nx = hsize_t(scm->get_node_count_x());
    const hsize_t ny = hsize_t(scm->get_node_count_y());
    {
      const int64_t counts[2] = {int64_t(nx), int64_t(ny)};
      const hsize_t two = 2;
      H5::Attribute a = node.createAttribute(
        "node_count", H5::PredType::NATIVE_INT64, H5::DataSpace(1, &two));
      a.write(H5::PredType::NATIVE_INT64, counts);
    }
    const std::pair<const char*, double> scalars[] = {
      {"x0", scm->NodeX(0)},
      {"y0", scm->NodeY(0)},
      {"spacing", scm->get_spacing()}};
    for(const auto& [name, value] : scalars) {
      H5::Attribute a = node.createAttribute(name, H5::PredType::NATIVE_DOUBLE,
                                             H5::DataSpace(H5S_SCALAR));
      a.write(H5::PredType::NATIVE_DOUBLE, &value);
    }
    {
      const ksl_SE3_t& d = scm->get_pose();
      double pose[4][4] = {{d.R.v0.x, d.R.v1.x, d.R.v2.x, d.t.x},
                           {d.R.v0.y, d.R.v1.y, d.R.v2.y, d.t.y},
                           {d.R.v0.z, d.R.v1.z, d.R.v2.z, d.t.z},
                           {0.0, 0.0, 0.0, 1.0}};
      const hsize_t dims[2] = {4, 4};
      H5::Attribute a = node.createAttribute(
        "pose", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(2, dims));
      a.write(H5::PredType::NATIVE_DOUBLE, pose);
    }
    {
      std::vector<double> z(nx * ny);
      for(hsize_t j = 0; j < ny; j++) {
        for(hsize_t i = 0; i < nx; i++) {
          z[j * nx + i] = scm->UndisturbedZ(long(i), long(j));
        }
      }
      const hsize_t dims[2] = {ny, nx};
      const hsize_t chunk[2] = {std::min<hsize_t>(ny, 256),
                                std::min<hsize_t>(nx, 256)};
      H5::DSetCreatPropList properties;
      properties.setChunk(2, chunk);
      properties.setDeflate(1);
      H5::DataSet dataset =
        node.createDataSet("undisturbed_z", H5::PredType::NATIVE_DOUBLE,
                           H5::DataSpace(2, dims), properties);
      dataset.write(z.data(), H5::PredType::NATIVE_DOUBLE);
    }
    TerrainLog log;
    log.scm = scm;
    log.index = CreateList(node, "index", H5::PredType::NATIVE_INT32);
    log.level = CreateList(node, "level", H5::PredType::NATIVE_FLOAT);
    log.row_start = CreateList(node, "row_start", H5::PredType::NATIVE_INT64);
    terrain_logs_.push_back(std::move(log));
  }
}

void shift::core::logger::HDF5::AppendTerrainLogs() {
  for(auto& log : terrain_logs_) {
    log.indices.clear();
    log.levels.clear();
    if(rows_ == 0) {
      /* every node that is not undisturbed, whatever the terrain started
      from */
      std::vector<float> all;
      log.scm->GetLevels(all);
      for(size_t k = 0; k < all.size(); k++) {
        if(all[k] != 0.0f) {
          log.indices.push_back(int32_t(k));
          log.levels.push_back(all[k]);
        }
      }
    } else {
      for(const auto& change : log.scm->get_output_changes()) {
        log.indices.push_back(change.index);
        log.levels.push_back(change.level);
      }
    }
    const int64_t start = int64_t(log.length);
    AppendList(log.row_start, rows_, 1, &start, H5::PredType::NATIVE_INT64);
    const hsize_t count = log.indices.size();
    AppendList(log.index, log.length, count, log.indices.data(),
               H5::PredType::NATIVE_INT32);
    AppendList(log.level, log.length, count, log.levels.data(),
               H5::PredType::NATIVE_FLOAT);
    log.length += count;
  }
}

void shift::core::logger::HDF5::WriteModel() {
  H5::Group model = hdf5_file_.createGroup("/model");
  const std::filesystem::path model_file =
    std::filesystem::absolute(context_->get_filename()).lexically_normal();
  auto write_string = [](H5::H5Object& object, const std::string& name,
                         const std::string& text) {
    H5::StrType type(H5::PredType::C_S1, std::max<size_t>(text.size(), 1));
    H5::Attribute attribute =
      object.createAttribute(name, type, H5::DataSpace(H5S_SCALAR));
    attribute.write(type, text.c_str());
  };
  {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    const std::string text =
      Json::writeString(builder, context_->get_model_json());
    H5::StrType type(H5::PredType::C_S1, std::max<size_t>(text.size(), 1));
    H5::DataSet json =
      model.createDataSet("json", type, H5::DataSpace(H5S_SCALAR));
    json.write(text.c_str(), type);
    write_string(json, "filename", model_file.filename().string());
  }
  H5::Group files = model.createGroup("files");
  int k = 0;
  for(const auto& path : context_->get_files_read()) {
    std::ifstream in(path, std::ios::binary);
    if(!in) {
      continue;
    }
    const std::vector<char> bytes((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
    const hsize_t size = bytes.size();
    H5::DataSet dataset;
    if(size > 0) {
      H5::DSetCreatPropList properties;
      const hsize_t chunk = std::min<hsize_t>(size, 1 << 20);
      properties.setChunk(1, &chunk);
      properties.setDeflate(1);
      dataset = files.createDataSet(std::to_string(k), H5::PredType::NATIVE_UINT8,
                                    H5::DataSpace(1, &size), properties);
      dataset.write(bytes.data(), H5::PredType::NATIVE_UINT8);
    } else {
      dataset = files.createDataSet(std::to_string(k), H5::PredType::NATIVE_UINT8,
                                    H5::DataSpace(H5S_NULL));
    }
    write_string(dataset, "path",
                 std::filesystem::path(path)
                   .lexically_relative(model_file.parent_path())
                   .string());
    write_string(dataset, "original", path);
    /* the names by which the model found it, one per line */
    std::string names;
    for(const auto& [name, found] : context_->get_file_names()) {
      if(found == path) {
        names += name + "\n";
      }
    }
    write_string(dataset, "names", names);
    k++;
  }
  hdf5_file_.flush(H5F_SCOPE_LOCAL);
}
