#ifndef SHIFT_LOGGER_HDF5_H_
#define SHIFT_LOGGER_HDF5_H_

#include <cstdint>
#include <string>
#include <vector>

#include <H5Cpp.h>

#include "shift/core/logger/base.h"

namespace shift {
namespace core {
namespace terrain {
class Scm;
}
namespace logger {

/*!
@brief Logs the simulation to an HDF5 file

At each output time step the logger appends a row to each of:

- /time: the time, one value per row
- /states/q, /states/dq: the generalized coordinates and velocities, one
  row per output step; /state_meta_data/generalized_variables gives each
  joint's offsets into them; /states/free: the states of the free bodies
  (body::Free), where the model has them, so that q, dq and free are the
  whole state of the integrators (Context::get_state_count)
- /force_modules/\<assembly\>__\<name\>/\<output\>: each output of each force module (such
  as a cylinder's head_pressure or force), one value per row
- /sensors/\<assembly\>__\<name\>/\<output\>: each output of each sensor that has
  outputs (such as the pose of a frame of a Marker), one value per row
- /terrain/\<name\>, for each deformable terrain (terrain::Scm): the nodes
  whose levels changed at each output time, not the whole grid. Its
  attributes node_count ([nx, ny]), x0, y0 and spacing place node (i, j) at
  x0 + i spacing, y0 + j spacing of the terrain frame, and pose (4 x 4) is
  the terrain frame in the inertial frame. undisturbed_z (ny x nx) is z
  (down) of the undisturbed surface at each node, written once. index
  (int32, j nx + i) and level (float32, displacement down from the
  undisturbed surface) list the changes of every row in turn, and
  row_start (int64, one per row) is the first of them for each row: the
  changes of row r run from row_start[r] to row_start[r + 1], or to the end
  for the last row. The first row lists every node whose level is not 0, so
  the levels at row r are those of all the changes of rows 0 to r applied
  in order to a grid of zeros.

The log also holds the model, so that it can be played back without the
files it came from (the viewer opens it as it opens a model file):

- /model/json: the model file as text (a YAML model as JSON), with the
  attribute filename, the name of the model file
- /model/files/\<k\>: a copy of each file the model read
  (Context::get_files_read), such as STEP and mesh files, their textures and
  elevation maps, as bytes, with the attributes path, its path relative to
  the directory of the model file (which may start with ..), original,
  its absolute path when it was read, and names, the names by which the
  model found it (Context::FindFile), one per line; a file found only
  through another, such as the .mtl of an .obj, has none

The datasets grow as the simulation runs, and the file is flushed after
each row so it can be read while the simulation runs.
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

  /*! \brief a dataset that grows by a row at each output step: rows x
   * columns doubles, or rows doubles if columns is 0 */
  struct Series {
    H5::DataSet dataset;
    hsize_t columns = 0;
  };
  Series CreateSeries(const H5::Group& group, const std::string& name,
                      hsize_t columns);
  void Append(Series& series, const double* values);

  H5::H5File hdf5_file_;
  H5::Group state_meta_data_;
  H5::Group generalized_variables_group_;
  std::vector<H5::Group> generalized_variables_;

  std::vector<H5std_string> dataset_names_;

  hsize_t rows_ = 0; //!< output steps written
  Series time_;
  std::vector<Series> states_;  //!< q and dq, where the model has them
  std::vector<size_t> state_offsets_, state_counts_;
  std::vector<Series> outputs_; //!< one per output of each force module
  std::vector<double> output_values_;

  /*! \brief the changes of the levels of a deformable terrain */
  struct TerrainLog {
    const shift::core::terrain::Scm* scm = nullptr;
    H5::DataSet index;     //!< int32
    H5::DataSet level;     //!< float32
    H5::DataSet row_start; //!< int64
    hsize_t length = 0;    //!< changes written
    std::vector<int32_t> indices;
    std::vector<float> levels;
  };
  /*! \brief /terrain/\<name\> for each deformable terrain, on the first
   * row */
  void CreateTerrainLogs();
  /*! \brief /model: the model and the files it read */
  void WriteModel();
  void AppendTerrainLogs();
  std::vector<TerrainLog> terrain_logs_;
};

} // namespace logger
} // namespace core
} // namespace shift

#endif
