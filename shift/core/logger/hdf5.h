#ifndef SHIFT_LOGGER_HDF5_H_
#define SHIFT_LOGGER_HDF5_H_

#include <string>
#include <vector>

#include <H5Cpp.h>

#include "shift/core/logger/base.h"

namespace shift {
namespace core {
namespace logger {

/*!
@brief Logs the simulation to an HDF5 file

At each output time step the logger appends a row to each of:

- /time: the time, one value per row
- /states/q, /states/dq: the generalized coordinates and velocities, one
  row per output step; /state_meta_data/generalized_variables gives each
  joint's offsets into them
- /force_modules/\<module\>/\<output\>: each output of each force module (such
  as a cylinder's head_pressure or force), one value per row

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
};

} // namespace logger
} // namespace core
} // namespace shift

#endif
