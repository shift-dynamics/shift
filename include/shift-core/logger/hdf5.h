#ifndef SHIFT_LOGGER_HDF5_H_
#define SHIFT_LOGGER_HDF5_H_

#include "shift-core/logger/base.h"

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
    addKeys();
  }
  HDF5()
    : Base("") {
    addKeys();
  }
  virtual ~HDF5() override {}

  const std::string& getDescription() {
    return description_;
  }

  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

  void readJSON(Json::Value&) override;
  virtual void initialize() override {}

private:
  const std::string description_ = "";
  static bool registered_;
  static std::unique_ptr<HDF5> create();
};

} // namespace Logger

} // namespace Shift

#endif
