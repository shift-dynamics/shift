#ifndef SHIFT_TERRAIN_BASIC_H_
#define SHIFT_TERRAIN_BASIC_H_

#include "shift/core/terrain/base.h"

namespace Shift {

namespace Terrain {

class Basic : public Base {
public:
  Basic()
    : Base() {
    addParameters();
  }
  Basic(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~Basic() override {}
  static std::unique_ptr<Basic> create();
  static bool isRegistered() {
    return registered_;
  }

private:
  virtual void addParameters() override;
  std::string texture_filename_;
  std::string property_map_filename_;
  double x_scale_ = 1.0;
  double y_scale_ = 1.0;
  std::string coordinate_type_ = "sae";
  std::string interpolation_type_ = "bilinear";

  static bool registered_;
};

} // namespace Terrain

} // namespace Shift

#endif
