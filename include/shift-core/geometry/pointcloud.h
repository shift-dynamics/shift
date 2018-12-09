#ifndef _SHIFT_POINTCLOUDGEOMETRY_H_
#define _SHIFT_POINTCLOUDGEOMETRY_H_

#include "shift-core/geometry/geometry.h"

namespace Shift {

namespace Geometry {

class PointCloud : public GeometryBase {
public:
  PointCloud()
    : GeometryBase() {
    addKeys();
  }
  PointCloud(std::string name)
    : GeometryBase(name) {
    addKeys();
  }
  ~PointCloud() {}
  static PointCloud* create();
  void addKeys();

  static bool isRegistered() {
    return registered_;
  }

private:
  static bool registered_;
};

} // namespace Geometry

} // namespace Shift

#endif
