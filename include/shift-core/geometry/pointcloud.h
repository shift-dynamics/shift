#ifndef SHIFT_GEOMETRY_POINTCLOUD_H_
#define SHIFT_GEOMETRY_POINTCLOUD_H_

#include "shift-core/geometry/base.h"

namespace Shift {

namespace Geometry {

class PointCloud : public Base {
public:
  PointCloud()
    : Base() {
    addKeys();
  }
  PointCloud(std::string name)
    : Base(name) {
    addKeys();
  }
  ~PointCloud() {}
  static std::unique_ptr<PointCloud> create();
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
