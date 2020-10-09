#ifndef SHIFT_GEOMETRY_POINTCLOUD_H_
#define SHIFT_GEOMETRY_POINTCLOUD_H_

#include "shift/core/geometry/base.h"

namespace Shift {

namespace Geometry {

class PointCloud : public Base {
public:
  PointCloud()
    : Base() {
    addParameters();
  }
  PointCloud(std::string name)
    : Base(name) {
    addParameters();
  }
  virtual ~PointCloud() override {}
  static std::unique_ptr<PointCloud> create();

  static bool isRegistered() {
    return registered_;
  }

private:
  void addParameters() override;
  static bool registered_;
};

} // namespace Geometry

} // namespace Shift

#endif
