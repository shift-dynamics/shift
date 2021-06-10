#ifndef SHIFT_GEOMETRY_POINTCLOUD_H_
#define SHIFT_GEOMETRY_POINTCLOUD_H_

#include "shift/core/geometry/Base.h"

namespace shift {
namespace core {
namespace geometry {

class PointCloud : public Base {
public:
  PointCloud()
    : Base() {
    DeclareParameters();
  }
  PointCloud(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~PointCloud() override {}
  static std::unique_ptr<PointCloud> create();

  static bool is_registered() {
    return registered_;
  }

private:
  void DeclareParameters() override;
  static bool registered_;
};

} // namespace geometry
}
} // namespace shift

#endif
