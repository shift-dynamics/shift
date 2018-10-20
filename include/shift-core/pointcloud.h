#ifndef _SHIFT_POINTCLOUD_H_
#define _SHIFT_POINTCLOUD_H_

#include <string>

#include "base.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class PointCloud : Base {

public:
  PointCloud(std::string name)
    : Base(name) {}
  ~PointCloud() {}

private:
};

} // namespace Shift

#endif
