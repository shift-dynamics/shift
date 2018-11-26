#ifndef _SHIFT_ASSEMBLY_H_
#define _SHIFT_ASSEMBLY_H_

#include <string>
#include <vector>

#include "shift-core/base.h"
#include "shift-core/body.h"
#include "shift-core/forcemodule.h"
#include "shift-core/io.h"
#include "shift-core/joint.h"
#include "shift-core/light.h"
#include "shift-core/pointcloud.h"
#include "shift-core/restraint.h"

namespace Shift {

/*!
@brief Shift Assembly datastructure
*/
class Assembly : Base {

public:
  Assembly(std::string name)
    : Base(name) {}
  ~Assembly() {}

private:
  std::vector<Assembly> subassemblies_;
  std::vector<Body> bodies_;
  std::vector<Joint> joints_;
  std::vector<Light> lights_;
  std::vector<Restraint> restraints_;
  std::vector<Io> io_;
  std::vector<ForceModule> forcemodules_;
  std::vector<PointCloud> pointclouds_;
};

} // namespace Shift

#endif
