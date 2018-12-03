#include "shift-core/geometry/pointcloud.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Geometry::GeometryBase>
  registration("pid", Shift::Geometry::PointCloud::create);
