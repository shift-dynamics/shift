#include "shift-core/geometry/mesh.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Geometry::GeometryBase>
  registration("mesh", Shift::Geometry::Mesh::create);
