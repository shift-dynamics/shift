#include "shift-core/geometry/ellipsoid.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Geometry::GeometryBase>
  registration("pid", Shift::Geometry::Ellipsoid::create);
