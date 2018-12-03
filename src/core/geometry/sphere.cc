#include "shift-core/geometry/sphere.h"
#include "shift-core/factory.h"

class Shift::Registration<Shift::Geometry::GeometryBase>
  registration("pid", Shift::Geometry::Sphere::create);
