#include "shift-core/geometry/mesh.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Mesh::registered_ =
  Shift::Factory<Shift::Geometry::GeometryBase>::get()->registerModule(
    "mesh", Shift::Geometry::Mesh::create);

Shift::Geometry::Mesh* Shift::Geometry::Mesh::create() {
  return new Shift::Geometry::Mesh;
}

void Shift::Geometry::Mesh::addKeys() {}
