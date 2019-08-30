#include "shift-core/geometry/mesh.h"
#include "shift-core/factory.h"

bool Shift::Geometry::Mesh::registered_ =
  Shift::Factory<Shift::Geometry::Base>::get()->registerModule(
    "mesh", Shift::Geometry::Mesh::create);

std::unique_ptr<Shift::Geometry::Mesh> Shift::Geometry::Mesh::create() {
  return std::make_unique<Shift::Geometry::Mesh>();
}

void Shift::Geometry::Mesh::addParameters() {}
