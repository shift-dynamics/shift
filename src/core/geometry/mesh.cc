#include "shift/core/geometry/mesh.h"
#include "shift/core/factory.h"

bool shift::Geometry::Mesh::registered_ =
  shift::Factory<shift::Geometry::Base>::get()->registerModule(
    "mesh", shift::Geometry::Mesh::create);

std::unique_ptr<shift::Geometry::Mesh> shift::Geometry::Mesh::create() {
  return std::make_unique<shift::Geometry::Mesh>();
}

void shift::Geometry::Mesh::addParameters() {}
