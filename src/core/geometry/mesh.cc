#include "shift/core/geometry/mesh.h"
#include "shift/core/factory.h"

bool shift::core::geometry::Mesh::registered_ =
  shift::core::Factory<shift::core::geometry::Base>::get()->registerModule(
    "mesh", shift::core::geometry::Mesh::create);

std::unique_ptr<shift::core::geometry::Mesh> shift::core::geometry::Mesh::create() {
  return std::make_unique<shift::core::geometry::Mesh>();
}

void shift::core::geometry::Mesh::addParameters() {}
