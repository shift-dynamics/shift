#include "shift/core/body/fixed.h"
#include "shift/core/factory.h"

bool shift::core::body::Fixed::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "fixed", shift::core::body::Fixed::Create);

std::unique_ptr<shift::core::body::Fixed> shift::core::body::Fixed::Create() {
  return std::make_unique<shift::core::body::Fixed>();
}

void shift::core::body::Fixed::DeclareParameters() {
  RemoveParameter("name");
  RemoveParameter("inertia");
  RemoveParameter("center_of_mass");
  RemoveParameter("active");

  DeclareParameter("terrain", &terrain_,
               "an array of one or more Shift terrain objects", false);
  // the root contact model has no name, type or active flag of its own
  contact_.RemoveParameter("name");
  contact_.RemoveParameter("type");
  contact_.RemoveParameter("active");
  contact_.set_environment();
  DeclareParameter("environment", &environment_,
                   "the sky, sun and fog as the viewer and the cameras see "
                   "them (see environment::Environment); optional",
                   false);
  DeclareParameter("surface_materials", &surface_materials_,
                   "the materials of the ground: the look of each (texture, "
                   "normal and roughness maps), what cameras label it and "
                   "its soil; a terrain's material_map lays them out by "
                   "area (see environment::SurfaceMaterial); optional",
                   false);
  DeclareParameter("scenery", &scenery_,
                   "objects strewn over the terrains, as trees, bushes, "
                   "grass and rocks: meshes placed at random by surface "
                   "material, drawn and seen by cameras but not colliding "
                   "(see environment::Scenery); optional",
                   false);
  DeclareParameter("contact", &contact_,
                   "the contact model of the environment, with the parameters "
                   "of a contact force module (materials, pairs, ...): the "
                   "materials of the ground and the objects of the "
                   "environment. A contact force module in an assembly "
                   "overrides it: its materials and pairs replace those of "
                   "the same names, and the settings it gives replace these; "
                   "optional",
                   false);
}

shift::core::environment::Environment*
shift::core::body::Fixed::get_environment() {
  const shift::core::parameter::Value* value = Find("environment");
  return value && value->is_loaded_from_file() ? &environment_ : nullptr;
}

std::vector<const shift::core::environment::SurfaceMaterial*>
shift::core::body::Fixed::get_surface_materials() {
  std::vector<const shift::core::environment::SurfaceMaterial*> materials;
  for(auto& material : surface_materials_.get_data()) {
    materials.push_back(material.get());
  }
  return materials;
}

shift::core::force_module::Contact* shift::core::body::Fixed::get_root_contact() {
  const shift::core::parameter::Value* value = Find("contact");
  return value && value->is_loaded_from_file() ? &contact_ : nullptr;
}

void shift::core::body::Fixed::CheckUniqueNames() {
  shift::core::body::Base::CheckUniqueNames();
  CheckObjectNames(frames_.get_data(), "Frame");
  CheckObjectNames(lights_.get_data(), "Light");
  CheckObjectNames(terrain_.get_data(), "Terrain");
  CheckObjectNames(surface_materials_.get_data(), "Surface material");
  CheckObjectNames(scenery_.get_data(), "Scenery");
}
