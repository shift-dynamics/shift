#ifndef SHIFT_BODY_FIXED_H_
#define SHIFT_BODY_FIXED_H_

#include "shift/core/body/base.h"
#include "shift/core/environment/environment.h"
#include "shift/core/environment/scenery.h"
#include "shift/core/environment/surface_material.h"
#include "shift/core/force_module/contact.h"
#include "shift/core/factory.h"

#include <memory>

namespace shift {
namespace core {

namespace joint {
class Base;
}

namespace terrain {
class Base;
}

namespace body {

class Fixed : public Base {
public:
  Fixed()
    : Base() {
    name_ = "fixed";
    DeclareParameters();
  }
  Fixed(std::string name)
    : Base(name) {
    name_ = "fixed";
    DeclareParameters();
  }
  virtual ~Fixed() override {}

  static bool is_registered() {
    return registered_;
  }

  static std::unique_ptr<Fixed> Create();

  virtual void DeclareParameters() override;

  /*! \brief as body::Base, with the frames, lights and terrain of the fixed
   * body */
  virtual void CheckUniqueNames() override;

  std::vector<std::unique_ptr<terrain::Base>>& get_terrain() {
    return terrain_.get_data();
  }

  /*! \brief the contact model of the environment ("contact" at the root of
   * the model file), or nullptr if the model file gives none */
  force_module::Contact* get_root_contact();

  /*! \brief the sky, sun and fog ("environment" at the root of the model
   * file), or nullptr if the model file gives none */
  environment::Environment* get_environment();

  /*! \brief the materials of the ground ("surface_materials" at the root of
   * the model file), which material maps of terrains refer to by name */
  std::vector<const environment::SurfaceMaterial*> get_surface_materials();
  std::vector<std::unique_ptr<environment::SurfaceMaterial>>&
  get_surface_material_data() {
    return surface_materials_.get_data();
  }

  /*! \brief the objects strewn over the terrains ("scenery" at the root of
   * the model file): trees, bushes, grass, rocks */
  std::vector<std::unique_ptr<environment::Scenery>>& get_scenery() {
    return scenery_.get_data();
  }

private:
  /*! \brief User defined frames attached to fixed inertial reference frame */
  shift::core::parameter::Array<shift::core::Frame> frames_;

  /*! \brief User defined lights attached to fixed inertial reference frame */
  shift::core::parameter::FactoryArray<shift::core::light::Base> lights_;

  /*! \brief User defined terrain attached to fixed inertial reference frame */
  shift::core::parameter::FactoryArray<shift::core::terrain::Base> terrain_;
  /*! \brief the contact model of the environment: the materials of its
   * ground, water and objects. A contact force module of an assembly
   * overrides it (see force_module::Contact::set_root) */
  force_module::Contact contact_{"contact"};
  environment::Environment environment_;
  parameter::Array<environment::SurfaceMaterial> surface_materials_;
  parameter::Array<environment::Scenery> scenery_;

  static bool registered_;
};

} // namespace body
}
} // namespace shift

#endif
