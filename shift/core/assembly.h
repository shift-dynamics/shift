#ifndef SHIFT_ASSEMBLY_H_
#define SHIFT_ASSEMBLY_H_

#include <string>

#include "shift/core/base.h"
#include "shift/core/controller/base.h"
#include "shift/core/body/base.h"
#include "shift/core/constraint/base.h"
#include "shift/core/force_module/base.h"
#include "shift/core/joint/base.h"
#include "shift/core/light/base.h"
#include "shift/core/parameter/list.h"
#include "shift/core/sensor/base.h"

namespace shift {
namespace core {

class Context;

/*!
@brief Shift Assembly datastructure
*/
class Assembly
  : public Base
  , public parameter::List {

public:
  Assembly(std::string name)
    : Base(name)
    , parameter::List() {
    DeclareParameters();
  }

  Assembly()
    : Base("")
    , parameter::List() {
    DeclareParameters();
  }

  virtual ~Assembly() override {}

  static const std::string get_array_key() {
    return "assemblies";
  }

  std::vector<std::unique_ptr<body::Base>>& get_bodies() {
    return bodies_.get_data();
  }
  std::vector<std::unique_ptr<joint::Base>>& get_joints() {
    return joints_.get_data();
  }
  std::vector<std::unique_ptr<force_module::Base>>& get_force_modules() {
    return force_modules_.get_data();
  }
  /*! \brief the assemblies nested in this one */
  std::vector<std::unique_ptr<Assembly>>& get_assemblies() {
    return assemblies_.get_data();
  }

  /*! \brief throw if a name is repeated at one level: among the assemblies,
   * bodies, joints, force modules, sensors, controllers or constraints of this
   * assembly, or within one of them, or in the assemblies nested in it */
  void CheckUniqueNames();

  /*! \brief the component of this assembly named name: a body, joint, force
   * module, sensor, controller, constraint or nested assembly; nullptr if
   * there is none. Names are unique within each kind (CheckUniqueNames), but
   * two kinds may share one: then name must give the kind by its key in the
   * model file, as "force_modules/motor", and a bare name that matches
   * components of two kinds throws. For a controller to find the component
   * whose input it sets. */
  parameter::List* FindComponent(const std::string& name);

  /*! \brief the component named name: one of this assembly (FindComponent),
   * else the one of that fully qualified name among assemblies (the model's)
   * and the assemblies nested in them, as
   * "<assembly>__<subassembly>__<name>", with its kind before it where two
   * kinds share the name, as "controllers/car__brake_pedal"; nullptr if
   * there is none. For a controller to find a component in another
   * assembly. */
  parameter::List* FindComponent(
    const std::string& name, std::vector<std::unique_ptr<Assembly>>& assemblies);

  /*! \brief the assembly of the fully qualified name qualified among
   * assemblies and the assemblies nested in them, or nullptr */
  static Assembly* FindQualifiedAssembly(
    std::vector<std::unique_ptr<Assembly>>& assemblies,
    const std::string& qualified);

private:
  void DeclareParameters() override;

  /*! \brief the object of objects named name, or nullptr */
  template <class T>
  static parameter::List* FindByName(std::vector<std::unique_ptr<T>>& objects,
                                     const std::string& name) {
    for(auto& object : objects) {
      if(object->get_name() == name) {
        return object.get();
      }
    }
    return nullptr;
  }

  shift::core::parameter::FactoryArray<body::Base> bodies_;
  shift::core::parameter::FactoryArray<controller::Base> controllers_;
  shift::core::parameter::FactoryArray<constraint::Base> constraints_;
  shift::core::parameter::FactoryArray<force_module::Base> force_modules_;
  shift::core::parameter::FactoryArray<joint::Base> joints_;
  shift::core::parameter::FactoryArray<sensor::Base> sensors_;

  shift::core::parameter::Array<Assembly> assemblies_;
};

}
} // namespace shift

#endif
