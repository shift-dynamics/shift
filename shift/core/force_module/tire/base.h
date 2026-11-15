#ifndef SHIFT_FORCEMODULE_TIRE_BASE_H_
#define SHIFT_FORCEMODULE_TIRE_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/soil.h"
#include "shift/core/force_module/tire/bekker_wong.h"
#include "shift/core/force_module/tire/fiala.h"
#include "shift/core/force_module/tire/iowa.h"
#include "shift/core/force_module/tire/model.h"
#include "shift/core/force_module/tire/pac2002.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {
namespace tire {

/*!
@brief Tire force module: wheel and terrain contact with a tire model

Finds the contact of the wheel disc with the terrain, builds the contact
frame and slip velocities (tire::Contact), evaluates the tire model selected
by tire_type and applies its force and moment to the wheel body.

The wheel spins about the y axis of the wheel body's origin frame. The
wheel touches the active terrains of the model (terrain::Base::Query), taken
as the plane tangent to the terrain at the contact point; without terrains
the ground is flat at z = 0 with the normal +z (SAE, z down), as in the
original implementation.

Each tire model declares its own parameters, read from the block of its name:
"fiala" (Fiala), "pac2002" (Pac2002), "bekker_wong" (BekkerWong) or "iowa" (Iowa).
\code
{"type": "tire", "name": "left_front", "tire_type": "fiala",
 "wheel_body": "left_front_wheel", "fiala": {"unloaded_radius": 0.3}}
{"type": "tire", "name": "left_front", "tire_type": "pac2002",
 "wheel_body": "left_front_wheel",
 "pac2002": {"definition": "pac2002_235_60R15.tir"}}
{"type": "tire", "name": "wheel", "tire_type": "bekker_wong",
 "wheel_body": "wheel",
 "bekker_wong": {"unloaded_radius": 0.08, "width": 0.032,
                 "soil": {"name": "tri1_lunar_simulant"}}}
{"type": "tire", "name": "left_front", "tire_type": "iowa",
 "wheel_body": "left_front_wheel",
 "iowa": {"friction_coefficient": 0.8, "aligning_coefficient": 0.03}}
\endcode
*/
class Base : public force_module::Base {
public:
  Base()
    : force_module::Base() {
    DeclareParameters();
  }

  Base(std::string name)
    : force_module::Base(name) {
    DeclareParameters();
  }

  virtual ~Base() override {}

  static std::unique_ptr<Base> Create();

  static bool is_registered() {
    return registered_;
  }

  const Contact& get_contact() const {
    return contact_;
  }

  /*! \brief the tire model */
  const Model* get_model() const {
    return model_;
  }

  /*! \brief force and moment on the wheel in the contact frame */
  const ksl_coscrew_t& get_force_w() const {
    return force_w_;
  }

  void DeclareParameters() override;

  /*! \brief reads the module, and refuses a member that it does not declare:
   * the parameters of a tire are in the block of its model, and one left beside
   * it would be ignored and the tire run with its default */
  void ReadJSON(Json::Value& json_value) override;

  void Load() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;

  /*! \brief Bekker-Wong: sinkage, slip, drawbar_pull (\f$f_x\f$), load (\f$-f_z\f$),
   * shear_torque (BekkerWong::get_shear_torque), camber
   * (BekkerWong::get_camber) and side_force (\f$f_y\f$), of the last force
   * evaluation; Iowa: deflection, slip, slip_angle, normal_force,
   * longitudinal_force, lateral_force and aligning_moment; the other tire
   * models have no outputs */
  /*! \brief the names of the outputs, in the order of outputs_ */
  std::vector<std::string> OutputNames() const;
  void UpdateOutputs(double t, const double* y) override;

private:
  /*! \brief path of a file next to the model file or in an include
   * directory */
  std::string FindFile(const std::string& name) const;

  /*! \brief contact of the wheel disc with the terrain */
  void UpdateContact(const ksl_SE3_t& hub, const ksl_screw_t& velocity);

  /*! \brief the same, from the area of the wheel below the terrain in the
   * given number of slices (see Iowa) */
  void UpdateSlicedContact(const ksl_SE3_t& hub, const ksl_screw_t& velocity,
                           int segments);

  /*! \brief the contact frame at origin, of the heading and the terrain normal
   * (unit vectors), and the velocities of the contact */
  void SetContact(const ksl_SE3_t& hub, const ksl_screw_t& velocity,
                  const ksl_vec3_t& heading, const ksl_vec3_t& normal,
                  const ksl_vec3_t& origin, double depth);

  static bool registered_;

  std::string tire_type_;
  std::string wheel_body_name_; //!< wheel body, in the same assembly
  Fiala fiala_;
  Pac2002 pac2002_;
  BekkerWong bekker_wong_;
  Iowa iowa_;
  Model* model_ = nullptr; //!< the one of them that tire_type selects

  shift::core::body::Base* wheel_body_ = nullptr;
  shift::core::Frame* wheel_origin_ = nullptr;
  int load_index_ = -2; //!< see Context::GetBodyLoadIndex, -2: unresolved

  Contact contact_;
  ksl_coscrew_t force_w_;
  std::vector<double> outputs_;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
