#ifndef SHIFT_FORCEMODULE_TIRE_BASE_H_
#define SHIFT_FORCEMODULE_TIRE_BASE_H_

#include <memory>
#include <string>

#include "shift/core/force_module/base.h"
#include "shift/core/force_module/tire/fiala.h"
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
terrain is flat at z = 0 with the normal +z (SAE, z down), as in the original
implementation, until terrain modules provide a height and normal query.

JSON:
\code
{"type": "tire", "name": "left_front", "tire_type": "fiala",
 "wheel_body": "left_front_wheel"}
\endcode
Fiala parameters (all optional): unloaded_radius, vertical_stiffness,
vertical_damping, rolling_resistance, width, cslip, calpha, u_max, u_min,
x_relaxation_length, y_relaxation_length.

PAC2002 parameters: definition (required), the .tir property file, found
next to the model file or in model_parameters/include_directories;
mounted_side (optional), "left" or "right": mirrors the tire if it differs
from TYRESIDE of the property file; use_transient_slip (optional, default
false) relaxes the slips with the relaxation lengths of the property file
(see Pac2002).
\code
{"type": "tire", "name": "left_front", "tire_type": "pac2002",
 "definition": "pac2002_235_60R15.tir", "wheel_body": "left_front_wheel"}
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

  /*! \brief force and moment on the wheel in the contact frame */
  const ksl_coscrew_t& get_force_w() const {
    return force_w_;
  }

  void DeclareParameters() override;
  void Load() override;
  void Connect() override;
  void Service(const double* y, double* dy) override;

private:
  /*! \brief path of a file next to the model file or in an include
   * directory */
  std::string FindFile(const std::string& name) const;

  /*! \brief contact of the wheel disc with the flat terrain */
  void UpdateContact(const ksl_SE3_t& hub, const ksl_screw_t& velocity);

  static bool registered_;

  std::string tire_type_;
  std::string wheel_body_name_; //!< wheel body, in the same assembly
  FialaParameters fiala_;
  std::string definition_;   //!< PAC2002 .tir file
  std::string mounted_side_; //!< PAC2002: "left", "right" or empty
  bool use_transient_slip_ = false;
  std::unique_ptr<Model> model_;

  shift::core::body::Base* wheel_body_ = nullptr;
  shift::core::Frame* wheel_origin_ = nullptr;
  int load_index_ = -2; //!< see Context::GetBodyLoadIndex, -2: unresolved

  Contact contact_;
  ksl_coscrew_t force_w_;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
