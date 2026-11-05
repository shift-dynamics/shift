#ifndef SHIFT_FORCEMODULE_TIRE_MODEL_H_
#define SHIFT_FORCEMODULE_TIRE_MODEL_H_

#include "ksl/types.h"

namespace shift {
namespace core {
namespace force_module {
namespace tire {

/*!
@brief Contact state of a wheel, computed by tire::Base for the tire models

Axis convention (SAE, matching the fixed frame of the vehicle models, whose
z axis points down): the contact frame w has its origin at the contact point
on the terrain surface, x along the wheel heading in the terrain plane, z
along the terrain normal pointing into the terrain (down) and y = z x x.
Forces and moments are computed in the w frame and act on the wheel at the
w frame origin.
*/
struct Contact {
  bool in_contact = false;
  double depth = 0.0;   //!< penetration of the lowest point of the wheel disc
  ksl_SE3_t w_frame;    //!< contact frame in the fixed frame
  /*! velocity of the material point of the wheel at the contact point, in
   * the w frame (the slip velocities in x and y) */
  ksl_vec3_t contact_velocity_w;
  /*! velocity of the contact center C, the w frame origin moving with the
   * wheel carrier (the wheel without its spin), in the w frame */
  ksl_vec3_t center_velocity_w;
  ksl_vec3_t spin_axis_w;    //!< wheel spin axis (hub y axis), in the w frame
  double vx = 0.0;    //!< hub velocity along the w frame x axis
  double omega = 0.0; //!< wheel spin rate about the wheel axis
};

/*!
@brief Tire force law, evaluated by tire::Base for each wheel

A model may have internal states (e.g. transient slip), integrated with the
equations of motion.
*/
class Model {
public:
  virtual ~Model() {}

  /*! \brief number of internal states */
  virtual unsigned int get_state_count() const = 0;

  /*! \brief unloaded radius, for the contact geometry */
  virtual double get_unloaded_radius() const = 0;

  /*! \brief compute the force and moment on the wheel in the w frame
   * (referred to the w frame origin) and the time derivatives of the
   * internal states
   * \param contact [in] contact state; if not in contact, the force is zero
   * \param states [in] internal states
   * \param state_rates [out] their time derivatives
   * \param force_w [out] force (lin) and moment (ang) in the w frame */
  virtual void Evaluate(const Contact& contact, const double* states,
                        double* state_rates, ksl_coscrew_t* force_w) = 0;
};

} // namespace tire
} // namespace force_module
} // namespace core
} // namespace shift

#endif
