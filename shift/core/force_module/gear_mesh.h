#ifndef SHIFT_CORE_FORCE_MODULE_GEAR_MESH_H_
#define SHIFT_CORE_FORCE_MODULE_GEAR_MESH_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "shift/core/force_module/base.h"
#include "shift/core/force_module/gear_teeth.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the mesh of a pair of gears, spur, helical or internal: the teeth in contact as springs and
dampers along the line of action, with a stiffness that changes as the teeth
roll through the mesh, backlash and the errors of the tooth profiles

Each gear is a body with a frame on its axis, whose z axis is the axis (the
two axes are parallel, and the z axes point the same way). The bodies are free
to turn about their axes, and to move: the mesh is a force between them, not a
constraint, so the gears can also carry the dynamics of their shafts and
bearings.

The teeth touch along the line of action, which passes through the pitch
point P on the line between the centers and makes the pressure angle
\f$\phi\f$ with the common tangent of the pitch circles. Driving one way, the flank on one
side of the teeth is in contact; driving the other, the flank on the other
side, and the line of action is the reflection of the first in the line of
centers. The module keeps the two as separate contacts, "plus" and "minus",
which push but never pull:

- the tangential displacement \f$\delta_t\f$ of the pitch point of gear 1 relative to
  that of gear 2, the integral of the relative velocity of P as part of each
  body along the tangent of the pitch circles (a state variable of the
  module). It is zero at the start, the gears meshing midway through the
  backlash;
- the approach of the centers, \f$d_0 - d\f$, from their distance \f$d\f$ and
  its value \f$d_0\f$ at the start;
- the compression of each flank along its line of action,
  \f[
  \begin{aligned}
  e_+ &= \cos\phi \, \delta_t + \sin\phi \, (d_0 - d) + p(x) - \cos\phi \, b / 2 \\
  e_- &= -\cos\phi \, \delta_t + \sin\phi \, (d_0 - d) + p(x) - \cos\phi \, b / 2,
  \end{aligned}
  \f]
  with the circumferential backlash \f$b\f$ at the pitch circle and the error
  \f$p(x)\f$ of the profile, the extra thickness of the teeth, at the position
  of the mesh \f$x\f$;
- the force along each line of action,
  \f[
  f = k(x) \, \mathrm{pos}(e) + c \, s(e) \, \dot{e},
  \f]
  where \f$\mathrm{pos}(e)\f$ is \f$e\f$ for \f$e > 0\f$ and zero otherwise, made
  smooth over the small length `smoothing`, \f$s(e)\f$ is its slope (1 once the
  flank touches, 0 before), \f$k(x)\f$ is the stiffness of the mesh and \f$c\f$
  the damping. The force pushes gear 2 along the tangent,
  \f$\pm f \cos\phi\f$, and apart from gear 1 along the line of centers,
  \f$f \sin\phi\f$, and the opposite on gear 1, at the pitch point;
- \f$x\f$ is the position in the mesh, the fraction of a tooth pitch that gear 1
  has turned since a pair of teeth met (the module integrates gear 1's rotation
  as a state variable): \f$x = z_1 \theta_1 / 2 \pi\f$, taken modulo 1, plus the
  `initial_phase`, with \f$z_1\f$ the number of teeth of gear 1.

*The stiffness of the mesh.* Teeth in contact are springs, and the number of
pairs in contact changes as the gears turn: with the contact ratio \f$\varepsilon\f$
between 1 and 2, two pairs share the load for the fraction
\f$\varepsilon - 1\f$ of the mesh and one pair for the rest (Kahraman and
Singh, 1990; the load sharing of teeth in contact is Elkholy's, 1985). With
tooth_stiffness \f$k_t\f$ for one pair,

\f[
k(x) = k_t (1 + m(x)),
\f]

where \f$m\f$ is 1 while two pairs are in contact and 0 while one is, changing
over a smooth ramp (a fraction `ramp` of the mesh) at each change, so that
the force has no jump and the variable step integrator sees none. The mean
stiffness over the mesh is \f$k_t \varepsilon\f$. A table of the stiffness against \f$x\f$
over \f$[0, 1]\f$ (stiffness_table), such as one computed for the actual teeth,
replaces this.

*Stiffness from the teeth.* With tooth_geometry the module computes
\f$k(x)\f$ and the contact ratio from the shape of the teeth (see GearTeeth), with
face_width \f$w\f$.

*Internal gears.* With internal, gear 2 is a ring and gear 1 runs inside it: the
pitch point is on the far side of gear 1 from the ring's center, the centers
are \f$r_2 - r_1\f$ apart, the gears turn the same way, and the teeth press the
gears' centers together along the line of centers.

*Helical gears.* With helix_angle \f$\beta\f$, pressure_angle is \f$\phi_n\f$, that of
the normal section, and the line of action leaves the plane of the gears: with
\f$\underline{n}\f$ the tangent of the pitch circles, \f$\underline{c}\f$ the line
of centers from gear 1 to gear 2 and \f$\underline{a}\f$ the axis, the plus flank's
is, for external gears,

\f[
\underline{u} = \cos\phi_n \cos\beta \, \underline{n} +
  \sin\phi_n \, \underline{c} - \cos\phi_n \sin\beta \, \underline{a},
\f]

so that the axial force on the gears is the tangential force times
\f$\tan\beta\f$, and the radial force that times
\f$\tan\phi_n / \cos\beta\f$. The compression \f$e\f$ follows the same direction,
with the motion of the gears along the axis. A positive angle is a right hand helix
on gear 1; external gears have opposite hands and a ring the same hand. The axial
force acts at the pitch point, so it also tips the gears. With face_width, a
section across the face lags the one before it, so the stiffness is the mean over
the overlap \f$w \tan\beta \, z_1 / 2 \pi r_1\f$ of pitches, which smooths it.

*The oil.* With oil_viscosity \f$\mu\f$, a flank that closes on its partner at the
rate \f$\dot{e}\f$ squeezes the oil out of the gap \f$h = \mathrm{pos}(-e) +\f$
min_film. The flanks are two cylinders, of relative radius \f$r_e\f$ at the pitch
point, along a line of length \f$w / \cos\beta_b\f$, and Reynolds' equation gives

\f[
f_s = 3 \sqrt{2} \pi \mu \frac{w}{\cos\beta_b}
  \left(\frac{r_e}{h}\right)^{3/2} \dot{e}
\f]

while the gap closes, and nothing as it opens (the film cavitates). This is the
damping of the teeth crossing the backlash.

*Gear whine.* The varying stiffness and the errors \f$p(x)\f$ of the profile
excite the transmission error \f$\delta_t\f$ at the mesh frequency
\f$z_1 \omega_1 / 2 \pi\f$ and its harmonics, which is the vibration heard as the whine of gears.

JSON:
\code
{"type": "gear_mesh", "name": "mesh",
 "gear_1": ["pinion", "origin"], "gear_2": ["wheel", "origin"],
 "pitch_radius_1": 0.05, "pitch_radius_2": 0.1, "teeth_1": 20,
 "pressure_angle": 0.349, "tooth_stiffness": 3e8, "contact_ratio": 1.6,
 "damping": 3e3, "backlash": 1e-4}
\endcode

- gear_1, gear_2: [body, frame] of each gear; the frame is on the gear's axis,
  its z axis along it.
- pitch_radius_1, pitch_radius_2; teeth_1 (the number of teeth of gear 1).
- pressure_angle (optional, radians, default 20 degrees).
- tooth_stiffness: of a pair of teeth, along the line of action (N/m).
- contact_ratio (optional, 1 to 2, default 1.5); ramp (optional, default 0.1,
  at most the shorter of the changes); stiffness_table (optional): [x0, k0, x1,
  k1, ...] over [0, 1].
- damping (optional, default 0): of the contact, N s/m.
- backlash (optional, default 0): circumferential, at the pitch circles.
- profile_error (optional): [x0, p0, x1, p1, ...] over [0, 1], the extra
  thickness of the teeth along the line of action, which can be negative.
- initial_phase (optional, default 0): the position in the mesh at the start.
- smoothing (optional, default 1e-7 m): the length over which a flank starts to
  load.

- internal (optional): gear 2 is a ring gear.
- helix_angle (optional, radians, default 0); face_width (optional): needed for
  tooth_geometry, oil_viscosity and the overlap of helical teeth.
- tooth_geometry (optional): see GearTeeth.
- oil_viscosity (optional, default 0), min_film (optional, default 1e-6 m).

Outputs: transmission_error (\f$\delta_t\f$), mesh_force (the net tangential
force on the pitch circles), force_plus, force_minus, stiffness, phase,
radial_force, axial_force and squeeze_force.
*/
class GearMesh : public Base {
public:
  GearMesh()
    : Base() {
    Initialize();
  }
  GearMesh(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~GearMesh() override {}

  static std::unique_ptr<GearMesh> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the number of pairs of teeth in contact, \f$m(x) + 1\f$, smooth: 2
   * for the fraction contact_ratio - 1 of the mesh, centered on \f$x = 0\f$, and 1
   * otherwise, changing over the fraction ramp of the mesh */
  static double PairsInContact(double x, double contact_ratio, double ramp);

  /*! \brief the stiffness of the mesh at the position x */
  double StiffnessAt(double x) const;

  double get_transmission_error() const {
    return transmission_error_;
  }
  double get_mesh_force() const {
    return mesh_force_;
  }
  double get_force_plus() const {
    return force_plus_;
  }
  double get_force_minus() const {
    return force_minus_;
  }
  double get_phase() const {
    return phase_;
  }
  double get_radial_force() const {
    return radial_force_;
  }
  double get_axial_force() const {
    return axial_force_;
  }
  double get_squeeze_force() const {
    return squeeze_force_;
  }
  double get_contact_ratio() const {
    return contact_ratio_;
  }

protected:
  void DeclareParameters() override;

private:
  void Initialize() {
    DeclareParameters();
  }

  struct Attachment {
    body::Base* body = nullptr;
    const Frame* frame = nullptr;
    int load_index = -2;
  };

  ksl_SE3_t Pose(const Attachment& attachment) const;
  Attachment Resolve(const std::vector<std::string>& names,
                     const std::string& what) const;
  void Compute(const double* y, double* dy, bool apply);
  /*! \brief the stiffness table from that of a spur gear, spread over the
   * overlap of helical teeth */
  void BuildStiffnessTable(const std::function<double(double)>& spur,
                           int points);
  /*! \brief the force of the film of oil on a flank of compression e closing
   * at the rate */
  double SqueezeForce(double e, double rate) const;

  static bool registered_;

  std::vector<std::string> gear_1_names_, gear_2_names_;
  double pitch_radius_1_ = 0.0, pitch_radius_2_ = 0.0;
  int teeth_1_ = 0;
  double pressure_angle_ = 0.3490658503988659; // 20 degrees
  double tooth_stiffness_ = 0.0;
  double contact_ratio_ = 1.5;
  double ramp_ = 0.1;
  std::vector<double> stiffness_points_, profile_points_;
  double damping_ = 0.0;
  double backlash_ = 0.0;
  double initial_phase_ = 0.0;
  double smoothing_ = 1e-7;
  bool internal_ = false;
  double helix_angle_ = 0.0;
  double face_width_ = 0.0;
  GearTeeth teeth_;
  double oil_viscosity_ = 0.0;
  double min_film_ = 1e-6;

  /* the direction cosines of the line of action of the plus flank */
  double tangential_ = 1.0, axial_ = 0.0, radial_ = 0.0;
  double transverse_angle_ = 0.0;
  double contact_length_ = 0.0; //!< of the teeth, along their helix
  double film_radius_ = 0.0;    //!< relative radius of the flanks
  double axial_offset_0_ = 0.0;

  Attachment gear_1_, gear_2_;
  double center_distance_0_ = 0.0;
  Table stiffness_table_, profile_table_;

  double transmission_error_ = 0.0, mesh_force_ = 0.0;
  double force_plus_ = 0.0, force_minus_ = 0.0;
  double stiffness_ = 0.0, phase_ = 0.0;
  double radial_force_ = 0.0, axial_force_ = 0.0, squeeze_force_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
