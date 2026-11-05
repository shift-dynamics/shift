#ifndef SHIFT_CORE_FORCE_MODULE_GEAR_MESH_H_
#define SHIFT_CORE_FORCE_MODULE_GEAR_MESH_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "shift/core/force_module/base.h"
#include "shift/core/force_module/table.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the mesh of a pair of spur gears: the teeth in contact as springs and
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

Outputs: transmission_error (\f$\delta_t\f$), mesh_force (the net tangential
force on the pitch circles, \f$(f_+ - f_-) \cos\phi\f$), force_plus, force_minus, stiffness and
phase.
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

  std::vector<std::string> get_output_names() const override;
  void UpdateOutputs(double t, const double* y) override;
  void AppendOutputs(std::vector<double>& values) const override;

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

  Attachment gear_1_, gear_2_;
  double center_distance_0_ = 0.0;
  Table stiffness_table_, profile_table_;

  double transmission_error_ = 0.0, mesh_force_ = 0.0;
  double force_plus_ = 0.0, force_minus_ = 0.0;
  double stiffness_ = 0.0, phase_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
