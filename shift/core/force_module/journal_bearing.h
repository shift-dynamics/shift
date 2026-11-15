#ifndef SHIFT_CORE_FORCE_MODULE_JOURNAL_BEARING_H_
#define SHIFT_CORE_FORCE_MODULE_JOURNAL_BEARING_H_

#include <memory>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/vec3.h"
#include "shift/core/force_module/base.h"

namespace shift {
namespace core {

class Frame;
namespace body {
class Base;
}

namespace force_module {

/*!
@brief the oil film of a plain journal bearing: the force of the film on the
shaft from the position and motion of the shaft in the bearing, and the
friction torque

The journal and the bearing are two bodies, each with a frame at the center of
its circle, whose z axis is the axis of the bearing (the axes are parallel).
The shaft runs in the clearance \f$c\f$ (the radial clearance, bearing radius
less journal radius) with the eccentricity
\f$\underline{e}\f$, the position of the journal center relative to the bearing
center, perpendicular to the axis, the eccentricity ratio \f$\epsilon = |\underline{e}| / c\f$ and the film
thickness \f$h = c (1 + \epsilon \cos\theta)\f$, with \f$\theta\f$ measured round
the bearing from the thickest part of the film.

The pressure in the film follows from the Reynolds equation of a narrow
(short) bearing, in which the flow along the axis dominates the flow round the
bearing (Ocvirk's short bearing theory; for a ratio of length to diameter up to
about 0.5, and as an approximation to about 1). With the oil of viscosity \f$\mu\f$,
the rotation speeds \f$\omega_j\f$ and \f$\omega_b\f$ of the journal and the
bearing about the axis, the rate \f$\dot{\epsilon}\f$ of the eccentricity ratio
and the velocity of the journal center across the line of centers
\f$e_t = \underline{e} \cdot \underline{u}_2\f$, with \f$\underline{u}_1\f$ along
the line of centers, \f$\underline{u}_2 = \underline{a} \times \underline{u}_1\f$
across it (\f$\underline{a}\f$ the axis):

\f[
p = \frac{3 \mu (l^2/4 - z^2) (s \sin\theta - q \cos\theta)}
  {c^2 (1 + \epsilon \cos\theta)^3}
\f]

with \f$s = \epsilon (\omega_j + \omega_b) - 2 e_t / c\f$, which is \f$\epsilon\f$
times the effective rotation \f$\omega_j + \omega_b - 2 \dot{\varphi}\f$ of the oil
about the line of centers (\f$\dot{\varphi}\f$ the speed of the line of centers
about the axis), \f$q = 2 \dot{\epsilon}\f$ (the squeeze), \f$z\f$ along the axis
from the middle of the bearing and \f$l\f$ its length. Where the pressure would
be negative the oil cavitates and the pressure is zero (the "pi film" of a half
Sommerfeld solution, with the film from \f$\theta_0 = \mathrm{atan2}(q, s)\f$ for
half a turn round the bearing). The force on the journal is the pressure
integrated over it:

\f[
\begin{aligned}
\underline{f} &= \frac{\mu r l^3}{2 c^2} (i_1 \underline{u}_1 + i_2 \underline{u}_2) \\
i_1 &= \int \frac{(s \sin\theta - q \cos\theta) \cos\theta}{(1 + \epsilon \cos\theta)^3} \, \mathrm{d}\theta \\
i_2 &= \int \frac{(s \sin\theta - q \cos\theta) \sin\theta}{(1 + \epsilon \cos\theta)^3} \, \mathrm{d}\theta
\end{aligned}
\f]

over the film, which the module integrates by quadrature (with Sommerfeld's
substitution, which makes the integrands smooth even for large \f$\epsilon\f$).
In steady running (\f$\dot{\epsilon} = 0\f$, \f$e_t = 0\f$) these are the
classical results

\f[
\begin{aligned}
\underline{f} \cdot \underline{u}_1 &= -\frac{\mu r l^3 \omega \epsilon^2}{c^2 (1 - \epsilon^2)^2}
  \quad \text{(the film pushes the journal back to the center)} \\
\underline{f} \cdot \underline{u}_2 &= \frac{\mu r l^3 \omega \pi \epsilon}{4 c^2 (1 - \epsilon^2)^{3/2}}
\end{aligned}
\f]

whose ratio is the attitude angle of the short bearing,
\f$\tan\phi = \pi \sqrt{1 - \epsilon^2} / (4 \epsilon)\f$, and whose size is the
load that the bearing carries,
\f$w = \frac{\mu r l^3 \omega \epsilon \sqrt{16 \epsilon^2 + \pi^2 (1 - \epsilon^2)}}{4 c^2 (1 - \epsilon^2)^2}\f$.
The component across the line of centers is the
cross-coupling of the film, which drives the oil whirl of lightly loaded
shafts; the module has it, and the damping of the squeeze, in its force.

The friction torque on the journal is that of the shear of the oil and of its
pressure gradient (Petroff's formula with the eccentricity, and the work of the
pressure force):

\f[
\tau = -\frac{2 \pi \mu r^3 l (\omega_j - \omega_b)}{c \sqrt{1 - \epsilon^2}} +
  \frac{c \epsilon}{2} \underline{f} \cdot (-\underline{u}_2),
\f]

where the shear of the oil is taken round the whole bearing, the cavitated
half filled with oil for the shear.

The eccentricity ratio is limited to 0.995 in the film (smoothly beyond 0.9),
where the film's force is large but finite; give wall_stiffness for contact of
the journal with the bearing at \f$|\underline{e}| > c\f$.

*A bearing of finite length.* With "model": "finite" the module drops the
narrow bearing's assumption and solves the Reynolds equation itself. With
\f$\bar{h} = 1 + \epsilon \cos\theta\f$, \f$\bar{z} = 2 z / l\f$ and
\f$p = (\mu r^2 / c^2) \bar{p}\f$, it is

\f[
\frac{\partial}{\partial\theta}\left(\bar{h}^3 \frac{\partial \bar{p}}{\partial\theta}\right) +
  \frac{4 r^2}{l^2} \frac{\partial}{\partial \bar{z}}\left(\bar{h}^3 \frac{\partial \bar{p}}{\partial \bar{z}}\right) =
  -6 (s \sin\theta - q \cos\theta)
\f]

with \f$\bar{p} = 0\f$ at the ends, which is the short bearing's pressure as
\f$l / r\f$ goes to zero and Sommerfeld's as it grows. It is linear in \f$s\f$ and
\f$q\f$, so \f$\bar{p} = s \bar{p}_s + q \bar{p}_q\f$, and when the model loads the module solves for
\f$\bar{p}_s\f$ and \f$\bar{p}_q\f$ at each of a table of eccentricity ratios, by finite volumes
on a grid that is fine where the film is thin (uniform in Sommerfeld's angle). The
oil cavitates where the pressure would be negative (the half Sommerfeld
condition, the "pi film" of the narrow bearing). For each direction
\f$\mathrm{atan2}(q, s)\f$ of a second table the module clips \f$\bar{p}\f$ at zero and
integrates it, and as the model runs the force is interpolated in the two tables,
in proportion to \f$\sqrt{s^2 + q^2}\f$.

*A groove.* groove_width is a circumferential oil groove at the middle of the
bearing, at no pressure, which divides it into two bearings, each of
length \f$(l - \mathrm{groove\_width}) / 2\f$, for either model.

*The heating of the oil.* With heat_capacity, the temperature rise \f$t_r\f$ of the
oil over the viscosity's reference is a state, with the power of the friction
and a heat loss, and the viscosity falls with it (Reynolds' exponential law):

\f[
c_h \dot{t_r} = |\tau (\omega_j - \omega_b)| - k_t t_r, \quad
\mu = \mu_0 e^{-\beta t_r}
\f]

with the heat_capacity \f$c_h\f$, cooling \f$k_t\f$ and viscosity_slope \f$\beta\f$.

Not in the module: the inertia of the oil, misalignment, axial grooves and the
pressure at which the oil is supplied, a temperature that varies over the film,
and the elasticity of the shaft and the bearing. With the finite model,
side_flow is still the short bearing's.

JSON:
\code
{"type": "journal_bearing", "name": "main_bearing",
 "journal": ["shaft", "origin"], "bearing": ["fixed", "origin"],
 "radius": 0.025, "length": 0.025, "clearance": 5e-5, "viscosity": 0.01}
\endcode

- journal, bearing: [body, frame] of each; the frames are at the centers of
  the journal and of the bearing, the z axes along the axis ("fixed" is the
  fixed body).
- radius: of the journal; length: of the bearing; clearance: the radial
  clearance; viscosity: of the oil (Pa s, for SI).
- wall_stiffness (optional, default 0): stiffness of the contact when the
  journal touches the bearing, \f$|\underline{e}| > c\f$; zero for none.
- model (optional, default "short"): "short" or "finite".
- groove_width (optional, default 0).
- heat_capacity, cooling, viscosity_slope (optional): the heating of the oil.

Outputs: eccentricity_ratio, min_film_thickness (\f$c (1 - \epsilon)\f$, negative
when the journal is in contact), peak_pressure, friction_torque (on the
journal), side_flow (of oil out of the ends of the bearing), radial_force and
tangential_force (\f$\underline{f} \cdot \underline{u}_1\f$,
\f$\underline{f} \cdot \underline{u}_2\f$), load
(\f$|\underline{f}|\f$), effective_viscosity and temperature_rise.
*/
class JournalBearing : public Base {
public:
  JournalBearing()
    : Base() {
    Initialize();
  }
  JournalBearing(std::string name)
    : Base(name) {
    Initialize();
  }
  virtual ~JournalBearing() override {}

  static std::unique_ptr<JournalBearing> Create();
  static bool is_registered() {
    return registered_;
  }

  void Connect() override;
  void Load() override;
  void Service(const double* y, double* dy) override;

  void UpdateOutputs(double t, const double* y) override;

  /*! \brief the integrals I_1 and I_2 over the film (see above) for the
   * eccentricity ratio epsilon, s and q */
  static void FilmIntegrals(double epsilon, double s, double q, double* i_1,
                            double* i_2);

  /*! \brief the force of the film of one land of the finite model over
   * \f$\mu r^3 l / (2 c^2)\f$, along and across the line of centers, and its
   * peak of \f$\bar{p}\f$, interpolated in the tables */
  void FiniteFilm(double epsilon, double s, double q, double* j_1, double* j_2,
                  double* peak) const;

  double get_viscosity() const {
    return effective_viscosity_;
  }
  double get_temperature_rise() const {
    return temperature_rise_;
  }
  double get_eccentricity_ratio() const {
    return eccentricity_ratio_;
  }
  double get_radial_force() const {
    return radial_force_;
  }
  double get_tangential_force() const {
    return tangential_force_;
  }
  double get_friction_torque() const {
    return friction_torque_;
  }
  double get_peak_pressure() const {
    return peak_pressure_;
  }
  double get_side_flow() const {
    return side_flow_;
  }
  double get_min_film_thickness() const {
    return min_film_thickness_;
  }
  /*! \brief the force of the film on the journal in the fixed frame */
  const ksl_vec3_t& get_force() const {
    return force_;
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

  /*! \brief solves the Reynolds equation for the tables of the finite model */
  void BuildFilmTable();

  static bool registered_;

  std::vector<std::string> journal_names_, bearing_names_;
  double radius_ = 0.0;
  double length_ = 0.0;
  double clearance_ = 0.0;
  double viscosity_ = 0.0;
  double wall_stiffness_ = 0.0;
  std::string model_ = "short";
  double groove_width_ = 0.0;
  double heat_capacity_ = 0.0, cooling_ = 0.0, viscosity_slope_ = 0.0;

  double land_ = 0.0; //!< length of a land
  int lands_ = 1;
  /* the finite model: j_1, j_2 and the peak for each eccentricity ratio and
  direction, times (1 - epsilon^2)^2, which takes out most of their growth */
  std::vector<double> film_table_;

  Attachment journal_, bearing_;

  ksl_vec3_t force_ = {{0.0, 0.0, 0.0}};
  double eccentricity_ratio_ = 0.0, min_film_thickness_ = 0.0;
  double peak_pressure_ = 0.0, friction_torque_ = 0.0, side_flow_ = 0.0;
  double radial_force_ = 0.0, tangential_force_ = 0.0, load_ = 0.0;
  double effective_viscosity_ = 0.0, temperature_rise_ = 0.0;
};

} // namespace force_module
} // namespace core
} // namespace shift

#endif
